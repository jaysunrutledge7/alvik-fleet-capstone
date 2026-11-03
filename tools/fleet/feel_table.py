#!/usr/bin/env python3
"""Feel out the table (Nick, 2 Nov 2026 18:33): on the first run on a new table, before the production run, each robot
feels the table's edges with its own edge sensors, and the camera's outline of the table is checked against them.

  1. calibration    every robot is fully calibrated first, as a start-up does (run_plan --calibrate), then stands
                    still SURFACE_HOLD_S at its home: the SURFACE CHECK (the risk review, 2 Nov: the edge thresholds
                    were set on plywood, never on a glossy conference top). Its line sensors must swing at most half
                    of edge_thr standing still (the cue needs edge_thr 3 times in a row), and its look-ahead (bottom
                    zone) must read at least BOT_MARGIN_CM under edge_bot_cm. Failed: NO crawls, a warning, the cameras' outline kept (saved, so later runs skip it);
  2. the feels      each robot crawls slowly (the planner's probe_cms, 3 cm/s, the warm-up's side-feel speed) straight
                    at SPOTS on the edges nearest it: 2 on each long side and 2 on each end (8 in all), split by
                    halves so neither robot crosses the other's half. A crawl starts START_IN_CM inside the camera's
                    edge line and is planned to end END_IN_CM INSIDE it: a robot never drives past the cameras' line.
                    The edge guard (safe_edge 3: line sensors and the bottom zone that looks ahead) ends it at the
                    drop; the robot then backs off STRAIGHT the way it came at the crawl's speed (planner
                    probe_back_cms), with no turn at the edge. The guard is never switched off, and the robots' own
                    virtual border is off (as in the warm-up: the crawls go to the edge on purpose). The cameras watch;
  3. the comparison every edge-guard trip gives a felt edge: where the camera saw the robot's middle stop, plus how far
                    ahead of its wheels the cue that tripped notices the drop (its REACH). Per side: felt more than
                    AGREE_CM further IN than the cameras' line: that side of the outline is moved in (at most
                    MAX_ADJUST_CM) and said in plain words; further in than that the outline is left alone and the
                    production run waits for Nick (the camera setup is suspect). Felt further OUT: said, and the
                    cameras' line kept (the robots never drove past it). A crawl that felt nothing: "not felt, the
                    cameras' line kept", and named as a possible sensor miss when the side's other feel found it;
  4. saved          as the table's calibration in the site's folder (<conf_dir>/feel_table/<table>.json), tied to
                    the camera setup it was made with (the corner clicks: setup_id()). Later runs on the same setup
                    use it and skip the feels; a new camera setup, or Nick's tick, runs it again;
  5. old data       each robot's reach (how far ahead of its wheels it notices a drop) is kept across sessions and
                    tables (<conf_dir>/feel_table/reach.json) and pooled with the new feels, so each new table needs
                    the camera less to tell the robots' reach from an outline error ("it can use old data once we
                    have some", Nick).

Safety (Claude's design, see the report): a crawl's planned end leaves the robot's middle END_IN_CM inside the camera's
line, so with the camera right and both cues missing the drop, the robot stops with its centre of mass (guessed
1.5 cm ahead of the axle) 1.5 cm on the table. One cue missing: the other stops it (the line sensors at 3.5 cm ahead
trip about 0.7 cm before the planned end; the bottom zone 9-11 cm ahead long before). NOT covered: the cameras'
outline too far out on a side AND both cues missing on the same crawl (two faults at once).

    python3 tools/fleet/feel_table.py plan --map map.json --out feel_map.json      # the feels as warm-up tasks
    python3 tools/fleet/feel_table.py analyze --map feel_map.json --session <run_plan session folder>
    python3 tools/fleet/feel_table.py status --conf-dir ~/.config/mirlt/sites/school-conference --table 396x168
"""
import argparse
import copy
import csv
import datetime as dt
import hashlib
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM    # noqa: E402

VERSION = "2026-11-02a"

SPOTS_LONG = 2           # feels on each long side (at 1/4 and 3/4 of its length: one in each robot's half). Nick: "about
                         # 6-8 spots in total"; 2 per side gives every side its own check (two feels that should
                         # agree). No DOE: coverage, not a measurement setting
SPOTS_END = 2            # feels on each end (at 30 % and 70 % of the width)
START_IN_CM = 22.0       # chosen: a crawl starts with the robot's middle this far inside the camera's line. The bottom
                         # zone notices a drop 9-11 cm ahead of the wheels, so an outline up to about 10 cm too big is
                         # still met by the crawl itself, not the drive there; planner-safe (over the 11.2 cm the
                         # planner keeps from an open side). No DOE: geometry
END_IN_CM = 3.0          # the crawl's planned end: the robot's middle this far inside the camera's line. The simulator's
                         # centre of mass is 1.5 cm ahead of the axle (a guess, tools/sim/robot.py HANG_COM_AHEAD_CM), so
                         # with the camera right and both cues missing it stops 1.5 cm on the table; the line sensors
                         # (3.5 cm ahead) trip about 0.7 cm before this end when the bottom zone misses. To MEASURE, not
                         # a DOE: the real robot's centre of mass (balance it on a ruler's edge)
AGREE_CM = 2.0           # chosen: a side whose felt edge is within this of the camera's line agrees (no change); a
                         # 3 cm error must be caught (Nick's test). From the parts: the camera's table fit (0.3-0.5 cm),
                         # a still tag's position (0.1-0.3 cm), the stop at 3 cm/s (0.3 cm), the reach's spread between
                         # feels (0.1-0.2 cm in the simulator, unknown on the real sensor). DOE: yes, worth it
                         # (docs/doe-backlog.md): repeated feel-outs on a tape-measured table give the spread of
                         # (felt - true) per cue; the line is about 3 x that spread, and 3 cm must stay caught
MAX_ADJUST_CM = 10.0     # chosen: a side is moved at most this far. Further apart, the camera setup itself is suspect
                         # (a corner clicked on the wrong place, a camera knocked): the outline is NOT moved, the
                         # production run waits, and the words say to do the camera setup again. No DOE: a limit
MOVE_OUT = False         # a side felt further OUT than the cameras' line: kept (the risk review, 2 Nov). True: moved
                         # out like an inward side (the first design, kept behind this switch: Nick's rule)
SPREAD_CM = 3.0          # two feels on one side further apart than this: the side's more inward feel is used (the safe
                         # side) and said. No DOE: the same as AGREE_CM
REACH_DEFAULT = {"line": 3.5, "bottom": 9.0}   # how far ahead of the wheels each cue notices a drop, before any feel:
                         # the line sensors' place (params.h grid_center_cm) and the bottom zone's (e02_map.py
                         # BOTTOM_AHEAD_CM 9.0, an open DOE row: "Bottom zone's reach ahead"). Learned from the feels
REACH_RANGE = {"line": (2.0, 6.0), "bottom": (5.0, 15.0)}   # a learned reach outside these is not used (said)
REACH_MIN_N = 4          # chosen: a cue's reach is learned once it has this many trips (this session plus old data);
                         # with 8 feels, a median of 4+ ignores one side's 2 feels being off. No DOE
REACH_KEEP = 60          # old trips kept per robot and cue (reach.json)
CAM_AFTER_S = (0.1, 0.8)  # the camera's place of a stopped robot: the median of its fixes this long after the stop
FACING_COS = 0.7         # a trip counts for a side when the robot faced it within about 45 deg

SIDE_WORDS = {"x_min": "corner-1 end (x = 0)", "x_max": "far end", "y_min": "near long side (y = 0)",
              "y_max": "far long side"}
SIDE_SHORT = {"x_min": "left end", "x_max": "right end", "y_min": "bottom side", "y_max": "top side"}
OUTWARD = {"x_min": (-1.0, 0.0), "x_max": (1.0, 0.0), "y_min": (0.0, -1.0), "y_max": (0.0, 1.0)}


# ---------------------------------------------------------------------------------------------------------- planning

def rect(m):
    f = m["floor"]
    return [f["x_min"], f["y_min"], f["x_max"], f["y_max"]]


def spots(m, long_n=SPOTS_LONG, end_n=SPOTS_END):
    """The feel spots on the map's outline: [{id, side, along, start [x, y], end [x, y], robot}]. Each spot goes to the
    robot whose start (home) is nearest the spot's crawl start, so two robots side by side split the table in halves
    and neither crosses the other's half."""
    m = FM.normalize(m)
    x0, y0, x1, y1 = rect(m)
    L, W = x1 - x0, y1 - y0
    long_sides = ("y_min", "y_max") if L >= W else ("x_min", "x_max")
    out = []
    for side in FM.SIDES:
        along_x = side.startswith("y")
        lo, hi = (x0, x1) if along_x else (y0, y1)
        n = long_n if side in long_sides else end_n
        fr = [(k + 0.5) / n for k in range(n)] if side in long_sides else \
            ([0.3, 0.7] if n == 2 else [(k + 0.5) / n for k in range(n)])
        line = m["floor"][side]
        ox, oy = OUTWARD[side]
        for f_ in fr:
            a = round(lo + (hi - lo) * f_, 1)
            if along_x:
                s0, s1 = [a, line - oy * START_IN_CM], [a, line - oy * END_IN_CM]
            else:
                s0, s1 = [line - ox * START_IN_CM, a], [line - ox * END_IN_CM, a]
            out.append({"side": side, "along": a, "start": [round(v, 2) for v in s0], "end": [round(v, 2) for v in s1]})
    robots = {r["name"]: r["start"][:2] for r in m["robots"]}
    if not robots:
        raise FM.MapError("no robots in the map")
    for k, s in enumerate(out):
        s["id"] = "S%d" % (k + 1)
        s["robot"] = min(robots, key=lambda n: math.dist(robots[n], s["start"]))
    return out


def label(s):
    return "feel %s (%s at %.0f)" % (s["id"], SIDE_SHORT[s["side"]], s["along"])


def plan_tasks(m):
    """The feel-out's map: the outline as it is, every robot's crawls nearest first from its start, then home (park =
    start). Returns (map with tasks, the spots)."""
    m = FM.normalize(copy.deepcopy(m))
    sp = spots(m)
    tasks = {}
    for r in m["robots"]:
        mine = [s for s in sp if s["robot"] == r["name"]]
        cur, seq = r["start"][:2], []
        while mine:
            k = min(range(len(mine)), key=lambda i: math.dist(cur, mine[i]["start"]))
            s = mine.pop(k)
            seq.append({"probe": s["start"] + s["end"], "label": label(s), "side": s["side"], "along": s["along"],
                        "kind": "feel"})
            cur = s["start"]
        tasks[r["name"]] = {"visit": seq, "park": True}
    m["tasks"] = tasks
    return FM.normalize(m), sp


# ---------------------------------------------------------------------------------------------------------- reading

def _jsonl(path):
    out = []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    return out


def _num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def cam_track(session):
    """{robot: [(t, x, y, heading)]} from run_plan's cam_track.csv (the camera's place, 5 a second)."""
    out = {}
    p = os.path.join(session, "cam_track.csv")
    if not os.path.exists(p):
        return out
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            x, y, t = _num(r.get("x_cm")), _num(r.get("y_cm")), _num(r.get("t"))
            if x is None or y is None or t is None:
                continue
            out.setdefault(r["robot"], []).append((t, x, y, _num(r.get("heading_deg"))))
    return out


def cam_place(track, robot, t):
    """The camera's place of a robot that stopped at time t: the median of its fixes CAM_AFTER_S after (still), else
    the nearest fix within 0.3 s, else None."""
    L = track.get(robot) or []
    w = [c for c in L if t + CAM_AFTER_S[0] <= c[0] <= t + CAM_AFTER_S[1]]
    if len(w) >= 2:
        xs, ys = sorted(c[1] for c in w), sorted(c[2] for c in w)
        return xs[len(xs) // 2], ys[len(ys) // 2], "camera (median of %d fixes after the stop)" % len(w)
    if L:
        c = min(L, key=lambda c: abs(c[0] - t))
        if abs(c[0] - t) < 0.3:
            return c[1], c[2], "camera (one fix)"
    return None


def trips(session):
    """Every edge-guard stop of the run: [{robot, t, own pose, cue, label (its feel), place}], from run_plan's step
    reports ("plywood edge at x y th cue") and the labelled feel steps."""
    ev = _jsonl(os.path.join(session, "coordinator.jsonl"))
    lab = [(e["robot"], float(e["t"]), e["label"]) for e in ev
           if e.get("event") == "labelled_step" and e.get("state") == "failed"]

    def label_at(robot, t):
        # the labelled record is logged right after its step report (run_plan on_step): the same robot within 0.2 s
        c = [x for x in lab if x[0] == robot and abs(x[1] - t) < 0.2]
        return min(c, key=lambda x: abs(x[1] - t))[2] if c else None
    track = cam_track(session)
    out = []
    for e in ev:
        if e.get("event") != "step_report" or e.get("state") != "failed":
            continue
        info = str(e.get("info") or "")
        if not info.startswith("plywood edge"):
            continue
        parts = info.split()
        cue = parts[-1] if parts and parts[-1] in REACH_DEFAULT else ("line" if "line" in info else "bottom")
        t = float(e["t"])
        th = float(e["th"])
        own = (float(e["x"]), float(e["y"]))
        cp = cam_place(track, e["robot"], t)
        place = (cp[0], cp[1]) if cp else own
        out.append({"robot": e["robot"], "t": t, "own": [round(own[0], 2), round(own[1], 2)], "heading_deg": th,
                    "cue": cue, "place": [round(place[0], 2), round(place[1], 2)],
                    "place_from": cp[2] if cp else "the robot's own pose (no camera fix)",
                    "label": label_at(e["robot"], t)})
    return out


def feel_ends(session):
    """Feels that reached their planned end with no trip (state done in the probe phase): {label: record}."""
    out = {}
    for e in _jsonl(os.path.join(session, "coordinator.jsonl")):
        if e.get("event") == "labelled_step" and e.get("phase") == "probe" and e.get("state") == "done":
            out[e["label"]] = e
    return out


# ---------------------------------------------------------------------------------------------------------- analysis

def side_of(m, place, heading_deg):
    """The side a stopped robot faced: the outward side whose direction is closest to its heading (cos >= FACING_COS)
    and, among those, the nearest line. Returns (side, how far its middle is inside the camera's line) or (None, None)."""
    h = math.radians(heading_deg)
    best = None
    for side, (ox, oy) in OUTWARD.items():
        c = ox * math.cos(h) + oy * math.sin(h)
        if c < FACING_COS:
            continue
        line = m["floor"][side]
        inside = (line - place[0]) * ox if side.startswith("x") else (line - place[1]) * oy
        if best is None or abs(inside) < abs(best[1]):
            best = (side, inside, c)
    return (best[0], best[1], best[2]) if best else (None, None, None)


def load_reach(conf_dir):
    p = os.path.join(conf_dir, "feel_table", "reach.json") if conf_dir else ""
    if p and os.path.exists(p):
        try:
            with open(p) as f:
                return json.load(f)
        except ValueError:
            pass
    return {}


def reach_used(samples_now, old, robots):
    """Each cue's reach (cm): the median of this run's trips and the old ones (every robot's: same robot model and
    firmware), when there are REACH_MIN_N or more and it is inside REACH_RANGE; else the default. Returns
    {cue: {cm, n, n_old, from, words}}."""
    out = {}
    for cue in REACH_DEFAULT:
        now_ = list(samples_now.get(cue, []))
        old_ = [v for r in robots for v in (old.get(r, {}).get(cue) or [])]
        allv = sorted(now_ + old_)
        lo, hi = REACH_RANGE[cue]
        if len(allv) >= REACH_MIN_N:
            med = allv[len(allv) // 2] if len(allv) % 2 else 0.5 * (allv[len(allv) // 2 - 1] + allv[len(allv) // 2])
            if lo <= med <= hi:
                out[cue] = {"cm": round(med, 2), "n": len(now_), "n_old": len(old_), "from": "learned",
                            "words": "%s: notices a drop %.1f cm ahead of its wheels (median of %d trips%s)" % (
                                cue_words(cue), med, len(allv), ", %d from earlier sessions" % len(old_) if old_ else "")}
                continue
            out[cue] = {"cm": REACH_DEFAULT[cue], "n": len(now_), "n_old": len(old_), "from": "default",
                        "words": "%s: the trips say %.1f cm ahead, outside %.0f-%.0f cm, so the usual %.1f cm is used "
                                 "(CHECK the sensor)" % (cue_words(cue), med, lo, hi, REACH_DEFAULT[cue])}
            continue
        out[cue] = {"cm": REACH_DEFAULT[cue], "n": len(now_), "n_old": len(old_), "from": "default",
                    "words": "%s: %d trips (fewer than %d), the usual %.1f cm used" % (
                        cue_words(cue), len(allv), REACH_MIN_N, REACH_DEFAULT[cue])}
    return out


def cue_words(cue):
    return {"line": "line sensors", "bottom": "the distance sensor's look-ahead (bottom zone)"}.get(cue, cue)


def analyze(m, session, sp=None, conf_dir=None):
    """Where the robots felt each edge against the camera's outline (m: the feel-out's map). Returns the result dict
    (feels, sides, reach, misses, words)."""
    m = FM.normalize(m)
    sp = sp or spots(m)
    tr = trips(session)
    robots = sorted({r["name"] for r in m["robots"]})
    for t in tr:
        side, inside, c = side_of(m, t["place"], t["heading_deg"])
        t["side"], t["inside_cm"], t["cos"] = side, None if inside is None else round(inside, 2), c
    # each cue's reach: assuming the camera's lines right on most sides, a trip's (inside / cos) is its reach
    now_ = {}
    for t in tr:
        if t["side"] and t["inside_cm"] is not None and t["inside_cm"] > -2.0:
            now_.setdefault(t["cue"], []).append(t["inside_cm"] / max(t["cos"], 0.5))
    old = load_reach(conf_dir)
    reach = reach_used(now_, old, robots)
    for t in tr:
        if not t["side"]:
            continue
        r = reach[t["cue"]]["cm"]
        t["offset_cm"] = round(r * t["cos"] - t["inside_cm"], 2)   # + = the edge is further OUT than the camera's line
        t["felt_line"] = round(m["floor"][t["side"]] + t["offset_cm"] * (1 if t["side"].endswith("max") else -1), 2)
    ends = feel_ends(session)
    feels = []
    for s in sp:
        lab = label(s)
        mine = [t for t in tr if t.get("label") == lab]
        if mine:
            t = mine[0]
            fx = None
            if t.get("felt_line") is not None and t.get("side") == s["side"]:
                fx = [s["along"], t["felt_line"]] if s["side"].startswith("y") else [t["felt_line"], s["along"]]
            feels.append(dict(s, label=lab, kind="felt", cue=t["cue"], offset_cm=t.get("offset_cm"),
                              inside_cm=t["inside_cm"], place=t["place"], place_from=t["place_from"], t=t["t"],
                              felt_xy=fx))
        elif lab in ends:
            e = ends[lab]
            feels.append(dict(s, label=lab, kind="nothing", place=(e.get("cam") or e.get("pose") or [None, None])[:2],
                              why="it reached its planned end (%.0f cm inside the camera's line) with no drop felt"
                                  % END_IN_CM))
        else:
            feels.append(dict(s, label=lab, kind="not run", why="this crawl did not run (the run ended or re-planned "
                                                                "without it)"))
    # trips on the way (not during a crawl), e.g. the drive to a crawl's start met the drop first
    extra = [t for t in tr if not t.get("label") and t.get("side")]
    sides = {}
    for side in FM.SIDES:
        offs = [f["offset_cm"] for f in feels if f["side"] == side and f["kind"] == "felt" and f.get("offset_cm") is not None]
        offs += [t["offset_cm"] for t in extra if t["side"] == side]
        nothing = [f for f in feels if f["side"] == side and f["kind"] == "nothing"]
        sides[side] = side_verdict(side, offs, nothing, reach)
    misses = suspected_misses(feels, sides, reach)
    res = {"version": VERSION, "made": dt.datetime.now().isoformat(timespec="seconds"), "outline": rect(m),
           "feels": feels, "trips": tr, "sides": sides, "reach": reach, "reach_samples": now_, "misses": misses,
           "session": session}
    res["hold"] = any(s["verdict"] == "suspect" for s in sides.values())
    res["adjusted"] = {k: s["move_cm"] for k, s in sides.items() if s.get("move_cm")}
    res["words"] = words(res)
    return res


def side_verdict(side, offs, nothing, reach):
    """One side: agree / adjusted / suspect / none felt."""
    name = SIDE_WORDS[side]
    if not offs:
        if nothing:
            far = (reach["bottom"]["cm"] - END_IN_CM)
            return {"verdict": "not felt", "n": 0, "move_cm": 0.0,
                    "words": "%s: no drop felt at %d spot(s): the table goes on at least %.0f cm past the camera's "
                             "line here; the camera's line is kept (the safe side)" % (name, len(nothing), far)}
        return {"verdict": "not felt", "n": 0, "move_cm": 0.0, "words": "%s: not felt (no crawl ran)" % name}
    v = sorted(offs)
    med = v[len(v) // 2] if len(v) % 2 else 0.5 * (v[len(v) // 2 - 1] + v[len(v) // 2])
    spread = v[-1] - v[0]
    use, how = med, "median of %d" % len(v)
    if spread > SPREAD_CM:
        use, how = v[0], "the most inward of %d feels (they differ by %.1f cm)" % (len(v), spread)
    d = {"n": len(v), "offsets_cm": [round(x, 2) for x in v], "offset_cm": round(use, 2), "spread_cm": round(spread, 2),
         "how": how}
    inout = "further out" if use > 0 else "further in"
    if abs(use) <= AGREE_CM:
        d.update(verdict="agree", move_cm=0.0, words="%s: felt %.1f cm %s than the camera's line (%s): agrees" % (
            name, abs(use), inout, how))
    elif use > 0 and not MOVE_OUT:
        # further OUT than the cameras' line: never moved out (risk review, 2 Nov): the robots never drove past the
        # cameras' line, only their look-ahead reached past it, so the cameras' line is kept (the safe side)
        d.update(verdict="further out", move_cm=0.0,
                 words="WARNING: %s: the look-ahead felt the drop %.1f cm further out than the camera's outline (%s). "
                       "The camera's line is kept (the safe side: the table is at least that big here)." % (
                           name, use, how))
    elif abs(use) <= MAX_ADJUST_CM:
        d.update(verdict="adjusted", move_cm=round(use, 1),
                 words="WARNING: %s: the robots felt the edge %.1f cm %s than the camera's outline (%s). The outline "
                       "is moved %s by %.1f cm on this side, so the robots keep their room from the real edge." % (
                           name, abs(use), inout, how, "out" if use > 0 else "in", abs(use)))
    else:
        d.update(verdict="suspect", move_cm=0.0,
                 words="STOP: %s: the robots felt the edge %.1f cm %s than the camera's outline (%s), more than the "
                       "%.0f cm this check may correct. The outline is NOT moved and the production run waits: do the "
                       "camera setup again (Cameras & table: corners, Measure), then feel out the table again." % (
                           name, abs(use), inout, how, MAX_ADJUST_CM))
    if spread > SPREAD_CM:
        d["words"] += " The feels on this side differ by %.1f cm: the edge may not be straight, or a sensor read " \
                      "late." % spread
    return d


def suspected_misses(feels, sides, reach):
    """A crawl that felt nothing where the side's other feels found the drop within its sensors' reach of the planned
    end: its edge sensors probably missed the drop (it stopped on its planned end, as designed)."""
    out = []
    for f in feels:
        if f["kind"] != "nothing":
            continue
        sd = sides[f["side"]]
        if sd.get("n") and sd.get("offset_cm") is not None and \
                sd["offset_cm"] < reach["bottom"]["cm"] - END_IN_CM - 1.0:
            # the side's drop is where the look-ahead (and, nearer, the line sensors) meet it before this crawl's end
            out.append({"robot": f["robot"], "spot": f["id"], "side": f["side"],
                        "words": "WARNING: %s felt NO drop at %s (%s) although the other feels there found it: its "
                                 "edge sensors probably missed it. It stopped on its planned end (%.0f cm inside the "
                                 "camera's line), as designed. Check %s's edge sensors before the next run." % (
                                     f["robot"], f["id"], SIDE_WORDS[f["side"]], END_IN_CM, f["robot"])})
    return out


def words(res):
    out = []
    n_f = len([f for f in res["feels"] if f["kind"] == "felt"])
    out.append("%d of %d spots felt the drop" % (n_f, len(res["feels"])))
    for side in FM.SIDES:
        out.append(res["sides"][side]["words"])
    for mi in res["misses"]:
        out.append(mi["words"])
    for cue in REACH_DEFAULT:
        out.append("reach, %s" % res["reach"][cue]["words"])
    if res.get("hold"):
        out.append("The production run waits for you: see the STOP line above.")
    elif res["adjusted"]:
        out.append("Outline adjusted on %d side(s); the production run uses it." % len(res["adjusted"]))
    else:
        out.append("The camera's outline agrees with the edges the robots felt; nothing changed.")
    return out


# ---------------------------------------------------------------------------------------------------------- applying

def adjusted_rect(r, moves):
    """The outline [x0, y0, x1, y1] with each side moved (+ = out)."""
    x0, y0, x1, y1 = r
    x0 -= moves.get("x_min", 0.0)
    y0 -= moves.get("y_min", 0.0)
    x1 += moves.get("x_max", 0.0)
    y1 += moves.get("y_max", 0.0)
    return [round(x0, 2), round(y0, 2), round(x1, 2), round(y1, 2)]


def apply_to_map(m, cal, names=None):
    """The workspace map with the table calibration's outline: the production map remade on it (homes at its middle,
    points moved in from its edges). Returns (map, words)."""
    import production as PR
    m = FM.normalize(m)
    names = names or [r["name"] for r in m["robots"]]
    r = cal.get("outline_used") or rect(m)
    pts = {k: v[:2] for k, v in (m.get("spots") or {}).items()}
    m2, w = PR.site_map(None, names, points=pts, outline=r)
    return m2, w


# ---------------------------------------------------------------------------------------------------------- storing

def table_key(m=None, table=None):
    """The table's name in the site's folder: its size in whole cm, e.g. "396x168" (the typed size if given)."""
    if table:
        L, W = float(table[0]), float(table[1])
    else:
        x0, y0, x1, y1 = rect(FM.normalize(m))
        L, W = x1 - x0, y1 - y0
    return "%dx%d" % (round(max(L, W)), round(min(L, W)))


def setup_id(tagcal_dir):
    """The camera setup a calibration belongs to: a short hash of the corner clicks and picture sizes in the setup's
    obs.json (a refit after a run keeps it; doing the corners again changes it). None without a setup."""
    p = os.path.join(tagcal_dir or "", "obs.json")
    if not tagcal_dir or not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            o = json.load(f)
    except ValueError:
        return None
    key = {"sizes": {c: v.get("size") for c, v in sorted((o.get("cameras") or {}).items())},
           "corners": {c: {k: [round(float(q) * 2) / 2 for q in uv] for k, uv in sorted((cc or {}).items())}
                       for c, cc in sorted((o.get("corners") or {}).items())}}
    return hashlib.sha1(json.dumps(key, sort_keys=True).encode()).hexdigest()[:12]


def cal_path(conf_dir, key):
    return os.path.join(conf_dir, "feel_table", "%s.json" % key)


def load_cal(conf_dir, key):
    p = cal_path(conf_dir, key)
    if not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            return json.load(f)
    except ValueError:
        return None


def needed(conf_dir, key, sid, force=False):
    """(run it?, the stored calibration or None, why in words)."""
    if force:
        return True, load_cal(conf_dir, key), "Feel out again was asked for"
    cal = load_cal(conf_dir, key)
    if cal is None:
        return True, None, "first run on this table (%s): no table calibration saved yet" % key
    if sid and cal.get("setup_id") and cal["setup_id"] != sid:
        return True, cal, "the camera setup was done again since this table was felt (%s): feel it again" % (
            cal.get("made", "?"))
    if cal.get("hold"):
        return True, cal, "the last feel-out found the outline too far off (it waited for you): feel it again"
    return False, cal, "felt on %s with this camera setup: saved outline used, the feels are skipped" % cal.get("made")


def save_cal(conf_dir, key, sid, res, site=None):
    """The table's calibration (and every robot's reach trips into reach.json, the old data for next time)."""
    d = os.path.join(conf_dir, "feel_table")
    os.makedirs(d, exist_ok=True)
    prev = load_cal(conf_dir, key)
    hist = list((prev or {}).get("history") or [])
    if prev:
        hist.append({k: prev.get(k) for k in ("made", "setup_id", "adjusted", "hold", "outline_used")})
    outline = res["outline"]
    used = outline if res.get("hold") else adjusted_rect(outline, res.get("adjusted") or {})
    cal = {"version": VERSION, "table": key, "site": site, "setup_id": sid, "made": res["made"],
           "outline_camera": outline, "outline_used": used, "adjusted": res.get("adjusted") or {},
           "hold": bool(res.get("hold")), "sides": res["sides"], "reach": res["reach"], "words": res["words"],
           "misses": res["misses"], "session": res.get("session"), "surface": res.get("surface"),
           "feels": [{k: f.get(k) for k in ("id", "robot", "side", "along", "kind", "cue", "offset_cm", "place",
                                            "felt_xy")}
                     for f in res["feels"]], "history": hist[-20:]}
    tmp = cal_path(conf_dir, key) + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cal, f, indent=1)
    os.replace(tmp, cal_path(conf_dir, key))
    # the robots' reach trips: kept per robot and cue (the old data later feel-outs pool with)
    rp = os.path.join(d, "reach.json")
    old = load_reach(conf_dir)
    for t in res.get("trips") or []:
        if t.get("side") and t.get("inside_cm") is not None and t["inside_cm"] > -2.0 and not res.get("hold"):
            # only trips on sides that agreed or were adjusted from many feels: a suspect outline would teach a wrong
            # reach. The reach sample is measured against the outline as it is now believed (adjusted)
            sd = res["sides"].get(t["side"]) or {}
            mv = float(sd.get("move_cm") or 0.0)
            v = (t["inside_cm"] + mv) / max(t["cos"], 0.5)
            old.setdefault(t["robot"], {}).setdefault(t["cue"], []).append(round(v, 2))
            old[t["robot"]][t["cue"]] = old[t["robot"]][t["cue"]][-REACH_KEEP:]
    with open(rp + ".tmp", "w") as f:
        json.dump(old, f, indent=1)
    os.replace(rp + ".tmp", rp)
    return cal


def card(cal, need=None, why=None, progress=None):
    """What Mission Control's "Feel out the table" card shows (plain words)."""
    c = {"needed": need, "why": why, "progress": progress}
    if cal:
        c.update(made=cal.get("made"), adjusted=cal.get("adjusted"), hold=cal.get("hold"), words=cal.get("words"),
                 outline_camera=cal.get("outline_camera"), outline_used=cal.get("outline_used"),
                 sides={k: {kk: v.get(kk) for kk in ("verdict", "offset_cm", "move_cm", "n", "offsets_cm")}
                        for k, v in (cal.get("sides") or {}).items()},
                 feels=cal.get("feels"), reach={k: v.get("cm") for k, v in (cal.get("reach") or {}).items()})
    return c


def progress(session, sp):
    """Each robot's crawls so far, from the run's log: {robot: {done, total, now}}."""
    out = {}
    for s in sp:
        out.setdefault(s["robot"], {"done": 0, "total": 0, "felt": 0, "now": None})["total"] += 1
    labs = {label(s): s for s in sp}
    seen = set()
    for e in _jsonl(os.path.join(session, "coordinator.jsonl")):
        if e.get("event") != "labelled_step" or e.get("label") not in labs:
            continue
        s = labs[e["label"]]
        o = out[s["robot"]]
        if e.get("state") == "started" and e.get("phase") == "probe":
            o["now"] = "crawling to %s (%s)" % (s["id"], SIDE_SHORT[s["side"]])
        elif e.get("state") in ("done", "failed") and e.get("phase") == "probe" and e["label"] not in seen:
            # a crawl is cut into drive pieces: only its last piece's "done" (or a stop by the edge guard) ends it
            if e["state"] == "done" and math.dist((e.get("to") or [0, 0])[:2], s["end"]) > 0.6:
                continue
            seen.add(e["label"])
            o["done"] += 1
            if e["state"] == "failed" and str(e.get("info", "")).startswith("plywood edge"):
                o["felt"] += 1
            o["now"] = "backing off %s" % s["id"]
    return out


# ---------------------------------------------------------------------------------------------------------- surface

SURFACE_HOLD_S = 4.0     # the surface check: each robot stands still this long at its home after its calibration
                         # (about 40 RAW rows at 10 a second). No DOE: enough rows for a spread
EDGE_THR = 150.0         # the line cue's threshold the runs send (mission_run.EDGE_GUARD "set edge_thr 150")
EDGE_BOT_CM = 11.0       # the bottom-zone cue's limit the runs send ("set edge_bot_cm 11")
LINE_NOISE_SHARE = 0.5      # chosen (risk review, 2 Nov): a still robot's line readings may wander at most half
                         # of edge_thr from their median (the cue needs a change of edge_thr, edge_n times in a row).
                         # DOE note: the still spread on the real tabletop against false trips while crawling
BOT_MARGIN_CM = 1.5      # chosen: a still robot's highest bottom-zone reading must stay this far under edge_bot_cm (a
                         # glossy top can read long). DOE note: the same runs
SURFACE_MIN_ROWS = 10


def surface_tasks(m):
    """The surface check's map: each robot stands still SURFACE_HOLD_S at its start (home), then parks there."""
    m = FM.normalize(copy.deepcopy(m))
    m["tasks"] = {r["name"]: {"visit": [{"wait_s": SURFACE_HOLD_S, "label": "surface check"}], "park": True}
                  for r in m["robots"]}
    return FM.normalize(m)


def _rows_still(session):
    """RAW rows of each robot during its "surface check" wait (by the laptop's time) or, without the label, standing
    still rows (no speed)."""
    win = {}
    for e in _jsonl(os.path.join(session, "coordinator.jsonl")):
        if e.get("event") == "labelled_step" and e.get("label") == "surface check":
            w = win.setdefault(e["robot"], [None, None])
            if e.get("state") == "started":
                w[0] = float(e["t"])
            elif e.get("state") == "done":
                w[1] = float(e["t"])
    out = {}
    p = os.path.join(session, "raw.csv")
    if not os.path.exists(p):
        return out
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            n = r.get("robot")
            t = _num(r.get("t_recv"))
            w = win.get(n)
            if w and w[0] is not None:
                if t is None or t < w[0] + 0.3 or (w[1] is not None and t > w[1]):
                    continue
            elif abs(_num(r.get("v_cms")) or 0.0) > 0.3 or abs(_num(r.get("w_dps")) or 0.0) > 3.0:
                continue
            out.setdefault(n, []).append(r)
    return out


def surface_check(session, edge_thr=EDGE_THR, edge_bot_cm=EDGE_BOT_CM):
    """Is the tabletop clearly apart from the edge guard's thresholds, standing still? Per robot: each line sensor's
    median and its largest swing from it (against edge_thr: the margin), the bottom zone's median and highest reading
    (against edge_bot_cm). ok only when every robot passes; otherwise the crawls are refused and the cameras' outline
    kept (the risk review, 2 Nov)."""
    rows = _rows_still(session)
    res = {"robots": {}, "edge_thr": edge_thr, "edge_bot_cm": edge_bot_cm, "ok": True, "words": []}
    if not rows:
        res.update(ok=False)
        res["words"].append("Surface check: no standing readings from any robot: the crawls are NOT run, the camera's "
                            "outline is kept.")
        return res
    for n, rr in sorted(rows.items()):
        d = {"rows": len(rr)}
        why = []
        line = {}
        for c in ("lineL", "lineC", "lineR"):
            v = sorted(x for x in (_num(r.get(c)) for r in rr) if x is not None)
            if not v:
                continue
            med = v[len(v) // 2]
            sw = max(abs(v[0] - med), abs(v[-1] - med))
            line[c] = {"median": med, "swing": round(sw, 1)}
        d["line"] = line
        sw = max([x["swing"] for x in line.values()] or [float("inf")])
        d["line_margin"] = round(edge_thr - sw, 1)
        if sw > edge_thr * LINE_NOISE_SHARE:
            why.append("its line sensors swing %.0f standing still (allowed %.0f: half the edge threshold %.0f, "
                       "which the cue needs 3 times in a row)"
                       % (sw, edge_thr * LINE_NOISE_SHARE, edge_thr))
        b = sorted(x for x in (_num(r.get("dbot_cm")) for r in rr) if x is not None and x > 0)
        if b:
            d["bottom"] = {"median": b[len(b) // 2], "highest": b[-1]}
            d["bottom_margin_cm"] = round(edge_bot_cm - b[-1], 2)
            if edge_bot_cm - b[-1] < BOT_MARGIN_CM:
                why.append("its look-ahead reads up to %.1f cm on the tabletop, within %.1f cm of its %.1f cm edge "
                           "limit" % (b[-1], BOT_MARGIN_CM, edge_bot_cm))
        else:
            why.append("no bottom-zone readings")
        if len(rr) < SURFACE_MIN_ROWS:
            why.append("only %d standing readings (needs %d)" % (len(rr), SURFACE_MIN_ROWS))
        d["ok"] = not why
        d["why"] = why
        res["robots"][n] = d
        if why:
            res["ok"] = False
            res["words"].append("Surface check FAILED for %s: %s." % (n, "; ".join(why)))
        else:
            res["words"].append("Surface check %s: line sensors swing at most %.0f standing (edge threshold %.0f: "
                                "margin %.0f); look-ahead reads %.1f cm at most (edge limit %.1f cm: margin %.1f cm)."
                                % (n, sw, edge_thr, d["line_margin"], b[-1], edge_bot_cm, d["bottom_margin_cm"]))
    if not res["ok"]:
        res["words"].append("The edge sensors are not clearly apart from the tabletop: the crawls are NOT run (they "
                            "would rely on those sensors), the camera's outline is kept as it is. Look at the "
                            "robots' sensors and the table's surface before trying again.")
    return res


def surface_cal(conf_dir, key, sid, outline, surface, site=None):
    """A failed surface check saved as the table's calibration: the cameras' outline kept, no crawls; later runs skip
    it until the camera setup is done again or Nick asks again."""
    res = {"made": dt.datetime.now().isoformat(timespec="seconds"), "outline": list(outline),
           "sides": {k: {"verdict": "not felt", "n": 0, "move_cm": 0.0, "words": "not felt (surface check failed)"}
                     for k in FM.SIDES}, "reach": {}, "misses": [], "feels": [], "trips": [], "hold": False,
           "adjusted": {}, "words": list(surface["words"])}
    cal = save_cal(conf_dir, key, sid, res, site=site)
    cal["surface"] = surface
    cal["skipped_for_surface"] = True
    with open(cal_path(conf_dir, key), "w") as f:
        json.dump(cal, f, indent=1)
    return cal


# ---------------------------------------------------------------------------------------------------------- running

CALIBRATE_ARGS = ["--rp-arg=--calibrate"]     # step 1: every robot fully calibrated first (the surface check's run)


def prepare_surface(m, folder):
    """The surface check's map and plan (each robot: its calibration, then standing still at home)."""
    import planner as P
    import missions as MS
    os.makedirs(folder, exist_ok=True)
    sm = surface_tasks(m)
    over = {}
    MS.gap_lock_planner(over, n_robots=len(sm["robots"]))
    plan = P.make_plan(sm, over=over or None)
    mp, pp = os.path.join(folder, "surface_map.json"), os.path.join(folder, "surface_plan.json")
    FM.save(sm, mp)
    P.save_plan(plan, pp)
    return mp, pp


def prepare(m, folder):
    """The feel-out's map and plan in folder (feel_map.json, feel_plan.json, feel_spots.json). Returns (map path, plan
    path, spots, the planned time in s)."""
    import planner as P
    import missions as MS
    os.makedirs(folder, exist_ok=True)
    fm, sp = plan_tasks(m)
    over = {}
    MS.gap_lock_planner(over, n_robots=len(fm["robots"]))     # the 8 cm lock, as every plan
    plan = P.make_plan(fm, over=over or None)
    mp, pp = os.path.join(folder, "feel_map.json"), os.path.join(folder, "feel_plan.json")
    FM.save(fm, mp)
    P.save_plan(plan, pp)
    with open(os.path.join(folder, "feel_spots.json"), "w") as f:
        json.dump(sp, f, indent=1)
    return mp, pp, sp, plan["summary"]["makespan_s"]


def mission_run_args(mp, pp, session, site=None, dry=False, extra=(), calibrate=False):
    """mission_run.py's arguments for a feel-out run: a made mission, the robots' own border off (the crawls go to the
    edge on purpose; the edge guard stays on), no turn pauses; calibrate: every robot fully calibrated first (the
    surface check's run; the crawl run then keeps that calibration, same power-on)."""
    a = [os.path.join(HERE, "mission_run.py"), "--mission", mp, "--plan", pp, "--session", session, "--border-cm", "0",
         "--no-turn-pause", "--tracking", "tags", "--feel-table", "skip"] + (CALIBRATE_ARGS if calibrate else [])
    if site:
        a += ["--site", site]
    if dry:
        a += ["--dry"]
    return a + list(extra)


def finish(fm, session, sp, conf_dir, key, sid, site=None, surface=None):
    """After the feel-out run: analyze, save as the table's calibration. Returns (result, calibration)."""
    res = analyze(fm, session, sp, conf_dir=conf_dir)
    if surface:
        res["surface"] = surface
        res["words"] = list(surface.get("words") or []) + res["words"]
    with open(os.path.join(session, "feel_result.json"), "w") as f:
        json.dump(res, f, indent=1)
    cal = save_cal(conf_dir, key, sid, res, site=site)
    return res, cal


# ---------------------------------------------------------------------------------------------------------- picture

def plot(cases, png, colours=None, note=None):
    """Felt against camera, per side, for one or more feel-outs: cases [(title, result, {side: true error cm})]. Each
    panel: the four sides across, each feel's (felt - camera) as a dot in its robot's colour and shape, the agreement
    band shaded, the change made as a bar outline, the simulator's true error (when known) as a black dash."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colours = colours or {"Alpha": "#2952CC", "Bravo": "#C2410C"}
    shapes = {"Alpha": "o", "Bravo": "s"}
    n = len(cases)
    cols = min(3, n)
    rows = int(math.ceil(n / float(cols)))
    fig, axs = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.3 * rows + 0.6), squeeze=False, sharey=True)
    order = ["x_min", "y_min", "y_max", "x_max"]
    names = {"x_min": "corner-1\nend", "y_min": "near\nlong side", "y_max": "far\nlong side", "x_max": "far\nend"}
    for k, (title, res, truth) in enumerate(cases):
        ax = axs[k // cols][k % cols]
        ax.axhspan(-AGREE_CM, AGREE_CM, color="#E7F5EE", zorder=0)
        ax.axhline(0, color="#98A2B3", lw=1, zorder=1)
        for i, sd in enumerate(order):
            fs = [f for f in res["feels"] if f["side"] == sd]
            if any(f["kind"] == "nothing" for f in fs):
                ax.annotate("felt nothing", (i, 0), textcoords="offset points", xytext=(0, 10), ha="center",
                            fontsize=7, color="#5B6474")
            for j, f in enumerate(fs):
                dx = (j - (len(fs) - 1) / 2.0) * 0.18
                if f["kind"] == "felt" and f.get("offset_cm") is not None:
                    ax.scatter(i + dx, f["offset_cm"], s=46, marker=shapes.get(f["robot"], "o"),
                               color=colours.get(f["robot"], "#5B6474"), edgecolor="white", linewidth=1.2, zorder=3)
                elif f["kind"] == "nothing":
                    ax.scatter(i + dx, 0, s=60, marker=shapes.get(f["robot"], "o"), facecolor="none",
                               edgecolor=colours.get(f["robot"], "#5B6474"), linewidth=1.6, zorder=3)
            mv = (res["sides"].get(sd) or {}).get("move_cm") or 0.0
            if mv:
                ax.bar(i, mv, width=0.6, fill=False, edgecolor="#101828", linewidth=1.2, zorder=2)
            if truth and sd in truth:
                ax.plot([i - 0.32, i + 0.32], [truth[sd]] * 2, color="#101828", lw=2.2, zorder=4)
        ax.set_xticks(range(4))
        ax.set_xticklabels([names[s] for s in order], fontsize=8)
        ax.set_title(title, fontsize=9.5, loc="left")
        ax.grid(axis="y", color="#F0F1F3")
        for sp_ in ("top", "right"):
            ax.spines[sp_].set_visible(False)
        if k % cols == 0:
            ax.set_ylabel("felt edge - cameras' outline (cm)\n+ further out, - further in", fontsize=8)
    for k in range(n, rows * cols):
        axs[k // cols][k % cols].axis("off")
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    hs = [Line2D([], [], marker=shapes[r], ls="", color=c, label="felt by %s" % r) for r, c in colours.items()]
    hs += [Line2D([], [], color="#101828", lw=2.2, label="true error (simulator)"),
           Patch(facecolor="none", edgecolor="#101828", label="change made to the outline"),
           Patch(facecolor="#E7F5EE", label="agrees (within %.0f cm)" % AGREE_CM)]
    fig.legend(handles=hs, loc="lower center", ncol=5, fontsize=8, frameon=False, bbox_to_anchor=(0.5, 0.025))
    fig.text(0.01, 0.005, note or "Measured: where each crawl felt the drop minus the cameras' outline, per side "
             "(feel_table.py).", fontsize=7.5, color="#5B6474")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(png, dpi=130)
    plt.close(fig)
    return png


# ---------------------------------------------------------------------------------------------------------- command

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    a1 = sub.add_parser("plan")
    a1.add_argument("--map", required=True)
    a1.add_argument("--out", required=True)
    a2 = sub.add_parser("analyze")
    a2.add_argument("--map", required=True)
    a2.add_argument("--session", required=True)
    a2.add_argument("--conf-dir", default="")
    a2.add_argument("--out")
    a3 = sub.add_parser("status")
    a3.add_argument("--conf-dir", required=True)
    a3.add_argument("--table", required=True, help="e.g. 396x168")
    a3.add_argument("--tagcal", default="")
    a = ap.parse_args(argv)
    if a.cmd == "plan":
        m, sp = plan_tasks(FM.load(a.map))
        FM.save(m, a.out)
        with open(os.path.splitext(a.out)[0] + "_spots.json", "w") as f:
            json.dump(sp, f, indent=1)
        print("feel-out: %s -> %s" % ({k: len(v["visit"]) for k, v in m["tasks"].items()}, a.out))
    elif a.cmd == "analyze":
        res = analyze(FM.load(a.map), a.session, conf_dir=a.conf_dir or None)
        print("\n".join(res["words"]))
        with open(a.out or os.path.join(a.session, "feel_result.json"), "w") as f:
            json.dump(res, f, indent=1)
    else:
        need, cal, why = needed(a.conf_dir, a.table, setup_id(a.tagcal) if a.tagcal else None)
        print(("feel-out NEEDED: " if need else "feel-out skipped: ") + why)
        for w in (cal or {}).get("words") or []:
            print("  " + w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
