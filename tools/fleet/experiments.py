#!/usr/bin/env python3
"""Experiment routes for Jay-Sun's RFID training runs (school build, Nick and Jay-Sun 2026-11-03).

Jay-Sun (3 Nov): "for training I need a few different point orders (not the same order every run), the robots swapping
halves every other run, and some legs that cross the middle line at x = 78 in. Same speed in all four experiments."
"I'll treat the first no-tag odometry run as a shakedown, not one of the four experiments."

  shakedown  the no-camera odometry run (tracking none) with the site's default orders (nearest neighbour + 2-opt,
             each robot its own half): not one of the four experiments
  e1         in order: each robot's half by point number (P01, P02, ...)
  e2         reversed: each robot's half by number, highest first
  e3         zig-zag: each robot alternates between the two zones of its half (each zone swept across the table's
             width), with points of the other half put in at even spacing (legs over x = 78 in)
  e4         shuffled: each robot's points in a random order (fixed seed, recorded), points of the other half included
  default    the site's default orders (as before; also "custom": the orders typed on the Production screen)

Halves: the site's production zones (Alpha zones 1-2, Bravo 3-4 at school). Swapped: Alpha takes the other half and
Bravo Alpha's. The homes do NOT move: each robot starts and ends on its own home sticker (P11 / P22), so a swapped run
drives over the middle at its start and end (Claude's choice: nobody re-places robots between runs; see the report).
Cross-middle legs: in e2-e4 each robot also takes CROSS points of the other robot's half (the same number each way, so
every point is still done once per run), put in at even spacing so each makes two legs over the middle line.
Same speed: the site's experiments.speed (production drive, turn and final-approach speeds) is pinned in the planner
for every experiment and written into the run's summary.

Run counter (swap halves every other run): <site conf_dir>/experiments_counter.json, one count per production run
started on the robots with an experiment (simulator runs never count). auto = swapped when the count so far is odd.

    python3 tools/fleet/experiments.py --site school-conference                 # every experiment's orders, crossings
    python3 tools/fleet/experiments.py --site school-conference --exp e4 --swap on
"""
import argparse
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

VERSION = "2026-11-03a"
KINDS = {
    "shakedown": {"name": "Shakedown (no camera, default orders)", "order": "default", "tracking": "none",
                  "experiment": False},
    "e1": {"name": "Experiment 1: in order", "order": "inorder", "experiment": True},
    "e2": {"name": "Experiment 2: reversed", "order": "reversed", "experiment": True},
    "e3": {"name": "Experiment 3: zig-zag across zones", "order": "zigzag", "experiment": True},
    "e4": {"name": "Experiment 4: shuffled", "order": "shuffled", "experiment": True},
    "default": {"name": "Default orders (nearest neighbour + 2-opt, own half)", "order": "default",
                "experiment": False},
}
MIDDLE_IN = 78.0          # Jay-Sun's middle line (the cameras' split), x in inches
CROSS_POINTS = 3          # chosen: points of the other half each robot takes in e2-e4 (2 legs over the middle each, so
                          # >= 12 crossing legs per run with both robots). DOE note: 2 / 3 / 5 against the run's time
                          # and the RFID model's error on held-out crossing legs (Jay-Sun's data decides)
ORDER_SEED = 20261103     # chosen: the shuffle's and the traded points' seed (recorded in each run). No DOE: any seed
MIN_CROSS_LEGS = 4        # each robot at least this many legs over the middle in e2-e4 (the shuffle is redrawn, the
                          # next seed recorded, until it holds). No DOE: a floor so "some legs" is never zero
SPEED = {"cruise_cms": 10.0, "turn_dps": 60.0, "confirm_cms": 5.0}   # the production speeds the planner has used
                          # since the conference-table build (planner.DEFAULTS): pinned so all experiments match. No DOE:
                          # Jay-Sun asked for one speed; changing it means re-running every experiment
COUNTER_FILE = "experiments_counter.json"


def conf(site):
    """The site's experiment settings ("experiments": {...}) with the defaults above filled in."""
    e = dict((site or {}).get("experiments") or {})
    e.setdefault("middle_in", MIDDLE_IN)
    e.setdefault("cross_points", CROSS_POINTS)
    e.setdefault("seed", ORDER_SEED)
    e.setdefault("swap_halves", "auto")
    e["speed"] = dict(SPEED, **(e.get("speed") or {}))
    w = dict(e.get("sync_wiggle") or {})
    w.setdefault("use", False)
    w.setdefault("deg", 90.0)
    w.setdefault("pause_s", 1.0)
    e["sync_wiggle"] = w
    return e


def num(lab):
    d = "".join(c for c in str(lab) if c.isdigit())
    return int(d) if d else 0


def halves(site, labels, zones, swap=False):
    """{robot: [labels]} from the site's production zones; swap: the two robots' halves exchanged."""
    pr = (site or {}).get("production") or {}
    zs = pr.get("zones") or {}
    out = {n: [l for l in labels if zones.get(l) in set(int(z) for z in v)] for n, v in zs.items()}
    if swap and len(out) == 2:
        a, b = list(out)
        out = {a: out[b], b: out[a]}
    return out


def side(x_cm, middle_cm):
    return x_cm >= middle_cm


def crossings(order, pts, home_xy, middle_cm, back=True):
    """Legs over the middle line: home -> first point, each point -> the next, the last -> home."""
    path = [list(home_xy[:2])] + [pts[l] for l in order] + ([list(home_xy[:2])] if back else [])
    return sum(1 for a, b in zip(path, path[1:]) if side(a[0], middle_cm) != side(b[0], middle_cm))


def spread_in(base, extra):
    """extra put into base at even spacing (never two extras side by side when base has room)."""
    if not extra:
        return list(base)
    out, n, k = [], len(base), len(extra)
    gaps = [int(round((j + 1) * n / float(k + 1))) for j in range(k)]
    e = list(extra)
    for i, l in enumerate(base):
        out.append(l)
        while e and gaps and gaps[0] == i + 1:
            out.append(e.pop(0))
            gaps.pop(0)
    return out + e


def zigzag(own, zones, pts):
    """Alternate between the half's two zones; each zone's points taken across the table's width (by y, then x)."""
    zs = sorted(set(zones[l] for l in own))
    lists = [sorted([l for l in own if zones[l] == z], key=lambda l: (pts[l][1], pts[l][0])) for z in zs]
    out = []
    while any(lists):
        for L in lists:
            if L:
                out.append(L.pop(0))
    return out


def trade(hv, pts, k, rng, middle_cm):
    """Each robot gives k of its points to the other (chosen at random with rng; the same number each way)."""
    names = list(hv)
    if len(names) != 2 or k <= 0:
        return {n: [] for n in names}, {n: list(v) for n, v in hv.items()}
    a, b = names
    ga = sorted(rng.sample(sorted(hv[a], key=num), min(k, len(hv[a]))), key=num)   # a's points that b takes
    gb = sorted(rng.sample(sorted(hv[b], key=num), min(k, len(hv[b]))), key=num)
    keep = {a: [l for l in hv[a] if l not in ga], b: [l for l in hv[b] if l not in gb]}
    return {a: gb, b: ga}, keep


def experiment_orders(site, pts, zones, homes_xy, exp, swap=False, seed=None):
    """({robot: [labels]}, meta) for experiment exp on the points pts ({label: [x, y]} cm, the map's spots), zones
    ({label: zone}), homes_xy ({robot: [x, y, ...]})."""
    import production as PR
    if exp not in KINDS:
        raise ValueError("no experiment %r (choose %s)" % (exp, ", ".join(KINDS)))
    c = conf(site)
    kind = KINDS[exp]["order"]
    mid = float(c["middle_in"]) * 2.54
    seed = int(c["seed"] if seed is None else seed)
    labels = sorted(pts, key=num)
    hv = halves(site, labels, zones, swap)
    rng = random.Random("%d-%s" % (seed, exp))
    traded = {n: [] for n in hv}
    if kind in ("reversed", "zigzag", "shuffled"):
        traded, hv = trade(hv, pts, int(c["cross_points"]), rng, mid)
    out, seed_used = {}, seed
    for n, own in hv.items():
        if kind == "default":
            out[n] = PR.short_order(homes_xy[n][:2], own, pts)
        elif kind == "inorder":
            out[n] = sorted(own, key=num)
        elif kind == "reversed":
            out[n] = spread_in(sorted(own, key=num, reverse=True), sorted(traded[n], key=num, reverse=True))
        elif kind == "zigzag":
            out[n] = spread_in(zigzag(own, zones, pts), traded[n])
        elif kind == "shuffled":
            allp = sorted(own + traded[n], key=num)
            for tries in range(200):
                r2 = random.Random("%d-%s-%s-%d" % (seed, exp, n, tries))
                cand = list(allp)
                r2.shuffle(cand)
                if not traded[n] or crossings(cand, pts, homes_xy[n], mid) >= MIN_CROSS_LEGS:
                    break
            out[n] = cand
            seed_used = "%d-%s-%s-%d" % (seed, exp, n, tries) if n == list(hv)[0] else seed_used
    meta = {"id": exp, "name": KINDS[exp]["name"], "order_kind": kind, "is_experiment": KINDS[exp]["experiment"],
            "swap_halves": bool(swap), "seed": seed, "middle_x_in": c["middle_in"],
            "cross_points_each": int(c["cross_points"]) if kind in ("reversed", "zigzag", "shuffled") else 0,
            "traded": {n: v for n, v in traded.items() if v},
            "orders": out, "speed": dict(c["speed"]), "version": VERSION,
            "crossings": {n: crossings(v, pts, homes_xy[n], mid) for n, v in out.items()},
            "halves": {n: sorted(set(zones[l] for l in v)) for n, v in halves(site, labels, zones, swap).items()}}
    if kind == "shuffled":
        meta["shuffle_seeds"] = {n: "%d-%s-%s-<try>" % (seed, exp, n) for n in out}
    return out, meta


# ---------------------------------------------------------------- the run counter (swap halves every other run)
def counter_path(site):
    return os.path.join((site or {}).get("conf_dir") or os.path.expanduser("~/.config/mirlt"), COUNTER_FILE)


def counter_read(site):
    try:
        with open(counter_path(site)) as f:
            d = json.load(f)
        return {"runs": int(d.get("runs", 0)), "history": list(d.get("history") or [])}
    except (OSError, ValueError):
        return {"runs": 0, "history": []}


def swap_for(site, mode="auto"):
    """(swapped?, words) for mode on|off|auto (auto: swapped when the count of runs so far is odd)."""
    mode = str(mode or "auto").lower()
    if mode in ("on", "true", "yes", "1"):
        return True, "halves swapped (chosen)"
    if mode in ("off", "false", "no", "0"):
        return False, "halves as usual (chosen)"
    n = counter_read(site)["runs"]
    return n % 2 == 1, "run %d at this site: halves %s (every other run)" % (n + 1, "swapped" if n % 2 else "as usual")


def counter_add(site, rec):
    """One more production run started on the robots (rec: experiment, swap, session): saved in the site's folder."""
    d = counter_read(site)
    d["runs"] += 1
    d["history"].append(dict(rec, n=d["runs"], t=time.strftime("%Y-%m-%dT%H:%M:%S")))
    p = counter_path(site)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=1)
    os.replace(tmp, p)
    return d


# ---------------------------------------------------------------- the sync wiggle
def wiggle_on(site, override=None):
    if override in ("on", "off"):
        return override == "on"
    return bool(conf(site)["sync_wiggle"]["use"])


def add_wiggle(prog, site=None):
    """Both robots, at the plan's start (after start-up), at their homes: a quarter turn one way, stop, a quarter turn
    back, stop (missions block sync-wiggle). The step log has each part's start and end (wiggle_events)."""
    import copy
    w = conf(site)["sync_wiggle"]
    out = copy.deepcopy(prog)
    out["blocks"] = [{"block": "sync-wiggle", "robots": list(out["robots"]), "deg": float(w["deg"]),
                      "pause_s": float(w["pause_s"])}] + out["blocks"]
    out["sync_wiggle"] = {"deg": float(w["deg"]), "pause_s": float(w["pause_s"])}
    return out


def wiggle_events(coordinator_jsonl):
    """{robot: {"start": t, "end": t, "parts": [[label, t_started, t_done], ...]}} from run_plan's step log (Pi time,
    s): start = the first wiggle step started, end = the last one done."""
    st, out = {}, {}
    try:
        with open(coordinator_jsonl) as f:
            for ln in f:
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("event") != "labelled_step" or not str(e.get("label", "")).startswith("sync wiggle"):
                    continue
                k = (e.get("robot"), e.get("label"), e.get("i"))
                if e.get("state") == "started":
                    st[k] = e.get("t")
                elif e.get("state") == "done" and k in st:
                    r = out.setdefault(e.get("robot"), {"parts": []})
                    r["parts"].append([e.get("label"), st.pop(k), e.get("t")])
    except OSError:
        return {}
    for n, r in out.items():
        r["start"] = min(p[1] for p in r["parts"])
        r["end"] = max(p[2] for p in r["parts"])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default="school-conference")
    ap.add_argument("--exp", default="", help="one experiment (default: all)")
    ap.add_argument("--swap", default="off", choices=["on", "off", "auto"])
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    import sites as SI
    import production as PR
    site = SI.load(a.site)
    sw, words = swap_for(site, a.swap)
    out = {}
    for exp in ([a.exp] if a.exp else list(KINDS)):
        m, _w, prog, orders = PR.site_production(site, experiment=exp, swap=sw)
        meta = prog["production"]["experiment"]
        out[exp] = meta
        if not a.json:
            print("%s (%s; %s)" % (meta["name"], words, "speed %s" % meta["speed"]))
            for n, labs in orders.items():
                print("  %s: %s   [legs over x = %g in: %d]" % (n, ", ".join(labs), meta["middle_x_in"],
                                                                 meta["crossings"][n]))
    if a.json:
        print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
