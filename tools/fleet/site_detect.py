#!/usr/bin/env python3
"""Where am I? (build e07, Nick 6 Nov: "one Mission Control that detects by itself whether it is at home, at school or
in a new area"). Every site is a starting guess (Observe, then trust): the evidence decides, the cameras most.

Evidence, strongest first (each scored for every known site; the best score wins when it is clear):
  picture    reference tags on the table (the school's corner tiles, ids 20-25) seen in a camera's picture; the
             table's size the cameras measured (Measure) against each site's table
  cameras    how many cameras are plugged in and their names (the school's C920e alone; home's two:
             tools/pi/camnames.py DEFAULTS)
  computer   this computer's name (home's Pi st1maxfini, the school's rfidthesis): the weakest, a hint only
  last       the site used last time (~/.config/mirlt/site): breaks a tie, never beats a camera

Nothing that matches well enough -> a NEW AREA: a site of its own, made from the closest kind (roof tags if any tag is
seen, else home's lights), with nothing typed (the cameras measure the table), saved in ~/.config/mirlt/sites-local/.

A site chosen by hand stays chosen only when it is pinned (`sites.py use <name> --pin`; Mission Control: Advanced,
"Always use this site"). Unpinned, the saved choice is only the "last" hint above.

    python3 tools/fleet/site_detect.py                 # what it sees and what it decides, in words
    python3 tools/fleet/site_detect.py --decide        # the site name alone (the launcher uses it)
    python3 tools/fleet/site_detect.py --room school   # a simulated room's evidence (rehearsals): home / school / new
"""
import argparse
import datetime as dt
import json
import os
import socket
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "pi")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sites as SI      # noqa: E402

# scores (chosen, no DOE: an order of trust, not a measurement; the rehearsal table shows each case decided)
W_REF_TAGS = 6.0         # the site's own reference tag ids seen in a picture: decisive
W_NO_REF_TAGS = 2.0      # a picture seen with NO reference tag where a site has them: against that site
W_TABLE = 5.0            # the measured table within TABLE_TOL of a site's table
W_TABLE_OFF = 5.0        # ... or far from it: against
W_CAMERA_NAMES = 3.0     # every plugged camera's name matches a site's camera names
W_CAMERA_COUNT = 1.5     # the number of cameras the site uses
W_HOST = 1.0             # this computer's name is the site's Pi
W_LAST = 0.5             # the site used last time
CLEAR = 2.0              # the winner must lead the next by this much, and score at least ENOUGH, else ...
ENOUGH = 3.0             # ... it is a new area
TABLE_TOL = 0.06         # a measured table within 6 % of a site's (both sides) is "that table"
LOCAL_DIR = os.path.join(SI.base_conf_dir(), "sites-local")
GRID_CM = 30.0           # a new area's points: a grid this far apart (chosen: two robot lengths and room to turn; DOE
                         # note: 20 / 30 / 40 cm against points done per minute and robot gaps, simulator)
GRID_EDGE_CM = 17.0      # ... this far in from the edges (home's edge room, sites.py edge_in_cm default)


def site_facts(site):
    """What a site expects to see: {ref_ids, camera_names, n_cameras, table_cm, host}."""
    rt = site.get("ref_tags") or {}
    ref_ids = set()
    if rt.get("use", True) and rt:
        ref_ids = {int(v) for v in list((rt.get("corners") or {}).values()) + list((rt.get("sides") or {}).values())}
    cams = site.get("cameras") or {}
    n_cam = sum(1 for k, v in cams.items() if (v or {}).get("use", True) is not False)
    names = []
    if site["name"] == "home":
        try:
            import camnames as CN
            names = [v for v in CN.DEFAULTS.values() if v]
        except Exception:   # noqa: BLE001
            names = []
    else:
        cn = site.get("camera_names") or {}
        names = [str(cn.get(r)) for r in ("overhead", "side") if cn.get(r)]
    tb = site.get("table") or {}
    table = (float(tb["length_cm"]), float(tb["width_cm"])) if tb.get("length_cm") else \
        ((122.24, 121.92) if site["name"] == "home" else None)      # home: the 4 x 4 ft plywood (freeroam HOME_BOARD)
    return {"ref_ids": ref_ids, "camera_names": names, "n_cameras": n_cam, "table_cm": table,
            "host": (site.get("pi") or {}).get("hostname")}


def _table_match(m, t):
    if not m or not t:
        return None
    a, b = sorted(m, reverse=True), sorted(t, reverse=True)
    return all(abs(x - y) <= TABLE_TOL * y for x, y in zip(a, b))


def score(site, ev):
    """(score, [reasons]) of one site against the evidence ev: {ref_ids_seen (set or None: no picture),
    camera_names [..] or None, n_cameras or None, table_cm (L, W) or None, host, last}."""
    f = site_facts(site)
    s, why = 0.0, []
    seen = ev.get("ref_ids_seen")
    if seen is not None:
        if f["ref_ids"] and set(seen) & f["ref_ids"]:
            s += W_REF_TAGS
            why.append("its reference tags %s are on the table" % ", ".join(str(i) for i in sorted(set(seen) & f["ref_ids"])))
        elif f["ref_ids"] and not seen:
            s -= W_NO_REF_TAGS
            why.append("none of its reference tags is in the picture")
        elif not f["ref_ids"] and seen:
            s -= W_NO_REF_TAGS
            why.append("reference tags are on the table, and it has none")
    tm = _table_match(ev.get("table_cm"), f["table_cm"])
    if tm is True:
        s += W_TABLE
        why.append("the table measures %.0f x %.0f cm, its size" % tuple(ev["table_cm"]))
    elif tm is False:
        s -= W_TABLE_OFF
        why.append("the table measures %.0f x %.0f cm, not its %.0f x %.0f" % (tuple(ev["table_cm"]) + tuple(f["table_cm"])))
    names = ev.get("camera_names")
    if names is not None and f["camera_names"]:
        low = [n.lower() for n in names]
        hit = [w for w in f["camera_names"] if any(w.lower() in n for n in low)]
        if names and len(hit) == len(f["camera_names"]):
            s += W_CAMERA_NAMES
            why.append("its camera%s %s plugged in" % ("s" if len(hit) > 1 else "", " and ".join(hit)))
        elif names and not hit:
            s -= W_CAMERA_NAMES / 2
            why.append("none of its cameras (%s) is plugged in" % ", ".join(f["camera_names"]))
    n = ev.get("n_cameras")
    if n is not None and n == f["n_cameras"]:
        s += W_CAMERA_COUNT
        why.append("%d camera%s, as it uses" % (n, "" if n == 1 else "s"))
    if ev.get("host") and f["host"] and ev["host"].split(".")[0].lower() == str(f["host"]).lower():
        s += W_HOST
        why.append("this computer is its Pi (%s)" % f["host"])
    if ev.get("last") == site["name"]:
        s += W_LAST
        why.append("used last time")
    return s, why


def known_sites():
    out = []
    for n in SI.names():
        try:
            out.append(SI.load(n))
        except SystemExit:
            continue
    return out


def decide(ev, sites=None):
    """{site: name or "new", kind: "home" | "school" | "new", scores {name: s}, why [words], words (one line)}."""
    sites = sites if sites is not None else known_sites()
    rows = sorted(((score(s, ev), s["name"]) for s in sites), key=lambda r: -r[0][0])
    scores = {n: round(sc, 1) for (sc, _w), n in rows}
    (best, why), name = rows[0]
    nxt = rows[1][0][0] if len(rows) > 1 else -99.0
    pinned = ev.get("pinned")
    if pinned and pinned in [s["name"] for s in sites]:
        return {"site": pinned, "kind": kind_of(pinned), "scores": scores, "why": ["chosen by hand (pinned)"],
                "words": "Using %s: chosen by hand (Advanced: Always use this site)." % title_of(pinned, sites)}
    if not ev.get("camera_names") and ev.get("ref_ids_seen") is None and not ev.get("table_cm"):
        # no camera at all (not plugged in yet, or not a Pi): nothing to go on; the last site stays, said
        last = ev.get("last") if ev.get("last") in [s["name"] for s in sites] else SI.DEFAULT_SITE
        return {"site": last, "kind": kind_of(last), "scores": scores, "why": ["no camera found"],
                "words": "No camera found yet: using %s (the last one used) until a camera shows where this is."
                         % title_of(last, sites), "no_evidence": True}
    if best >= ENOUGH and best - nxt >= CLEAR:
        pos = [w for w in why if not w.startswith(("none of", "reference tags are on", "used last"))][:2]
        return {"site": name, "kind": kind_of(name), "scores": scores, "why": why,
                "words": "This looks like %s (%s)." % (title_of(name, sites), " and ".join(pos) or "best match")}
    tagged = bool(ev.get("ref_ids_seen")) or bool(ev.get("robot_tags_seen"))
    return {"site": "new", "kind": "new", "template": "school-conference" if tagged else "home", "scores": scores,
            "why": why, "words": "A new area: nothing here matches a known table well enough (%s). The cameras measure "
                                 "it; nothing needs typing." % ", ".join("%s %.1f" % kv for kv in scores.items())}


def kind_of(name):
    return "home" if name == "home" else ("school" if name.startswith("school") else "other")


def title_of(name, sites):
    short = {"home": "home (the 4 x 4 ft board)", "school-conference": "the school conference table"}
    if name in short:
        return short[name]
    for s in sites:
        if s["name"] == name:
            return s.get("title") or name
    return name


# ------------------------------------------------------------------------------------------------- evidence
def hardware_evidence():
    """What can be known without a picture: cameras plugged in (names), this computer's name, the last site."""
    try:
        import camnames as CN
        cams = CN.scan()
        names = [c["name"] for c in cams]
    except Exception:   # noqa: BLE001
        names = None
    last = None
    try:
        with open(SI.chosen_file()) as f:
            last = (f.read().split() or [None])[0]
    except (OSError, IndexError):
        last = None
    return {"camera_names": names, "n_cameras": None if names is None else len(names),
            "host": socket.gethostname(), "last": last, "pinned": pinned_site()}


def picture_evidence(grey_by_cam):
    """{ref_ids_seen, robot_tags_seen} from the cameras' pictures ({camera: grey picture})."""
    import ref_tags as RT
    import tagcore as TC
    seen, robots = set(), set()
    try:
        det = TC.TagDetector(RT.FAMILY, ids=None)
        for g in grey_by_cam.values():
            if g is None:
                continue
            for d in det.detect_full(g):
                (seen if 20 <= d.id <= 29 else robots).add(int(d.id))
    except Exception:   # noqa: BLE001  (no detector: no picture evidence)
        return {}
    return {"ref_ids_seen": seen, "robot_tags_seen": robots}


SIM_ROOMS = {
    # the rehearsals' rooms (what each would show): the school (one C920e, its reference tags, its table), home (two
    # webcams, no tags, the 4 x 4 ft board) and a new area (one other camera, no reference tags, a 90 x 180 cm table)
    "school": {"camera_names": ["HD Pro Webcam C920e"], "n_cameras": 1, "ref_ids_seen": {20, 21, 23, 24, 25},
               "robot_tags_seen": {2}, "table_cm": (395.9, 167.4), "host": "rfidthesis"},
    "home": {"camera_names": ["USB 2.0 Camera", "EMEET SmartCam C950"], "n_cameras": 2, "ref_ids_seen": set(),
             "robot_tags_seen": set(), "table_cm": (122.0, 121.7), "host": "st1maxfini"},
    "new": {"camera_names": ["Logitech BRIO"], "n_cameras": 1, "ref_ids_seen": set(), "robot_tags_seen": {0, 1},
            "table_cm": (182.0, 91.0), "host": "classroom-pi"},
    # broken guesses: the school with a camera named like home's and the last site saved as home
    "school-misnamed": {"camera_names": ["USB 2.0 Camera"], "n_cameras": 1, "ref_ids_seen": {20, 23, 24},
                        "robot_tags_seen": {3}, "table_cm": None, "host": "st1maxfini", "last": "home"},
}


# ------------------------------------------------------------------------------------------------- pins, new sites
def pin_file():
    return os.path.join(SI.base_conf_dir(), "site-pinned")


def pinned_site():
    try:
        with open(pin_file()) as f:
            v = f.read().strip()
        return v or None
    except OSError:
        return None


def set_pin(name):
    os.makedirs(SI.base_conf_dir(), exist_ok=True)
    if not name:
        try:
            os.remove(pin_file())
        except OSError:
            pass
        return None
    SI.load(name)
    with open(pin_file(), "w") as f:
        f.write(name + "\n")
    return name


def make_new_site(template="school-conference", table_cm=None, when=None):
    """A new area's site, from the closest kind: the template's ways (tracking, tags, policies) with nothing of the
    old room in it (no typed table, points, keep-outs, homes, pins, robot addresses, camera names). Saved in
    LOCAL_DIR as new-<date>.json (sites.load finds it). Returns its name."""
    import copy
    base = SI.load(template)
    d = copy.deepcopy({k: v for k, v in base.items() if k not in ("name", "conf_dir", "mc_dir")})
    when = when or dt.datetime.now()
    name = "new-%s" % when.strftime("%m%d")
    d["title"] = "New area (found %s; the cameras measured it)" % when.strftime("%d %b %H:%M")
    d["table"] = None if not table_cm else {"length_cm": round(max(table_cm), 1), "width_cm": round(min(table_cm), 1),
                                             "how": "measured by the cameras (site_detect)"}
    for k in ("points", "keepouts", "homes", "missing_stickers", "guesses", "experiments", "camera_hw", "pi",
              "camera_names"):
        d.pop(k, None)
    d.pop("production", None)
    for r in d.get("robots", []):
        r["ip"] = None
    if isinstance(d.get("tags"), dict):
        d["tags"].pop("pins", None)
    if isinstance(d.get("calibration"), dict):
        d["calibration"].pop("robot_ips", None)
        d["calibration"].pop("park_spots", None)
    d["measure_trust"] = "cameras"
    d["_note_e07"] = "Made by site_detect (e07) from %s: a new area. Nothing typed: the cameras measure the table." % template
    os.makedirs(LOCAL_DIR, exist_ok=True)
    if d.get("tracking") == "tags" and table_cm:
        # points to visit, discovered from the table the cameras measured (no stickers here): a grid GRID_CM apart,
        # EDGE_IN_CM in from the edges, in four zones along the table (runs share what the camera reads)
        L, W = max(table_cm), min(table_cm)
        import math
        nx = max(2, int(math.floor((L - 2 * GRID_EDGE_CM) / GRID_CM)) + 1)
        ny = max(1, int(math.floor((W - 2 * GRID_EDGE_CM) / GRID_CM)) + 1)
        rows, k = [], 1
        for i in range(nx):
            x = GRID_EDGE_CM + (L - 2 * GRID_EDGE_CM) * (i / float(nx - 1))
            for j in range(ny):
                y = W / 2.0 if ny == 1 else GRID_EDGE_CM + (W - 2 * GRID_EDGE_CM) * (j / float(ny - 1))
                rows.append("P%02d,%.2f,%.2f,%d" % (k, x / 2.54, y / 2.54, min(4, 1 + int(4 * x / L))))
                k += 1
        pf = os.path.join(LOCAL_DIR, name + "-points.csv")
        with open(pf, "w") as f:
            f.write("id,x_in,y_in,zone\n" + "\n".join(rows) + "\n")
        d["points"] = {"use": "typed", "file": pf, "units": "in", "flag_cm": 3.0, "overrides": [],
                       "_note": "made from the measured table (site_detect): a grid, not stickers"}
        d["edge_in_cm"] = GRID_EDGE_CM
        d["homes"] = {"use": "middle"}
        d["production"] = {"zones": {}, "order": "nearest+2opt", "orders": None, "dwell_s": 3.0, "split": "auto"}
    with open(os.path.join(LOCAL_DIR, name + ".json"), "w") as f:
        json.dump(d, f, indent=1)
    return name


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--decide", action="store_true", help="print the site's name only")
    ap.add_argument("--room", choices=sorted(SIM_ROOMS), help="a simulated room's evidence (rehearsals)")
    a = ap.parse_args(argv)
    ev = dict(SIM_ROOMS[a.room]) if a.room else hardware_evidence()
    r = decide(ev)
    if a.decide:
        print(r["site"] if r["site"] != "new" else "")
        return 0
    print(r["words"])
    print("scores: %s" % r["scores"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
