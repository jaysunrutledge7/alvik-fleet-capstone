#!/usr/bin/env python3
"""Production points and production runs (conference-table build, Nick 2026-11-02).

  points    the numbered stickers on the table, each a production point: its label (the sticker's number), its spot
            on the table (from the site fit: clicked in the cameras' pictures), optional typed coordinates (a check).
            Saved per site in <conf_dir>/tagcal/points.json (tag_setup.py) and as spots in Mission Control's map.
  homes     the robots park near the table's middle, equidistant from it, HOME_GAP_CM apart (centre to centre), along
            the table's length, all facing the same way (+y). The middle is in both cameras' pictures.
  run       each robot: start-up (run_plan: calibrated since power-on? if not, it calibrates; calibrated today on this
            power-on: kept), then its production points in the order Nick chose, waiting DWELL_S on each (still
            readings for the external recorder), then home. The planner keeps the two 8 cm apart (the net); a point
            the robots cannot reach safely is moved in from the edge and said, never a stop.

    python3 tools/fleet/production.py --table 396x168 --points 1:40,30 2:300,120 --order Alpha=1,2 --order Bravo=2,1
"""
import argparse
import copy
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM     # noqa: E402
import workspace as WS    # noqa: E402

HOME_GAP_CM = 30.0       # chosen: centre to centre of the two homes. Each robot's turn circle is 6.72 cm (the 9.5 cm
                         # square), so two spinning in place keep 30 - 2 x 6.72 = 16.6 cm, twice the 8 cm net. DOE note:
                         # 22 / 30 / 40 cm against the closest approach while both leave and come home together
HOME_FACE_DEG = 90.0     # homes face +y (across the table, away from corner 1's long side). No DOE: tidy and the same
DWELL_S = 3.0            # each production point: the robot stands this long (the external recorder gets still reads to
                         # compare). Claude's addition, not asked for: 0 turns it off. DOE note: 0 / 3 / 10 s against
                         # the RFID position's spread at a still robot
EDGE_IN_CM = 17.0        # a point closer than this to an open edge (the robot's centre) is moved in to it: Nick's
                         # edge room "everywhere in the mission" (missions.SETTINGS edge_room_cm, 28 Oct 21:27: the
                         # body's nearest corner 10 cm from an open edge by plan, turns included). No DOE: Nick's rule


def homes(L, W, names, gap_cm=HOME_GAP_CM, x0=0.0, y0=0.0):
    """{robot: [x, y, heading]}: equidistant from the table's middle along x, gap_cm apart, facing HOME_FACE_DEG."""
    n = len(names)
    cx, cy = x0 + L / 2.0, y0 + W / 2.0
    out = {}
    for k, nm in enumerate(names):
        off = (k - (n - 1) / 2.0) * gap_cm
        out[nm] = [round(cx + off, 2), round(cy, 2), HOME_FACE_DEG]
    return out


def clamp_points(points, L, W, room_cm=EDGE_IN_CM):
    """Points too near an edge moved in to room_cm. Returns ({label: [x, y]}, [words for each moved])."""
    out, words = {}, []
    for lab, xy in points.items():
        x, y = float(xy[0]), float(xy[1])
        nx, ny = min(max(x, room_cm), L - room_cm), min(max(y, room_cm), W - room_cm)
        if abs(nx - x) > 1e-6 or abs(ny - y) > 1e-6:
            words.append("WARNING: point %s at (%.1f, %.1f) is %.1f cm from an open edge: the robot goes to (%.1f, %.1f) "
                         "instead (its centre stays %.0f cm in)" % (lab, x, y, min(x, y, L - x, W - y), nx, ny, room_cm))
        out[str(lab).upper()] = [round(nx, 2), round(ny, 2)]
    return out, words


def site_map(table, names, points=None, gap_cm=HOME_GAP_CM):
    """The workspace map of a site's table: the measured rectangle, every side an open edge (a conference table has no
    walls), the production points as spots, each robot's start and park at its home."""
    L, W = table
    hs = homes(L, W, names, gap_cm)
    m = WS.blank_map((0, 0, L, W), [(n, hs[n]) for n in names])
    for k in FM.SIDES:
        m = WS.set_side(m, k, "open", source="conference table (Nick: no walls)")
    pts, words = clamp_points(points or {}, L, W)
    m = copy.deepcopy(m)
    m["spots"] = pts
    m["notes"] = ["production points from the site fit"] + words
    m.setdefault("homes", hs)
    return FM.normalize(m), words


def save_map(mw, path):
    m = mw[0] if isinstance(mw, tuple) else mw
    FM.save(m, path)
    return path


def program(orders, dwell_s=DWELL_S, home=True, name="production"):
    """A production program (missions.build blocks). orders: {robot: [point labels in order]}."""
    blocks = []
    robots = list(orders)
    for n, labs in orders.items():
        for lab in labs:
            blocks.append({"block": "go-to-spot", "robot": n, "spot": str(lab).upper(),
                           "label": "point %s" % str(lab).upper()})
            if dwell_s > 0:
                blocks.append({"block": "wait", "robots": [n], "seconds": float(dwell_s),
                               "label": "at point %s" % str(lab).upper()})
        if home:
            blocks.append({"block": "go-home", "robot": n})
    return {"name": name, "robots": robots, "blocks": blocks, "production": {"orders": orders, "dwell_s": dwell_s,
                                                                             "home": home}}


def parse_orders(specs):
    out = {}
    for s in specs:
        n, _, labs = s.partition("=")
        out[n.strip()] = [x.strip().upper() for x in labs.split(",") if x.strip()]
    return out


def check_orders(orders, points):
    """Plain words for any label that is not a point (raised as ValueError by the caller)."""
    bad = []
    for n, labs in orders.items():
        for lab in labs:
            if str(lab).upper() not in points:
                bad.append("%s: no point %s (points: %s)" % (n, lab, ", ".join(sorted(points)) or "none yet"))
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--table", default="396x168")
    ap.add_argument("--points", nargs="*", default=[], help="LABEL:x,y")
    ap.add_argument("--order", action="append", default=[], help="ROBOT=1,2,3")
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    L, W = (float(v) for v in a.table.split("x"))
    pts = {}
    for p in a.points:
        lab, _, xy = p.partition(":")
        pts[lab] = [float(v) for v in xy.split(",")]
    orders = parse_orders(a.order)
    m, words = site_map((L, W), list(orders), pts)
    bad = check_orders(orders, m["spots"])
    if bad:
        raise SystemExit("; ".join(bad))
    prog = program(orders)
    print("\n".join(words))
    print(json.dumps(prog, indent=1))
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        save_map(m, os.path.join(a.out, "map.json"))
        with open(os.path.join(a.out, "program.json"), "w") as f:
            json.dump(prog, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
