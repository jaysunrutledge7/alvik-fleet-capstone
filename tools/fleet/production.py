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

import numpy as np

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


def clamp_points(points, L, W, room_cm=EDGE_IN_CM, x0=0.0, y0=0.0):
    """Points too near an edge moved in to room_cm. Returns ({label: [x, y]}, [words for each moved]). The table runs
    from (x0, y0) to (x0 + L, y0 + W) (an outline the feel-out moved: feel_table.py)."""
    out, words = {}, []
    for lab, xy in points.items():
        x, y = float(xy[0]), float(xy[1])
        nx = min(max(x, x0 + room_cm), x0 + L - room_cm)
        ny = min(max(y, y0 + room_cm), y0 + W - room_cm)
        if abs(nx - x) > 1e-6 or abs(ny - y) > 1e-6:
            words.append("WARNING: point %s at (%.1f, %.1f) is %.1f cm from an open edge: the robot goes to (%.1f, %.1f) "
                         "instead (its centre stays %.0f cm in)" % (lab, x, y, min(x - x0, y - y0, x0 + L - x, y0 + W - y),
                                                                    nx, ny, room_cm))
        out[str(lab).upper()] = [round(nx, 2), round(ny, 2)]
    return out, words


BODY_R_CM = 6.8           # the robot's half diagonal (9.5 cm square): a point's robot stays this far off a keep-out


def _nearest_on_poly(p, poly):
    """(nearest point on the polygon's boundary, inside?) for a convex polygon."""
    P = [np.asarray(v, float) for v in poly]
    best, bd = None, 1e18
    inside = True
    sgn = None
    for k in range(len(P)):
        a, b = P[k], P[(k + 1) % len(P)]
        ab = b - a
        t = max(0.0, min(1.0, float(np.dot(p - a, ab) / max(1e-12, np.dot(ab, ab)))))
        q = a + t * ab
        d = float(np.linalg.norm(p - q))
        if d < bd:
            best, bd = q, d
        cr = ab[0] * (p - a)[1] - ab[1] * (p - a)[0]
        if sgn is None:
            sgn = cr >= 0
        elif (cr >= 0) != sgn:
            inside = False
    return best, inside


def clear_keepouts(points, keepouts, r=BODY_R_CM):
    """Points whose robot would overlap a keep-out moved just clear of it. Returns (points, words)."""
    out, words = {}, []
    for lab, xy in points.items():
        p = np.array(xy[:2], float)
        moved = False
        for _ in range(3):
            for k in keepouts or []:
                q, inside = _nearest_on_poly(p, k["polygon"])
                d = float(np.linalg.norm(p - q))
                if inside or d < r:
                    v = (p - q) / d if d > 1e-9 else np.array([0.0, 1.0])
                    if inside:
                        c = np.mean(np.asarray(k["polygon"], float), axis=0)
                        v = (q - c) / max(1e-9, np.linalg.norm(q - c))
                    p = q + v * (r + 0.5)
                    moved = True
        if moved:
            words.append("WARNING: point %s at (%.1f, %.1f) was on or beside a reference tile: the robot goes to "
                         "(%.1f, %.1f) instead" % (lab, xy[0], xy[1], p[0], p[1]))
        out[lab] = [round(float(p[0]), 2), round(float(p[1]), 2)]
    return out, words


def site_map(table, names, points=None, gap_cm=HOME_GAP_CM, outline=None, keepouts=None):
    """The workspace map of a site's table: the measured rectangle, every side an open edge (a conference table has no
    walls), the production points as spots, each robot's start and park at its home. outline [x0, y0, x1, y1]: the
    table's outline when it is not (0, 0, L, W) (the feel-out moved a side: feel_table.py); table is then ignored.
    keepouts: areas no robot drives over (the reference tags' tiles, 2026-11-03: ref_tags.keepouts)."""
    if outline:
        x0, y0, x1, y1 = (float(v) for v in outline)
    else:
        x0, y0, x1, y1 = 0.0, 0.0, float(table[0]), float(table[1])
    L, W = x1 - x0, y1 - y0
    hs = homes(L, W, names, gap_cm, x0=x0, y0=y0)
    m = WS.blank_map((x0, y0, x1, y1), [(n, hs[n]) for n in names])
    for k in FM.SIDES:
        m = WS.set_side(m, k, "open", source="conference table (Nick: no walls)")
    pts, words = clamp_points(points or {}, L, W, x0=x0, y0=y0)
    m = copy.deepcopy(m)
    if keepouts:
        pts, w2 = clear_keepouts(pts, keepouts)
        words = words + w2
    m["spots"] = pts
    if keepouts:
        m["keepouts"] = list(m.get("keepouts") or []) + [dict(k) for k in keepouts]
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
