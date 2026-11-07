#!/usr/bin/env python3
"""(e10) The one tracking layer replayed on a real free-roam session's logs: the bridge's fixes (bridge/track.csv) as
unnamed sightings, the robots' own poses (follow.csv, ~0.44 s after the picture), and which robot goes how long without
a fix. Logs only.

    python3 tools/analysis/layer_replay.py <session folder> [--gap-s 3] [--lag-s 0.44] [--watch]

7 Nov home run (20261107-122722-freeroam): with e09's layer Left went 13.2 s without a fix at 12:33:47 (an own-pose
re-set read as motion); with e10's none over 3 s. MIRLT_OWN_RESETS=0 gives the e09 behaviour.
"""
import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "fleet"))
import tracking_layer as TL   # noqa: E402


def replay(folder, lag_s=0.44, watch=False, gap_s=3.0):
    own = [(float(r["t_recv"]), r["robot"], float(r["x_cm"]), float(r["y_cm"]), float(r["th_deg"]), r["state"])
           for r in csv.DictReader(open(os.path.join(folder, "follow.csv"))) if r["state"] != "nopose"]
    fx = [(float(r["t"]), r["robot"], float(r["x"]), float(r["y"]))
          for r in csv.DictReader(open(os.path.join(folder, "bridge", "track.csv"))) if r["status"] == "seen"]
    names = sorted({q[1] for q in own} | {q[1] for q in fx})
    logs = []
    L = TL.TrackingLayer(names, TL.sources_for("lights"), camera_watch=watch, log=lambda k, r: logs.append(r))
    own.sort()
    batch = {}
    for t, n, x, y in sorted(fx):
        batch.setdefault(round(t, 3), []).append((n, x, y))
    seeded, oi, got = set(), 0, {n: [] for n in names}
    for tp in sorted(batch):
        while oi < len(own) and own[oi][0] <= tp + lag_s:
            t, n, x, y, th, st = own[oi]
            L.own(n, t - lag_s, x, y, th, st == "idle")
            oi += 1
        for n, x, y in batch[tp]:
            if n not in seeded:
                L.seed(n, tp, x, y, "start")
                seeded.add(n)
        for f in L.feed(tp, [TL.Det("motion", tp, x, y) for n, x, y in batch[tp]]):
            got[f["robot"]].append(tp)
    gaps = [dict(robot=n, t=round(a, 2), gap_s=round(b - a, 1)) for n in names for a, b in zip(got[n], got[n][1:])
            if b - a > gap_s]
    return dict(layer=TL.VERSION, own_resets=TL.OWN_RESETS, gaps=gaps, counts={k: v for k, v in L.counts.items() if v},
                fixes={n: len(v) for n, v in got.items()}, events=[r for r in logs if r.get("what") in (
                    "new object", "own pose re-set", "camera not trusted")][:50])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--gap-s", type=float, default=3.0)
    ap.add_argument("--lag-s", type=float, default=0.44)
    ap.add_argument("--watch", action="store_true", help="the camera watch on")
    a = ap.parse_args(argv)
    print(json.dumps(replay(a.folder, a.lag_s, a.watch, a.gap_s), indent=1))


if __name__ == "__main__":
    main()
