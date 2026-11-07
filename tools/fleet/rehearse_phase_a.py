#!/usr/bin/env python3
"""e08 Phase A rehearsals (7 Nov): each case before (the rules before e08, through their switches) and after (the e08
rules), many seeds, in the fast simulator (freeroam_fastsim.Sim: the same freeroam_core the Pi runs).

    python3 tools/fleet/rehearse_phase_a.py all --out docs/results/e08-rehearsals-2026-11-07 [--jobs 2]
    python3 tools/fleet/rehearse_phase_a.py a b c d       # some cases
    python3 tools/fleet/rehearse_phase_a.py e --hours 4   # the long one

Cases (docs/build-reports/e08-nov07-phase-a-edge-safety.md):
  a  6 Nov run 1: Left and Center; Center starts at its move-45 spot near the y = 0 edge; every 12 s its own pose jumps
     4.6 cm inward of the truth for 2 s with the camera's fixes refused (what the robot drove by on 6 Nov)
  b  a robot hung at an edge (a wheel over x = 0, 6.5 deg, the 6 Nov spot), the old "no room to back away"
  c  Right's heading crawl failed (the camera re-found it 15 cm off): before = out of the run; after = held, a nudge
  d  Right silent 3 min (deaf and mute), then back
  e  4 h, 6 false front stops per robot-hour (nothing there): keep-outs fading vs permanent (the rest as e08)
"""
import argparse
import json
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

OLD = dict(edge_rule="fixed", pose_rule="wait2s", keepout_rule="permanent", silent_out=True, tilt_recovery=False,
           heading_watch=False)
NEW = dict(edge_rule="derived", pose_rule="widen", keepout_rule="fade", silent_out=False, tilt_recovery=True,
           heading_watch=True)
FW = "2026-11-05d"


def case_runs(case, hours=None):
    """[(name, seed, kwargs for Sim)]"""
    out = []
    if case == "a":
        for sd in range(1, 13):
            off = [("Center", 5.0 + 12.0 * k, None, 4.6, 2.0) for k in range(30)]
            for tag, kw in (("before", OLD), ("after", NEW)):
                out.append(("a-" + tag, sd, dict(hours=hours or 0.1, names=("Left", "Center"), edge_physics=True,
                                                  starts={"Center": (31.0, 17.7, -121.0), "Left": (92.0, 82.0, 41.0)},
                                                  own_off=off, dead_robot=False, pi_freeze=False, ctrl_kw=kw)))
    elif case == "b":
        for sd in range(1, 11):
            for tag, kw in (("before", OLD), ("after", NEW)):
                out.append(("b-" + tag, sd, dict(hours=hours or 0.05, edge_physics=True,
                                                  starts={"Center": (3.5, 18.6, 117.0), "Left": (90.0, 80.0, 40.0),
                                                          "Right": (70.0, 50.0, -70.0)},
                                                  dead_robot=False, pi_freeze=False,
                                                  # (e10) Center stands for a robot hung DURING a run: the start
                                                  # check (which holds a robot put down at an edge) is off here
                                                  ctrl_kw=dict(kw, start_edge=False))))
    elif case == "c":
        for sd in range(1, 9):
            for tag, kw, held in (("before", OLD, "out"), ("after", NEW, "heading")):
                out.append(("c-" + tag, sd, dict(hours=hours or 0.1, edge_physics=True,
                                                  starts={"Right": (77.8, 28.0, -72.0), "Left": (92.0, 82.0, 41.0),
                                                          "Center": (44.0, 84.0, -66.0)},
                                                  own_start={"Right": (77.8, 28.0, -22.0)}, held={"Right": held},
                                                  cam_misplace=[("Right", 0.0, 8.0, 15.0, 0.0)],
                                                  dead_robot=False, pi_freeze=False, ctrl_kw=kw)))
    elif case == "d":
        for sd in range(1, 9):
            for tag, kw in (("before", OLD), ("after", NEW)):
                out.append(("d-" + tag, sd, dict(hours=hours or 0.15, edge_physics=True,
                                                  silent=[("Right", 60.0, 180.0)], dead_robot=False, pi_freeze=False,
                                                  ctrl_kw=kw)))
    elif case == "e":
        for sd in (1, 2):
            for tag, kw in (("after-permanent", dict(NEW, keepout_rule="permanent")), ("after-fade", NEW)):
                out.append(("e-" + tag, sd, dict(hours=hours or 4.0, edge_physics=True, phantoms=6.0, ctrl_kw=kw)))
    return out


def one(job):
    name, sd, kw = job
    import freeroam_fastsim as FS
    import safety_rules as SR
    kw = dict(kw)
    hours = kw.pop("hours")
    t0 = time.time()
    s = FS.Sim(sd, hours, fw=FW, **kw)
    # the robots' tilt stop as each rule sets it at the start (before: the firmware's 5 deg; after: 4 from the logs)
    for n in s.tilt_lim:
        s.tilt_lim[n] = 5.0 if kw.get("ctrl_kw", {}).get("edge_rule") == "fixed" else SR.tilt_limits()["tilt_stop_deg"]
    r = s.run()
    ev = s.__dict__.get("kept", {}).get("events", [])
    r["case"], r["wall_s"] = name, round(time.time() - t0, 1)
    r["rejoined_t"] = {e["robot"]: e["t"] for e in ev if e["kind"] == "rejoined"}
    r["recovered_t"] = [e["t"] for e in ev if e["kind"] == "tilt recovered"]
    r["events_kinds"] = {}
    for e in ev:
        r["events_kinds"][e["kind"]] = r["events_kinds"].get(e["kind"], 0) + 1
    r["recovery_rows"] = s.__dict__.get("kept", {}).get("recovery", [])[:20]
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="+")
    ap.add_argument("--out", default="")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--hours", type=float, default=None)
    a = ap.parse_args(argv)
    cases = "abcde" if "all" in a.cases else "".join(a.cases)
    jobs = [j for c in cases for j in case_runs(c, a.hours)]
    if a.out:
        os.makedirs(a.out, exist_ok=True)
    with mp.Pool(a.jobs) as pool:
        for r in pool.imap_unordered(one, jobs):
            line = json.dumps(r, default=str)
            print(line, flush=True)
            if a.out:
                with open(os.path.join(a.out, "runs.jsonl"), "a") as f:
                    f.write(line + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
