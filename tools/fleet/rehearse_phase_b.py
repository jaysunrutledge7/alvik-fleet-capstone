#!/usr/bin/env python3
"""e09 Phase B rehearsals (7 Nov): each case before (the pipeline before e09, through its switches: the old camera
sources, identity by prediction, the one-step blend) and after (the one tracking layer, the filter in the loop), many
seeds, in the fast simulator (freeroam_fastsim.Sim: the same freeroam_core the Pi runs; --camera detections: sightings
with no names, so names CAN swap).

    python3 tools/fleet/rehearse_phase_b.py all --out docs/results/e09-rehearsals-2026-11-07 [--jobs 2]
    python3 tools/fleet/rehearse_phase_b.py a d          # some cases

Cases (docs/build-reports/e09-nov07-phase-b-tracking.md):
  a  school table (396 x 168 cm), tags read only to x = 100 cm: % of pictures each robot is tracked (all, and beyond
     x = 100), position error, points of a 30 cm grid reached. Before: tags only, blend. After: tags + motion + lock
  b  home, the 22-23 Oct losses: a sunlit strip (x 30-45, measured: insight 15) where lights wash out and the motion
     detector fails, a dark corner, and Right standing (held, silent) in the strip for 3 min: % in view.
     Before: motion + lights. After: + lock-on. Also "motion alone" (the 22-23 Oct detector alone)
  c  a 40 s camera gap for one robot, every seed: the controller's position error at the gap's end and its worst in
     the gap. Before: the one-step blend. After: the filter
  d  the 27 Oct swap: Right standing (silent, held) in a glare patch where the camera sees it badly, Left driving past
     it closely: name swaps against the truth, wrong fixes. Before: identity by prediction. After: wiggle first
  e  Phase A's cases a-d (the 6 Nov repeat is a) with the filter live (sensors on): no loss against e08
  f  a 30 min home free roam end to end, after: no contact, closest body to an edge (+ a brick put down at 10 min and a
     parked robot pushed: the brick watch and the parked watch)
  g  clock skew 0.7 s, a wrong camera name (the other camera's calibration: sightings mirrored), the camera missing
     from 60 s: the run narrows rather than ends
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

SCHOOL = (0.0, 0.0, 396.0, 168.0)
FW = "2026-11-05d"
OLD = dict(pose_source="blend")
NEW = dict(pose_source="ekf")


def grid(board, step=30.0, inset=17.0):
    xs, ys = [], []
    x = board[0] + inset
    while x <= board[2] - inset:
        xs.append(x)
        x += step
    y = board[1] + inset
    while y <= board[3] - inset:
        ys.append(y)
        y += step
    return [(a, b) for a in xs for b in ys]


def case_runs(case, hours=None):
    out = []
    base = dict(dead_robot=False, pi_freeze=False, sensors=True, fw=FW)
    if case == "a":
        for sd in range(1, 9):
            for tag, old, kw in (("before", True, OLD), ("after", False, NEW)):
                out.append(("a-" + tag, sd, dict(base, hours=hours or 0.25, names=("Alpha", "Bravo"), board=SCHOOL,
                                                  camera="detections", dropouts=False,
                                                  scene=dict(tracking="tags", tag_x_max=100.0),
                                                  layer_kw=dict(old=old), ctrl_kw=dict(kw, edge_rule="derived"),
                                                  starts={"Alpha": (50.0, 50.0, 0.0), "Bravo": (60.0, 120.0, 0.0)},
                                                  points=grid(SCHOOL))))
    elif case == "b":
        strip = [(30.0, 0.0, 45.0, 121.92)]
        dark = [(90.0, 0.0, 122.24, 30.0)]
        for sd in range(1, 9):
            for tag, old, srcs, kw in (("motion-alone", True, ("motion",), OLD), ("before", True, None, OLD),
                                       ("after", False, None, NEW)):
                lk = dict(old=old)
                if srcs:
                    lk["sources"] = srcs
                out.append(("b-" + tag, sd, dict(base, hours=hours or 0.15, camera="detections", dropouts=False,
                                                  scene=dict(tracking="lights", glare=strip, busy=strip, dark=dark),
                                                  layer_kw=lk, ctrl_kw=dict(kw),
                                                  starts={"Right": (38.0, 60.0, 90.0), "Left": (90.0, 85.0, 40.0),
                                                          "Center": (75.0, 35.0, -60.0)},
                                                  silent=[("Right", 30.0, 180.0)])))
    elif case == "c":
        for sd in range(1, 13):
            for tag, kw in (("before", OLD), ("after", NEW)):
                out.append(("c-" + tag, sd, dict(base, hours=hours or 0.06, camera="names", dropouts=False,
                                                  ctrl_kw=dict(kw), cam_gaps=[("Center", 120.0, 40.0)])))
    elif case == "d":
        # the 27 Oct swap: a robot standing where the camera sees it badly (Right silent, held, in a glare patch: its
        # lights wash out, the motion detector misses a still robot), the other driving past it closely (30 % of moves
        # planned ignoring it). The camera's sightings carry no names: a sighting of Left can be taken as Right
        glare = [(40.0, 40.0, 82.0, 82.0)]
        for sd in range(1, 11):
            for tag, old, kw in (("before", True, OLD), ("after", False, NEW)):
                out.append(("d-" + tag, sd, dict(base, hours=hours or 0.15, names=("Left", "Right"), camera="detections",
                                                  dropouts=False, drill=0.3,
                                                  scene=dict(tracking="lights", glare=glare, busy=glare,
                                                             hide_still=("Right",)),
                                                  starts={"Right": (61.0, 61.0, 0.0), "Left": (95.0, 95.0, 180.0)},
                                                  silent=[("Right", 0.1, 3600.0)],
                                                  layer_kw=dict(old=old), ctrl_kw=dict(kw))))
    elif case == "e":
        import rehearse_phase_a as RA
        for c in ("a", "b", "c", "d"):
            for name, sd, kw in RA.case_runs(c):
                if not name.endswith("after"):
                    continue
                k2 = dict(kw)
                k2["sensors"] = True
                k2["ctrl_kw"] = dict(k2.get("ctrl_kw") or {}, pose_source="ekf")
                out.append(("e-" + name, sd, k2))
    elif case == "f":
        for sd in (1, 2):
            out.append(("f-after", sd, dict(base, hours=hours or 0.5, camera="detections", dropouts=False,
                                            dead_robot=True, pi_freeze=True, scene=dict(tracking="lights"),
                                            layer_kw={}, ctrl_kw=dict(NEW), bricks=[(600.0, 61.0, 61.0)],
                                            pushes=[("Left", 900.0, 4.0, 3.0)])))
    elif case == "g":
        for sd in range(1, 5):
            for tag, extra in (("clock", dict(cam_clock=0.7)), ("wrong-name", dict(fault="mirror")),
                               ("missing", dict(fault="none", fault_t=60.0))):
              for when, old, ck in (("before", True, OLD), ("after", False, NEW)):
                sc = dict(tracking="lights")
                kw = dict(base, hours=hours or 0.08, camera="detections", dropouts=False, layer_kw=dict(old=old),
                          ctrl_kw=dict(ck), edge_physics=True)
                if "cam_clock" in extra:
                    kw["cam_clock"] = extra["cam_clock"]
                else:
                    sc.update(extra)
                kw["scene"] = sc
                out.append(("g-%s-%s" % (tag, when), sd, kw))
    return out


def one(job):
    import freeroam_fastsim as S
    name, sd, kw = job
    if name.startswith("e-"):
        # Phase A's own run (its firmware, tilt limits and the rejoin / recovery times), with the filter live
        import rehearse_phase_a as RA
        r = RA.one((name, sd, dict(kw)))
        r.pop("kept", None)
        return r
    kw = dict(kw)
    pts = kw.pop("points", None)
    t = time.time()
    try:
        sim = S.Sim(sd, kw.pop("hours"), **kw)
        if pts:
            sim.points = pts
        r = sim.run()
    except Exception as e:      # noqa: BLE001
        import traceback
        return dict(case=name, seed=sd, error=repr(e), tb=traceback.format_exc()[-2000:])
    r["case"] = name
    r["wall_s"] = round(time.time() - t, 1)
    r["ended_early"] = sim.t < (kw.get("hours", 0) or 0) * 3600 - 1 if False else bool(sim.ctrl.all_out())
    r["sim_s"] = round(sim.t, 1)
    if kw.get("cam_gaps"):
        nm, t0, dur = kw["cam_gaps"][0]
        pe = [row for row in getattr(sim, "pe_t", [])]
        r["gap"] = gap_errors(sim, nm, t0, dur)
    for k in ("kept",):
        r.pop(k, None)
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="+")
    ap.add_argument("--out", default="logs/e09-rehearsals")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--hours", type=float, default=None)
    ap.add_argument("--seeds", type=int, default=0, help="at most this many seeds per case variant (0: all)")
    a = ap.parse_args(argv)
    cases = list("abcdefg") if a.cases == ["all"] else a.cases
    os.makedirs(a.out, exist_ok=True)
    jobs, seen = [], {}
    for c in cases:
        for j in case_runs(c, a.hours):
            seen[j[0]] = seen.get(j[0], 0) + 1
            if not a.seeds or seen[j[0]] <= a.seeds:
                jobs.append(j)
    print("%d runs" % len(jobs), flush=True)
    path = os.path.join(a.out, "runs.jsonl")
    with mp.Pool(a.jobs) as pool, open(path, "a") as f:
        for r in pool.imap_unordered(one, jobs):
            f.write(json.dumps(r, default=str) + "\n")
            f.flush()
            print(r.get("case"), r.get("seed"), "error" if "error" in r else "ok", r.get("wall_s"), flush=True)


def gap_errors(sim, name, t0, dur):
    """The controller's position error for `name` at the gap's end and its worst inside the gap (sampled 0.5 s)."""
    rows = getattr(sim, "pe_t", {}).get(name, [])
    inside = [e for (t, e) in rows if t0 <= t <= t0 + dur]
    end = [e for (t, e) in rows if t0 + dur - 0.6 <= t <= t0 + dur]
    return dict(end=end[-1] if end else None, worst=max(inside) if inside else None, n=len(inside))


if __name__ == "__main__":
    main()
