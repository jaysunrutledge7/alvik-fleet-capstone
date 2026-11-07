#!/usr/bin/env python3
"""e10 Phase C rehearsals (7 Nov): each case before (e09's defaults, through the switches: no start discovery, the typed
board as the workspace, the blend, the camera watch as e09 built it) and after (e10: the proof drive at the start, the
proven workspace), many seeds, in the fast simulator (freeroam_fastsim.Sim: the same freeroam_core the Pi runs, camera
sightings with no names through the one tracking layer, the robots' raw sensors, the edge physics with the look-ahead
bottom zone).

    python3 tools/fleet/rehearse_phase_c.py all --out docs/results/e10-rehearsals-2026-11-07 [--jobs 2]
    python3 tools/fleet/rehearse_phase_c.py a b          # some cases

Cases (docs/build-reports/e10-nov07-phase-c-discovery.md):
  a  a fresh start on a board nobody described right, home (3 robots, 122 x 122) and school (2 robots, 396 x 168, roof
     tags read to x = 100): the board's size typed 10 % too big, a turned roof tag (school), the camera's name wrong
     (its picture mirrored), no camera, the camera's clock 0.7 s ahead, a dark patch, and all of them but the missing
     camera at once. Each: found / narrowed, no body over an edge
  b  the proven workspace: the camera cannot see x > 85 at home. Before (the typed board), after (only what is
     proven), after + the probe-ahead setting: where the robots go
  c  school free roam, 30 min: contacts, stickers (Jay-Sun's 40 points) reached, tracked share beyond the tags
  d  the filter in charge: Phase A's cases a-d (b = the hung robot), 12 seeds each, blend vs filter
  e  the camera watch's false alarms: e09's cases a and b with the watch on, e09's method vs e10's, 12 seeds each; and
     whether it still catches a mirrored camera mid-run (no start discovery)
  f  30 min home free roam, end to end, every e10 default on (a dead robot, a Pi freeze)
"""
import argparse
import copy
import csv
import json
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

FW = "2026-11-05d"
HOME = (0.0, 0.0, 122.24, 121.92)
SCHOOL = (0.0, 0.0, 396.0, 168.0)
BEFORE = dict(discover=False, workspace=False)
AFTER = dict(discover=True, workspace=True)


def school_points():
    pts = []
    with open(os.path.join(HERE, "sites", "school-conference-points.csv")) as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x_cm"]), float(row["y_cm"])))
    return pts


def faults(site):
    dark = [(70.0, 70.0, 110.0, 110.0)] if site == "home" else [(150.0, 100.0, 220.0, 160.0)]
    out = [("size", dict(cam_fault=dict(scale=1.1)), {}),
           ("mirror", dict(cam_fault=dict(mirror=True)), {}),
           ("none", {}, dict(fault="none")),
           ("clock", dict(cam_clock=0.7), {}),
           ("dark", {}, dict(dark=dark))]
    if site == "school":
        out.append(("tag", dict(tag_yaw={"Alpha": 90.0, "Bravo": 180.0}), {}))
        out.append(("all", dict(cam_fault=dict(scale=1.1, mirror=True), cam_clock=0.7,
                                tag_yaw={"Alpha": 90.0, "Bravo": 180.0}), dict(dark=dark)))
    else:
        out.append(("all", dict(cam_fault=dict(scale=1.1, mirror=True), cam_clock=0.7), dict(dark=dark)))
    return out


def base(site):
    b = dict(dead_robot=False, pi_freeze=False, sensors=True, fw=FW, edge_physics=True, bottom_zone=True,
             dropouts=False, camera="detections", layer_kw={}, ctrl_kw=dict(pose_source="blend"))
    if site == "school":
        b.update(names=("Alpha", "Bravo"), board=SCHOOL, scene=dict(tracking="tags", tag_x_max=100.0),
                 starts={"Alpha": (60.0, 50.0, 0.0), "Bravo": (70.0, 120.0, 180.0)})
    else:
        b.update(scene=dict(tracking="lights"))
    return b


def case_runs(case, hours=None, seeds=None):
    out = []
    if case == "a":
        for site in ("home", "school"):
            for tag, kw, sc in faults(site):
                for sd in range(1, (seeds or 4) + 1):
                    for when, w in (("before", BEFORE), ("after", AFTER)):
                        k = base(site)
                        k.update(w)
                        k.update({a: v for a, v in kw.items()})
                        k["scene"] = dict(k["scene"], **sc)
                        out.append(("a-%s-%s-%s" % (site, tag, when), sd, dict(k, hours=hours or 0.08)))
    elif case == "b":
        for sd in range(1, (seeds or 4) + 1):
            for when, w, probe in (("before", BEFORE, False), ("after", AFTER, False), ("after-probe", AFTER, True)):
                k = base("home")
                k.update(w)
                k["cover"] = [(-30.0, -30.0, 85.0, 160.0)]
                k["starts"] = {"Left": (40.0, 90.0, 0.0), "Center": (62.0, 40.0, 90.0), "Right": (25.0, 30.0, 45.0)}
                k["ctrl_kw"] = dict(k["ctrl_kw"], probe_ahead=probe)
                out.append(("b-%s" % when, sd, dict(k, hours=hours or 0.15)))
    elif case == "c":
        for sd in range(1, (seeds or 2) + 1):
            for when, w, old in (("before", BEFORE, True), ("after", AFTER, False)):
                k = base("school")
                k.update(w)
                k["layer_kw"] = dict(old=old)
                out.append(("c-%s" % when, sd, dict(k, hours=hours or 0.5, points=school_points())))
    elif case == "d":
        import rehearse_phase_a as RA
        for c in ("a", "b", "c", "d"):
            tmpl = [kw for name, sd, kw in RA.case_runs(c) if name.endswith("after")][0]
            for sd in range(1, (seeds or 12) + 1):
                for src in ("blend", "ekf"):
                    k2 = dict(tmpl)
                    k2["sensors"] = True
                    k2["ctrl_kw"] = dict(k2.get("ctrl_kw") or {}, pose_source=src)
                    out.append(("d-%s-%s" % (c, src), sd, k2))
    elif case in ("e", "e2"):
        # e2: as e, with the filter in charge (as e09's runs that alarmed)
        import rehearse_phase_b as RB
        for c in ("a", "b"):
            tmpl = [kw for name, sd, kw in RB.case_runs(c) if name.endswith("after")][0]
            for sd in range(1, (seeds or 12) + 1):
                for meth in ("e09", "e10"):
                    k2 = copy.deepcopy(tmpl)
                    k2["layer_kw"] = dict(tmpl.get("layer_kw") or {}, camera_watch=True)
                    k2["_watch_method"] = meth
                    k2["ctrl_kw"] = dict(pose_source="blend" if case == "e" else "ekf", edge_rule="derived")
                    out.append(("%s-%s-%s" % (case, c, meth), sd, k2))
        for sd in range(1, 7 if case == "e" else 1):
            for meth in ("e09", "e10"):
                k = base("home")
                k.update(BEFORE)
                k["scene"] = dict(tracking="lights", fault="mirror")
                k["layer_kw"] = dict(camera_watch=True)
                k["_watch_method"] = meth
                out.append(("e-mirror-%s" % meth, sd, dict(k, hours=0.08)))
    elif case == "g":
        # (e10, the 7 Nov run 2 incident) a long straight 25 deg off towards an edge with a piece queued behind it
        # (freeroam_fastsim hdg_fault), and a robot that starts at an edge; before: the three switches off
        sw_off = dict(queue_watch=False, mark_unc=False, start_edge=False)
        for sd in range(1, (seeds or 8) + 1):
            for when, sw in (("before", sw_off), ("after", {})):
                k = base("home")
                k.update(AFTER)
                k["hdg_fault"] = dict(deg=25.0, near_cm=14.0, every_s=30.0)
                k["ctrl_kw"] = dict(k["ctrl_kw"], **sw)
                out.append(("g-heading-%s" % when, sd, dict(k, hours=hours or 0.15)))
                k = base("home")
                k.update(AFTER)
                k["starts"] = {"Right": (60.0, 3.0, -90.0)}
                k["ctrl_kw"] = dict(k["ctrl_kw"], **sw)
                out.append(("g-start-%s" % when, sd, dict(k, hours=0.03)))
    elif case == "f":
        for sd in (1, 2):
            k = base("home")
            k.update(AFTER)
            k.update(dead_robot=True, pi_freeze=True)
            out.append(("f-after", sd, dict(k, hours=hours or 0.5)))
    return out


def one(job):
    name, sd, kw = job
    kw = dict(kw)
    if name.startswith("d-"):
        import rehearse_phase_a as RA
        r = RA.one((name, sd, kw))
        r.pop("kept", None)
        return slim(r)
    meth = kw.pop("_watch_method", None)
    if meth:
        import tracking_layer as TL
        TL.CAM_WATCH_METHOD = meth
    import freeroam_fastsim as S
    pts = kw.pop("points", None)
    t = time.time()
    try:
        sim = S.Sim(sd, kw.pop("hours"), **kw)
        if pts:
            sim.points = pts
        r = sim.run()
    except Exception as e:      # noqa: BLE001
        import traceback
        return dict(case=name, seed=sd, error=repr(e), tb=traceback.format_exc()[-2500:])
    r["case"] = name
    r["wall_s"] = round(time.time() - t, 1)
    if sim.src is not None:
        r["layer_counts"] = dict(sim.src.layer.counts)
    return slim(r)


def slim(r):
    for k in ("gap_hist", "kept"):
        r.pop(k, None)
    pb = r.get("phase_b") or {}
    if isinstance(pb, dict):
        pb.pop("filters", None)
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="+")
    ap.add_argument("--out", default="logs/e10-rehearsals")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--hours", type=float, default=None)
    ap.add_argument("--seeds", type=int, default=None)
    a = ap.parse_args(argv)
    cases = list("abcdef") if a.cases == ["all"] else a.cases
    os.makedirs(a.out, exist_ok=True)
    jobs = [j for c in cases for j in case_runs(c, a.hours, a.seeds)]
    print("%d runs" % len(jobs), flush=True)
    path = os.path.join(a.out, "runs.jsonl")
    with mp.Pool(a.jobs, maxtasksperchild=4) as pool, open(path, "a") as f:
        for r in pool.imap_unordered(one, jobs):
            f.write(json.dumps(r, default=str) + "\n")
            f.flush()
            print(r.get("case"), r.get("seed"), "error" if "error" in r else "ok", r.get("wall_s"), flush=True)


if __name__ == "__main__":
    main()
