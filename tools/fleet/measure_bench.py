#!/usr/bin/env python3
"""Automatic mapping against typed values (build e07, 2026-11-06): how well the cameras alone measure the table, side by
side with the typed tape, through the REAL picture chain (the simulator's tag cameras draw the table, the reference tags
and the robots' roof tags with blur, noise and JPEG; tagcore reads them; site_fit fits them).

Ways compared (each on the same pictures):
  typed          the table's size typed and trusted to 3 mm (site measure_trust "typed"; e06's default), detector corners
  cameras-e06    the cameras win (e06's switch): the typed size only a 5 cm seed, the detector's own corners, k = 1
  cameras-edges  the cameras win, corners from the tags' edges (tagcore.refine_edges), the reference tiles the ruler and
                 the roof tags' read size fitted (e07); the robots standing still only
  cameras-drive  as cameras-edges, plus the robots' tags at many spots while they drive (a run's first minutes; Mission
                 Control's refit after a run uses the same reads)
  nothing-typed  as cameras-drive with nothing typed at all (a new area)

Setups: school (ONE camera on a tripod at the TV end looking down the table's length, as at school since 5 Nov) and
two-down (two cameras straight down over the halves: the setup the e06 simulator used).

Measured per run (simulator truth): the worst and median floor error over the table (a grid at the table top and at the
roof tags' height, where the camera sees it: where a robot is put by the camera model) and the table's size error.

    python3 tools/fleet/measure_bench.py --seeds 1,2,3 --out docs/results/e07-measure/
"""
import argparse
import copy
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import site_fit as SF    # noqa: E402
import tagcore as TC     # noqa: E402

TABLE = (396.24, 167.64)
TAG_H = 6.19
STILL_SPOTS = 6          # robots standing where they were put (the binding wiggle's spots): reads pooled per tag
DRIVE_SPOTS = 40         # the drive: this many spots over the area the camera reads. Chosen (a minute of two robots at
                         # 10 cm/s gives ~1,000 reads; MAX_GROUPS 100 are used). DOE note in the e07 report


def cameras(setup, seed):
    import tag_cam as CAMS
    L, W = TABLE
    rng = np.random.default_rng(seed)
    refs = CAMS.ref_config(TABLE, seed=seed)
    rl = CAMS.ref_list({"ref_tags": refs})
    if setup == "school":
        # the C920e on its tripod at the TV end (corner 1 end), looking down the table's length (Jay-Sun 3 Nov)
        # (all four corners in its picture; the far end's robot tags at 1.8-2.3 px a cell, as the real reads thinned out)
        cams = [CAMS.TagCamera((-90 + rng.normal(0, 5), W / 2 + rng.normal(0, 8), 230 + rng.normal(0, 8)),
                               (150 + rng.normal(0, 10), W / 2 + rng.normal(0, 5), 0.0), size=(1280, 960),
                               hfov_deg=70.0 + rng.normal(0, 2), roll_deg=float(rng.normal(0, 2)), blur_px=0.9,
                               noise=3.5, k1=float(rng.normal(0, 0.03)), seed=31 + seed, name="overhead",
                               table=(0, 0, L, W), refs=rl)]
    else:
        cfg = CAMS.default_config(table=TABLE, seed=seed)
        cfg["ref_tags"] = refs
        cams = CAMS.cams_from_config(cfg)
    return cams, refs


def spots(cams, n, rng, margin=12.0):
    """n robot spots the cameras read (a robot is only driven where it is seen)."""
    L, W = TABLE
    out = []
    tries = 0
    while len(out) < n and tries < 20000:
        tries += 1
        x, y = rng.uniform(margin, L - margin), rng.uniform(margin, W - margin)
        for c in cams:
            uv = c.project([[x, y, TAG_H]])[0]
            if np.all(np.isfinite(uv)) and 30 < uv[0] < c.W - 30 and 30 < uv[1] < c.H - 30 and \
                    c.px_per_cm(x, y, TAG_H) * 8.0 / 7.0 >= 2.0:
                out.append((x, y, rng.uniform(-math.pi, math.pi)))
                break
    return out


def read_scene(cams, robots, dets):
    """{way: {cam: {id: corners}}} for one picture per camera with these robots (dicts as tag_cam draws them)."""
    out = {k: {} for k in dets}
    for c in cams:
        img = c.render(robots)
        for k, det in dets.items():
            got = {}
            for d in det.detect_full(img):
                got[d.id] = d.corners.tolist()
            out[k][c.name] = got
    return out


def robot(x, y, th, i):
    return dict(x=x, y=y, th=th, z=TAG_H, family="tag25h9", id=i, tag_cm=8.0, roof_cm=9.5)


def gather(setup, seed, drive=DRIVE_SPOTS):
    """Every read the four ways need, from one simulated room: refs (6 pictures, median), still robots, a drive."""
    import ref_tags as RT
    cams, refs = cameras(setup, seed)
    rng = np.random.default_rng(seed + 7)
    ref_ids = [int(i) for i in RT.ids()]
    dets = {"det": TC.TagDetector("tag25h9", ids=[0, 1] + ref_ids, refine=False),
            "edges": TC.TagDetector("tag25h9", ids=[0, 1] + ref_ids, refine=True)}
    still = spots(cams, 2 * 1, rng)
    reads = {"refs": {k: {} for k in dets}, "still": {k: [] for k in dets}, "drive": {k: [] for k in dets}}
    # the empty table plus the two robots where they stand: the reference tags read in 3 pictures
    for rep in range(3):
        sc = read_scene(cams, [robot(*still[0], 0), robot(*still[1], 1)], dets)
        for k in dets:
            for c, got in sc[k].items():
                for i, cs in got.items():
                    if i in ref_ids:
                        reads["refs"][k].setdefault(c, {}).setdefault(str(i), []).append(cs)
                    else:
                        reads["still"][k].append((c, i, cs))
    for (x, y, th) in spots(cams, drive, rng):
        i = int(rng.integers(0, 2))
        sc = read_scene(cams, [robot(x, y, th, i)], dets)
        for k in dets:
            g = {"id": i, "reads": {c: got[i] for c, got in sc[k].items() if i in got}, "truth": [x, y, th]}
            if g["reads"]:
                reads["drive"][k].append(g)
    truth = {c.name: c.true_model() for c in cams}
    sizes = {c.name: [c.W, c.H] for c in cams}
    return reads, truth, sizes, refs


def obs_for(way, reads, sizes, refs):
    import ref_tags as RT
    k = "det" if way in ("typed", "cameras-e06") else "edges"
    drive = way in ("cameras-drive", "nothing-typed")
    groups = []
    by = {}
    for c, i, cs in reads["still"][k]:
        by.setdefault(i, {}).setdefault(c, []).append(np.asarray(cs, float))
    for i, cams in by.items():
        g = {"id": i, "reads": {c: np.median(np.array(v), axis=0).tolist() for c, v in cams.items()}}
        groups.append(g)
    if drive:
        groups += [dict(g) for g in reads["drive"][k]][:SF.MAX_GROUPS]
    rr = {c: {i: np.median(np.array(v), axis=0).tolist() for i, v in d.items()} for c, d in reads["refs"][k].items()}
    rc = RT.default_config()
    obs = {"cameras": {c: {"size": s} for c, s in sizes.items()}, "corners": {c: {} for c in sizes}, "stickers": {},
           "tag_cm": 8.0, "tag_height_cm": TAG_H, "tag_groups": groups, "distances": [], "roof_fit": False,
           "ref_tags": {"layout": RT.layout(rc), "tag_cm": RT.TAG_CM, "z_cm": RT.Z_CM, "offset_cm": RT.OFFSET_CM,
                        "reads": rr},
           "measure_trust": "typed" if way == "typed" else "cameras"}
    if way != "nothing-typed":
        obs["table"] = {"length_cm": TABLE[0], "width_cm": TABLE[1]}
    return obs


def score(res, truth):
    cmp_ = SF.compare(res, truth, {}, H=TAG_H, table=TABLE)
    cams = {c: v for c, v in cmp_.items() if isinstance(v, dict)}
    return {"floor_worst_cm": round(max(v["worst_cm"] for v in cams.values()), 2),
            "floor_median_cm": round(max(v["median_cm"] for v in cams.values()), 2),
            "size_cm": res["size_cm"], "size_err_cm": [round(res["size_cm"][0] - TABLE[0], 2),
                                                       round(res["size_cm"][1] - TABLE[1], 2)],
            "tag_scale": res.get("tag_scale"), "ref_tag_scale": res.get("ref_tag_scale")}


WAYS = ("typed", "cameras-e06", "cameras-edges", "cameras-drive", "nothing-typed")


def run(setups, seeds, out=None, say=print):
    rows = []
    for setup in setups:
        for seed in seeds:
            t0 = time.time()
            reads, truth, sizes, refs = gather(setup, seed)
            for way in WAYS:
                obs = obs_for(way, reads, sizes, refs)
                try:
                    res = SF.full(obs, untyped_check=False)
                    sc = score(res, truth)
                except Exception as e:   # noqa: BLE001  (a failed fit is a row too)
                    sc = {"error": str(e)}
                row = dict(setup=setup, seed=seed, way=way, groups=len(obs["tag_groups"]), **sc)
                rows.append(row)
                say(json.dumps(row))
            say("(%s seed %d: %.0f s)" % (setup, seed, time.time() - t0))
    if out:
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "measure_bench.json"), "w") as f:
            json.dump(rows, f, indent=1)
    return rows


def summary(rows):
    out = {}
    for r in rows:
        if "error" in r:
            continue
        k = (r["setup"], r["way"])
        d = out.setdefault(k, {"worst": [], "median": [], "size": []})
        d["worst"].append(r["floor_worst_cm"])
        d["median"].append(r["floor_median_cm"])
        d["size"].append(max(abs(v) for v in r["size_err_cm"]))
    return {"%s / %s" % k: {"floor_worst_cm_max": max(v["worst"]), "floor_worst_cm_mean": round(float(np.mean(v["worst"])), 2),
                            "floor_median_cm_mean": round(float(np.mean(v["median"])), 2),
                            "size_err_cm_max": max(v["size"]), "runs": len(v["worst"])} for k, v in out.items()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--setups", default="school,two-down")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    rows = run(a.setups.split(","), [int(s) for s in a.seeds.split(",")], a.out)
    sm = summary(rows)
    print(json.dumps(sm, indent=1))
    if a.out:
        with open(os.path.join(a.out, "measure_summary.json"), "w") as f:
            json.dump(sm, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
