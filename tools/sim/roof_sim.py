#!/usr/bin/env python3
"""The roof-tag height / tilt / offset check against the simulator (2026-11-03, report e02-nov03-roof-tag-height-tilt).

Offline (no network, no live simulator): pictures drawn straight from tag_cam.TagCamera, read by the real detector.
One case = one table layout (two half cameras, or one camera over the whole table) and each robot's true roof:
  1. Measure: the robots standing still where the setup puts them (both cameras see the middle), 24 pictures a
     camera, the reference tags on the table; tag_setup's make_obs / fit_and_save (corner clicks off, typed size on)
     -> roof_tags.json (height, tilt, lean per robot)
  2. a spin on the spot (sync spin) for each robot -> tag_poses.csv / tag_obs.csv -> site_fit.refit -> offsets
  3. positions: each robot at a grid of spots x 8 headings; the turning point from the flat placeholder model (before)
     and from the measured roof (after, tag_bridge's roofs_for_bridge), against the truth; reads lost per spot/heading

    python3 tools/sim/roof_sim.py --case tilt30 --layout one --out /tmp/x
"""
import argparse
import csv
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np       # noqa: E402

import tag_cam as CAMS   # noqa: E402
import tag_geom as TG    # noqa: E402

PLACEHOLDER = 6.0
CASES = {
    # name: {robot: true roof}, placeholder height used by the setup
    "flat6": {"Alpha": {"height_cm": 6.0}, "Bravo": {"height_cm": 6.0}},
    "flat9": {"Alpha": {"height_cm": 9.0}, "Bravo": {"height_cm": 9.0}},
    "tilt5": {"Alpha": {"height_cm": 6.0, "tilt_deg": 5.0, "lean_deg": 90.0},
              "Bravo": {"height_cm": 6.0, "tilt_deg": 5.0, "lean_deg": 200.0}},
    "tilt15": {"Alpha": {"height_cm": 6.0, "tilt_deg": 15.0, "lean_deg": 90.0},
               "Bravo": {"height_cm": 6.0, "tilt_deg": 15.0, "lean_deg": 200.0}},
    "tilt30": {"Alpha": {"height_cm": 6.0, "tilt_deg": 30.0, "lean_deg": 90.0},
               "Bravo": {"height_cm": 6.0, "tilt_deg": 30.0, "lean_deg": 200.0}},
    "offset": {"Alpha": {"height_cm": 6.0, "offset_cm": [1.5, 0.0]},
               "Bravo": {"height_cm": 6.0, "offset_cm": [-1.0, 1.5]}},
}


def config(layout, seed=5):
    cfg = CAMS.default_config(size=(1280, 960), seed=seed)
    if layout == "one":
        L, W = cfg["table"]
        # one camera over the whole table (Nick, 3 Nov: "probably visible in one camera"): 260 cm up, 78 deg wide,
        # a little off-centre and turned, as a hand-mounted camera is
        cfg["cams"] = [dict(name="overhead", pos=[L / 2 + 6, W / 2 - 4, 260.0], look_at=[L / 2 - 4, W / 2 + 3, 0.0],
                            size=[1280, 960], hfov_deg=78.0, blur_px=0.9, noise=3.5, k1=0.02, roll_deg=1.5, port=8765)]
    return cfg


def robot_dict(cfg, name, x, y, th, truth):
    tags = cfg["tags"]
    d = dict(x=x, y=y, th=th, z=float(tags["height_cm"]), family=tags["family"], id=int(tags["ids"][name]),
             tag_cm=float(tags["tag_cm"]), roof_cm=float(tags["roof_cm"]))
    d.update(CAMS.roof_truth({"roof_truth": truth}, name))
    return d


def homes(cfg):
    L, W = cfg["table"]
    return {"Alpha": (L / 2 - 35.0, W / 2 + 10.0, math.radians(30.0)),
            "Bravo": (L / 2 + 35.0, W / 2 - 10.0, math.radians(200.0))}


def measure(cfg, cams, truth, out, n_pics=24, say=print, old=False):
    import tag_setup as TSU
    import ref_tags as RT
    hm = homes(cfg)
    rs = [robot_dict(cfg, n, *hm[n], truth) for n in hm]
    pics = {c.name: [(k * 0.08, c.render(rs, t=k * 0.08)) for k in range(n_pics)] for c in cams}
    tags = dict(cfg["tags"], height_placeholder=True)
    reads, ms = TSU.read_tags(pics, tags)
    rcfg = cfg["ref_tags"].get("config") or RT.default_config()
    refs = RT.read_refs(pics, rcfg)
    sizes = {c.name: [c.W, c.H] for c in cams}
    L, W = cfg["table"]
    obs = TSU.make_obs(sizes, {c: {} for c in sizes}, {}, tags, TSU.groups_from_reads(reads, still=True),
                       table={"length_cm": L, "width_cm": W}, refs=refs, ref_cfg=rcfg)
    t0 = time.time()
    res = TSU.fit_and_save(obs, os.path.join(out, "tagcal"))
    say("measure: %d reads, fit %.0f s" % (len(reads), time.time() - t0))
    if old:
        # the setup as it was before this build (every roof flat at the placeholder): the "before" cameras
        TSU.fit_and_save(dict(obs, roof_fit=False), os.path.join(out, "tagcal_old"))
    return res


def spin(cfg, cams, truth, out, roof_tags, say=print):
    """Each robot spins once on the spot at its home (the sync spin); tag_poses.csv / tag_obs.csv as tag_bridge writes
    them (flat model at the measured height, as the bridge would before the offset is known)."""
    import tagcore as TC
    from cameras import CameraModel
    cal = os.path.join(out, "tagcal")
    models = {c.name: CameraModel.load(os.path.join(cal, c.name + ".json")) for c in cams}
    det = {c.name: TC.TagDetector(cfg["tags"]["family"], ids=list(cfg["tags"]["ids"].values())) for c in cams}
    hm = homes(cfg)
    run = os.path.join(out, "run")
    os.makedirs(run, exist_ok=True)
    fo, fp = open(os.path.join(run, "tag_obs.csv"), "w", newline=""), open(os.path.join(run, "tag_poses.csv"), "w",
                                                                           newline="")
    wo, wp = csv.writer(fo), csv.writer(fp)
    wo.writerow(["t", "cam", "id", "u0", "v0", "u1", "v1", "u2", "v2", "u3", "v3"])
    wp.writerow(["t", "cam", "robot", "id", "x", "y", "heading", "sigma_cm", "sigma_deg", "size_ratio", "px_per_cm",
                 "u", "v", "used"])
    t = 100.0
    robots_by_id = {int(i): n for n, i in cfg["tags"]["ids"].items()}
    for k in range(48):                       # two half turns, 7.5 deg a picture
        rs = []
        for n, (x, y, th) in hm.items():
            rs.append(robot_dict(cfg, n, x, y, th + math.radians(7.5 * k), truth))
        t += 0.2
        for c in cams:
            g = c.render(rs, t=t)
            for d in det[c.name].detect_full(g, cam=c.name, t=t):
                wo.writerow(["%.4f" % t, c.name, d.id] + ["%.2f" % v for v in d.corners.ravel()])
                n = robots_by_id[d.id]
                h = (roof_tags["robots"].get(n) or {}).get("height_cm", PLACEHOLDER)
                p = TG.pose_from_detection(models[c.name], d, h, float(cfg["tags"]["tag_cm"]))
                if p is not None:
                    wp.writerow(["%.4f" % t, c.name, n, d.id, "%.2f" % p.x, "%.2f" % p.y, "%.2f" % p.heading,
                                 "%.3f" % p.sigma_cm, "%.2f" % p.sigma_deg, "%.3f" % p.size_ratio, "%.2f" % p.px_per_cm,
                                 0, 0, "yes"])
    fo.close()
    fp.close()
    import site_fit as SF
    t0 = time.time()
    rf = SF.refit(os.path.join(cal, "obs.json"), os.path.join(run, "tag_obs.csv"), out=os.path.join(out, "tagcal_refit"),
                  max_groups=40)
    say("refit: %.0f s" % (time.time() - t0))
    return rf


def positions(cfg, cams, truth, sets, out_csv, step=(5, 3), headings=8):
    """Every spot x heading: per robot the fused turning point from each of `sets` ({label: calibration folder};
    label "before": flat at the placeholder, centred, as before this build; others: roofs_for_bridge from that folder's
    roof_tags.json), against the truth; reads lost. Rows to out_csv; returns them."""
    import tagcore as TC
    import roof_cal as RC
    from cameras import CameraModel
    tags = dict(cfg["tags"])
    models = {k: {c.name: CameraModel.load(os.path.join(cal, c.name + ".json")) for c in cams} for k, cal in sets.items()}
    roofs = {k: ({} if k == "before" else RC.roofs_for_bridge(cal, tags)[0]) for k, cal in sets.items()}
    det = {c.name: TC.TagDetector(tags["family"], ids=list(tags["ids"].values())) for c in cams}
    L, W = cfg["table"]
    xs = np.linspace(25, L - 25, step[0])
    ys = np.linspace(20, W - 20, step[1])
    rows = []
    S = float(tags["tag_cm"])
    for x in xs:
        for y in ys:
            for k in range(headings):
                th = math.radians(360.0 * k / headings + 10.0)
                for n in ("Alpha", "Bravo"):
                    r = robot_dict(cfg, n, float(x), float(y), th, truth)
                    got, seen = {lab: [] for lab in sets}, []
                    for c in cams:
                        g = c.render([r], t=0.0)
                        for d in det[c.name].detect_full(g, cam=c.name, t=0.0):
                            if d.id != r["id"]:
                                continue
                            seen.append(c.name)
                            for lab in sets:
                                got[lab].append(TG.pose_from_detection(models[lab][c.name], d, PLACEHOLDER, S,
                                                                       roof=roofs[lab].get(d.id)))
                    vis = [c.name for c in cams if _in_view(c, r)]
                    row = dict(robot=n, x=round(float(x), 1), y=round(float(y), 1), heading=round(math.degrees(th), 1),
                               in_view=len(vis), read=len(seen))
                    for lab in sets:
                        f, _ = TG.fuse(got[lab])
                        row["err_" + lab] = None if f is None else round(math.hypot(f.x - x, f.y - y), 3)
                        row["hd_" + lab] = None if f is None else round(abs(TG.wrap180(f.heading - math.degrees(th))), 2)
                        row["size_" + lab] = None if f is None else round(f.size_ratio, 3)
                    rows.append(row)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return rows


def _in_view(cam, r, margin=8):
    P = TG.roof_corners_world(r["x"], r["y"], r["th"], r["z"], r["tag_cm"], r.get("tv"), r.get("tag_off") or (0, 0))
    uv = cam.project(P)
    return bool(np.all(np.isfinite(uv)) and np.all(uv[:, 0] > margin) and np.all(uv[:, 0] < cam.W - margin)
                and np.all(uv[:, 1] > margin) and np.all(uv[:, 1] < cam.H - margin))


def run_case(case, layout, out, seed=5, say=print, do_spin=True, do_pos=True):
    os.makedirs(out, exist_ok=True)
    truth = CASES[case]
    cfg = config(layout, seed)
    cfg["roof_truth"] = truth
    with open(os.path.join(out, "cfg.json"), "w") as f:
        json.dump(cfg, f, indent=1)
    cams = CAMS.cams_from_config(cfg)
    res = measure(cfg, cams, truth, out, say=say, old=do_pos)
    rt = res.get("roof_tags") or {"robots": {}}
    summ = {"case": case, "layout": layout, "truth": truth, "measure": rt, "words": res["words"]}
    sets = {"before": os.path.join(out, "tagcal_old"), "measure": os.path.join(out, "tagcal")}
    if do_spin:
        rf = spin(cfg, cams, truth, out, rt, say=say)
        summ["refit"] = rf.get("roof_tags")
        sets["refit"] = os.path.join(out, "tagcal_refit")
    if do_pos:
        rows = positions(cfg, cams, truth, sets, os.path.join(out, "positions.csv"))
        summ["positions"] = stats(rows, list(sets))
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summ, f, indent=1, default=str)
    return summ


def stats(rows, labels):
    out = {}
    for key in ["%s_%s" % (a, b) for b in labels for a in ("err", "hd")]:
        v = np.array([r[key] for r in rows if r[key] is not None])
        out[key] = dict(median=round(float(np.median(v)), 2), p95=round(float(np.percentile(v, 95)), 2),
                        max=round(float(v.max()), 2)) if len(v) else None
    iv = [r for r in rows if r["in_view"] > 0]
    out["in_view"] = len(iv)
    out["read"] = sum(1 for r in iv if r["read"] > 0)
    out["lost"] = [(r["robot"], r["x"], r["y"], r["heading"]) for r in iv if r["read"] == 0]
    out["camera_reads_lost"] = sum(r["in_view"] - r["read"] for r in iv)
    out["camera_reads_possible"] = sum(r["in_view"] for r in iv)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", default="flat6", choices=sorted(CASES))
    ap.add_argument("--layout", default="two", choices=("two", "one"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--no-spin", action="store_true")
    ap.add_argument("--no-positions", action="store_true")
    a = ap.parse_args(argv)
    s = run_case(a.case, a.layout, a.out, a.seed, do_spin=not a.no_spin, do_pos=not a.no_positions)
    for n, d in (s.get("refit") or s["measure"]).get("robots", {}).items():
        print(d["words"])
    print(json.dumps(s.get("positions"), default=str)[:800])
    return 0


if __name__ == "__main__":
    sys.exit(main())
