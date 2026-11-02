#!/usr/bin/env python3
"""Which tag, how big, and how many pixels (conference-table build, 2026-11-02).

Two parts, both run here on the simulator's camera (tools/sim/tag_cam.py) with the real detector (tagcore.py):

  budget   for each camera layout over the 396 x 168 cm table (one camera over all of it; two cameras each over a
           half; an angled camera from one end) and each resolution (640 x 480, 1280 x 720, 1920 x 1080): the image
           scale at the farthest point of the table the camera must cover, and so a tag's pixels and pixels per cell.
  reads    synthetic pictures of one tag at a sweep of image scales, with lens blur, sensor noise, motion blur, JPEG
           and a tilted view: the share read, the position error and the heading error, per family and search way
           (a window round the robot, enlarged 1x or 2x; the whole picture). Each case TRIALS random spots and headings.

    python3 tools/fleet/tag_sizing.py --out docs/results/tag-sizing            # both parts: CSVs + charts
    python3 tools/fleet/tag_sizing.py --quick                                  # a few cases (tests)
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import tagcore as TC     # noqa: E402
import tag_geom as TG    # noqa: E402
import tag_cam as CAMS   # noqa: E402

TABLE_CM = (396.0, 168.0)
ROOF_CM = 9.5            # the roof's footprint = the Alvik's (Nick: "the same footprint")
TAG_CM = 8.0             # the black square: the roof less a 0.75 cm white margin each side (QUIET_CELLS). Chosen as the
                         # largest that keeps a white margin of >= 0.5 cell for every family. No DOE: as large as fits
TAG_HEIGHT_CM = 6.0      # PLACEHOLDER until Nick measures the roof (body 4 cm + roof plate ~2 cm). Only the
                         # parallax uses it; the sizing barely depends on it
TRIALS = 40              # random spots and headings per case. No DOE: enough to tell 100 % from 90 % (+-5 %)
PX_PER_CM = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0]   # the sweep
READ_GOOD = 0.98         # a case "reads" when at least this share of pictures gives the right tag. No DOE: a line


# ------------------------------------------------------------------------------------------------- the budget
def layouts(res):
    """Camera layouts over the table: (name, [TagCamera], what it must cover as an (x0, y0, x1, y1) per camera)."""
    W_, H_ = res
    L, Wt = TABLE_CM
    hfov = 70.0
    out = []
    # one camera straight down over the middle, as high as needed to see the whole table (+5 % margin)
    need_w = max(L * 1.05, Wt * 1.05 * W_ / H_)
    z = (need_w / 2.0) / math.tan(math.radians(hfov / 2)) + TAG_HEIGHT_CM
    out.append(("one camera straight down (lens %.0f cm above the table)" % z,
                [CAMS.TagCamera((L / 2, Wt / 2, z), (L / 2 + 0.01, Wt / 2, 0), size=res, hfov_deg=hfov)],
                [(0, 0, L, Wt)]))
    # two cameras straight down, each over a half + 20 cm (overlap 40 cm in the middle)
    half = L / 2 + 20.0
    need_w = max(half * 1.05, Wt * 1.05 * W_ / H_)
    z2 = (need_w / 2.0) / math.tan(math.radians(hfov / 2)) + TAG_HEIGHT_CM
    cams = [CAMS.TagCamera((x, Wt / 2, z2), (x + 0.01, Wt / 2, 0), size=res, hfov_deg=hfov) for x in (half / 2, L - half / 2)]
    out.append(("two cameras straight down, each over half + 20 cm (lenses %.0f cm up)" % z2, cams,
                [(0, 0, half, Wt), (L - half, 0, L, Wt)]))
    # one camera at one end, 200 cm up, 60 cm back, looking along the table (like home's side camera)
    c = CAMS.TagCamera((-60.0, Wt / 2, 200.0), (L * 0.45, Wt / 2, 0.0), size=res, hfov_deg=hfov)
    out.append(("one camera at one end, angled (200 cm up, 60 cm back)", [c], [(0, 0, L, Wt)]))
    # two angled cameras, one at each end, each covering its half + 20 cm
    cs = [CAMS.TagCamera((-60.0, Wt / 2, 200.0), (L * 0.27, Wt / 2, 0.0), size=res, hfov_deg=hfov),
          CAMS.TagCamera((L + 60.0, Wt / 2, 200.0), (L * 0.73, Wt / 2, 0.0), size=res, hfov_deg=hfov)]
    out.append(("two cameras, one at each end, angled (200 cm up, 60 cm back)", cs, [(0, 0, half, Wt), (L - half, 0, L, Wt)]))
    return out


def budget():
    rows = []
    for res in ((640, 480), (1280, 720), (1280, 960), (1920, 1080)):
        for name, cams, areas in layouts(res):
            worst_px, worst_cell, sees_all = 1e9, None, True
            for cam, (x0, y0, x1, y1) in zip(cams, areas):
                model = cam.true_model()
                for x in np.linspace(x0 + 5, x1 - 5, 9):
                    for y in np.linspace(y0 + 5, y1 - 5, 5):
                        px = TG.tag_px_at(model, x, y, TAG_HEIGHT_CM, TAG_CM)
                        u, v = model.project((x, y, TAG_HEIGHT_CM))
                        if not model.in_image(u, v, margin=px):
                            sees_all = False
                        worst_px = min(worst_px, px)
            row = {"resolution": "%dx%d" % res, "layout": name, "lens_height_cm": round(float(cams[0].C[2]), 0),
                   "worst_tag_px": round(worst_px, 1),
                   "worst_px_per_cm": round(worst_px / TAG_CM, 2), "sees_its_area": sees_all}
            for fam, (_, n, _) in TC.FAMILIES.items():
                row["px_per_cell_" + fam] = round(worst_px / n, 2)
            rows.append(row)
    return rows


# ------------------------------------------------------------------------------------------------- the reads
def read_case(family, ppcm, blur=0.8, noise=3.0, speed=0.0, tilt=0.0, trials=TRIALS, seed=1, size=(1280, 720),
              ways=("window2", "window1", "full"), backend=None):
    """One case: a camera whose scale at the tag plane is ppcm px/cm, tilted `tilt` deg from straight down. Returns
    {way: {read, false, err_cm (median, p95), err_deg (median, p95), ms}}."""
    rng = np.random.default_rng(seed)
    Z = 190.0
    hfov = 2 * math.degrees(math.atan((size[0] / 2.0) / (ppcm * (Z - TAG_HEIGHT_CM))))
    if tilt:
        # the camera moved back along -x and turned to look at the same spot (the tag's distance kept, so its scale
        # along the view is kept and across it squeezed by cos(tilt))
        d = Z - TAG_HEIGHT_CM
        pos = (100.0 - d * math.sin(math.radians(tilt)), 80.0, TAG_HEIGHT_CM + d * math.cos(math.radians(tilt)))
    else:
        pos = (100.0, 80.0, Z)
    cam = CAMS.TagCamera(pos, (100.01, 80.0, TAG_HEIGHT_CM), size=size, hfov_deg=hfov, blur_px=blur, noise=noise,
                         jpeg=85, seed=seed, table=(0, 0, 300, 200), table_grey=120)
    model = cam.true_model()
    dets = {w: TC.TagDetector(family, ids=[0, 1, 2, 3, 4, 5, 6, 7], backend=backend) for w in ways}
    res = {w: dict(read=0, false=0, err=[], derr=[]) for w in ways}
    for k in range(trials):
        x, y = 100.0 + rng.uniform(-15, 15), 80.0 + rng.uniform(-15, 15)
        th = rng.uniform(-math.pi, math.pi)
        tid = int(rng.integers(0, 6))
        vx, vy = speed * math.cos(th), speed * math.sin(th)
        r = dict(x=x, y=y, th=th, z=TAG_HEIGHT_CM, family=family, id=tid, tag_cm=TAG_CM, roof_cm=ROOF_CM, vx=vx, vy=vy)
        img = cam.render([r])
        u, v = model.project((x, y, TAG_HEIGHT_CM))
        for w in ways:
            det = dets[w]
            if w == "full":
                found = det.detect(img)
            else:
                half = TC.WINDOW_TAGS * ppcm * TAG_CM / 2.0 + 6
                ou, ov = rng.uniform(-6, 6, 2)
                found = det.detect_window(img, u + ou, v + ov, half, upscale=2.0 if w == "window2" else 1.0)
            good = [d for d in found if d.id == tid]
            res[w]["false"] += len([d for d in found if d.id != tid])
            if good:
                res[w]["read"] += 1
                p = TG.pose_from_detection(model, good[0], TAG_HEIGHT_CM, TAG_CM)
                res[w]["err"].append(math.hypot(p.x - x, p.y - y))
                res[w]["derr"].append(abs(TG.wrap180(p.heading - math.degrees(th))))
    out = {}
    for w in ways:
        e, de = res[w]["err"], res[w]["derr"]
        out[w] = dict(read=res[w]["read"] / trials, false=res[w]["false"],
                      err_cm_med=float(np.median(e)) if e else None, err_cm_p95=float(np.percentile(e, 95)) if e else None,
                      err_deg_med=float(np.median(de)) if de else None,
                      err_deg_p95=float(np.percentile(de, 95)) if de else None,
                      ms=dets[w].ms_per_call())
    return out


def reads(quick=False, backend=None):
    fams = ["tag16h5", "tag25h9", "tag36h11", "aruco4x4"]
    spoil = [dict(name="sharp", blur=0.6, noise=2.0, speed=0.0, tilt=0.0),
             dict(name="typical webcam", blur=0.9, noise=4.0, speed=10.0, tilt=0.0),
             dict(name="soft + tilted 35 deg", blur=1.3, noise=6.0, speed=10.0, tilt=35.0)]
    ppcs = PX_PER_CM if not quick else [2.0, 4.0]
    if quick:
        fams, spoil = ["tag25h9"], spoil[1:2]
    rows = []
    for fam in fams:
        if TC.backend_for(fam, backend) is None:
            continue
        for sp in spoil:
            for ppcm in ppcs:
                r = read_case(fam, ppcm, blur=sp["blur"], noise=sp["noise"], speed=sp["speed"], tilt=sp["tilt"],
                              trials=8 if quick else TRIALS, backend=backend)
                for w, v in r.items():
                    rows.append(dict(family=fam, spoilers=sp["name"], px_per_cm=ppcm,
                                     px_per_cell=round(ppcm * TAG_CM / TC.FAMILIES[fam][1], 2), way=w,
                                     **{k: (round(val, 3) if isinstance(val, float) else val) for k, val in v.items()}))
                print("%-9s %-22s %.1f px/cm: %s" % (fam, sp["name"], ppcm, "; ".join(
                    "%s %.0f%%" % (w, 100 * v["read"]) for w, v in r.items())), flush=True)
    return rows


def write_csv(rows, path):
    if not rows:
        return
    keys = list(rows[0].keys())
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def needed_px_per_cm(rows, family, way="window2", spoilers="typical webcam"):
    """The smallest scale (px/cm) from which every larger scale in the sweep reads >= READ_GOOD."""
    pts = sorted((r["px_per_cm"], r["read"]) for r in rows if r["family"] == family and r["way"] == way
                 and r["spoilers"] == spoilers)
    need = None
    for p, rd in reversed(pts):
        if rd >= READ_GOOD:
            need = p
        else:
            break
    return need


def chart(rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fams = [f for f in TC.FAMILIES if any(r["family"] == f for r in rows)]
    sps = []
    for r in rows:
        if r["spoilers"] not in sps:
            sps.append(r["spoilers"])
    fig, axs = plt.subplots(1, len(sps), figsize=(4.2 * len(sps), 3.6), sharey=True)
    axs = np.atleast_1d(axs)
    cols = {"tag16h5": "#2952CC", "tag25h9": "#12805C", "tag36h11": "#C2410C", "aruco4x4": "#7A3EC8"}
    for ax, sp in zip(axs, sps):
        for fam in fams:
            for way, ls in (("window2", "-"), ("full", ":")):
                pts = sorted((r["px_per_cm"], r["read"]) for r in rows if r["family"] == fam and r["spoilers"] == sp
                             and r["way"] == way)
                if pts:
                    ax.plot([p for p, _ in pts], [100 * q for _, q in pts], ls, color=cols.get(fam, "k"), marker="o",
                            ms=3, label="%s, %s" % (fam, "window 2x" if way == "window2" else "whole picture"))
        for x, lab in ((1.5, "640x480\none cam"), (3.0, "640x480\ntwo cams"), (3.8, "1280x720\ntwo cams")):
            ax.axvline(x, color="#D0D5DD", lw=1)
        ax.set_title(sp, fontsize=10)
        ax.set_xlabel("image scale at the tag (px per cm)")
        ax.grid(alpha=0.3)
    axs[0].set_ylabel("tags read (%)")
    axs[-1].legend(fontsize=6, loc="lower right")
    fig.suptitle("Synthetic pictures of one 8 cm tag on a 9.5 cm roof, %d random spots each (tagcore.py, simulator "
                 "camera)" % TRIALS, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(REPO, "docs", "results", "tag-sizing"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--backend", choices=["opencv", "pupil"], default=None)
    ap.add_argument("--no-budget", action="store_true")
    ap.add_argument("--no-reads", action="store_true")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    import cv2
    meta = {"opencv": cv2.__version__, "backend": a.backend or "default", "tag_cm": TAG_CM, "roof_cm": ROOF_CM,
            "tag_height_cm": TAG_HEIGHT_CM, "trials": TRIALS, "date": time.strftime("%Y-%m-%d %H:%M")}
    if not a.no_budget:
        b = budget()
        write_csv(b, os.path.join(a.out, "budget.csv"))
        for r in b:
            print("%-9s %-75s tag %5.1f px (%.2f px/cm), tag25h9 %.2f px/cell%s" % (
                r["resolution"], r["layout"], r["worst_tag_px"], r["worst_px_per_cm"], r["px_per_cell_tag25h9"],
                "" if r["sees_its_area"] else "  (does NOT see all of its area)"))
    if not a.no_reads:
        rows = reads(quick=a.quick, backend=a.backend)
        suffix = "" if not a.backend else "-" + a.backend
        write_csv(rows, os.path.join(a.out, "reads%s.csv" % suffix))
        if not a.quick:
            chart(rows, os.path.join(a.out, "reads%s.png" % suffix))
            need = {f: {sp: needed_px_per_cm(rows, f, spoilers=sp) for sp in ("sharp", "typical webcam",
                                                                                "soft + tilted 35 deg")}
                    for f in TC.FAMILIES}
            meta["needed_px_per_cm_window2"] = need
            print(json.dumps(need, indent=1))
    with open(os.path.join(a.out, "meta%s.json" % ("" if not a.backend else "-" + a.backend)), "w") as f:
        json.dump(meta, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
