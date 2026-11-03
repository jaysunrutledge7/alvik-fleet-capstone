#!/usr/bin/env python3
"""Will a 3D-printed reference tag read like a paper one? (2026-11-03, Nick's tiles: tools/fleet/ref_tags.py,
hardware/dock/build_ref_tiles.py)

Two questions, each answered in the simulated school cameras (tools/sim/tag_cam.py: 1280 x 960, 175 cm up, lens blur,
noise, JPEG) at every reference tag's spot, read by the real detector:
  nozzle    a 0.4 mm nozzle lays 0.45 mm lines: a printed square's corners round by about half a line and its sides
            come out up to 0.1 mm off (over- or under-extrusion). Cell 17.1 mm (reference), 11.4 mm (roof).
  raised    the black squares stand 0.6 mm above the white: a camera off to one side sees their black side walls (a
            sliver up to 0.6 mm x the view's slope), and a ceiling light off to one side casts a 0.6 mm shadow on the
            white beside them. Drawn in 3D: tops at 1.8 mm, walls, the base at 1.2 mm, a shadow (light 45 deg up).
For each: read or not, and how far the four corners the detector finds sit from where a flat paper tag's would be
(pixels, and the tag middle's shift in cm on the table).

    python3 tools/fleet/ref_printcheck.py      # prints the table; --out DIR writes printcheck.csv and a picture
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import ref_tags as RT     # noqa: E402

LINE_MM = 0.45            # a 0.4 mm nozzle's line (slicers' default width)
XY_ERR_MM = 0.1           # chosen: a printed side's error after a first-layer squish and a little over-extrusion
                          # (typical PLA, 0.4 mm nozzle). No DOE here: the real tiles measured with a ruler say it
LIGHT_ELEV_DEG = 45.0     # chosen: a ceiling light off to one side (the worst usual case is lower, 30 deg: also run)
SHADOW_GREY = 0.55        # chosen: a shadowed white reads at 55 % of lit white (a soft room light; 0 = black)


def cell_polys(tag_id, tag_cm, cx, cy, th_deg, grow_cm=0.0, round_cm=0.0, n_round=4):
    """Each black cell as a polygon (x, y) on the table, grown by grow_cm and its corners rounded by round_cm."""
    b = RT.bits(RT.FAMILY, tag_id)
    n = b.shape[0]
    cell = tag_cm / n
    c, s = math.cos(math.radians(th_deg - 90)), math.sin(math.radians(th_deg - 90))
    out = []
    for r in range(n):
        for q in range(n):
            if not b[r, q]:
                continue
            x0 = (q - n / 2.0) * cell - grow_cm
            y0 = (n / 2.0 - r - 1) * cell - grow_cm
            w = cell + 2 * grow_cm
            pts = []
            if round_cm > 0:
                rr = min(round_cm, w / 2)
                for (ax, ay, a0) in ((x0 + w - rr, y0 + rr, -90), (x0 + w - rr, y0 + w - rr, 0),
                                     (x0 + rr, y0 + w - rr, 90), (x0 + rr, y0 + rr, 180)):
                    for k in range(n_round + 1):
                        a = math.radians(a0 + 90.0 * k / n_round)
                        pts.append((ax + rr * math.cos(a), ay + rr * math.sin(a)))
            else:
                pts = [(x0, y0), (x0 + w, y0), (x0 + w, y0 + w), (x0, y0 + w)]
            out.append([(cx + c * px - s * py, cy + s * px + c * py) for px, py in pts])
    return out


def render(cam, tag, mode, seed=0):
    """One camera's picture of one reference tile at its spot. mode: flat | nozzle | raised | raised_shadow |
    raised_shadow30 (light 30 deg up)."""
    import cv2
    rng = np.random.default_rng(seed)
    x, y, th = tag["x"], tag["y"], tag["th"]
    s, half = RT.TAG_CM, RT.OFFSET_CM
    img = cam.background().copy()
    zb = 0.12
    zt = 0.18
    flat = mode in ("flat", "nozzle")
    z_top = zt if not flat else zt
    # the white tile (its top face; flat: the whole tag at 1.8 mm)
    c, s_ = math.cos(math.radians(th - 90)), math.sin(math.radians(th - 90))
    sq = [(x + c * a - s_ * b, y + s_ * a + c * b) for a, b in ((-half, -half), (half, -half), (half, half), (-half, half))]
    cam._poly(img, [(px, py, zb if not flat else zt) for px, py in sq], 242.0)
    grow = XY_ERR_MM / 10.0 if mode == "nozzle" else 0.0
    rnd = LINE_MM / 20.0 if mode == "nozzle" else 0.0
    polys = cell_polys(tag["id"], s, x, y, th, grow_cm=grow, round_cm=rnd)
    if mode.startswith("raised_shadow"):
        elev = 30.0 if mode.endswith("30") else LIGHT_ELEV_DEG
        d = (zt - zb) / math.tan(math.radians(elev))          # the shadow's reach on the base, cm
        az = math.radians(30.0)                               # chosen: the light off to one side, 30 deg round
        sx, sy = d * math.cos(az), d * math.sin(az)
        for P in polys:
            sh = [(px + sx, py + sy, zb) for px, py in P]
            cam._poly(img, sh, 242.0 * SHADOW_GREY)
    if not flat:
        # the walls: each side of each cell, from the base to the top (black filament: as black as the top)
        for P in polys:
            for k in range(len(P)):
                (ax, ay), (bx, by) = P[k], P[(k + 1) % len(P)]
                cam._poly(img, [(ax, ay, zb), (bx, by, zb), (bx, by, zt), (ax, ay, zt)], 22.0)
    for P in polys:
        cam._poly(img, [(px, py, z_top) for px, py in P], 22.0)
    if cam.blur_px > 0:
        img = cv2.GaussianBlur(img, (0, 0), cam.blur_px)
    img = img * cam.gain + cam.offset + rng.normal(0, cam.noise, img.shape).astype(np.float32)
    out = np.clip(img, 0, 255).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)


def run(out_dir=None, seeds=3, say=print):
    import cv2
    import tag_cam as CAMS
    import site_fit as SF
    cfg = CAMS.default_config(height_cm=175.0, refs=False)
    cams = CAMS.cams_from_config(cfg)
    L, W = cfg["table"]
    poses = RT.world_poses(RT.default_config(), L, W)
    det = RT.detector()
    rows = []
    crops = {}
    modes = ("flat", "nozzle", "raised", "raised_shadow", "raised_shadow30")
    for cam in cams:
        for i, (x, y, th) in sorted(poses.items()):
            truth = cam.project(SF.tag_corners_world(x, y, th, 0.18, RT.TAG_CM))
            if not (np.all(np.isfinite(truth)) and np.all((truth[:, 0] > 2) & (truth[:, 0] < cam.W - 2)
                                                          & (truth[:, 1] > 2) & (truth[:, 1] < cam.H - 2))):
                continue
            ppc = cam.px_per_cm(x, y, 0.18)
            nadir = cam.C[:2]
            slope = math.hypot(x - nadir[0], y - nadir[1]) / cam.C[2]
            for mode in modes:
                errs, mids, reads = [], [], 0
                for sd in range(seeds):
                    img = render(cam, dict(id=int(i), x=x, y=y, th=th), mode, seed=sd)
                    got = [d for d in det.detect_full(img) if d.id == int(i)]
                    if got:
                        reads += 1
                        e = np.linalg.norm(got[0].corners - truth, axis=1)
                        errs.append(float(np.median(e)))
                        mids.append(float(np.linalg.norm(got[0].center - truth.mean(0))) / ppc)
                        if sd == 0 and mode in ("flat", "raised_shadow30") and i == "22":
                            c = truth.mean(0).astype(int)
                            crops[(cam.name, mode)] = img[max(0, c[1] - 45):c[1] + 45, max(0, c[0] - 45):c[0] + 45]
                rows.append(dict(camera=cam.name, tag=int(i), mode=mode, px_per_cm=round(ppc, 2),
                                 view_slope=round(slope, 2), wall_px=round(0.06 * slope * ppc, 2),
                                 read=reads, of=seeds, corner_err_px=round(float(np.median(errs)), 3) if errs else None,
                                 middle_shift_cm=round(float(np.median(mids)), 3) if mids else None))
    say("%-9s %-4s %-16s %7s %6s %7s %6s %12s %14s" % ("camera", "tag", "print", "px/cm", "slope", "wall px", "read",
                                                      "corner px", "middle cm"))
    for r in rows:
        say("%-9s %-4d %-16s %7.2f %6.2f %7.2f %3d/%-2d %12s %14s" % (r["camera"], r["tag"], r["mode"], r["px_per_cm"],
                                                                  r["view_slope"], r["wall_px"], r["read"], r["of"],
                                                                  r["corner_err_px"], r["middle_shift_cm"]))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "printcheck.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        if crops:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            ks = sorted(crops)
            fig, axs = plt.subplots(1, len(ks), figsize=(3.2 * len(ks), 3.6), dpi=110)
            axs = np.atleast_1d(axs)
            for ax, k in zip(axs, ks):
                ax.imshow(crops[k], cmap="gray", vmin=0, vmax=255, interpolation="nearest")
                ax.set_title("%s camera, %s" % (k[0], "flat paper" if k[1] == "flat" else
                                                "raised squares + shadow (light 30 deg up)"), fontsize=9)
                ax.axis("off")
            fig.text(0.5, 0.02, "Tag 22 (corner 3) in the simulated school camera at 1:1 pixels; drawn by "
                     "tools/fleet/ref_printcheck.py.", ha="center", fontsize=8)
            fig.tight_layout(rect=(0, 0.06, 1, 1))
            fig.savefig(os.path.join(out_dir, "printcheck_crops.png"))
            plt.close(fig)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None)
    ap.add_argument("--seeds", type=int, default=3)
    a = ap.parse_args(argv)
    cell = RT.TAG_CM * 10 / RT.CELLS
    print("Nozzle: reference cell %.2f mm = %.0f lines of %.2f mm; roof cell %.2f mm = %.0f lines. A corner rounds by "
          "about %.2f mm (%.1f %% / %.1f %% of a cell)." % (cell, cell / LINE_MM, LINE_MM, 80 / 7.0, 80 / 7.0 / LINE_MM,
                                                          LINE_MM / 2, 100 * LINE_MM / 2 / cell,
                                                          100 * LINE_MM / 2 / (80 / 7.0)))
    rows = run(a.out, a.seeds)
    bad = [r for r in rows if r["read"] < r["of"]]
    print("not read every time: %s" % (bad or "none"))
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
