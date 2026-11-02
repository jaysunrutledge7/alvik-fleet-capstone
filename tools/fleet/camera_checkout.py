#!/usr/bin/env python3
"""Camera check-out before a run (conference-table build, Nick 2026-11-02): each camera's picture size, picture rate,
sharpness, exposure, how much of the table it sees, and whether a robot's tag is readable at the farthest point it
must cover. Each line says PASS, WARN or FAIL in plain words, with what to do.

Measured on ~2 s of each camera's pictures (the recorder's /live, or the simulator's):

  size       the picture's size against what the tags need (tag_sizing.py: tag25h9 at 8 cm reads in 100 % of the
             synthetic pictures from 2.5 px/cm with a typical webcam's blur, from 3.5 px/cm soft and tilted)
  rate       pictures a second, from the pictures' own Pi times
  sharpness  a tag's edge width when a tag is in view (10-90 % of the step across its black border, in pixels), else
             the picture's Tenengrad focus measure (a weaker, scene-dependent sign)
  exposure   white-clipped and black-clipped shares; a tag's black and white levels (its contrast) when one is in view
  coverage   with a camera model (after Measure): which table corners and how much of the table it sees, and with
             the other camera: whether every part of the table is seen by at least one (the overlap in the middle);
             before Measure: the corners clicked so far
  tags       the farthest table point it must cover: the tag's size there in pixels and pixels per cell, against the
             sizing tests; and the share of pictures where the robots' tags were read

    python3 tools/fleet/camera_checkout.py --stream overhead=127.0.0.1:8000 --stream side=127.0.0.1:8001
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "live"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np       # noqa: E402

import tagcore as TC     # noqa: E402
import tag_geom as TG    # noqa: E402

NEED_PX_PER_CELL = 2.9     # tag25h9's 7 cells at 2.5 px/cm (8 cm tag): read in 100 % of typical-webcam pictures
                           # (tag_sizing.py, docs/results/tag-sizing). Under this: WARN
FAIL_PX_PER_CELL = 2.3     # at 2.0 px/cm reads fell to 85-95 %; under this: FAIL. DOE: the real cameras at three heights
RATE_PASS, RATE_FAIL = 10.0, 5.0   # pictures a second: the bridge fixes at up to 10 a second. No DOE: lines
EDGE_PASS_PX, EDGE_FAIL_PX = 4.0, 5.5   # a tag edge's 10-90 % width. Chosen from the simulator: a 0.9 px lens blur
                           # (tags read in 100 % of pictures) measures 3.6 px here (blur, pixel size and JPEG together;
                           # the first lines, 2.2 / 3.5, called that sharp picture out of focus: Playwright run, 2 Nov).
                           # DOE note: the real cameras focused and turned slightly out, read rate against edge width
CLIP_WARN, CLIP_FAIL = 0.02, 0.15  # share of the table's pixels at 250+ (washed out). Chosen. No DOE: lines
CONTRAST_PASS, CONTRAST_FAIL = 80.0, 40.0    # a tag's white minus black (grey levels). Chosen from the sizing tests'
                           # synthetic tags (150-200); real matte paper under room light: 90-160 expected. DOE note
SEEN_PASS, SEEN_FAIL = 0.95, 0.6    # share of pictures with every visible robot's tag read


def verdict(v, pass_if, fail_if):
    if pass_if(v):
        return "PASS"
    return "FAIL" if fail_if(v) else "WARN"


def edge_width(grey, det):
    """10-90 % rise width (px) across the tag's black border, the median over its four sides (profiles outward)."""
    c = det.corners
    mid = c.mean(0)
    ws = []
    for k in range(4):
        a, b = c[k], c[(k + 1) % 4]
        e = b - a
        L = np.linalg.norm(e)
        if L < 6:
            continue
        n = np.array([e[1], -e[0]]) / L
        if np.dot((a + b) / 2 - mid, n) < 0:
            n = -n
        for f in (0.3, 0.5, 0.7):
            p0 = a + e * f
            ts = np.linspace(-4.0, 4.0, 33)
            pts = p0[None, :] + ts[:, None] * n[None, :]
            xi, yi = pts[:, 0], pts[:, 1]
            H, W = grey.shape
            if xi.min() < 1 or yi.min() < 1 or xi.max() > W - 2 or yi.max() > H - 2:
                continue
            x0, y0 = np.floor(xi).astype(int), np.floor(yi).astype(int)
            fx, fy = xi - x0, yi - y0
            g = grey.astype(np.float32)
            v = (g[y0, x0] * (1 - fx) * (1 - fy) + g[y0, x0 + 1] * fx * (1 - fy) + g[y0 + 1, x0] * (1 - fx) * fy
                 + g[y0 + 1, x0 + 1] * fx * fy)
            lo, hi = float(np.percentile(v[:8], 50)), float(np.percentile(v[-8:], 50))
            if hi - lo < 20:
                continue
            q = (v - lo) / (hi - lo)
            try:
                t10 = ts[np.argmax(q >= 0.1)]
                t90 = ts[np.argmax(q >= 0.9)]
                ws.append(abs(t90 - t10))
            except ValueError:
                pass
    return float(np.median(ws)) if ws else None


def tag_levels(grey, det):
    """(black, white) grey levels of a tag: inside its black border and in the white just outside it."""
    import cv2
    c = det.corners.astype(np.float32)
    mid = c.mean(0)
    inner = mid + (c - mid) * 0.93
    outer = mid + (c - mid) * 1.12
    m_in = np.zeros(grey.shape, np.uint8)
    cv2.fillConvexPoly(m_in, np.round(inner).astype(np.int32), 1)
    m_bd = np.zeros(grey.shape, np.uint8)
    cv2.fillConvexPoly(m_bd, np.round(c).astype(np.int32), 1)
    m_out = np.zeros(grey.shape, np.uint8)
    cv2.fillConvexPoly(m_out, np.round(outer).astype(np.int32), 1)
    ring_out = (m_out > 0) & (m_bd == 0)
    ring_in = (m_bd > 0) & (m_in == 0)
    if ring_in.sum() < 4 or ring_out.sum() < 4:
        return None
    return float(np.median(grey[ring_in])), float(np.median(grey[ring_out]))


def tenengrad(grey):
    import cv2
    gx = cv2.Sobel(grey, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(grey, cv2.CV_32F, 0, 1, ksize=3)
    return float(np.mean(gx * gx + gy * gy))


def check_camera(name, pics, tags=None, model=None, area=None, want_size=None, clicks=None, label=None):
    """pics: [(t_pi, grey)]. Returns {camera, label, items: [{what, verdict, words, todo}], numbers}."""
    items, num = [], {}
    lab = label or name
    if not pics:
        return {"camera": name, "label": lab, "items": [dict(what="pictures", verdict="FAIL", words="no pictures at all",
                                                             todo="Is it plugged in, and is the recorder running? "
                                                                  "(./run-fleet-pi.sh rec)")], "numbers": {}}
    H, W = pics[-1][1].shape[:2]
    num["size"] = [W, H]
    ts = [t for t, _ in pics]
    rate = (len(ts) - 1) / max(1e-6, ts[-1] - ts[0]) if len(ts) > 1 else 0.0
    num["rate"] = round(rate, 1)
    ws = want_size or [W, H]
    v = "PASS" if [W, H] == list(ws) else "WARN"
    items.append(dict(what="picture size", verdict=v, words="%d x %d%s" % (W, H, "" if v == "PASS" else
                      " (the site asks %d x %d)" % tuple(ws)),
                      todo="" if v == "PASS" else "The camera did not take the size asked for: it may not offer it. "
                                                  "See the tag line below for whether this size is enough."))
    v = verdict(rate, lambda r: r >= RATE_PASS, lambda r: r < RATE_FAIL)
    items.append(dict(what="picture rate", verdict=v, words="%.1f pictures a second" % rate,
                      todo="" if v == "PASS" else "Too few pictures: lower the size or the rate in the site profile, "
                                                  "or plug the cameras into different USB ports (blue = USB 3)."))
    g = pics[-1][1]
    # exposure: on the table when the model says where it is, else the whole picture
    sel = np.ones(g.shape, bool)
    if model is not None and area is not None:
        import cv2
        x0, y0, x1, y1 = area
        poly = [model.project((x, y, 0.0)) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        if all(np.isfinite(u) for u, _ in poly):
            m = np.zeros(g.shape, np.uint8)
            cv2.fillConvexPoly(m, np.round(np.array(poly)).astype(np.int32), 1)
            if m.sum() > 100:
                sel = m > 0
    vals = g[sel]
    clip_hi, clip_lo = float(np.mean(vals >= 250)), float(np.mean(vals <= 5))
    num.update(clip_white=round(clip_hi, 4), clip_black=round(clip_lo, 4), median_grey=float(np.median(vals)))
    # tags in view
    reads, levels, edges, sizes = 0, [], [], []
    det = None
    if tags:
        det = TC.TagDetector(tags.get("family", TC.TAG_FAMILY), ids=list(int(i) for i in tags["ids"].values()))
        per = []
        for t, gi in pics[-12:]:
            ds = det.detect_full(gi)
            per.append(len(ds))
            for d in ds:
                lv = tag_levels(gi, d)
                if lv:
                    levels.append(lv)
                e = edge_width(gi, d)
                if e:
                    edges.append(e)
                sizes.append(d.size_px)
        reads = max(per) if per else 0
        num["tags_read_share"] = round(float(np.mean([p >= reads for p in per])) if reads else 0.0, 3)
        num["tags_in_view"] = reads
    v = verdict(clip_hi, lambda c: c <= CLIP_WARN, lambda c: c > CLIP_FAIL)
    words = "%.1f %% of the %s white-clipped, %.1f %% black, middle grey %d" % (
        100 * clip_hi, "table" if sel is not None and sel.sum() < g.size else "picture", 100 * clip_lo,
        num["median_grey"])
    todo = "" if v == "PASS" else ("Washed out: turn down the exposure (Details, camera controls), close a blind or "
                                   "move a lamp so it does not shine on the table.")
    if levels:
        blk, wht = float(np.median([a for a, _ in levels])), float(np.median([b for _, b in levels]))
        num.update(tag_black=round(blk, 1), tag_white=round(wht, 1))
        vc = verdict(wht - blk, lambda c: c >= CONTRAST_PASS, lambda c: c < CONTRAST_FAIL)
        words += "; tag black %d, white %d (contrast %d)" % (blk, wht, wht - blk)
        if vc != "PASS":
            todo = (todo + " " if todo else "") + ("Low tag contrast: more light on the table, or matte paper "
                                                   "(a glossy tag reflects the lights).")
        v = max(v, vc, key=["PASS", "WARN", "FAIL"].index)
    items.append(dict(what="exposure", verdict=v, words=words, todo=todo))
    # sharpness
    if edges:
        e = float(np.median(edges))
        num["edge_px"] = round(e, 2)
        v = verdict(e, lambda x: x <= EDGE_PASS_PX, lambda x: x > EDGE_FAIL_PX)
        items.append(dict(what="sharpness", verdict=v, words="tag edges %.1f px wide (sharp is under %.1f)" % (
            e, EDGE_PASS_PX), todo="" if v == "PASS" else "Out of focus: turn the lens ring (or set the focus in the "
                                                          "camera controls) until the tag's squares look crisp, then "
                                                          "check again."))
    else:
        tg = tenengrad(g)
        num["tenengrad"] = round(tg, 1)
        items.append(dict(what="sharpness", verdict="WARN", words="no tag in view to measure it on (focus measure %.0f)"
                          % tg, todo="Put a robot (or a printed spare tag) on the table in this camera's view and "
                                     "check again."))
    # coverage and the farthest tag
    if model is not None and area is not None:
        x0, y0, x1, y1 = area
        corners = {1: (0, 0), 2: (x1, 0), 3: (x1, y1), 4: (0, y1)}
        seen_c = [k for k, (x, y) in corners.items() if model.in_image(*model.project((x, y, 0.0)), margin=2)]
        num["corners_seen"] = seen_c
        h = float((tags or {}).get("height_cm") or 6.0)
        s = float((tags or {}).get("tag_cm") or 8.0)
        worst = (1e9, None)
        xs, ys = np.linspace(x0, x1, 25), np.linspace(y0, y1, 9)
        seen = 0
        for x in xs:
            for y in ys:
                u, v_ = model.project((x, y, h))
                if not model.in_image(u, v_, margin=4):
                    continue
                seen += 1
                px = TG.tag_px_at(model, x, y, h, s)
                if px < worst[0]:
                    worst = (px, (round(float(x), 1), round(float(y), 1)))
        num["sees_share"] = round(seen / float(len(xs) * len(ys)), 3)
        if worst[1] is not None:
            n = TC.FAMILIES[(tags or {}).get("family", TC.TAG_FAMILY)][1]
            ppc = worst[0] / n
            num.update(farthest_tag_px=round(worst[0], 1), farthest_px_per_cell=round(ppc, 2), farthest_at=worst[1])
            v = verdict(ppc, lambda q: q >= NEED_PX_PER_CELL, lambda q: q < FAIL_PX_PER_CELL)
            items.append(dict(what="tag at the farthest point", verdict=v,
                              words="%.0f px across at (%.0f, %.0f) cm: %.1f px a cell (reads from %.1f)" % (
                                  worst[0], worst[1][0], worst[1][1], ppc, NEED_PX_PER_CELL),
                              todo="" if v == "PASS" else "Too small there: a larger picture size (site profile "
                                                          "capture), the camera lower or nearer that end, or the other "
                                                          "camera covering that part."))
    elif clicks:
        num["corners_clicked"] = sorted(clicks)
        items.append(dict(what="table corners", verdict="PASS" if len(clicks) >= 2 else "WARN",
                          words="corners %s clicked" % ", ".join(str(k) for k in sorted(clicks)),
                          todo="" if len(clicks) >= 2 else "Click at least the two table corners this camera sees."))
    if tags and sizes:
        med = float(np.median(sizes))
        n = TC.FAMILIES[tags.get("family", TC.TAG_FAMILY)][1]
        items.append(dict(what="tags read now", verdict="PASS" if num.get("tags_read_share", 0) >= SEEN_PASS else
                          ("FAIL" if num.get("tags_read_share", 0) < SEEN_FAIL else "WARN"),
                          words="%d tag%s read, %.0f px across (%.1f px a cell), in %.0f %% of the pictures" % (
                              num["tags_in_view"], "" if num["tags_in_view"] == 1 else "s", med, med / n,
                              100 * num["tags_read_share"]),
                          todo="" if num.get("tags_read_share", 0) >= SEEN_PASS else
                          "Tags read only sometimes: check focus, glare on the tag, and the picture size."))
    return {"camera": name, "label": lab, "items": items, "numbers": num,
            "verdict": max((i["verdict"] for i in items), key=["PASS", "WARN", "FAIL"].index)}


def coverage_together(models, area, height_cm=6.0):
    """With both cameras: the share of the table seen by none, by one, by both; the overlap's x range."""
    x0, y0, x1, y1 = area
    xs, ys = np.linspace(x0, x1, 60), np.linspace(y0, y1, 15)
    none, both = 0, 0
    over_x = []
    for x in xs:
        for y in ys:
            k = sum(1 for m in models.values() if m.in_image(*m.project((x, y, height_cm)), margin=4))
            none += k == 0
            if k >= 2:
                both += 1
                over_x.append(x)
    n = float(len(xs) * len(ys))
    out = {"unseen_share": round(none / n, 3), "both_share": round(both / n, 3),
           "overlap_x_cm": [round(min(over_x), 1), round(max(over_x), 1)] if over_x else None}
    v = "PASS" if none == 0 else ("FAIL" if none / n > 0.05 else "WARN")
    words = ("every part of the table is seen by at least one camera" if none == 0 else
             "%.0f %% of the table is seen by neither camera" % (100 * none / n))
    if out["overlap_x_cm"]:
        words += "; both see x = %.0f to %.0f cm" % tuple(out["overlap_x_cm"])
    out.update(verdict=v, words=words, todo="" if v == "PASS" else "Turn or move a camera until the gap closes "
                                                                  "(the robots go blind there).")
    return out


def run(streams, tags=None, models=None, area=None, want_size=None, labels=None, clicks=None, seconds=2.0):
    """Every camera's check (streams {camera: url}); pictures grabbed here."""
    import tag_setup as TSU
    pics = TSU.grab_pictures(streams, seconds, max_n=40)
    out = {"cameras": [], "t": time.time()}
    for c in streams:
        out["cameras"].append(check_camera(c, pics.get(c) or [], tags=tags, model=(models or {}).get(c), area=area,
                                           want_size=want_size, clicks=(clicks or {}).get(c),
                                           label=(labels or {}).get(c)))
    if models and area and len(models) >= 2:
        out["together"] = coverage_together(models, area, float((tags or {}).get("height_cm") or 6.0))
    vs = [c["verdict"] for c in out["cameras"]] + ([out["together"]["verdict"]] if "together" in out else [])
    out["verdict"] = max(vs, key=["PASS", "WARN", "FAIL"].index) if vs else "FAIL"
    return out


def words(res):
    lines = []
    for c in res["cameras"]:
        lines.append("%s: %s" % (c["label"], c.get("verdict", "?")))
        for i in c["items"]:
            lines.append("  %-5s %-26s %s%s" % (i["verdict"], i["what"], i["words"], ("  -> " + i["todo"]) if i["todo"]
                                                 else ""))
    if "together" in res:
        t = res["together"]
        lines.append("Both cameras: %s %s%s" % (t["verdict"], t["words"], ("  -> " + t["todo"]) if t["todo"] else ""))
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stream", action="append", default=[], metavar="NAME=HOST:PORT")
    ap.add_argument("--site", default=None)
    ap.add_argument("--cal-dir", default=None, help="the camera models (default: the site's tagcal folder)")
    a = ap.parse_args(argv)
    import sites as SI
    s = SI.load(a.site)
    tags = dict(s.get("tags") or {})
    tags.setdefault("height_cm", None)
    if tags.get("height_cm") is None:
        tags["height_cm"] = 6.0
    streams = dict(x.split("=", 1) for x in a.stream) or {"overhead": "127.0.0.1:8000", "side": "127.0.0.1:8001"}
    cd = a.cal_dir or os.path.join(s["conf_dir"], "tagcal")
    models, area = {}, None
    import e02_camera as CAM
    for c in streams:
        p = os.path.join(cd, c + ".json")
        if os.path.exists(p):
            models[c] = CAM.load_camera(p)
    fit = os.path.join(cd, "site_fit.json")
    if os.path.exists(fit):
        with open(fit) as f:
            L, W = json.load(f)["size_cm"]
        area = (0, 0, L, W)
    res = run(streams, tags=tags, models=models or None, area=area,
              want_size=[s["capture"]["width"], s["capture"]["height"]],
              labels={k: v.get("label", k) for k, v in s["cameras"].items()})
    print("\n".join(words(res)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
