#!/usr/bin/env python3
"""The tag setup (conference-table build, 2026-11-02): from what Nick clicks and what the cameras read, each camera's
model for the tag bridge, the table's size, and every production point's spot. Mission Control's Setup screen runs it
("Measure"); the dry runs run it against the simulator's tag cameras.

  1. pictures   a few seconds of both cameras with the robots standing on the table (anywhere both see is best:
                their homes near the middle are in both pictures)
  2. reads      every tag read in those pictures (tagcore), grouped into moments (site_fit.groups)
  3. clicks     the table corners each camera sees (1-2-3-4 counter-clockwise from above), each sticker in each
                picture it is in, with its number. (2026-11-03) With reference tags on the table (ref_tags.py) the
                corners need no clicks: the tags are found by themselves; corner clicking is a switch per site
                ("corner_clicks": off at school, on at home), still used when on
  3b. refs      the reference tags read in the pictures (6 pictures each, median corners)
  4. typed      optional: the table's size, sticker coordinates, distances between stickers (checks)
  5. fit        site_fit.full: the cameras' own measure first, the typed values compared, then used if they agree
  6. saved      <site conf_dir>/tagcal/<camera>.json (camera models), site_fit.json, obs.json (what went in),
                points.json (production points: label -> x, y, and how sure)

    python3 tools/fleet/tag_setup.py --dry --out /tmp/x        # the simulator's two cameras: setup and truth check
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "live"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "sim"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np       # noqa: E402

import tagcore as TC     # noqa: E402
import site_fit as SF    # noqa: E402

PICTURE_S = 3.0          # pictures taken for the setup's tag reads. No DOE: still robots, 3 s gives 30-45 reads each
MOMENT_S = 0.07          # reads this close in time by different cameras are one moment (tag_bridge.PAIR_S)
STILL_GROUP = 6          # still robots: their reads are pooled into this many moments per tag (more add nothing)


def tagcal_dir(conf_dir):
    return os.path.join(conf_dir, "tagcal")


def grab_pictures(streams, seconds=PICTURE_S, max_n=60):
    """{camera: [(t, grey)]} from each /live stream for `seconds`."""
    import sources as S
    out = {c: [] for c in streams}
    stop = threading.Event()

    def run(c, url):
        src = S.StreamSource(url, log=lambda s: None)
        try:
            for fr in src:
                if stop.is_set() or len(out[c]) >= max_n:
                    break
                out[c].append((fr.t_pi - 0.04, fr.img))
        except Exception:      # noqa: BLE001
            pass
        finally:
            src.close()
    th = [threading.Thread(target=run, args=(c, u), daemon=True) for c, u in streams.items()]
    for t in th:
        t.start()
    time.sleep(seconds)
    stop.set()
    for t in th:
        t.join(timeout=3)
    return out


def read_tags(pictures, tags):
    """Every read: [(t, camera, id, corners 4x2)] (the whole picture, enlarged)."""
    ids = list(int(i) for i in tags["ids"].values())
    det = TC.TagDetector(tags.get("family", TC.TAG_FAMILY), ids=ids)
    reads = []
    for c, pics in pictures.items():
        for t, g in pics:
            for d in det.detect_full(g, cam=c, t=t):
                reads.append((t, c, d.id, d.corners.tolist()))
    return reads, det.ms_per_call()


def groups_from_reads(reads, still=True):
    """Moments: with still robots each tag's reads are pooled (median corners) into STILL_GROUP moments per tag, each
    seen by every camera that read it; else reads within MOMENT_S by different cameras."""
    if still:
        out = []
        by = {}
        for t, c, i, cs in reads:
            by.setdefault(i, {}).setdefault(c, []).append((t, np.asarray(cs, float)))
        for i, cams in by.items():
            for k in range(STILL_GROUP):
                g = {"id": i, "reads": {}}
                for c, lst in cams.items():
                    part = lst[k::STILL_GROUP]
                    if part:
                        g["reads"][c] = np.median(np.array([p for _, p in part]), axis=0).tolist()
                if g["reads"]:
                    out.append(g)
        return out
    out, open_ = [], {}
    for t, c, i, cs in sorted(reads, key=lambda r: r[0]):
        g = open_.get(i)
        if g is not None and t - g["t"] <= MOMENT_S and c not in g["reads"]:
            g["reads"][c] = cs
            continue
        g = {"id": i, "t": t, "reads": {c: cs}}
        out.append(g)
        open_[i] = g
    return out


def make_obs(sizes, corners, stickers, tags, groups, table=None, distances=None, refs=None, ref_cfg=None,
             typed_along=None):
    """The site fit's input. sizes {cam: [W, H]}; corners {cam: {"1": [u, v], ...}}; stickers {label: {"cams": {cam:
    [u, v]}, "xy_cm": [x, y] or absent}}; table {"length_cm", "width_cm"} or None; distances [[a, b, cm]].
    refs {cam: {id: corners}} (ref_tags.read_refs) with ref_cfg (the site's ref_tags); typed_along {id: cm}."""
    obs = {"cameras": {c: {"size": list(sz)} for c, sz in sizes.items()}, "corners": corners,
           "stickers": stickers or {}, "tag_cm": float(tags["tag_cm"]), "tag_height_cm": float(tags["height_cm"]),
           "tag_groups": groups, "table": table or None, "distances": distances or []}
    if refs and any(refs.values()):
        import ref_tags as RT
        rc = ref_cfg or RT.default_config()
        obs["ref_tags"] = {"layout": RT.layout(rc), "tag_cm": float(rc.get("tag_cm", RT.TAG_CM)),
                           "z_cm": float(rc.get("z_cm", RT.Z_CM)), "offset_cm": float(rc.get("offset_cm", RT.OFFSET_CM)),
                           "reads": {c: dict(v) for c, v in refs.items() if c in sizes},
                           "typed_along_cm": {str(k): float(v) for k, v in (typed_along or {}).items()}}
    return obs


def fit_and_save(obs, out_dir):
    res = SF.full(obs)
    os.makedirs(out_dir, exist_ok=True)
    SF.save(res, out_dir)
    with open(os.path.join(out_dir, "obs.json"), "w") as f:
        json.dump(obs, f, indent=1)
    pts = {k: {"xy": v, "label": k} for k, v in res["stickers"].items()}
    with open(os.path.join(out_dir, "points.json"), "w") as f:
        json.dump(pts, f, indent=1)
    return res


def ref_words(res, layout=None):
    """Plain lines about the reference tags: which camera read which, the side tags' places."""
    rt = res.get("ref_tags")
    if not rt:
        return []
    import ref_tags as RT
    lay = layout or RT.layout()
    seen = rt.get("seen") or {}
    out = ["Reference tags read: %s." % "; ".join("%s %s" % (c, ", ".join(v) if v else "none") for c, v in seen.items())]
    every = {i for v in seen.values() for i in v}
    missing = [i for i in sorted(lay, key=int) if i not in every]
    if missing:
        out.append("Reference tag%s %s not seen by any camera (%s): the fit used the others." % (
            "s" if len(missing) > 1 else "", ", ".join(missing),
            "; ".join(RT.where_words(i) for i in missing)))
    for i, a in sorted((rt.get("along_cm") or {}).items()):
        out.append("Side tag %s: %.1f cm along its edge (+- %.1f), measured by the cameras." % (
            i, a, (rt.get("along_sd_cm") or {}).get(i, 0.0)))
    return out


def result_words(res):
    L, W = res["size_cm"]
    lines = ["The table measures %.1f x %.1f cm (+- %.1f / %.1f)." % (L, W, *res.get("size_sd_cm", [0, 0]))]
    lines += ref_words(res)
    for c, d in res["cameras"].items():
        rms = res["rms_px"].get(c, {})
        lines.append("%s: lens %.0f cm above the table, tilted %.0f deg; tags fit to %.2f px%s%s%s" % (
            c, d["camera_position_cm"][2], d["tilt_from_vertical_deg"], rms.get("tags", float("nan")),
            (", corners %.1f px" % rms["corners"]) if "corners" in rms else "",
            (", reference tags %.2f px" % rms["ref"]) if "ref" in rms else "",
            (", stickers %.1f px" % rms["stickers"]) if "stickers" in rms else ""))
    for ck in res.get("checks") or []:
        lines.append("check %s: cameras %s, typed %s, %s by %s" % (ck["what"], ck["cameras"], ck["typed"],
                                                                     "agree" if ck["ok"] else "DISAGREE", ck["off"]))
    lines += res.get("words") or []
    return lines


# ------------------------------------------------------------------------------------------------- simulator
def sim_clicks(true_models, table, stickers, sd_px=1.5, seed=0):
    """What Nick would click, from the true cameras: the corners each sees, each sticker where it is seen."""
    rng = np.random.default_rng(seed)
    L, W = table
    corners, st = {}, {}
    for c, m in true_models.items():
        Wd, Hd = m.image_size
        cc = {}
        for k, (x, y) in {1: (0, 0), 2: (L, 0), 3: (L, W), 4: (0, W)}.items():
            u, v = m.project((x, y, 0.0))
            if np.isfinite(u) and 3 <= u < Wd - 3 and 3 <= v < Hd - 3:
                cc[str(k)] = [u + rng.normal(0, sd_px), v + rng.normal(0, sd_px)]
        corners[c] = cc
        for lab, (x, y) in stickers.items():
            u, v = m.project((x, y, 0.0))
            if np.isfinite(u) and 5 <= u < Wd - 5 and 5 <= v < Hd - 5:
                st.setdefault(lab, {"cams": {}})["cams"][c] = [u + rng.normal(0, sd_px), v + rng.normal(0, sd_px)]
    return corners, st


def sim_setup(true_dir, streams, cfg, out_dir, typed=True, say=print, click_sd=1.5, seconds=PICTURE_S,
              corner_clicks=True, use_refs=True):
    """The whole setup against the simulator: wait for its cameras, grab, read, click (from the truth + noise), fit,
    save. Returns the fit result (and writes truth_check.json: the fit against the true cameras). corner_clicks: the
    corners clicked (off: the reference tags alone, 2026-11-03); use_refs: read the reference tags the config draws."""
    from cameras import CameraModel
    t0 = time.time()
    paths = {c: os.path.join(true_dir, "true_%s.json" % c) for c in streams}
    while not all(os.path.exists(p) for p in paths.values()) and time.time() - t0 < 60:
        time.sleep(0.2)
    truth = {c: CameraModel.load(p) for c, p in paths.items()}
    pics = grab_pictures(streams, seconds)
    sizes = {c: [int(p[0][1].shape[1]), int(p[0][1].shape[0])] for c, p in pics.items() if p}
    reads, ms = read_tags(pics, cfg["tags"])
    say("setup: %s pictures, %d tag reads (%.0f ms a picture)" % (
        ", ".join("%s %d" % (c, len(p)) for c, p in pics.items()), len(reads), ms))
    L, W = cfg["table"]
    # a camera sending smaller pictures than its truth (low_res): scale the true model for the clicks
    tm = {}
    for c, m in truth.items():
        if c in sizes and tuple(sizes[c]) != tuple(m.image_size):
            k = sizes[c][0] / float(m.image_size[0])
            K = m.K.copy()
            K[:2] *= k
            m = CameraModel(K, m.dist, m.rvec, m.tvec, sizes[c], name=c)
        tm[c] = m
    corners, st = sim_clicks(tm, (L, W), {k: tuple(v) for k, v in cfg["stickers"].items()}, sd_px=click_sd)
    if not corner_clicks:
        corners = {c: {} for c in corners}
    groups = groups_from_reads(reads, still=True)
    table = {"length_cm": L, "width_cm": W} if typed else None
    refs, rcfg = None, None
    if use_refs and (cfg.get("ref_tags") or {}).get("poses"):
        import ref_tags as RT
        rcfg = cfg["ref_tags"].get("config") or RT.default_config()
        refs = RT.read_refs(pics, rcfg)
        say("setup: reference tags read: %s" % "; ".join("%s %s" % (c, ", ".join(sorted(v, key=int)) or "none")
                                                        for c, v in refs.items()))
    obs = make_obs(sizes, corners, st, cfg["tags"], groups, table=table, refs=refs, ref_cfg=rcfg)
    res = fit_and_save(obs, out_dir)
    cmp_ = SF.compare(res, tm, {k: tuple(v) for k, v in cfg["stickers"].items()}, H=float(cfg["tags"]["height_cm"]),
                      table=(L, W))
    rt = res.get("ref_tags") or {}
    tr = (cfg.get("ref_tags") or {}).get("along") or {}
    if rt.get("along_cm") and tr:
        cmp_["side_tags_along_off_cm"] = {i: round(rt["along_cm"][i] - tr[i], 2) for i in rt["along_cm"] if i in tr}
    with open(os.path.join(out_dir, "truth_check.json"), "w") as f:
        json.dump(cmp_, f, indent=1)
    for w in result_words(res):
        say("setup: " + w)
    say("setup (simulator truth): %s" % json.dumps(cmp_))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry", action="store_true", help="the simulator: start it, set up, compare with the truth")
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-typed", action="store_true")
    ap.add_argument("--size", default="1280x960")
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--no-corner-clicks", action="store_true", help="the reference tags alone (no corner clicks)")
    ap.add_argument("--no-refs", action="store_true", help="no reference tags on the table (the clicks alone)")
    ap.add_argument("--hide-ref", default="", help="reference tag ids not on the table, e.g. 20")
    a = ap.parse_args(argv)
    if not a.dry:
        print(__doc__)
        return 1
    import tag_cam as CAMS
    import production as PR
    out = os.path.abspath(a.out or os.path.join(REPO, "logs", time.strftime("%Y%m%d-%H%M%S") + "-tagsetup-dry"))
    os.makedirs(out, exist_ok=True)
    w, h = (int(v) for v in a.size.split("x"))
    cfg = CAMS.default_config(size=(w, h), seed=a.seed, refs=not a.no_refs,
                              hide_refs=[x for x in a.hide_ref.split(",") if x.strip()])
    cfgp = os.path.join(out, "tag_cams.json")
    with open(cfgp, "w") as f:
        json.dump(cfg, f, indent=1)
    L, W = cfg["table"]
    mp = os.path.join(out, "map.json")
    PR.save_map(PR.site_map((L, W), list(cfg["tags"]["ids"]), points={}), mp)
    sim = subprocess.Popen([sys.executable, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--out",
                            os.path.join(out, "sim"), "--tag-cams", cfgp, "--fix-hz", "0", "--max-s", "40", "--quiet",
                            "--persist"], cwd=REPO)
    try:
        streams = {c["name"]: "127.0.0.1:%d" % c["port"] for c in cfg["cams"]}
        sim_setup(os.path.join(out, "sim"), streams, cfg, os.path.join(out, "tagcal"), typed=not a.no_typed,
                  corner_clicks=not a.no_corner_clicks)
    finally:
        sim.terminate()
        sim.wait(timeout=10)
    return 0


if __name__ == "__main__":
    sys.exit(main())
