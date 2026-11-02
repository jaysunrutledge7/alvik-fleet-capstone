#!/usr/bin/env python3
"""Compare our robot positions with an external recorder's (conference-table build, Nick 2026-11-02).

The external recorder (here: another team's RFID system on its own laptop, kept separate and unchanged) writes its
own CSV of timestamped positions per robot, in its own units, its own coordinates and on its own clock. This tool
lines the two recordings up and says how far apart they are, robot by robot, over time.

  1. columns   which of their columns is the time, the robot, x, y (and optionally heading, an event column with
               the sync marks); their units (cm, m, in, ft) and time format (unix s / ms / us, ISO). A JSON config
               (--config) or the --col options; robot names mapped (their id -> our name) when they differ.
  2. clock     their clock minus ours, from the sync marks: ours (sync_marks.jsonl: Mission Control's Start / End
               mark, the Pi's exact time) against theirs (rows whose event column says a mark, or --their-mark
               start=<time>). Both marks: the offset and the drift between them. No marks: the offset that best
               lines up the two tracks (searched over +- --search-s), said plainly as a fallback.
  3. frames    their x, y -> our table frame, fitted from shared reference points (the stickers measured in both:
               --refs a CSV of label, x, y in their frame; ours from the site fit's points.json or the map's spots):
               rotation, shift, scale (a check on the units: a scale off 1 by more than 2 % is said) and a mirror if
               their axes go round the other way. Residuals at each point are reported.
  4. compare   their each row at its time -> our position then (interpolated from our pose timeline, camera tags
               first, else the robot's own pose; never across a gap over --max-gap-s) -> the difference.

Out (--out): merged.csv (each of their rows with ours and the difference), summary.json (per robot: n, median,
mean, p95, max, RMS, bias; the clock offset and how it was found; the transform and its residuals), compare.png
(distance over time per robot; both tracks on the table).

    python3 tools/fleet/merge_external.py --ours logs/<mission> --theirs their.csv --refs refs.csv \
        --col time=timestamp --col robot=robot --col x=x --col y=y --units ft --time-format unix --out cmp/
    python3 tools/fleet/merge_external.py --selftest
"""
import argparse
import bisect
import csv
import datetime as dt
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

UNITS_CM = {"cm": 1.0, "m": 100.0, "mm": 0.1, "in": 2.54, "ft": 30.48}
MAX_GAP_S = 0.5         # our track is not interpolated across a gap longer than this. Chosen: 5 camera fixes at 10/s.
                        # No DOE: a definition (a longer gap is a missing comparison, not a guessed one)
SEARCH_S = 120.0        # no sync marks: the clock offset searched over +- this. No DOE: a search window
SCALE_WARN = 0.02       # the fitted scale more than 2 % from 1 (after the units): the units or a reference point are off
DRIFT_WARN_S = 0.2      # start and end marks giving offsets this far apart: one clock drifts (or a mark was late)


def parse_time(v, fmt):
    v = str(v).strip()
    if fmt == "unix":
        return float(v)
    if fmt == "unix_ms":
        return float(v) / 1000.0
    if fmt == "unix_us":
        return float(v) / 1e6
    if fmt == "iso":
        d = dt.datetime.fromisoformat(v.replace("Z", "+00:00"))
        if d.tzinfo is None:
            d = d.astimezone()
        return d.timestamp()
    raise ValueError("time format %r: unix, unix_ms, unix_us or iso" % fmt)


def default_config():
    return {"time": {"column": "t", "format": "unix"}, "robot": {"column": "robot", "map": {}},
            "x": {"column": "x"}, "y": {"column": "y"}, "units": "cm", "heading": None, "event": None}


def read_theirs(path, cfg):
    """[(t, robot, x_cm, y_cm, heading or None)], [(t, what)] marks."""
    rows, marks = [], []
    tc, rc = cfg["time"]["column"], (cfg.get("robot") or {}).get("column")
    rmap = (cfg.get("robot") or {}).get("map") or {}
    k = UNITS_CM[cfg.get("units", "cm")]
    hc = (cfg.get("heading") or {}).get("column") if cfg.get("heading") else None
    ev = cfg.get("event") or {}
    evc, evv = ev.get("column"), [str(v).lower() for v in (ev.get("sync_values") or ["sync", "start", "end"])]
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                t = parse_time(r[tc], cfg["time"].get("format", "unix"))
            except (KeyError, ValueError):
                continue
            if evc and str(r.get(evc, "")).strip().lower() in evv:
                marks.append((t, str(r.get(evc)).strip().lower()))
                continue
            try:
                x, y = float(r[cfg["x"]["column"]]) * k, float(r[cfg["y"]["column"]]) * k
            except (KeyError, ValueError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            name = str(r.get(rc, "")) if rc else ""
            name = rmap.get(name, name)
            h = None
            if hc and r.get(hc) not in (None, ""):
                try:
                    h = float(r[hc])
                except ValueError:
                    h = None
            rows.append((t, name, x, y, h))
    return rows, marks


def read_ours(folder, prefer="camera"):
    """{robot: (times, xs, ys, hs)} from pose_timeline.csv (camera tag rows, else the robot's own pose)."""
    p = os.path.join(folder, "pose_timeline.csv") if os.path.isdir(folder) else folder
    by = {}
    with open(p) as f:
        for r in csv.DictReader(f):
            src = "camera" if r["source"].startswith("camera") else "robot"
            by.setdefault(r["robot"], {}).setdefault(src, []).append(
                (float(r["t_unix"]), float(r["x_cm"]), float(r["y_cm"]),
                 float(r["heading_deg"]) if r.get("heading_deg") else float("nan")))
    out = {}
    for n, d in by.items():
        lst = d.get(prefer) or d.get("robot") or d.get("camera")
        lst.sort()
        a = np.array(lst)
        out[n] = (a[:, 0], a[:, 1], a[:, 2], a[:, 3], prefer if d.get(prefer) else "robot")
    return out


def interp(track, t, max_gap=MAX_GAP_S):
    ts, xs, ys, hs = track[:4]
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(ts):
        return None
    t0, t1 = ts[i - 1], ts[i]
    if t1 - t0 > max_gap:
        return None
    w = (t - t0) / max(1e-9, t1 - t0)
    h = hs[i - 1] + w * (((hs[i] - hs[i - 1] + 180) % 360) - 180) if np.isfinite(hs[i - 1]) and np.isfinite(hs[i]) \
        else float("nan")
    return xs[i - 1] + w * (xs[i] - xs[i - 1]), ys[i - 1] + w * (ys[i] - ys[i - 1]), h


def fit_similarity(src, dst, allow_scale=True):
    """dst ~ s R src + t (2-D, Umeyama), trying a mirror too. Returns dict(R, t, s, mirror, rms, resid)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    best = None
    for mirror in (False, True):
        S = src * [1, -1] if mirror else src
        ms, md = S.mean(0), dst.mean(0)
        a, b = S - ms, dst - md
        U, D, Vt = np.linalg.svd(b.T @ a / len(S))
        E = np.eye(2)
        if np.linalg.det(U @ Vt) < 0:
            E[1, 1] = -1
        R = U @ E @ Vt
        s = float(np.trace(np.diag(D) @ E) / max(1e-12, np.mean(np.sum(a ** 2, 1)))) if allow_scale else 1.0
        t = md - s * R @ ms
        res = dst - (s * (S @ R.T) + t)
        rms = float(np.sqrt(np.mean(np.sum(res ** 2, 1))))
        cand = dict(R=R, t=t, s=s, mirror=mirror, rms=rms, resid=np.linalg.norm(res, axis=1))
        if best is None or rms < best["rms"] - 1e-9:
            best = cand
    return best


def apply(T, xy):
    p = np.asarray(xy, float)
    if T["mirror"]:
        p = p * [1, -1]
    return T["s"] * (p @ T["R"].T) + T["t"]


def heading_map(T, h_deg):
    """Their heading (deg, their frame) -> ours."""
    a = math.radians(h_deg)
    v = np.array([math.cos(a), math.sin(a)])
    if T["mirror"]:
        v = v * [1, -1]
    w = T["R"] @ v
    return math.degrees(math.atan2(w[1], w[0]))


def offset_from_marks(ours, theirs):
    """ours / theirs: [(t, what)]. Matched by name when both name their marks the same, else in order. Returns (offset
    their - ours, how, pairs) or (None, why, [])."""
    if not ours or not theirs:
        return None, ("no sync marks on our side" if not ours else "no sync marks in their file"), []
    o = sorted(ours)
    t_ = sorted(theirs)
    pairs = []
    named = {w: t for t, w in o}
    for t, w in t_:
        if w in named:
            pairs.append((w, named[w], t))
    if not pairs:
        pairs = [("mark %d" % (k + 1), a[0], b[0]) for k, (a, b) in enumerate(zip(o, t_))]
    offs = [b - a for _, a, b in pairs]
    return float(np.median(offs)), "%d sync mark%s" % (len(pairs), "" if len(pairs) == 1 else "s"), pairs


def offset_from_motion(ours, theirs_rows, T, search_s=SEARCH_S, step=0.05):
    """No marks: the offset (their - ours) that puts their positions nearest ours (coarse 1 s, then fine)."""
    def cost(off):
        d = []
        for t, n, x, y, _ in theirs_rows[:: max(1, len(theirs_rows) // 600)]:
            tr = ours.get(n)
            if tr is None:
                continue
            p = interp(tr, t - off, max_gap=MAX_GAP_S * 2)
            if p is None:
                continue
            q = apply(T, [x, y])
            d.append(math.hypot(q[0] - p[0], q[1] - p[1]))
        return (float(np.median(d)) if len(d) > 20 else float("inf")), len(d)
    grid = np.arange(-search_s, search_s + 1e-9, 1.0)
    cs = [cost(o)[0] for o in grid]
    o0 = float(grid[int(np.argmin(cs))])
    fine = np.arange(o0 - 1.0, o0 + 1.0 + 1e-9, step)
    cf = [cost(o)[0] for o in fine]
    return float(fine[int(np.argmin(cf))]), float(min(cf))


def summarise(d):
    d = np.asarray(d, float)
    if len(d) == 0:
        return None
    return dict(n=int(len(d)), median_cm=round(float(np.median(d)), 2), mean_cm=round(float(np.mean(d)), 2),
                p95_cm=round(float(np.percentile(d, 95)), 2), max_cm=round(float(np.max(d)), 2),
                rms_cm=round(float(np.sqrt(np.mean(d ** 2))), 2))


def run(ours_folder, theirs_csv, cfg, refs_csv=None, our_points=None, their_marks=None, out=None,
        search_s=SEARCH_S, prefer="camera", max_gap_s=MAX_GAP_S, chart=True):
    words = []
    import sync_tools as ST
    ours = read_ours(ours_folder, prefer)
    theirs, their_ev = read_theirs(theirs_csv, cfg)
    for w, v in (their_marks or {}).items():
        their_ev.append((float(v), w))
    om = [(m["t_unix"], m["what"]) for m in ST.read_marks(ours_folder)] if os.path.isdir(ours_folder) else []
    # the transform
    if our_points is None and os.path.isdir(ours_folder):
        our_points = points_of(ours_folder)
    refs = {}
    if refs_csv:
        k = UNITS_CM[cfg.get("units", "cm")]
        with open(refs_csv) as f:
            for r in csv.DictReader(f):
                refs[str(r["label"]).strip().upper()] = (float(r["x"]) * k, float(r["y"]) * k)
    common = [l_ for l_ in refs if l_ in (our_points or {})]
    if len(common) >= 2:
        T = fit_similarity([refs[c] for c in common], [our_points[c] for c in common])
        words.append("their frame -> ours from %d shared points: rotation %.1f deg, scale %.4f%s, %.2f cm RMS at the "
                     "points" % (len(common), math.degrees(math.atan2(T["R"][1, 0], T["R"][0, 0])), T["s"],
                                 ", their axes mirrored against ours" if T["mirror"] else "", T["rms"]))
        if abs(T["s"] - 1.0) > SCALE_WARN:
            words.append("WARNING: the scale is %.3f, not 1: check the units (%s) and the points' measurements"
                         % (T["s"], cfg.get("units", "cm")))
    else:
        T = dict(R=np.eye(2), t=np.zeros(2), s=1.0, mirror=False, rms=float("nan"), resid=np.zeros(0))
        words.append("WARNING: fewer than 2 shared reference points: their coordinates are taken as ours (no "
                     "transform). Give --refs with the stickers measured in their frame")
    # the clock
    off, how, pairs = offset_from_marks(om, their_ev)
    drift = None
    if off is not None:
        offs = [b - a for _, a, b in pairs]
        drift = (max(offs) - min(offs)) if len(offs) > 1 else None
        words.append("their clock minus ours: %+.3f s, from %s%s" % (off, how, "" if drift is None else
                                                                   " (start and end %.3f s apart)" % drift))
        if drift is not None and drift > DRIFT_WARN_S:
            words.append("WARNING: the marks disagree by %.2f s: a clock drifts or a mark was pressed late" % drift)
        off_src = "sync marks"
    else:
        off, med = offset_from_motion(ours, theirs, T, search_s)
        words.append("WARNING: %s: the clock offset was found by lining up the tracks instead: %+.2f s (median gap "
                     "%.1f cm at it). Less sure than marks: a robot standing still gives no timing" % (how, off, med))
        off_src = "track alignment (no marks)"
    # compare
    merged, per = [], {}
    for t, n, x, y, h in theirs:
        tr = ours.get(n)
        if tr is None:
            continue
        p = interp(tr, t - off, max_gap_s)
        if p is None:
            continue
        q = apply(T, [x, y])
        dx, dy = q[0] - p[0], q[1] - p[1]
        dh = ""
        if h is not None and math.isfinite(p[2]):
            dh = ((heading_map(T, h) - p[2] + 180) % 360) - 180
        merged.append([t - off, t, n, p[0], p[1], q[0], q[1], dx, dy, math.hypot(dx, dy), dh])
        per.setdefault(n, []).append((t - off, math.hypot(dx, dy), dx, dy, dh))
    if not merged:
        theirs_names = sorted(set(r[1] for r in theirs))
        words.append("WARNING: no row compared. Their robot names %s; ours %s: map theirs to ours (robot map, e.g. "
                     "%s=%s), or check the clock offset and the time column" % (
                         ", ".join(theirs_names[:6]) or "none", ", ".join(sorted(ours)) or "none",
                         theirs_names[0] if theirs_names else "their-id", sorted(ours)[0] if ours else "JayA"))
    summ = {"clock_offset_s": round(off, 4), "clock_offset_from": off_src, "drift_s": drift,
            "marks": [[w, a, b] for w, a, b in pairs] if pairs else [],
            "transform": {"rotation_deg": round(math.degrees(math.atan2(T["R"][1, 0], T["R"][0, 0])), 3),
                          "scale": round(T["s"], 5), "mirror": T["mirror"], "shift_cm": [round(v, 2) for v in T["t"]],
                          "rms_cm": round(T["rms"], 3) if math.isfinite(T["rms"]) else None,
                          "points": {c: round(float(r), 3) for c, r in zip(common, T["resid"])}},
            "ours_from": {n: v[4] for n, v in ours.items()}, "robots": {}, "words": words,
            "their_rows": len(theirs), "compared_rows": len(merged)}
    for n, lst in per.items():
        d = [v[1] for v in lst]
        s = summarise(d)
        s["bias_cm"] = [round(float(np.mean([v[2] for v in lst])), 2), round(float(np.mean([v[3] for v in lst])), 2)]
        hd = [v[4] for v in lst if v[4] != ""]
        if hd:
            s["heading_median_deg"] = round(float(np.median(np.abs(hd))), 2)
        summ["robots"][n] = s
        words.append("%s: %d rows compared, apart median %.1f cm, p95 %.1f, max %.1f (bias %+.1f, %+.1f cm)"
                     % (n, s["n"], s["median_cm"], s["p95_cm"], s["max_cm"], *s["bias_cm"]))
    if out:
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "merged.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_ours", "t_theirs", "robot", "ours_x", "ours_y", "theirs_x", "theirs_y", "dx", "dy",
                        "apart_cm", "heading_diff_deg"])
            for r in merged:
                w.writerow([("%.4f" % v) if isinstance(v, float) else v for v in r])
        with open(os.path.join(out, "summary.json"), "w") as f:
            json.dump(summ, f, indent=1, default=float)
        if chart and merged:
            make_chart(merged, ours, T, off, os.path.join(out, "compare.png"))
    return summ


def points_of(folder):
    """Our reference points: the run's map spots (mission.json), which came from the site fit's points."""
    for p in (os.path.join(folder, "mission.json"), os.path.join(folder, "tagcal", "points.json")):
        if os.path.exists(p):
            with open(p) as f:
                d = json.load(f)
            if "spots" in d:
                return {k.upper(): tuple(v[:2]) for k, v in d["spots"].items()}
            return {k.upper(): tuple(v["xy"]) for k, v in d.items()}
    return {}


def make_chart(merged, ours, T, off, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = sorted(set(r[2] for r in merged))
    cols = ["#2952CC", "#C2410C", "#7A3EC8", "#12805C"]
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw={"width_ratios": [1.1, 1.4]})
    t0 = min(r[0] for r in merged)
    for k, n in enumerate(names):
        rs = [r for r in merged if r[2] == n]
        axs[0].plot([r[0] - t0 for r in rs], [r[9] for r in rs], ".", ms=2, color=cols[k % 4], label=n)
        tr = ours[n]
        axs[1].plot(tr[1], tr[2], "-", lw=0.8, color=cols[k % 4], label="%s, ours (cameras)" % n)
        axs[1].plot([r[5] for r in rs], [r[6] for r in rs], ".", ms=1.5, color=cols[k % 4], alpha=0.35,
                    label="%s, external (mapped)" % n)
    axs[0].set_xlabel("time since the first compared row (s)")
    axs[0].set_ylabel("distance apart (cm)")
    axs[0].legend(fontsize=8)
    axs[0].grid(alpha=0.3)
    axs[1].set_aspect("equal")
    axs[1].set_xlabel("x (cm, table frame)")
    axs[1].set_ylabel("y (cm)")
    axs[1].legend(fontsize=7, loc="upper right")
    fig.suptitle("Our camera positions against the external recorder's (mapped to our table frame; clock offset "
                 "%+.2f s)" % off, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ------------------------------------------------------------------------------------------------- self-test
def synth_external(ours_folder, out_csv, refs_csv, our_points, offset_s=1503.25, rot_deg=90.0, mirror=True,
                   shift_cm=(250.0, -40.0), units="ft", noise_cm=8.0, outlier_pct=5.0, hz=2.0, marks=True, seed=3,
                   truth_csv=None, our_marks=None):
    """A made-up external recording of the same run: the TRUE positions (simulator truth.csv) in another frame
    (rotated, mirrored, shifted, in feet), on a clock offset_s ahead, with noise and outliers, at hz rows a second,
    and the sync marks as event rows. Their reference points: our sticker spots in their frame (+ 0.5 cm)."""
    rng = np.random.default_rng(seed)
    k = UNITS_CM[units]
    a = math.radians(rot_deg)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])

    def to_theirs(p):
        q = R @ np.asarray(p, float) + shift_cm
        if mirror:
            q = q * [1, -1]
        return q / k
    tru = {}
    with open(truth_csv) as f:
        for r in csv.DictReader(f):
            tru.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "tag", "pos_x_ft", "pos_y_ft", "note"])
        for t, what in (our_marks or []):
            if marks:
                w.writerow(["%.3f" % (t + offset_s), "", "", "", what])
        for n, lst in tru.items():
            lst.sort()
            tt = lst[0][0]
            for t, x, y in lst:
                if t < tt:
                    continue
                tt = t + 1.0 / hz
                p = np.array([x, y]) + rng.normal(0, noise_cm, 2)
                if rng.uniform(0, 100) < outlier_pct:
                    p = p + rng.normal(0, 40, 2)
                q = to_theirs(p)
                w.writerow(["%.3f" % (t + offset_s), "tag-" + n, "%.3f" % q[0], "%.3f" % q[1], ""])
    with open(refs_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "x", "y"])
        for lab, p in our_points.items():
            q = to_theirs(np.asarray(p) + rng.normal(0, 0.5, 2))
            w.writerow([lab, "%.4f" % q[0], "%.4f" % q[1]])


def _selftest(folder):
    import tempfile
    import sync_tools as ST
    d = tempfile.mkdtemp()
    pts = points_of(folder)
    om = [(m["t_unix"], m["what"]) for m in ST.read_marks(folder)]
    cfg = {"time": {"column": "timestamp", "format": "unix"},
           "robot": {"column": "tag", "map": {"tag-JayA": "JayA", "tag-JayB": "JayB"}},
           "x": {"column": "pos_x_ft"}, "y": {"column": "pos_y_ft"}, "units": "ft",
           "event": {"column": "note", "sync_values": ["start", "end"]}}
    ok = True
    for name, kw in (("marks", {}), ("marks missed", {"marks": False})):
        th, rf = os.path.join(d, "theirs.csv"), os.path.join(d, "refs.csv")
        synth_external(folder, th, rf, pts, truth_csv=os.path.join(folder, "run", "sim", "truth.csv"), our_marks=om,
                       **kw)
        s = run(folder, th, cfg, refs_csv=rf, out=os.path.join(d, name.replace(" ", "_")), search_s=1600)
        err = abs(s["clock_offset_s"] - 1503.25)
        print("%-13s offset %+.3f s (true +1503.250, from %s), mirror %s, scale %.4f, %s" % (
            name, s["clock_offset_s"], s["clock_offset_from"], s["transform"]["mirror"], s["transform"]["scale"],
            "; ".join("%s median %.1f p95 %.1f" % (n, r["median_cm"], r["p95_cm"]) for n, r in s["robots"].items())))
        ok = ok and err < (0.3 if name == "marks" else 1.0) and s["transform"]["mirror"] \
            and all(r["median_cm"] < 15 for r in s["robots"].values())
    print("self-test %s (files in %s)" % ("passed" if ok else "FAILED", d))
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ours", help="our mission folder (pose_timeline.csv, sync_marks.jsonl, mission.json)")
    ap.add_argument("--theirs", help="the external recorder's CSV")
    ap.add_argument("--config", help="a JSON column config (see default_config)")
    ap.add_argument("--col", action="append", default=[], metavar="ROLE=COLUMN",
                    help="time=, robot=, x=, y=, heading=, event= (each overrides the config)")
    ap.add_argument("--robot-map", action="append", default=[], metavar="THEIRS=OURS")
    ap.add_argument("--units", choices=sorted(UNITS_CM))
    ap.add_argument("--time-format", choices=["unix", "unix_ms", "unix_us", "iso"])
    ap.add_argument("--refs", help="CSV label,x,y: the reference points (stickers) in their frame and units")
    ap.add_argument("--their-mark", action="append", default=[], metavar="NAME=TIME",
                    help="a sync mark's time on their clock (in their time format), e.g. start=1730561234.5")
    ap.add_argument("--search-s", type=float, default=SEARCH_S)
    ap.add_argument("--prefer", choices=["camera", "robot"], default="camera")
    ap.add_argument("--out", default="")
    ap.add_argument("--selftest", metavar="DRY_MISSION_FOLDER",
                    help="made-up external data from a simulator run's truth, compared (marks, and marks missed)")
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest(a.selftest)
    if not (a.ours and a.theirs):
        print(__doc__)
        return 1
    cfg = default_config()
    if a.config:
        with open(a.config) as f:
            cfg.update(json.load(f))
    for c in a.col:
        role, _, col = c.partition("=")
        if role in ("time", "x", "y"):
            cfg[role] = dict(cfg.get(role) or {}, column=col)
        elif role == "robot":
            cfg["robot"] = dict(cfg.get("robot") or {}, column=col)
        elif role == "heading":
            cfg["heading"] = {"column": col}
        elif role == "event":
            cfg["event"] = dict(cfg.get("event") or {}, column=col)
    for m in a.robot_map:
        k, _, v = m.partition("=")
        cfg.setdefault("robot", {}).setdefault("map", {})[k] = v
    if a.units:
        cfg["units"] = a.units
    if a.time_format:
        cfg["time"]["format"] = a.time_format
    marks = {}
    for m in a.their_mark:
        k, _, v = m.partition("=")
        marks[k] = parse_time(v, cfg["time"].get("format", "unix"))
    s = run(a.ours, a.theirs, cfg, refs_csv=a.refs, their_marks=marks, out=a.out or None, search_s=a.search_s,
            prefer=a.prefer)
    print("\n".join(s["words"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
