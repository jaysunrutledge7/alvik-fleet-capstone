#!/usr/bin/env python3
"""After a school session: line the other team's RFID capture up with our run, label it, and fit our own model
(build e02-nov03-rfid-pipeline, 2026-11-03). ANALYSIS ONLY: nothing here steers a robot (Nick: "never use RFID to
steer this week").

    python3 tools/rfid/school_rfid.py <our session folder> <their rfid csv> [--out <folder>] [--stickers <csv>]
    python3 tools/rfid/school_rfid.py latest ~/rfid-today.csv        # the newest school run on this Pi

Our session folder is the mission's folder (the one with run/, bridge/, pose_timeline.csv). Their CSV is the reader
capture in its own columns (host_ts_unix, reader_ts_us, reader_first_ts_us, epc, antenna_id, channel_index,
rssi_impinj_dbm, phase_deg, tag_seen_count, ...). Everything written goes to --out (default <session>/rfid/: inside
logs/, which git ignores). Nothing from their file is ever written into the repo.

Steps (each in the output's report.md):
  1. load      their rows; the reader's time of each row = the middle of its first and last sighting
               (reader_first_ts_us .. reader_ts_us); host_ts_unix is used only for the clock estimate in 2
  2. clock     reader -> their laptop: the lower envelope of host_ts_unix - reader_ts_us (median of 30 s minima).
               Their laptop -> our Pi: searched by fitting each tag's signal to each robot's camera track at many
               time shifts; the shift where the signal is best explained by position wins (bootstrap over 60 s blocks
               gives its 95 % range). A second, separate estimate lines up "signal changing" with "robot moving".
  3. tags      the two tags whose signal our robots' positions explain best (same shift for both): EPC -> robot.
               Other tags in the room (same prefix, never moving with us) score near zero
  4. labels    each row of the two tags: our pose at its time (camera tags first, else the robot's own pose), the ±
               (95 %) and where it came from; "stand" (inside a 3 s stop at a point), "still" or "moving"
  5. model     windows (1 s and 3 s) of per-antenna RSSI / read counts / phase spread -> our own nearest-neighbour
               model; tested holding out whole zones (x 0-39, 39-78, 78-117, 117-156 in) and holding out points;
               against a table-centre guess and a strongest-antenna guess (rfid_model.py)
  6. stickers  where each robot really stood at each stop (our camera) against the typed sticker spot
  7. report    report.md + charts (rfid_report.py)
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

VERSION = "2026-11-03b"   # b: the sync wiggle as the clock's primary anchor
IN_CM = 2.54
ENV_BLOCK_S = 30.0       # lower envelope: the minimum of host - reader in each 30 s block, then their median. No DOE:
                         # a robust-statistics choice (one late NTP step or a stalled block cannot move the median)
MAX_GAP_S = 0.5          # our camera track is not interpolated across a gap longer than this (as merge_external.py)
SEARCH_S = 10.0          # laptop -> Pi shift searched over +- this (both clocks on NTP); 180 s when the Pi had none
COARSE_STEP_S = 0.5
MOTION_HALF_S = 10.0     # the motion check's own search. Chosen from the simulator sweep (false peaks beyond); no DOE
FINE_HALF_S = 1.5        # fine search +- this round the coarse best, in FINE_STEP_S steps
FINE_STEP_S = 0.05
MIN_ROWS = 40            # a tag read fewer times than this is not a candidate
SYS_FLOOR_S = 0.15       # smallest stated half-range of the clock shift (s). From the simulator sweep (6 runs, build
SYS_FLOOR_NO_FIRST_S = 0.3   # report): errors up to 0.11 s with reader_first_ts_us, 0.17 s without. Measured on fakes
                         # only; DOE note: the real session's sync spin / marks against this estimate
BOOT_N = 60
BOOT_BLOCK_S = 60.0
FIT_INLIER_CM = 10.0     # the frame check uses stops within this of their sticker (a mistyped sticker would turn the fit).
                         # Chosen; no DOE (a robustness cut for a check)


# ---------------------------------------------------------------- 1. their capture
def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def load_capture(path):
    """Their CSV -> dict of numpy arrays. RSSI: rssi_impinj_dbm, else rssi_impinj_raw / 100, else rssi_llrp_dbm.
    Phase (radians): phase_deg, else phase_raw * 2 pi / 4096. Rows with no reader time are counted and dropped."""
    cols = {k: [] for k in ("epc", "ant", "ch", "rssi", "phase", "n", "r_last", "r_first", "host", "rid")}
    dropped = 0
    with open(path, newline="") as fh:
        rd = csv.DictReader(fh)
        need = {"epc", "antenna_id", "reader_ts_us"}
        miss = need - set(rd.fieldnames or [])
        if miss:
            raise SystemExit("%s: missing column(s) %s (is this the reader capture?)" % (path, ", ".join(sorted(miss))))
        for r in rd:
            last = _f(r.get("reader_ts_us"))
            if not math.isfinite(last) or not r.get("epc"):
                dropped += 1
                continue
            rs = _f(r.get("rssi_impinj_dbm"))
            if not math.isfinite(rs):
                rs = _f(r.get("rssi_impinj_raw")) / 100.0
            if not math.isfinite(rs):
                rs = _f(r.get("rssi_llrp_dbm"))
            ph = _f(r.get("phase_deg"))
            ph = math.radians(ph) if math.isfinite(ph) else _f(r.get("phase_raw")) * 2 * math.pi / 4096.0
            cols["epc"].append(r["epc"].strip().upper())
            cols["ant"].append(int(_f(r.get("antenna_id")) if math.isfinite(_f(r.get("antenna_id"))) else -1))
            cols["ch"].append(int(_f(r.get("channel_index"))) if math.isfinite(_f(r.get("channel_index"))) else -1)
            cols["rssi"].append(rs)
            cols["phase"].append(ph)
            n = _f(r.get("tag_seen_count"))
            cols["n"].append(int(n) if math.isfinite(n) else 1)
            cols["r_last"].append(last / 1e6)
            cols["r_first"].append(_f(r.get("reader_first_ts_us")) / 1e6)
            cols["host"].append(_f(r.get("host_ts_unix")))
            cols["rid"].append((r.get("robot_id") or "").strip())
    cap = {k: np.array(v) for k, v in cols.items()}
    if not len(cap["epc"]):
        raise SystemExit("%s: no rows with a reader time" % path)
    first = np.where(np.isfinite(cap["r_first"]), cap["r_first"], cap["r_last"])
    cap["r_mid"] = (first + cap["r_last"]) / 2.0
    cap["span"] = cap["r_last"] - first
    cap["has_first"] = float(np.mean(np.isfinite(cap["r_first"])))
    cap["dropped"] = dropped
    return cap


def lower_envelope(host, reader, block_s=ENV_BLOCK_S):
    """Reader clock -> laptop clock: median over 30 s blocks of min(host - reader). Returns {offset_s, spread_s, drift
    _ms_per_min, blocks}. A row arrives at the laptop after its last sighting, so the minimum is the offset plus the
    shortest delivery time (a few ms)."""
    ok = np.isfinite(host) & np.isfinite(reader)
    d, r = host[ok] - reader[ok], reader[ok]
    if not len(d):
        return {"offset_s": None, "words": "no host_ts_unix column: the laptop clock cannot be estimated"}
    b = np.floor((r - r.min()) / block_s).astype(int)
    mins, mids = [], []
    for k in np.unique(b):
        s = b == k
        if s.sum() >= 5:
            mins.append(d[s].min())
            mids.append(r[s].mean())
    mins, mids = np.array(mins), np.array(mids)
    off = float(np.median(mins)) if len(mins) else float(d.min())
    drift = None
    if len(mins) >= 3:     # Theil-Sen slope (median of pairwise slopes): one stray block cannot tilt it
        i, j = np.triu_indices(len(mins), 1)
        drift = float(np.median((mins[j] - mins[i]) / ((mids[j] - mids[i]) / 60.0)) * 1000.0)
    spread = float(np.percentile(mins, 90) - np.percentile(mins, 10)) if len(mins) >= 3 else None
    return {"offset_s": off, "spread_s": spread, "drift_ms_per_min": drift, "blocks": int(len(mins)),
            "lag_median_s": float(np.median(d) - off)}


# ---------------------------------------------------------------- our session
def find_file(session, name):
    for p in (os.path.join(session, name), os.path.join(session, "run", name)):
        if os.path.exists(p):
            return p
    hits = sorted(glob.glob(os.path.join(session, "**", name), recursive=True))
    return hits[0] if hits else None


def load_poses(session, out):
    """{robot: {"cam": array t,x,y,h,sigma ; "own": array t,x,y,h,sd}} from pose_timeline.csv (made by sync_tools.py
    export if missing; written into --out, never into the session)."""
    p = find_file(session, "pose_timeline.csv")
    if p is None:
        import sync_tools as ST
        p, _ = ST.export_timeline(session, out_csv=os.path.join(out, "pose_timeline.csv"))
    tracks = {}
    with open(p) as f:
        for r in csv.DictReader(f):
            kind = "cam" if r["source"].startswith("camera") else "own"
            try:
                row = (float(r["t_unix"]), float(r["x_cm"]), float(r["y_cm"]), _f(r.get("heading_deg")),
                       _f(r.get("sigma_cm")))
            except ValueError:
                continue
            tracks.setdefault(r["robot"], {"cam": [], "own": []})[kind].append(row)
    for rob in tracks:
        for k in ("cam", "own"):
            a = np.array(sorted(tracks[rob][k])) if tracks[rob][k] else np.zeros((0, 5))
            if len(a):
                _, keep = np.unique(a[:, 0], return_index=True)
                a = a[keep]
            tracks[rob][k] = a
    if not tracks:
        raise SystemExit("%s: no poses (did the run record camera fixes?)" % p)
    return tracks, p


class PoseAt:
    """Our pose at any Pi time: the cameras' track (not across a gap > MAX_GAP_S), else the robot's own pose."""

    def __init__(self, tr):
        self.cam, self.own = tr["cam"], tr["own"]
        c = self.cam
        self.cam_h = np.degrees(np.unwrap(np.radians(np.nan_to_num(c[:, 3])))) if len(c) else c

    def __call__(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        x = np.full(len(t), np.nan)
        y, pm, h = x.copy(), x.copy(), x.copy()
        src = np.zeros(len(t), int)          # 0 none, 1 camera, 2 own pose
        c = self.cam
        if len(c) >= 2:
            i = np.clip(np.searchsorted(c[:, 0], t), 1, len(c) - 1)
            gap = c[i, 0] - c[i - 1, 0]
            ok = (t >= c[0, 0]) & (t <= c[-1, 0]) & (gap <= MAX_GAP_S)
            x[ok] = np.interp(t[ok], c[:, 0], c[:, 1])
            y[ok] = np.interp(t[ok], c[:, 0], c[:, 2])
            h[ok] = np.interp(t[ok], c[:, 0], self.cam_h)
            pm[ok] = 2.0 * np.nan_to_num(np.interp(t[ok], c[:, 0], c[:, 4]), nan=1.0)
            src[ok] = 1
        o = self.own
        if len(o) >= 2:
            need = (src == 0) & (t >= o[0, 0]) & (t <= o[-1, 0])
            i = np.clip(np.searchsorted(o[:, 0], t), 1, len(o) - 1)
            need &= (o[i, 0] - o[i - 1, 0]) <= 2.0
            x[need] = np.interp(t[need], o[:, 0], o[:, 1])
            y[need] = np.interp(t[need], o[:, 0], o[:, 2])
            h[need] = np.interp(t[need], o[:, 0], o[:, 3])
            pm[need] = np.maximum(2.0 * np.nan_to_num(np.interp(t[need], o[:, 0], o[:, 4]), nan=1.0), 1.0)
            src[need] = 2
        return x, y, pm, src, h

    def speed(self, t, dt=0.25):
        """cm/s and deg/s from the track, over +- dt."""
        x1, y1, _, _, h1 = self(np.asarray(t) - dt)
        x2, y2, _, _, h2 = self(np.asarray(t) + dt)
        return np.hypot(x2 - x1, y2 - y1) / (2 * dt), np.abs(h2 - h1) / (2 * dt)


def norm_label(v):
    """'point 7', '7', 'P7', 'P07' -> 'P07'."""
    s = str(v).strip()
    m = re.search(r"(\d+)\s*$", s)
    return "P%02d" % int(m.group(1)) if m else s.upper()


def load_stands(session):
    """[{robot, point, t0, t1, x, y, pm, source}] from run_plan's point_confidence events (the 3 s stops)."""
    out = []
    for p in sorted(glob.glob(os.path.join(session, "**", "coordinator.jsonl"), recursive=True)):
        with open(p) as f:
            for ln in f:
                if '"point_confidence"' not in ln:
                    continue
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                t1 = e.get("t_abs") or e.get("t")
                out.append({"robot": e.get("robot"), "point": norm_label(e.get("point")), "t0": t1 - float(
                    e.get("dwell_s") or 3.0), "t1": t1, "x": e.get("x"), "y": e.get("y"), "pm": e.get("plus_minus_cm"),
                    "source": e.get("source")})
    return sorted(out, key=lambda s: s["t0"])


def clock_words(session):
    p = find_file(session, "sync_marks.jsonl")
    if not p:
        return None, "no sync marks in the session"
    ntp = None
    try:
        with open(p) as f:
            for ln in f:
                c = (json.loads(ln).get("clock") or {})
                if c.get("ntp_synced") is not None:
                    ntp = c.get("ntp_synced")
    except (OSError, ValueError):
        pass
    return ntp, {True: "the Pi's clock was set by the internet (NTP)", False: "the Pi's clock was NOT on NTP",
                 None: "not known whether the Pi's clock was on NTP"}[ntp]


# ---------------------------------------------------------------- 2-3. clock shift and tags
def _dechannel(rssi, ant, ch):
    """RSSI minus its (antenna, channel) mean: removes each channel's own offset (a still tag reads differently on
    each channel), so what is left changes with place."""
    out = rssi.astype(float).copy()
    key = ant * 1000 + ch
    for k in np.unique(key):
        s = key == k
        out[s] -= np.nanmean(rssi[s])
    return out


def explained(t, val, ant, pose, min_rows=20):
    """How much of a tag's signal its robot's position explains, at one clock shift: per antenna, a smooth surface
    (RSSI ~ 1, x, y, x^2, xy, y^2 in metres, plus cos / sin of twice the heading for the tag's own pattern) fitted by
    least squares; score = 1 - sum of squared misses / sum of variance over ALL the tag's rows (a row with no pose at
    this shift counts as unexplained, so a shift with less overlap cannot score better by having fewer rows).
    Returns (score, per-row miss, per-row variance). A tag not on that robot scores about 0.

    (A first version predicted each row from its nearest rows in place, a fingerprint: it failed on the simulator
    because a production run seldom passes the same spot twice, so there were no near rows to predict from.)"""
    x, y, _, _, h = pose(t)
    fin = np.isfinite(val)
    ok = np.isfinite(x) & fin
    var = np.zeros(len(t))
    for a in np.unique(ant[fin]):
        s = fin & (ant == a)
        var[s] = np.var(val[s])
    miss = var.copy()                       # unexplained unless fitted below
    hr = np.radians(np.nan_to_num(h))
    for a in np.unique(ant[ok]):
        s = np.nonzero(ok & (ant == a))[0]
        if len(s) < min_rows:
            continue
        X, Y = x[s] / 100.0, y[s] / 100.0
        D = np.c_[np.ones(len(s)), X, Y, X * X, X * Y, Y * Y, np.cos(2 * hr[s]), np.sin(2 * hr[s])]
        coef, *_ = np.linalg.lstsq(D, val[s], rcond=None)
        miss[s] = (val[s] - D @ coef) ** 2
    tot = var.sum()
    if tot <= 0 or ok.sum() < min_rows:
        return float("nan"), miss, var
    return float(1.0 - miss.sum() / tot), miss, var


def tag_signal(cap, epc):
    s = cap["epc"] == epc
    return s, _dechannel(cap["rssi"][s], cap["ant"][s], cap["ch"][s])


def align(cap, tracks, env_off, search_s=SEARCH_S, log=print, only_epcs=None):
    """Clock shift (laptop -> Pi) and the EPC -> robot mapping, together. Returns a dict with the coarse score table
    (every candidate tag x robot x shift), the chosen pair, the fine curve and its bootstrap range."""
    robots = sorted(tracks)
    poses = {r: PoseAt(tracks[r]) for r in robots}
    epcs, counts = np.unique(cap["epc"], return_counts=True)
    cand = [e for e, n in zip(epcs, counts) if n >= MIN_ROWS and (only_epcs is None or e in only_epcs)]
    lags = np.arange(-search_s, search_s + 1e-9, COARSE_STEP_S)
    t_mac = cap["r_mid"] + env_off
    score = np.full((len(cand), len(robots), len(lags)), np.nan)
    sig = {}
    for i, e in enumerate(cand):
        s, v = tag_signal(cap, e)
        sig[e] = (s, v)
        tt, aa = t_mac[s], cap["ant"][s]
        # 1 s thinning for the coarse pass: one row per antenna per second (fast enough for 30 tags x 2 robots x 41
        # shifts; the fine pass uses every row)
        key = np.floor(tt).astype(np.int64) * 16 + aa
        _, keep = np.unique(key, return_index=True)
        for j, r in enumerate(robots):
            for k, L in enumerate(lags):
                score[i, j, k] = explained(tt[keep] + L, v[keep], aa[keep], poses[r])[0]
    # the pair of tags (one per robot, different tags) and ONE shift that together score best
    best = None
    for k in range(len(lags)):
        S = np.nan_to_num(score[:, :, k], nan=-1.0)
        if len(robots) == 1:
            i = int(np.argmax(S[:, 0]))
            tot, pair = S[i, 0], (i,)
        else:
            tot, pair = -9, None
            for i in range(len(cand)):
                for i2 in range(len(cand)):
                    if i2 != i and S[i, 0] + S[i2, 1] > tot:
                        tot, pair = S[i, 0] + S[i2, 1], (i, i2)
        if best is None or tot > best[0]:
            best = (tot, pair, k)
    tot, pair, kbest = best
    at_edge = kbest in (0, len(lags) - 1)
    if at_edge:
        log("  WARNING: the best clock shift is at the edge of the search (+- %.0f s): the clocks may be further "
            "apart; run again with --search-s 300" % search_s)
    mapping = {cand[pair[j]]: robots[j] for j in range(len(robots))}
    # how sure: each robot's chosen tag against the best other candidate at that shift
    margins = {}
    for j, r in enumerate(robots):
        col = np.nan_to_num(score[:, j, kbest], nan=-1.0)
        others = [col[i] for i in range(len(cand)) if i != pair[j]]
        margins[r] = {"score": float(col[pair[j]]), "next_best": float(max(others)) if others else None,
                      "next_epc": cand[int(np.argsort(col)[-2])] if len(cand) > 1 else None}
    # fine pass: every row of the two tags, both robots summed
    fl = lags[kbest] + np.arange(-FINE_HALF_S, FINE_HALF_S + 1e-9, FINE_STEP_S)
    per = []
    for e, r in mapping.items():
        s, v = sig[e]
        tt, aa = t_mac[s], cap["ant"][s]
        rows = [explained(tt + L, v, aa, poses[r]) for L in fl]
        per.append((tt, rows))
    fine = np.array([1.0 - sum(p[1][q][1].sum() for p in per) / sum(p[1][q][2].sum() for p in per)
                     for q in range(len(fl))])
    kf = int(np.nanargmax(fine))
    # bootstrap over 60 s blocks: resample blocks, re-pick the best shift
    rng = np.random.default_rng(7)
    tall = np.concatenate([p[0] for p in per])
    blk = np.floor((tall - tall.min()) / BOOT_BLOCK_S).astype(int)
    nb = blk.max() + 1
    Mb = np.array([np.bincount(blk, weights=np.concatenate([p[1][q][1] for p in per]), minlength=nb)
                   for q in range(len(fl))])
    Vb = np.array([np.bincount(blk, weights=np.concatenate([p[1][q][2] for p in per]), minlength=nb)
                   for q in range(len(fl))])
    picks = []
    for _ in range(BOOT_N):
        w = np.bincount(rng.integers(0, nb, nb), minlength=nb)
        sc = 1.0 - (Mb @ w) / np.maximum(Vb @ w, 1e-9)
        picks.append(fl[int(np.argmax(sc))])
    lo, hi = np.percentile(picks, [2.5, 97.5])
    shift = float(fl[kf])
    # the bootstrap sees only the noise; on the simulator the answer was also 0.03-0.11 s off with first sightings
    # (0.17 s without), outside a bootstrap range 4 times in 6: so the range is never narrower than this floor
    floor = SYS_FLOOR_S if cap["has_first"] >= 0.9 else SYS_FLOOR_NO_FIRST_S
    lo, hi = min(lo, shift - floor), max(hi, shift + floor)
    ci = max(float(hi - lo) / 2.0, FINE_STEP_S)
    log("  clock: laptop -> Pi shift %+.2f s (95 %% range %+.2f .. %+.2f s)" % (shift, lo, hi))
    return {"candidates": cand, "robots": robots, "lags": lags, "score": score, "mapping": mapping,
            "margins": margins, "coarse_shift_s": float(lags[kbest]), "fine_lags": fl, "fine": fine,
            "shift_s": shift, "shift_lo_s": float(lo), "shift_hi_s": float(hi), "shift_ci_s": ci,
            "boot": [float(p) for p in picks], "at_edge": at_edge}


def motion_check(cap, tracks, mapping, env_off, center_s, half_s=SEARCH_S, step_s=0.05, bin_s=0.25):
    """A separate shift estimate: 'the tag's signal is changing' against 'the robot is moving'. Per 0.5 s bin, the
    mean absolute change of the de-channelled RSSI between a row and the previous row on the same antenna; correlated
    with our speed (cm/s + 0.1 x turn deg/s) over shifts. Returns (shifts, correlation, best shift)."""
    lags = center_s + np.arange(-half_s, half_s + 1e-9, step_s)
    tot = np.zeros(len(lags))
    for e, r in mapping.items():
        s, v = tag_signal(cap, e)
        t = cap["r_mid"][s] + env_off
        a = cap["ant"][s]
        o = np.argsort(t)
        t, a, v = t[o], a[o], v[o]
        ch = np.full(len(t), np.nan)
        tc = t.copy()
        for an in np.unique(a):
            q = np.nonzero(a == an)[0]
            dv = np.abs(np.diff(v[q]))
            dt = np.diff(t[q])
            ch[q[1:]] = np.where(dt < 1.5, dv, np.nan)
            tc[q[1:]] = (t[q[1:]] + t[q[:-1]]) / 2.0      # the change happened between the two rows
        t = tc
        b0 = np.floor(t.min())
        nb = int((t.max() - b0) / bin_s) + 1
        bi = ((t - b0) / bin_s).astype(int)
        ok = np.isfinite(ch)
        sm = np.bincount(bi[ok], weights=ch[ok], minlength=nb)
        cn = np.bincount(bi[ok], minlength=nb)
        m = np.where(cn > 0, sm / np.maximum(cn, 1), np.nan)
        tb = b0 + (np.arange(nb) + 0.5) * bin_s
        pa = PoseAt(tracks[r])
        for k, L in enumerate(lags):
            sp, tn = pa.speed(tb + L)
            mv = sp + 0.1 * tn
            g = np.isfinite(m) & np.isfinite(mv)
            if g.sum() > 20 and np.std(mv[g]) > 0 and np.std(m[g]) > 0:
                tot[k] += np.corrcoef(m[g], mv[g])[0, 1] / len(mapping)
    return lags, tot, float(lags[int(np.argmax(tot))])


# ---------------------------------------------------------------- 2b. the sync wiggle (2026-11-03b)
WIGGLE_PRE_S = 6.0       # the wiggle's window: from this long before its start (the robots stand still at home) ...
WIGGLE_POST_S = 0.5      # ... to this long after its end (the experiments drive off at once). Chosen; DOE note: 3 / 6 /
                         # 10 s against the offset error on fakes
WIGGLE_STEP_S = 0.02
WIGGLE_FLOOR_S = 0.1     # smallest stated half-range of the wiggle's clock shift (s): from the fakes (build report
                         # e02-nov03-experiments-routes-data); DOE note: real wiggle vs the reader's own clock


def load_wiggle(session):
    """{robot: {"start": t, "end": t}} (Pi clock) from the run's summary (mission_run's sync_wiggle), else {}."""
    for p in (os.path.join(session, "mission_summary.json"), os.path.join(session, "run", "experiment.json")):
        try:
            with open(p) as f:
                w = json.load(f).get("sync_wiggle") or {}
        except (OSError, ValueError):
            continue
        w = {r: v for r, v in w.items() if isinstance(v, dict) and "start" in v and "end" in v}
        if w:
            return w
    return {}


def wiggle_anchor(cap, tracks, mapping, env_off, wiggle, center_s=0.0, half_s=SEARCH_S, step_s=WIGGLE_STEP_S):
    """The laptop -> Pi shift from the sync wiggle: both robots turn a quarter turn and back on the spot at a known
    Pi time, so each tag's signal (de-channelled RSSI) changes with the heading only. For each shift, per antenna,
    RSSI ~ 1 + cos 2h + sin 2h (the tag's own pattern; h = the robot's heading from our track) fitted over the rows
    whose shifted time falls in the wiggle's window; score = the share of the rows' variance explained (both robots
    pooled). Returns {"shift_s", "lags", "score", "per_robot": {robot: shift}, "ci_s"} or None."""
    lags = center_s + np.arange(-half_s, half_s + 1e-9, step_s)
    num = np.zeros(len(lags))
    den = np.zeros(len(lags))
    per = {}
    for e, r in mapping.items():
        w = wiggle.get(r)
        if not w or r not in tracks:
            continue
        s, v = tag_signal(cap, e)
        t = cap["r_mid"][s] + env_off
        a = cap["ant"][s]
        pa = PoseAt(tracks[r])
        tr_ = [a_ for a_ in (tracks[r]["cam"], tracks[r]["own"]) if len(a_)]
        first = None
        if tr_:
            t_first = min(a_[0, 0] for a_ in tr_)
            h_first = pa(np.array([t_first + 1e-3]))[4][0]
            first = (t_first, h_first) if np.isfinite(h_first) else None
        w0, w1 = float(w["start"]) - WIGGLE_PRE_S, float(w["end"]) + WIGGLE_POST_S
        rn, rd = np.zeros(len(lags)), np.zeros(len(lags))
        for k, L in enumerate(lags):
            tt = t + L
            sel = (tt >= w0) & (tt <= w1) & np.isfinite(v)
            if sel.sum() < 12:
                continue
            _, _, _, _, h = pa(tt[sel])
            if first is not None:
                # before the robot's first pose of the run it stands at home, as it starts the wiggle (no camera:
                # the own pose starts just before the wiggle)
                h = np.where(~np.isfinite(h) & (tt[sel] < first[0]), first[1], h)
            fin = np.isfinite(h)
            if fin.sum() < 12:
                continue
            hr = np.radians(h[fin])
            vv, aa = v[sel][fin], a[sel][fin]
            for an in np.unique(aa):
                q = aa == an
                if q.sum() < 4:
                    continue
                D = np.c_[np.ones(q.sum()), np.cos(2 * hr[q]), np.sin(2 * hr[q])]
                coef, *_ = np.linalg.lstsq(D, vv[q], rcond=None)
                res = vv[q] - D @ coef
                rn[k] += float(np.sum(res ** 2))
                rd[k] += float(np.sum((vv[q] - vv[q].mean()) ** 2))
        ok = rd > 0
        if ok.any():
            sc = np.where(ok, 1.0 - rn / np.maximum(rd, 1e-12), -np.inf)
            per[r] = float(lags[int(np.argmax(sc))])
        num += rn
        den += rd
    if not per or not (den > 0).any():
        return None
    score = np.where(den > 0, 1.0 - num / np.maximum(den, 1e-12), np.nan)
    best = float(lags[int(np.nanargmax(score))])
    spread = max(abs(v - best) for v in per.values())
    return {"shift_s": best, "lags": lags, "score": score, "per_robot": per,
            "ci_s": max(WIGGLE_FLOOR_S, spread, 2 * step_s)}


# ---------------------------------------------------------------- 4. labels
def label_rows(cap, mapping, tracks, stands, to_pi, ci_s):
    """One labelled row per reading of our two tags: Pi times, pose, ± (95 %), source, state."""
    rows = []
    for e, r in mapping.items():
        s = np.nonzero(cap["epc"] == e)[0]
        pa = PoseAt(tracks[r])
        tf = np.where(np.isfinite(cap["r_first"][s]), cap["r_first"][s], cap["r_last"][s]) + to_pi
        tl = cap["r_last"][s] + to_pi
        tm = (tf + tl) / 2.0
        x, y, pm, src, h = pa(tm)
        sp, _ = pa.speed(tm)
        sp = np.nan_to_num(sp)
        # ± : the camera's own, the move during the row's sightings, the move during the clock's own range
        pm_all = np.sqrt(pm ** 2 + (sp * (tl - tf) / 2.0) ** 2 + (sp * ci_s) ** 2)
        st = [q for q in stands if q["robot"] == r]
        for n, i in enumerate(s):
            state, pt = ("moving" if sp[n] >= 0.5 else "still"), ""
            for q in st:
                if q["t0"] <= tf[n] - ci_s and tl[n] + ci_s <= q["t1"]:
                    state, pt = "stand", q["point"]
                    break
            rows.append({"t_pi": tm[n], "t_first_pi": tf[n], "t_last_pi": tl[n], "robot": r, "epc": e,
                         "antenna": int(cap["ant"][i]), "channel": int(cap["ch"][i]), "rssi_dbm": cap["rssi"][i],
                         "phase_rad": cap["phase"][i], "seen": int(cap["n"][i]), "x_cm": x[n], "y_cm": y[n],
                         "heading_deg": h[n], "pm95_cm": pm_all[n], "source": ("none", "camera", "own pose")[src[n]],
                         "speed_cm_s": sp[n], "state": state, "point": pt})
    rows.sort(key=lambda q: q["t_pi"])
    return rows


LABEL_COLS = ["t_pi", "t_first_pi", "t_last_pi", "robot", "epc", "antenna", "channel", "rssi_dbm", "phase_rad",
              "seen", "x_cm", "y_cm", "heading_deg", "pm95_cm", "source", "speed_cm_s", "state", "point"]


def write_rows(path, rows, cols):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for q in rows:
            w.writerow([("%.4f" % q[c]) if isinstance(q[c], float) else q[c] for c in cols])


# ---------------------------------------------------------------- 6. stickers
def default_stickers(session):
    """The typed sticker sheet: the session's own copy if any, else the school site's points file."""
    for p in (find_file(session, "stickers.csv"), os.path.join(TOOLS, "fleet", "sites", "school-conference-points.csv")):
        if p and os.path.exists(p):
            return p
    return None


def load_stickers(path, units="in"):
    """{label: (x_cm, y_cm)} from a CSV with id/label and x_in,y_in (or x_cm,y_cm, or x,y in --sticker-units)."""
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            lab = r.get("id") or r.get("label") or r.get("point") or ""
            if not lab:
                continue
            if r.get("x_cm") not in (None, ""):
                x, y = float(r["x_cm"]), float(r["y_cm"])
            elif r.get("x_in") not in (None, ""):
                x, y = float(r["x_in"]) * IN_CM, float(r["y_in"]) * IN_CM
            else:
                k = IN_CM if units == "in" else 1.0
                x, y = float(r["x"]) * k, float(r["y"]) * k
            out[norm_label(lab)] = (x, y)
    return out


def site_used_points(site="school-conference"):
    """{label: (x, y)} the site actually sends the robots to (its overrides applied; sticker_points.py), or {}."""
    try:
        import sites as SI
        import sticker_points as SP
        pts, _, _ = SP.points_cm(SI.load(site))
        return {norm_label(k): tuple(v) for k, v in pts.items()}
    except Exception:       # an older checkout without typed points: no override column
        return {}


def sticker_compare(stands, stickers, used=None):
    """Each stop: where our cameras put the robot vs the typed sticker; plus the best-fit shift and turn between the
    two (a frame check: a large one means the frames are not the same)."""
    rows = []
    for q in stands:
        st = stickers.get(q["point"])
        if st is None or q.get("x") is None:
            continue
        rows.append({"robot": q["robot"], "point": q["point"], "sticker_x": st[0], "sticker_y": st[1],
                     "ours_x": q["x"], "ours_y": q["y"], "dist_cm": math.hypot(q["x"] - st[0], q["y"] - st[1]),
                     "dx_cm": q["x"] - st[0], "dy_cm": q["y"] - st[1], "pm95_cm": q.get("pm"),
                     "source": q.get("source"), "site_override": ""})
        u = (used or {}).get(q["point"])
        if u is not None and math.hypot(u[0] - st[0], u[1] - st[1]) > 1.0:
            rows[-1]["site_override"] = "site sends the robot to (%.1f, %.1f) cm" % (u[0], u[1])
    fit = None
    inl = [r for r in rows if r["dist_cm"] <= FIT_INLIER_CM]
    if len(inl) >= 3:
        P = np.array([[r["sticker_x"], r["sticker_y"]] for r in inl])
        Q = np.array([[r["ours_x"], r["ours_y"]] for r in inl])
        pc, qc = P.mean(0), Q.mean(0)
        U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
        R = (U @ Vt).T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = (U @ Vt).T
        tr = qc - R @ pc
        res = np.linalg.norm((R @ P.T).T + tr - Q, axis=1)
        fit = {"turn_deg": float(math.degrees(math.atan2(R[1, 0], R[0, 0]))), "shift_cm": [float(tr[0]), float(tr[1])],
               "mean_offset_cm": [float((Q - P)[:, 0].mean()), float((Q - P)[:, 1].mean())],
               "residual_median_cm": float(np.median(res)), "stops_used": len(inl), "stops_left_out": len(rows) - len(inl)}
    return rows, fit


# ---------------------------------------------------------------- main
def latest_run(site="school-conference"):
    """The newest run folder of the site's Mission Control (logs/mission-control-<site>/runs/) with a run log."""
    base = os.path.join(REPO, "logs", "mission-control-" + site, "runs")
    def has_stops(d):     # a production run (stops at points), not a feel-out or a warm-up
        for p in glob.glob(os.path.join(d, "**", "coordinator.jsonl"), recursive=True):
            with open(p) as f:
                if any('"point_confidence"' in ln for ln in f):
                    return True
        return False
    runs = sorted((d for d in glob.glob(os.path.join(base, "*")) if os.path.isdir(d) and has_stops(d)),
                  key=os.path.getmtime)
    if not runs:
        raise SystemExit("no run with stops at points under %s: give the run's folder instead of 'latest'" % base)
    return runs[-1]


def run(session, capture, out=None, stickers=None, sticker_units="in", search_s=None, windows=(1.0, 3.0),
        charts=True, log=print, doe=True, site="school-conference", use_wiggle=True):
    if session == "latest":
        session = latest_run(site)
        log("our session: %s (the newest run)" % session)
    session = os.path.abspath(session)
    out = os.path.abspath(out or os.path.join(session, "rfid"))
    os.makedirs(out, exist_ok=True)
    import rfid_model as RM
    log("RFID analysis %s (analysis only, nothing steers a robot)" % VERSION)
    cap = load_capture(capture)
    log("  their capture: %d rows, %d tags, reader_first_ts_us on %.0f %% of rows"
        % (len(cap["epc"]), len(np.unique(cap["epc"])), 100 * cap["has_first"]))
    env = lower_envelope(cap["host"], cap["r_last"])
    if env.get("offset_s") is None:
        raise SystemExit(env["words"])
    log("  reader -> laptop: %+.3f s (lower envelope over %d blocks; drift %s ms/min)"
        % (env["offset_s"], env["blocks"], "%.2f" % env["drift_ms_per_min"] if env["drift_ms_per_min"] is not None
           else "?"))
    tracks, pose_file = load_poses(session, out)
    stands = load_stands(session)
    ntp, ntp_words = clock_words(session)
    if search_s is None:
        search_s = SEARCH_S if ntp else 180.0
    log("  ours: %s; %d stands; %s; laptop -> Pi searched over +- %.0f s"
        % (", ".join("%s %d camera fixes" % (r, len(tracks[r]["cam"])) for r in sorted(tracks)), len(stands),
           ntp_words, search_s))
    al = align(cap, tracks, env["offset_s"], search_s=search_s, log=log)
    to_pi = env["offset_s"] + al["shift_s"]
    # the sync wiggle (2026-11-03b): when the run has one, it is the clock's primary anchor; the route fit above is
    # the cross-check (its EPC -> robot mapping is used either way)
    wig = load_wiggle(session) if use_wiggle else {}
    wa = None
    if wig:
        wa = wiggle_anchor(cap, tracks, al["mapping"], env["offset_s"], wig,
                           center_s=0.0 if search_s <= MOTION_HALF_S else al["shift_s"],
                           half_s=min(search_s, MOTION_HALF_S))
    route_shift, route_ci = al["shift_s"], al["shift_ci_s"]
    if wa is not None:
        w_agree = abs(wa["shift_s"] - route_shift) <= max(0.3, 2 * max(route_ci, wa["ci_s"]))
        log("  sync wiggle: laptop -> Pi shift %+.2f s (+- %.2f; per robot %s); the route fit %s (%+.2f s)" % (
            wa["shift_s"], wa["ci_s"], ", ".join("%s %+.2f" % kv for kv in sorted(wa["per_robot"].items())),
            "agrees" if w_agree else "DISAGREES", route_shift))
        al["shift_s"], al["shift_ci_s"] = wa["shift_s"], wa["ci_s"]
        al["shift_lo_s"], al["shift_hi_s"] = wa["shift_s"] - wa["ci_s"], wa["shift_s"] + wa["ci_s"]
        to_pi = env["offset_s"] + al["shift_s"]
    else:
        log("  sync wiggle: %s; the route fit is the clock" % ("none in this run" if not wig else "not usable"))
    # the motion check: on its own over +- 10 s round zero when the clocks are near (NTP); with a wider search it
    # looks +- 10 s round the position fit's answer (on the simulator it found false peaks over +- 180 s)
    mc0 = 0.0 if search_s <= MOTION_HALF_S else al["shift_s"]
    mlags, mcorr, mshift = motion_check(cap, tracks, al["mapping"], env["offset_s"], mc0, half_s=MOTION_HALF_S)
    log("  tags: " + "; ".join("%s -> %s (score %.2f; next best %.2f)" % (e, r, al["margins"][r]["score"],
                                                                          al["margins"][r]["next_best"] or float("nan"))
                               for e, r in al["mapping"].items()))
    agree = abs(mshift - al["shift_s"]) <= max(0.3, 2 * al["shift_ci_s"])
    log("  motion check: %+.2f s (%s the position fit)" % (mshift, "agrees with" if agree else "DISAGREES with"))
    rows = label_rows(cap, al["mapping"], tracks, stands, to_pi, al["shift_ci_s"])
    write_rows(os.path.join(out, "labelled_rows.csv"), rows, LABEL_COLS)
    res = RM.evaluate(rows, stands, windows=windows, doe=doe, log=log)
    sp = stickers or default_stickers(session)
    st_rows, st_fit = ([], None)
    if sp:
        st_rows, st_fit = sticker_compare(stands, load_stickers(sp, sticker_units), site_used_points(site))
        if st_rows:
            write_rows(os.path.join(out, "stickers_vs_camera.csv"), st_rows,
                       ["robot", "point", "sticker_x", "sticker_y", "ours_x", "ours_y", "dist_cm", "dx_cm", "dy_cm",
                        "pm95_cm", "source", "site_override"])
    summary = {
        "version": VERSION, "session": session, "capture": os.path.abspath(capture), "pose_file": pose_file,
        "rows": int(len(cap["epc"])), "tags_in_capture": int(len(np.unique(cap["epc"]))),
        "rows_dropped_no_reader_time": int(cap["dropped"]), "first_ts_share": cap["has_first"],
        "reader_to_laptop": env, "pi_ntp": ntp, "pi_ntp_words": ntp_words, "search_s": search_s,
        "laptop_to_pi_shift_s": al["shift_s"], "shift_95_s": [al["shift_lo_s"], al["shift_hi_s"]],
        "reader_to_pi_s": to_pi, "motion_check_shift_s": mshift, "motion_check_independent": mc0 == 0.0, "motion_check_agrees": bool(agree),
        "clock_anchor": "sync wiggle" if wa is not None else "route fit",
        "route_fit_shift_s": route_shift, "route_fit_ci_s": route_ci,
        "wiggle_shift_s": wa["shift_s"] if wa else None, "wiggle_ci_s": wa["ci_s"] if wa else None,
        "wiggle_per_robot_s": wa["per_robot"] if wa else None,
        "wiggle_agrees_with_route_fit": (abs(wa["shift_s"] - route_shift) <= max(0.3, 2 * max(route_ci, wa["ci_s"])))
        if wa else None,
        "epc_to_robot": al["mapping"], "tag_scores": al["margins"], "shift_at_search_edge": al["at_edge"],
        "their_robot_id": {e: sorted(set(cap["rid"][cap["epc"] == e]) - {""}) for e in al["mapping"]},
        "candidates": len(al["candidates"]), "labelled_rows": len(rows),
        "states": {k: sum(1 for q in rows if q["state"] == k) for k in ("stand", "still", "moving")},
        "model": res["summary"], "stickers_file": sp, "sticker_fit": st_fit,
        "stickers": {"n": len(st_rows), "median_cm": float(np.median([q["dist_cm"] for q in st_rows])) if st_rows
                     else None, "p90_cm": float(np.percentile([q["dist_cm"] for q in st_rows], 90)) if st_rows
                     else None}}
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=float)
    if charts:
        import rfid_report as RR
        RR.write(out, summary, al, (mlags, mcorr), rows, stands, res, st_rows, tracks, cap)
    log("  written: %s (report.md, summary.json, labelled_rows.csv, windows_*.csv, charts)" % out)
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="our session (mission or Mission Control run) folder, or 'latest' for the newest "
                                    "school run")
    ap.add_argument("capture", help="their RFID capture CSV")
    ap.add_argument("--out", help="output folder (default <session>/rfid)")
    ap.add_argument("--stickers", help="typed sticker spots CSV (id, x_in, y_in); default the school site's sheet")
    ap.add_argument("--sticker-units", default="in", choices=("in", "cm"))
    ap.add_argument("--search-s", type=float, help="laptop -> Pi shift searched over +- this (default 10 s with NTP, "
                                                   "180 s without)")
    ap.add_argument("--windows", default="1,3", help="window lengths in s (default 1,3)")
    ap.add_argument("--no-doe", action="store_true", help="skip the window/feature/k sweep")
    ap.add_argument("--no-charts", action="store_true")
    ap.add_argument("--no-wiggle", action="store_true", help="ignore the run's sync wiggle: the route fit is the clock "
                                                             "(the way before 3 Nov)")
    a = ap.parse_args(argv)
    run(a.session, a.capture, a.out, a.stickers, a.sticker_units, a.search_s,
        tuple(float(v) for v in a.windows.split(",")), not a.no_charts, doe=not a.no_doe, use_wiggle=not a.no_wiggle)
    return 0


if __name__ == "__main__":
    sys.exit(main())
