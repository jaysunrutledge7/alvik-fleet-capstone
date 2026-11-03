#!/usr/bin/env python3
"""Our own conference-room RFID position model (build e02-nov03-rfid-pipeline, 2026-11-03): windows of the labelled
rows -> features -> a nearest-neighbour fingerprint model, tested two ways, against two simple guesses. numpy only
(the Pi has no scikit-learn).

  windows    per robot tag, every STEP_S, of length L (1 s and 3 s): per antenna the rows read, the sightings
             (tag_seen_count summed), RSSI mean / max / spread, and the phase spread after each (antenna, channel)'s
             own mean is taken off (a still tag keeps its phase on a channel; a moving one does not)
  label      the mean of the rows' poses (our cameras), its ± the largest row ±; "stand" if every row is inside the
             same 3 s stop
  model      k nearest neighbours in feature space (features scaled on the training part only), weighted 1/distance
  baselines  the table's centre; the strongest antenna (the mean training position of windows whose strongest
             antenna was the same)
  tests      zones held out (x 0-39, 39-78, 78-117, 117-156 in: the model never saw that quarter of the table) and points
             held out (5 folds of whole stops: the windows at a point and on the way to it)
"""
import csv
import math
import os

import numpy as np

STEP_S = 0.5             # a window every 0.5 s. No DOE: a sampling rate (more windows, same information)
FILL_DBM = -85.0         # an antenna that did not read the tag in a window: this RSSI (below the weakest read)
K_DEFAULT = 7            # neighbours. DOE in evaluate(): 3 / 7 / 15
FEATS_DEFAULT = "all"    # rssi | rssi+counts | all (rssi + counts + phase spread)
TABLE_CM = (396.24, 167.64)
ZONE_EDGES_CM = tuple(v * 2.54 for v in (0.0, 39.0, 78.0, 117.0, 156.0))
N_FOLDS = 5
STAND_SHARE = 0.6        # a window counts as standing when its centre is in a stop and this share of its rows stood there.
                         # Chosen; DOE note: 0.5 / 0.6 / 0.8 against the standing error (a definition for reporting)


def zone_of(x_cm):
    z = np.searchsorted(np.array(ZONE_EDGES_CM[1:-1]), np.asarray(x_cm), side="right") + 1
    return np.clip(z, 1, 4)


def _circ_spread(ph):
    if len(ph) < 2:
        return 0.0
    return float(1.0 - abs(np.mean(np.exp(1j * ph))))


def make_windows(rows, stands, L, step=STEP_S):
    """[{robot, t, feats{...}, x, y, pm, state, group, n}] for one window length."""
    ants = sorted({q["antenna"] for q in rows})
    # phase with each (robot, antenna, channel)'s circular mean taken off
    by = {}
    for q in rows:
        by.setdefault((q["robot"], q["antenna"], q["channel"]), []).append(q["phase_rad"])
    mean_ph = {k: float(np.angle(np.mean(np.exp(1j * np.array([p for p in v if math.isfinite(p)] or [0.0])))))
               for k, v in by.items()}
    out = []
    for rob in sorted({q["robot"] for q in rows}):
        R = [q for q in rows if q["robot"] == rob and math.isfinite(q["x_cm"])]
        if not R:
            continue
        t = np.array([q["t_pi"] for q in R])
        st = sorted([s for s in stands if s["robot"] == rob], key=lambda s: s["t0"])
        st_t = np.array([s["t0"] for s in st]) if st else np.zeros(0)
        for c in np.arange(t[0] + L / 2, t[-1] - L / 2 + 1e-9, step):
            i0, i1 = np.searchsorted(t, c - L / 2), np.searchsorted(t, c + L / 2)
            if i1 - i0 < 2:
                continue
            W = R[i0:i1]
            f = {}
            for a in ants:
                Q = [q for q in W if q["antenna"] == a]
                rs = np.array([q["rssi_dbm"] for q in Q])
                f["n_%d" % a] = float(len(Q))
                f["seen_%d" % a] = float(sum(q["seen"] for q in Q))
                f["rssi_mean_%d" % a] = float(rs.mean()) if len(rs) else FILL_DBM
                f["rssi_max_%d" % a] = float(rs.max()) if len(rs) else FILL_DBM
                f["rssi_sd_%d" % a] = float(rs.std()) if len(rs) > 1 else 0.0
                ph = np.array([q["phase_rad"] - mean_ph[(rob, a, q["channel"])] for q in Q
                               if math.isfinite(q["phase_rad"])])
                f["phase_spread_%d" % a] = _circ_spread(ph)
            states = {q["state"] for q in W}
            k = np.searchsorted(st_t, c, side="right") - 1
            inside = st[k] if (k >= 0 and st[k]["t0"] <= c <= st[k]["t1"]) else None
            share = (sum(1 for q in W if q["state"] == "stand" and q["point"] == inside["point"]) / float(len(W))
                     if inside else 0.0)
            if inside and share >= STAND_SHARE:
                # centre inside a stop and most rows standing there (a 3 s window seldom fits a 3 s stop exactly)
                state, group = "stand", inside["point"]
            else:
                state = "moving" if "moving" in states else "still"
                k = np.searchsorted(st_t, c)
                group = st[k]["point"] if k < len(st) else "home"
            out.append({"robot": rob, "t": float(c), "feats": f, "x": float(np.mean([q["x_cm"] for q in W])),
                        "y": float(np.mean([q["y_cm"] for q in W])), "pm": float(max(q["pm95_cm"] for q in W)),
                        "state": state, "group": group, "n": len(W)})
    return out, ants


def feature_names(ants, which):
    names = []
    for a in ants:
        names += ["rssi_mean_%d" % a, "rssi_max_%d" % a, "rssi_sd_%d" % a]
        if which in ("rssi+counts", "all"):
            names += ["n_%d" % a, "seen_%d" % a]
        if which == "all":
            names += ["phase_spread_%d" % a]
    return names


def knn_predict(Xtr, Ytr, Xte, k=K_DEFAULT):
    mu, sd = Xtr.mean(0), Xtr.std(0)
    sd[sd < 1e-9] = 1.0
    A, B = (Xtr - mu) / sd, (Xte - mu) / sd
    out = np.zeros((len(B), 2))
    k = min(k, len(A))
    for i0 in range(0, len(B), 512):
        d = np.sqrt(((B[i0:i0 + 512, None, :] - A[None, :, :]) ** 2).sum(-1))
        idx = np.argpartition(d, k - 1, axis=1)[:, :k]
        dd = np.take_along_axis(d, idx, 1)
        w = 1.0 / (dd + 1e-6)
        out[i0:i0 + 512] = (Ytr[idx] * w[:, :, None]).sum(1) / w.sum(1)[:, None]
    return out


def strongest_predict(Xtr_rssi, Ytr, Xte_rssi):
    a_tr, a_te = Xtr_rssi.argmax(1), Xte_rssi.argmax(1)
    centre = np.array([TABLE_CM[0] / 2, TABLE_CM[1] / 2])
    means = {a: Ytr[a_tr == a].mean(0) for a in np.unique(a_tr)}
    return np.array([means.get(a, centre) for a in a_te])


def folds(wins, scheme, seed=3):
    """[(name, test mask)] for 'zones' or 'points'."""
    if scheme == "zones":
        z = zone_of([w["x"] for w in wins])
        return [("zone %d" % k, z == k) for k in (1, 2, 3, 4) if (z == k).any() and (z != k).any()]
    g = np.array([w["group"] for w in wins])
    ug = np.unique(g)
    rng = np.random.default_rng(seed)
    rng.shuffle(ug)
    return [("fold %d" % (i + 1), np.isin(g, ug[i::N_FOLDS])) for i in range(min(N_FOLDS, len(ug)))]


def cross_val(wins, ants, scheme, feats=FEATS_DEFAULT, k=K_DEFAULT):
    """{method: predictions array (n, 2)} with every window predicted once (in the fold that held it out)."""
    names = feature_names(ants, feats)
    X = np.array([[w["feats"][n] for n in names] for w in wins])
    Xr = np.array([[w["feats"]["rssi_mean_%d" % a] for a in ants] for w in wins])
    Y = np.array([[w["x"], w["y"]] for w in wins])
    P = {m: np.full((len(wins), 2), np.nan) for m in ("model", "centre", "strongest antenna")}
    for _, te in folds(wins, scheme):
        tr = ~te
        if tr.sum() < 5:
            continue
        P["model"][te] = knn_predict(X[tr], Y[tr], X[te], k)
        P["centre"][te] = [TABLE_CM[0] / 2, TABLE_CM[1] / 2]
        P["strongest antenna"][te] = strongest_predict(Xr[tr], Y[tr], Xr[te])
    return P, Y


def stats(err):
    err = err[np.isfinite(err)]
    if not len(err):
        return {"n": 0, "median_cm": None, "p90_cm": None}
    return {"n": int(len(err)), "median_cm": float(np.median(err)), "p90_cm": float(np.percentile(err, 90))}


def evaluate(rows, stands, windows=(1.0, 3.0), doe=True, log=print):
    """Windows, both tests, all methods. Returns {"summary": ..., "detail": {L: {...}}, "doe": [...]}."""
    summ, detail = {}, {}
    for L in windows:
        wins, ants = make_windows(rows, stands, L)
        if len(wins) < 20:
            log("  model: %.0f s windows: only %d windows, skipped" % (L, len(wins)))
            continue
        floor = float(np.median([w["pm"] for w in wins]))
        d = {"wins": wins, "ants": ants, "floor": floor, "schemes": {}}
        s_L = {"windows": len(wins), "camera_pm95_median_cm": floor, "schemes": {}}
        for scheme in ("zones", "points"):
            P, Y = cross_val(wins, ants, scheme)
            d["schemes"][scheme] = (P, Y)
            s_L["schemes"][scheme] = {}
            for m, pr in P.items():
                e = np.hypot(pr[:, 0] - Y[:, 0], pr[:, 1] - Y[:, 1])
                st = np.array([w["state"] for w in wins])
                rb = np.array([w["robot"] for w in wins])
                s_L["schemes"][scheme][m] = {"all": stats(e), "stand": stats(e[st == "stand"]),
                                             "moving": stats(e[st == "moving"]),
                                             **{"robot %s" % r: stats(e[rb == r]) for r in np.unique(rb)}}
            mm = s_L["schemes"][scheme]["model"]["all"]
            log("  model: %.0f s windows, %s held out: median %.1f cm, p90 %.1f cm (centre %.1f, strongest antenna "
                "%.1f; camera floor %.1f)" % (L, scheme, mm["median_cm"], mm["p90_cm"],
                                              s_L["schemes"][scheme]["centre"]["all"]["median_cm"],
                                              s_L["schemes"][scheme]["strongest antenna"]["all"]["median_cm"], floor))
        summ["%gs" % L] = s_L
        detail[L] = d
    doe_rows = []
    if doe:
        for L in (1.0, 3.0, 5.0):
            wins, ants = make_windows(rows, stands, L)
            if len(wins) < 20:
                continue
            for feats in ("rssi", "rssi+counts", "all"):
                for k in (3, 7, 15):
                    P, Y = cross_val(wins, ants, "points", feats, k)
                    e = np.hypot(P["model"][:, 0] - Y[:, 0], P["model"][:, 1] - Y[:, 1])
                    doe_rows.append({"window_s": L, "features": feats, "k": k, **stats(e)})
    return {"summary": {"by_window": summ, "doe_points_held_out": doe_rows}, "detail": detail}


def write_windows(path, wins, ants, preds=None):
    names = feature_names(ants, "all")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        extra = sorted(preds) if preds else []
        w.writerow(["robot", "t_pi", "x_cm", "y_cm", "pm95_cm", "state", "group", "rows"] + names +
                   ["%s_x" % e for e in extra] + ["%s_y" % e for e in extra])
        for i, q in enumerate(wins):
            w.writerow([q["robot"], "%.3f" % q["t"], "%.2f" % q["x"], "%.2f" % q["y"], "%.2f" % q["pm"], q["state"],
                        q["group"], q["n"]] + ["%.3f" % q["feats"][n] for n in names] +
                       ["%.2f" % preds[e][i, 0] for e in extra] + ["%.2f" % preds[e][i, 1] for e in extra])
