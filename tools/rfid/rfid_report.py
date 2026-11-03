#!/usr/bin/env python3
"""The one-page report and charts of school_rfid.py (build e02-nov03-rfid-pipeline, 2026-11-03). Every chart says
what is measured and where the data comes from, with one plain sentence under it in report.md."""
import math
import os

import numpy as np

import rfid_model as RM

BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
GREY, INK, INK2 = "#9a9893", "#0b0b0b", "#52514e"
ROBOT_C = [BLUE, ORANGE, AQUA, YELLOW]
ANT_C = {1: BLUE, 2: ORANGE, 3: AQUA, 4: YELLOW}
FAR_CM = 5.0     # a stop further than this from its sticker is listed. Chosen: about 4x the camera's ± (1.2 cm); a
                 # reporting line, no DOE
METHOD_C = {"model": BLUE, "strongest antenna": ORANGE, "centre": GREY}


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GREY, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.color": "#e6e5e1", "grid.linewidth": 0.6, "figure.dpi": 110})
    return plt


def _source(fig, text):
    import textwrap
    w = int(fig.get_size_inches()[0] * 17)
    fig.text(0.01, 0.005, "\n".join(textwrap.wrap(text, w)), fontsize=7, color=INK2, ha="left", va="bottom")


def _table(ax, L=RM.TABLE_CM[0], W=RM.TABLE_CM[1]):
    ax.add_patch(__import__("matplotlib.patches", fromlist=["Rectangle"]).Rectangle((0, 0), L, W, fill=False,
                                                                                     ec=GREY, lw=1.2))
    for e in RM.ZONE_EDGES_CM[1:-1]:
        ax.axvline(e, color="#e6e5e1", lw=1, zorder=0)
    ax.set_xlim(-10, L + 10)
    ax.set_ylim(-10, W + 10)
    ax.set_aspect("equal")
    ax.set_xlabel("x along the 156 in edge (cm, table frame)")
    ax.set_ylabel("y (cm)")


def chart_clock(path, summ, al, motion, src):
    plt = _plt()
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.4))
    lags, sc = al["lags"], al["score"]
    rob = al["robots"]
    pair = [al["candidates"].index(e) for e, r in sorted(al["mapping"].items(), key=lambda kv: rob.index(kv[1]))]
    tot = sum(np.nan_to_num(sc[i, j], nan=-1) for j, i in enumerate(pair)) / len(pair)
    axs[0].plot(lags, tot, color=BLUE, lw=2)
    axs[0].axvline(al["coarse_shift_s"], color=INK2, lw=0.8, ls="--")
    axs[0].set_title("Coarse: whole search range", fontsize=9, color=INK)
    axs[0].set_xlabel("laptop -> Pi shift tried (s)")
    axs[0].set_ylabel("share of the two tags' RSSI\nexplained by our robots' positions")
    axs[1].plot(al["fine_lags"], al["fine"], color=BLUE, lw=2)
    axs[1].axvspan(al["shift_lo_s"], al["shift_hi_s"], color=BLUE, alpha=0.12, lw=0)
    axs[1].axvline(al["shift_s"], color=BLUE, lw=0.8, ls="--")
    axs[1].set_title("Fine: position fit (shaded = 95 % range)", fontsize=9, color=INK)
    axs[1].set_xlabel("laptop -> Pi shift tried (s)")
    ml, mc = motion
    near = np.abs(ml - al["shift_s"]) <= 5
    axs[2].plot(ml[near], mc[near], color=ORANGE, lw=2)
    axs[2].axvline(summ["motion_check_shift_s"], color=ORANGE, lw=0.8, ls="--")
    axs[2].axvline(al["shift_s"], color=BLUE, lw=0.8, ls="--")
    axs[2].set_title("Check: signal changing vs robot moving", fontsize=9, color=INK)
    axs[2].set_xlabel("laptop -> Pi shift tried (s)")
    axs[2].set_ylabel("correlation")
    fig.suptitle("Clock: how far the RFID laptop's clock is from the Pi's (reader -> laptop already taken off)",
                 fontsize=10, color=INK, x=0.01, ha="left")
    _source(fig, src)
    fig.tight_layout(rect=(0, 0.08, 1, 0.94))
    fig.savefig(path)
    plt.close(fig)


def chart_tags(path, al, src):
    plt = _plt()
    k = list(al["lags"]).index(al["coarse_shift_s"])
    cand = al["candidates"]
    fig, ax = plt.subplots(figsize=(7.5, 0.32 * len(cand) + 1.4))
    order = np.argsort(np.nanmax(al["score"][:, :, k], axis=1))
    for j, r in enumerate(al["robots"]):
        v = al["score"][order, j, k]
        ax.scatter(v, np.arange(len(cand)) + (j - 0.5) * 0.18, s=34, color=ROBOT_C[j], label=r, zorder=3)
    labs = []
    for i in order:
        e = cand[i]
        labs.append("...%s%s" % (e[-6:], ("  = " + al["mapping"][e]) if e in al["mapping"] else ""))
    ax.set_yticks(np.arange(len(cand)))
    ax.set_yticklabels(labs, fontsize=8)
    ax.axvline(0, color=GREY, lw=0.8)
    ax.set_xlabel("share of the tag's RSSI explained by the robot's position (at the chosen clock shift)")
    ax.legend(title="robot", fontsize=8, frameon=False, loc="lower right")
    ax.set_title("Which tag is on which robot: every tag the reader saw", fontsize=10, color=INK, loc="left")
    _source(fig, src)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def chart_timeline(path, rows, stands, src, span_s=120.0):
    plt = _plt()
    robots = sorted({q["robot"] for q in rows})
    fig, axs = plt.subplots(len(robots), 1, figsize=(11, 2.4 * len(robots)), sharex=False)
    axs = np.atleast_1d(axs)
    for ax, r in zip(axs, robots):
        R = [q for q in rows if q["robot"] == r]
        st = [s for s in stands if s["robot"] == r]
        t0 = st[0]["t0"] - 10 if st else R[0]["t_pi"]
        R = [q for q in R if t0 <= q["t_pi"] <= t0 + span_s]
        for s in st:
            if s["t1"] >= t0 and s["t0"] <= t0 + span_s:
                ax.axvspan(s["t0"] - t0, s["t1"] - t0, color="#e6e5e1", lw=0)
                ax.text((s["t0"] + s["t1"]) / 2 - t0, 1.0, s["point"], transform=ax.get_xaxis_transform(),
                        ha="center", va="bottom", fontsize=6.5, color=INK2)
        for a in sorted({q["antenna"] for q in R}):
            Q = [q for q in R if q["antenna"] == a]
            ax.scatter([q["t_pi"] - t0 for q in Q], [q["rssi_dbm"] for q in Q], s=5, color=ANT_C.get(a, GREY),
                       label="antenna %d" % a)
        ax.set_ylabel("%s tag RSSI (dBm)" % r)
        ax.legend(fontsize=7, frameon=False, ncol=4, loc="lower right", markerscale=2)
    axs[-1].set_xlabel("seconds from 10 s before the robot's first stop (first 2 minutes; Pi clock, after alignment)")
    fig.suptitle("Each RFID row placed on our run: grey = the robot standing 3 s at a point (from our run log)",
                 fontsize=10, color=INK, x=0.01, ha="left")
    _source(fig, src)
    fig.tight_layout(rect=(0, 0.07, 1, 0.95))
    fig.savefig(path)
    plt.close(fig)


def chart_errors(path, res, L, src):
    plt = _plt()
    d = res["detail"][L]
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
    for ax, scheme, title in ((axs[0], "zones", "Whole zones held out (never seen that quarter)"),
                              (axs[1], "points", "Whole points held out (5 folds)")):
        P, Y = d["schemes"][scheme]
        for m in ("model", "strongest antenna", "centre"):
            e = np.sort(np.hypot(P[m][:, 0] - Y[:, 0], P[m][:, 1] - Y[:, 1]))
            e = e[np.isfinite(e)]
            ax.plot(e, np.arange(1, len(e) + 1) / len(e), color=METHOD_C[m], lw=2,
                    label="%s (median %.0f cm)" % (m, np.median(e)))
        ax.axvline(d["floor"], color=INK2, lw=0.8, ls=":")
        ax.text(d["floor"], 0.97, " camera ±%.1f cm" % d["floor"], fontsize=7, color=INK2, va="top")
        ax.set_title(title, fontsize=9, color=INK)
        ax.set_xlabel("position error (cm) against our camera")
        ax.legend(fontsize=7.5, frameon=False, loc="lower right")
    axs[0].set_ylabel("share of %g s windows" % L)
    fig.suptitle("Our RFID model vs two simple guesses, %g s windows" % L, fontsize=10, color=INK, x=0.01, ha="left")
    _source(fig, src)
    fig.tight_layout(rect=(0, 0.08, 1, 0.94))
    fig.savefig(path)
    plt.close(fig)


def chart_map(path, res, L, src):
    plt = _plt()
    d = res["detail"][L]
    P, Y = d["schemes"]["points"]
    wins = d["wins"]
    fig, ax = plt.subplots(figsize=(10, 4.6))
    _table(ax)
    for j, r in enumerate(sorted({w["robot"] for w in wins})):
        groups = {}
        for i, w in enumerate(wins):
            if w["robot"] == r and w["state"] == "stand" and np.isfinite(P["model"][i, 0]):
                groups.setdefault(w["group"], []).append(i)
        for g, ii in groups.items():
            t, p = Y[ii].mean(0), P["model"][ii].mean(0)
            ax.annotate("", xy=p, xytext=t, arrowprops=dict(arrowstyle="->", color=ROBOT_C[j], lw=1.2))
            ax.scatter(*t, s=16, color=ROBOT_C[j], zorder=3)
        ax.scatter([], [], s=16, color=ROBOT_C[j], label="%s: where it stood -> what the model said" % r)
    ax.legend(fontsize=8, frameon=False, loc="upper left", bbox_to_anchor=(0, 1.13), ncol=2)
    ax.set_title("Model's answer at each stop, that point held out (%g s windows)" % L, fontsize=10, color=INK,
                 loc="left", pad=22)
    _source(fig, src)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def chart_stickers(path, st_rows, src, mag=10.0):
    plt = _plt()
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 4.0), gridspec_kw={"width_ratios": [3, 1]})
    _table(ax)
    robots = sorted({q["robot"] for q in st_rows})
    for j, r in enumerate(robots):
        Q = [q for q in st_rows if q["robot"] == r]
        for q in Q:
            far = q["dist_cm"] > FAR_CM          # a far stop is drawn at its true length (x10 would leave the table)
            k = 1.0 if far else mag
            ax.annotate("", xy=(q["sticker_x"] + k * q["dx_cm"], q["sticker_y"] + k * q["dy_cm"]),
                        xytext=(q["sticker_x"], q["sticker_y"]),
                        arrowprops=dict(arrowstyle="->", color=ROBOT_C[j], lw=1.1, ls="--" if far else "-"))
            if far:
                ax.text(q["sticker_x"] + q["dx_cm"] + 3, q["sticker_y"] + q["dy_cm"], "%.0f cm (true length)"
                        % q["dist_cm"], fontsize=6.5, color=INK2, va="center")
            ax.text(q["sticker_x"], q["sticker_y"] - 6, q["point"][1:], fontsize=5.5, color=INK2, ha="center",
                    va="top")
        ax.scatter([q["sticker_x"] for q in Q], [q["sticker_y"] for q in Q], s=10, color=ROBOT_C[j], label=r,
                   zorder=3)
    ax.legend(fontsize=8, frameon=False, loc="upper left", bbox_to_anchor=(0, 1.12), ncol=2)
    fig.suptitle("Typed sticker -> where the robot stood (arrows x%g; dashed = over %g cm, drawn true length)"
                 % (mag, FAR_CM), fontsize=10, color=INK, x=0.01, ha="left")
    d = sorted(st_rows, key=lambda q: q["dist_cm"])
    for j, r in enumerate(robots):
        v = [q["dist_cm"] for q in d if q["robot"] == r]
        ax2.scatter(v, np.full(len(v), j) + np.random.default_rng(j).uniform(-0.2, 0.2, len(v)), s=14,
                    color=ROBOT_C[j])
        pm = [q["pm95_cm"] for q in d if q["robot"] == r and q["pm95_cm"] is not None]
        if pm:
            ax2.plot([0, np.median(pm)], [j + 0.35, j + 0.35], color=INK2, lw=2)
    ax2.set_yticks(range(len(robots)))
    ax2.set_yticklabels(robots)
    ax2.set_xscale("symlog", linthresh=1.0)
    ax2.set_xlim(left=0)
    ax2.set_xlabel("cm off (log above 1); bar = camera ±")
    ax2.set_ylim(-0.6, len(robots) - 0.3)
    ax2.set_title("Each stop", fontsize=9, color=INK)
    _source(fig, src)
    fig.tight_layout(rect=(0, 0.08, 1, 0.93))
    fig.savefig(path)
    plt.close(fig)


def _f(v, fmt="%.1f"):
    return "-" if v is None or (isinstance(v, float) and not math.isfinite(v)) else fmt % v


def write(out, summ, al, motion, rows, stands, res, st_rows, tracks, cap):
    """report.md + charts + windows_<L>s.csv into out."""
    src_cap = "Source: their RFID capture (%s), our run log and camera poses (%s)." % (
        os.path.basename(summ["capture"]), os.path.basename(summ["session"]))
    charts = []
    try:
        chart_clock(os.path.join(out, "clock.png"), summ, al, motion,
                    "Measured: share of each tag's RSSI a smooth surface of our robot's position explains, at each "
                    "clock shift; and the correlation of RSSI change with our speed. " + src_cap)
        chart_tags(os.path.join(out, "tags.png"), al, "Measured: as the clock chart, per tag and robot. " + src_cap)
        chart_timeline(os.path.join(out, "timeline.png"), rows, stands,
                       "Measured: RSSI of every row of the two tags (dots, by antenna); stops from our run log. "
                       + src_cap)
        charts += ["clock", "tags", "timeline"]
    except ImportError:
        pass
    Ls = sorted(res["detail"])
    for L in Ls:
        d = res["detail"][L]
        P, _ = d["schemes"]["points"]
        RM.write_windows(os.path.join(out, "windows_%gs.csv" % L), d["wins"], d["ants"],
                         {"pred_points_held_out": P["model"], "pred_zones_held_out": d["schemes"]["zones"][0]["model"]})
    Lm = 3.0 if 3.0 in Ls else (Ls[-1] if Ls else None)
    if Lm is not None:
        try:
            chart_errors(os.path.join(out, "errors.png"), res, Lm,
                         "Measured: distance between each window's estimate and our camera's position over the window. "
                         + src_cap)
            chart_map(os.path.join(out, "model_map.png"), res, Lm,
                      "Measured: camera position at each 3 s stop and the model's mean estimate there. " + src_cap)
            charts += ["errors", "model_map"]
        except ImportError:
            pass
    if st_rows:
        try:
            chart_stickers(os.path.join(out, "stickers.png"), st_rows,
                           "Measured: our cameras' median position during each 3 s stop against the typed sticker "
                           "spot (%s)." % os.path.basename(summ.get("stickers_file") or "?"))
            charts.append("stickers")
        except ImportError:
            pass
    m = summ["model"]["by_window"]
    L = []
    L.append("# RFID vs our run: %s" % os.path.basename(summ["session"]))
    L.append("")
    L.append("Analysis only (nothing here steers a robot). Made by `tools/rfid/school_rfid.py` %s. "
             "**Claude's conclusion** marks a judgement; the rest is measured from the two files." % summ["version"])
    L.append("")
    L.append("## Premise")
    L.append("")
    L.append("Their reader recorded our two robots' tags on its own clock while our cameras recorded where the robots "
             "were. Line the two up, find which tag is on which robot, label every RFID row with our camera position, "
             "and see how well an RFID-only model of our own can place the robots on this table.")
    L.append("")
    L.append("## Data")
    L.append("")
    lt = summ["reader_to_laptop"]
    L.append("| Item | Value |")
    L.append("|---|---|")
    L.append("| Their capture | %d rows, %d tags (%d read often enough to test); reader_first_ts_us on %.0f %% of rows |"
             % (summ["rows"], summ["tags_in_capture"], summ["candidates"], 100 * summ["first_ts_share"]))
    L.append("| Reader -> their laptop | %+.3f s (lower envelope of host - reader time, %d blocks of 30 s; spread %s "
             "s; drift %s ms/min) |" % (lt["offset_s"], lt["blocks"], _f(lt.get("spread_s"), "%.3f"),
                                         _f(lt.get("drift_ms_per_min"), "%.2f")))
    L.append("| Their laptop -> our Pi | **%+.2f s** (95 %% range %+.2f .. %+.2f s: block bootstrap, never under "
             "+-0.15 s); motion check %+.2f s (%s; %s); %s |" % (
                 summ["laptop_to_pi_shift_s"], summ["shift_95_s"][0], summ["shift_95_s"][1],
                 summ["motion_check_shift_s"], "searched on its own" if summ.get("motion_check_independent")
                 else "searched round the position fit", "agrees" if summ.get("motion_check_agrees") else
                 "**disagrees: check the clock chart**", summ["pi_ntp_words"]))
    L.append("| Reader -> Pi (used) | %+.3f s |" % summ["reader_to_pi_s"])
    for e, r in summ["epc_to_robot"].items():
        t = summ["tag_scores"][r]
        rid = ", ".join((summ.get("their_robot_id") or {}).get(e) or []) or "none"
        L.append("| Tag on %s | `%s` (their robot_id: %s; explained %.2f; best other tag %.2f) |"
                 % (r, e, rid, t["score"], t["next_best"] if t["next_best"] is not None else float("nan")))
    if summ.get("shift_at_search_edge"):
        L.append("| **Warning** | the best clock shift is at the edge of the search: run again with a wider "
                 "--search-s |")
    s = summ["states"]
    L.append("| Labelled rows | %d: %d standing at a point, %d moving, %d still elsewhere |"
             % (summ["labelled_rows"], s["stand"], s["moving"], s["still"]))
    L.append("")
    for name, sent in (("clock", "The left panel's peak is the clock shift where our positions explain the tags' "
                                 "signal best; the middle panel narrows it down, and the right panel checks it a "
                                 "second, separate way (the signal changes when the robot moves)."),
                       ("tags", "Each row is a tag the reader saw; the two tags far to the right of the rest are "
                                "the ones riding on our robots."),
                       ("timeline", "When a robot stands at a point (grey) its tag's signal stays level; between "
                                    "points it changes.")):
        if name in charts:
            L.append("![%s](%s.png)" % (name, name))
            L.append("")
            L.append(sent)
            L.append("")
    L.append("## Model (our own, nearest neighbours on per-antenna RSSI, counts, phase spread)")
    L.append("")
    L.append("| Window | Test | Model median / p90 (cm) | Strongest antenna | Table centre | Camera ± (median) |")
    L.append("|---|---|---|---|---|---|")
    for key, v in m.items():
        for scheme, nm in (("zones", "zones held out"), ("points", "points held out")):
            q = v["schemes"][scheme]
            L.append("| %s | %s | %s / %s | %s / %s | %s / %s | %.1f |" % (
                key, nm, _f(q["model"]["all"]["median_cm"]), _f(q["model"]["all"]["p90_cm"]),
                _f(q["strongest antenna"]["all"]["median_cm"]), _f(q["strongest antenna"]["all"]["p90_cm"]),
                _f(q["centre"]["all"]["median_cm"]), _f(q["centre"]["all"]["p90_cm"]), v["camera_pm95_median_cm"]))
    L.append("")
    if Lm is not None:
        q = m["%gs" % Lm]["schemes"]["points"]["model"]
        L.append("At %g s, points held out: standing %s cm median, moving %s cm; %s." % (
            Lm, _f(q["stand"]["median_cm"]), _f(q["moving"]["median_cm"]),
            "; ".join("%s %s cm" % (k[6:], _f(v["median_cm"])) for k, v in q.items() if k.startswith("robot "))))
        L.append("")
    for name, sent in (("errors", "Each line climbs as more windows fall within that error; further left is "
                                  "better, and the dotted line is how well our camera itself knows the position."),
                       ("model_map", "Each arrow runs from where a robot stood to where the model put it, with that "
                                     "point left out of the training.")):
        if name in charts:
            L.append("![%s](%s.png)" % (name, name))
            L.append("")
            L.append(sent)
            L.append("")
    doe = summ["model"].get("doe_points_held_out") or []
    if doe:
        L.append("**Model DOE (points held out, median / p90 cm)**: window length x features x k neighbours.")
        L.append("")
        L.append("| Window | Features | k=3 | k=7 | k=15 |")
        L.append("|---|---|---|---|---|")
        for w in sorted({d["window_s"] for d in doe}):
            for f in ("rssi", "rssi+counts", "all"):
                cells = []
                for k in (3, 7, 15):
                    d = [x for x in doe if x["window_s"] == w and x["features"] == f and x["k"] == k]
                    cells.append("%s / %s" % (_f(d[0]["median_cm"], "%.0f"), _f(d[0]["p90_cm"], "%.0f")) if d
                                 else "-")
                L.append("| %g s | %s | %s |" % (w, f, " | ".join(cells)))
        L.append("")
    L.append("## Stickers vs where the robots stood")
    L.append("")
    if st_rows:
        st = summ["stickers"]
        fit = summ.get("sticker_fit") or {}
        L.append("%d stops: median %.1f cm, p90 %.1f cm from the typed sticker (%s). Best-fit frame difference: "
                 "turn %s deg, mean offset %s cm over the %s stops within 10 cm (a large one would mean the two frames differ, "
                 "not the robots)." % (st["n"], st["median_cm"], st["p90_cm"], os.path.basename(summ["stickers_file"]),
                                      _f(fit.get("turn_deg"), "%.2f"),
                                      ", ".join("%+.1f" % v for v in fit.get("mean_offset_cm") or []),
                                      fit.get("stops_used", "-")))
        L.append("")
        if "stickers" in charts:
            L.append("![stickers](stickers.png)")
            L.append("")
            L.append("Each arrow points from a typed sticker to where our cameras saw the robot stand, drawn ten "
                     "times longer than it really is.")
            L.append("")
        far = sorted([q for q in st_rows if q["dist_cm"] > FAR_CM], key=lambda q: -q["dist_cm"])
        if far:
            L.append("**Stops more than %g cm from their typed sticker** (check the sticker on the table, the typed "
                     "number, or the site's override):" % FAR_CM)
            L.append("")
            L.append("| Point | Robot | Off (cm) | dx, dy (cm) | Camera ± | Note |")
            L.append("|---|---|---|---|---|---|")
            for q in far:
                L.append("| %s | %s | %.1f | %+.1f, %+.1f | %s | %s |" % (q["point"], q["robot"], q["dist_cm"],
                                                                     q["dx_cm"], q["dy_cm"], _f(q["pm95_cm"]),
                                                                     q.get("site_override") or "-"))
        else:
            L.append("Every stop was within %g cm of its typed sticker." % FAR_CM)
    else:
        L.append("No sticker sheet given or no stop matched a sticker label.")
    L.append("")
    L.append("## Files")
    L.append("")
    L.append("`labelled_rows.csv` (every row of the two tags with our pose, ±, source, stand/moving), "
             "`windows_<L>s.csv` (features, labels, held-out predictions), `stickers_vs_camera.csv`, `summary.json`. "
             "Their data stays in this folder; never copy it into the repo.")
    with open(os.path.join(out, "report.md"), "w") as f:
        f.write("\n".join(L) + "\n")
