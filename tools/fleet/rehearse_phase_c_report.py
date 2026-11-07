#!/usr/bin/env python3
"""e10: the Phase C rehearsals' tables and pictures from runs.jsonl (rehearse_phase_c.py).

    python3 tools/fleet/rehearse_phase_c_report.py docs/results/e10-rehearsals-2026-11-07 --png-dir docs/screenshots/2026-11-07
"""
import argparse
import json
import os
import statistics as STS
import sys

BEFORE, AFTER, AFTER2 = "#eb6834", "#2a78d6", "#1baf7a"
INK, MUTED, GRID = "#1a1a19", "#5b6474", "#e4e7ec"


def load(d):
    rows = []
    for line in open(os.path.join(d, "runs.jsonl")):
        r = json.loads(line)
        if "error" not in r:
            rows.append(r)
    return rows


def med(v):
    v = [x for x in v if x is not None]
    return round(STS.median(v), 1) if v else None


def mean_moving(r):
    m = r.get("moving_pct") or {}
    return sum(m.values()) / len(m) if m else None


def case_a(rows):
    out = {}
    for r in rows:
        c = r["case"]
        if not c.startswith("a-"):
            continue
        _, site, fault, when = c.split("-", 3)
        d = ((r.get("phase_c") or {}).get("discovery") or {})
        k = out.setdefault((site, fault, when), dict(n=0, body=[], falls=0, contacts=0, moving=[], found=[], err=[]))
        k["n"] += 1
        k["body"].append(r["edge_min_body_cm"])
        k["falls"] += r["falls"]
        k["contacts"] += r["contacts"]
        k["moving"].append(mean_moving(r))
        k["found"].append("%s/%s" % (d.get("camera"), d.get("calibration")) if d else "-")
        pe = ((r.get("phase_b") or {}).get("pos_err") or {})
        k["err"].append(med([v.get("median") for v in pe.values()]))
    return out


def words_found(site, fault, found):
    f = found[0] if found else ""
    return f


def table_a(out):
    lines = ["| Site | Broken guess | Pipeline | Runs | Found at the start | Closest body to an edge, worst / median (cm) | "
             "Falls | Contacts | Moving (median of runs) | Position error (cm, median) |", "|" + "---|" * 10]
    for (site, fault, when) in sorted(out, key=lambda k: (k[0], k[1], k[2] != "before")):
        k = out[(site, fault, when)]
        fs = sorted(set(k["found"]))
        lines.append("| %s | %s | %s | %d | %s | %.1f / %.1f | %d | %d | %s %% | %s |" % (
            site, fault, when, k["n"], ", ".join(fs) if when == "after" else "-", min(k["body"]),
            STS.median(k["body"]), k["falls"], k["contacts"], med(k["moving"]), med(k["err"])))
    return "\n".join(lines)


def case_b(rows):
    out = {}
    for r in rows:
        if not r["case"].startswith("b-"):
            continue
        k = out.setdefault(r["case"], dict(n=0, body=[], falls=0, contacts=0, moving=[], unseen=[], visits={},
                                           proven=[], probes=[], felt=[]))
        pc = r.get("phase_c") or {}
        k["n"] += 1
        k["body"].append(r["edge_min_body_cm"])
        k["falls"] += r["falls"]
        k["contacts"] += r["contacts"]
        k["moving"].append(mean_moving(r))
        k["unseen"].append(pc.get("unseen_visit_pct"))
        ws = pc.get("workspace") or {}
        k["proven"].append(ws.get("board_proven_pct"))
        k["felt"].append(ws.get("felt_cm2"))
        k["probes"].append((pc.get("counts") or {}).get("probes") or 0)
        for key, v in (pc.get("visits") or {}).items():
            k["visits"][key] = k["visits"].get(key, 0.0) + v
    return out


def case_d(rows):
    out = {}
    for r in rows:
        if not r["case"].startswith("d-"):
            continue
        _, c, src = r["case"].split("-")
        k = out.setdefault((c, src), dict(n=0, body=[], falls=0, contacts=0, worst_mov=[], mov=[], hung=[]))
        k["n"] += 1
        k["body"].append(r["edge_min_body_cm"])
        k["falls"] += r["falls"]
        k["contacts"] += r["contacts"]
        mv = r.get("moving_pct") or {}
        k["worst_mov"].append(min(mv.values()) if mv else None)
        robot = {"a": "Center", "b": "Center", "c": "Right", "d": "Right"}[c]
        k["mov"].append(mv.get(robot))
        k["hung"].append(r.get("hung_s"))
    return out


def case_e(rows):
    out = {}
    for r in rows:
        if not r["case"].startswith(("e-", "e2-")):
            continue
        parts = r["case"].split("-")
        c, meth = (parts[1] if parts[0] == "e" else parts[1] + "-ekf"), parts[2]
        k = out.setdefault((c, meth), dict(n=0, alarms=0, runs_alarm=0, refused=[], tracked=[], body=[], falls=0,
                                           votes=0, bad=0))
        lc = r.get("layer_counts") or {}
        k["n"] += 1
        k["votes"] += lc.get("camera_votes", 0) or 0
        k["bad"] += lc.get("camera_votes_bad", 0) or 0
        a = lc.get("camera_distrusted", 0) or 0
        k["alarms"] += a
        k["runs_alarm"] += int(a > 0)
        k["refused"].append(lc.get("refused_camera", 0) or 0)
        tr = (((r.get("phase_b") or {}).get("tracking") or {}).get("tracked_pct") or {})
        k["tracked"].append(min(tr.values()) if tr else None)
        k["body"].append(r["edge_min_body_cm"])
        k["falls"] += r["falls"]
    return out


def charts(rows, png_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "font.family": "DejaVu Sans"})
    os.makedirs(png_dir, exist_ok=True)
    made = []
    # 1. rehearsal a: the closest body to an edge, every run, before vs after, each broken guess
    A = case_a(rows)
    if A:
        faults = ["size", "mirror", "none", "clock", "dark", "tag", "all"]
        names = {"size": "board size\ntyped 10 % big", "mirror": "camera name\nwrong (mirrored)", "none": "no camera",
                 "clock": "camera clock\n0.7 s ahead", "dark": "a dark\npatch", "tag": "a roof tag\nturned",
                 "all": "all at once"}
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6), sharey=True)
        for ax, site in zip(axes, ("home", "school")):
            fs = [f for f in faults if (site, f, "before") in A]
            for i, f in enumerate(fs):
                for j, (when, col) in enumerate((("before", BEFORE), ("after", AFTER))):
                    v = A.get((site, f, when), {}).get("body", [])
                    ax.scatter([i + (j - 0.5) * 0.28] * len(v), v, s=34, color=col, edgecolor="white", linewidth=1.2,
                               zorder=3, label=("before e10 (no start check, typed board)" if when == "before" else
                                                "e10 (proof drive at the start, proven workspace)") if i == 0 else None)
            ax.axhline(0, color=INK, linewidth=1.0)
            ax.text(len(fs) - 0.5, 0.4, "the board's edge", color=MUTED, fontsize=8.5, ha="right", va="bottom")
            ax.set_xticks(range(len(fs)))
            ax.set_xticklabels([names[f] for f in fs], fontsize=8.5)
            ax.set_title("%s (%s)" % (site.capitalize(), "3 robots, 122 x 122 cm" if site == "home" else
                                      "2 robots, 396 x 168 cm, roof tags"), color=INK, fontsize=11, loc="left")
            ax.grid(axis="y", color=GRID, linewidth=0.8)
            ax.set_axisbelow(True)
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
        axes[0].set_ylabel("closest any robot's body came to an edge (cm)\nbelow 0 = over the edge")
        axes[0].legend(loc="lower left", frameon=False, fontsize=9)
        fig.suptitle("Rehearsal a: a fresh start with the setup described wrong (fast simulator, 4 seeds each, 5 min "
                     "runs; source: docs/results/e10-rehearsals-2026-11-07)", fontsize=10.5, color=INK, x=0.01,
                     ha="left")
        fig.tight_layout()
        p = os.path.join(png_dir, "e10-feature-discovery-before-after.png")
        fig.savefig(p, dpi=130)
        plt.close(fig)
        made.append(p)
    # 2. rehearsal b: where the robots went
    B = case_b(rows)
    if B:
        import numpy as np
        order = [("b-before", "Before e10: the typed board"), ("b-after", "e10: only proven ground"),
                 ("b-after-probe", "e10 + probe ahead")]
        fig, axes = plt.subplots(1, 3, figsize=(13, 4.6))
        vmax = 0
        grids = []
        for key, _ in order:
            g = np.zeros((25, 25))
            for kk, v in (B.get(key, {}).get("visits") or {}).items():
                i, j = (int(q) for q in kk.split(","))
                if 0 <= i < 25 and 0 <= j < 25:
                    g[i, j] += v
            g = g / max(1e-9, g.sum()) * 100.0
            grids.append(g)
            vmax = max(vmax, float(np.percentile(g[g > 0], 98)) if (g > 0).any() else 1)
        from matplotlib.colors import LinearSegmentedColormap
        cm = LinearSegmentedColormap.from_list("blue", ["#ffffff", "#cde2fb", "#6da7ec", "#2a78d6", "#104281"])
        for ax, g, (key, title) in zip(axes, grids, order):
            ax.imshow(g.T, origin="lower", extent=(0, 125, 0, 125), cmap=cm, vmin=0, vmax=vmax)
            ax.add_patch(matplotlib.patches.Rectangle((0, 0), 122.24, 121.92, fill=False, edgecolor=INK, lw=1.2))
            ax.add_patch(matplotlib.patches.Rectangle((85, 0), 37.24, 121.92, facecolor="#5b6474", alpha=0.10,
                                                      edgecolor="#5b6474", lw=0.8, hatch="//"))
            k = B.get(key, {})
            ax.set_title("%s\nin the unseen strip: %s %% of robot time; falls %d" % (
                title, med(k.get("unseen")), k.get("falls", 0)), fontsize=9.5, color=INK, loc="left")
            ax.set_xlim(-2, 125)
            ax.set_ylim(-2, 125)
            ax.set_xlabel("x (cm)")
            ax.text(103.6, 60, "the camera\ncannot see\n(x > 85)", ha="center", va="center", fontsize=8,
                    color=MUTED)
        axes[0].set_ylabel("y (cm)")
        fig.suptitle("Rehearsal b: where the robots' middles spent their time (share of robot time per 5 cm cell, "
                     "darker = more; 4 seeds x 9 min each; fast simulator)", fontsize=10.5, color=INK, x=0.01,
                     ha="left")
        fig.tight_layout()
        p = os.path.join(png_dir, "e10-feature-proven-workspace-where-robots-go.png")
        fig.savefig(p, dpi=130)
        plt.close(fig)
        made.append(p)
    # 3. the fixes: the hung robot with the filter in charge; the camera watch's false alarms
    Dd, E = case_d(rows), case_e(rows)
    if Dd or E:
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.4))
        ax = axes[0]
        cases = [("a", "6 Nov repeat\n(own pose off)"), ("b", "hung at the edge\n(the e09 failure)"),
                 ("c", "heading crawl\nfailed"), ("d", "silent 3 min")]
        for i, (c, nm) in enumerate(cases):
            for j, (src, col, lab) in enumerate((("blend", BEFORE, "the blend in charge (default)"),
                                                 ("ekf", AFTER, "the filter in charge (e10 fixes)"))):
                v = [x for x in Dd.get((c, src), {}).get("mov", []) if x is not None]
                ax.scatter([i + (j - 0.5) * 0.3] * len(v), v, s=30, color=col, edgecolor="white", linewidth=1.1,
                           zorder=3, label=lab if i == 0 else None)
        ax.set_xticks(range(len(cases)))
        ax.set_xticklabels([n for _, n in cases], fontsize=8.5)
        ax.set_ylabel("the case's robot: % of its time moving")
        ax.set_ylim(0, 102)
        ax.set_title("Phase A's cases, 12 seeds each", fontsize=10.5, color=INK, loc="left")
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False, fontsize=8.5)
        ax.grid(axis="y", color=GRID)
        ax.set_axisbelow(True)
        ax = axes[1]
        cs = [("a", "school, tags\n(blend)"), ("b", "home, sunlit strip\n(blend)"),
              ("a-ekf", "school, tags\n(filter in charge)"), ("b-ekf", "home, sunlit strip\n(filter in charge)")]
        for i, (c, nm) in enumerate(cs):
            for j, (meth, col, lab) in enumerate((("e09", BEFORE, "e09's watch"), ("e10", AFTER, "e10's watch"))):
                k = E.get((c, meth), {})
                n, ra = k.get("n", 0), k.get("runs_alarm", 0)
                ax.bar(i + (j - 0.5) * 0.34, ra, width=0.3, color=col, label=lab if i == 0 else None, zorder=3)
                ax.text(i + (j - 0.5) * 0.34, ra + 0.15, "%d of %d" % (ra, n), ha="center", fontsize=8.5, color=INK)
        ax.set_xticks(range(len(cs)))
        ax.set_xticklabels([n for _, n in cs], fontsize=8.5)
        ax.set_ylabel("runs where the camera watch\nwrongly stopped trusting the camera")
        ax.set_title("The camera watch with a RIGHT camera, 12 seeds each", fontsize=10.5, color=INK, loc="left")
        ax.set_ylim(0, 4)
        ax.set_yticks([0, 1, 2, 3, 4])
        ax.legend(loc="upper left", frameon=False, fontsize=8.5)
        ax.grid(axis="y", color=GRID)
        ax.set_axisbelow(True)
        for a_ in axes:
            for s in ("top", "right"):
                a_.spines[s].set_visible(False)
        fig.suptitle("The two e09 failures, before and after their fixes (fast simulator; source: "
                     "docs/results/e10-rehearsals-2026-11-07)", fontsize=10.5, color=INK, x=0.01, ha="left")
        fig.tight_layout()
        p = os.path.join(png_dir, "e10-problem-fixed-hang-and-camera-watch.png")
        fig.savefig(p, dpi=130)
        plt.close(fig)
        made.append(p)
    return made


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--png-dir", default="")
    a = ap.parse_args(argv)
    rows = load(a.folder)
    print("## a\n" + table_a(case_a(rows)))
    print("\n## b")
    for k, v in sorted(case_b(rows).items()):
        print(k, dict(n=v["n"], body_worst=min(v["body"]), body_med=med(v["body"]), falls=v["falls"],
                      contacts=v["contacts"], moving=med(v["moving"]), unseen=med(v["unseen"]),
                      proven=med(v["proven"]), felt=med(v["felt"]), probes=med(v["probes"])))
    print("\n## c")
    for r in rows:
        if r["case"].startswith("c-"):
            pb = r.get("phase_b") or {}
            tr = pb.get("tracking") or {}
            print(r["case"], r["seed"], dict(contacts=r["contacts"], falls=r["falls"], body=r["edge_min_body_cm"],
                                             moving=r.get("moving_pct"), points=pb.get("points"),
                                             far=tr.get("far_tracked_pct"), tracked=tr.get("tracked_pct"),
                                             wiggles=(pb.get("counts") or {}).get("identity_wiggles"),
                                             swaps=tr.get("swaps"), wrong=tr.get("wrong_fixes")))
    print("\n## d")
    for k, v in sorted(case_d(rows).items()):
        print(k, dict(n=v["n"], body_worst=min(v["body"]), body_med=med(v["body"]), falls=v["falls"],
                      contacts=v["contacts"], robot_moving_med=med(v["mov"]), robot_moving_min=min(
                          [x for x in v["mov"] if x is not None] or [None]), worst_robot_min=min(
                          [x for x in v["worst_mov"] if x is not None] or [None])))
    print("\n## e")
    for k, v in sorted(case_e(rows).items()):
        print(k, dict(n=v["n"], runs_alarm=v["runs_alarm"], alarms=v["alarms"], votes=v["votes"], bad=v["bad"], refused_med=med(v["refused"]),
                      tracked_min_med=med(v["tracked"]), body_worst=min(v["body"]), falls=v["falls"]))
    print("\n## f")
    for r in rows:
        if r["case"].startswith("f-"):
            print(r["case"], r["seed"], dict(contacts=r["contacts"], falls=r["falls"], body=r["edge_min_body_cm"],
                                             moving=r.get("moving_pct"), disc=((r.get("phase_c") or {}).get(
                                                 "discovery") or {}).get("say")))
    if a.png_dir:
        print(charts(rows, a.png_dir))


if __name__ == "__main__":
    sys.exit(main())
