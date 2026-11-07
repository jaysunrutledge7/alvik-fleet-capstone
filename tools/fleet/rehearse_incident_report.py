#!/usr/bin/env python3
"""(e10) The 7 Nov run 2 incident's rehearsal (rehearse_phase_c.py case g): before (the three switches off) and after,
as a table and one chart. Usage: python3 rehearse_incident_report.py <folder with runs.jsonl> [--png out.png]"""
import argparse
import json
import os
import statistics


def load(folder):
    return [json.loads(l) for l in open(os.path.join(folder, "runs.jsonl")) if l.strip()]


def summary(rows):
    out = {}
    for r in rows:
        c = r.get("case", "")
        if not c.startswith("g-") or "error" in r:
            continue
        d = out.setdefault(c, dict(n=0, body=[], hangs=0, falls=0, contacts=0, moving=[], faults=0, cured=0,
                                   watch=0, reaimed=0, resets=0, held=0, deferred=0))
        d["n"] += 1
        d["body"].append(r["edge_min_body_cm"])
        d["hangs"] += r["hangs"] > 0
        d["falls"] += r["falls"]
        d["contacts"] += r["contacts"]
        d["moving"].append(statistics.median(r["moving_pct"].values()))
        inc = r.get("incident") or {}
        d["faults"] += inc.get("hdg_faults", 0)
        d["cured"] += inc.get("hdg_fault_cured", 0)
        d["watch"] += inc.get("queue_watch", 0)
        d["reaimed"] += inc.get("reaimed", 0)
        d["resets"] += inc.get("queue_heading_resets", 0)
        d["held"] += inc.get("start_held", 0)
        d["deferred"] += inc.get("marks_deferred", 0)
    return out


def table(s):
    lines = ["| Case | Runs | Faults injected | Closest body to an edge, worst / median (cm) | Runs with a hang | Falls | "
             "Contacts | Moving (median) | Queue watch acted (re-aimed / heading re-set) | Held at the start |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for c in sorted(s):
        d = s[c]
        lines.append("| %s | %d | %s | %.1f / %.1f | %d | %d | %d | %.1f %% | %d (%d / %d) | %d |" % (
            c[2:], d["n"], d["faults"] if "heading" in c else "-", min(d["body"]), statistics.median(d["body"]),
            d["hangs"], d["falls"], d["contacts"], statistics.median(d["moving"]), d["watch"], d["reaimed"],
            d["resets"], d["held"]))
    return "\n".join(lines)


def table_h(rows):
    """(Nick 7 Nov) case h: a robot put down near / hung over the edge: always wait (before) or free itself first."""
    out = {}
    for r in rows:
        c = r.get("case", "")
        if not c.startswith("h-") or "error" in r:
            continue
        who = "Right" if "-near-" in c else "Center"
        d = out.setdefault(c, dict(n=0, freed=0, body=[], end=[], hangs_after=0, falls=0, moving=[], asked=0))
        d["n"] += 1
        pa = r.get("phase_a") or {}
        d["freed"] += int(r["moving_pct"].get(who, 0.0) > 5.0)        # it drove on (freed by itself or after a hold)
        d["falls"] += r["falls"]
        d["moving"].append(r["moving_pct"].get(who, 0.0))
        d["asked"] += int((pa.get("recover_holds") or 0) > 0 or ((r.get("incident") or {}).get("start_held") or 0) > 0)
    lines = ["| Case | Runs | Drove on (over 5 % moving) | Held or asked at some point | Falls | The robot's moving time "
             "(median / lowest) |", "|---|---|---|---|---|---|"]
    for c in sorted(out):
        d = out[c]
        lines.append("| %s | %d | %d | %d | %d | %.1f / %.1f %% |" % (c[2:], d["n"], d["freed"], d["asked"], d["falls"],
                                                                   statistics.median(d["moving"]), min(d["moving"])))
    return "\n".join(lines)


def chart(rows, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.6), sharey=True)
    for ax, case, title in ((axs[0], "heading", "A long straight 25 deg off towards an edge,\na piece queued behind it"),
                            (axs[1], "start", "A robot that starts 3 cm from the edge")):
        for k, (when, col) in enumerate((("before", "#d9534f"), ("after", "#1a7f37"))):
            v = sorted((r["seed"], r["edge_min_body_cm"]) for r in rows
                       if r.get("case") == "g-%s-%s" % (case, when) and "error" not in r)
            ax.scatter([s + (k - 0.5) * 0.25 for s, _ in v], [b for _, b in v], s=46, color=col,
                       label="before (the switches off)" if when == "before" else "after (e10)", zorder=3)
        ax.axhline(0, color="#a33", lw=1.6)
        if case == "heading":
            ax.text(0.6, -0.9, "body over the edge below this line", color="#a33", fontsize=9, va="top")
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("seed")
        ax.grid(alpha=0.3)
    axs[0].set_ylabel("closest any robot's body came to an edge (cm)")
    axs[1].legend(loc="center", fontsize=9.5)
    axs[1].text(0.6, -1.3, "after: held where it was put (half its body over), never moved", fontsize=8.5,
                color="#1a7f37", va="bottom")
    axs[1].text(0.6, -3.2, "before: its first moves hung it", fontsize=8.5, color="#d9534f", va="bottom")
    fig.suptitle("The 7 Nov run 2 incident in the simulator: before and after the fix", fontsize=12)
    fig.tight_layout()
    fig.savefig(png, dpi=130)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("folder")
    ap.add_argument("--png", default="")
    a = ap.parse_args()
    rows = load(a.folder)
    s = summary(rows)
    print(table(s))
    print()
    print(table_h(rows))
    if a.png:
        chart(rows, a.png)
        print(a.png)


if __name__ == "__main__":
    main()
