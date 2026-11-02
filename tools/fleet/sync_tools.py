#!/usr/bin/env python3
"""Recording in step with an external recorder (conference-table build, Nick 2026-11-02). The other team's system
(an external RFID recorder on its own laptop) records the robots' positions at the same time as ours; it stays
separate and unchanged. This module is our side of keeping the two comparable afterwards:

  session id     one short name both teams write on their recordings (Mission Control shows it, big)
  clock status   is this Pi's clock set by the internet (NTP)? how far off might it be? (timedatectl / chronyc)
  sync marks     "start" and "end" (and any extra) marks: the Pi's exact time when Nick presses the button, with the
                 clock status; optionally a robot spins on the spot (a visible event in both recordings)
  pose timeline  our per-robot positions over time in table coordinates (CSV): the cameras' tag poses, and the
                 robots' own poses, each with its uncertainty

Files (in the session's folder): sync_marks.jsonl (one mark a line), pose_timeline.csv.

    python3 tools/fleet/sync_tools.py id                       # a new session id
    python3 tools/fleet/sync_tools.py clock                    # the Pi's clock status in words
    python3 tools/fleet/sync_tools.py mark <folder> start      # a sync mark now
    python3 tools/fleet/sync_tools.py export <mission folder>  # pose_timeline.csv
"""
import csv
import datetime as dt
import json
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))

TIMELINE_COLS = ["t_unix", "t_iso", "robot", "x_cm", "y_cm", "heading_deg", "source", "sigma_cm", "sigma_deg"]


def session_id(site="", when=None):
    """MIRLT-<date>-<time>[-<site>]: short enough to say out loud, unique per minute."""
    t = when or dt.datetime.now()
    s = re.sub(r"[^A-Za-z0-9]", "", str(site or ""))[:10]
    return "MIRLT-%s%s" % (t.strftime("%Y%m%d-%H%M"), ("-" + s) if s else "")


def _run(cmd, timeout=3.0):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def clock_status(run=_run):
    """{ntp_synced: True/False/None, source, offset_ms (chrony's estimate, or None), words}. None = not known here."""
    out = {"ntp_synced": None, "source": "unknown", "offset_ms": None, "t_unix": round(time.time(), 6),
           "t_iso": dt.datetime.now().astimezone().isoformat(timespec="milliseconds")}
    if shutil.which("timedatectl"):
        txt = run(["timedatectl", "show", "-p", "NTPSynchronized", "-p", "NTP", "-p", "Timezone"])
        m = re.search(r"NTPSynchronized=(yes|no)", txt)
        if m:
            out["ntp_synced"] = m.group(1) == "yes"
            out["source"] = "timedatectl"
        tz = re.search(r"Timezone=(\S+)", txt)
        if tz:
            out["timezone"] = tz.group(1)
    if shutil.which("chronyc"):
        txt = run(["chronyc", "tracking"])
        m = re.search(r"System time\s*:\s*([\d.]+) seconds (fast|slow)", txt)
        if m:
            v = float(m.group(1)) * 1000.0
            out["offset_ms"] = round(-v if m.group(2) == "slow" else v, 3)
            out["source"] = "chronyc"
        if re.search(r"Leap status\s*:\s*Normal", txt):
            out["ntp_synced"] = True if out["ntp_synced"] is None else out["ntp_synced"]
    if out["ntp_synced"] is True:
        out["words"] = ("The Pi's clock is set by the internet (NTP)%s: marks are good to a few ms."
                        % ((", %+.1f ms off" % out["offset_ms"]) if out["offset_ms"] is not None else ""))
    elif out["ntp_synced"] is False:
        out["words"] = ("The Pi's clock is NOT set by the internet (no NTP: the room's network may have no internet). "
                        "Its time can be seconds or minutes off: the sync marks are what lines the two recordings up "
                        "(press Start mark and End mark, and the other team writes down their laptop's time at each).")
    else:
        out["words"] = "Not known whether this computer's clock is set by the internet (no timedatectl or chronyc)."
    return out


def mark(folder, what, session=None, extra=None, clock=None):
    """Append a sync mark to <folder>/sync_marks.jsonl and return it."""
    os.makedirs(folder, exist_ok=True)
    t = time.time()
    m = {"kind": "sync_mark", "what": str(what), "t_unix": round(t, 6),
         "t_iso": dt.datetime.fromtimestamp(t).astimezone().isoformat(timespec="milliseconds"),
         "session_id": session, "clock": clock if clock is not None else clock_status()}
    if extra:
        m.update(extra)
    with open(os.path.join(folder, "sync_marks.jsonl"), "a") as f:
        f.write(json.dumps(m) + "\n")
    return m


def read_marks(folder_or_file):
    p = folder_or_file
    if os.path.isdir(p):
        p = os.path.join(p, "sync_marks.jsonl")
    out = []
    try:
        with open(p) as f:
            for ln in f:
                try:
                    out.append(json.loads(ln))
                except ValueError:
                    pass
    except OSError:
        pass
    return out


def spin_events(coordinator_jsonl):
    """The sync spins' times from run_plan's step log: [(robot, t_started, t_done)] of steps labelled 'sync spin'."""
    st, out = {}, []
    try:
        with open(coordinator_jsonl) as f:
            for ln in f:
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("event") == "labelled_step" and "sync spin" in str(e.get("label", "")):
                    k = (e.get("robot"), e.get("label"))
                    if e.get("state") == "started":
                        st[k] = e.get("t")
                    elif e.get("state") == "done" and k in st:
                        out.append((e.get("robot"), st.pop(k), e.get("t")))
    except OSError:
        pass
    return out


def export_timeline(mission_folder, out_csv=None, robots=None):
    """pose_timeline.csv from a mission's folder: the cameras' poses (bridge/fixes.csv from the tag bridge, else
    bridge/track.csv from the lights bridge) and the robots' own (run/follow.csv). Returns (path, counts)."""
    out_csv = out_csv or os.path.join(mission_folder, "pose_timeline.csv")
    rows = []
    fx = os.path.join(mission_folder, "bridge", "fixes.csv")
    tr = os.path.join(mission_folder, "bridge", "track.csv")
    if os.path.exists(fx):
        with open(fx) as f:
            for r in csv.DictReader(f):
                rows.append([float(r["t"]), r["robot"], float(r["x"]), float(r["y"]),
                             float(r["heading"]) if r.get("heading") not in (None, "") else "",
                             "camera tag (%s)" % r.get("cams", ""), float(r.get("sigma_cm") or 1.0),
                             float(r.get("sigma_deg") or 3.0)])
    elif os.path.exists(tr):
        with open(tr) as f:
            for r in csv.DictReader(f):
                if r.get("status") != "seen":
                    continue
                rows.append([float(r["t"]), r["robot"], float(r["x"]), float(r["y"]),
                             float(r["sent_th"]) if r.get("sent_th") else "", "camera (%s)" % (r.get("src") or "overhead"),
                             1.0, ""])
    fo = os.path.join(mission_folder, "run", "follow.csv")
    if os.path.exists(fo):
        with open(fo) as f:
            for r in csv.DictReader(f):
                if r.get("state") in ("nopose",):
                    continue
                try:
                    rows.append([float(r["t_recv"]), r["robot"], float(r["x_cm"]), float(r["y_cm"]),
                                 float(r["th_deg"]), "robot's own pose (wheels, gyro, camera fixes)",
                                 float(r.get("pos_sd_cm") or 0.0), float(r.get("th_sd_deg") or 0.0)])
                except (KeyError, ValueError):
                    pass
    if robots:
        rows = [r for r in rows if r[1] in robots]
    rows.sort(key=lambda r: r[0])
    counts = {}
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(TIMELINE_COLS)
        for r in rows:
            t = r[0]
            w.writerow(["%.4f" % t, dt.datetime.fromtimestamp(t).astimezone().isoformat(timespec="milliseconds"),
                        r[1], "%.2f" % r[2], "%.2f" % r[3], ("%.2f" % r[4]) if r[4] != "" else "", r[5],
                        "%.2f" % r[6] if r[6] != "" else "", "%.2f" % r[7] if r[7] != "" else ""])
            k = (r[1], r[5].split(" (")[0])
            counts["%s, %s" % k] = counts.get("%s, %s" % k, 0) + 1
    return out_csv, counts


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    cmd = argv[0] if argv else ""
    if cmd == "id":
        print(session_id(argv[1] if len(argv) > 1 else ""))
    elif cmd == "clock":
        c = clock_status()
        print(c["words"])
        print(json.dumps(c, indent=1))
    elif cmd == "mark" and len(argv) >= 3:
        print(json.dumps(mark(argv[1], argv[2]), indent=1))
    elif cmd == "export" and len(argv) >= 2:
        p, counts = export_timeline(argv[1])
        print("%s: %s" % (p, json.dumps(counts)))
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
