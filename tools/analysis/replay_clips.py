#!/usr/bin/env python3
"""Program's-eye replay (Nick, 2026-11-07): short clips of a free-roam run's interesting moments, drawn with what the
program saw and decided, plus the data a replay page needs to show the program's map beside the video.

Logs and video only: it never contacts a robot, a Pi or a camera.

    python3 tools/analysis/replay_clips.py pick  <session logs folder>                     # list the moments it would pick
    python3 tools/analysis/replay_clips.py make  <session logs folder> --video <folder> [--video <folder>] --out <folder>
        [--top 8] [--at 21:09:11] [--kinds edge,tilt] [--no-video] [--label "6 Nov run 1"]
    python3 tools/analysis/replay_clips.py page  <made folder> [<made folder> ...] --to <page folder>
    python3 tools/analysis/replay_clips.py auto  <session logs folder> [--video <folder>] [--upload]   # after a run

What it does
  1. Picks moments from the logs alone (events.csv, breaks.csv, keepouts.csv, lost.csv, reflex.csv, moves.csv,
     follow.csv, camera_track.csv, edge_judgement.csv, recovery.csv when present): edge breaks, tilt stops, trophy
     breaks, front stops and their keep-outs, the closest passes between robot bodies, camera losses and regains,
     own-pose vs camera disagreements and refused fixes, reflex / dodge / stuck / silent / rejoin, and (e08 / e09 runs)
     "unsure near an edge", recoveries, identity wiggles and "moved while parked". Ranked, the top N kept, spread
     across kinds (at most KIND_CAP of one kind while other kinds wait), never two within MIN_GAP_S.
  2. Finds the overhead video that covers each moment: the full track videos (overhead_track_HHMMSS_*.avi, with the
     per-frame times in frames/overhead/track/HHMMSS_*.csv) or, when the run kept no full video (the default), the
     short event clips (frames/overhead/clips/*.avi + .csv). Without a frame-time CSV the times are spread evenly
     from the file's start (its name) to the next file's start (or its frame count at the measured rate), and the
     result says so.
  3. Maps board cm to pixels by fitting it from the frames ("observe, then trust"): the board's bright quad gives a
     first guess and the orientation (the one that puts the camera fixes on dark robot-sized blobs), then a homography
     is fitted (RANSAC) from robot blobs to the camera fixes at the same picture time. That mapping is for the robot's
     top (the camera fixes are the robots' middles as the tracker reports them); everything drawn in board cm uses it,
     so the program's edges and bands sit where the program believes they are relative to the robots. The fit's
     error, number of points and time lag are written to mapping.json. --calib <pinhole json> or --homography <json>
     override it.
  4. Draws each clip at half speed (H.264 MP4, <= 2 MB, 640 wide): camera fixes on the picture they were TAKEN from
     (green dot: the robot's fix counter rose = used; red X: its rejected counter rose, or the program called the
     robot lost = refused; grey dot: neither), each robot's own belief (a coloured ring, a 2-sigma circle from
     pos_sd, a heading tick), the planned move's path (an arc from its curvature, else the chord), the edge safety
     band, keep-outs and a caption bar (clock time, robot, the log line behind the decision).
  5. Writes moments.json (time, kind, robot, a plain explanation, clip, poster) and moment_<n>.json (the program's
     state sampled 10 times a second over the clip, the log lines, for the page's map and log).

Time base: camera_track.csv's t is the picture's time; a fix arrives about 0.45 s later (its "arrived"). Fixes are
drawn on the frame whose time is nearest t, never on arrival. follow.csv's rows carry t_recv (arrival at the
laptop / Pi, about 0.05-0.1 s after the robot measured it); the robot's own pose is drawn at t_recv.

Explanations are written from the log lines; anything the tool works out itself (a disagreement at plan time, a
minimum gap) says "(worked out from the logs)", and anything it guesses says "(inferred)".

Switch (CLAUDE.md "keep features, switch them off"): the after-run step is on by default; the file
~/.config/mirlt/replay_clips_off (or MIRLT_REPLAY_CLIPS=0) turns it off.
"""
# Changelog
#   2026-11-07a  first version (moments, clips, page data, page, after-run hook)
import argparse
import bisect
import csv
import glob
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time

VERSION = "replay-clips-2026-11-07a"

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

TOP_N = 8
KIND_CAP = 2            # at most this many of one kind while other kinds still have candidates. Chosen (spread); DOE: no
MIN_GAP_S = 20.0        # two moments closer than this become one. Chosen (a clip is ~10 s of real time); DOE: no
PRE_S, POST_S = 4.0, 4.5    # real seconds shown before / after the moment. Chosen; DOE: no (a viewing choice)
MAX_SPAN_S = 10.5       # real seconds per clip at most (21 s at half speed). Chosen (the 8-20 s brief); DOE: no
SLOW = 0.5              # playback speed of the clips (half speed)
OUT_FPS = 12            # output frames a second (each camera picture shows about twice). Not a choice that matters
MAX_BYTES = 2 * 1024 * 1024
BODY_CM = 9.5           # the Alvik's square body (tools/fleet/safety_rules: HALF_DIAG_CM 6.72 = half its diagonal)
POSE_AGREE_CM = 5.0     # observe.POSE_AGREE_CM: own pose vs camera over this = a disagreement (the logs' own limit)
EDGE_BAND_OLD_CM = 16.0  # freeroam_core.EDGE_CC before e08: where a move's end may lie (the band drawn on older runs)
UPLOAD_WAIT_S = 12 * 3600   # how long the after-run step waits for the run's own upload before giving up (sleeping)
CAP_S = 900             # the after-run step stops after this long (soft). Chosen (a 4 h run's step takes ~3-6 min)
ROBOT_COLORS = {"Left": (216, 123, 47), "Center": (143, 60, 210), "Right": (26, 138, 224)}   # BGR
ROBOT_HEX = {"Left": "#2f7bd8", "Center": "#d23c8f", "Right": "#e08a1a"}
OTHER_COLORS = [(160, 160, 30), (60, 160, 60)]

# kind -> (base score, plain label)
KINDS = {
    "edge": (90, "edge: too near to turn"),
    "tilt": (95, "tilt stop / trophy break"),
    "recovery": (70, "recovery"),
    "front": (60, "front stop + keep-out"),
    "keepout": (62, "keep-outs building up"),
    "backout": (64, "backing out of an edge / corner"),
    "pass": (65, "closest pass between robots"),
    "lost": (55, "camera lost and found"),
    "pose": (75, "camera vs own pose"),
    "reflex": (62, "reflex / dodge"),
    "stuck": (58, "stuck / did not move"),
    "silent": (50, "silent / rejoined"),
    "unsure": (52, "unsure near an edge"),
    "identity": (60, "identity wiggle"),
    "parked": (60, "moved while parked"),
}


# =============================================================================================== small helpers
def _f(v, d=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", errors="replace") as fh:
        return list(csv.DictReader(fh))


def clock(t, tz_off_h=None):
    """HH:MM:SS of a Unix time in the run's own time zone (session.json's start carries it)."""
    off = tz_off_h if tz_off_h is not None else -time.timezone / 3600.0
    s = (t + off * 3600.0) % 86400.0
    return "%02d:%02d:%02d" % (int(s // 3600), int(s % 3600 // 60), int(s % 60))


def clock_ms(t, tz):
    return clock(t, tz) + ".%d" % int((t % 1) * 10)


# =============================================================================================== the logs
class Session:
    """One run's logs, loaded once. Times are Unix seconds on the run's computer."""

    def __init__(self, d):
        self.dir = d
        self.name = os.path.basename(os.path.normpath(d))
        self.run = self._json("run.json")
        sj = self._json("session.json")
        self.tz = None
        m = re.search(r"([+-])(\d\d):(\d\d)$", str(sj.get("start", "")))
        if m:
            self.tz = (1 if m.group(1) == "+" else -1) * (int(m.group(2)) + int(m.group(3)) / 60.0)
        mp = self._json("map.json")
        fl = mp.get("floor") or {}
        rect = self.run.get("rect") or [fl.get("x_min", 0.0), fl.get("y_min", 0.0), fl.get("x_max", 122.0),
                                        fl.get("y_max", 122.0)]
        self.rect = [float(v) for v in rect]
        self.robots = list(self.run.get("robots") or [r.get("name") for r in mp.get("robots", [])])
        self.events = []
        for r in read_csv(os.path.join(d, "events.csv")):
            t = _f(r.get("t"))
            if t is not None:
                self.events.append(dict(t=t, robot=r.get("robot", ""), kind=r.get("kind", ""), text=r.get("text", "")))
        self.events.sort(key=lambda e: e["t"])
        self.moves = []
        for r in read_csv(os.path.join(d, "moves.csv")):
            t = _f(r.get("t"))
            if t is None:
                continue
            m = dict(t=t, robot=r.get("robot"), n=r.get("n"), kind=r.get("kind"), turn=_f(r.get("turn_deg"), 0.0),
                     length=_f(r.get("length_cm"), 0.0), speed=_f(r.get("speed")), x0=_f(r.get("x0")),
                     y0=_f(r.get("y0")), x1=_f(r.get("x1")), y1=_f(r.get("y1")), dur=_f(r.get("dur_s"), 0.0),
                     est=_f(r.get("est_s"), 0.0), result=r.get("result", ""), style=r.get("style", ""),
                     k=_f(r.get("curvature")))
            if None in (m["x0"], m["y0"], m["x1"], m["y1"]):
                continue
            self.moves.append(m)
        self.moves.sort(key=lambda m: m["t"])
        self.keepouts = read_csv(os.path.join(d, "keepouts.csv"))
        self.breaks = read_csv(os.path.join(d, "breaks.csv"))
        self.lost = read_csv(os.path.join(d, "lost.csv"))
        self.reflex = read_csv(os.path.join(d, "reflex.csv"))
        self.edgej = read_csv(os.path.join(d, "edge_judgement.csv"))
        self.recovery = read_csv(os.path.join(d, "recovery.csv"))
        self._load_follow()
        self._load_cam()
        self.lost_iv = self._lost_intervals()

    def _json(self, n):
        try:
            with open(os.path.join(self.dir, n)) as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return {}

    def _load_follow(self):
        """follow.csv per robot: sorted columns (t_recv, x, y, th, sd, used, rejected, front, state)."""
        self.own = {}
        p = os.path.join(self.dir, "follow.csv")
        if not os.path.exists(p):
            return
        with open(p, newline="", errors="replace") as fh:
            rd = csv.reader(fh)
            hd = next(rd, None) or []
            ix = {k: i for i, k in enumerate(hd)}
            need = ("robot", "t_recv", "x_cm", "y_cm", "th_deg", "pos_sd_cm", "fixes_used", "fixes_rejected",
                    "front_cm", "state")
            if any(k not in ix for k in need):
                return
            ir, it, ixx, iy, ith, isd, iu, irj, ifr, ist = (ix[k] for k in need)
            for row in rd:
                try:
                    st = row[ist]
                    if st == "nopose":
                        continue
                    o = self.own.setdefault(row[ir], ([], [], [], [], [], [], [], [], []))
                    o[0].append(float(row[it]))
                    o[1].append(float(row[ixx]))
                    o[2].append(float(row[iy]))
                    o[3].append(float(row[ith]))
                    o[4].append(float(row[isd]))
                    o[5].append(float(row[iu]))
                    o[6].append(float(row[irj]))
                    o[7].append(_f(row[ifr], 0.0))
                    o[8].append(st)
                except (ValueError, IndexError):
                    continue
        for n, o in self.own.items():
            order = sorted(range(len(o[0])), key=lambda i: o[0][i])
            self.own[n] = tuple([c[i] for i in order] for c in o)

    def _load_cam(self):
        """camera_track.csv per robot: (picture t, x, y, arrived)."""
        self.cam = {}
        p = os.path.join(self.dir, "camera_track.csv")
        if not os.path.exists(p):
            return
        with open(p, newline="", errors="replace") as fh:
            rd = csv.reader(fh)
            hd = next(rd, None) or []
            ix = {k: i for i, k in enumerate(hd)}
            if not all(k in ix for k in ("t", "robot", "x_cm", "y_cm")):
                return
            ia = ix.get("arrived")
            for row in rd:
                try:
                    t, x, y = float(row[ix["t"]]), float(row[ix["x_cm"]]), float(row[ix["y_cm"]])
                except (ValueError, IndexError):
                    continue
                a = _f(row[ia]) if ia is not None and ia < len(row) else None
                c = self.cam.setdefault(row[ix["robot"]], ([], [], [], []))
                c[0].append(t)
                c[1].append(x)
                c[2].append(y)
                c[3].append(a if a is not None else t)
        for n, c in self.cam.items():
            order = sorted(range(len(c[0])), key=lambda i: c[0][i])
            self.cam[n] = tuple([v[i] for i in order] for v in c)
        if not self.robots:
            self.robots = sorted(set(self.cam) | set(self.own))

    def _lost_intervals(self):
        iv = {}
        for r in self.lost:
            t0, t1 = _f(r.get("t0")), _f(r.get("t1"))
            if t0 is not None and t1 is not None:
                iv.setdefault(r.get("robot"), []).append((t0, t1))
        if not iv:    # no lost.csv: from the events
            op = {}
            for e in self.events:
                if e["kind"] == "lost":
                    op[e["robot"]] = e["t"]
                elif e["kind"] == "seen again" and e["robot"] in op:
                    iv.setdefault(e["robot"], []).append((op.pop(e["robot"]), e["t"]))
        return iv

    # ------------------------------------------------------------------------------------------ queries
    def own_at(self, n, t, max_gap=0.6):
        o = self.own.get(n)
        if not o or not o[0]:
            return None
        i = bisect.bisect_left(o[0], t)
        if i <= 0:
            j = 0
        elif i >= len(o[0]):
            j = len(o[0]) - 1
        else:
            j = i if abs(o[0][i] - t) < abs(o[0][i - 1] - t) else i - 1
            # interpolate position between the two rows around t
            t0, t1 = o[0][i - 1], o[0][i]
            if t1 - t0 < max_gap and t1 > t0:
                a = (t - t0) / (t1 - t0)
                return dict(t=t, x=o[1][i - 1] + a * (o[1][i] - o[1][i - 1]), y=o[2][i - 1] + a * (o[2][i] - o[2][i - 1]),
                            th=o[3][j], sd=o[4][j], used=o[5][j], rej=o[6][j], front=o[7][j], state=o[8][j])
        if abs(o[0][j] - t) > max_gap:
            return None
        return dict(t=o[0][j], x=o[1][j], y=o[2][j], th=o[3][j], sd=o[4][j], used=o[5][j], rej=o[6][j], front=o[7][j],
                    state=o[8][j])

    def cam_near(self, n, t, tol=0.3):
        c = self.cam.get(n)
        if not c or not c[0]:
            return None
        i = bisect.bisect_left(c[0], t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(c[0]) and (best is None or abs(c[0][j] - t) < abs(c[0][best] - t)):
                best = j
        if best is None or abs(c[0][best] - t) > tol:
            return None
        return dict(t=c[0][best], x=c[1][best], y=c[2][best], arrived=c[3][best])

    def cam_between(self, n, t0, t1):
        c = self.cam.get(n)
        if not c:
            return []
        i, j = bisect.bisect_left(c[0], t0), bisect.bisect_right(c[0], t1)
        return [dict(t=c[0][k], x=c[1][k], y=c[2][k], arrived=c[3][k]) for k in range(i, j)]

    def is_lost(self, n, t):
        return any(a <= t <= b for a, b in self.lost_iv.get(n, []))

    def fix_status(self, n, fx):
        """'used' / 'refused' / 'none' for one camera fix, from the robot's own counters: the follow row just before
        the fix arrived against the first row at least 0.25 s after (the robot's reply time). Refused when only the
        rejected counter rose (or the program had the robot lost); used when the used counter rose. Inferred per fix:
        the counters count, they do not name fixes."""
        if self.is_lost(n, fx["t"]):
            return "refused"
        o = self.own.get(n)
        if not o or not o[0]:
            return "none"
        a = fx["arrived"]
        i0 = max(0, bisect.bisect_left(o[0], a) - 1)
        i1 = min(len(o[0]) - 1, bisect.bisect_left(o[0], a + 0.25))
        du, dr = o[5][i1] - o[5][i0], o[6][i1] - o[6][i0]
        if dr > 0 and du <= 0:
            return "refused"
        if du > 0:
            return "used"
        return "none"

    def move_at(self, n, t):
        """The move this robot was running (or had just planned) at t."""
        best = None
        for m in self.moves:
            if m["robot"] != n or m["t"] > t:
                continue
            if m["t"] + max(m["dur"], 0.3) + 0.3 >= t:
                best = m
        return best


# =============================================================================================== path geometry
def move_path(m, n=24):
    """Points (cm) of a planned move: an arc from its curvature (the chord direction is the start heading plus half
    the turn), else the chord."""
    x0, y0, x1, y1 = m["x0"], m["y0"], m["x1"], m["y1"]
    k, turn = m.get("k"), math.radians(m.get("turn") or 0.0)
    L = m.get("length") or math.hypot(x1 - x0, y1 - y0)
    if m.get("kind") == "reverse" or k is None or abs(k) < 1e-4 or m.get("style") == "stopturn":
        return [(x0, y0), (x1, y1)]
    th0 = math.atan2(y1 - y0, x1 - x0) - turn / 2.0
    pts = []
    for i in range(n + 1):
        s = L * i / n
        pts.append((x0 + (math.sin(th0 + k * s) - math.sin(th0)) / k, y0 - (math.cos(th0 + k * s) - math.cos(th0)) / k))
    # the logged end wins if the arc drifts (rounding): shift the tail onto it
    ex, ey = pts[-1][0] - x1, pts[-1][1] - y1
    if math.hypot(ex, ey) > 1.5:
        return [(x0, y0), (x1, y1)]
    return [(px - ex * i / n, py - ey * i / n) for i, (px, py) in enumerate(pts)]


def edge_dist(rect, x, y):
    return min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y)


# =============================================================================================== picking moments
def _cand(t, kind, robot, score, why, extra=None):
    d = dict(t=float(t), kind=kind, robot=robot or "", score=float(score), why=why)
    if extra:
        d.update(extra)
    return d


def closest_passes(S, limit_gap_cm=6.0):
    """Local minima of the gap between two robots' bodies (camera centres minus one body width), at picture times."""
    out = []
    names = [n for n in S.cam if S.cam[n][0]]
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            ca, cb = S.cam[a], S.cam[b]
            best = None
            jb = 0
            for k, t in enumerate(ca[0]):
                while jb + 1 < len(cb[0]) and cb[0][jb + 1] <= t:
                    jb += 1
                if not cb[0] or abs(cb[0][jb] - t) > 0.15:
                    if best:
                        out.append(best)
                        best = None
                    continue
                gap = math.hypot(ca[1][k] - cb[1][jb], ca[2][k] - cb[2][jb]) - BODY_CM
                if gap < limit_gap_cm:
                    if best is None or gap < best[2]:
                        best = (t, (a, b), gap)
                elif best:
                    out.append(best)
                    best = None
            if best:
                out.append(best)
    return out


def pose_disagreements(S, lim=POSE_AGREE_CM, step=3):
    """Spans where the robot's own pose (at the picture's time) and the camera disagree by over lim cm."""
    out = []
    for n, c in S.cam.items():
        span = None
        for k in range(0, len(c[0]), step):
            t = c[0][k]
            if S.is_lost(n, t):
                continue
            o = S.own_at(n, t)
            if o is None:
                continue
            d = math.hypot(o["x"] - c[1][k], o["y"] - c[2][k])
            if d > lim:
                if span is None:
                    span = [t, t, d, t]
                span[1] = t
                if d > span[2]:
                    span[2], span[3] = d, t
            elif span is not None:
                out.append((n, span))
                span = None
        if span:
            out.append((n, span))
    return out


def candidates(S):
    C = []
    ev = S.events
    for e in ev:
        k, txt, n, t = e["kind"], e["text"], e["robot"], e["t"]
        if k == "trophy break":
            kk = "tilt" if "tilt" in txt else ("reflex" if "reflex" in txt else "edge")
            C.append(_cand(t, kk, n, {"tilt": 100, "edge": 86, "reflex": 66}[kk], txt))
        elif k == "tilt stop":
            m = re.search(r"tilted ([\d.]+) deg", txt)
            C.append(_cand(t, "tilt", n, 92 + (float(m.group(1)) if m else 0), txt))
        elif k in ("tilt recovery", "tilt recovered", "hazard"):
            C.append(_cand(t, "recovery", n, 70, txt))
        elif k == "front stop":
            m = re.search(r"obstacle ([\d.]+) cm", txt)
            C.append(_cand(t, "front", n, 60 + (10 - float(m.group(1)) if m else 0), txt))
        elif k in ("dodge", "felt a robot", "wall?"):
            C.append(_cand(t, "reflex", n, 60 if k == "dodge" else 64, txt))
        elif k in ("stuck", "did not move"):
            C.append(_cand(t, "stuck", n, 60 if k == "stuck" else 56, txt))
        elif k in ("silent", "rejoined", "heard again"):
            C.append(_cand(t, "silent", n, 52, txt))
        elif k in ("camera disagrees", "pose re-seeded"):
            m = re.search(r"([\d.]+) cm from the camera", txt)
            C.append(_cand(t, "pose", n, 66 + (min(20.0, float(m.group(1))) if m else 0), txt))
        elif k == "identity wiggle":
            C.append(_cand(t, "identity", n, 62, txt))
        elif k == "moved while parked":
            C.append(_cand(t, "parked", n, 62, txt))
        elif k == "unsure near an edge":
            C.append(_cand(t, "unsure", n, 48, txt))
    for b in S.breaks:
        t = _f(b.get("t"))
        if t is not None and "edge" in (b.get("why") or ""):
            C.append(_cand(t, "edge", b.get("robot"), 85, b.get("why", "")))
    # keep-outs building up: 3 or more found within 60 s
    found = sorted((_f(k.get("t")), k.get("robot")) for k in S.keepouts if "cleared" not in (k.get("what") or "")
                   and _f(k.get("t")) is not None)
    i = 0
    while i < len(found):
        j = i
        while j + 1 < len(found) and found[j + 1][0] - found[i][0] <= 60.0:
            j += 1
        if j - i >= 2:
            grp = found[i:j + 1]
            C.append(_cand(grp[0][0] + 2.0, "keepout", grp[0][1], 58 + 4 * len(grp),
                           "%d keep-outs marked within %.0f s (%s; keepouts.csv)" % (
                               len(grp), grp[-1][0] - grp[0][0], ", ".join(sorted({g[1] for g in grp}))),
                           dict(group=[g[0] for g in grp])))
            i = j + 1
        else:
            i += 1
    # backing out: a reverse / escape that started within 12 cm of an edge and really moved (its camera end > 6 cm away)
    raw = {}
    for r in read_csv(os.path.join(S.dir, "moves.csv")):
        raw[(r.get("robot"), r.get("n"))] = r
    for m in S.moves:
        if m["kind"] not in ("reverse", "escape") or m["dur"] < 1.0:
            continue
        rr = raw.get((m["robot"], m["n"])) or {}
        cx, cy = _f(rr.get("cam_x")), _f(rr.get("cam_y"))
        if cx is None or cy is None:
            continue
        e = edge_dist(S.rect, m["x0"], m["y0"])
        moved = math.hypot(cx - m["x0"], cy - m["y0"])
        if e < 12.0 and moved > 6.0:
            ds = sorted([m["x0"] - S.rect[0], S.rect[2] - m["x0"], m["y0"] - S.rect[1], S.rect[3] - m["y0"]])
            C.append(_cand(m["t"] + 0.5, "backout", m["robot"], 56 + (12 - e) + (8 if ds[1] < 25 else 0),
                           "%s %d (%s) started %.1f cm from the edge%s and moved %.1f cm by the camera" % (
                               m["kind"], int(_f(m["n"], 0)), m["result"], e,
                               " in a corner (%.0f cm from the other side)" % ds[1] if ds[1] < 25 else "", moved)))
    for r in S.reflex:
        t = _f(r.get("t"))
        if t is None:
            continue
        mc = _f(r.get("actual_min_cc"), 99.0)
        C.append(_cand(t, "reflex", r.get("actor"), 58 + max(0.0, 20.0 - mc),
                       "reflex %s against %s (%s): closest %.1f cm centre to centre, %s" % (
                           r.get("kind"), r.get("other"), r.get("method"), mc, r.get("outcome"))))
    for r in S.recovery:
        t = _f(r.get("t"))
        if t is not None:
            C.append(_cand(t, "recovery", r.get("robot"), 72, ", ".join("%s %s" % (k, v) for k, v in r.items()
                                                                          if k not in ("t", "robot") and v)))
    # long camera losses (lost.csv) score by how long and how far it went blind
    for r in S.lost:
        d = _f(r.get("dur_s"), 0.0)
        if d < 2.0:
            continue
        t0 = _f(r.get("t0"))
        C.append(_cand(t0 + min(d, 6.0) / 2.0, "lost", r.get("robot"), 45 + min(30.0, d * 2) + min(10.0, _f(r.get(
            "blind_cm"), 0.0) / 5.0), "camera lost it for %.1f s, %.1f cm blind (lost.csv); found %.1f cm from where "
            "its own wheels said" % (d, _f(r.get("blind_cm"), 0.0), _f(r.get("jump_cm"), 0.0)),
            dict(t_lost=t0, t_found=_f(r.get("t1")))))
    if not S.lost:
        for n, ivs in S.lost_iv.items():
            for a, b in ivs:
                if b - a >= 2.0:
                    C.append(_cand(a + min(b - a, 6.0) / 2, "lost", n, 45 + min(30.0, (b - a) * 2),
                                   "camera lost it for %.1f s (events)" % (b - a), dict(t_lost=a, t_found=b)))
    for t, (a, b), gap in closest_passes(S):
        C.append(_cand(t, "pass", a, 60 + max(0.0, 6.0 - gap) * 4,
                       "the bodies of %s and %s came within %.1f cm of each other (camera; worked out from the logs)" % (
                           a, b, max(gap, 0.0)), dict(other=b, gap_cm=round(gap, 1))))
    for n, (t0, t1, dmax, tmax) in pose_disagreements(S):
        if t1 - t0 < 0.8:
            continue
        # a disagreement lasting minutes is a run gone wrong, not a moment a 10 s clip can show: ranked lower
        C.append(_cand(tmax, "pose", n, 60 + min(25.0, dmax) + min(10.0, t1 - t0) - (20.0 if t1 - t0 > 120 else 0.0),
                       "its own pose and the camera's disagreed by up to %.1f cm for %.1f s (worked out from the logs)"
                       % (dmax, t1 - t0), dict(d_cm=round(dmax, 1), span=[t0, t1])))
    return C


def pick(S, top=TOP_N, kinds=None, at=()):
    """Rank and spread: the best of each kind first (up to KIND_CAP each), then the rest by score; no two within
    MIN_GAP_S (the better one stays and absorbs the other as related). Forced times (--at) come first."""
    C = sorted(candidates(S), key=lambda c: -c["score"])
    if kinds:
        C = [c for c in C if c["kind"] in kinds]
    chosen = []

    def clash(c):
        return any(abs(c["t"] - x["t"]) < MIN_GAP_S for x in chosen)

    for a in at:
        a, want_k, want_r = a if isinstance(a, tuple) else (a, None, None)
        near = [c for c in C if abs(c["t"] - a) < 6.0 and (not want_k or c["kind"] == want_k)
                and (not want_r or c["robot"] == want_r)]
        c = max(near, key=lambda c: c["score"]) if near else _cand(a, "pose", "", 0, "asked for (--at)")
        if not clash(c):
            chosen.append(dict(c, forced=True))
    per = {}
    for c in chosen:
        per[c["kind"]] = per.get(c["kind"], 0) + 1
    for rnd in range(2):
        for c in C:
            if len(chosen) >= top:
                break
            if clash(c):
                continue
            if rnd == 0 and per.get(c["kind"], 0) >= KIND_CAP:
                continue
            if rnd == 0 and per.get(c["kind"], 0) >= 1 and any(
                    k not in per for k in {x["kind"] for x in C if not clash(x)}):
                continue      # first pass: one of each kind before any second
            chosen.append(c)
            per[c["kind"]] = per.get(c["kind"], 0) + 1
        if rnd == 0:
            # second helping per kind (up to KIND_CAP) before the free-for-all
            for c in C:
                if len(chosen) >= top:
                    break
                if not clash(c) and per.get(c["kind"], 0) < KIND_CAP:
                    chosen.append(c)
                    per[c["kind"]] = per.get(c["kind"], 0) + 1
    chosen.sort(key=lambda c: c["t"])
    for c in chosen:
        c["span"] = span_for(S, c)
        c["explain"] = explain(S, c)
        c["label"] = KINDS.get(c["kind"], (0, c["kind"]))[1]
    return chosen


def span_for(S, c):
    """Real-time window shown: PRE_S before to POST_S after, stretched to the move planned just before (when within
    3 s) and the robot's own follow-up events within 6 s, capped at MAX_SPAN_S."""
    t = c["t"]
    a, b = t - PRE_S, t + POST_S
    if c.get("group"):
        a, b = min(a, c["group"][0] - 2.0), max(b, c["group"][-1] + 1.5)
    if c.get("t_lost"):
        a, b = c["t_lost"] - 2.5, min(c["t_found"], c["t_lost"] + 6.0) + 2.5
    if c.get("span") and isinstance(c["span"], list) and len(c["span"]) == 2 and c["kind"] == "pose":
        a, b = min(a, c["span"][0] - 2.0), max(b, c["span"][1] + 1.5)
    m = [x for x in S.moves if x["robot"] == c["robot"] and t - 3.0 <= x["t"] <= t]
    if m:
        a = min(a, m[-1]["t"] - 1.0)
    for e in S.events:
        if e["robot"] == c["robot"] and t < e["t"] <= t + 7.0 and e["kind"] in (
                "pose re-seeded", "did not move", "tilt stop", "keep-out cleared", "step failed", "seen again",
                "tilt recovered", "rejoined"):
            b = max(b, e["t"] + 1.2)
    if b - a > MAX_SPAN_S:
        # keep the moment and as much of the lead-in as fits
        a = max(a, min(t - 1.5, b - MAX_SPAN_S))
        b = a + MAX_SPAN_S
    return [round(a, 2), round(b, 2)]


def explain(S, c):
    """One or two plain sentences: what the program did and why, from the log lines. Numbers the tool works out
    itself are marked "(worked out from the logs)"; guesses are marked "(inferred)"."""
    n, t, k, tz = c["robot"], c["t"], c["kind"], S.tz
    txt = c["why"]
    after = [e for e in S.events if e["robot"] == n and t - 0.3 <= e["t"] <= t + 7.0]
    nxt = [m for m in S.moves if m["robot"] == n and t - 0.2 <= m["t"] <= t + 4.0]

    def then(kinds, nmax=3):
        ev = [e for e in after if e["kind"] in kinds][:nmax]
        return ", then ".join("%s (%s)" % (e["kind"], clock(e["t"], tz)) for e in ev)

    if k == "tilt":
        mv = [x for x in S.moves if x["robot"] == n and t - 4.0 <= x["t"] <= t + 0.2]
        s = ""
        if mv:
            m = mv[-1]
            s = "%s started move %s (a %s turning %+.0f deg over %.0f cm) from its own pose, %.1f cm from the edge" % (
                n, m["n"], "curve" if m["k"] else m["kind"], m["turn"], m["length"], edge_dist(S.rect, m["x0"], m["y0"]))
            fx = S.cam_near(n, m["t"], 0.4)
            if fx:
                d = math.hypot(fx["x"] - m["x0"], fx["y"] - m["y0"])
                s += "; the camera put it %.1f cm from the edge, %.1f cm away (worked out from the logs)%s" % (
                    edge_dist(S.rect, fx["x"], fx["y"]), d,
                    ", under the %.0f cm limit, so nothing acted on it" % POSE_AGREE_CM if d <= POSE_AGREE_CM else "")
            s += ". "
        s += "Its own tilt stop ended the move; then %s." % (then(("trophy break", "tilt stop", "did not move",
                                                                    "pose re-seeded", "tilt recovered")) or "it stood")
        return s
    if k == "edge":
        s = "%s was too near the edge for any planned move to fit, including a turn on the spot (%s)." % (n, txt)
        if nxt:
            m = nxt[0]
            s += " Next it was sent a %s of %.0f cm from (%.1f, %.1f): %s." % (m["kind"], m["length"], m["x0"], m["y0"],
                                                                              m["result"] or "no result logged")
        return s
    if k == "backout":
        return "%s backed away from the edge: %s. The program backs straight out when there is no room to turn." % (
            n, txt)
    if k == "front":
        m = re.search(r"obstacle ([\d.]+) cm", txt)
        s = "%s's own front sensor stopped it %s ahead of an obstacle" % (n, (m.group(1) + " cm") if m else "just")
        dg = [e for e in S.events if e["robot"] == n and t - 2.0 <= e["t"] < t and e["kind"] == "dodge"]
        s += " after it had already tried to curve round it." if dg else "."
        ko = [e for e in after if e["kind"] == "keep-out"]
        if ko:
            s += " The program marked the spot as a keep-out and plans around it from then on."
        return s
    if k == "keepout":
        g = c.get("group") or [t]
        who = sorted({kk.get("robot") for kk in S.keepouts if _f(kk.get("t")) is not None
                      and g[0] - 0.1 <= _f(kk.get("t")) <= g[-1] + 0.1})
        s = ("%d keep-outs went up in %.0f s: the front sensors of %s felt something, and the program marked each spot "
             "to plan around." % (len(g), g[-1] - g[0], " and ".join(who)))
        walls = [e for e in S.events if e["kind"] == "wall?" and g[0] - 1 <= e["t"] <= g[-1] + 10]
        if walls:
            s += " One more felt there would move that side of the board in (a wall?)."
        return s
    if k == "pass":
        react = [e for e in S.events if abs(e["t"] - t) < 3.0 and e["kind"] in ("dodge", "felt a robot", "front stop")]
        rf = [r for r in S.reflex if _f(r.get("t")) is not None and abs(_f(r.get("t")) - t) < 3.0]
        s = "%s and %s passed with %.1f cm between their bodies (camera; worked out from the logs)." % (
            n, c.get("other", "another robot"), max(0.0, c.get("gap_cm", 0.0)))
        if react or rf:
            s += " The program reacted: %s." % ", ".join(sorted({"%s %s" % (e["robot"], e["kind"]) for e in react} |
                                                                {"%s reflex" % r.get("actor") for r in rf}))
        else:
            s += " Nothing in the log reacted to it."
        return s
    if k == "reflex":
        rf = [r for r in S.reflex if _f(r.get("t")) is not None and abs(_f(r.get("t")) - t) < 0.5]
        if rf:
            r = rf[0]
            return ("%s's reflex saw it heading to within %.1f cm of %s and %s; the two came to %.1f cm centre to "
                    "centre (%s)." % (r.get("actor"), _f(r.get("pred_before"), 0.0), r.get("other"),
                               {"speed": ("slowed it (x%.2f)" if _f(r.get("scale"), 1.0) < 1 else "sped it up (x%.2f)") % _f(r.get("scale"), 1.0), "stop": "stopped it",
                                "replan": "dropped the move for a new one", "steer": "steered it"}.get(
                                   r.get("kind"), r.get("kind")), _f(r.get("actual_min_cc"), 0.0), r.get("outcome")))
        m = re.search(r"dodged obstacle ([\d.]+) cm ahead; curving (\w+)", txt)
        if m:
            return ("%s's own front check saw an obstacle %s cm ahead and curved %s round it without stopping; the "
                    "program marked the spot as a keep-out." % (n, m.group(1), m.group(2)))
        return "%s: %s." % (n, txt)
    if k == "lost":
        lo = [r for r in S.lost if r.get("robot") == n and abs(_f(r.get("t0"), 0) - c.get("t_lost", t)) < 0.5]
        if lo:
            r = lo[0]
            s = ("The camera lost %s for %.1f s; the program kept driving it on its own wheels and gyro (%.1f cm blind). "
                 "When the camera found it again it was %.1f cm from where its wheels said." % (
                     n, _f(r.get("dur_s"), 0), _f(r.get("blind_cm"), 0), _f(r.get("jump_cm"), 0)))
        else:
            s = "The camera lost %s and the program drove it on its own wheels and gyro until it was found again." % n
        same = [r for r in S.lost if abs(_f(r.get("t0"), 0) - c.get("t_lost", t)) < 0.6]
        if len(same) >= 2 and len(same) == len(S.robots):
            s += " Every robot was lost at the same moment, so the camera itself paused (inferred)."
        return s
    if k == "pose":
        rs = [e for e in S.events if e["robot"] == n and abs(e["t"] - t) <= 6.0 and e["kind"] in (
            "pose re-seeded", "camera disagrees", "lost")]
        if not c.get("d_cm"):           # from the program's own event line
            return "%s: %s." % (n, txt.rstrip("."))
        s = "%s's own pose and the camera disagreed by up to %.1f cm (worked out from the logs)." % (n, c["d_cm"])
        if rs:
            s += " The program: %s." % _short(rs[0]["text"], 150)
        else:
            s += " Nothing in the log acted on it."
        return s
    if k == "stuck":
        m = re.search(r"move ([\d.]+) cm in ([\d.]+) s", txt)
        if m:
            return ("%s was told to drive, but the camera saw it move only %s cm in %s s, so the program dropped the move "
                    "and planned a new one%s." % (n, m.group(1), m.group(2),
                                                 "; then " + then(("trophy break", "tilt stop")) if then(
                                                     ("trophy break", "tilt stop")) else ""))
        return "%s: %s." % (n, _short(txt, 200))
    if k == "unsure":
        return ("%s was near an edge and its own pose and the camera disagreed (%s), so the program was not sure how "
                "near; it moves such a robot only in a direction both call inward." % (n, _short(txt, 80)))
    return "%s: %s." % (n, _short(txt, 200))


def _short(s, n):
    s = " ".join(str(s).split())
    return s if len(s) <= n else s[:n - 1].rstrip() + "..."


# =============================================================================================== video sources
def _hhmmss_to_t(name_hms, near_t, tz):
    """A file's HHMMSS (the run's local time) to Unix time, on the day of near_t."""
    h, m, s = int(name_hms[:2]), int(name_hms[2:4]), int(name_hms[4:6])
    off = (tz if tz is not None else -time.timezone / 3600.0) * 3600.0
    day0 = math.floor((near_t + off) / 86400.0) * 86400.0 - off
    t = day0 + h * 3600 + m * 60 + s
    if t - near_t > 43200:
        t -= 86400
    elif near_t - t > 43200:
        t += 86400
    return t


def find_sources(video_dirs, S):
    """Overhead videos with per-frame times. Full track videos first, then event clips."""
    srcs = []
    seen = set()
    t_ref = S.events[0]["t"] if S.events else time.time()
    for vd in video_dirs:
        if not vd or not os.path.isdir(vd):
            continue
        csvs = {}
        for p in glob.glob(os.path.join(vd, "**", "frames", "overhead", "track", "*.csv"), recursive=True) + \
                glob.glob(os.path.join(vd, "frames", "overhead", "track", "*.csv")):
            csvs[os.path.basename(p)[:-4]] = p
        vids = sorted(set(glob.glob(os.path.join(vd, "**", "overhead_track_*.avi*"), recursive=True)))
        tracks = []
        for v in vids:
            if v in seen or v.endswith(".part-ab") or re.search(r"\.part-a[c-z]$", v):
                continue
            m = re.search(r"overhead_track_(\d{6})_([^.]+)\.avi", os.path.basename(v))
            if not m:
                continue
            seen.add(v)
            key = "%s_%s" % (m.group(1), m.group(2))
            tracks.append(dict(path=v, key=key, hms=m.group(1), csv=csvs.get(key), kind="track"))
        tracks.sort(key=lambda s: s["hms"])
        for i, s in enumerate(tracks):
            s["t_name"] = _hhmmss_to_t(s["hms"], t_ref, S.tz)
            s["t_next"] = _hhmmss_to_t(tracks[i + 1]["hms"], t_ref, S.tz) if i + 1 < len(tracks) else None
            srcs.append(s)
        for v in sorted(glob.glob(os.path.join(vd, "**", "frames", "overhead", "clips", "*.avi"), recursive=True)):
            if v in seen:
                continue
            seen.add(v)
            c = v[:-4] + ".csv"
            srcs.append(dict(path=v, key=os.path.basename(v)[:-4], csv=c if os.path.exists(c) else None, kind="clip",
                             hms=os.path.basename(v)[:6], t_name=_hhmmss_to_t(os.path.basename(v)[:6], t_ref, S.tz),
                             t_next=None))
    return srcs


def source_times(src):
    """Per-frame Unix times of a source: its CSV's t column, else spread evenly (name start to the next file's start,
    or the frame count at 12 / s), flagged 'derived'."""
    if src.get("times") is not None:
        return src["times"]
    import cv2
    if src.get("csv"):
        rows = read_csv(src["csv"])
        ts = [_f(r.get("t")) for r in rows]
        ts = [t for t in ts if t is not None]
        if ts:
            src["times"] = ts
            src["timing"] = (rows[0].get("derived") or "derived") if "derived" in rows[0] else "frame-time CSV"
            return ts
    cap = cv2.VideoCapture(src["path"])
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    fps = cap.get(cv2.CAP_PROP_FPS) or 12.0
    cap.release()
    t0 = src["t_name"]
    if src.get("t_next") and n:
        dt = (src["t_next"] - t0) / n
        src["timing"] = "derived: frames spread evenly from the file's start time (its name) to the next file's start"
    else:
        dt = 1.0 / fps
        src["timing"] = "derived: the file's start time (its name) and %.0f frames a second" % fps
    src["times"] = [t0 + i * dt for i in range(n)]
    return src["times"]


def source_for(srcs, a, b):
    """The source holding most of [a, b] (full track videos before event clips)."""
    best, cover = None, 0.0
    for s in srcs:
        lo = s["t_name"] - 2
        hi = (s["t_next"] or s["t_name"] + 400) + 2
        if hi < a or lo > b:
            continue
        ts = source_times(s)
        if not ts:
            continue
        c = max(0.0, min(b, ts[-1]) - max(a, ts[0]))
        c += 0.01 if s["kind"] == "track" else 0.0
        if c > cover:
            best, cover = s, c
    return best if cover > 1.0 else None


# =============================================================================================== mapping cm <-> px
def board_quad(gray):
    """The board's bright quad in a picture (Otsu + the largest region, opened to drop thin attachments)."""
    import cv2
    import numpy as np
    b = cv2.GaussianBlur(gray, (7, 7), 0)
    _, th = cv2.threshold(b, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((41, 41), np.uint8))
    cs, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea)
    if cv2.contourArea(c) < 0.05 * gray.shape[0] * gray.shape[1]:
        return None
    hull = cv2.convexHull(c)
    for eps in (0.01, 0.02, 0.03, 0.05, 0.08):
        ap = cv2.approxPolyDP(hull, eps * cv2.arcLength(hull, True), True)
        if len(ap) == 4:
            q = ap.reshape(4, 2).astype(np.float32)
            # order: start top-left, clockwise in the image
            cen = q.mean(axis=0)
            ang = np.arctan2(q[:, 1] - cen[1], q[:, 0] - cen[0])
            q = q[np.argsort(ang)]
            i0 = int(np.argmin(q.sum(axis=1)))
            return np.roll(q, -i0, axis=0)
    return None


def robot_blobs(gray, mask):
    """Dark robot-sized blobs on the bright board (a local median minus the picture)."""
    import cv2
    import numpy as np
    loc = cv2.medianBlur(gray, 51)
    dark = ((loc.astype(np.int16) - gray) > 35).astype(np.uint8) * 255
    dark &= mask
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(dark)
    return np.array([cen[q] for q in range(1, n) if 120 < st[q, 4] < 1500 and max(st[q, 2], st[q, 3]) < 50],
                    dtype=np.float32).reshape(-1, 2)


def _pairs(S, frames, H, lag=0.0, tol=0.045, radius=30.0):
    import cv2
    import numpy as np
    src, dst = [], []
    for t, blobs in frames:
        if not len(blobs):
            continue
        for n in S.cam:
            fx = S.cam_near(n, t + lag, tol)
            if fx is None:
                continue
            p = np.float32([[[fx["x"], fx["y"]]]])
            q = cv2.perspectiveTransform(p, H)[0, 0]
            dd = np.linalg.norm(blobs - q, axis=1)
            a = int(np.argmin(dd))
            if dd[a] < radius and np.sum(dd < radius * 1.3) == 1:
                src.append(p[0, 0])
                dst.append(blobs[a])
    return np.float32(src).reshape(-1, 2), np.float32(dst).reshape(-1, 2)


def _cand_pairs(S, fb, lag=0.0, tol=0.045):
    """Every (fix, blob) pairing in each frame: the true ones agree on one mapping, the rest do not."""
    import numpy as np
    A, B = [], []
    for t, blobs in fb:
        if not len(blobs) or len(blobs) > 6:
            continue
        for n in S.cam:
            fx = S.cam_near(n, t + lag, tol)
            if fx is None:
                continue
            for bl in blobs:
                A.append((fx["x"], fx["y"]))
                B.append(bl)
    return np.float32(A).reshape(-1, 2), np.float32(B).reshape(-1, 2)


def _ransac_affine(A, B, thr=6.0, iters=4000, seed=1):
    """Affine RANSAC that only accepts plausible maps: both scales 0.8-12 px per cm, within 1.6x of each other, not
    folded. (cv2.estimateAffine2D accepted a map squashing every fix onto a standing obstacle: all such pairs agree.)"""
    import numpy as np
    rng = np.random.default_rng(seed)
    n = len(A)
    Ah = np.hstack([A, np.ones((n, 1), np.float32)])
    best, best_in = None, None
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        P = Ah[idx]
        if abs(np.linalg.det(P)) < 50.0:
            continue
        try:
            M = np.linalg.solve(P, B[idx]).T          # 2 x 3
        except np.linalg.LinAlgError:
            continue
        sv = np.linalg.svd(M[:, :2], compute_uv=False)
        if sv[1] < 0.8 or sv[0] > 12.0 or sv[0] / sv[1] > 1.6:
            continue
        e = np.linalg.norm(Ah @ M.T - B, axis=1)
        inl = e < thr
        if best_in is None or inl.sum() > best_in.sum():
            best, best_in = M, inl
    if best is None:
        return None, None
    # refit on the inliers (least squares)
    M = np.linalg.lstsq(Ah[best_in], B[best_in], rcond=None)[0].T
    return M, best_in


def _affine_H(M):
    import numpy as np
    return np.vstack([np.asarray(M, float), [0.0, 0.0, 1.0]])


def fit_mapping(S, srcs, max_frames=400, log=print):
    """Fit board cm -> pixel from the frames themselves, with no calibration file and no board outline needed:
      1. dark robot-sized blobs on the bright board, in up to max_frames pictures;
      2. every (camera fix, blob) pairing of each picture; an affine map fitted by RANSAC over them (the true pairings
         agree on one map, the wrong ones scatter);
      3. each video's time lag (camera fix time minus frame time; within 0.3 s with a frame-time CSV, 1.5 s with
         derived times), then the pairs within 25 px refitted: a homography when the sightings cover over half the
         board both ways and its corners stay within 25 px of the affine map's, else the affine map.
    Returns a dict (H, errors, lags) or None; each source gets src["lag"]."""
    import cv2
    import numpy as np
    rect = S.rect
    per = []
    mask = None
    for s in srcs:
        ts = source_times(s)
        if not ts:
            continue
        cap = cv2.VideoCapture(s["path"])
        n = len(ts)
        want = max(40, max_frames // max(1, len(srcs)))
        step = max(1, n // want)
        fr = []
        for i in range(0, n, step):
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, f = cap.read()
            if not ok:
                break
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f
            fr.append((ts[i], g))
        cap.release()
        if fr:
            per.append((s, fr))
    if not per:
        return None
    # the bright board (a loose mask: blobs off the board are dropped)
    g0 = per[0][1][len(per[0][1]) // 2][1]
    _, th = cv2.threshold(cv2.GaussianBlur(g0, (9, 9), 0), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    cs, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask = np.zeros(g0.shape, np.uint8)
    if cs:
        cv2.drawContours(mask, [cv2.convexHull(max(cs, key=cv2.contourArea))], -1, 255, -1)
    else:
        mask[:] = 255
    per = [(s, [(t, robot_blobs(g, mask)) for t, g in fr]) for s, fr in per]
    A = np.concatenate([_cand_pairs(S, fb)[0] for _, fb in per])
    B = np.concatenate([_cand_pairs(S, fb)[1] for _, fb in per])
    if len(A) < 12:
        log("mapping: too few robot sightings (%d)" % len(A))
        return None
    M, inl = _ransac_affine(A, B)
    if M is None or inl.sum() < 12:
        log("mapping: no consistent map between the camera fixes and the robots in the pictures")
        return None
    H = Ha = _affine_H(M)

    def best_lag(s, fb, H):
        wide = "derived" in (s.get("timing") or "")
        lags = [x / 10.0 for x in range(-15, 16)] if wide else [-0.3, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 0.3]
        best = None
        for lag in lags:
            a, b = _pairs(S, fb, H, lag, radius=12.0)
            if len(a) < 4:
                continue
            e = float(np.median(np.linalg.norm(cv2.perspectiveTransform(a[None], H)[0] - b, axis=1)))
            key = (len(a), -e, -abs(lag))
            if best is None or key > best[0]:
                best = (key, lag)
        return best[1] if best else 0.0

    res, kind = None, "affine"
    for rnd in range(3):
        AA, BB = [], []
        for s, fb in per:
            s["lag"] = best_lag(s, fb, H)
            a, b = _pairs(S, fb, H, s["lag"], radius=25.0)
            AA.append(a)
            BB.append(b)
        a, b = np.concatenate(AA), np.concatenate(BB)
        if len(a) < 12:
            break
        M2, inl = _ransac_affine(a, b, thr=5.0, iters=1500)
        Ha = _affine_H(M2) if M2 is not None else Ha
        Hn, kind = Ha, "affine"
        span_x = (a[:, 0].max() - a[:, 0].min()) / (rect[2] - rect[0])
        span_y = (a[:, 1].max() - a[:, 1].min()) / (rect[3] - rect[1])
        if span_x > 0.5 and span_y > 0.5:
            Hh, inh = cv2.findHomography(a, b, cv2.RANSAC, 5.0)
            if Hh is not None:
                cc = np.float32([[(rect[0], rect[1]), (rect[2], rect[1]), (rect[2], rect[3]), (rect[0], rect[3])]])
                dev = np.linalg.norm(cv2.perspectiveTransform(cc, Hh)[0] - cv2.perspectiveTransform(cc, Ha)[0], axis=1)
                sv = np.linalg.svd(Hh[:2, :2] / Hh[2, 2], compute_uv=False)
                if dev.max() < 25.0 and abs(Hh[2, 0]) + abs(Hh[2, 1]) < 0.004 and sv[0] / max(sv[1], 1e-6) < 1.6:
                    Hn, kind, inl = Hh, "homography", inh
        e = np.linalg.norm(cv2.perspectiveTransform(a[None], Hn)[0] - b, axis=1)
        res = (float(np.median(e[inl.ravel() == 1])), int(inl.sum()), len(a), round(float(span_x), 2), round(float(span_y), 2))
        H = Hn
    if res is None:
        return None
    out = dict(method="fitted from the frames: robot blobs vs camera fixes (RANSAC %s)" % kind, H=H.tolist(),
               median_err_px=round(res[0], 2), inliers=res[1], pairs=res[2], spread=[res[3], res[4]],
               lags={os.path.basename(s["path"]): s.get("lag", 0.0) for s, _ in per},
               frames=sum(len(fb) for _, fb in per))
    log("mapping: %s, median %.1f px on %d of %d robot sightings (spread %.0f%% x %.0f%% of the board); lag per "
        "video (camera fix time minus frame time): %s" % (kind, res[0], res[1], res[2], 100 * res[3], 100 * res[4],
                                                          ", ".join("%+.1f s" % v for v in out["lags"].values())))
    if res[0] > 8.0 or res[1] < 15:
        log("mapping: poor fit; drawn anyway, flagged")
        out["poor"] = True
    return out


def mapping_from_calib(path):
    """A pinhole calibration (tools/recognition/cameras.py format) as a floor homography."""
    import cv2
    import numpy as np
    c = json.load(open(path))
    if "H" in c:
        return dict(method="homography file %s" % os.path.basename(path), H=c["H"], H_floor=c.get("H_floor", c["H"]))
    K, d, rv, tv = (np.array(c[k], float) for k in ("K", "dist", "rvec", "tvec"))
    my = bool(c.get("mirror_y"))
    P = np.array([(x, -y if my else y, 0.0) for x, y in [(0, 0), (122, 0), (122, 122), (0, 122)]], float)
    uv, _ = cv2.projectPoints(P, rv, tv, K, d)
    H = cv2.getPerspectiveTransform(np.float32([(0, 0), (122, 0), (122, 122), (0, 122)]), uv.reshape(4, 2).astype(
        np.float32))
    return dict(method="pinhole calibration %s (floor plane)" % os.path.basename(path), H=H.tolist(), H_floor=H.tolist())


# =============================================================================================== drawing
class Painter:
    def __init__(self, S, mapping, w=640, h=480):
        import numpy as np
        self.S, self.np = S, np
        self.H = np.array(mapping["H"], float) if mapping else None
        self.w, self.h = w, h

    def px(self, x, y):
        H = self.H
        v = H @ (x, y, 1.0)
        return (int(round(v[0] / v[2])), int(round(v[1] / v[2])))

    def cm_per_px(self, x, y):
        a, b = self.px(x, y), self.px(x + 10.0, y)
        return 10.0 / max(1e-6, math.hypot(a[0] - b[0], a[1] - b[1]))

    def draw(self, img, t, focus, band_cm, layers=None):
        import cv2
        S = self.S
        if self.H is None:
            return img
        r = S.rect
        ov = img.copy()
        # edge safety band: the board's outline and the band inside it
        outer = [self.px(r[0], r[1]), self.px(r[2], r[1]), self.px(r[2], r[3]), self.px(r[0], r[3])]
        inner = [self.px(r[0] + band_cm, r[1] + band_cm), self.px(r[2] - band_cm, r[1] + band_cm),
                 self.px(r[2] - band_cm, r[3] - band_cm), self.px(r[0] + band_cm, r[3] - band_cm)]
        np = self.np
        band = np.zeros(img.shape[:2], np.uint8)
        cv2.fillPoly(band, [np.int32(outer)], 255)
        cv2.fillPoly(band, [np.int32(inner)], 0)
        ov[band > 0] = (ov[band > 0] * 0.72 + np.array((40, 170, 230)) * 0.28).astype(np.uint8)
        # keep-outs active at t: a light fill and a solid outline
        for k in keepouts_at(S, t):
            c = self.px(k["x"], k["y"])
            rr = max(3, int(k["r"] / self.cm_per_px(k["x"], k["y"])))
            cv2.circle(ov, c, rr, (40, 40, 220), -1, cv2.LINE_AA)
        img = cv2.addWeighted(ov, 0.6, img, 0.4, 0)
        cv2.polylines(img, [np.int32(outer)], True, (40, 170, 230), 1, cv2.LINE_AA)
        cv2.polylines(img, [np.int32(inner)], True, (40, 170, 230), 1, cv2.LINE_AA)
        for k in keepouts_at(S, t):
            c = self.px(k["x"], k["y"])
            rr = max(3, int(k["r"] / self.cm_per_px(k["x"], k["y"])))
            cv2.circle(img, c, rr, (40, 40, 220), 1, cv2.LINE_AA)
        for n in S.robots:
            col = ROBOT_COLORS.get(n, OTHER_COLORS[hash(n) % 2])
            m = S.move_at(n, t)
            if m:
                pts = [self.px(x, y) for x, y in move_path(m)]
                if m.get("kind") == "reverse":
                    _dashed(img, pts[0], pts[-1], col)
                else:
                    cv2.polylines(img, [np.int32(pts)], False, col, 2, cv2.LINE_AA)
                cv2.drawMarker(img, pts[-1], col, cv2.MARKER_TILTED_CROSS, 8, 2)
            # camera fixes: the last 1.2 s of pictures up to t, at their picture time
            for fx in S.cam_between(n, t - 1.2, t + 0.04):
                st = fx.get("st") or S.fix_status(n, fx)
                fx["st"] = st
                p = self.px(fx["x"], fx["y"])
                fresh = t - fx["t"] < 0.15
                if st == "refused":
                    cv2.drawMarker(img, p, (30, 30, 235), cv2.MARKER_TILTED_CROSS, 11 if fresh else 7, 2)
                else:
                    cv2.circle(img, p, 4 if fresh else 2, (60, 200, 60) if st == "used" else (150, 150, 150), -1,
                               cv2.LINE_AA)
            o = S.own_at(n, t)
            if o:
                c = self.px(o["x"], o["y"])
                cpp = self.cm_per_px(o["x"], o["y"])
                cv2.circle(img, c, max(4, int(BODY_CM / 2 / cpp)), col, 2, cv2.LINE_AA)
                sd = 2.0 * o["sd"]
                if sd / cpp >= 2:
                    _dashed_circle(img, c, int(sd / cpp), col)
                h = math.radians(o["th"])
                tip = self.px(o["x"] + 8.0 * math.cos(h), o["y"] + 8.0 * math.sin(h))
                cv2.line(img, c, tip, col, 2, cv2.LINE_AA)
                cv2.putText(img, n, (c[0] + 9, c[1] - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, n, (c[0] + 9, c[1] - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)
        return img


def _dashed(img, a, b, col, dash=6):
    import cv2
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(L / dash))
    for i in range(0, n, 2):
        p = (int(a[0] + (b[0] - a[0]) * i / n), int(a[1] + (b[1] - a[1]) * i / n))
        q = (int(a[0] + (b[0] - a[0]) * min(n, i + 1) / n), int(a[1] + (b[1] - a[1]) * min(n, i + 1) / n))
        cv2.line(img, p, q, col, 2, cv2.LINE_AA)


def _dashed_circle(img, c, r, col):
    import cv2
    for a in range(0, 360, 30):
        cv2.ellipse(img, c, (r, r), 0, a, a + 15, col, 1, cv2.LINE_AA)


def keepouts_at(S, t):
    out = []
    cleared = {}
    for e in S.events:
        if e["kind"] == "keep-out cleared":
            m = re.search(r"at ([\d.-]+), ([\d.-]+)", e["text"])
            if m:
                cleared[(round(float(m.group(1)), 1), round(float(m.group(2)), 1))] = e["t"]
    for k in S.keepouts:
        if "cleared" in (k.get("what") or ""):
            continue
        x, y, r, t0 = _f(k.get("x")), _f(k.get("y")), _f(k.get("r"), 2.0), _f(k.get("t"))
        if None in (x, y, t0) or t0 > t:
            continue
        tc = cleared.get((round(x, 1), round(y, 1)))
        if tc is not None and tc <= t:
            continue
        out.append(dict(x=x, y=y, r=r, t=t0, src=k.get("src", ""), robot=k.get("robot", "")))
    return out


def band_at(S, t, robot=None):
    """The edge band drawn: the e08 derived clearance (edge_judgement need_cm, nearest row) when the run logged it,
    else the 16 cm move-end rule of the runs before e08."""
    best = None
    for r in S.edgej:
        tt = _f(r.get("t"))
        if tt is None or (robot and r.get("robot") != robot):
            continue
        if best is None or abs(tt - t) < abs(best[0] - t):
            best = (tt, _f(r.get("need_cm"), EDGE_BAND_OLD_CM))
    return (best[1], "derived clearance (edge_judgement.csv)") if best else (EDGE_BAND_OLD_CM, "16 cm move-end rule")


def log_lines(S, a, b):
    """Every log line in [a, b] for the page and the captions: events, planned moves, breaks, reflexes."""
    L = [dict(t=e["t"], robot=e["robot"], kind=e["kind"], text=e["text"]) for e in S.events if a <= e["t"] <= b
         and e["kind"] not in ("find me",)]
    for m in S.moves:
        if a <= m["t"] <= b:
            L.append(dict(t=m["t"], robot=m["robot"], kind="move %s planned" % m["n"],
                          text="%s %s: turn %+.0f deg over %.0f cm at %.1f cm/s, from (%.1f, %.1f) to (%.1f, %.1f); "
                               "result: %s" % (m["kind"], m["style"], m["turn"], m["length"], m["speed"] or 0, m["x0"],
                                               m["y0"], m["x1"], m["y1"], m["result"])))
    for r in S.breaks:
        t = _f(r.get("t"))
        if t is not None and a <= t <= b:
            L.append(dict(t=t, robot=r.get("robot"), kind="break", text=r.get("why", "")))
    for r in S.reflex:
        t = _f(r.get("t"))
        if t is not None and a <= t <= b:
            L.append(dict(t=t, robot=r.get("actor"), kind="reflex", text="%s vs %s: %s, %s" % (
                r.get("kind"), r.get("other"), r.get("method"), r.get("outcome"))))
    L.sort(key=lambda x: x["t"])
    return L


def caption(img, S, t, mom, lines):
    """A bar under the picture: clock time, robot, the moment's kind; then the newest log line for that robot,
    wrapped over two lines."""
    import cv2
    import numpy as np
    w = img.shape[1]
    bar = np.full((60, w, 3), 24, np.uint8)
    n = mom["robot"]
    head = "%s  %s  %s" % (clock_ms(t, S.tz), n or "-", KINDS.get(mom["kind"], (0, mom["kind"]))[1].upper())
    cv2.putText(bar, head, (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(bar, "0.5x", (w - 42, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 200, 255), 1, cv2.LINE_AA)
    last = [x for x in lines if x["t"] <= t and (not n or x["robot"] in (n, "-"))]
    if last:
        x = last[-1]
        s = " ".join(("%s %s: %s" % (clock(x["t"], S.tz), x["kind"], x["text"])).split())
        per = max(30, int((w - 16) / 6.6))
        cut = s.rfind(" ", 0, per) if len(s) > per else len(s)
        cut = cut if cut > 0 else per
        l1, l2 = s[:cut], _short(s[cut:].strip(), per)
        for k, ln in enumerate((l1, l2)):
            cv2.putText(bar, ln, (8, 36 + 17 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 230, 200), 1, cv2.LINE_AA)
    return np.vstack([img, bar])


# =============================================================================================== clips
def ffmpeg_ok():
    return shutil.which("ffmpeg") is not None


def render_clip(S, mom, src, mapping, out_dir, idx, log=print):
    """One clip (MP4, H.264) and its poster. Returns (clip name, poster name, t0 of the video)."""
    import cv2
    a, b = mom["span"]
    ts = source_times(src)
    i0 = max(0, bisect.bisect_left(ts, a) - 1)
    i1 = min(len(ts) - 1, bisect.bisect_right(ts, b))
    if i1 - i0 < 5:
        return None
    a, b = max(a, ts[i0]), min(b, ts[i1])
    lag = src.get("lag", 0.0) or 0.0
    cap = cv2.VideoCapture(src["path"])
    cap.set(cv2.CAP_PROP_POS_FRAMES, i0)
    frames = []
    for i in range(i0, i1 + 1):
        ok, f = cap.read()
        if not ok:
            break
        if f.ndim == 2:
            f = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)
        frames.append((ts[i], f))
    cap.release()
    if len(frames) < 5:
        return None
    h0, w0 = frames[0][1].shape[:2]
    # crop to the board (plus a margin) and scale it up to at most 640 x 520: the drawing is done after scaling,
    # with the mapping moved into the cropped picture, so lines stay sharp
    import numpy as np
    x0c, y0c, scale = 0, 0, min(1.0, 640.0 / w0)
    cw, ch = w0, h0
    if mapping:
        Hm = np.array(mapping["H"], float)
        r = S.rect
        cc = cv2.perspectiveTransform(np.float32([[(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]]), Hm)[0]
        m = 28
        x0c, y0c = max(0, int(cc[:, 0].min()) - m), max(0, int(cc[:, 1].min()) - m)
        x1c, y1c = min(w0, int(cc[:, 0].max()) + m), min(h0, int(cc[:, 1].max()) + m)
        cw, ch = x1c - x0c, y1c - y0c
        scale = min(640.0 / cw, 520.0 / ch, 2.5)
        Hs = np.array([[scale, 0, -x0c * scale], [0, scale, -y0c * scale], [0, 0, 1.0]]) @ Hm
        mapping = dict(mapping, H=Hs.tolist())
    ow, oh = int(cw * scale) // 2 * 2, int(ch * scale) // 2 * 2
    P = Painter(S, mapping, ow, oh)
    lines = log_lines(S, a - 5, b + 1)
    band, _ = band_at(S, mom["t"], mom["robot"])
    name = "clip_%02d.mp4" % idx
    poster = "poster_%02d.jpg" % idx
    drawn = []
    for t, f in frames:
        g = f[y0c:y0c + ch, x0c:x0c + cw]
        g = cv2.resize(g, (ow, oh), interpolation=cv2.INTER_LINEAR if scale > 1 else cv2.INTER_AREA)
        g = P.draw(g, t + lag, mom["robot"], band)
        drawn.append((t, caption(g, S, t + lag, mom, lines)))
    # poster: the frame nearest the moment
    pt = min(drawn, key=lambda d: abs(d[0] - mom["t"]))
    cv2.imwrite(os.path.join(out_dir, poster), pt[1], [cv2.IMWRITE_JPEG_QUALITY, 80])
    t_video0 = drawn[0][0] + lag
    path = os.path.join(out_dir, name)
    for crf in (28, 32, 36, 40):
        if not _encode(drawn, path, crf):
            log("clip %d: no H.264 encoder (ffmpeg missing): poster only" % idx)
            return None, poster, t_video0
        if os.path.getsize(path) <= MAX_BYTES:
            break
    return name, poster, t_video0


def _encode(drawn, path, crf):
    """Half speed at OUT_FPS: output frame k shows the newest picture taken at or before t0 + k * SLOW / OUT_FPS."""
    h, w = drawn[0][1].shape[:2]
    w2, h2 = w - w % 2, h - h % 2
    if not ffmpeg_ok():
        return False
    cmd = ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", "%dx%d" % (w, h), "-r",
           str(OUT_FPS), "-i", "-", "-vf", "crop=%d:%d:0:0" % (w2, h2), "-c:v", "libx264", "-preset", "veryfast",
           "-crf", str(crf), "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", path]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    t0, t1 = drawn[0][0], drawn[-1][0]
    k, j = 0, 0
    try:
        while True:
            tk = t0 + k * SLOW / OUT_FPS
            if tk > t1 + 1e-6:
                break
            while j + 1 < len(drawn) and drawn[j + 1][0] <= tk:
                j += 1
            p.stdin.write(drawn[j][1].tobytes())
            k += 1
        p.stdin.close()
    except BrokenPipeError:
        pass
    return p.wait() == 0


# =============================================================================================== page data
def moment_data(S, mom, t_video0):
    """The program's state sampled 10 times a second over the clip (for the page's map), plus its log lines."""
    a, b = mom["span"]
    if t_video0 is not None:
        a = t_video0
    samples = []
    t = a
    while t <= b + 1e-6:
        rob = {}
        for n in S.robots:
            o = S.own_at(n, t)
            fx = S.cam_between(n, t - 0.6, t + 0.02)
            m = S.move_at(n, t)
            rob[n] = dict(
                own=[round(o["x"], 1), round(o["y"], 1), round(o["th"], 1), round(o["sd"], 2)] if o else None,
                fix=[[round(f["x"], 1), round(f["y"], 1), round(t - f["t"], 2), (f.get("st") or S.fix_status(n, f))[0]]
                     for f in fx[-6:]],
                move=m["n"] if m else None,
                lost=S.is_lost(n, t))
        samples.append(dict(t=round(t, 2), r=rob, ko=[i for i, k in enumerate(keepouts_all(S, a, b))
                                                      if k["t"] <= t and (k["t_off"] is None or k["t_off"] > t)]))
        t += 0.1
    moves = {}
    for m in S.moves:
        if a - 6 <= m["t"] <= b:
            moves[m["n"] + "@" + m["robot"]] = dict(robot=m["robot"], n=m["n"], kind=m["kind"], t=m["t"],
                                                    path=[[round(x, 1), round(y, 1)] for x, y in move_path(m)],
                                                    result=m["result"], turn=m["turn"])
    band, band_src = band_at(S, mom["t"], mom["robot"])
    return dict(t0=round(a, 3), t1=round(b, 3), slow=SLOW, rect=S.rect, band_cm=band, band_src=band_src,
                robots={n: ROBOT_HEX.get(n, "#3a9a7a") for n in S.robots}, samples=samples,
                keepouts=keepouts_all(S, a, b), moves=list(moves.values()),
                log=[dict(t=round(x["t"], 2), clock=clock(x["t"], S.tz), robot=x["robot"], kind=x["kind"],
                          text=x["text"]) for x in log_lines(S, a - 4, b + 0.5)])


def keepouts_all(S, a, b):
    out = []
    cleared = {}
    for e in S.events:
        if e["kind"] == "keep-out cleared":
            m = re.search(r"at ([\d.-]+), ([\d.-]+)", e["text"])
            if m:
                cleared.setdefault((round(float(m.group(1)), 1), round(float(m.group(2)), 1)), e["t"])
    for k in S.keepouts:
        if "cleared" in (k.get("what") or ""):
            continue
        x, y, r, t0 = _f(k.get("x")), _f(k.get("y")), _f(k.get("r"), 2.0), _f(k.get("t"))
        if None in (x, y, t0) or t0 > b:
            continue
        toff = cleared.get((round(x, 1), round(y, 1)))
        if toff is not None and toff < a:
            continue
        out.append(dict(x=x, y=y, r=r, t=round(t0, 2), t_off=round(toff, 2) if toff else None, robot=k.get("robot"),
                        src=k.get("src")))
    return out


# =============================================================================================== commands
def make(sess_dir, video_dirs, out_dir, top=TOP_N, kinds=None, at=(), label=None, calib=None, no_video=False,
         deadline=None, log=print, notes=None):
    t_start = time.time()
    S = Session(sess_dir)
    os.makedirs(out_dir, exist_ok=True)
    at_t = [_at_time(x, S) for x in at]
    moms = pick(S, top, kinds, at_t)
    for key, text in (notes or {}).items():      # hand-written explanations (--notes file: {"HH:MM:SS": "..."})
        tn = _at_time(key, S)
        tn = tn[0] if isinstance(tn, tuple) else tn
        for m in moms:
            if abs(m["t"] - tn) < 6.0:
                m["explain"], m["explain_by"] = text, "written by hand from the logs (--notes)"
    log("%s: %d moments picked (%.1f s)" % (S.name, len(moms), time.time() - t_start))
    srcs = [] if no_video else find_sources(video_dirs, S)
    used = []
    for m in moms:
        s = source_for(srcs, *m["span"]) if srcs else None
        m["_src"] = s
        if s and s not in used:
            used.append(s)
    mapping = None
    if used:
        mapping = mapping_from_calib(calib) if calib else fit_mapping(S, used, log=log)
        with open(os.path.join(out_dir, "mapping.json"), "w") as fh:
            json.dump(dict(mapping or {}, sources=[dict(file=os.path.basename(s["path"]), timing=s.get("timing"))
                                                   for s in used]), fh, indent=1)
    rows = []
    for i, m in enumerate(moms, 1):
        clip = poster = None
        t_video0 = None
        s = m.pop("_src", None)
        if s is not None and (deadline is None or time.time() < deadline):
            try:
                r = render_clip(S, m, s, mapping, out_dir, i, log)
                if r:
                    clip, poster, t_video0 = r
            except Exception as e:     # noqa: BLE001  (one clip failing never stops the rest)
                log("clip %d failed: %s" % (i, e))
        if clip is None and poster is None:
            poster = still_near(sess_dir, video_dirs, m["t"], S, out_dir, i)
        data = moment_data(S, m, t_video0)
        dn = "moment_%02d.json" % i
        with open(os.path.join(out_dir, dn), "w") as fh:
            json.dump(data, fh, separators=(",", ":"))
        rows.append(dict(n=i, time=clock(m["t"], S.tz), t=round(m["t"], 2), kind=m["kind"], label=m["label"],
                         robot=m["robot"], explain=m["explain"], explain_by=m.get("explain_by", "the tool's template"),
                         why=m["why"], score=round(m["score"], 1),
                         clip=clip, poster=poster, data=dn, video_t0=t_video0, slow=SLOW,
                         timing=s.get("timing") if s else None, forced=bool(m.get("forced")),
                         source=os.path.basename(s["path"]) if s else None))
        log("  %d. %s %-6s %-9s %s %s" % (i, rows[-1]["time"], m["kind"], m["robot"], "clip" if clip else "no clip",
                                         _short(m["explain"], 90)))
    res = dict(version=VERSION, session=S.name, label=label or S.name, rect=S.rect, robots=S.robots,
               made=time.strftime("%Y-%m-%d %H:%M"), seconds=round(time.time() - t_start, 1),
               mapping=(mapping or {}).get("method"), mapping_err_px=(mapping or {}).get("median_err_px"),
               moments=rows)
    with open(os.path.join(out_dir, "moments.json"), "w") as fh:
        json.dump(res, fh, indent=1)
    log("wrote %s (%.0f s)" % (os.path.join(out_dir, "moments.json"), time.time() - t_start))
    return res


def still_near(sess_dir, video_dirs, t, S, out_dir, idx, within_s=5.0):
    """No clip: a still the run saved within 5 s (pictures/HHMMSS_*overhead.jpg, frames/overhead/stills/*.jpg)."""
    pics = glob.glob(os.path.join(sess_dir, "pictures", "*overhead*.jpg"))
    for vd in video_dirs:
        if vd and os.path.isdir(vd):
            pics += glob.glob(os.path.join(vd, "**", "frames", "overhead", "stills", "*.jpg"), recursive=True)
    best = None
    for p in pics:
        m = re.match(r"(\d{6})", os.path.basename(p))
        if not m:
            continue
        d = abs(_hhmmss_to_t(m.group(1), t, S.tz) - t)
        if d <= within_s and (best is None or d < best[0]):
            best = (d, p)
    if not best:
        return None
    name = "poster_%02d.jpg" % idx
    shutil.copy2(best[1], os.path.join(out_dir, name))
    return name


def _at_time(s, S):
    """HH:MM:SS or Unix time, optionally /kind and /robot: 17:34:45/reflex/Right."""
    s = str(s)
    if "/" in s:
        p = s.split("/")
        return (_at_time(p[0], S), p[1] or None, p[2] if len(p) > 2 else None)
    if re.match(r"^\d{1,2}:\d\d(:\d\d)?$", s):
        p = [int(v) for v in s.split(":")] + [0]
        ref = S.events[0]["t"] if S.events else time.time()
        return _hhmmss_to_t("%02d%02d%02d" % (p[0], p[1], p[2]), ref, S.tz)
    return float(s)


def build_page(made_dirs, page_dir, title="Robot Replay", log=print):
    """One self-contained index.html (the data embedded) with the clips and posters beside it."""
    os.makedirs(page_dir, exist_ok=True)
    runs = []
    for k, d in enumerate(made_dirs):
        mj = json.load(open(os.path.join(d, "moments.json")))
        pre = "r%d_" % (k + 1)
        for m in mj["moments"]:
            for f in ("clip", "poster"):
                if m.get(f):
                    shutil.copy2(os.path.join(d, m[f]), os.path.join(page_dir, pre + m[f]))
                    m[f] = pre + m[f]
            m["d"] = json.load(open(os.path.join(d, m.pop("data"))))
        if k == 0 and mj["moments"]:
            max(mj["moments"], key=lambda m: m.get("score", 0))["featured"] = True
        runs.append(mj)
    tpl = open(os.path.join(HERE, "replay_page.html")).read()
    blob = json.dumps(dict(version=VERSION, runs=runs), separators=(",", ":")).replace("</", "<\\/")
    html = tpl.replace("/*__DATA__*/null", blob).replace("__TITLE__", title)
    p = os.path.join(page_dir, "index.html")
    with open(p, "w") as fh:
        fh.write(html)
    log("page: %s (%.1f MB page, %d runs, %d moments)" % (p, os.path.getsize(p) / 1e6, len(runs),
                                                          sum(len(r["moments"]) for r in runs)))
    return p


OFF_FILE = os.path.expanduser("~/.config/mirlt/replay_clips_off")   # the switch: this file present = off


def switched_on():
    """The after-run step's switch (on by default): MIRLT_REPLAY_CLIPS=0 or the file OFF_FILE turns it off."""
    return os.environ.get("MIRLT_REPLAY_CLIPS", "1") != "0" and not os.path.exists(OFF_FILE)


def replay_dir(sess_dir):
    """Where the after-run step writes: beside the session folder, never inside it (the session's logs zip is being
    made while this runs)."""
    return os.path.normpath(sess_dir) + "-replay"


def auto(sess_dir, video_dirs, upload=False, cap_s=CAP_S, log=print):
    """After a free-roam run (run-fleet-pi.sh freeroam-finish starts it in the background, beside the upload): never
    raises, stops making clips after cap_s, writes <session>-replay/ (clips, moments.json, page/) and, with upload,
    adds replay-<session>.zip to the run's release once upload_session.py has made it. While it reads the videos it
    keeps <session>-replay/.busy, which upload_session.py's auto-clean waits for (up to MIRLT_REPLAY_WAIT_S) before
    deleting them. MIRLT_REPLAY_CLIPS=0 turns it off."""
    if not switched_on():
        log("replay clips: switched off (MIRLT_REPLAY_CLIPS=0 or %s)" % OFF_FILE)
        return 0
    out = replay_dir(sess_dir)
    busy = os.path.join(out, ".busy")
    ok = False
    try:
        os.makedirs(out, exist_ok=True)
        with open(busy, "w") as fh:
            fh.write(str(os.getpid()))
        make(sess_dir, video_dirs, out, deadline=time.time() + cap_s, log=log)
        build_page([out], os.path.join(out, "page"), log=log)
        ok = True
    except Exception as e:     # noqa: BLE001
        log("replay clips: did not finish (%s); the run's upload is not affected" % e)
    finally:
        try:
            os.remove(busy)
        except OSError:
            pass
    if ok and upload:
        try:
            _upload(sess_dir, out, log)
        except Exception as e:     # noqa: BLE001
            log("replay clips: upload did not happen (%s); the clips stay in %s" % (e, out))
    return 0


def _upload(sess_dir, out, log):
    """replay-<session>.zip into the release upload_session.py made for this session (its manifest names the tag)."""
    sys.path.insert(0, os.path.join(REPO, "tools", "pi"))
    import zipfile
    import upload_session as U
    name = os.path.basename(os.path.normpath(sess_dir))
    mf = os.path.join(U.STATE_DIR, "uploads", name + ".json")
    t0 = time.time()
    while time.time() - t0 < UPLOAD_WAIT_S:    # the run's upload may still be going (hours for a 4 h run): wait for it
        if os.path.exists(mf) and os.path.getmtime(mf) >= t0 - 6 * 3600:
            break
        time.sleep(60)
    if not os.path.exists(mf):
        log("replay clips: the run's upload did not finish in %.0f h: kept in %s" % (UPLOAD_WAIT_S / 3600, out))
        return
    man = json.load(open(mf))
    tag = man.get("tag")
    tok = U.token()
    if not tag or not tok:
        log("replay clips: no release tag or token: kept in %s" % out)
        return
    z = os.path.join(out, "replay-%s.zip" % name)
    with zipfile.ZipFile(z, "w", zipfile.ZIP_STORED) as zf:
        for f in sorted(os.listdir(os.path.join(out, "page"))):
            zf.write(os.path.join(out, "page", f), "replay/" + f)
    rel = U.release_for(tag, tok, "")
    have = U._assets(rel, tok)
    U.upload_file(rel, os.path.basename(z), z, tok, have, lambda n: None)
    os.remove(z)
    log("replay clips: uploaded %s to release %s" % (os.path.basename(z), tag))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=["pick", "make", "page", "auto"])
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--video", action="append", default=[], help="a folder holding the overhead video / frames")
    ap.add_argument("--out", default="")
    ap.add_argument("--to", default="", help="page: the page folder")
    ap.add_argument("--top", type=int, default=TOP_N)
    ap.add_argument("--kinds", default="", help="only these kinds, comma-separated (%s)" % ",".join(KINDS))
    ap.add_argument("--at", action="append", default=[], help="also this moment (HH:MM:SS or Unix time)")
    ap.add_argument("--label", default=None)
    ap.add_argument("--calib", default=None, help="a pinhole calibration or homography JSON instead of the fit")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--notes", default=None, help='a JSON file {"HH:MM:SS": "a hand-written explanation"}')
    ap.add_argument("--upload", action="store_true")
    ap.add_argument("--cap-s", type=float, default=CAP_S)
    a = ap.parse_args(argv)
    kinds = [k for k in a.kinds.split(",") if k] or None
    if a.cmd == "pick":
        S = Session(a.dirs[0])
        for m in pick(S, a.top, kinds, [_at_time(x, S) for x in a.at]):
            print("%s %-8s %-7s %5.1f  %s" % (clock(m["t"], S.tz), m["kind"], m["robot"], m["score"],
                                              _short(m["explain"], 150)))
        return 0
    if a.cmd == "make":
        make(a.dirs[0], a.video, a.out or os.path.join(a.dirs[0], "replay"), a.top, kinds, a.at, a.label, a.calib,
             a.no_video, notes=json.load(open(a.notes)) if a.notes else None)
        return 0
    if a.cmd == "page":
        build_page(a.dirs, a.to or "replay-page")
        return 0
    if a.cmd == "auto":
        vids = a.video or []
        fw = os.path.join(a.dirs[0], "frames_where.txt")
        if os.path.exists(fw):
            vids.append(os.path.dirname(open(fw).read().strip()))
        return auto(a.dirs[0], vids, a.upload, a.cap_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
