#!/usr/bin/env python3
"""Cautious calibration run at the school table, zones 1-2 (Nick and Jay-Sun, 4 Nov 2026).

Nick (4 Nov): "Get enough data without calibration to use as a base level of measurement. But use calibration for
safety purposes like keeping it from the edge of the table and returning it to a parked position. After that, perform
a series of runs with updated calibration." Experiment speed (10 cm/s drive, 60 deg/s turns, 5 cm/s final approach) so
the calibration matches the experiments.

In order (every step logged in the session folder, logs/<date-time>-calib[-dry]/):
  0. find      the robot(s) on the WiFi (`status` to the given addresses and the broadcast; a sleeping robot is sent
               `wake` until it answers), the edge guard switched on, the robot's CURRENT corrections read with `get`
               and saved (robot_values_before.json) before anything changes.
  1. bind      each robot does the sync wiggle (experiments.py: a quarter turn and back) and the roof tag whose heading
               follows that wiggle is bound to it (binding.json). Refused, in plain words, when no tag moves or two do.
               Named from the WiFi name (AL-1DC0) unless the site names it. `--binding fixed`: the old way (the site's
               tags.ids), kept as a switch.
  A. baseline  with the robot's current values: in-place turns at the park point (both directions; also gives the
               tag's offset from the turning point), straight rolls of known lengths, a still period (gyro drift), then
               the P01-P20 tour. Every leg is driven on wheels + gyro alone (`turn` / `go` legs, the robot's own pose,
               never corrected by the camera); the camera measures where it actually stopped. The camera is used for
               SAFETY only: before a leg that would take the robot (from where the camera sees it) near an edge, a
               keep-out or the edge of the camera's view, the pose is corrected first (logged), and to park at P11.
  B. fit       wheel_scale, gyro_scale and gyro_drift_dps from phase A's legs (least squares, with +- values), `set`
               on the robot, read back with `get` (sent again once if it does not read back; the old values kept to
               undo: --undo-from).
  C. series    TOURS more tours of P01-P20 with the new values (in order, reversed, shuffled with a recorded seed), the
               same per-leg errors measured. Stops cleanly on low battery (engine_checks.battery_floor). Always ends
               parked at P11 and sends `sleep 10`.
Cautious: the roof tag lost for more than LOST_HOLD_S while driving -> `stop` and hold until it is seen again (up to
LOST_GIVEUP_S; then the run ends where it is). `--lost-policy keep_going` is the experiments' policy (drive on).

Writes: status.json (Mission Control's card), events.jsonl, legs.csv (every leg: commanded, robot's own, camera's),
camera_track.csv (every camera read), odometry_raw.csv (the robot's own pose, anchored once, never camera-corrected),
odometry_corrected.csv (the same with the safety corrections), fit.json, binding.json, report.md + charts.

    python3 tools/fleet/calib_run.py --robot 192.168.1.134                    # the school, on the Pi
    python3 tools/fleet/calib_run.py --dry                                    # the simulator dress rehearsal
    python3 tools/fleet/calib_run.py --dry --cam-z-cm 228.6                   # camera height read as above the table
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import random
import re
import signal
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np   # noqa: E402

VERSION = "2026-11-04a"
PY = sys.executable
LOG_PORT, CMD_PORT = 5005, 5006
FIX_PORT = 5013          # the tag bridge's (or the simulator's) camera reads come here (run_plan uses 5010)

# ------------------------------------------------------------------------------------------------ chosen numbers
SPEED_CMS = 10.0         # Nick: experiment speed (experiments.SPEED cruise). No DOE: matches the experiments
TURN_DPS = 60.0          # Nick: experiment turn speed. Set through turn_max_rpm (below). No DOE
FINAL_CMS = 5.0          # Nick: the final approach speed. No DOE
FINAL_CM = 5.0           # chosen: the last 5 cm of every leg at FINAL_CMS. DOE note: 3 / 5 / 10 cm against the leg's
                         # end error and time per point
WHEEL_R_CM, TRACK_CM = 1.7, 9.0    # the Alvik's nominal wheel radius and track (tools/sim/config.py)
TURN_RPM = round(math.radians(TURN_DPS) * TRACK_CM / 2.0 / (2 * math.pi * WHEEL_R_CM) * 60.0, 1)   # 26.5 rpm top wheel
                         # speed in a turn = 60 deg/s on nominal wheels. Derived, not measured: the report gives the
                         # turn rate the camera saw
DWELL_S = 3.0            # Nick / Jay-Sun: 3 s stop at each point
SETTLE_S = 0.4           # chosen: after a leg ends, wait this long, then average the camera over MEASURE_S
MEASURE_S = 1.2          # chosen: ~12 reads at 10 a second. DOE note: 0.6 / 1.2 / 2.4 s against the per-leg error's
                         # spread with the robot standing still (still period)
LOST_HOLD_S = 1.0        # chosen (cautious): the tag unseen this long while driving -> stop and hold. At 10 cm/s that
                         # is 10 cm blind at most. DOE note: 0.5 / 1 / 2 s against holds per tour and how far it drives
                         # blind (the camera's dropout lengths from camera_track.csv)
LOST_GIVEUP_S = 30.0     # chosen: held this long and still not seen -> the run ends where it is (said plainly)
VIEW_X_MAX_CM = 78 * 2.54  # Jay-Sun: the one camera reads a roof tag anywhere in zones 1-2 (x < 78 in)
VIEW_MARGIN_CM = 6.0     # chosen: the robot's middle stays this far inside that line (half the body + 1 cm)
EDGE_AIM_CM = 1.0        # chosen: a point nearer an edge than edge_in_cm + this is aimed this much further in (P04:
                         # 13.3 cm from the edge is aimed at 14.0), so a leg's own error cannot take it under the margin
KEEPOUT_CLEAR_CM = 6.72 + 2.0   # the body's half diagonal + 2 cm from any keep-out or reference tile
SAFE_BUF_CM = 3.0        # chosen: a leg's predicted path (from where the camera sees the robot) must stay this much
                         # inside the edge margin + 4 % of the leg's length, else the pose is corrected first
NEAR_EDGE_STAGE_CM = 10.0  # chosen: a point within edge_in + 8 cm of an edge: driven to a stop 10 cm short (odometry),
                         # then the last 10 cm from the camera's pose (safety)
TURNS_A = (90, 90, 90, 90, -90, -90, -90, -90, 170, -170, 170, -170)   # chosen: 12 in-place turns, 1400 deg, both
                         # directions (170, not 180: a half turn's direction is ambiguous, the robot takes the short way
                         # to its target heading). Gives gyro_scale to about +-0.002 (1 sigma) with 0.5 deg per measured turn.
                         # DOE note: 6 / 12 / 24 turns against the SE (fit.json)
ROLLS_A = (20.0, 45.0, 70.0, 20.0, 45.0, 70.0)   # chosen: out-and-back rolls (540 cm, 12 legs) from P11 towards -x
                         # (70 cm keeps 4 cm off the zone 1 outlet's clearance): with the tour's legs, wheel_scale and
                         # the coast per leg to about +-0.003 (1 sigma). DOE note: lengths and count against the SE
STILL_S = 60.0           # chosen: gyro drift from 60 s standing still (+-0.01-0.02 deg/s). DOE note: 30 / 60 / 120 s
STILL_C_S = 30.0         # chosen: a shorter still period at the start of each phase-C tour (drift after the fit)
TOURS_C = 3              # Nick: "a series of runs" (2-3): in order, reversed, shuffled
TOUR_SEED = 20261104     # chosen: the shuffled tour's seed (recorded)
ENOUGH = {"wheel_scale": 0.008, "gyro_scale": 0.005, "gyro_drift_dps": 0.03}   # "enough" phase-A data: each value's
                         # 95 % +- (1.96 x SE) under this before it is used. Chosen: 0.8 % in the wheels is 8 mm per
                         # metre (the coast per leg is fitted beside it, which costs precision), 0.5 % in the gyro is
                         # 0.45 deg per quarter turn, both well under the ~3 % expected to be removed; drift 0.03 deg/s
                         # is 1.8 deg a minute. DOE note: turns / rolls / still seconds against these
BOUNDS = {"wheel_scale": (0.90, 1.10), "gyro_scale": (0.90, 1.15), "gyro_drift_dps": (-2.0, 2.0)}
PARAM_TOL = 0.0006       # a read-back matches within this (the firmware prints 3 decimals; doe_loop.PARAM_TOL)
PARAMS = ("wheel_scale", "gyro_scale", "gyro_drift_dps")
EDGE_GUARD = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
              "set bumper_cm 0"]       # mission_run.EDGE_GUARD (lost at every power-off)
LEG_TIMEOUT_S = 40.0
GO_PIECE_CM = 100.0      # a longer drive is made in equal pieces of at most this (the firmware refuses go legs over
                         # e02_go_max_cm, 110 by default). Each piece is measured as its own leg
MAX_TURN_DEG = 170.0     # a bigger turn is made as two halves (a half turn's direction is ambiguous)
BATTERY_CHECK_EVERY = 6  # chosen: `status` every 6 points
JAY_IN = {"P01": 2.4, "P05": 2.7, "P11": 4.5, "P15": 5.2}   # Jay-Sun 3-4 Nov: how far the robot came out, no height
                         # correction, always away from the camera
READ_PX = 8.0            # simulator: a tag is read when its shorter side in the picture is at least this many pixels.
                         # Chosen so the simulated camera (lens 145 cm up) reads the roof tag in all of zones 1-2
                         # (9.5 px at x = 78 in) and not the far corner tags 21/22 (6.6 px), as Jay-Sun saw


def loopback_up():
    """In a fresh network namespace (unshare -rn) the loopback starts down (sweeps/rp_shim.py's way)."""
    import fcntl
    import struct
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ifr = struct.pack("16sH14s", b"lo", 0, b"\0" * 14)
        flags = struct.unpack("16sH14s", fcntl.ioctl(s, 0x8913, ifr))[1]
        if not flags & 1:
            fcntl.ioctl(s, 0x8914, struct.pack("16sH14s", b"lo", flags | 1, b"\0" * 14))
    except OSError:
        pass
    finally:
        s.close()


def say(s):
    print(time.strftime("%H:%M:%S ") + s, flush=True)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# =============================================================================================== the site
class Site:
    """The school site's table, points, keep-outs (outlets and reference tiles), edge margin and the camera's view."""

    def __init__(self, site):
        import sticker_points as SP
        import production as PR
        self.site = site
        t = site.get("table") or {}
        self.L, self.W = float(t.get("length_cm", 396.2)), float(t.get("width_cm", 167.6))
        pts, zones, _w = SP.points_cm(site)
        self.points = {k: tuple(v) for k, v in pts.items()}
        self.zones = zones
        self.edge = float(site.get("edge_in_cm", 13.0))
        self.keepouts = [dict(k, kind="outlet") for k in PR.site_keepouts(site)]
        try:
            import ref_tags as RT
            rc = site.get("ref_tags") or {}
            if rc.get("use", True):
                cfg = dict(RT.default_config(), **rc)
                for k in RT.keepouts(RT.world_poses(cfg, self.L, self.W), cfg):
                    self.keepouts.append(dict(k, kind="tile"))
        except Exception as e:  # noqa: BLE001
            say("(reference tiles not added as keep-outs: %s)" % e)
        self.view_x = VIEW_X_MAX_CM - VIEW_MARGIN_CM
        hp = (site.get("homes") or {})
        self.park_label = "P11"
        self.park_face = float(hp.get("face_deg", 90.0))

    def zone12(self):
        return sorted([k for k, z in self.zones.items() if z in (1, 2)], key=lambda s: int(s[1:]))

    def aim(self, lab):
        """Where a point is aimed: EDGE_AIM_CM further in when it lies within edge + EDGE_AIM_CM of an edge."""
        x, y = self.points[lab]
        m = self.edge + EDGE_AIM_CM
        return (min(max(x, m), self.L - m), min(max(y, m), self.W - m))

    def edge_dist(self, p):
        x, y = p
        return min(x, y, self.L - x, self.W - y)

    def keepout_dist(self, p):
        best = (1e9, "")
        for k in self.keepouts:
            d = poly_dist(p, k["polygon"])
            if d < best[0]:
                best = (d, k["name"])
        return best

    def near_edge(self, lab):
        return self.edge_dist(self.points[lab]) < self.edge + 8.0

    def via(self, a, b):
        """One waypoint round a keep-out in the way of a -> b (None if no single waypoint works): the corners of each
        keep-out pushed out by its clearance + 3 cm, the shortest clear one."""
        best = None
        for k in self.keepouts:
            P = np.array(k["polygon"], float)
            c = P.mean(0)
            for q in P:
                v = q - c
                w = q + v / (np.linalg.norm(v) or 1) * (KEEPOUT_CLEAR_CM + 3.0) * 1.42
                w = (float(w[0]), float(w[1]))
                if self.problems(a, w) or self.problems(w, b):
                    continue
                d = math.hypot(w[0] - a[0], w[1] - a[1]) + math.hypot(b[0] - w[0], b[1] - w[1])
                if best is None or d < best[0]:
                    best = (d, w)
        return best[1] if best else None

    def problems(self, a, b, buf=0.0):
        """Why the straight line a -> b is not safe ([] = safe): the robot's middle under the edge margin (less 0.5 cm
        of measuring), near a keep-out or a reference tile, or past the camera's view; buf: the leg's own possible
        error (cm) added to each."""
        out = []
        for k in range(1, 11):
            p = (a[0] + (b[0] - a[0]) * k / 10.0, a[1] + (b[1] - a[1]) * k / 10.0)
            e = self.edge_dist(p)
            if e - buf < self.edge - 0.5:
                out.append("the edge margin (%.1f cm from the edge, %.1f with the leg's possible error)" % (e, e - buf))
            d, n = self.keepout_dist(p)
            if d - buf < KEEPOUT_CLEAR_CM:
                out.append("%s (%.1f cm)" % (n, d))
            if p[0] + buf > self.view_x:
                out.append("the edge of the camera's view (x %.0f cm)" % p[0])
            if out:
                break
        return out


def compose(a, rel):
    """Pose a (x, y, th deg) moved by rel (dx, dy in a's frame, dth)."""
    c, s_ = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
    return (a[0] + c * rel[0] - s_ * rel[1], a[1] + s_ * rel[0] + c * rel[1], wrap180(a[2] + rel[2]))


def relative(b0, b1):
    """The robot's own motion over a leg, in the frame of its pose at the leg's start (its LEG records). A robot with no
    camera fix starts every idle leg at (0, 0, 0) (firmware pose::start(false)); this works either way."""
    c, s_ = math.cos(math.radians(b0[2])), math.sin(math.radians(b0[2]))
    dx, dy = b1[0] - b0[0], b1[1] - b0[1]
    return (c * dx + s_ * dy, -s_ * dx + c * dy, wrap180(b1[2] - b0[2]))


def poly_dist(p, poly):
    """Distance from p to a polygon (0 inside)."""
    x, y = p
    inside = False
    n = len(poly)
    best = 1e9
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1:
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / ((dx * dx + dy * dy) or 1e-12)))
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return 0.0 if inside else best


# =============================================================================================== the robots' WiFi
class Link:
    """UDP to the robots (alvik_console's ports): commands to <ip>:5006, every line back on :5005 ("name|seq|line")."""

    def __init__(self, folder, bind_host=""):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.rx.bind((bind_host, LOG_PORT))       # no SO_REUSEADDR: another program on the port is found here
        except OSError:
            raise SystemExit("STOPPED: port %d is in use (Mission Control's own run, the console or robot_sleep.py is "
                             "talking to the robots). Close it first. Nothing moved." % LOG_PORT)
        self.rx.settimeout(0.2)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.lines = []          # (t, name, ip, text)
        self.ips = {}            # name -> ip
        self.cond = threading.Condition()
        self.stop_ = threading.Event()
        self.f = open(os.path.join(folder, "messages.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "robot", "ip", "line"])
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self):
        while not self.stop_.is_set():
            try:
                data, (ip, _) = self.rx.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            txt = data.decode("utf-8", "replace").strip()
            name, _, rest = txt.partition("|")
            _seq, _, line = rest.partition("|")
            t = time.time()
            with self.cond:
                self.lines.append((t, name, ip, line))
                self.ips[name] = ip
                self.w.writerow(["%.3f" % t, name, ip, line])
                self.cond.notify_all()

    def send(self, ip, cmd):
        try:
            self.tx.sendto(cmd.encode(), (ip, CMD_PORT))
        except OSError:
            pass

    def mark(self):
        with self.cond:
            return len(self.lines)

    def wait_for(self, pred, since, timeout):
        """The first line from index `since` on for which pred(name, line) is truthy, or None after timeout."""
        end = time.time() + timeout
        k = since
        with self.cond:
            while True:
                while k < len(self.lines):
                    t, n, ip, line = self.lines[k]
                    k += 1
                    r = pred(n, line)
                    if r:
                        return r
                left = end - time.time()
                if left <= 0:
                    return None
                self.cond.wait(min(left, 0.2))

    def close(self):
        self.stop_.set()
        try:
            self.f.close()
        except OSError:
            pass


class Robot:
    """One robot through the Link: status, get / set with read-back, idle `turn` / `go` legs and their LEG records."""

    def __init__(self, link, name, ip):
        self.link, self.name, self.ip = link, name, ip
        self.leg_id = int(time.time()) % 100000 * 10 + 1000000   # above any id this robot heard before today
        self.cmds = None

    def ask(self, cmd, pred, timeout=3.0, tries=3):
        for _ in range(tries):
            k = self.link.mark()
            self.link.send(self.ip, cmd)
            r = self.link.wait_for(lambda n, l: n == self.name and pred(l), k, timeout)
            if r:
                return r
        return None

    def status(self):
        import engine_checks as EC
        r = self.ask("status", lambda l: EC.parse_status(l), 3.0)
        return r

    def get(self, param):
        rx = re.compile(r"^M,%s = (-?[0-9.]+)" % re.escape(param))

        def p(line):
            m = rx.match(line)
            return (float(m.group(1)),) if m else None     # a tuple: 0.000 is an answer too
        r = self.ask("get %s" % param, p, 2.5)
        return r[0] if r else None

    def set(self, param, value):
        self.link.send(self.ip, "set %s %.4f" % (param, value))
        time.sleep(0.15)

    def set_checked(self, param, value):
        """`set`, then `get`; sent again once if it does not read back. (ok, read value)."""
        got = None
        for _ in range(2):
            self.set(param, value)
            got = self.get(param)
            if got is not None and abs(got - value) <= PARAM_TOL:
                return True, got
        return False, got

    def leg(self, kind, value, timeout=LEG_TIMEOUT_S, on_tick=None):
        """An idle leg (`turn <deg> <id>` / `go <cm> <id>`): resent until queued, then waited for. Returns dict(result,
        info, start (x, y, th, ms), end (x, y, th, ms))."""
        self.leg_id += 1
        lid = self.leg_id
        cmd = "%s %.2f %d" % ("turn" if kind == "turn" else "go", value, lid)
        out = {"result": None, "info": "", "start": None, "end": None, "id": lid}
        rx = re.compile(r"^D,LEG,[^,]*,(\d+),(\w+),(-?[0-9.]+),(\w+),(\d+),(-?[0-9.]+),(-?[0-9.]+),(-?[0-9.]+),?(.*)$")
        k = self.link.mark()
        t_end = time.time() + timeout
        last_send = 0
        queued = False
        while time.time() < t_end:
            if not queued and time.time() - last_send > 1.0:
                self.link.send(self.ip, cmd)
                last_send = time.time()
            r = self.link.wait_for(lambda n, l: n == self.name and rx.match(l), k, 0.25)
            if on_tick and on_tick():
                self.link.send(self.ip, "stop")
            if not r:
                continue
            with self.link.cond:
                k = self._index_after(r, k)
            if int(r.group(1)) != lid:
                continue
            res = r.group(4)
            pose = (float(r.group(6)), float(r.group(7)), float(r.group(8)), int(r.group(5)))
            if res in ("queued", "dup"):
                queued = True
            elif res == "started":
                queued = True
                out["start"] = pose
            elif res in ("done", "failed", "refused"):
                out.update(result=res, info=r.group(9), end=pose)
                if out["start"] is None:
                    out["start"] = pose
                return out
        out["result"] = "timeout"
        return out

    def _index_after(self, match, k):
        for i in range(k, len(self.link.lines)):
            if self.link.lines[i][3] == match.string:
                return i + 1
        return len(self.link.lines)


# =============================================================================================== the camera
class Eye:
    """The newest camera reads per tag id: {id: (t, x, y, th)} (the turning point's frame is applied by the run)."""

    def __init__(self, folder, port=FIX_PORT):
        self.latest = {}
        self.hist = {}
        self.lock = threading.Lock()
        self.stop_ = threading.Event()
        self.f = open(os.path.join(folder, "camera_track.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "tag_id", "x_cm", "y_cm", "th_deg", "sigma_cm", "source"])
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("127.0.0.1", port))
        self.rx.settimeout(0.2)
        threading.Thread(target=self._rx, daemon=True).start()

    def tag_of(self, msg):
        r = str(msg.get("robot", ""))
        return int(r[3:]) if r.startswith("tag") and r[3:].isdigit() else None

    def transform(self, msg):
        return msg

    def _rx(self):
        while not self.stop_.is_set():
            try:
                data, _ = self.rx.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except ValueError:
                continue
            if msg.get("type") != "fix":
                continue
            msg = self.transform(msg)
            if not msg:
                continue
            tid = self.tag_of(msg)
            if tid is None:
                continue
            t = time.time()
            rec = (t, float(msg["x"]), float(msg["y"]), float(msg.get("th", 0.0)))
            with self.lock:
                self.latest[tid] = rec
                self.hist.setdefault(tid, []).append(rec)
                if len(self.hist[tid]) > 4000:
                    del self.hist[tid][:1000]
                self.w.writerow(["%.3f" % t, tid, "%.2f" % rec[1], "%.2f" % rec[2], "%.2f" % rec[3],
                                 "%.2f" % float(msg.get("sigma", 0) or 0), msg.get("source", "camera")])
                self.nrows = getattr(self, "nrows", 0) + 1
                if self.nrows % 50 == 0:
                    self.f.flush()

    def seen_ids(self, within_s=1.5):
        now = time.time()
        with self.lock:
            return sorted(i for i, r in self.latest.items() if now - r[0] <= within_s)

    def last(self, tid):
        with self.lock:
            return self.latest.get(tid)

    def age(self, tid):
        r = self.last(tid)
        return 1e9 if r is None else time.time() - r[0]

    def window(self, tid, t0, t1):
        with self.lock:
            return [r for r in self.hist.get(tid, []) if t0 <= r[0] <= t1]

    def measure(self, tid, dur=MEASURE_S):
        """Average pose over the next dur s (circular mean heading). None if fewer than 3 reads."""
        t0 = time.time()
        time.sleep(dur)
        rs = self.window(tid, t0, time.time())
        if len(rs) < 3:
            return None
        xs, ys = np.median([r[1] for r in rs]), np.median([r[2] for r in rs])
        th = math.degrees(math.atan2(np.mean([math.sin(math.radians(r[3])) for r in rs]),
                                     np.mean([math.cos(math.radians(r[3])) for r in rs])))
        return (float(xs), float(ys), float(th), len(rs))

    def close(self):
        self.stop_.set()
        try:
            self.f.close()
        except OSError:
            pass


class SchoolCamera:
    """The school's one camera as a pinhole (simulator and the four-point check): lens at (x, y, z) cm in the table frame
    (z above the table top), looking down the table's length. C920e: 1280 x 960 (the site's capture), ~70 deg across."""

    def __init__(self, z_cm=145.0, behind_cm=45 * 2.54, y_cm=167.6 / 2, size=(1280, 960), hfov_deg=70.4,
                 look_at=(150.0, 83.8, 0.0)):
        import tag_cam as TCAM
        self.C = np.array([-behind_cm, y_cm, z_cm], float)
        self.R = TCAM.look_rotation(self.C, look_at)
        self.W, self.H = size
        self.f = (self.W / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)

    def project(self, P):
        Pc = (np.asarray(P, float) - self.C) @ self.R.T
        if np.any(Pc[:, 2] <= 1):
            return None
        u = self.f * Pc[:, 0] / Pc[:, 2] + self.W / 2.0
        v = self.f * Pc[:, 1] / Pc[:, 2] + self.H / 2.0
        return np.stack([u, v], 1)

    def tag_px(self, x, y, z, s, th_deg=0.0):
        """The tag's shorter side in pixels (None outside the picture)."""
        c, sn = math.cos(math.radians(th_deg)), math.sin(math.radians(th_deg))
        h = s / 2.0
        P = [[x + c * a - sn * b, y + sn * a + c * b, z] for a, b in ((-h, -h), (h, -h), (h, h), (-h, h))]
        uv = self.project(P)
        if uv is None or np.any(uv[:, 0] < 2) or np.any(uv[:, 0] > self.W - 2) or np.any(uv[:, 1] < 2) or \
                np.any(uv[:, 1] > self.H - 2):
            return None
        sides = [np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4)]
        return float(min(sides))

    def flat(self, x, y, h):
        """Where a point h cm above the table is put when it is taken to lie ON the table (no height correction)."""
        C = self.C
        k = C[2] / (C[2] - h)
        return (C[0] + (x - C[0]) * k, C[1] + (y - C[1]) * k)


def four_point(cam_c, h, points):
    """Jay-Sun's four points: how far a tag h cm up is put off with no height correction, for a camera lens at cam_c.
    {P: (in off, cm off)}; always away from the camera."""
    out = {}
    C = np.asarray(cam_c, float)
    for p in JAY_IN:
        x, y = points[p]
        k = C[2] / (C[2] - h)
        fx, fy = C[0] + (x - C[0]) * k, C[1] + (y - C[1]) * k
        d = math.hypot(fx - x, fy - y)
        out[p] = (d / 2.54, d)
    return out


class SimEye(Eye):
    """The simulator's camera: fleet_sim's true poses (its fixes, no noise) -> what the school's one camera would report
    for the robot's roof tag: only where its tag is big enough in the picture (READ_PX), position noise growing with
    range, and with (bridge as built) or without the tag-height correction. The simulator's robot -> tag id mapping is
    the simulator's own truth: the run never reads it (it binds by the wiggle)."""

    def __init__(self, folder, port, cam, sim_tags, tag_h=6.19, tag_cm=8.0, correct=True, seed=3, decoys=(), gaps=()):
        self.cam, self.sim_tags, self.h, self.s, self.correct = cam, dict(sim_tags), tag_h, tag_cm, correct
        self.t_made = time.time()
        self.gaps = [tuple(float(v) for v in g.split(":")) for g in gaps]   # "T:DUR": the tag unread (a hand over it)
        self.rng = random.Random(seed)
        self.unread = 0
        super().__init__(folder, port)
        for d in decoys:    # tests: a tag lying still on the table (a spare), id, x, y, th
            threading.Thread(target=self._decoy, args=(d,), daemon=True).start()

    def _decoy(self, d):
        tid, x, y, th = d
        while not self.stop_.is_set():
            with self.lock:
                self.latest[tid] = (time.time(), x + self.rng.gauss(0, 0.2), y + self.rng.gauss(0, 0.2), th)
                self.hist.setdefault(tid, []).append(self.latest[tid])
            time.sleep(0.1)

    def transform(self, msg):
        name = msg.get("robot")
        if name not in self.sim_tags:
            return None
        el = time.time() - self.t_made
        if any(t <= el < t + d for t, d in self.gaps):
            return None
        x, y, th = float(msg["x"]), float(msg["y"]), float(msg["th"])
        px = self.cam.tag_px(x, y, self.h, self.s, th)
        if px is None or px < READ_PX:
            self.unread += 1
            return None
        rng_cm = float(np.linalg.norm(np.array([x, y, self.h]) - self.cam.C))
        sd = 0.08 + 0.0012 * rng_cm          # chosen: ~0.4 cm at 3 m (sub-pixel corners, one camera). Not measured
        if not self.correct:
            x, y = self.cam.flat(x, y, self.h)
        return {"type": "fix", "robot": "tag%d" % self.sim_tags[name], "x": x + self.rng.gauss(0, sd),
                "y": y + self.rng.gauss(0, sd), "th": th + self.rng.gauss(0, 0.8 + 0.002 * rng_cm), "sigma": sd,
                "source": "sim one camera"}


# =============================================================================================== binding
def bind_by_wiggle(robots, eye, wiggle_deg=90.0, pause_s=1.0, log=say, moved_deg=None):
    """Each robot (in turn) does the sync wiggle; the tag whose heading follows it is bound. {name: tag id} or raises
    BindError with plain words. robots: [Robot]. moved_deg: a tag 'moves' when its heading changes more than this
    (default half the wiggle); a robot's tag must turn within 40 % of the wiggle and come back within 25 deg."""
    moved_deg = moved_deg if moved_deg is not None else wiggle_deg / 2.0
    out = {}
    for rb in robots:
        before = {}
        t0 = time.time()
        time.sleep(1.0)
        for i in eye.seen_ids(2.0):
            rs = eye.window(i, t0, time.time())
            if rs:
                before[i] = rs[-1][3]
        if not before:
            raise BindError("no roof tag is seen by the camera (nothing to bind %s to). Check that the robot is in "
                            "zones 1-2 and its tag faces up, and that the camera's picture is sharp." % rb.name)
        r1 = rb.leg("turn", wiggle_deg)
        mid = {i: (eye.last(i) or (0, 0, 0, before[i]))[3] for i in before}
        time.sleep(pause_s)
        mid2 = {i: (eye.last(i) or (0, 0, 0, before[i]))[3] for i in before}
        r2 = rb.leg("turn", -wiggle_deg)
        time.sleep(0.6)
        after = {i: (eye.last(i) or (0, 0, 0, before[i]))[3] for i in before}
        if r1["result"] != "done" or r2["result"] != "done":
            raise BindError("%s did not do its wiggle (%s %s / %s %s)" % (rb.name, r1["result"], r1["info"],
                                                                         r2["result"], r2["info"]))
        moved = []
        for i in before:
            d1 = wrap180(0.5 * (mid[i] + mid2[i]) - before[i]) if abs(wrap180(mid[i] - mid2[i])) < 20 else \
                wrap180(mid2[i] - before[i])
            back = wrap180(after[i] - before[i])
            log("  tag %d: turned %+.0f deg with %s's wiggle, %+.0f deg from where it started after it" % (
                i, d1, rb.name, back))
            if abs(d1) > moved_deg:
                ok = abs(d1 - wiggle_deg) <= 0.4 * abs(wiggle_deg) and abs(back) <= 25
                moved.append((i, ok, d1))
        good = [i for i, ok, _ in moved if ok]
        if not moved:
            raise BindError("no roof tag moved when %s did its wiggle (tags seen: %s). Is that robot in the camera's "
                            "view, with its tag on its roof?" % (rb.name, ", ".join(map(str, before))))
        if len(moved) > 1:
            raise BindError("%d tags moved when %s did its wiggle (%s): cannot tell which one is on it. Only one robot "
                            "may move at a time; check nobody moved a tag." % (len(moved), rb.name,
                                                                                 ", ".join(str(m[0]) for m in moved)))
        if not good:
            raise BindError("tag %d moved with %s's wiggle but not as commanded (%+.0f deg for %+.0f): not bound"
                            % (moved[0][0], rb.name, moved[0][2], wiggle_deg))
        if good[0] in out.values():
            raise BindError("tag %d moved with two robots' wiggles" % good[0])
        out[rb.name] = good[0]
    return out


class BindError(Exception):
    pass


def site_name_for(site, wifi_name, ip):
    """The site's name for a robot found on the WiFi (a site robot listing its ip or wifi name), else the WiFi name."""
    for r in (site or {}).get("robots") or []:
        if (r.get("ip") and r.get("ip") == ip) or (r.get("wifi") and r.get("wifi") == wifi_name):
            return r["name"]
    return wifi_name


def fixed_binding(site, names):
    """The old way (switch --binding fixed): the site's tags.ids by robot name."""
    ids = ((site or {}).get("tags") or {}).get("ids") or {}
    out = {n: int(ids[n]) for n in names if n in ids}
    missing = [n for n in names if n not in ids]
    if missing:
        raise BindError("--binding fixed: the site's tags.ids does not list %s (it lists %s)" % (
            ", ".join(missing), ", ".join("%s=%s" % kv for kv in ids.items())))
    return out


# =============================================================================================== the fit
def fit_values(turn_rows, still_rows, roll_rows, old):
    """From phase A's measured legs: wheel_scale, gyro_scale, gyro_drift_dps with 1-sigma SEs.
    turn_rows: [(belief dtheta deg, dt s, camera dtheta deg)]; still_rows: [(belief dtheta, dt)];
    roll_rows: [(belief length cm, camera length cm)]; old: the values the robot used (to undo them).
    Firmware model (motion.h, corr.h): belief heading = gs x raw - gd x t; real = g x raw - d x t (what we want);
    belief length = ws x counted; real = w x counted."""
    gs0, gd0, ws0 = old["gyro_scale"], old["gyro_drift_dps"], old["wheel_scale"]
    rows = []
    for db, dt_, dc in turn_rows:
        raw = (db + gd0 * dt_) / gs0
        rows.append((raw, -dt_, dc, 1.0))
    for db, dt_ in still_rows:
        raw = (db + gd0 * dt_) / gs0
        rows.append((raw, -dt_, 0.0, 1.0))
    out = {"n_turns": len(turn_rows), "n_still": len(still_rows), "n_rolls": len(roll_rows)}
    if len(rows) >= 3 and still_rows:
        X = np.array([[r[0], r[1]] for r in rows])
        y = np.array([r[2] for r in rows])
        beta, *_ = np.linalg.lstsq(X, y, rcond=None)
        res = y - X @ beta
        dof = max(1, len(rows) - 2)
        s2 = max(float(res @ res) / dof, 0.05 ** 2)
        cov = np.linalg.inv(X.T @ X) * s2
        out.update(gyro_scale=float(beta[0]), gyro_drift_dps=float(beta[1]), se_gyro_scale=float(math.sqrt(cov[0, 0])),
                   se_gyro_drift_dps=float(math.sqrt(cov[1, 1])), rms_turn_deg=float(math.sqrt(np.mean(res ** 2))))
    if roll_rows:
        # camera length = w x counted + coast: the wheels still turn a little after the leg's `done` record (the robot
        # brakes after it), the same few mm on every leg; with legs of several lengths both are found (Claude's model)
        b = np.array([r[0] / ws0 for r in roll_rows])     # counted cm
        c = np.array([r[1] for r in roll_rows])
        if len(b) >= 4 and np.ptp(b) > 10:
            X = np.stack([b, np.ones_like(b)], 1)
            beta, *_ = np.linalg.lstsq(X, c, rcond=None)
            res = c - X @ beta
            s2 = max(float(res @ res) / max(1, len(b) - 2), 0.1 ** 2)
            cov = np.linalg.inv(X.T @ X) * s2
            w, se, coast = float(beta[0]), float(math.sqrt(cov[0, 0])), float(beta[1])
        else:
            w = float(b @ c / (b @ b))
            res = c - w * b
            s2 = max(float(res @ res) / max(1, len(b) - 1), 0.1 ** 2)
            se, coast = float(math.sqrt(s2 / (b @ b))), 0.0
        out.update(wheel_scale=w, se_wheel_scale=se, coast_cm=coast,
                   rms_roll_cm=float(math.sqrt(np.mean(res ** 2))), counted_cm=float(b.sum()))
    verdict = {}
    for k in PARAMS:
        if k not in out:
            verdict[k] = "not enough data"
            continue
        lo, hi = BOUNDS[k]
        pm = 1.96 * out["se_" + k]
        if not lo <= out[k] <= hi:
            verdict[k] = "outside %.2f-%.2f: refused" % (lo, hi)
        elif pm > ENOUGH[k]:
            verdict[k] = "not pinned down: +-%.4f (need +-%.4f)" % (pm, ENOUGH[k])
        else:
            verdict[k] = "ok"
    out["verdict"] = verdict
    return out


def tag_offset(samples):
    """The tag's offset from the turning point from in-place turns: [(x, y, th deg)] at rest between turns. Solves
    tag = c + R(th) o for c (turning point) and o (offset in the robot frame: ahead, left). (c, o, rms cm)."""
    if len(samples) < 3:
        return None
    A, b = [], []
    for x, y, th in samples:
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        A.append([1, 0, c, -s])
        b.append(x)
        A.append([0, 1, s, c])
        b.append(y)
    A, b = np.array(A), np.array(b)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    rms = float(math.sqrt(np.mean((A @ sol - b) ** 2)))
    return (float(sol[0]), float(sol[1])), (float(sol[2]), float(sol[3])), rms


# =============================================================================================== the run
class CalibRun:
    def __init__(self, a, site, folder):
        self.a, self.S, self.folder = a, Site(site), folder
        self.site = site
        self.events = open(os.path.join(folder, "events.jsonl"), "a")
        self.status = {"version": VERSION, "phase": "starting", "robot": None, "tag": None, "words": "Starting",
                       "started": time.time(), "dry": bool(a.dry), "result": None, "folder": folder}
        self.legs_f = open(os.path.join(folder, "legs.csv"), "w", newline="")
        self.legs = csv.writer(self.legs_f)
        self.legs.writerow(["phase", "tour", "n", "kind", "target", "commanded", "result", "info", "t0", "t1",
                            "cam0_x", "cam0_y", "cam0_th", "cam1_x", "cam1_y", "cam1_th", "bel0_x", "bel0_y", "bel0_th",
                            "bel1_x", "bel1_y", "bel1_th", "dt_s", "cam_len_cm", "bel_len_cm", "cam_turn_deg",
                            "bel_turn_deg", "along_err_cm", "cross_err_cm", "turn_err_deg", "end_miss_cm",
                            "corrected_first", "held_s", "edge_cm_min", "keepout_cm_min"])
        self.raw_f = open(os.path.join(folder, "odometry_raw.csv"), "w", newline="")
        self.raw = csv.writer(self.raw_f)
        self.raw.writerow(["t", "x_cm", "y_cm", "th_deg", "robot_ms", "anchor"])
        self.cor_f = open(os.path.join(folder, "odometry_corrected.csv"), "w", newline="")
        self.cor = csv.writer(self.cor_f)
        self.cor.writerow(["t", "x_cm", "y_cm", "th_deg", "robot_ms", "anchor"])
        self.Draw = None             # the robot's own pose in the table frame, anchored ONCE (never camera-corrected)
        self.D = None                # the same, moved to the camera's pose by each safety correction / tour start
        self.n_anchor = 0
        self.offset = (0.0, 0.0)     # the tag's offset from the turning point (ahead, left), cm
        self.closest = {"edge_cm": 1e9, "keepout_cm": 1e9, "view_x_max": -1e9}
        self.stop_flag = threading.Event()
        self.holds = []
        self.battery = []
        self.leg_n = 0
        self._leg_moving = False

    # ------------------------------------------------------------------------------------------------- bookkeeping
    def ev(self, kind, **kw):
        kw.update(kind=kind, t=round(time.time(), 3))
        self.events.write(json.dumps(kw, default=str) + "\n")
        self.events.flush()

    def set_status(self, **kw):
        self.status.update(kw)
        self.status["updated"] = time.time()
        p = os.path.join(self.folder, "status.json")
        with open(p + ".tmp", "w") as f:
            json.dump(self.status, f, indent=1, default=str)
        os.replace(p + ".tmp", p)

    # ------------------------------------------------------------------------------------------------- poses
    def cam_pose(self, dur=MEASURE_S):
        """The turning point's pose from the camera (tag offset taken off), averaged; None if the tag is not seen."""
        m = self.eye.measure(self.tag, dur)
        if m is None:
            return None
        x, y, th, n = m
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        ox, oy = self.offset
        p = (x - (c * ox - s * oy), y - (s * ox + c * oy), th)
        self.closest["edge_cm"] = min(self.closest["edge_cm"], self.S.edge_dist(p[:2]))
        self.closest["keepout_cm"] = min(self.closest["keepout_cm"], self.S.keepout_dist(p[:2])[0])
        self.closest["view_x_max"] = max(self.closest["view_x_max"], p[0])
        return p

    def odo_step(self, r):
        """After a leg: the robot's own motion added to both odometry tracks (raw and corrected)."""
        if r.get("start") is None or r.get("end") is None or self.D is None:
            return
        rel = relative(r["start"], r["end"])
        self.Draw = compose(self.Draw, rel)
        self.D = compose(self.D, rel)
        t = time.time()
        self.raw.writerow(["%.3f" % t, "%.2f" % self.Draw[0], "%.2f" % self.Draw[1], "%.2f" % self.Draw[2],
                           r["end"][3], 0])
        self.cor.writerow(["%.3f" % t, "%.2f" % self.D[0], "%.2f" % self.D[1], "%.2f" % self.D[2], r["end"][3],
                           self.n_anchor])

    def reanchor(self, cam, why):
        if self.Draw is None:
            self.Draw = tuple(cam)
            self.D = tuple(cam)
            return
        self.n_anchor += 1
        self.ev("pose_corrected", why=why, camera=cam, odometry=self.D)
        self.D = tuple(cam)

    # ------------------------------------------------------------------------------------------------- safety
    def watch(self):
        """During a leg: the cautious policy (tag lost > LOST_HOLD_S -> stop). True = stop sent."""
        if self.a.lost_policy != "hold":
            return self.stop_flag.is_set()
        if self.stop_flag.is_set():
            return True
        if self.eye.age(self.tag) > LOST_HOLD_S and self._leg_moving:
            self._lost_stop = True
            return True
        return False

    def hold_until_seen(self):
        t0 = time.time()
        self.ev("hold", why="roof tag lost for over %.1f s while driving" % LOST_HOLD_S)
        self.set_status(words="HOLDING: the camera lost the roof tag; waiting for it (up to %.0f s)" % LOST_GIVEUP_S)
        say("HOLD: the roof tag was lost while driving; the robot stopped. Waiting for the camera to see it again ...")
        while time.time() - t0 < LOST_GIVEUP_S:
            if self.eye.age(self.tag) < 0.3:
                d = time.time() - t0
                self.holds.append(d)
                say("  seen again after %.1f s: carrying on" % d)
                return d
            time.sleep(0.1)
        raise RunEnd("the camera lost the roof tag for over %.0f s: the robot stopped where it is (not parked)"
                     % LOST_GIVEUP_S)

    def drive_leg(self, kind, value, label, phase, tour=0, corrected=False, cam0=None):
        """One leg, measured: commanded `kind value`, the robot's own record, the camera before and after."""
        if cam0 is None:
            cam0 = self.cam_pose()
            if cam0 is None:
                self.hold_until_seen()
                cam0 = self.cam_pose()
        self._leg_moving = True
        self._lost_stop = False
        t0 = time.time()
        held = 0.0
        speed = None
        if kind == "go":
            speed = FINAL_CMS if label.endswith("final") else SPEED_CMS
            self.rb.set("e02_cms", speed)
        r = self.rb.leg(kind, value, on_tick=self.watch)
        self._leg_moving = False
        self.odo_step(r)
        if self._lost_stop:
            held = self.hold_until_seen()
        if self.stop_flag.is_set():
            raise RunEnd("stopped (Ctrl+C or Mission Control's Stop)")
        time.sleep(SETTLE_S)
        cam1 = self.cam_pose()
        if cam1 is None:
            held += self.hold_until_seen()
            cam1 = self.cam_pose()
        t1 = time.time()
        row = self.leg_row(phase, tour, kind, label, value, r, cam0, cam1, t0, t1, corrected, held)
        if r["result"] in ("refused", "timeout"):
            raise RunEnd("the robot %s the %s %.0f leg (%s): stopped" % (r["result"], kind, value, r["info"]))
        return cam1, r, row

    def leg_row(self, phase, tour, kind, label, value, r, c0, c1, t0, t1, corrected, held):
        self.leg_n += 1
        b0, b1 = r["start"], r["end"]
        if b0 is None or b1 is None:
            b0 = b1 = (0, 0, 0, 0)
        dt_ = (b1[3] - b0[3]) / 1000.0
        cam_len = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
        bel_len = math.hypot(b1[0] - b0[0], b1[1] - b0[1])
        cam_turn = wrap180(c1[2] - c0[2])
        bel_turn = wrap180(b1[2] - b0[2])
        along = cross = terr = miss = ""
        if kind == "go":
            h = math.radians(c0[2])
            ex, ey = c0[0] + value * math.cos(h), c0[1] + value * math.sin(h)
            dx, dy = c1[0] - ex, c1[1] - ey
            along = dx * math.cos(h) + dy * math.sin(h)
            cross = -dx * math.sin(h) + dy * math.cos(h)
            miss = math.hypot(dx, dy)
        elif kind == "turn":
            terr = wrap180(c1[2] - (c0[2] + value))      # the end heading's error (unambiguous, +-180 turns too)
        row = [phase, tour, self.leg_n, kind, label, "%.2f" % value, r["result"], r["info"], "%.3f" % t0, "%.3f" % t1,
               "%.2f" % c0[0], "%.2f" % c0[1], "%.2f" % c0[2], "%.2f" % c1[0], "%.2f" % c1[1], "%.2f" % c1[2],
               "%.2f" % b0[0], "%.2f" % b0[1], "%.2f" % b0[2], "%.2f" % b1[0], "%.2f" % b1[1], "%.2f" % b1[2],
               "%.3f" % dt_, "%.2f" % cam_len, "%.2f" % bel_len, "%.2f" % cam_turn, "%.2f" % bel_turn,
               _f(along), _f(cross), _f(terr), _f(miss), int(bool(corrected)), "%.1f" % held,
               "%.1f" % self.S.edge_dist(c1[:2]), "%.1f" % self.S.keepout_dist(c1[:2])[0]]
        self.legs.writerow(row)
        self.legs_f.flush()
        d = dict(phase=phase, tour=tour, kind=kind, label=label, value=value, result=r["result"], dt=dt_,
                 cam0=c0, cam1=c1, bel0=b0, bel1=b1, cam_len=cam_len, bel_len=bel_len, cam_turn=cam_turn,
                 bel_turn=bel_turn, along=along, cross=cross, turn_err=terr, miss=miss, corrected=corrected)
        self.rows.append(d)
        return d

    # ------------------------------------------------------------------------------------------------- moving
    def goto(self, lab, phase, tour, cam=None, odometry=True, depth=0, name=None):
        """Drive to point lab: from the robot's own pose (odometry) unless safety needs the camera's first. Returns the
        camera pose at the end."""
        tx, ty = self.S.aim(lab) if (isinstance(lab, str) and lab in self.S.points) else lab
        nm = name or str(lab)
        cam = cam or self.cam_pose()
        if cam is None:
            self.hold_until_seen()
            cam = self.cam_pose()
        corrected = False
        d = self.D if (odometry and self.D is not None) else cam
        near = isinstance(lab, str) and lab in self.S.points and self.S.near_edge(lab)
        if depth == 0:
            blocked = [q for q in self.S.problems(cam[:2], (tx, ty)) if "outlet" in q or "reference" in q]
            if blocked:
                w = self.S.via(cam[:2], (tx, ty))
                if w is not None:
                    say("  %s: round %s by (%.0f, %.0f)" % (lab, blocked[0], w[0], w[1]))
                    self.ev("detour", point=lab, via=w, why=blocked[0])
                    cam = self.goto(w, phase, tour, cam=cam, odometry=odometry, depth=1, name="%s via" % lab)
        for attempt in range(2):
            dist = math.hypot(tx - d[0], ty - d[1])
            turn = wrap180(math.degrees(math.atan2(ty - d[1], tx - d[0])) - d[2])
            go = dist - (NEAR_EDGE_STAGE_CM if near and dist > NEAR_EDGE_STAGE_CM + 2 else 0.0)
            # where it would really go, from where the camera sees it (+ the leg's possible error while on odometry)
            h = math.radians(cam[2] + turn)
            end = (cam[0] + go * math.cos(h), cam[1] + go * math.sin(h))
            buf = (SAFE_BUF_CM + 0.04 * go) if (odometry and not corrected) else 0.0
            prob = self.S.problems(cam[:2], end, buf=buf)
            if prob and attempt == 0 and odometry:
                say("  safety: from where the camera sees it, the leg to %s would reach %s: pose corrected first"
                    % (lab, prob[0]))
                self.reanchor(cam, "leg to %s would reach %s" % (lab, prob[0]))
                d = cam
                corrected = True
                continue
            if prob:
                self.ev("skip_point", point=lab, why=prob[0])
                say("  %s skipped: even from the camera's pose the leg reaches %s" % (lab, prob[0]))
                return cam
            break
        if abs(turn) > MAX_TURN_DEG:
            cam, _, _ = self.drive_leg("turn", turn / 2.0, "%s turn half" % nm, phase, tour, corrected, cam0=cam)
            cam, _, _ = self.drive_leg("turn", turn / 2.0, "%s turn half" % nm, phase, tour, corrected, cam0=cam)
        elif abs(turn) > 1.0:
            cam, _, _ = self.drive_leg("turn", turn, "%s turn" % nm, phase, tour, corrected, cam0=cam)
        if go > FINAL_CM + 1:
            n = int(math.ceil((go - FINAL_CM) / GO_PIECE_CM))
            for _ in range(n):
                cam, _, _ = self.drive_leg("go", (go - FINAL_CM) / n, "%s drive" % nm, phase, tour, corrected, cam0=cam)
            cam, _, _ = self.drive_leg("go", FINAL_CM, "%s final" % nm, phase, tour, corrected, cam0=cam)
        elif go > 0.5:
            cam, _, _ = self.drive_leg("go", go, "%s final" % nm, phase, tour, corrected, cam0=cam)
        if near and dist > NEAR_EDGE_STAGE_CM + 2:
            # the last 10 cm near an edge: from the camera's pose (safety), at the final approach speed
            dist2 = math.hypot(tx - cam[0], ty - cam[1])
            turn2 = wrap180(math.degrees(math.atan2(ty - cam[1], tx - cam[0])) - cam[2])
            if abs(turn2) > 1.0 and dist2 > 1.0 and abs(turn2) <= MAX_TURN_DEG:
                cam, _, _ = self.drive_leg("turn", turn2, "%s edge turn" % nm, phase, tour, True, cam0=cam)
            if 0.5 < dist2 <= GO_PIECE_CM:
                cam, _, _ = self.drive_leg("go", dist2, "%s edge final" % nm, phase, tour, True, cam0=cam)
        return cam

    def park(self, phase):
        """Back to P11 by the camera (safety), facing the park heading."""
        self.set_status(words="Returning to park at P11")
        cam = self.goto(self.S.park_label, phase, 0, odometry=False)
        t = wrap180(self.S.park_face - cam[2])
        for t_ in ((t / 2.0, t / 2.0) if abs(t) > MAX_TURN_DEG else (t,)):
            if abs(t_) > 3:
                cam, _, _ = self.drive_leg("turn", t_, "park face", phase, 0, True, cam0=cam)
        x, y = self.S.points[self.S.park_label]
        miss = math.hypot(cam[0] - x, cam[1] - y)
        self.ev("parked", miss_cm=miss, pose=cam)
        say("Parked at P11: %.1f cm from the sticker, facing %.0f deg" % (miss, cam[2]))
        return cam, miss

    def check_battery(self):
        import engine_checks as EC
        st = self.rb.status()
        if not st:
            return None
        b = st["battery"]
        self.battery.append((time.time(), b))
        self.status["battery"] = b
        floor = EC.battery_floor()
        if b < floor:
            raise LowBattery("battery %d %% is under the floor %d %% (engine_checks.battery_floor)" % (b, floor))
        return b

    # ------------------------------------------------------------------------------------------------- phases
    def tour(self, phase, k, order):
        say("Tour %s%d: %s" % (phase, k, " ".join(order)))
        cam = self.cam_pose()
        if cam is not None:
            self.D = tuple(cam)             # each tour starts from where the camera has it (the raw track does not)
        for j, lab in enumerate(order):
            if self.stop_flag.is_set():
                raise RunEnd("stopped")
            self.set_status(words="Phase %s tour %d: point %s (%d of %d)" % (phase, k, lab, j + 1, len(order)),
                            point=lab)
            cam = self.goto(lab, phase, k, cam=cam)
            x, y = self.S.points[lab]
            self.ev("point", phase=phase, tour=k, point=lab, cam=cam, off_cm=math.hypot(cam[0] - x, cam[1] - y))
            time.sleep(max(0.0, DWELL_S - MEASURE_S - SETTLE_S))
            if (j + 1) % BATTERY_CHECK_EVERY == 0:
                self.check_battery()
        return cam

    def still(self, secs, phase, tour=0):
        """Gyro drift: one `turn 0` leg whose standing-still start (edge_learn_ms, the edge guard learning the table)
        is made secs long, so the robot's own heading is tracked through it (a robot with no camera fix starts each
        leg's pose afresh, so the drift between legs is not in its records). edge_learn_ms is put back after."""
        self.set_status(words="Standing still %.0f s (gyro drift)" % secs)
        old = self.rb.get("edge_learn_ms")
        old = 300.0 if old is None else old
        self.rb.set_checked("edge_learn_ms", secs * 1000.0)
        c0 = self.cam_pose()
        t0 = time.time()
        r = self.rb.leg("turn", 0.0, timeout=secs + 15.0, on_tick=lambda: self.stop_flag.is_set())
        self.rb.set_checked("edge_learn_ms", old)
        c1 = self.cam_pose()
        if r["result"] != "done" or None in (c0, c1) or r["start"] is None:
            say("  still period: no usable record (%s %s)" % (r["result"], r["info"]))
            return None
        b0, b1 = r["start"], r["end"]
        dt_ = (b1[3] - b0[3]) / 1000.0
        d = dict(phase=phase, tour=tour, kind="still", label="still", value=secs, dt=dt_, cam0=c0, cam1=c1, bel0=b0,
                 bel1=b1, bel_turn=wrap180(b1[2] - b0[2]), cam_turn=wrap180(c1[2] - c0[2]), result="done")
        self.rows.append(d)
        self.legs.writerow([phase, tour, 0, "still", "still", "%.1f" % secs, "done", "", "%.3f" % t0,
                            "%.3f" % time.time()] + ["%.2f" % v for v in (c0[0], c0[1], c0[2], c1[0], c1[1], c1[2],
                                                                           b0[0], b0[1], b0[2], b1[0], b1[1], b1[2])]
                           + ["%.3f" % dt_, "", "", "%.2f" % d["cam_turn"], "%.2f" % d["bel_turn"], "", "", "", "", 0,
                              0, "", ""])
        self.legs_f.flush()
        say("  still %.0f s: the robot's own heading moved %+.2f deg (camera %+.2f): %+.4f deg/s" % (
            dt_, d["bel_turn"], d["cam_turn"], d["bel_turn"] / max(dt_, 1e-6)))
        return d

    def phase_a(self):
        self.set_status(phase="A", words="Phase A: in-place turns at P11 (current values)")
        say("Phase A: baseline with the robot's current values %s" % self.before)
        cam = self.cam_pose()
        rest = [self.eye.measure(self.tag)]
        for k, t in enumerate(TURNS_A):
            cam, r, row = self.drive_leg("turn", float(t), "turn %+d" % t, "A", 0, cam0=cam)
            m = self.eye.measure(self.tag, 0.4)
            if m:
                rest.append(m)
        off = tag_offset([(m[0], m[1], m[2]) for m in rest if m])
        if off:
            self.offset = off[1]
            self.fit_offset = {"turning_point": off[0], "offset_cm": off[1], "rms_cm": off[2],
                               "offset_len_cm": math.hypot(*off[1])}
            say("  tag offset from the turning point: %.1f cm ahead, %.1f cm left (%.1f cm, fit +- %.1f cm)" % (
                off[1][0], off[1][1], math.hypot(*off[1]), off[2]))
            self.ev("tag_offset", **self.fit_offset)
        self.still(STILL_S, "A")
        # rolls: towards -x from P11 (open table, away from the middle outlet and the camera's far limit)
        self.set_status(words="Phase A: straight rolls %s cm out and back" % " / ".join("%.0f" % v for v in
                                                                                   sorted(set(ROLLS_A))))
        cam = self.cam_pose()
        t = wrap180(180.0 - cam[2])
        for t_ in ((t / 2.0, t / 2.0) if abs(t) > MAX_TURN_DEG else (t,)):
            cam, _, _ = self.drive_leg("turn", t_, "face -x", "A", 0, cam0=cam)
        for L in ROLLS_A:
            for leg in ("out", "back"):
                end = (cam[0] + L * math.cos(math.radians(cam[2])), cam[1] + L * math.sin(math.radians(cam[2])))
                prob = self.S.problems(cam[:2], end, buf=0.04 * L)
                if prob:
                    say("  roll %.0f cm %s skipped: would reach %s" % (L, leg, prob[0]))
                    continue
                cam, _, _ = self.drive_leg("go", L, "roll %.0f %s" % (L, leg), "A", 0, cam0=cam)
                sg = 1.0 if leg == "out" else -1.0      # round by two quarter turns, left out, right back
                cam, _, _ = self.drive_leg("turn", 90.0 * sg, "roll turn", "A", 0, cam0=cam)
                cam, _, _ = self.drive_leg("turn", 90.0 * sg, "roll turn", "A", 0, cam0=cam)
        # back to P11 by the camera (safety), then anchor and tour
        cam = self.goto(self.S.park_label, "A", 0, odometry=False)
        self.set_status(words="Phase A: tour P01-P20 on wheels + gyro (camera for safety)")
        order = self.S.zone12()
        self.tour("A", 1, order + [self.S.park_label])

    def phase_b(self):
        self.set_status(phase="B", words="Phase B: fitting wheel_scale, gyro_scale, gyro_drift_dps")
        A = [r for r in self.rows if r["phase"] == "A"]
        turns = [_unwrap_turn(r) for r in A if r["kind"] == "turn" and r["result"] == "done" and abs(r["value"]) >= 30]
        still = [(r["bel_turn"], r["dt"]) for r in A if r["kind"] == "still"]
        rolls = [(r["bel_len"], r["cam_len"]) for r in A if r["kind"] == "go" and r["result"] == "done"
                 and r["value"] >= 15]
        fit = fit_values(turns, still, rolls, self.before)
        fit["old"] = dict(self.before)
        fit["enough"] = dict(ENOUGH)
        new = dict(self.before)
        for k in PARAMS:
            if fit["verdict"].get(k) == "ok":
                new[k] = round(fit[k], 4)
        fit["new"] = new
        say("Fit: " + ", ".join("%s %.4f +- %.4f (%s)" % (k, fit.get(k, float("nan")), 1.96 * fit.get("se_" + k, 0),
                                                          fit["verdict"][k]) for k in PARAMS))
        rb = {}
        for k in PARAMS:
            ok, got = self.rb.set_checked(k, new[k])
            rb[k] = {"sent": new[k], "read": got, "ok": ok}
            if not ok:
                say("  %s did NOT read back (%s): the old value is put back" % (k, got))
                self.rb.set_checked(k, self.before[k])
        fit["readback"] = rb
        self.fit = fit
        with open(os.path.join(self.folder, "fit.json"), "w") as f:
            json.dump(fit, f, indent=1, default=str)
        self.ev("fit", **{k: v for k, v in fit.items() if not isinstance(v, dict)})
        with open(os.path.join(self.folder, "robot_values_after.json"), "w") as f:
            json.dump({"robot": self.name, "values": {k: rb[k]["read"] for k in PARAMS}}, f, indent=1)
        self.set_status(fit={k: rb[k] for k in PARAMS})

    def phase_c(self):
        base = self.S.zone12()
        rng = random.Random(TOUR_SEED)
        shuf = list(base)
        rng.shuffle(shuf)
        orders = [("in order", base), ("reversed", base[::-1]), ("shuffled (seed %d)" % TOUR_SEED, shuf)]
        self.tours_c = []
        for k in range(min(self.a.tours, len(orders))):
            name, order = orders[k]
            self.set_status(phase="C", words="Phase C tour %d of %d (%s)" % (k + 1, self.a.tours, name))
            self.check_battery()
            self.still(STILL_C_S, "C", k + 1)
            self.tour("C", k + 1, order + [self.S.park_label])
            self.tours_c.append(name)

    # ------------------------------------------------------------------------------------------------- the whole run
    def start_robot(self):
        self.set_status(phase="find", words="Finding the robot on the WiFi (waking it if it sleeps)")
        found = discover(self.link, self.a.robot, self.a.wake_min, self.a.dry)
        if not found:
            raise RunEnd("no robot answered on the WiFi (sent status and wake to %s for %.0f min). Is it switched on "
                         "and on AGV_SWARM?" % (", ".join(self.a.robot) or "the broadcast", self.a.wake_min))
        say("Found: %s" % ", ".join("%s at %s" % kv for kv in found.items()))
        self.robots = [Robot(self.link, n, ip) for n, ip in found.items()]
        for rb in self.robots:
            rb.link.send(rb.ip, "wake")
            for c in EDGE_GUARD + ["set turn_max_rpm %.1f" % TURN_RPM]:
                rb.link.send(rb.ip, c)
                time.sleep(0.05)
        return found

    def bind(self, found):
        self.set_status(phase="bind", words="Binding: each robot does the sync wiggle; the tag that follows it is its")
        if self.a.binding == "fixed":
            names = [site_name_for(self.site, n, ip) for n, ip in found.items()]
            b = fixed_binding(self.site, names)
            how = "fixed (the site's tags.ids)"
        else:
            import experiments as EX
            w = EX.conf(self.site)["sync_wiggle"]
            b0 = bind_by_wiggle(self.robots, self.eye, wiggle_deg=float(w["deg"]), pause_s=float(w["pause_s"]))
            b = {site_name_for(self.site, n, found[n]): t for n, t in b0.items()}
            how = "wiggle (experiments.py sync wiggle, %.0f deg and back)" % float(w["deg"])
        rec = {"how": how, "robots": [{"name": site_name_for(self.site, rb.name, rb.ip), "wifi_name": rb.name,
                                       "ip": rb.ip, "tag_id": b.get(site_name_for(self.site, rb.name, rb.ip))}
                                      for rb in self.robots], "time": dt.datetime.now().isoformat(timespec="seconds"),
               "version": VERSION}
        with open(os.path.join(self.folder, "binding.json"), "w") as f:
            json.dump(rec, f, indent=1)
        if not self.a.dry:
            cd = self.site.get("conf_dir")
            if cd:
                os.makedirs(cd, exist_ok=True)
                with open(os.path.join(cd, "robot_binding.json"), "w") as f:
                    json.dump(rec, f, indent=1)
        say("Bound (%s): %s" % (how, ", ".join("%s = tag %s" % (r["name"], r["tag_id"]) for r in rec["robots"])))
        self.ev("bound", **rec)
        return rec

    def run(self, eye_factory):
        self.rows = []
        self.link = Link(self.folder)
        found = self.start_robot()
        self.eye = eye_factory()
        time.sleep(2.0)
        rec = self.bind(found)
        self.binding = rec
        if len(self.robots) > 1:
            say("NOTE: %d robots found; tonight's calibration drives the first one only (%s), the others stay still"
                % (len(self.robots), self.robots[0].name))
        self.rb = self.robots[0]
        self.name = rec["robots"][0]["name"]
        self.tag = rec["robots"][0]["tag_id"]
        self.set_status(robot=self.name, tag=self.tag, wifi_name=self.rb.name)
        self.before = {}
        for k in PARAMS:
            v = self.rb.get(k)
            if v is None:
                raise RunEnd("the robot did not answer `get %s`: stopped before anything changed" % k)
            self.before[k] = v
        with open(os.path.join(self.folder, "robot_values_before.json"), "w") as f:
            json.dump({"robot": self.name, "wifi_name": self.rb.name, "values": self.before,
                       "how": "read with get before anything was sent"}, f, indent=1)
        say("The robot's current values (saved first): %s" % self.before)
        b0 = self.check_battery()
        self.status["battery_start"] = b0
        cam = self.cam_pose()
        if cam is None:
            raise RunEnd("the camera does not see the robot's tag where it stands")
        self.reanchor(cam, "start")
        result = "finished"
        try:
            self.phase_a()
            self.phase_b()
            self.phase_c()
        except LowBattery as e:
            result = "low battery: %s" % e
            say("LOW BATTERY: %s. Series stopped; parking." % e)
        except RunEnd as e:
            result = "ended: %s" % e
            say("ENDED: %s" % e)
        except BindError:
            raise
        self.result = result
        parked = None
        if self.stop_flag.is_set():
            say("Stopped by hand (Ctrl+C, `stop` or Mission Control's Stop): NOT driven back to park")
        elif "camera lost" not in str(result):
            try:
                parked = self.park("park")
            except (RunEnd, LowBattery) as e:
                say("Could not park: %s" % e)
        self.parked = parked
        try:
            self.status["battery_end"] = self.check_battery()
        except LowBattery:
            self.status["battery_end"] = self.battery[-1][1] if self.battery else None
        self.rb.link.send(self.rb.ip, "sleep 10")
        k = self.link.mark()
        slept = self.link.wait_for(lambda n, l: n == self.rb.name and "sleeping" in l, k, 4.0)
        self.slept = bool(slept)
        say("Sent `sleep 10` (%s): it wakes for 20 s every 10 min; robot_sleep.py wake brings it back" % (
            "it answered" if slept else "no answer seen"))
        self.ev("end", result=result, parked=parked, slept=self.slept)
        return result


class RunEnd(Exception):
    pass


class LowBattery(Exception):
    pass


def _f(v):
    return "" if v == "" or v is None else "%.2f" % v


def _unwrap_turn(r):
    """(belief turn, dt, camera turn) unwrapped round the commanded turn (a +-180 turn's change is ambiguous by 360)."""
    v = r["value"]
    b = v + wrap180(r["bel1"][2] - (r["bel0"][2] + v))
    c = v + wrap180(r["cam1"][2] - (r["cam0"][2] + v))
    return (b, r["dt"], c)


def discover(link, ips, wake_min, dry=False):
    """{wifi name: ip} of the robots that answer `status` (sent to the given addresses and the broadcast; `wake` too, so
    a sleeping robot answers at its next check-in), for up to wake_min minutes."""
    targets = list(ips) + ([] if dry else ["255.255.255.255"])
    end = time.time() + wake_min * 60
    found = {}
    k = link.mark()
    said = 0
    while time.time() < end:
        for ip in targets:
            link.send(ip, "wake")
            link.send(ip, "status")
        r = link.wait_for(lambda n, l: (n, l) if l.startswith("M,status ") or ": awake" in l else None, k, 2.0)
        if r:
            with link.cond:
                for t, n, ip, line in link.lines[k:]:
                    if line.startswith("M,status ") or ": awake" in line:
                        found[n] = ip
            time.sleep(2.0)       # a second robot's answer
            with link.cond:
                for t, n, ip, line in link.lines[k:]:
                    if line.startswith("M,status ") or ": awake" in line:
                        found[n] = ip
            return found
        if time.time() - said > 30:
            said = time.time()
            say("  no robot answered yet (a sleeping robot answers at its check-in, every 10 min): still trying, "
                "%.0f min left" % ((end - time.time()) / 60))
    return found


# =============================================================================================== the simulator
def sim_map(S, folder, name):
    import production as PR
    pts = {k: list(v) for k, v in S.points.items()}
    m, _w = PR.site_map((S.L, S.W), [name], points=pts, keepouts=S.keepouts)
    p = os.path.join(folder, "sim", "map.json")
    os.makedirs(os.path.dirname(p), exist_ok=True)
    PR.save_map(m, p)
    return p


def start_sim(a, S, folder):
    """fleet_sim.py: one robot named from its WiFi (not in the site), on P11 facing +y, ASLEEP, with the deliberately
    wrong true values (what it really needs: --true-cal), its fixes to SimEye."""
    name = a.sim_name
    mp = sim_map(S, folder, name)
    x, y = S.points["P11"]
    starts = os.path.join(folder, "sim", "starts.json")
    with open(starts, "w") as f:
        json.dump({name: [x, y, 90.0]}, f)
    cmd = [PY, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--out", os.path.join(folder, "sim"),
           "--seed", str(a.seed), "--fixes-to", "127.0.0.1:%d" % (FIX_PORT + 100), "--fix-hz", "10", "--fix-sigma", "0",
           "--fix-sigma-deg", "0", "--fix-delay", "0.08", "--starts", starts, "--start-noise", "1.0",
           "--max-s", str(a.sim_max_s), "--drop-cm", "74", "--true-cal", "%s=%s" % (name, a.sim_true),
           "--asleep", "%s:%s" % (name, a.sim_sleep), "--quiet"]
    f = open(os.path.join(folder, "sim", "fleet_sim.txt"), "w")
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO)
    time.sleep(3.0)
    return p, ["127.0.0.2"]


# =============================================================================================== real camera
def start_bridge(a, site, folder):
    """The tag bridge on the recorder's pictures with the setup's camera model, every roof tag of the site (listed and
    spare ids) under the name tag<id>, its fixes to us."""
    tg = dict(site.get("tags") or {})
    ids = sorted(set(int(v) for v in (tg.get("ids") or {}).values()) | set(int(v) for v in tg.get("spares") or []))
    hs = [float(v) for v in (tg.get("heights_cm") or {}).values() if v]
    h = tg.get("height_cm") or (sorted(hs)[len(hs) // 2] if hs else None)
    tags = {"family": tg.get("family", "tag25h9"), "tag_cm": tg.get("tag_cm", 8.0), "roof_cm": tg.get("roof_cm", 9.5),
            "height_cm": h, "ids": {"tag%d" % i: i for i in ids}}
    tp = os.path.join(folder, "tags_bind.json")
    with open(tp, "w") as f:
        json.dump(tags, f, indent=1)
    tc = os.path.join(site.get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
    cal = a.calib or os.path.join(tc, "overhead.json")
    if not os.path.exists(cal):
        raise RunEnd("no camera model at %s: run Measure first (Mission Control, Setup). Nothing moved." % cal)
    cmd = [PY, os.path.join(HERE, "tag_bridge.py"), "--tags", tp, "--cam", "overhead=%s,%s" % (a.stream, cal),
           "--out", os.path.join(folder, "bridge"), "--to", "127.0.0.1:%d" % FIX_PORT, "--wait-calib-s", "60"]
    f = open(os.path.join(folder, "bridge.txt"), "w")
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO), cal, h


def cam_centre_from_model(path):
    try:
        from cameras import CameraModel
        m = CameraModel.load(path)
        return [float(v) for v in m.center()]
    except Exception:  # noqa: BLE001
        return None


# =============================================================================================== main
def session_dir(a):
    if a.session:
        return a.session
    return os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-calib" + ("-dry" if a.dry else ""))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default="", help="the site (default: the one in use; tonight school-conference)")
    ap.add_argument("--robot", action="append", default=[], help="a robot's IP (also: found by the broadcast)")
    ap.add_argument("--wake-min", type=float, default=12.0, help="keep sending wake/status this long (sleep 10 = a "
                                                                  "check-in every 10 min)")
    ap.add_argument("--binding", choices=["wiggle", "fixed", "auto"], default="auto",
                    help="wiggle: bind the roof tag that follows each robot's wiggle (new); fixed: the site's tags.ids "
                         "(the old way); auto: the site's tags.binding, else wiggle")
    ap.add_argument("--lost-policy", choices=["hold", "keep_going"], default="hold",
                    help="hold (cautious, tonight): the tag lost > %.1f s while driving -> stop and wait; keep_going: "
                         "the experiments' policy" % LOST_HOLD_S)
    ap.add_argument("--tours", type=int, default=TOURS_C, help="phase C tours (1-3)")
    ap.add_argument("--session", default="")
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--calib", default="", help="the camera model (default: the site's tagcal/overhead.json)")
    ap.add_argument("--no-focus", action="store_true", help="do not set the camera's focus first")
    ap.add_argument("--quick", action="store_true", help="simulator: fewer turns, a shorter still period, 1 tour of "
                                                          "6 points (code checks)")
    g = ap.add_argument_group("simulator")
    g.add_argument("--dry", action="store_true")
    g.add_argument("--seed", type=int, default=1)
    g.add_argument("--sim-name", default="AL-1DC0")
    g.add_argument("--sim-tag", type=int, default=2)
    g.add_argument("--sim-true", default="0.97,1.03,0.3", help="what the simulated robot really needs: wheel_scale,"
                                                              "gyro_scale,gyro_drift_dps")
    g.add_argument("--sim-sleep", default="1:0.4", help="the simulated robot sleeps MIN minutes, next check-in after "
                                                        "FRAC of it")
    g.add_argument("--cam-z-cm", type=float, default=145.0, help="the simulated camera's lens above the table top "
                   "(145: 7.5 ft above the floor; 228.6: 7.5 ft above the table)")
    g.add_argument("--no-height", action="store_true", help="simulator: the camera reads with NO height correction")
    g.add_argument("--sim-max-s", type=float, default=3600)
    g.add_argument("--sim-gap", action="append", default=[], metavar="T:DUR",
                   help="simulator: the roof tag unread for DUR s from T s after the camera starts (a hand over it)")
    a = ap.parse_args(argv)
    import sites as SI
    if a.site:
        os.environ["MIRLT_SITE"] = a.site
    site = SI.load(a.site or None)
    cal_conf = site.get("calibration") or {}
    if not a.robot and not a.dry:
        a.robot = list(cal_conf.get("robot_ips") or [])     # the site's hint (school: Alvik 3), plus the broadcast
    if a.binding == "auto":
        a.binding = ((site.get("tags") or {}).get("binding") or "wiggle")
    if a.quick:
        global TURNS_A, ROLLS_A, STILL_S, STILL_C_S
        TURNS_A, ROLLS_A, STILL_S, STILL_C_S = (90, -90, 170, -170), (20.0, 40.0), 20.0, 10.0
    folder = session_dir(a)
    os.makedirs(folder, exist_ok=True)
    say("Calibration run %s, session %s" % (VERSION, folder))
    run = CalibRun(a, site, folder)
    if a.quick:
        run.S_quick = True
        z = run.S.zone12()
        run.S.zone12 = lambda: ["P10", "P14", "P17", "P12", "P20", "P13"] if z else z
    procs = []

    def stop(*_):
        run.stop_flag.set()
        try:
            for rb in getattr(run, "robots", []):
                rb.link.send(rb.ip, "stop")
        except Exception:  # noqa: BLE001
            pass
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    cam_c, tag_h = None, None
    try:
        if a.dry:
            loopback_up()
            sp, ips = start_sim(a, run.S, folder)
            procs.append(sp)
            a.robot = a.robot or ips
            cam = SchoolCamera(z_cm=a.cam_z_cm)
            cam_c = cam.C.tolist()
            tag_h = 6.19
            run.sim_cam = cam
            run.sim_true = dict(zip(PARAMS, (float(v) for v in a.sim_true.split(","))))
            eye_factory = lambda: SimEye(folder, FIX_PORT + 100, cam, {a.sim_name: a.sim_tag}, tag_h=tag_h,  # noqa
                                         correct=not a.no_height, seed=a.seed, gaps=a.sim_gap)
        else:
            if not a.no_focus:
                set_focus()
            bp, cal, tag_h = start_bridge(a, site, folder)
            procs.append(bp)
            cam_c = cam_centre_from_model(cal)
            eye_factory = lambda: Eye(folder, FIX_PORT)    # noqa: E731
        run.cam_centre, run.tag_h = cam_c, tag_h
        result = run.run(eye_factory)
    except (RunEnd, BindError) as e:
        result = "refused: %s" % e
        say("STOPPED: %s" % e)
        run.result = result
    finally:
        for p in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        for c in ("eye", "link"):
            o = getattr(run, c, None)
            if o:
                o.close()
        for f in (run.legs_f, run.raw_f, run.cor_f, run.events):
            f.close()
    import calib_report as CR
    try:
        words = CR.write(folder, run, a)
    except Exception as e:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        words = "the report could not be written: %s" % e
    run.set_status(phase="done", result=result, words=words, ended=time.time())
    say("RESULT: " + words)
    say("Report: %s" % os.path.join(folder, "report.md"))
    return 0 if result == "finished" else 1


def set_focus(dev="/dev/video0"):
    """The C920e's focus: autofocus off, focus_absolute 0 (both reset when it is unplugged or the Pi restarts)."""
    for ctrl in ("focus_automatic_continuous=0", "focus_auto=0", "focus_absolute=0"):
        try:
            subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl=%s" % ctrl], capture_output=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            say("(v4l2-ctl not found: the camera's focus was not set. ./run-fleet-pi.sh school-setup installs it)")
            return False
    return True


if __name__ == "__main__":
    sys.exit(main())
