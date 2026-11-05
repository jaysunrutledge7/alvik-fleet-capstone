#!/usr/bin/env python3
"""Home free roam (MIRLT, 2026-11-05): Left, Center and Right moving at the same time, all the time, on the home board
until each battery reaches 10 % (or the time cap, 4 h), unattended on the home Pi, then everything uploaded.

Nick, 5 Nov: "Continuous simultaneous movement of multiple robots is the accomplishment, that's a trophy. Everything
else is bonus." 11:23: "the trophy is shiniest if they never stop moving".

    python3 tools/fleet/freeroam.py run [--hours 4] --robot IP ... --stream 127.0.0.1:8000 --calib C.json
    python3 tools/fleet/freeroam.py run --dry [--hours 0.5] [--sim-minutes 30]     the simulator (fleet_sim.py), no robots
    python3 tools/fleet/freeroam.py status [--dir logs/<...>-freeroam]
    python3 tools/fleet/freeroam.py report <folder>                                   the report again
run-fleet-pi.sh starts it in tmux session "freeroam" (./run-fleet-pi.sh freeroam [hours]), so it keeps running when
the SSH window closes or Tailscale is switched to another account.

  start   (robots) the remote ladder's L0 (find the robots by their lights, battery, firmware, the edge guard) and L1
          (calibrate; kept when calibrated today on this power-on), a heading crawl for a robot whose heading is not
          known; (--dry) the simulated board, robots at random spots 30 cm in and 40 cm apart
  roam    the camera bridge (cam_bridge.py, every robot followed from where L0 found it), then the controller
          (freeroam_core.py: balanced menu, reservations, fallbacks, reflex, dead reckoning, battery) at 20 Hz;
          every robot in follow mode, its watchdog on (firmware 2026-11-05b), its edge guard and own border on
  end     every robot parked and stopped at 10 % (asleep: `sleep 0`, switch off and on to wake), or the time cap, or
          the card nearly full (diskguard), or Ctrl+C / freeroam-stop: every robot stops; the logs, the camera track and
          the report (report.md + charts) in logs/<date-time>-freeroam/; run-fleet-pi.sh uploads it.
"""
import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import queue
import random
import re
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import freeroam_core as FC   # noqa: E402

PY = sys.executable
VERSION = "freeroam-runner-2026-11-05a"
HOME_BOARD = (0.0, 0.0, 122.24, 121.92)    # roam.py's board (the home plywood in the floor calibration's cm)
FOLLOW_LOOK_CM = 8.0    # the robots' own front stop while driving: roam.py's 8 (the plywood reads ~10 cm ahead, 12
                        # stopped drives). Another robot's face at the planned gap is 7.9 cm or more away; the reflex
                        # keeps 2.5 cm between the circles (about 6.5 face to face). No DOE: a backstop
FIX_HZ = 10.0           # camera fixes forwarded to each robot a second (run_plan --fix-hz). Not chosen here
TICK_S = 0.05           # the controller's loop (20 a second; the camera bridge sends ~12 fixes a second per robot)
SAFE = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
        "set bumper_cm 0"]                 # the robots' edge guard (ladder.SAFE; not kept over a power-off)
DISK_CHECK_S = 60.0


def say(s=""):
    print(s, flush=True)


class Tables:
    def __init__(self, folder):
        self.dir = folder
        self.w = {}

    def __call__(self, table, row):
        w = self.w.get(table)
        if w is None:
            p = os.path.join(self.dir, table + ".csv")
            f = open(p, "w", newline="", buffering=1)
            w = (csv.DictWriter(f, fieldnames=list(row.keys()), extrasaction="ignore"), f)
            w[0].writeheader()
            self.w[table] = w
        try:
            w[0].writerow(row)
        except ValueError:
            pass

    def close(self):
        for _, f in self.w.values():
            try:
                f.close()
            except OSError:
                pass


class Runner:
    """The controller on a live link: robots (alvik_console.Link) and camera fixes (UDP JSON lines)."""

    def __init__(self, a, outdir, names, ips, rect, fix_port=5010, pose_to=None, frames=None, guard=None):
        import alvik_console as AC
        from run_plan import UdpSource
        self.AC = AC
        self.a, self.dir, self.names = a, outdir, list(names)
        self.rect = rect
        self.q = queue.Queue()
        self.link = AC.Link(outdir, ips)
        self.link.session.quiet = True
        self.link.session.listeners.append(lambda *v: self.q.put((AC.now(),) + v))
        self.source = UdpSource(fix_port)
        self.tables = Tables(outdir)
        self.ctrl = FC.Controller(names, rect, self.send, self.logrow, seed=a.seed, lost_mode=a.lost_mode,
                                  battery_floor=a.battery_floor, park_sleep=not a.no_sleep)
        self.offsets = {n: [] for n in names}
        self.fix_last = {n: 0.0 for n in names}
        self.run_exp = {n: None for n in names}
        self.fw = {}
        self.restarts = {n: 0 for n in names}
        self.track = open(os.path.join(outdir, "camera_track.csv"), "w", buffering=1)
        self.track.write("t,robot,x_cm,y_cm,th_deg\n")
        self.pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if pose_to else None
        self.pose_to = pose_to
        self.frames, self.guard = frames, guard
        self.t0 = None
        self.joined = set()

    # -- out ---------------------------------------------------------------------------------------------------
    def send(self, name, text):
        self.link.send_name(name, text)

    def logrow(self, table, row):
        self.tables(table, row)
        if table == "events":
            k = row.get("kind", "")
            if k not in ("halt",):
                say("[%7.1f] %-7s %s: %s" % (row["t"] - (self.t0 or row["t"]), row["robot"], k, row["text"]))
            if self.frames is not None and k in ("trophy break", "front stop", "stuck", "lost", "find me", "silent",
                                                 "watchdog", "out"):
                self.frames.event(k, row["text"], robot=row["robot"])

    # -- in ----------------------------------------------------------------------------------------------------
    def offset(self, name, t, ms):
        o = self.offsets.setdefault(name, [])
        o.append((t, t - ms / 1000.0))
        if len(o) > 300:
            del o[:100]
        rec = [v for tt, v in o if tt > t - 30.0]
        return min(rec) if rec else None

    def drain(self):
        c = self.ctrl
        while True:
            try:
                t, name, kind, rtype, vals = self.q.get_nowait()
            except queue.Empty:
                return
            if name not in c.R:
                continue
            c.on_heard(name, t)
            if kind == "M":
                low = vals.lower()
                m = re.search(r"\(firmware ([0-9a-z-]+)", vals)
                if m:
                    self.fw[name] = m.group(1)
                    c.on_fw(name, m.group(1))
                m = re.search(r"battery (\d+)%", vals)
                if m:
                    c.on_battery(name, int(m.group(1)), t)
                if "not calibrated" in low:
                    c.event(c.R[name], "not calibrated", vals)
                continue
            try:
                if rtype == "STEP" and len(vals) >= 9:
                    self.offset(name, t, int(float(vals[3])))
                    c.on_step(name, int(vals[1]), vals[2], ",".join(vals[8:]))
                elif rtype == "FOLLOW" and len(vals) >= 10:
                    ms = int(float(vals[1]))
                    self.offset(name, t, ms)
                    if vals[3] != "nopose":
                        c.on_follow(name, t, vals[3], float(vals[7]), float(vals[8]), float(vals[9]), int(vals[2]))
                        if self.pose_sock is not None:
                            off = self.offset(name, t, ms)
                            msg = {"type": "pose", "robot": name, "t": round(ms / 1000.0 + off, 4),
                                   "x": float(vals[7]), "y": float(vals[8]), "th": float(vals[9]), "state": vals[3]}
                            try:
                                self.pose_sock.sendto(json.dumps(msg).encode(), self.pose_to)
                            except OSError:
                                pass
                elif rtype == "RUNINFO" and len(vals) >= 6:
                    self.run_exp[name] = vals[1]
                    self.fw[name] = vals[2]
                    c.on_fw(name, vals[2])
                    try:
                        c.on_battery(name, int(float(vals[5])), t)
                    except ValueError:
                        pass
                elif rtype == "RUNEND" and len(vals) >= 2:
                    if self.run_exp.get(name) == "follow" and c.R[name].mode not in ("out", "gone"):
                        self.run_exp[name] = None
                        c.event(c.R[name], "follow ended", "its follow run ended (%s): started again" % vals[1])
                        self.restarts[name] += 1
                        r = c.R[name]
                        if r.move is not None:
                            c.end_move(r)
                        if self.restarts[name] <= 20:
                            self.send(name, "run follow")
                elif rtype == "FSPD":
                    c.on_fspd(name)
            except (ValueError, IndexError):
                continue

    def fixes(self):
        c = self.ctrl
        now = self.AC.now()
        for ev in self.source.poll():
            if ev.get("type") != "fix":
                continue
            n = ev.get("robot")
            if n not in c.R:
                continue
            th = ev.get("th")
            c.on_fix(n, float(ev["t"]), float(ev["x"]), float(ev["y"]), None if th is None else float(th))
            self.track.write("%.3f,%s,%.2f,%.2f,%s\n" % (ev["t"], n, ev["x"], ev["y"], "" if th is None else
                                                          "%.1f" % th))
            o = self.offsets.get(n) or []
            if not o or now - self.fix_last[n] < 1.0 / FIX_HZ or c.R[n].lost is not None:
                continue
            off = min(v for tt, v in o if tt > now - 30.0) if any(tt > now - 30.0 for tt, _ in o) else o[-1][1]
            ms = int(round((float(ev["t"]) - off) * 1000))
            self.send(n, "fix %d %.2f %.2f %s %.2f %.2f" % (ms, ev["x"], ev["y"], "-" if th is None else "%.2f" % th,
                                                            ev.get("sigma", 1.0), ev.get("sigma_th", 3.0)))
            self.fix_last[n] = now
            c.sent(n)

    def pump(self, s):
        end = time.time() + s
        while time.time() < end:
            self.drain()
            self.fixes()
            self.ctrl.t = self.AC.now()
            time.sleep(0.02)

    # -- start ---------------------------------------------------------------------------------------------------
    def start(self):
        c = self.ctrl
        say("Waiting for the robots: %s ..." % ", ".join(self.names))
        end = time.time() + 30
        while time.time() < end and not set(self.names) <= set(self.link.robots()):
            self.pump(0.3)
        missing = sorted(set(self.names) - set(self.link.robots()))
        if missing:
            say("Not heard from: %s: they stay out of the run." % ", ".join(missing))
            for n in missing:
                c.R[n].mode, c.R[n].out_why = "gone", "not heard at the start"
        live = [n for n in self.names if n not in missing]
        if not live:
            raise SystemExit("No robot answered.")
        for n in live:
            self.send(n, "stop")
        self.pump(1.0)
        for n in live:
            for cmd in SAFE + ["set follow_look_cm %.1f" % FOLLOW_LOOK_CM] + border_cmds(self.rect):
                self.send(n, cmd)
                time.sleep(0.01)
            self.send(n, "status")
        self.pump(2.0)
        for n in live:
            fw = self.fw.get(n) or "?"
            new = fw >= FC.FRESH_FW
            if new:
                self.send(n, "set follow_wd_ms %d" % FC.WD_MS)
            say("  %s: firmware %s, battery %s%%: %s" % (
                n, fw, c.R[n].battery if c.R[n].battery is not None else "?",
                "speed and steer changes while driving, watchdog on" if new else
                "OLDER FIRMWARE: speed and steer changes by halt + resend (a short stop each), no watchdog"))
            c.event(c.R[n], "firmware", "%s (%s)" % (fw, "fspd/fsteer/watchdog" if new else "halt + resend"))
        for attempt in range(3):
            for n in live:
                if self.run_exp.get(n) != "follow":
                    self.send(n, "run follow")
            self.pump(3.0)
            if all(self.run_exp.get(n) == "follow" for n in live):
                break
        for n in live:
            if self.run_exp.get(n) != "follow":
                say("  %s did not start following: out of the run." % n)
                c.R[n].mode, c.R[n].out_why = "gone", "did not start following"
        live = [n for n in live if c.R[n].mode != "gone"]
        say("Waiting for the camera and each robot's own pose ...")
        end = time.time() + 25
        while time.time() < end:
            self.pump(0.3)
            if all(c.R[n].cam is not None and c.R[n].own is not None for n in live):
                break
        for n in live:
            r = c.R[n]
            if r.cam is None or r.own is None:
                say("  %s: %s: out of the run." % (n, "no camera fix" if r.cam is None else "no own pose"))
                r.mode, r.out_why = "gone", "not seen at the start"
                continue
            c.t = self.AC.now()
            c.begin(n)
            self.joined.add(n)
            say("  %s joins at %.1f, %.1f (facing %.0f deg)" % (n, r.cam[1], r.cam[2], r.own[3]))
        if not self.joined:
            raise SystemExit("No robot could join.")

    # -- the roam --------------------------------------------------------------------------------------------------
    def roam(self, hours):
        c = self.ctrl
        self.t0 = self.AC.now()
        t_end = self.t0 + hours * 3600.0
        last_disk = last_say = self.t0
        if self.frames is not None:
            self.frames.active(True, "freeroam")
        say("Free roam: %s, until each is at %d %% or %.1f h. Ctrl+C (or ./run-fleet-pi.sh freeroam-stop) stops "
            "every robot." % (", ".join(sorted(self.joined)), c.floor, hours))
        why = None
        while True:
            now = self.AC.now()
            self.drain()
            self.fixes()
            c.tick(now)
            if c.all_out():
                why = "every robot is out (battery %d %%, or silent)" % c.floor
                break
            if now >= t_end:
                why = "the time cap (%.1f h)" % hours
                break
            if self.guard is not None and now - last_disk > DISK_CHECK_S:
                last_disk = now
                st = self.frames.check_space() if self.frames is not None else self.guard.state()
                if st and st.get("stop"):
                    why = "the Pi's card is nearly full (%s)" % st.get("why")
                    break
            if now - last_say > 60.0:
                last_say = now
                say("[%7.1f] " % (now - self.t0) + "; ".join(
                    "%s %s %s%% %d moves" % (n, r.mode, r.battery if r.battery is not None else "?", r.n_moves)
                    for n, r in c.R.items()))
                self.write_state(why=None)
            time.sleep(max(0.0, TICK_S - (self.AC.now() - now)))
        say("\nThe run ends: %s." % why)
        c.event(None, "end", why)
        return why

    def write_state(self, why):
        c = self.ctrl
        st = dict(version=VERSION, core=FC.VERSION, t0=self.t0, now=self.AC.now(), ended=why, seed=self.a.seed,
                  robots={n: dict(mode=r.mode, out=r.out_why, battery=r.battery, moves=r.n_moves, fw=self.fw.get(n),
                                  lost=r.lost is not None) for n, r in c.R.items()},
                  counts=c.counts)
        tmp = os.path.join(self.dir, "state.json.tmp")
        with open(tmp, "w") as f:
            json.dump(st, f, indent=1, default=str)
        os.replace(tmp, os.path.join(self.dir, "state.json"))

    def close(self):
        for n in self.names:
            try:
                self.link.send_name(n, "stop")
            except Exception:
                pass
        time.sleep(0.5)
        try:
            self.link.close()
        except Exception:
            pass
        self.source.close()
        self.tables.close()
        self.track.close()


def loopback_up():
    """In a fresh network namespace (unshare -rn) the loopback starts down (calib_run.py / sweeps/rp_shim.py's way);
    a no-op where it is up already."""
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


def border_cmds(rect):
    """The robots' own four-sided border 6 cm inside the board (run_plan.border_commands on a plain map)."""
    try:
        import run_plan as RP
        m = {"format": "alvik-fleet-map", "version": 1,
             "floor": {"x_min": rect[0], "y_min": rect[1], "x_max": rect[2], "y_max": rect[3]}, "robots": [],
             "tasks": {}}
        import fleetmap as FM
        return RP.border_commands(FM.normalize(m), RP.BORDER_CM)
    except Exception as e:      # noqa: BLE001  (the edge guard still stands)
        say("  (the robots' own border was not set: %s)" % e)
        return []


# ---------------------------------------------------------------------------------------------- the dry board
def dry_board(a, outdir, names):
    rng = random.Random(a.seed)
    x0, y0, x1, y1 = HOME_BOARD
    while True:
        pts = [(rng.uniform(30, x1 - 30), rng.uniform(30, y1 - 30)) for _ in names]
        if all(math.dist(pts[i], pts[j]) >= 40 for i in range(len(pts)) for j in range(i + 1, len(pts))):
            break
    starts = {n: [round(p[0], 1), round(p[1], 1), round(rng.uniform(-180, 180), 1)] for n, p in zip(names, pts)}
    sd = os.path.join(outdir, "sim")
    os.makedirs(sd, exist_ok=True)
    m = {"format": "alvik-fleet-map", "version": 1, "floor": {"x_min": x0, "x_max": x1, "y_min": y0, "y_max": y1},
         "robots": [{"name": n, "start": s} for n, s in starts.items()], "tasks": {}}
    mp = os.path.join(sd, "dry_map.json")
    json.dump(m, open(mp, "w"), indent=1)
    rate = 90.0 / (a.sim_minutes / 60.0)          # moving: 100 % -> 10 % in --sim-minutes of driving
    cfg = {"robot": {"battery_start_pct": [100.0, 100.0], "battery_moving_pct_per_h": rate,
                     "battery_idle_pct_per_h": rate / 4.0}}
    cp = os.path.join(sd, "sim_config.json")
    json.dump(cfg, open(cp, "w"), indent=1)
    cmd = [PY, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--out", sd, "--seed", str(a.seed), "--quiet",
           "--fixes-to", "127.0.0.1:%d" % a.fix_port, "--fix-hz", "12", "--fix-delay", "0.15", "--fix-sigma", "0.5",
           "--config", cp, "--max-s", str(int(a.hours * 3600 + 300))]
    gaps = []
    T = a.hours * 3600.0
    for n in names:                                # tag dropouts: about one per robot every 6 min, 3-20 s long
        t = rng.uniform(60, 360)
        while t < T - 30:
            d = rng.uniform(3, 20)
            gaps.append("%s@%.0f:%.0f" % (n, t, d))
            t += d + rng.uniform(200, 520)
    for g in gaps:
        cmd += ["--cam-gap", g]
    for fw in a.sim_fw:
        cmd += ["--fw", fw]
    for s in a.sim_silent:
        cmd += ["--silent", s]
    for w in a.sim_wifi_drop:
        cmd += ["--wifi-drop", w]
    json.dump(dict(starts=starts, gaps=gaps, config=cfg, cmd=cmd), open(os.path.join(sd, "dry_setup.json"), "w"),
              indent=1)
    p = subprocess.Popen(cmd, stdout=open(os.path.join(sd, "sim_stdout.txt"), "w"), stderr=subprocess.STDOUT,
                         start_new_session=True)
    time.sleep(2.0)
    return p, ["127.0.0.%d" % (2 + k) for k in range(len(names))], gaps


# ---------------------------------------------------------------------------------------------- the real start
def real_start(a, outdir):
    """L0 + L1 of the remote ladder (as the DOE loop starts), a heading for every robot, then the camera bridge.
    Returns (names, ips, bridge process, frames, guard)."""
    import ladder as LD
    import diskguard as DG
    from ladder import LadderStop
    a.dir = outdir
    L = LD.Ladder(a)                  # a.dry (--dry-chain): the ladder's simulated board (pictures, not fixes)
    a._ladder = L
    st = DG.Guard(card=outdir).state()
    if st["stop"]:
        raise SystemExit("NOT STARTED: %s. Nothing moved. Free space first (./run-fleet-pi.sh space)." % st["why"])
    if not L.run_step("L0", in_order=True):
        raise SystemExit("L0 found no robot that can go on: nothing moved.")
    if not any(L.st["passed"].get("L1:%s" % n) for n in L.active()):
        L.run_step("L1", in_order=True)
    names = [n for n in L.active("L1")]
    if not names:
        raise SystemExit("No robot passed L1 (calibration).")
    sub = L.sub("headings")
    for n in names:
        if L.heading(n) is None:
            say("  %s: heading not known: heading crawl first" % n)
            try:
                L.need_bench(sub)
                L.need_eye()
                q0 = L.eye.still(n) or L.pos(n)
                h, _ = L.crawl(n, q0, None, **L.hint_for(n, q0))
                q = L.eye.still(n) or q0
                L.set_pose(n, q[0], q[1], h, "crawl")
            except LadderStop as e:
                say("  %s: no heading (%s): it stays out of the run, parked where it is." % (n, e))
                L.keep_out(n, "no heading for the free roam: %s" % e)
    names = [n for n in L.active("L1") if L.heading(n) is not None]
    rect = tuple(L.rect)
    L.drop_bench()
    L.drop_eye()
    m = {"format": "alvik-fleet-map", "version": 1,
         "floor": {"x_min": rect[0], "y_min": rect[1], "x_max": rect[2], "y_max": rect[3]},
         "robots": [{"name": n, "start": [*L.pos(n), L.heading(n)]} for n in names], "tasks": {},
         "notes": "home free roam"}
    mp = os.path.join(outdir, "map.json")
    json.dump(m, open(mp, "w"), indent=1)
    bcmd = [PY, os.path.join(HERE, "cam_bridge.py"), "--map", mp, "--calib", L.calib, "--stream", L.stream,
            "--out", os.path.join(outdir, "bridge")]
    for n in names:
        x, y = L.pos(n)
        bcmd += ["--at", "%s=%.2f,%.2f,%.2f" % (n, x, y, L.heading(n))]
    if L.side_calib:
        bcmd += ["--side-stream", L.side_stream, "--side-calib", L.side_calib]
        kb = L.kept_bias()
        if kb is not None:
            bcmd += LD.side_bias_args(kb)
    if a.no_lights:
        bcmd += ["--no-lights"]
    bf = open(os.path.join(outdir, "bridge.txt"), "w")
    br = subprocess.Popen(["nice", "-n", "5"] + bcmd, stdout=bf, stderr=subprocess.STDOUT)
    say("Camera bridge started (%s)." % ", ".join(names))
    time.sleep(5.0)
    frames = guard = None
    if not a.no_frames and not a.dry:
        try:
            import doe_loop as DL
            fdir, _ = DG.frames_dir_for(outdir)
            guard = DG.Guard(card=outdir, frames_dir=fdir)
            streams = [("overhead", L.stream, LD.Eye.OVERHEAD_HZ)]
            if L.side_stream:
                streams.append(("side", L.side_stream, DL.SIDE_HZ))
            frames = DL.Frames(outdir, streams, frames_dir=fdir, guard=guard)
        except Exception as e:     # noqa: BLE001  (the run goes on without frames)
            say("  (frames not saved: %s)" % e)
    if guard is None:
        guard = DG.Guard(card=outdir)
    return names, L.ips, br, frames, guard, rect


# ---------------------------------------------------------------------------------------------- status
def latest_dir(dry=False):
    ds = sorted(glob.glob(os.path.join(REPO, "logs", "*-freeroam" + ("-dryrun" if dry else ""))))
    return ds[-1] if ds else None


def status(folder):
    if not folder or not os.path.isdir(folder):
        say("No free-roam folder found.")
        return 1
    p = os.path.join(folder, "state.json")
    if not os.path.exists(p):
        say("%s: starting (no state yet)." % folder)
        return 0
    st = json.load(open(p))
    mins = (st["now"] - st["t0"]) / 60.0 if st.get("t0") else 0.0
    say("Free roam %s: %.0f min%s" % (os.path.basename(folder), mins, "; ENDED: %s" % st["ended"] if st.get("ended")
                                       else ""))
    for n, r in st["robots"].items():
        say("  %-7s %-8s battery %s%%  moves %d  %s%s" % (n, r["mode"], r["battery"], r["moves"],
                                                       "firmware %s" % r["fw"] if r.get("fw") else "",
                                                       ("  out: %s" % r["out"]) if r.get("out") else ""))
    cn = st.get("counts", {})
    say("  reflex events %s; fallbacks %s; full stops (trophy breaks) %s" % (
        cn.get("reflex"), cn.get("fallback"), cn.get("breaks")))
    return 0


# ---------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["run", "status", "report"])
    ap.add_argument("folder", nargs="?", default="")
    ap.add_argument("--hours", type=float, default=4.0, help="time cap (Nick: 4 h)")
    ap.add_argument("--battery-floor", type=int, default=FC.BATTERY_FLOOR)
    ap.add_argument("--lost-mode", choices=("reckon", "hold"), default="reckon",
                    help="reckon (default): dead reckoning with a growing bubble; hold: the old cautious halt")
    ap.add_argument("--no-sleep", action="store_true", help="a parked robot is stopped, not put to sleep")
    ap.add_argument("--seed", type=int, default=None, help="the menu's seed (default: from the clock; recorded)")
    ap.add_argument("--robots", default="Left,Center,Right")
    ap.add_argument("--dir")
    ap.add_argument("--dry", action="store_true", help="tools/sim/fleet_sim.py instead of the robots and camera")
    ap.add_argument("--dry-chain", action="store_true",
                    help="the real start (ladder L0 + L1, heading crawls, the camera bridge on simulated pictures) "
                         "against the ladder's simulated board: the whole chain, slower")
    ap.add_argument("--sim-minutes", type=float, default=200.0,
                    help="--dry: minutes of driving from 100 %% to 10 %% (time-compressed battery)")
    ap.add_argument("--sim-fw", action="append", default=[], metavar="NAME=VERSION")
    ap.add_argument("--sim-silent", action="append", default=[], metavar="NAME@T:DUR")
    ap.add_argument("--sim-wifi-drop", action="append", default=[], metavar="NAME@T:DUR")
    ap.add_argument("--fix-port", type=int, default=5010)
    ap.add_argument("--no-frames", action="store_true")
    # the ladder's connection (the real start; as doe_loop.py)
    ap.add_argument("--robot", action="append", default=[])
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--calib", default=os.path.join(REPO, "tools", "experiment", "data", "overhead_floorcal_roam1.json"))
    ap.add_argument("--side-stream", default="127.0.0.1:8001")
    ap.add_argument("--side-calib", default="")
    ap.add_argument("--no-lights", action="store_true")
    ap.add_argument("--no-facing", action="store_true")
    ap.add_argument("--ignore", default="")
    ap.add_argument("--attended", action="store_true")
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--day-state", default="")
    ap.add_argument("--turn-err", action="append", default=[])
    ap.add_argument("--sim-washout", action="store_true")
    ap.add_argument("--sim-side-bias", default="")
    ap.add_argument("--sim-no-side", action="store_true")
    ap.add_argument("--sim-calibrated", action="store_true")
    ap.add_argument("--sim-place", action="append", default=[])
    ap.add_argument("--sim-from", default="")
    a = ap.parse_args(argv)
    if a.what == "status":
        return status(a.folder or a.dir or latest_dir(a.dry) or latest_dir(True))
    if a.what == "report":
        import freeroam_report as FR
        say(FR.make(a.folder or a.dir or latest_dir(a.dry)))
        return 0
    if a.seed is None:
        a.seed = int(time.time()) % 100000
    names = [n.strip() for n in a.robots.split(",") if n.strip()]
    outdir = a.dir or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-freeroam" +
                                   ("-dryrun" if a.dry else ""))
    os.makedirs(outdir, exist_ok=True)

    def hup(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGHUP, hup)
    signal.signal(signal.SIGTERM, hup)
    sim = br = frames = guard = None
    runner = None
    why = "stopped before the roam"
    code = 0
    say("Free roam: %s (seed %d, battery floor %d %%, lost: %s)" % (outdir, a.seed, a.battery_floor, a.lost_mode))
    run_info = dict(version=VERSION, core=FC.VERSION, seed=a.seed, hours=a.hours, battery_floor=a.battery_floor,
                    lost_mode=a.lost_mode, dry=a.dry, started=dt.datetime.now().isoformat(timespec="seconds"),
                    argv=sys.argv[1:])
    try:
        if a.dry_chain:
            loopback_up()
            a.dry = True
            names, ips, br, frames, guard, rect = real_start(a, outdir)
            pose_to = ("127.0.0.1", 5011)
        elif a.dry:
            loopback_up()              # run inside `unshare -rn` (its own network namespace): the loopback starts down
            sim, ips, gaps = dry_board(a, outdir, names)
            run_info["sim_gaps"] = gaps
            rect = HOME_BOARD
            pose_to = None
        else:
            names, ips, br, frames, guard, rect = real_start(a, outdir)
            pose_to = ("127.0.0.1", 5011)
        run_info.update(robots=names, rect=rect)
        json.dump(run_info, open(os.path.join(outdir, "run.json"), "w"), indent=1)
        runner = Runner(a, outdir, names, ips, rect, fix_port=a.fix_port, pose_to=pose_to, frames=frames, guard=guard)
        runner.start()
        why = runner.roam(a.hours)
    except KeyboardInterrupt:
        why = "stopped by hand (Ctrl+C, freeroam-stop, or the tmux session closed)"
        say("\n%s: every robot is sent stop." % why)
        code = 130
    except SystemExit as e:
        why = "STOPPED: %s" % e
        say(why)
        code = 1
    finally:
        if runner is not None:
            runner.write_state(why)
            runner.close()
        if br is not None:
            try:
                br.send_signal(signal.SIGINT)
                br.wait(timeout=8)
            except Exception:
                br.kill()
        if frames is not None:
            say("Frames: %s" % frames.words())
            frames.close()
        if getattr(a, "_ladder", None) is not None and a.dry:
            try:
                a._ladder.stop_dry()
            except Exception:
                pass
        if sim is not None:
            try:
                os.killpg(sim.pid, signal.SIGTERM)
                sim.wait(timeout=20)
            except Exception:
                pass
        run_info.update(ended=why, finished=dt.datetime.now().isoformat(timespec="seconds"))
        json.dump(run_info, open(os.path.join(outdir, "run.json"), "w"), indent=1)
        try:
            import freeroam_report as FR
            say("Report: %s" % FR.make(outdir))
        except Exception as e:     # noqa: BLE001
            say("(report not made: %s)" % e)
        say("Free roam folder: %s" % outdir)
    return code


if __name__ == "__main__":
    sys.exit(main())
