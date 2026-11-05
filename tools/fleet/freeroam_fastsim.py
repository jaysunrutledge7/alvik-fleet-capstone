#!/usr/bin/env python3
"""Free roam stress simulator (2026-11-05): the free-roam controller (freeroam_core.Controller, the same code the Pi
runs) against a kinematic model of three robots, many times faster than real time, for many seeds and hours.

What it models (each switchable, all on by default; numbers below, all CHOSEN to be harsher than the home logs):
  robots     9.5 cm squares; turn on the spot at up to 92 deg/s (slower near the end), drive with the firmware's speed
             ramp (drive_accel 20) and top speed 12.5 cm/s; each robot's own pose (wheels + gyro) with a wheel-scale
             error (+-3 %) and a gyro-scale error (+-1.5 %), pulled to the camera's fix as the firmware's fusion does;
             a bad turn now and then (1 in 50 ends 8-20 deg off); the step queue, acks, fspd / fsteer (new firmware)
             or halt + resend (old), the front sensor stop (follow_look_cm 8, path +-6 cm), the watchdog (1.5 s)
  network    each packet both ways 5-40 ms late, PACKET_LOSS lost (default 3 %)
  camera     a fix per robot 12 times a second, 0.5 cm noise, 0.12-0.2 s late, a heading only now and then;
             tag dropouts: about one every 8 minutes per robot, 2-25 s long (the camera does not see it)
  faults     one robot stops answering (dead where it stands) at a random time in some runs; the Pi frozen for 4 s
             (controller silent: the watchdog should stop every robot) once per run
  battery    100 % at the start, 24 %/h moving, 6 %/h standing (the 2 Nov low-battery run: about 14 %/h idle with the
             lights on; CHOSEN, the real curve is one of the run's outputs)
Truth it measures: the gap between the two square bodies (exact polygon distance) every step; contact = overlap.

    python3 tools/fleet/freeroam_fastsim.py --hours 4 --seeds 1-8 --out logs/freeroam-stress
    python3 tools/fleet/freeroam_fastsim.py --hours 0.5 --seeds 1 --old-fw          # the halt + resend method
    python3 tools/fleet/freeroam_fastsim.py summarize logs/freeroam-stress/*.jsonl --png=gaps.png
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import freeroam_core as FC   # noqa: E402

DT = 0.05
TICK = 0.05
BOARD = (0.0, 0.0, 122.24, 121.92)
PACKET_LOSS = 0.03
TURN_MAX_DPS = 92.0
V_TOP = 12.5


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def corners(x, y, th):
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    h = FC.BODY / 2
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in ((h, h), (-h, h), (-h, -h), (h, -h))]


def _pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _inside(p, poly):
    sg = None
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if sg is None:
            sg = cr > 0
        elif (cr > 0) != sg:
            return False
    return True


def body_gap(A, B):
    """Distance between two square bodies (poses (x, y, th)); 0 if they overlap."""
    if math.hypot(A[0] - B[0], A[1] - B[1]) > 2 * FC.HALF + 0.01:
        return math.hypot(A[0] - B[0], A[1] - B[1]) - 2 * FC.HALF + 0.0   # lower bound is enough far apart
    pa, pb = corners(*A), corners(*B)
    if any(_inside(p, pb) for p in pa) or any(_inside(p, pa) for p in pb):
        return 0.0
    d = min(_pt_seg(p, pb[i], pb[(i + 1) % 4]) for p in pa for i in range(4))
    return min(d, min(_pt_seg(p, pa[i], pa[(i + 1) % 4]) for p in pb for i in range(4)))


class Net:
    def __init__(self, rng, loss, lat_max=0.04):
        self.rng, self.loss, self.q, self.lat_max = rng, loss, [], lat_max

    def send(self, t, fn, *args):
        if self.rng.random() < self.loss:
            return
        self.q.append((t + self.rng.uniform(0.005, self.lat_max), fn, args))

    def due(self, t):
        if not self.q:
            return []
        out = [e for e in self.q if e[0] <= t]
        if out:
            self.q = [e for e in self.q if e[0] > t]
            out.sort(key=lambda e: e[0])
        return out


class Bot:
    def __init__(self, name, x, y, th, rng, new_fw):
        self.name, self.rng = name, rng
        self.x, self.y, self.th = x, y, th          # truth
        self.ox, self.oy, self.oth = x, y, th       # own pose
        self.wheel_err = rng.uniform(-0.03, 0.03)
        self.gyro_err = rng.uniform(-0.015, 0.015)
        self.q, self.cur, self.max_id = [], None, 0
        self.last_done = self.last_failed = 0
        self.v, self.scale, self.steer = 0.0, 1.0, None
        self.state = "idle"
        self.battery = 100.0
        self.dead = False
        self.last_rx = 0.0
        self.new_fw = new_fw
        self.turn_target = None
        self.started = False
        self.following = True

    def report(self, sim, sid, st, info=""):
        if st == "done":
            self.last_done = sid
        elif st == "failed" and sid:
            self.last_failed = sid
        sim.up(self, "step", sid, st, info)


class Sim:
    def __init__(self, seed, hours, new_fw=True, dead_robot=True, pi_freeze=True, dropouts=True, loss=PACKET_LOSS,
                 lost_mode="reckon", out_dir=None, names=("Left", "Center", "Right"), lat_max=0.04):
        self.rng = random.Random(seed * 7919 + 13)
        self.seed, self.hours = seed, hours
        self.t = 0.0
        self.down = Net(random.Random(seed * 31 + 1), loss, lat_max)
        self.upn = Net(random.Random(seed * 31 + 2), loss, lat_max)
        self.rows = {}
        self.out_dir = out_dir
        self.writers = {}
        self.names = list(names)
        # start spots: 30 cm in, 40 apart, random headings (as the ladder's dry board)
        while True:
            pts = [(self.rng.uniform(30, 92), self.rng.uniform(30, 92)) for _ in names]
            if all(math.dist(pts[i], pts[j]) >= 40 for i in range(3) for j in range(i + 1, 3)):
                break
        self.bots = {n: Bot(n, p[0], p[1], self.rng.uniform(-180, 180), random.Random(seed * 101 + k), new_fw)
                     for k, (n, p) in enumerate(zip(names, pts))}
        self.ctrl = FC.Controller(names, BOARD, self.ctrl_out, self.ctrl_log, seed=seed, lost_mode=lost_mode)
        self.frozen_until = -1.0
        self.freeze_at = self.rng.uniform(0.2, 0.8) * hours * 3600 if pi_freeze else None
        self.dead_at = (self.rng.uniform(0.15, 0.7) * hours * 3600, self.rng.choice(names)) if dead_robot else None
        self.gaps = {n: [] for n in names}
        if dropouts:
            for n in names:
                t = self.rng.expovariate(1 / 480.0)
                while t < hours * 3600:
                    d = self.rng.uniform(2, 25)
                    self.gaps[n].append((t, t + d))
                    t += d + self.rng.expovariate(1 / 480.0)
        self.cam_q = []
        self.next_fix = {n: self.rng.uniform(0, 0.08) for n in names}
        self.next_follow = {n: 0.0 for n in names}
        self.stats = dict(contacts=0, contact_s=0.0, min_gap=float("inf"), gap_windows=[], front_stops=0,
                          watchdog_stops=0, edge_min=float("inf"), off_board=0)
        self.win_min = {}
        self.win_t = 0.0
        self.in_contact = set()

    # -- logs ---------------------------------------------------------------------------------------------------
    def ctrl_log(self, table, row):
        if table == "motion":
            m = self.__dict__.setdefault("mot", {})
            k = m.setdefault(row["t"], [0, 0, 0])
            k[0] += row["any"]
            k[1] += row["translating"]
            k[2] += int(row["mode"] not in ("out", "gone"))
            pr = self.__dict__.setdefault("mot_robot", {}).setdefault(row["robot"], [0, 0, 0])
            if row["mode"] not in ("out", "gone"):
                pr[0] += row["any"]
                pr[1] += row["translating"]
                pr[2] += 1
        self.rows.setdefault(table, 0)
        self.rows[table] += 1
        if table in ("reflex", "fallbacks", "lost", "breaks", "events", "moves") or self.out_dir:
            self.keep(table, row)

    def keep(self, table, row):
        if self.out_dir is None:
            self.__dict__.setdefault("kept", {}).setdefault(table, []).append(row)
            return
        w = self.writers.get(table)
        if w is None:
            f = open(os.path.join(self.out_dir, table + ".csv"), "w", newline="")
            w = csv.DictWriter(f, fieldnames=list(row.keys()), extrasaction="ignore")
            w.writeheader()
            self.writers[table] = w
            self.__dict__.setdefault("files", []).append(f)
        w.writerow(row)

    # -- controller -> robots -----------------------------------------------------------------------------------
    def ctrl_out(self, name, text):
        if self.t < self.frozen_until:
            return
        self.down.send(self.t, self.robot_cmd, name, text)

    def up(self, b, kind, *args):
        if b.dead:
            return
        self.upn.send(self.t, self.ctrl_in, b.name, kind, args)

    def ctrl_in(self, name, kind, args):
        c = self.ctrl
        c.on_heard(name, self.t)
        if kind == "step":
            c.on_step(name, *args)
        elif kind == "follow":
            c.on_follow(name, self.t, *args)
        elif kind == "status":
            c.on_fw(name, args[0])
            c.on_battery(name, args[1], self.t)
        elif kind == "fspd":
            c.on_fspd(name)

    def robot_cmd(self, name, text):
        b = self.bots[name]
        if b.dead:
            return
        b.last_rx = self.t
        parts = text.split()
        verb = parts[0]
        if verb == "status":
            self.up(b, "status", "2026-11-05b" if b.new_fw else "2026-10-29b", int(b.battery))
        elif verb == "fix":
            pass
        elif verb == "fpose":
            b.ox, b.oy, b.oth = float(parts[1]), float(parts[2]), float(parts[3])
        elif verb == "halt":
            for s in b.q:
                b.report(self, s["id"], "dropped", "halt")
            b.q = []
            if b.cur:
                b.report(self, b.cur["id"], "failed", "halted")
                b.cur, b.state = None, "idle"
            b.v = 0.0
            b.report(self, 0, "halted")
        elif verb == "hold":
            for s in b.q:
                b.report(self, s["id"], "dropped", "hold")
            b.q = []
            b.report(self, 0, "held")
        elif verb in ("fspd", "fsteer"):
            if not b.new_fw:
                return
            if verb == "fspd":
                b.scale = float(parts[1])
            else:
                b.steer = (float(parts[1]), float(parts[2]))
            self.up(b, "fspd")
        elif verb == "stop":
            b.q, b.cur, b.v, b.state, b.following = [], None, 0.0, "idle", False
        elif verb == "sleep":
            b.dead = True                      # asleep: deaf and still (it is parked: the run is over for it)
        elif verb == "fs":
            sid = int(parts[1])
            if sid <= b.max_id:
                b.report(self, sid, "dup")
                return
            if len(b.q) >= 8:
                b.report(self, sid, "full")
                return
            b.max_id = sid
            typ = parts[2]
            if typ == "T":
                b.q.append(dict(id=sid, type="T", h=float(parts[3])))
            elif typ == "D":
                v = [float(q) for q in parts[3:9]]
                b.q.append(dict(id=sid, type="D", x0=v[0], y0=v[1], x1=v[2], y1=v[3], h=v[4], v=v[5]))
            b.report(self, sid, "queued")

    # -- physics ------------------------------------------------------------------------------------------------
    def bot_step(self, b):
        if b.dead or not b.following:
            return
        moving = b.state in ("turn", "drive")
        b.battery -= (24.0 if moving else 6.0) * DT / 3600.0
        if b.battery < 9.5:
            b.q, b.v = [], 0.0          # the motor board stops the motors under 10 %
            if b.cur:
                b.report(self, b.cur["id"], "failed", "motors off (battery)")
                b.cur, b.state = None, "idle"
            return
        if b.new_fw and b.cur and self.t - b.last_rx > FC.WD_MS / 1000.0:
            b.report(self, b.cur["id"], "failed", "watchdog")
            for s in b.q:
                b.report(self, s["id"], "dropped", "an earlier step failed")
            b.q, b.cur, b.v, b.state = [], None, 0.0, "idle"
            self.stats["watchdog_stops"] += 1
            return
        if b.cur is None:
            if not b.q:
                b.state = "idle"
                return
            if b.new_fw and self.t - b.last_rx > FC.WD_MS / 1000.0:
                return
            b.cur = b.q.pop(0)
            b.scale, b.steer = 1.0, None
            b.state = "turn" if b.cur["type"] == "T" else "drive"
            b.report(self, b.cur["id"], "started")
            if b.cur["type"] == "T":
                bad = b.rng.random() < 0.02
                b.cur["err"] = (b.rng.uniform(8, 20) * b.rng.choice((-1, 1))) if bad else b.rng.gauss(0, 1.5)
            else:
                if abs(wrap180(b.cur["h"] - b.oth)) > 10:
                    b.cur["square"] = True
            b.v = 0.0
        s = b.cur
        if s["type"] == "T" or s.get("square"):
            tgt = s["h"]
            err = wrap180(tgt - b.oth)
            if abs(err) <= 1.5:
                if s["type"] == "T":
                    b.th = wrap180(b.th + s.get("err", 0.0))
                    b.report(self, s["id"], "done")
                    b.cur, b.state = None, "idle"
                else:
                    s["square"] = False
                return
            w = max(25.0, min(TURN_MAX_DPS, 2.3 * abs(err))) * (1 if err > 0 else -1)
            d = w * DT
            if abs(d) > abs(err):
                d = err
            b.oth = wrap180(b.oth + d)
            b.th = wrap180(b.th + d * (1 + b.gyro_err))
            return
        # drive: on the own pose, as the firmware's driveLine
        if b.steer is not None:
            s["x1"], s["y1"] = b.steer
            s["h"] = math.degrees(math.atan2(b.steer[1] - b.oy, b.steer[0] - b.ox))
            b.steer = None
        ux, uy = math.cos(math.radians(s["h"])), math.sin(math.radians(s["h"]))
        rem = (s["x1"] - b.ox) * ux + (s["y1"] - b.oy) * uy
        if rem <= b.v * 0.03 + 0.05:
            b.v = 0.0
            b.report(self, s["id"], "done")
            b.cur, b.state = None, "idle"
            return
        # front sensor: another body in the path within follow_look_cm (8) of the front face
        for o in self.bots.values():
            if o is b:
                continue
            dx, dy = o.x - b.x, o.y - b.y
            c, sn = math.cos(math.radians(b.th)), math.sin(math.radians(b.th))
            fwd, lat = dx * c + dy * sn, -dx * sn + dy * c
            if 0 < fwd - 4.75 - 4.75 < 8.0 and abs(lat) < 6.0 + 4.75 and b.v > 0.5:
                b.v = 0.0
                b.report(self, s["id"], "failed", "obstacle %.1f cm ahead" % (fwd - 9.5))
                for q in b.q:
                    b.report(self, q["id"], "dropped", "an earlier step failed")
                b.q, b.cur, b.state = [], None, "idle"
                self.stats["front_stops"] += 1
                return
        sc = max(0.0, min(1.5, b.scale))
        vt = min(s["v"] * sc, V_TOP, math.sqrt(2 * 20.0 * max(rem, 0)) + 2.0 * min(sc, 1.0))
        b.v = min(vt, b.v + 20.0 * DT + 0.5)
        cross = ux * (b.oy - s["y1"]) - uy * (b.ox - s["x1"])
        corr = max(-15.0, min(15.0, -3.0 * cross)) if rem > 3 else 0.0
        herr = wrap180(s["h"] + corr - b.oth)
        w = max(-60.0, min(60.0, 4.0 * herr))
        b.oth = wrap180(b.oth + w * DT)
        b.th = wrap180(b.th + w * DT * (1 + b.gyro_err))
        d = b.v * DT
        b.ox += d * math.cos(math.radians(b.oth))
        b.oy += d * math.sin(math.radians(b.oth))
        b.x += d * (1 + b.wheel_err) * math.cos(math.radians(b.th))
        b.y += d * (1 + b.wheel_err) * math.sin(math.radians(b.th))

    def cam_gap(self, n, t):
        return any(a <= t < b for a, b in self.gaps[n])

    def camera(self):
        for n, b in self.bots.items():
            if self.t >= self.next_fix[n]:
                self.next_fix[n] = self.t + 1 / 12.0
                if self.cam_gap(n, self.t):
                    continue
                lat = self.rng.uniform(0.12, 0.2)
                fx, fy = b.x + self.rng.gauss(0, 0.5), b.y + self.rng.gauss(0, 0.5)
                fth = b.th + self.rng.gauss(0, 2.0) if (b.state == "drive" and self.rng.random() < 0.3) else None
                self.cam_q.append((self.t + lat, n, self.t, fx, fy, fth))
        due = [e for e in self.cam_q if e[0] <= self.t]
        if due:
            self.cam_q = [e for e in self.cam_q if e[0] > self.t]
            for _, n, tf, fx, fy, fth in due:
                self.ctrl.on_fix(n, tf, fx, fy, fth)
                # the runner forwards each fix to the robot (fix command): the robot's fusion pulls its pose over
                if self.t >= self.frozen_until:
                    self.ctrl.sent(n)
                    self.down.send(self.t, self.fuse, n, fx, fy, fth)

    def fuse(self, n, fx, fy, fth):
        b = self.bots[n]
        if b.dead:
            return
        b.last_rx = self.t
        b.ox += 0.35 * (fx - b.ox)
        b.oy += 0.35 * (fy - b.oy)
        if fth is not None:
            b.oth = wrap180(b.oth + 0.3 * wrap180(fth - b.oth))
        # the camera knows where it is: the own pose's heading error is pulled in slowly too (fixes along the line)
        b.oth = wrap180(b.oth + 0.02 * wrap180(b.th - b.oth))

    def follow_records(self):
        for n, b in self.bots.items():
            if self.t >= self.next_follow[n] and not b.dead and b.following:
                self.next_follow[n] = self.t + 0.1
                self.up(b, "follow", b.state, b.ox, b.oy, b.oth, b.cur["id"] if b.cur else 0, b.last_done,
                        b.last_failed)

    def truth(self):
        bs = list(self.bots.values())
        for i in range(len(bs)):
            a = bs[i]
            e = FC.edge_dist(BOARD, a.x, a.y) - FC.HALF
            self.stats["edge_min"] = min(self.stats["edge_min"], e + FC.HALF - FC.BODY / 2 * 1.0)
            if FC.edge_dist(BOARD, a.x, a.y) < FC.BODY / 2:
                self.stats["off_board"] += 1
            for j in range(i + 1, len(bs)):
                b = bs[j]
                g = body_gap((a.x, a.y, a.th), (b.x, b.y, b.th))
                key = (a.name, b.name)
                self.win_min[key] = min(self.win_min.get(key, float("inf")), g)
                if g < self.stats["min_gap"]:
                    self.stats["min_gap"] = g
                    self.stats["min_gap_at"] = dict(t=round(self.t, 2), a=a.name, b=b.name, a_state=a.state,
                                                    b_state=b.state, a_dead=a.dead, b_dead=b.dead,
                                                    a_mode=self.ctrl.R[a.name].mode, b_mode=self.ctrl.R[b.name].mode,
                                                    a_lost=self.ctrl.R[a.name].lost is not None,
                                                    b_lost=self.ctrl.R[b.name].lost is not None,
                                                    a_kind=(self.ctrl.R[a.name].move or {}).get("kind"),
                                                    b_kind=(self.ctrl.R[b.name].move or {}).get("kind"))
                if g <= 0.0:
                    self.stats["contact_s"] += DT
                    if key not in self.in_contact:
                        self.in_contact.add(key)
                        self.stats["contacts"] += 1
                        self.keep("contacts", dict(t=round(self.t, 2), a=a.name, b=b.name, ax=round(a.x, 1),
                                                   ay=round(a.y, 1), bx=round(b.x, 1), by=round(b.y, 1)))
                elif g > 0.5:
                    self.in_contact.discard(key)
        if self.t - self.win_t >= 10.0:
            for k, v in self.win_min.items():
                self.stats["gap_windows"].append(v)
            self.win_min, self.win_t = {}, self.t

    def run(self):
        c = self.ctrl
        T = self.hours * 3600.0
        for n, b in self.bots.items():
            c.on_fw(n, "2026-11-05b" if b.new_fw else "2026-10-29b")
            c.on_battery(n, 100, 0.0)
            c.on_follow(n, 0.0, "idle", b.ox, b.oy, b.oth, 0)
            c.on_fix(n, 0.0, b.x, b.y, b.th)
            c.on_heard(n, 0.0)
        c.tick(0.0)
        for n in self.names:
            c.begin(n)
        next_tick = 0.0
        while self.t < T:
            self.t += DT
            if self.freeze_at is not None and self.t >= self.freeze_at:
                self.frozen_until = self.t + 4.0
                c.event(None, "pi frozen (sim)", "the controller sends nothing for 4 s")
                self.freeze_at = None
            if self.dead_at is not None and self.t >= self.dead_at[0]:
                b = self.bots[self.dead_at[1]]
                b.dead, b.v, b.state = True, 0.0, "idle"
                c.event(c.R[b.name], "dead (sim)", "stops answering where it is")
                self.dead_at = None
            for b in self.bots.values():
                self.bot_step(b)
            self.camera()
            self.follow_records()
            for _, fn, args in self.down.due(self.t):
                fn(*args)
            for _, fn, args in self.upn.due(self.t):
                fn(*args)
            if self.t >= next_tick:
                next_tick = self.t + TICK
                if self.t >= self.frozen_until:
                    c.tick(self.t)
            self.truth()
            if c.all_out():
                break
        for f in getattr(self, "files", []):
            f.close()
        return self.summary()

    def summary(self):
        c, st = self.ctrl, self.stats
        h = self.t / 3600.0
        kept = getattr(self, "kept", {})
        refl = kept.get("reflex", [])
        gw = sorted(st["gap_windows"])

        def pct(q):
            return round(gw[min(len(gw) - 1, int(q * len(gw)))], 2) if gw else None
        mot = None
        m = getattr(self, "mot", {})
        if m:
            n = len(m)
            mot = dict(any_ge2=round(100.0 * sum(1 for v in m.values() if v[0] >= 2) / n, 1),
                       any_3=round(100.0 * sum(1 for v in m.values() if v[0] >= 3) / n, 1),
                       trans_ge2=round(100.0 * sum(1 for v in m.values() if v[1] >= 2) / n, 1),
                       trans_3=round(100.0 * sum(1 for v in m.values() if v[1] >= 3) / n, 1),
                       robot={k: (round(100.0 * v[0] / max(1, v[2]), 1), round(100.0 * v[1] / max(1, v[2]), 1))
                              for k, v in getattr(self, "mot_robot", {}).items()})
        hist = [0] * 41                          # 10 s windows by their smallest body gap: 1 cm bins, 40+ last
        for g in gw:
            hist[min(40, int(g))] += 1
        return dict(seed=self.seed, hours=round(h, 3), gap_hist=hist, contacts=st["contacts"], contact_s=round(st["contact_s"], 2),
                    min_gap_cm=round(st["min_gap"], 2), min_gap_at=st.get("min_gap_at"), gap_p1=pct(0.01), gap_p5=pct(0.05), gap_p50=pct(0.5),
                    reflex=dict(c.counts["reflex"]), fallbacks=dict(c.counts["fallback"]),
                    trophy_breaks=c.counts["breaks"], front_stops=st["front_stops"],
                    watchdog_stops=st["watchdog_stops"], moves=sum(r.n_moves for r in c.R.values()),
                    out={n: (r.out_why, round(r.battery or 0)) for n, r in c.R.items()},
                    edge_min_cm=round(st["edge_min"], 2), off_board=st["off_board"],
                    lost_episodes=len(kept.get("lost", [])),
                    slow_moves=sum(1 for e in kept.get("events", []) if e["kind"] == "slow move"),
                    lost_reports=sum(1 for e in kept.get("events", []) if e["kind"] == "lost report"),
                    break_why=Counter_sum({e["why"].split(":")[0]: 1} for e in kept.get("breaks", [])),
                    reflex_outcomes={k: sum(1 for e in refl if e["outcome"] == k) for k in set(e["outcome"] for e in refl)},
                    menu_cells_used=sum(1 for v in c.menu.n.values() if v), menu_min=min(c.menu.n.values()),
                    menu_max=max(c.menu.n.values()), motion=mot)


def summarize(files, png=None):
    """The stress runs' results per group (one .jsonl per group), per simulated hour, as a markdown table; png: the
    distribution of the smallest body gap per 10 s window, all runs together."""
    rows, hist = [], [0] * 41
    for fp in files:
        rs = [json.loads(l) for l in open(fp) if l.startswith("{")]
        if not rs:
            continue
        H = sum(r["hours"] for r in rs)
        tot = lambda f: sum(f(r) for r in rs)      # noqa: E731
        for r in rs:
            for k, v in enumerate(r.get("gap_hist") or []):
                hist[k] += v
        mo = [r["motion"] for r in rs if r.get("motion")]
        rows.append(dict(
            group=os.path.basename(fp).rsplit(".", 1)[0], runs=len(rs), hours=round(H, 1),
            contacts=tot(lambda r: r["contacts"]), min_gap=min(r["min_gap_cm"] for r in rs),
            p1=min(r["gap_p1"] for r in rs if r["gap_p1"] is not None),
            speed=tot(lambda r: r["reflex"].get("speed", 0)) / H, steer=tot(lambda r: r["reflex"].get("steer", 0)) / H,
            replan=tot(lambda r: r["reflex"].get("replan", 0)) / H, stop=tot(lambda r: r["reflex"].get("stop", 0)) / H,
            breaks=tot(lambda r: r["trophy_breaks"]) / H,
            fallbacks=tot(lambda r: sum(v for k, v in r["fallbacks"].items() if k != "find-me wiggle")) / H,
            wiggles=tot(lambda r: r["fallbacks"].get("find-me wiggle", 0)) / H,
            front=tot(lambda r: r["front_stops"]) / H, watchdog=tot(lambda r: r["watchdog_stops"]),
            off_board=tot(lambda r: r["off_board"]), edge_min=min(r["edge_min_cm"] for r in rs),
            any2=sum(m["any_ge2"] for m in mo) / max(1, len(mo)), any3=sum(m["any_3"] for m in mo) / max(1, len(mo)),
            tr2=sum(m["trans_ge2"] for m in mo) / max(1, len(mo)), tr3=sum(m["trans_3"] for m in mo) / max(1, len(mo)),
            moves=tot(lambda r: r["moves"]) / H, slow=tot(lambda r: r.get("slow_moves", 0)) / H, outcomes=Counter_sum(r.get("reflex_outcomes", {}) for r in rs)))
    L = ["| group | runs | sim h | contacts | smallest gap cm | 1st pct of 10 s minima cm | speed /h | steer /h | "
         "new move /h | hard stop /h | full stops /h | fallbacks /h | find-me /h | front stops /h | watchdog stops | "
         "off board | closest to edge cm | >=2 moving % (driving %) | 3 moving % (driving %) | moves /h | slow moves /h |",
         "|" + "---|" * 21]
    for r in rows:
        L.append("| %s | %d | %.1f | %d | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.1f | %.1f | %.2f | %d | %d "
                 "| %.1f | %.1f (%.1f) | %.1f (%.1f) | %.0f | %.2f |" % (
                     r["group"], r["runs"], r["hours"], r["contacts"], r["min_gap"], r["p1"], r["speed"], r["steer"],
                     r["replan"], r["stop"], r["breaks"], r["fallbacks"], r["wiggles"], r["front"], r["watchdog"],
                     r["off_board"], r["edge_min"], r["any2"], r["tr2"], r["any3"], r["tr3"], r["moves"], r["slow"]))
    if png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 3.4))
        ax.bar(range(41), hist, color="#2952CC")
        ax.set_yscale("log")
        ax.set_xlabel("smallest gap between two robots' bodies in a 10 s window (cm; 40 = 40 or more)")
        ax.set_ylabel("10 s windows (log scale)")
        ax.set_title("How close the robots came: %d windows, %.0f simulated hours" % (
            sum(hist), sum(r["hours"] for r in rows)))
        fig.text(0.01, 0.01, "Measured: the true body gap (exact square-to-square distance) in the stress simulator, "
                 "every pair, every 0.05 s; source: tools/fleet/freeroam_fastsim.py runs, logs/freeroam-stress",
                 fontsize=7, color="#555")
        fig.tight_layout(rect=(0, 0.05, 1, 1))
        fig.savefig(png, dpi=110)
        plt.close(fig)
    return "\n".join(L), rows, hist


def Counter_sum(ds):
    out = {}
    for d in ds:
        for k, v in d.items():
            out[k] = out.get(k, 0) + v
    return out


def main(argv=None):
    if argv is None and len(sys.argv) > 1 and sys.argv[1] == "summarize":
        png = None
        fs = [f for f in sys.argv[2:] if not f.startswith("--png=")]
        for f in sys.argv[2:]:
            if f.startswith("--png="):
                png = f[6:]
        print(summarize(fs, png)[0])
        return 0
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hours", type=float, default=1.0)
    ap.add_argument("--seeds", default="1")
    ap.add_argument("--out", default="")
    ap.add_argument("--old-fw", action="store_true", help="robots on 2026-10-29b: no fspd / fsteer / watchdog")
    ap.add_argument("--loss", type=float, default=PACKET_LOSS)
    ap.add_argument("--lat-max-ms", type=float, default=40.0, help="each packet 5 ms to this late (default 40)")
    ap.add_argument("--no-dead", action="store_true")
    ap.add_argument("--no-freeze", action="store_true")
    ap.add_argument("--no-dropouts", action="store_true")
    ap.add_argument("--lost-mode", default="reckon", choices=("reckon", "hold"))
    a = ap.parse_args(argv)
    lo, _, hi = a.seeds.partition("-")
    seeds = range(int(lo), int(hi or lo) + 1)
    res = []
    for sd in seeds:
        od = None
        if a.out:
            od = os.path.join(a.out, "seed%02d%s" % (sd, "-oldfw" if a.old_fw else ""))
            os.makedirs(od, exist_ok=True)
        t0 = time.time()
        s = Sim(sd, a.hours, new_fw=not a.old_fw, dead_robot=not a.no_dead, pi_freeze=not a.no_freeze,
                dropouts=not a.no_dropouts, loss=a.loss, lost_mode=a.lost_mode, out_dir=od,
                lat_max=a.lat_max_ms / 1000.0)
        r = s.run()
        r["wall_s"] = round(time.time() - t0, 1)
        r.update(old_fw=a.old_fw, loss=a.loss, lat_max_ms=a.lat_max_ms, lost_mode=a.lost_mode)
        print(json.dumps(r), flush=True)
        res.append(r)
        if od:
            json.dump(r, open(os.path.join(od, "summary.json"), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
