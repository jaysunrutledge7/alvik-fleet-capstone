#!/usr/bin/env python3
"""Home free roam, the controller (2026-11-05, Nick: "continuous simultaneous movement of multiple robots is the
accomplishment, that's a trophy"; 11:23: "the trophy is shiniest if they never stop moving").

No sockets and no clock of its own: every input comes in through on_*() with the time, every command goes out through
`out(name, text)`, every log row through `log(table, row)`. tools/fleet/freeroam.py runs it on the Pi (robots and
camera) and against tools/sim/fleet_sim.py; tools/fleet/freeroam_fastsim.py runs it many times faster than real time
for the stress tests. Same code in all three.

Layers, from planning to the last safety net:
  1. MENU      each robot's next move = turn x length x speed from a balanced menu: cells drawn least-used first
               (seeded; FREE_SHARE of the picks in a random order), the first one that is feasible is taken.
  2. PLAN      feasible = the end spot keeps EDGE_CC from every edge (plus its uncertainty) and the move, as a
               reservation in space AND time (turn spot, swept line sampled every SAMPLE_S, then standing at the end
               until its next move), never comes within PLAN_CC + execution margins of another robot's reservation
               in overlapping time windows (+ time slack for latency and timing error).
  3. FALLBACKS no menu move fits: off-menu (any heading, shorter), escape to open space, a holding turn on the spot
               (always possible: it never leaves its own circle). Deadlock (two robots holding next to each other):
               the lower-priority one yields (escape with the gap relaxed to REFLEX_CC). No waiting rung: Nick 11:23.
  4. REFLEX    every camera frame, ACTUAL positions and commanded motion, the next REFLEX_H_S: a predicted gap under
               REFLEX_CC -> the lower-priority mover changes speed (fspd), else steers (fsteer), else the other mover
               does; a hard stop only if nothing keeps it above HARD_CC within HARD_T_S (a "trophy break", logged).
  5. ROBOT     the robots' own front stop (follow_look_cm), edge guard, virtual border, heartbeat watchdog
               (firmware 2026-11-05b follow_wd_ms): their own safety stops, under all of the above.
Lost by the camera: dead reckoning with a growing bubble (BLIND_*), find-me wiggle when it grows too big; the old
cautious hold is kept behind lost_mode="hold".
"""
import math
import random

import numpy as np

VERSION = "freeroam-2026-11-05c"

# ------------------------------------------------------------------------------------------------- body and board
HALF = 6.72             # the 9.5 cm square's half diagonal: a circle round the body however it turns. Not chosen
BODY = 9.5              # the body's side, cm. Not chosen
# ------------------------------------------------------------------------------------------------- chosen numbers
EDGE_CC = 16.0          # chosen: a move's end spot (its middle) at least this far from every edge: the body's corner
                        # 9.3 cm in (roam.py's 3 in + 1.5 cm was 15.8). The robot's own border (6 cm) and edge guard
                        # are under it. DOE: yes, 14 / 16 / 20 cm against edge-guard trips and menu coverage (long
                        # moves need room); docs/doe-backlog.md
TURN_EDGE_CC = HALF + 2.5   # a robot turns on the spot only with its middle this far in (corners swing 6.72 cm).
                            # Follows from the body plus 2.5 cm chosen; no DOE
PLAN_GAP = 4.0          # chosen: planned reservations keep the circles round the bodies this far apart (squares face
                        # to face: 7.9 cm). DOE: yes, 3 / 4 / 6 cm against fallbacks per hour and reflex events
PLAN_CC = 2 * HALF + PLAN_GAP          # 17.4 cm middle to middle, before margins
REFLEX_GAP = 2.5        # chosen: the reflex acts when the predicted circles come closer than this. DOE: yes, with
REFLEX_CC = 2 * HALF + REFLEX_GAP      # PLAN_GAP (16.0 cm middle to middle)
HARD_CC = 2 * HALF + 0.5               # 13.9 cm: under this the circles touch (two squares can first touch at
                                       # 13.44 when corner meets corner). Hard stop only if heading for this
HARD_T_S = 1.2          # chosen: ... within this many seconds. No DOE: a reaction time (stop takes ~0.3 s at 13 cm/s)
REFLEX_H_S = 2.0        # chosen: reflex looks this far ahead (Nick: "~1-2 s"). DOE: 1.5 / 2 / 3 s against events
REFLEX_STEP_S = 0.1
CLEAR_HOLD_S = 1.0      # an adjustment is undone after the nominal prediction is clear (+2 cm) for this long
SPEED_OPTIONS = (0.6, 1.35, 0.35, 1.5, 0.15)   # chosen order: smallest change first, slower before faster
STEER_OPTIONS = (25, -25, 50, -50, 80, -80)    # deg off the current heading; chosen
V_MAX = 12.0            # the wheels' top speed in a straight line is ~12.5 cm/s (70 rpm, 1.7 cm wheels); a faster
                        # setting is not asked for. Not chosen
PLAN_M0 = 1.0           # chosen: execution margin at the start of a move (the camera's ~1 sigma, FIX_SIGMA_CM)
PLAN_M_PER_CM = 0.06    # ... growing this much per cm along the move (turn error 3 deg = 0.05 cm per cm, distance
PLAN_M_MAX = 3.0        # error 1-3 %), capped here: with camera fixes the robots stay within 2-3 cm of the line 95 %
                        # of the time (28 Oct, Phase 0 gate). DOE: yes, against the reflex events per hour
SLACK0_S = 0.6          # time slack of every reservation: camera frame 0.08 s + tracking 0.1 + command and ack
                        # latency with one resend 0.3 + step start 0.1. Chosen from those; DOE: no (a sum)
SLACK_FRAC = 0.15       # ... plus this share of the time since the move was committed (turn time and the speed ramps
                        # are estimates). Chosen; DOE: from moves.csv est_s vs dur_s after the first run
SAMPLE_S = 0.25         # reservation samples (3.3 cm apart at 13 cm/s; PLAN_M0 covers half of that). Not chosen
TURN_DPS = 75.0         # turn time = TURN_S0 + |deg| / TURN_DPS (40 rpm wheels: 92 deg/s, slower near the end).
TURN_S0 = 0.6           # Estimates; moves.csv logs the real times
ACCEL = 20.0            # firmware drive_accel, cm/s per s. Not chosen
# the balanced menu (Nick approved 11:23: "mostly balanced, a small share of free picks")
TURN_BINS = (-150, -90, -45, 0, 45, 90, 150)   # deg (+ = left), each drawn +-15 (0: +-10). Chosen; DOE note below
LEN_BINS = ((10, 20), (20, 35), (35, 55), (55, 80))   # cm. Chosen
SPEED_BINS = (5.0, 7.5, 10.0, 13.0)           # cm/s: the DOE loop's speeds (Nick's figures)
FREE_SHARE = 0.10       # chosen: 1 pick in 10 tries the cells in a random order, not least-used first, so the three
                        # robots do not chase the same rare long cells at once (they cluster on the long diagonals).
                        # DOE: yes, 0 / 10 / 25 % against coverage evenness and fallbacks per hour
PAUSE_S = 0.15          # between moves (the robot's own step queue; no visible stop). Not chosen
# dead reckoning (lost by the camera)
LOST_AFTER_S = 0.6      # no camera fix this long = lost (run_plan.BLIND_AFTER_S). Not chosen here
BLIND_BASE = 2.0        # bubble p95 = 2.0 cm + 0.07 cm per cm driven blind: MEASURED, 29 Oct logs, 22,000 stretches,
BLIND_PER_CM = 0.07     # straight drives up to 40 cm (run_plan.BLIND_ERR_*). Beyond 40 cm extrapolated
BLIND_TURN_FRAC = 0.03  # heading error 3 % of the degrees turned blind (the e03 rerun: 0.85-0.90 deg per turn after the
                        # fit, 6 deg on a 173 deg turn before it): CHOSEN between the two, its lateral effect added
BUBBLE_CAP = 8.0        # chosen: over this the robot eases to a stop and wiggles (find me). DOE: yes, 6 / 8 / 12 cm
REACQUIRE_CM = 2.5      # seen again: its pose is set to the camera's when they differ by more (run_plan). Not chosen
REACQUIRE_MAX_CM = 12.0  # ... over this, 3 agreeing fixes first (the camera may have the wrong robot)
WIGGLE_DEG = 20.0       # find-me wiggle: +-20 deg on the spot. Chosen; no DOE (any visible motion works)
LOST_HOLD_S = 1.5       # lost_mode "hold" (the old roam.py rule): halt a mover unseen this long
# robots
STUCK_S = 3.0           # driving with no camera travel (< 1 cm) for this long = stuck. Chosen; DOE no
SILENT_S = 6.0          # nothing heard for this long: "silent", kept as an obstacle where the camera sees it
OUT_SILENT_S = 120.0    # silent this long: out for the rest of the run
RESEND_S = 0.3          # a step or control not acknowledged is sent again after this (run_plan --resend-s 0.3)
HEARTBEAT_S = 0.5       # something goes to every robot at least this often (its watchdog: follow_wd_ms 1500)
WD_MS = 1500            # the robots' watchdog. Chosen: 15 camera fixes; DOE no (a safety timeout)
STATUS_S = 20.0         # battery asked this often
BATTERY_FLOOR = 10      # Nick: each robot drives down to 10 %
PARK_EDGE_CC = EDGE_CC + 3.0   # a parked robot's spot: near an edge, out of the way
FRESH_FW = "2026-11-05b"       # fspd / fsteer / follow_wd_ms
# (11-05c, the first real run, 5 Nov 14:35) camera timing, discovered keep-outs, backing off, the edge trap
CAM_DELAY0 = 0.15       # a fix's delay (picture taken -> here) assumed until measured. The real home Pi: 0.42-0.46 s
                        # (the camera bridge's own processing, bridge/events.jsonl, 5 Nov). Measured online after ~10 fixes
CLOCK_DEADBAND_S = 0.25 # the camera's and this controller's clocks are taken as one clock unless the fixes line up with
                        # the robots' own poses this far or more apart (the own poses' WiFi lag is 0.1-0.2 s: 5 Nov logs).
                        # Chosen; no DOE (a threshold above a measured lag)
CLOCK_EVERY_S = 5.0     # the clock alignment is redone this often (only from robots that move)
VERIFY_CM = 5.0         # a camera spot far from the robot's own pose (> REACQUIRE_MAX_CM) is believed only after the
                        # camera has followed the robot's own motion for this far with an unchanged offset (5 Nov: the
                        # camera put Center 45 cm off, on Right's spot, and the controller set Center's pose there).
                        # Chosen; DOE: no (any travel the camera must agree with; 5 cm is 5 camera sigmas)
VERIFY_STILL_S = 2.0    # ... or, standing still, the camera's spot steady this long (and nobody else there). Chosen
WALL_MIN_IN = 3.0       # ... and at least this far (closer is the calibration's own error and the felt point's). Chosen
WALL_MAX_IN = 20.0      # a side moves in at most this far from the calibration's (more: an object). Chosen
KEEP_R = 2.0            # a felt obstacle point (front stop, edge guard) is a keep-out disc this big: the front sensor's
                        # zones are ~1 cm apart at 3 cm. Chosen; DOE no
KEEP_GAP = 1.0          # planned bodies' circles keep this much more from a keep-out. Chosen; DOE with EDGE_CC
BACK_CM = (8.0, 6.0, 4.0)   # backing off after the robot's own front stop: the longest of these that fits. Chosen:
                            # 8 cm takes a 2.9 cm obstacle past the turn check (safe_front_cm 7) with room. DOE no
BACK_V = 5.0            # cm/s: backwards is blind (no rear sensor): slowest menu speed
EDGE_TURN_MIN = HALF + 1.0  # the edge trap's last resort: a turn on the spot with the middle this far in (corners swing
                            # 6.72 cm: they stay 1 cm inside the edge). Only after reversing found nothing


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def edge_dist(rect, x, y):
    return min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y)


def turn_s(deg):
    return 0.0 if abs(deg) < 3.0 else TURN_S0 + abs(deg) / TURN_DPS


def drive_s(L, v):
    return L / v + v / ACCEL + 0.3


# ------------------------------------------------------------------------------------------------ reservations
INF = 1e12


class Traj:
    """A robot's reservation: samples (t_lo, t_hi, x, y, margin, slack) and the open-ended stand at its end."""

    def __init__(self, rows):
        a = np.array(rows, dtype=float).reshape(-1, 6)
        self.lo, self.hi, self.x, self.y, self.m, self.sl = a.T

    @staticmethod
    def stand(t, x, y, m=PLAN_M0, slack=SLACK0_S):
        return Traj([(t, INF, x, y, m, slack)])

    @staticmethod
    def move(t0, p0, h0, turn, L, v, m0=PLAN_M0, bubble_grow=0.0, slack0=SLACK0_S):
        """Turn on the spot by `turn` deg at p0, then drive L cm at v cm/s. bubble_grow: extra margin per cm (lost)."""
        rows = []
        h1 = h0 + turn
        tt = turn_s(turn)
        if tt > 0:
            rows.append((t0, t0 + tt, p0[0], p0[1], m0, slack0 + SLACK_FRAC * tt))
        td = drive_s(L, v) if L > 0 else 0.0
        ux, uy = math.cos(math.radians(h1)), math.sin(math.radians(h1))
        n = max(1, int(math.ceil(td / SAMPLE_S))) if L > 0 else 0
        for k in range(n):
            a, b = t0 + tt + k * td / n, t0 + tt + (k + 1) * td / n
            s = min(L, L * (k + 0.5) / n)
            m = m0 + min(PLAN_M_MAX, PLAN_M_PER_CM * s) + bubble_grow * s
            rows.append((a, b, p0[0] + ux * s, p0[1] + uy * s, m, slack0 + SLACK_FRAC * (b - t0)))
        t1 = t0 + tt + td
        s = L
        m = m0 + min(PLAN_M_MAX, PLAN_M_PER_CM * s) + bubble_grow * s
        rows.append((t1, INF, p0[0] + ux * L, p0[1] + uy * L, m, slack0 + SLACK_FRAC * (t1 - t0)))
        tr = Traj(rows)
        tr.t_end, tr.end, tr.h_end = t1, (p0[0] + ux * L, p0[1] + uy * L), h1
        return tr


def concat(*ts):
    rows = np.vstack([np.stack([t.lo, t.hi, t.x, t.y, t.m, t.sl], 1) for t in ts])
    out = Traj(rows)
    out.end = getattr(ts[-1], "end", (rows[-1, 2], rows[-1, 3]))
    return out


def clearance_profile(A, B, now=None):
    """Per sample of A: the smallest (distance - margins) to B over overlapping time windows (+inf: none)."""
    lo_a, hi_a = A.lo[:, None] - A.sl[:, None] - B.sl[None, :], A.hi[:, None]
    over = (lo_a <= B.hi[None, :]) & (B.lo[None, :] - A.sl[:, None] - B.sl[None, :] <= hi_a)
    if now is not None:
        over &= (A.hi[:, None] >= now) & (B.hi[None, :] >= now)
    d = np.hypot(A.x[:, None] - B.x[None, :], A.y[:, None] - B.y[None, :]) - A.m[:, None] - B.m[None, :]
    d = np.where(over, d, np.inf)
    return d.min(axis=1)


def clearance(A, B, now=None):
    """The smallest (distance - margins) between two reservations over overlapping time windows (+ slack). +inf if
    they never overlap in time."""
    lo_a, hi_a = A.lo[:, None] - A.sl[:, None] - B.sl[None, :], A.hi[:, None]
    over = (lo_a <= B.hi[None, :]) & (B.lo[None, :] - A.sl[:, None] - B.sl[None, :] <= hi_a)
    if now is not None:
        over &= (A.hi[:, None] >= now) & (B.hi[None, :] >= now)
    if not over.any():
        return float("inf")
    d = np.hypot(A.x[:, None] - B.x[None, :], A.y[:, None] - B.y[None, :]) - A.m[:, None] - B.m[None, :]
    return float(d[over].min())


# ------------------------------------------------------------------------------------------------ the menu
class Menu:
    def __init__(self, rng):
        self.rng = rng
        self.cells = [(a, b, c) for a in range(len(TURN_BINS)) for b in range(len(LEN_BINS))
                      for c in range(len(SPEED_BINS))]
        self.n = {c: 0 for c in self.cells}
        self.n_robot = {}

    def order(self, name, free=False):
        cs = list(self.cells)
        self.rng.shuffle(cs)
        if free:
            return cs
        nr = self.n_robot.setdefault(name, {})
        return sorted(cs, key=lambda c: (self.n[c], nr.get(c, 0)))   # stable: the shuffle breaks ties

    def draw(self, cell):
        a, b, c = cell
        jit = 10.0 if TURN_BINS[a] == 0 else 15.0
        turn = TURN_BINS[a] + self.rng.uniform(-jit, jit)
        L = self.rng.uniform(*LEN_BINS[b])
        return turn, L, SPEED_BINS[c]

    def use(self, cell, name):
        self.n[cell] += 1
        nr = self.n_robot.setdefault(name, {})
        nr[cell] = nr.get(cell, 0) + 1

    @staticmethod
    def label(cell):
        a, b, c = cell
        return "t%+d_L%d-%d_v%g" % (TURN_BINS[a], LEN_BINS[b][0], LEN_BINS[b][1], SPEED_BINS[c])


# ------------------------------------------------------------------------------------------------ one robot
class Robot:
    def __init__(self, name, rank):
        self.name, self.rank = name, rank
        self.cam = None                 # (t, x, y, th or None)
        self.cam_hist = []              # [(t, x, y)] last ~4 s
        self.own = None                 # (t, x, y, th, state, cur)
        self.own_hist = []
        self.mode = "start"             # start, idle, move, parking, parked, out
        self.out_why = None
        self.move = None                # dict: kind, cell, steps, traj, t0, v, end, h, phase
        self.traj = None
        self.next_id = 1
        self.steps = {}                 # sid -> dict(text, status, t_sent)
        self.controls = []              # dict(text, t_last, kind)
        self.scale = 1.0
        self.adj = None                 # reflex adjustment in force (event dict)
        self.battery = None
        self.fw = None
        self.heard = None
        self.silent = False
        self.silent_t = None
        self.bat_logged = (None, -1e9)
        self.lost = None                # dict while lost: t0, drive, turn, bubble, last_own
        self.bubble = 0.0
        self.hold_n = 0                 # holding turns in a row
        self.hold_sign = 1
        self.busy_until = 0.0
        self.last_sent = 0.0
        self.last_status = -1e9
        self.wiggle_due = 0.0
        self.reacq = []
        self.n_moves = 0
        self.stopped_hard = None        # t of a hard stop (trophy break) in force
        self.lost_hold = False
        self.cam_arr = None             # when the newest fix arrived (this controller's clock)
        self.cam_raw = []               # [(t as stamped, x, y)] for the clock alignment
        self.cam_off = None             # camera minus own pose at the picture's time, the last accepted fix
        self.back_off = None            # dict after its own front stop: the next move backs away
        self.pc = None                  # (t, pos) cache
        self.disagree_t = -1e9
        self.jump_n = 0

    @property
    def new_fw(self):
        return bool(self.fw) and self.fw >= FRESH_FW


# ------------------------------------------------------------------------------------------------ the controller
class Controller:
    def __init__(self, names, rect, out, log, seed=1, lost_mode="reckon", battery_floor=BATTERY_FLOOR,
                 park_sleep=True, fw_override=None, drill=0.0):
        self.rect = tuple(float(v) for v in rect)
        self.out_fn, self.log = out, log
        self.rng = random.Random(seed)
        self.seed = seed
        self.menu = Menu(random.Random(seed + 1))
        self.R = {n: Robot(n, k) for k, n in enumerate(names)}
        self.lost_mode = lost_mode
        self.floor = battery_floor
        self.park_sleep = park_sleep
        self.fw_override = fw_override
        self.drill = float(drill)       # SIMULATOR ONLY (freeroam.py refuses it on robots): this share of moves is
                                        # planned ignoring the others' reservations, to make the reflex work
        self.t = 0.0
        self.t_start = None
        self.ev_id = 0
        self.last_motion = -1e9
        self.counts = dict(fallback={}, reflex={}, breaks=0, contacts=0, front_stops=0, keepouts=0, backoffs=0,
                           cam_refused=0)
        self.ended = False
        self.keepouts = []              # discovered: dict(x, y, r, t, robot, src); the planner keeps out for the run
        self.delays = []                # arrival - picture time (this clock), recent fixes
        self.cam_delay, self.cam_delay_hi = CAM_DELAY0, CAM_DELAY0
        self.clock = 0.0                # camera clock minus this clock, applied (0 inside the deadband)
        self.clock_seen = None          # the alignment's own estimate (reported even inside the deadband)
        self.clock_ests = []
        self.clock_min = None           # the camera clock is at least this far ahead (a fix cannot arrive before it
                                        # was taken)
        self.last_clock = self.last_delay = self.last_camlog = -1e9

    # -- I/O ---------------------------------------------------------------------------------------------------
    def out(self, r, text):
        r.last_sent = self.t
        self.out_fn(r.name, text)

    def event(self, r, kind, text):
        self.log("events", dict(t=round(self.t, 2), robot=r.name if r else "-", kind=kind, text=text))

    def send_step(self, r, fmt):
        sid = r.next_id
        r.next_id += 1
        r.steps[sid] = dict(text=fmt % sid, status="queued_local", t_sent=-1e9)
        return sid

    def control(self, r, text, kind):
        r.controls = [c for c in r.controls if c["kind"] != kind]
        r.controls.append(dict(text=text, t_last=self.t, kind=kind))
        self.out(r, text)

    def pump(self, r):
        """Steps go one at a time: the next only after the one before was acked (ids must go up); none while a halt is
        not yet acknowledged (a resent halt arriving after the new step would stop it)."""
        for c in r.controls:
            if self.t - c["t_last"] > RESEND_S:
                c["t_last"] = self.t
                self.out(r, c["text"])
        if any(c["kind"] == "halt" for c in r.controls):
            return
        for sid in sorted(r.steps):
            st = r.steps[sid]
            if st["status"] == "queued_local":
                if any(o["status"] == "sent" for o in r.steps.values()):
                    return
                st["status"], st["t_sent"] = "sent", self.t
                self.out(r, st["text"])
                return
            if st["status"] == "sent" and self.t - st["t_sent"] > RESEND_S:
                st["t_sent"] = self.t
                self.out(r, st["text"])
                return

    # -- inputs ------------------------------------------------------------------------------------------------
    def on_heard(self, name, t):
        r = self.R.get(name)
        if r is None:
            return
        r.heard = t
        if r.silent:
            r.silent = False
            self.event(r, "heard again", "heard again after %.1f s" % (t - (r.silent_t or t)))

    def on_fw(self, name, fw):
        r = self.R.get(name)
        if r is not None:
            r.fw = self.fw_override or fw

    def on_battery(self, name, pct, t):
        r = self.R.get(name)
        if r is None or pct is None:
            return
        r.battery = pct
        if pct != r.bat_logged[0] or t - r.bat_logged[1] > 60.0:
            r.bat_logged = (pct, t)
            self.log("battery", dict(t=round(t, 1), robot=name, pct=pct))

    def on_fix(self, name, t, x, y, th=None, arrival=None):
        """A camera fix stamped t (the picture's time on the camera's clock), arrived at `arrival` (this controller's
        clock; None: it counts as arriving now with no delay, as the tests and older callers give it). (11-05c) The
        delay is measured online; the fix is compared with the robot's own pose AT THE PICTURE'S TIME, never now."""
        r = self.R.get(name)
        if r is None:
            return
        if arrival is not None:
            lead = t - arrival
            self.clock_min = lead if self.clock_min is None else max(self.clock_min, lead)
            r.cam_raw.append((t, x, y))
            if len(r.cam_raw) > 200:
                del r.cam_raw[:50]
            t = t - self.clock
            self.delays.append(arrival - t)
            if len(self.delays) > 400:
                del self.delays[:100]
        arr = t if arrival is None else arrival
        r.pc = None
        oa = self.own_at(r, t)
        if r.lost is None and oa is not None and r.cam_off is not None:
            off = (x - oa[0], y - oa[1])
            r.jump_n = r.jump_n + 1 if math.dist(off, r.cam_off) > REACQUIRE_MAX_CM else 0
            if r.jump_n >= 3:
                # the camera's spot jumped away from where its own wheels say it went (three fixes in a row): not
                # believed until it agrees
                r.jump_n = 0
                self.start_lost(r, "the camera's spot jumped %.0f cm from its own pose's track" %
                                math.dist(off, r.cam_off))
            elif r.jump_n:
                return                       # one odd fix: skipped
        if r.lost is not None:
            self.reacquire(r, t, x, y, th, arr)
            if r.lost is not None:
                return
        r.cam = (t, x, y, th)
        r.cam_arr = arr
        if oa is not None:
            r.cam_off = (x - oa[0], y - oa[1])
        r.cam_hist.append((t, x, y))
        if len(r.cam_hist) > 80:
            del r.cam_hist[:20]

    def on_follow(self, name, t, state, x, y, th, cur=0, last_done=None, last_failed=None):
        r = self.R.get(name)
        if r is None:
            return
        # a step's own end report can be lost (a WiFi drop: the watchdog stopped it and said so into the gap); the
        # FOLLOW record (10 a second) carries the last done and last failed step ids, so the end is noticed anyway
        if r.move is not None and (last_done is not None or last_failed is not None):
            ids = [i for i in (r.move.get("turn_id"), r.move.get("drive_id")) if i]
            st = {i: r.steps.get(i, {}).get("status") for i in ids}
            ev = None
            if last_failed and last_failed in st and st[last_failed] not in ("done", "failed", "dropped"):
                ev = (last_failed, "failed")
            elif last_done and ids and max(ids) <= last_done and st[max(ids)] not in ("done", "failed", "dropped") \
                    and state == "idle":
                ev = (max(ids), "done")
            if ev is None:
                r.lost_rep = None
            elif getattr(r, "lost_rep", None) is None or r.lost_rep[0] != ev:
                r.lost_rep = (ev, t)                  # the step's own report may still be on its way: 0.5 s
            elif t - r.lost_rep[1] >= 0.5:
                r.lost_rep = None
                self.event(r, "lost report", "step %d %s (its own report was lost: seen in FOLLOW)" % ev)
                self.on_step(name, ev[0], ev[1], "report lost; seen in FOLLOW" if ev[1] == "failed" else "")
        if t - getattr(r, "fpose_t", -1e9) < 0.15:
            return          # (11-05c) a record sent before our fpose reached it: the old frame
        prev = r.own
        r.own = (t, x, y, th, state, cur)
        r.pc = None
        r.own_hist.append((t, x, y, th))
        if len(r.own_hist) > 200:          # ~20 s at 10 a second: covers a camera delay of several seconds
            del r.own_hist[:50]
        if r.lost is not None and prev is not None:
            d = math.hypot(x - prev[1], y - prev[2])
            r.lost["drive"] += d
            r.lost["turn"] += abs(wrap180(th - prev[3]))
            r.lost["after_turn"] += d
            self.grow_bubble(r)

    def on_step(self, name, sid, state, info=""):
        r = self.R.get(name)
        if r is None:
            return
        if state in ("held", "halted"):
            r.controls = [c for c in r.controls if c["kind"] not in ("halt", "hold")]
            return
        st = r.steps.get(sid)
        if st is None or st["status"] == "dropped":
            return                  # a step this controller already dropped (halted, replaced): its report is old news
        if state in ("queued", "dup") and st["status"] == "sent":
            st["status"] = "acked"
        elif state == "full":
            st["status"] = "queued_local"
        elif state == "bad":
            st["status"] = "failed"
            self.step_over(r, sid, "failed", "bad: " + info)
        elif state == "started":
            st["status"] = "started"
            if r.move and sid == r.move.get("drive_id"):
                r.move["phase"] = "drive"
                r.move["t_drive"] = self.t
        elif state in ("done", "failed", "dropped"):
            st["status"] = state
            self.step_over(r, sid, state, info)

    def on_fspd(self, name):
        r = self.R.get(name)
        if r is not None:
            r.controls = [c for c in r.controls if c["kind"] not in ("fspd", "fsteer")]

    # -- where is it -------------------------------------------------------------------------------------------
    def fresh(self, r):
        """(11-05c) Lost = no fix arrived for LOST_AFTER_S, or the newest is older than LOST_AFTER_S beyond the
        measured delay (5 Nov: fixes arrived 0.45 s old, so every 0.15 s gap read as lost)."""
        if r.cam is None:
            return False
        arr = r.cam_arr if r.cam_arr is not None else r.cam[0]
        return self.t - arr < LOST_AFTER_S and self.t - r.cam[0] < LOST_AFTER_S + self.cam_delay_hi

    def own_at(self, r, t):
        """The robot's own pose (x, y, th) at time t, from its FOLLOW records (interpolated); None if t is older than
        the history kept."""
        h = r.own_hist
        if not h or t < h[0][0] - 0.05:
            return None
        if t >= h[-1][0]:
            return h[-1][1:4]
        k = len(h) - 1
        while k > 0 and h[k - 1][0] > t:
            k -= 1
        if k == 0:
            return h[0][1:4]
        a, b = h[k - 1], h[k]
        f = (t - a[0]) / max(1e-6, b[0] - a[0])
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), wrap180(a[3] + f * wrap180(b[3] - a[3])))

    def projected(self, r, t, x, y, th=None):
        """A camera spot taken at t moved on to now by the robot's own motion since then (x, y, th or None);
        None when its own pose does not reach back that far or is not current."""
        if r.own is None or self.t - r.own[0] > 1.0:
            return None
        oa = self.own_at(r, t)
        if oa is None:
            return None
        return (x + r.own[1] - oa[0], y + r.own[2] - oa[1],
                None if th is None else wrap180(th + wrap180(r.own[3] - oa[2])))

    def pos(self, r):
        if r.pc is not None and r.pc[0] == self.t:
            return r.pc[1]
        p = None
        if r.lost is None and r.cam is not None:
            q = self.projected(r, r.cam[0], r.cam[1], r.cam[2])
            p = (q[0], q[1]) if q else (r.cam[1], r.cam[2])
            r.pc_proj = q is not None
        elif r.own is not None:
            p = (r.own[1], r.own[2])
        elif r.cam:
            p = (r.cam[1], r.cam[2])
        r.pc = (self.t, p)
        return p

    # -- camera timing (11-05c) ----------------------------------------------------------------------------------
    def camera_timing(self):
        if self.t - self.last_delay >= 1.0 and len(self.delays) >= 10:
            self.last_delay = self.t
            d = np.array(self.delays[-150:])
            self.cam_delay = float(np.median(d))
            self.cam_delay_hi = float(min(3.0, max(0.0, np.percentile(d, 90)) + 0.1))
        if self.t - self.last_clock >= CLOCK_EVERY_S:
            self.last_clock = self.t
            self.align_clock()
        if self.t - self.last_camlog >= 30.0 and self.delays:
            self.last_camlog = self.t
            self.log("camera", dict(t=round(self.t, 2), delay_med_s=round(self.cam_delay, 3),
                                    delay_p90_s=round(self.cam_delay_hi - 0.1, 3), lost_after_s=round(
                                        LOST_AFTER_S + self.cam_delay_hi, 2), clock_applied_s=round(self.clock, 3),
                                    clock_seen_s=round(self.clock_seen, 3) if self.clock_seen is not None else "",
                                    n=len(self.delays)))

    def align_clock(self):
        """Which time on this clock the camera's stamps mean: the shift that makes the camera's track move as the
        robots' own poses moved (displacements over ~0.5 s, the last 10 s, robots that drove). Applied only outside
        CLOCK_DEADBAND_S; never so that a fix would arrive before it was taken."""
        found = []
        lags = np.arange(-3.0, 3.001, 0.02)
        for r in self.R.values():
            c = np.array([q for q in r.cam_raw if q[0] > r.cam_raw[-1][0] - 10.0]) if r.cam_raw else None
            o = np.array(r.own_hist) if len(r.own_hist) > 10 else None
            if c is None or o is None or len(c) < 20:
                continue
            # speeds over ~0.5 s (camera vs own): starts and stops line up; the own pose's pulls towards the fixes
            # (the firmware's fusion) shift positions, not when it starts and stops
            i = np.arange(0, len(c) - 6, 2)
            j = i + 5
            dt = np.maximum(0.05, c[j, 0] - c[i, 0])
            sc = np.hypot(*(c[j, 1:3] - c[i, 1:3]).T) / dt
            if sc.max() - sc.min() < 4.0:
                continue                     # no start or stop in the window: nothing to line up
            errs = []
            for L in lags:
                ts = c[:, 0] - L
                ok = (ts >= o[0, 0]) & (ts <= o[-1, 0])
                m = ok[i] & ok[j]
                if m.sum() < 8:
                    errs.append(np.inf)
                    continue
                ox, oy = np.interp(ts, o[:, 0], o[:, 1]), np.interp(ts, o[:, 0], o[:, 2])
                so = np.hypot(ox[j] - ox[i], oy[j] - oy[i]) / dt
                errs.append(float(np.mean(np.abs(sc - so)[m])))
            e = np.array(errs)
            fin = e[np.isfinite(e)]
            # only a clear best (a robot that changed speed or direction): a steady drive fits every shift alike
            if len(fin) > 20 and fin.min() < 0.5 * np.median(fin):
                found.append(float(lags[int(np.argmin(e))]))
        if not found:
            return
        self.clock_ests.append(float(np.median(found)))
        del self.clock_ests[:-9]
        est = float(np.median(self.clock_ests))
        self.clock_seen = est
        # applied only when the last five agree (spread under 0.2 s) and lie outside the deadband
        last = self.clock_ests[-5:]
        steady = len(last) == 5 and max(last) - min(last) < 0.2
        new = est if (steady and abs(est) >= CLOCK_DEADBAND_S) else (self.clock if not steady else 0.0)
        if self.clock_min is not None:
            new = max(new, self.clock_min - 0.02) if self.clock_min > 0.02 else new
        if abs(new - self.clock) > 0.05:
            self.delays = [d + new - self.clock for d in self.delays]     # the same fixes, read on the new clock
            self.cam_delay_hi += new - self.clock
            for r in self.R.values():
                if r.cam is not None:
                    r.cam = (r.cam[0] - (new - self.clock),) + tuple(r.cam[1:])
            self.event(None, "camera clock", "the camera's time stamps are %+.2f s from this clock (from the robots' "
                       "own tracks); fixes are read %s" % (est, "with that shift" if new else "as they are (within "
                                                            "%.2f s)" % CLOCK_DEADBAND_S))
            self.clock = new

    def heading(self, r):
        if r.own is not None:
            return r.own[3]
        return r.cam[3] if r.cam and r.cam[3] is not None else 0.0

    def margin0(self, r):
        return PLAN_M0 + (r.bubble if r.lost is not None else 0.0) + (3.0 if r.silent else 0.0)

    # -- dead reckoning --------------------------------------------------------------------------------------------
    def grow_bubble(self, r):
        L = r.lost
        lat = L["after_turn"] * math.sin(math.radians(BLIND_TURN_FRAC * L["turn"]))
        r.bubble = BLIND_BASE + BLIND_PER_CM * L["drive"] + lat

    def start_lost(self, r, why="camera lost it"):
        p = self.pos(r)
        r.lost = dict(t0=self.t, drive=0.0, turn=0.0, after_turn=0.0, at=p, n_ok=0, last=None, chain=[])
        r.pc = None
        r.bubble = BLIND_BASE
        self.event(r, "lost", "%s at %.1f, %.1f: dead reckoning, bubble %.1f cm" % (why, p[0], p[1], r.bubble)
                   if p else why)
        if self.lost_mode == "hold" and r.mode == "move":
            r.lost_hold = True
            self.halt(r, "lost (hold mode)")
            self.trophy_break(r, "lost by the camera (cautious hold switch)")

    def reacquire(self, r, t, x, y, th, arr=None):
        """(11-05c) The fix is compared with the robot's own pose at the picture's time t. A spot further off than
        REACQUIRE_MAX_CM is believed only once the camera's spot has moved as the robot's own pose moved (offset
        unchanged within 3 cm) over VERIFY_CM of its own travel; a camera on something else (a stationary object, the
        other robot) never passes. The pose sent (fpose) is the camera's spot moved on to now by its own motion."""
        L = r.lost
        arr = t if arr is None else arr
        oa = self.own_at(r, t)
        if oa is None and r.own is not None:
            oa = r.own[1:4]
        jump = math.hypot(x - oa[0], y - oa[1]) if oa else 0.0
        if jump > REACQUIRE_MAX_CM and oa is not None:
            off = (x - oa[0], y - oa[1])
            ch = L.setdefault("chain", [])
            if ch and math.dist(off, ch[0][1]) > 3.0:
                if self.t - r.disagree_t > 5.0:
                    r.disagree_t = self.t
                    self.counts["cam_refused"] += 1
                    self.event(r, "camera disagrees", "the camera's spot %.1f, %.1f (%.0f cm from its own pose) did "
                               "not move as its wheels did: not believed, dead reckoning on" % (x, y, jump))
                ch.clear()
            ch.append(((oa[0], oa[1]), off, t))
            travel = sum(math.dist(ch[k][0], ch[k + 1][0]) for k in range(len(ch) - 1))
            # a robot that stands still cannot show it: a steady spot for VERIFY_STILL_S is believed unless another
            # robot is (or is believed to be) there (5 Nov: the camera put Center on Right's spot)
            span = ch[-1][2] - ch[0][2]
            taken = [o.name for o in self.R.values() if o is not r and self.pos(o) is not None and
                     math.dist(self.pos(o), (x, y)) < 2 * HALF + 2.0]
            if len(ch) < 3 or (travel < VERIFY_CM and (span < VERIFY_STILL_S or len(ch) < 10 or taken)):
                if taken and span >= VERIFY_STILL_S and self.t - r.disagree_t > 5.0:
                    r.disagree_t = self.t
                    self.counts["cam_refused"] += 1
                    self.event(r, "camera disagrees", "the camera's spot %.1f, %.1f is where %s is: not believed"
                               % (x, y, taken[0]))
                return
        within = jump <= r.bubble
        self.log("lost", dict(robot=r.name, t0=round(L["t0"], 2), t1=round(arr, 2), dur_s=round(arr - L["t0"], 2),
                              blind_cm=round(L["drive"], 1), blind_deg=round(L["turn"], 1),
                              bubble_cm=round(r.bubble, 2), jump_cm=round(jump, 2), within=int(within),
                              x=round(x, 1), y=round(y, 1), fix_age_s=round(arr - t, 2)))
        self.event(r, "seen again", "after %.1f s, %.1f cm and %.0f deg blind: odometry was %.1f cm off at the "
                   "picture's time (bubble %.1f; the fix was %.2f s old)"
                   % (arr - L["t0"], L["drive"], L["turn"], jump, r.bubble, arr - t))
        if jump > REACQUIRE_CM:
            q = self.projected(r, t, x, y, th)
            if q is None:                       # its own pose does not reach back: the fix as it is (old behaviour)
                q = (x, y, th)
            h = q[2] if q[2] is not None else (r.own[3] if r.own else 0.0)
            self.out(r, "fpose %.2f %.2f %.2f 1.0 3.0" % (q[0], q[1], h))
            if r.own is not None:               # its own track from now on is in the new frame: shift the history
                dx, dy = q[0] - r.own[1], q[1] - r.own[2]
                r.own_hist = [(a, b + dx, c + dy, d) for a, b, c, d in r.own_hist]
                r.own = (r.own[0], q[0], q[1], h) + tuple(r.own[4:])
            r.fpose_t = self.t
        r.lost, r.bubble = None, 0.0
        r.pc = None
        if r.lost_hold:
            r.lost_hold = False
        if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
            self.close_break(r)
        r.cam = (t, x, y, th)
        r.cam_arr = arr
        oa2 = self.own_at(r, t)
        r.cam_off = (x - oa2[0], y - oa2[1]) if oa2 else None

    # -- moves ---------------------------------------------------------------------------------------------------
    def others(self, r):
        return [o for o in self.R.values() if o is not r and o.traj is not None and o.mode != "gone"]

    def feasible(self, r, tr, edge_extra=0.0, gap_relax=0.0, ignore=(), away=False):
        """away: a robot that already stands too close may still take a move that never brings it closer (each
        sample at least its start's clearance - 0.5 cm) and ends clear (the escape and yield rungs)."""
        e = tr.end
        if edge_dist(self.rect, e[0], e[1]) < EDGE_CC + edge_extra + (r.bubble if r.lost else 0.0):
            return False
        if not self.keepout_ok(r, tr):
            return False
        for o in self.others(r):
            if o.name in ignore:
                need = REFLEX_CC
                ob = Traj.stand(self.t, *self.pos(o), m=self.margin0(o))
            else:
                need = PLAN_CC - gap_relax
                ob = o.traj
            if not away:
                if clearance(tr, ob, now=self.t) < need:
                    return False
                continue
            prof = clearance_profile(tr, ob, now=self.t)
            if prof[-1] < need:
                return False
            if (prof < np.minimum(need, prof[0] - 0.5)).any():
                return False
        return True

    def keepout_ok(self, r, tr):
        """(11-05c) Every sample of the move keeps its circle KEEP_GAP from every discovered keep-out; a robot that
        already stands too close may take a move that never brings it closer and ends clear (backing away)."""
        if not self.keepouts:
            return True
        k = np.array([(q["x"], q["y"], q["r"]) for q in self.keepouts])
        d = np.hypot(tr.x[:, None] - k[None, :, 0], tr.y[:, None] - k[None, :, 1]) - k[None, :, 2] - tr.m[:, None] \
            - HALF - KEEP_GAP + PLAN_M0
        dm = d.min(axis=1)
        if (dm >= 0).all():
            return True
        return bool(dm[-1] >= 0 and (dm >= np.minimum(0.0, dm[0] - 0.5)).all())

    def felt(self, r, info):
        """(11-05c) Its own front sensor (or edge guard) stopped a step: where the thing it felt is. Next to another
        robot's known spot it is that robot (no keep-out); otherwise a keep-out for the rest of the run (discovered:
        a wall, an object, a robot the camera does not see), cleared again only if a robot's camera track later
        drives through it. The next move backs away."""
        p, h = self.pos(r), self.heading(r)
        if p is None:
            return
        import re as _re
        m = _re.search(r"obstacle (-?[0-9.]+) cm", info)
        d = max(0.0, float(m.group(1))) if m else 1.0
        u = (math.cos(math.radians(h)), math.sin(math.radians(h)))
        f = (p[0] + u[0] * (BODY / 2 + d), p[1] + u[1] * (BODY / 2 + d))
        r.back_off = dict(t=self.t, h=h)
        self.counts["front_stops"] += 1
        who = [o.name for o in self.R.values() if o is not r and self.pos(o) is not None and
               math.dist(self.pos(o), f) < HALF + 3.0]
        src = "edge guard" if "edge" in info and not m else "front sensor"
        if who:
            self.event(r, "felt a robot", "%s stopped it %.1f cm ahead at %.1f, %.1f: that is %s (no keep-out)" % (
                src, d, f[0], f[1], who[0]))
            return
        self.wall_check(r, f, src, h)
        for q in self.keepouts:
            if math.dist((q["x"], q["y"]), f) < 1.0:
                return
        q = dict(x=round(f[0], 1), y=round(f[1], 1), r=KEEP_R, t=round(self.t, 2), robot=r.name, src=src,
                 edge_cm=round(edge_dist(self.rect, *f), 1), d_cm=d)
        self.keepouts.append(q)
        self.counts["keepouts"] += 1
        self.log("keepouts", dict(q, what="found"))
        self.event(r, "keep-out", "%s felt something %.1f cm ahead at %.1f, %.1f (%.1f cm inside the board's edge): "
                   "kept out of for the rest of the run" % (src, d, f[0], f[1], q["edge_cm"]))

    def wall_check(self, r, f, src, h=None):
        """(11-05c) Discover, don't pre-map: a thing felt inside the edge band (closer to a side than a planned body's
        face can be, EDGE_CC - HALF) is that side's real wall or edge: the side moves in to it for the rest of the run
        (never out; at most WALL_MAX_IN, beyond that it is an object, a keep-out only)."""
        x0, y0, x1, y1 = self.rect
        sides = [(f[0] - x0, 0), (f[1] - y0, 1), (x1 - f[0], 2), (y1 - f[1], 3)]
        d, k = min(sides)
        if d < WALL_MIN_IN or d > EDGE_CC - HALF or d > WALL_MAX_IN:
            return
        if h is not None:                    # only when it faced that side (a glancing touch says little)
            normal = (180.0, -90.0, 0.0, 90.0)[k]
            if abs(wrap180(h - normal)) > 35.0:
                return
        # two felt points on the same side at about the same depth (+-3 cm), 15 cm or more apart along it
        pend = self.__dict__.setdefault("wall_pend", {}).setdefault(k, [])
        twin = [q for q in pend if abs(q[0] - d) <= 3.0 and abs(q[1] - f[1 - k % 2]) >= 15.0]
        pend.append((d, f[1 - k % 2]))
        if not twin:
            self.event(r, "wall?", "its %s felt something %.1f cm inside the %s side: one more there and that side "
                       "moves in" % (src, d, ("x_min", "y_min", "x_max", "y_max")[k]))
            return
        d = min(d, min(q[0] for q in twin))
        rect = list(self.rect)
        rect[k] = rect[k] + d if k < 2 else rect[k] - d
        self.rect = tuple(rect)
        self.log("keepouts", dict(x=round(f[0], 1), y=round(f[1], 1), r=0.0, t=round(self.t, 2), robot=r.name, src=src,
                                  edge_cm=round(d, 1), d_cm="", what="wall: the %s side moved in %.1f cm" % (
                                      ("x_min", "y_min", "x_max", "y_max")[k], d)))
        self.event(r, "wall found", "its %s felt the board's %s side %.1f cm inside where the calibration has it: "
                   "board now x %.1f-%.1f, y %.1f-%.1f cm" % (src, ("x_min", "y_min", "x_max", "y_max")[k], d,
                                                             *[self.rect[i] for i in (0, 2, 1, 3)]))

    def clear_keepouts(self):
        """A keep-out a robot's verified camera track drives through was not a fixed thing (a robot the camera did not
        see then, a hand): dropped."""
        for r in self.R.values():
            if r.lost is not None or not self.fresh(r) or r.mode in ("out", "gone"):
                continue
            p = self.pos(r)
            for q in list(self.keepouts):
                if math.dist(p, (q["x"], q["y"])) < HALF - 1.0 and self.t - q["t"] > 2.0:
                    self.keepouts.remove(q)
                    self.log("keepouts", dict(q, what="cleared by %s's camera track" % r.name))
                    self.event(r, "keep-out cleared", "its camera track went through the keep-out at %.1f, %.1f"
                               % (q["x"], q["y"]))

    def back_move(self, r, Ls=BACK_CM, why="its own front stop", inward=False):
        """Straight backwards (firmware rev drive: no front check; no rear sensor, so short and slow) along its
        heading: the longest of Ls that keeps the others' reservations and the keep-outs. inward: only if it ends
        further from the edge than it starts."""
        p, h = self.pos(r), self.heading(r)
        e0 = edge_dist(self.rect, *p)
        for L in Ls:
            tr = Traj.move(self.t + 0.2, p, wrap180(h + 180.0), 0.0, L, BACK_V, m0=self.margin0(r))
            e1 = edge_dist(self.rect, *tr.end)
            # backwards is blind: away from the edges by the turn margin, or (inward) at least further in than now
            if e1 < (HALF + 1.0 if inward else TURN_EDGE_CC) + (r.bubble if r.lost else 0.0) or \
                    (inward and e1 < e0 + 1.0):
                continue
            if not self.keepout_ok(r, tr):
                continue
            if all(clearance_profile(tr, o.traj, now=self.t)[-1] >= REFLEX_CC for o in self.others(r)):
                return ("back off" if not inward else "reverse"), None, 0.0, L, BACK_V, tr
        return None

    def make(self, r, turn, L, v, t0=None):
        p, h = self.pos(r), self.heading(r)
        grow = BLIND_PER_CM if r.lost is not None else 0.0
        return Traj.move(self.t + 0.2 if t0 is None else t0, p, h, turn, L, v, m0=self.margin0(r), bubble_grow=grow)

    def can_turn_here(self, r):
        p = self.pos(r)
        return edge_dist(self.rect, *p) >= TURN_EDGE_CC + (r.bubble if r.lost else 0.0)

    def pick(self, r):
        """Menu first; then the fallback ladder. Returns (kind, cell, turn, L, v, traj)."""
        p, h = self.pos(r), self.heading(r)
        turn_ok = self.can_turn_here(r)
        if self.drill > 0 and turn_ok and self.rng.random() < self.drill:
            for cell in self.menu.order(r.name, free=True):
                turn, L, v = self.menu.draw(cell)
                tr = self.make(r, turn, L, v)
                e = tr.end
                if edge_dist(self.rect, *e) >= EDGE_CC and all(
                        math.dist(e, self.pos(o)) >= PLAN_CC + 5 for o in self.R.values()
                        if o is not r and self.pos(o) is not None):
                    return "drill", cell, turn, L, v, tr
        free = self.rng.random() < FREE_SHARE
        if turn_ok:
            for cell in self.menu.order(r.name, free=free):
                turn, L, v = self.menu.draw(cell)
                tr = self.make(r, turn, L, v)
                if self.feasible(r, tr):
                    return ("free" if free else "menu"), cell, turn, L, v, tr
        why = "no menu move fits" if turn_ok else "too near an edge to turn on the spot"
        # rung 1: off-menu, any heading, the longest that fits (shorter than the menu's least too)
        best = None
        for L in (70, 50, 35, 22, 14, 8, 5):
            hs = list(range(-180, 180, 15)) if turn_ok else [0]
            self.rng.shuffle(hs)
            for d in hs:
                tr = self.make(r, d, L, 7.5)
                if self.feasible(r, tr):
                    best = ("off-menu", None, d, L, 7.5, tr)
                    break
            if best:
                break
        if best:
            self.fallback(r, 1, "off-menu", why)
            return best
        # rung 2: escape to open space (board centre side), the longest feasible length, gap relaxed to the reflex's
        cx, cy = (self.rect[0] + self.rect[2]) / 2, (self.rect[1] + self.rect[3]) / 2
        h_c = math.degrees(math.atan2(cy - p[1], cx - p[0]))
        for dh in (0, 30, -30, 60, -60, 90, -90, 120, -120, 180):
            d = wrap180(h_c + dh - h) if turn_ok else 0.0
            for L in (40, 25, 15, 8, 4):
                tr = self.make(r, d, L, 5.0)
                if self.feasible(r, tr, gap_relax=PLAN_CC - REFLEX_CC, away=True):
                    self.fallback(r, 2, "escape", why + "; off-menu too")
                    return "escape", None, d, L, 5.0, tr
            if not turn_ok:
                break
        # deadlock: holding next to another holding robot -> the lower-priority one yields (others as they stand now)
        if r.hold_n >= 3:
            near = [o for o in self.R.values() if o is not r and o.mode in ("idle", "move") and o.hold_n >= 3 and
                    math.dist(self.pos(o), p) < 45.0]
            if near and all(o.rank < r.rank for o in near):
                ign = tuple(o.name for o in near)
                q = self.pos(near[0])
                h_a = math.degrees(math.atan2(p[1] - q[1], p[0] - q[0]))
                for dh in (0, 35, -35, 70, -70, 110, -110):
                    d = wrap180(h_a + dh - h) if turn_ok else 0.0
                    for L in (30, 18, 10, 5):
                        tr = self.make(r, d, L, 5.0)
                        if self.feasible(r, tr, gap_relax=PLAN_CC - REFLEX_CC, ignore=ign, away=True):
                            self.fallback(r, 3, "yield", "deadlock with %s: the lower-priority one moves away" %
                                          ", ".join(ign))
                            return "yield", None, d, L, 5.0, tr
        # rung 3: a holding turn on the spot (never leaves its own circle; only with that circle clear of everyone)
        room = all(math.dist(self.pos(o), p) >= HARD_CC + 1.0 + self.margin0(o) for o in self.R.values()
                   if o is not r and self.pos(o) is not None)
        if turn_ok and room:
            r.hold_sign = -r.hold_sign
            d = 40.0 * r.hold_sign
            tr = Traj.move(self.t + 0.2, p, h, d, 0.0, 5.0, m0=self.margin0(r))
            self.fallback(r, 4, "holding turn", why + "; no off-menu or escape move either")
            return "holding", None, d, 0.0, 5.0, tr
        if not turn_ok:
            # (11-05c) the edge trap (5 Nov: Right stood 3 min 7 cm from the edge): reverse away from the edge, else
            # turn to face the board's middle where the corners still clear the edge
            res = self.back_move(r, Ls=(14.0, 10.0, 6.0, 3.0), inward=True)
            if res is not None:
                self.fallback(r, 5, "reverse", why + ": backs away from the edge")
                return res
            if edge_dist(self.rect, *p) >= EDGE_TURN_MIN + (r.bubble if r.lost else 0.0) and room and \
                    self.keepout_ok(r, Traj.stand(self.t, *p, m=self.margin0(r))):
                d = wrap180(h_c - h)
                tr = Traj.move(self.t + 0.2, p, h, d, 0.0, 5.0, m0=self.margin0(r))
                self.fallback(r, 5, "edge turn", why + ": turns to face the middle (corners stay inside)")
                return "edge turn", None, d, 0.0, 5.0, tr
        self.fallback(r, 6, "stand", "nothing fits and no room to turn on the spot: stands (trophy break)")
        return None

    def fallback(self, r, level, kind, why):
        self.counts["fallback"][kind] = self.counts["fallback"].get(kind, 0) + 1
        self.log("fallbacks", dict(t=round(self.t, 2), robot=r.name, level=level, kind=kind, why=why))

    def start_move(self, r):
        res = None
        if r.back_off is not None:
            r.back_off = None
            res = self.back_move(r)
            if res is not None:
                self.counts["backoffs"] += 1
                self.fallback(r, 0, "back off", "its own front stop: backs away before anything else (never the "
                                                "same heading again)")
        if res is None:
            res = self.pick(r)
        if res is None:
            r.busy_until = self.t + 1.0
            if not r.stopped_hard:
                self.trophy_break(r, "nothing fits from a spot too near the edge to turn")
            return
        if r.stopped_hard and (r.adj is None or r.adj["kind"] != "stop"):
            self.close_break(r)          # it moves again: the full stop (if any) ends here
        kind, cell, turn, L, v, tr = res
        r.hold_n = r.hold_n + 1 if kind == "holding" else 0
        if cell is not None:
            self.menu.use(cell, r.name)
        p, h = self.pos(r), self.heading(r)
        h1 = wrap180(h + turn)
        rev = kind in ("back off", "reverse")
        mv = dict(kind=kind, cell=cell, turn=turn, L=L, v=v, t0=self.t, p0=p, h0=h, h1=h1, end=tr.end,
                  phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None, adj_scale=1.0, rev=rev)
        if abs(turn) >= 3.0:
            mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % h1)
        if L > 0:
            mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f %d -" % (
                p[0], p[1], tr.end[0], tr.end[1], h1, v, 1 if rev else 0))
        else:
            mv["phase"] = "turn"
        r.move, r.traj, r.mode = mv, tr, "move"
        r.n_moves += 1
        r.scale = 1.0
        self.pump(r)

    def step_over(self, r, sid, state, info):
        mv = r.move
        if mv is None:
            return
        last = mv["drive_id"] or mv["turn_id"]
        if state == "done" and sid != last:
            return
        result = "done" if state == "done" else ("%s: %s" % (state, info))
        p = self.pos(r)
        end_err = math.dist(p, mv["end"]) if p and state == "done" else ""
        self.log("moves", dict(t=round(mv["t0"], 2), robot=r.name, n=mv["n"], kind=mv["kind"],
                               cell=Menu.label(mv["cell"]) if mv["cell"] else "",
                               turn_deg=round(mv["turn"], 1), length_cm=round(mv["L"], 1), speed=mv["v"],
                               x0=round(mv["p0"][0], 1), y0=round(mv["p0"][1], 1),
                               x1=round(mv["end"][0], 1), y1=round(mv["end"][1], 1),
                               est_s=round(turn_s(mv["turn"]) + (drive_s(mv["L"], mv["v"]) if mv["L"] else 0), 2),
                               dur_s=round(self.t - mv["t0"], 2), result=result,
                               end_err_cm=round(end_err, 2) if end_err != "" else "",
                               # (CLAUDE.md, 5 Nov: surface features found from the robots' own data) the robot's own
                               # pose against the camera where the move ended, with the camera's position: later tools
                               # can map where odometry disagrees (seams, bumps, slippery patches); no map is made here
                               # (11-05c) own pose at the PICTURE's time, not now (the fix is ~0.45 s old on the Pi)
                               own_vs_cam_cm=round(math.dist(self.own_at(r, r.cam[0])[:2], (r.cam[1], r.cam[2])), 2)
                               if (r.own and r.cam and r.lost is None and self.own_at(r, r.cam[0])) else "",
                               cam_x=round(r.cam[1], 1) if r.cam else "", cam_y=round(r.cam[2], 1) if r.cam else ""))
        if state != "done":
            if "obstacle" in info or "plywood edge" in info:
                self.event(r, "front stop", "its own %s stopped the step: %s" % (
                    "front sensor" if "obstacle" in info else "edge guard", info))
                self.felt(r, info)
            elif "watchdog" in info:
                self.event(r, "watchdog", "its watchdog stopped the step (nothing heard from the Pi)")
            elif state == "failed":
                self.event(r, "step failed", info)
        self.end_move(r)

    def end_move(self, r):
        p = self.pos(r)
        r.move = None
        if r.adj is not None:
            self.close_adj(r, "move ended")
        r.scale = 1.0
        for st in r.steps.values():
            if st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        if p is not None:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r))
        if r.mode == "move":
            r.mode = "idle"
        r.busy_until = self.t + PAUSE_S
        if r.mode == "parking":
            self.parked(r)

    def halt(self, r, why):
        self.control(r, "halt", "halt")
        self.event(r, "halt", why)

    # -- trophy breaks (full stops) ----------------------------------------------------------------------------
    def trophy_break(self, r, why):
        self.counts["breaks"] += 1
        r.stopped_hard = dict(t=self.t, why=why)
        self.event(r, "trophy break", why)

    def close_break(self, r):
        b = r.stopped_hard
        if b:
            self.log("breaks", dict(t=round(b["t"], 2), robot=r.name, why=b["why"], dur_s=round(self.t - b["t"], 2)))
        r.stopped_hard = None

    # -- the reflex --------------------------------------------------------------------------------------------
    def motion(self, r, scale=None, steer=None):
        """(p, u, v, rem): where it is now (camera, moved on by the fix's age), unit direction, speed, cm left."""
        p = self.pos(r)
        mv = r.move
        if p is None or mv is None or mv["phase"] != "drive" or r.lost_hold:
            return p, (0.0, 0.0), 0.0, 0.0
        if r.own is not None and (r.own[4] != "drive" or self.t - r.own[0] > 0.6):
            return p, (0.0, 0.0), 0.0, 0.0
        end = steer if steer is not None else mv["end"]
        dx, dy = end[0] - p[0], end[1] - p[1]
        rem = math.hypot(dx, dy)
        if rem < 0.5:
            return p, (0.0, 0.0), 0.0, 0.0
        u = (dx / rem, dy / rem)
        v = mv["v"] * (r.scale if scale is None else scale)
        if r.cam and r.lost is None and not getattr(r, "pc_proj", False):   # (11-05c) a projected pos is now already
            age = max(0.0, self.t - r.cam[0])
            adv = min(rem, v * age)
            p = (p[0] + u[0] * adv, p[1] + u[1] * adv)
            rem -= adv
        return p, u, v, rem

    @staticmethod
    def path(m, H=REFLEX_H_S, dt=REFLEX_STEP_S):
        p, u, v, rem = m
        ts = np.arange(0.0, H + 1e-9, dt)
        s = np.minimum(v * ts, rem)
        return np.stack([p[0] + u[0] * s, p[1] + u[1] * s], axis=1), ts

    def pair_min(self, ma, mb):
        """Closest predicted distance and when; a pair that is not closing in (its closest is now, or within 0.3 cm
        of now) comes back as +inf: moving apart is the cure, never a reason to stop."""
        pa, ts = self.path(ma)
        pb, _ = self.path(mb)
        d = np.hypot(pa[:, 0] - pb[:, 0], pa[:, 1] - pb[:, 1])
        k = int(np.argmin(d))
        if d[k] >= d[0] - 0.3:
            return float("inf"), float(ts[k])
        return float(d[k]), float(ts[k])

    def need(self, a, b):
        return REFLEX_CC + (a.bubble if a.lost else 0.0) + (b.bubble if b.lost else 0.0)

    def worst_for(self, r, mr, skip=(), hedge=True):
        """The worst predicted (gap - need) of r's motion mr against everyone else's: each other robot both as
        commanded and standing where the camera has it (a robot that stopped answering, a stop not yet reported)."""
        worst, who, tmin = float("inf"), None, None
        for o in self.R.values():
            if o is r or o.name in skip or self.pos(o) is None or o.mode in ("gone",):
                continue
            mo = self.motion(o)
            d, tm = self.pair_min(mr, mo)
            if hedge and mo[2] > 0:
                d2, tm2 = self.pair_min(mr, (self.pos(o), (0.0, 0.0), 0.0, 0.0))
                if d2 < d:
                    d, tm = d2, tm2
            g = d - self.need(r, o)
            if g < worst:
                worst, who, tmin = g, o, tm
        return worst, who, tmin

    def reflex(self):
        movers = [r for r in self.R.values() if r.mode in ("move", "parking") and r.move is not None]
        if not movers:
            return
        seen = set()
        for r in sorted(movers, key=lambda q: -q.rank):          # lowest priority first
            if r.adj is not None:
                continue
            mr = self.motion(r)
            if mr[2] <= 0:
                continue
            g, o, tm = self.worst_for(r, mr, hedge=True)
            if o is None or g >= 0 or (r.name, o.name) in seen:
                continue
            seen.add((o.name, r.name))
            # who acts: the lower-priority mover (r is the lower one unless o moves and ranks lower)
            mo = self.motion(o)
            actors = [r]
            if mo[2] > 0 and o.adj is None:
                actors = sorted([r, o], key=lambda q: -q.rank)      # the lower-priority mover first
            pred_before = g + self.need(r, o)
            done = False
            for a in actors:
                if self.try_adjust(a, o if a is r else r, pred_before):
                    done = True
                    break
            if done:
                continue
            a = actors[0]
            hard = pred_before < HARD_CC + (r.bubble if r.lost else 0) + (o.bubble if o.lost else 0) and tm <= HARD_T_S
            if hard:
                self.apply(a, o if a is r else r, "stop", 0.0, None, pred_before, pred_before)
            else:
                self.replan(a, o if a is r else r, pred_before)

    def try_adjust(self, a, other, pred_before, speeds=True):
        """Speed first (only against a robot that moves: slowing towards one that stands only delays it), then a
        steer; the first that keeps every predicted gap is applied. False: none does."""
        cands = []
        other_moves = self.motion(other)[2] > 0
        for s in (SPEED_OPTIONS if (speeds and other_moves) else ()):
            if a.move["v"] * s > V_MAX or abs(s - a.scale) < 0.01:
                continue
            cands.append(("speed", s, None))
        p, u, v, rem = self.motion(a)
        if rem > 3.0:
            h = math.degrees(math.atan2(u[1], u[0]))
            for dh in STEER_OPTIONS:
                hh = math.radians(h + dh)
                L = min(rem, 30.0)
                end = (p[0] + math.cos(hh) * L, p[1] + math.sin(hh) * L)
                if edge_dist(self.rect, *end) < EDGE_CC + (a.bubble if a.lost else 0.0):
                    continue
                cands.append(("steer", 1.0, end))
        for kind, s, end in cands:
            m = self.motion(a, scale=s, steer=end)
            g, o2, _ = self.worst_for(a, m)
            if g >= 0.5:
                if kind == "steer":
                    tr = Traj.move(self.t, p, h, 0.0, math.dist(p, end), a.move["v"], m0=self.margin0(a))
                    tr.end = end
                    if not self.feasible(a, tr, gap_relax=PLAN_CC - REFLEX_CC):
                        continue
                self.apply(a, other, kind, s, end, pred_before, g + self.need(a, o2) if o2 else 99.0)
                return True
        return False

    def apply(self, a, other, kind, scale, end, pred_before, pred_after):
        self.ev_id += 1
        method = "fspd" if a.new_fw else "reissue"
        if kind == "steer":
            method = "fsteer" if a.new_fw else "reissue"
        ev = dict(id=self.ev_id, t=self.t, actor=a.name, other=other.name, kind=kind, scale=scale,
                  steer=end, pred_before=pred_before, pred_after=pred_after, method=method, clear_since=None,
                  min_cc=float("inf"))
        a.adj = ev
        self.counts["reflex"][kind] = self.counts["reflex"].get(kind, 0) + 1
        if kind == "stop":
            self.trophy_break(a, "reflex hard stop: %s predicted %.1f cm from %s in %.1f s" % (
                a.name, pred_before, other.name, HARD_T_S))
        self.send_motion(a, scale, end)
        a.scale = scale
        if end is not None:
            a.move["end"] = end
            a.move["h1"] = math.degrees(math.atan2(end[1] - self.pos(a)[1], end[0] - self.pos(a)[0]))
        self.retraj(a)

    def replan(self, a, other, pred_before):
        """Neither a speed nor a steer keeps the gap: this drive ends and the planner picks a new move from here (a
        turn on the spot first: the wheels keep turning). Logged as a reflex event of kind replan."""
        self.ev_id += 1
        self.counts["reflex"]["replan"] = self.counts["reflex"].get("replan", 0) + 1
        mc = math.dist(self.pos(a), self.pos(other))
        self.log("reflex", dict(id=self.ev_id, t=round(self.t, 2), actor=a.name, other=other.name, kind="replan",
                                scale="", steer="", pred_before=round(pred_before, 2), pred_after="",
                                method="halt+new move", dur_s=0.0, actual_min_cc=round(mc, 2),
                                outcome="kept" if mc >= REFLEX_CC - 1.0 else (
                                    "closer than the gap, no touch" if mc >= HARD_CC else "circles touched"),
                                closed="replanned"))
        self.halt(a, "reflex: no speed or steer keeps %.1f cm from %s: new move" % (REFLEX_CC, other.name))
        self.end_move(a)
        a.busy_until = self.t

    def send_motion(self, a, scale, end):
        if a.new_fw:
            if end is not None:
                self.control(a, "fsteer %.2f %.2f" % end, "fsteer")
            else:
                self.control(a, "fspd %.2f" % scale, "fspd")
            return
        # older firmware: halt, then the rest of the drive as a new step at the new speed / to the new end
        self.control(a, "halt", "halt")
        mv = a.move
        for st in a.steps.values():
            if st["status"] in ("queued_local", "sent", "acked", "started"):
                st["status"] = "dropped"
        if scale <= 0.0:
            mv["drive_id"] = None
            mv["paused"] = True
            return
        p = self.pos(a)
        tgt = end if end is not None else mv["end"]
        h = math.degrees(math.atan2(tgt[1] - p[1], tgt[0] - p[0]))
        mv["drive_id"] = self.send_step(a, "fs %%d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (
            p[0], p[1], tgt[0], tgt[1], h, max(2.5, mv["v"] * scale)))
        mv["phase"] = "drive"
        self.pump(a)

    def retraj(self, a):
        mv = a.move
        p = self.pos(a)
        if mv is None or p is None:
            return
        L = math.dist(p, mv["end"])
        v = max(0.5, mv["v"] * a.scale)
        if a.scale <= 0.0:
            tr = Traj([(self.t, self.t + 3.0, p[0], p[1], self.margin0(a), SLACK0_S)])
            tr2 = Traj.move(self.t + 3.0, p, mv["h1"], 0.0, L, mv["v"], m0=self.margin0(a))
            a.traj = Traj(np.vstack([np.stack([tr.lo, tr.hi, tr.x, tr.y, tr.m, tr.sl], 1),
                                     np.stack([tr2.lo, tr2.hi, tr2.x, tr2.y, tr2.m, tr2.sl], 1)]))
            a.traj.end = mv["end"]
            return
        h = math.degrees(math.atan2(mv["end"][1] - p[1], mv["end"][0] - p[0]))
        a.traj = Traj.move(self.t, p, h, 0.0, L, v, m0=self.margin0(a))

    def adj_watch(self):
        for a in self.R.values():
            ev = a.adj
            if ev is None:
                continue
            o = self.R[ev["other"]]
            pa, po = self.pos(a), self.pos(o)
            if pa and po:
                ev["min_cc"] = min(ev["min_cc"], math.dist(pa, po))
            if a.move is None:
                self.close_adj(a, "move ended")
                continue
            if ev["kind"] == "steer":
                self.close_adj(a, "steered")
                continue
            m = self.motion(a, scale=1.0)
            if a.move.get("paused"):
                p = self.pos(a)
                m = (p, m[1], a.move["v"], math.dist(p, a.move["end"])) if m[2] == 0 else m
                if m[2] == 0:
                    p = self.pos(a)
                    e = a.move["end"]
                    rem = math.dist(p, e)
                    u = ((e[0] - p[0]) / rem, (e[1] - p[1]) / rem) if rem > 0.5 else (0.0, 0.0)
                    m = (p, u, a.move["v"], rem)
            g, _, _ = self.worst_for(a, m)
            if ev["kind"] == "stop" and g < 2.0 and self.t - ev["t"] >= 0.6:
                # a full stop lasts only until the planner has a move that goes away (Nick: no stops)
                o = self.R[ev["other"]]
                self.halt(a, "after the hard stop: a new move away from %s" % o.name)
                self.end_move(a)
                a.busy_until = self.t
                continue
            if g >= 2.0:
                if ev["clear_since"] is None:
                    ev["clear_since"] = self.t
                elif self.t - ev["clear_since"] >= CLEAR_HOLD_S:
                    a.scale = 1.0
                    if a.new_fw:
                        self.control(a, "fspd 1.00", "fspd")
                    else:
                        if a.move.get("paused"):
                            a.move.pop("paused", None)
                        self.send_motion(a, 1.0, None)
                    self.retraj(a)
                    self.close_adj(a, "clear")
            else:
                ev["clear_since"] = None
                if ev["kind"] == "speed" and self.t - ev["t"] > 0.4:
                    gc, oc, tm = self.worst_for(a, self.motion(a))
                    if gc < 0 and oc is not None:
                        # the speed in force no longer keeps the gap: escalate (steer, then a new move; a hard stop
                        # only when the circles would touch within HARD_T_S)
                        pb = gc + self.need(a, oc)
                        self.close_adj(a, "escalated")
                        if pb < HARD_CC and tm <= HARD_T_S:
                            self.apply(a, oc, "stop", 0.0, None, pb, pb)
                        elif not self.try_adjust(a, oc, pb):
                            self.replan(a, oc, pb)

    def close_adj(self, a, why):
        ev = a.adj
        a.adj = None
        if ev is None:
            return
        if ev["kind"] == "stop":
            self.close_break(a)
        mc = ev["min_cc"]
        out = "kept" if mc >= REFLEX_CC - 1.0 else ("closer than the gap, no touch" if mc >= HARD_CC else
                                                     "circles touched")
        self.log("reflex", dict(id=ev["id"], t=round(ev["t"], 2), actor=ev["actor"], other=ev["other"],
                                kind=ev["kind"], scale=ev["scale"],
                                steer="" if ev["steer"] is None else "%.1f;%.1f" % tuple(ev["steer"]),
                                pred_before=round(ev["pred_before"], 2), pred_after=round(min(ev["pred_after"], 99), 2),
                                method=ev["method"], dur_s=round(self.t - ev["t"], 2),
                                actual_min_cc=round(mc, 2) if mc < 1e9 else "", outcome=out, closed=why))

    # -- battery, parking ----------------------------------------------------------------------------------------
    def park_spot(self, r):
        p = self.pos(r)
        if p is None:
            return None
        x0, y0, x1, y1 = self.rect
        k = PARK_EDGE_CC
        cands = []
        for i in range(48):
            f = i / 48.0
            per = 2 * (x1 - x0 - 2 * k) + 2 * (y1 - y0 - 2 * k)
            s = f * per
            w, hgt = x1 - x0 - 2 * k, y1 - y0 - 2 * k
            if s < w:
                q = (x0 + k + s, y0 + k)
            elif s < w + hgt:
                q = (x1 - k, y0 + k + s - w)
            elif s < 2 * w + hgt:
                q = (x1 - k - (s - w - hgt), y1 - k)
            else:
                q = (x0 + k, y1 - k - (s - 2 * w - hgt))
            cands.append(q)
        cands.sort(key=lambda q: math.dist(q, p))
        h = self.heading(r)
        for q in cands[:24]:
            L = math.dist(q, p)
            if L > 90:
                continue
            d = wrap180(math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) - h) if L > 1 else 0.0
            if L > 1 and not self.can_turn_here(r):
                continue
            tr = self.make(r, d, L, 5.0)
            if all(math.dist(q, self.pos(o)) > PLAN_CC + 8 for o in self.R.values()
                   if o is not r and self.pos(o) is not None) and self.feasible(r, tr):
                return d, L, tr
        return None

    def go_park(self, r, why):
        self.event(r, "parking", why)
        sp = self.park_spot(r)
        r.mode = "parking"
        if sp is None:
            self.event(r, "parking", "no free spot by an edge reachable now: parks where it stands")
            self.parked(r)
            return
        d, L, tr = sp
        p, h = self.pos(r), self.heading(r)
        h1 = wrap180(h + d)
        mv = dict(kind="park", cell=None, turn=d, L=L, v=5.0, t0=self.t, p0=p, h0=h, h1=h1, end=tr.end,
                  phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None)
        if abs(d) >= 3.0:
            mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % h1)
        mv["drive_id"] = self.send_step(r, "fs %%d D %.2f %.2f %.2f %.2f %.2f 5.00 0 -" % (
            p[0], p[1], tr.end[0], tr.end[1], h1))
        r.move, r.traj = mv, tr
        self.pump(r)

    def parked(self, r):
        p = self.pos(r)
        r.mode = "out"
        r.out_why = r.out_why or "battery"
        if p is not None:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r) + 1.0)
        self.out(r, "stop")
        if self.park_sleep:
            self.out(r, "sleep 0")
        self.event(r, "out", "parked at %s and stopped (%s)" % ("%.1f, %.1f" % p if p else "?", r.out_why))

    # -- every tick ----------------------------------------------------------------------------------------------
    def tick(self, t):
        self.t = t
        if self.t_start is None:
            self.t_start = t
        for r in self.R.values():
            self.watch(r)
        self.camera_timing()
        if self.keepouts and int(t * 2) != int((t - 0.05) * 2):
            self.clear_keepouts()
        self.reflex()
        self.adj_watch()
        for r in self.R.values():
            if r.mode == "idle" and t >= r.busy_until and self.pos(r) is not None and not r.silent:
                if r.lost is not None and r.bubble > BUBBLE_CAP:
                    self.find_me(r)
                elif r.battery is not None and r.battery <= self.floor:
                    r.out_why = "battery %d %%" % r.battery
                    self.go_park(r, "battery %d %% (floor %d %%)" % (r.battery, self.floor))
                elif not r.lost_hold:
                    self.start_move(r)
            if r.mode not in ("gone",):
                self.pump(r)
            if r.mode not in ("out", "gone", "start") and t - r.last_sent > HEARTBEAT_S:
                self.out(r, "status")
                r.last_status = t
            elif r.mode not in ("gone", "start") and t - r.last_status > STATUS_S:
                self.out(r, "status")
                r.last_status = t
        if t - self.last_motion >= 0.5:
            self.last_motion = t
            self.motion_row()

    def watch(self, r):
        if r.mode in ("out", "gone", "start"):
            return
        t = self.t
        # silence
        if r.heard is not None and t - r.heard > SILENT_S and not r.silent:
            r.silent, r.silent_t = True, t
            self.event(r, "silent", "nothing heard for %.0f s: kept as an obstacle where the camera sees it" % SILENT_S)
            p = self.pos(r)
            if p:
                r.traj = Traj.stand(t, *p, m=self.margin0(r))
        if r.silent:
            if r.cam is not None and self.fresh(r):
                r.traj = Traj.stand(t, r.cam[1], r.cam[2], m=self.margin0(r))
            if t - r.silent_t > OUT_SILENT_S:
                r.mode, r.out_why = "out", "silent"
                self.event(r, "out", "silent for %.0f s: out of the run (an obstacle where it is)" % OUT_SILENT_S)
            return
        # nothing from the robot for a moment while it should be moving: it may have stopped where the camera has it
        if r.own is not None and t - r.own[0] > 1.0 and r.mode == "move" and r.cam is not None and \
                not getattr(r, "stale_cover", False):
            r.stale_cover = True
            r.traj = concat(Traj.stand(t, r.cam[1], r.cam[2], m=self.margin0(r)), r.traj) if r.traj is not None \
                else Traj.stand(t, r.cam[1], r.cam[2], m=self.margin0(r))
        elif r.own is not None and t - r.own[0] <= 1.0:
            r.stale_cover = False
        # lost by the camera
        if r.lost is None and r.cam is not None and not self.fresh(r):
            self.start_lost(r)
        if r.lost is not None and r.mode == "move" and r.move is not None:
            p = self.pos(r)
            if p and (r.bubble > BUBBLE_CAP or edge_dist(self.rect, *p) - r.bubble < TURN_EDGE_CC) and \
                    r.move["kind"] != "wiggle":
                self.ease_stop(r, "bubble %.1f cm" % r.bubble)
        # stuck: driving, wheels on, no camera travel
        mv = r.move
        if mv is not None and mv["phase"] == "drive" and r.scale > 0.3 and r.lost is None and mv.get("t_drive") and \
                t - mv["t_drive"] > STUCK_S + 1.0:
            hist = [q for q in r.cam_hist if q[0] > t - STUCK_S]
            if len(hist) >= 5 and math.dist(hist[0][1:], hist[-1][1:]) < 1.0 and r.own and r.own[4] == "drive":
                self.event(r, "stuck", "drive commanded, the camera saw it move %.1f cm in %.0f s: new move" % (
                    math.dist(hist[0][1:], hist[-1][1:]), STUCK_S))
                self.halt(r, "stuck")
                self.end_move(r)
        # a move that takes far too long
        if mv is not None:
            est = turn_s(mv["turn"]) + (drive_s(mv["L"], mv["v"]) if mv["L"] else 0.0)
            if t - mv["t0"] > 3.0 * est + 15.0 and not mv.get("paused"):
                self.event(r, "slow move", "move %d took %.0f s (estimate %.0f): ended" % (mv["n"], t - mv["t0"], est))
                self.halt(r, "move timed out")
                self.end_move(r)

    def ease_stop(self, r, why):
        if r.new_fw:
            self.control(r, "fspd 0.30", "fspd")
        self.halt(r, "eases to a stop: " + why)
        self.event(r, "find me", "bubble too big (%s): eases to a stop, then wiggles" % why)
        self.end_move(r)

    def find_me(self, r):
        if self.t < r.wiggle_due:
            return
        if not self.can_turn_here(r):
            r.wiggle_due = self.t + 3.0
            if not r.stopped_hard:
                self.trophy_break(r, "lost near an edge: stands until seen")
            return
        p, h = self.pos(r), self.heading(r)
        mv = dict(kind="wiggle", cell=None, turn=WIGGLE_DEG, L=0.0, v=5.0, t0=self.t, p0=p, h0=h, h1=h, end=p,
                  phase="turn", n=r.n_moves + 1, drive_id=None, turn_id=None, t_drive=None)
        self.send_step(r, "fs %%d T %.2f -" % wrap180(h + WIGGLE_DEG))
        self.send_step(r, "fs %%d T %.2f -" % wrap180(h - WIGGLE_DEG))
        mv["turn_id"] = self.send_step(r, "fs %%d T %.2f -" % wrap180(h))
        r.move, r.mode = mv, "move"
        r.traj = Traj.stand(self.t, *p, m=self.margin0(r))
        r.n_moves += 1
        r.wiggle_due = self.t + 4.0
        self.fallback(r, 6, "find-me wiggle", "lost, bubble %.1f cm over %.0f" % (r.bubble, BUBBLE_CAP))
        self.pump(r)

    def motion_row(self):
        for r in self.R.values():
            h = [q for q in r.own_hist if q[0] >= self.t - 0.6]
            trans = anym = 0
            if r.own is not None and self.t - r.own[0] < 1.0 and len(h) >= 2 and r.mode not in ("out", "gone"):
                d = math.dist(h[0][1:3], h[-1][1:3])
                dth = abs(wrap180(h[-1][3] - h[0][3]))
                dt = max(0.1, h[-1][0] - h[0][0])
                trans = int(d / dt >= 1.0)
                anym = int(trans or dth / dt >= 4.0)
            p = self.pos(r)
            self.log("motion", dict(t=round(self.t, 2), robot=r.name, any=anym, translating=trans,
                                    x=round(p[0], 1) if p else "", y=round(p[1], 1) if p else "",
                                    bubble=round(r.bubble, 2) if r.lost else 0.0, mode=r.mode,
                                    lost=int(r.lost is not None), battery=r.battery if r.battery is not None else ""))

    def begin(self, name):
        """The robot is following (run follow) and seen: it joins the roam."""
        r = self.R[name]
        r.mode = "idle"
        p = self.pos(r)
        if p:
            r.traj = Traj.stand(self.t, *p, m=self.margin0(r))

    def sent(self, name):
        """The runner forwarded something to this robot outside the controller (a camera fix): heartbeat met."""
        r = self.R.get(name)
        if r is not None:
            r.last_sent = self.t

    def all_out(self):
        return all(r.mode in ("out", "gone") for r in self.R.values())
