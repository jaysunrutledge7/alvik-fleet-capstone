#!/usr/bin/env python3
"""ONE tracking layer (e09, Phase B1, 7 Nov; Nick: one pipeline, whatever the site): every camera source a site has
goes into the same layer, which says which robot each sighting is, and hands out named fixes (with the source and its
sigma) to the robot's filter (fused_pose.py) and the planner. Used by free roam (home) and Mission Control (school),
by the Pi's bridges (cam_bridge.py, tag_bridge.py) and by the fast simulator (freeroam_fastsim.py --camera detections).

Sources (each a switch; SOURCES_BY_TRACKING gives a site's defaults):
  tags     roof AprilTags: the tag's id names the robot (school). Read only where the picture is good enough (about
           x < 100 cm of the school table, room light)
  motion   the motion detector (floor memory, e02_multitrack): an outline, NO name; it does not see a robot that stands
           still, and loses robots on dark patches and against a busy background
  lock     the lock-on tracker (e02_lock): seeded on ONE robot (by a tag read, or by a sure motion sighting), it follows
           that robot's own look picture by picture, standing or moving. 22-23 Oct evening losses: seen in 94 / 93 / 96 %
           of pictures against 41 / 40 / 72 % for the detector alone (tests/test_e02_realclips.py). Its name is the
           robot it was seeded on, so a lock can carry a WRONG name if it jumped to a look-alike: it is checked like an
           unnamed sighting whenever two robots are close
  lights   the robots' lights (brightest spot near the prediction; home): no name
  ml       the ML detector (tools/detector, the MIRLT model): a slot, switched OFF (MIRLT_ML_DETECTOR=1 turns it on once
           the model's real accuracy is measured); handled as an unnamed sighting

Who is who ("identity"):
  - a tag names its robot; everything else is matched to the robots' predictions (their filter's pose, or their own
    pose) inside a gate, the least total distance; a sighting about as near two predictions (CONTESTED_CM), or one
    outline as big as two (merged), names NOBODY (unsure is flagged, never guessed: the 27 Oct swap was a guess);
  - WIGGLE FIRST (identity="wiggle", the default): two robots that came within CLOSE_CM of each other are a "close
    pass". When they part, each camera track's motion since the pass is compared with each robot's OWN motion (its
    wheels and gyro): kept or swapped, whichever explains it clearly better (SWAP_MARGIN_CM). When neither does (both
    stood, or moved alike) the layer asks for a WIGGLE of one of them (a quarter turn and back on its spot): the outline
    that turns in place without moving is that robot. Until then neither robot takes a camera fix (it drives on its
    filter, its uncertainty growing). Four swaps on the real table (22 Oct Right for Left; 27 Oct Left with a parked
    Right; 30 Oct lights matched twice; 5 Nov the camera put Center on Right's spot) were each fixed piecewise before;
  - identity="predict" (the switch's other side): nearest prediction only, as cam_bridge did before e09.

Also here (shared by both sites):
  - seeding: a tag read seeds the motion detector and the lock-on on THAT robot, so where tags stop reading (the far
    half of the school table) the robot is still followed, by name;
  - the parked-robot watch: a robot whose own wheels stand still while the camera sees it move PARKED_MOVE_CM is
    "moved while parked" (pushed by another robot, a hand): its pose is taken from the camera (evidence wins), the
    moment is logged and a picture asked for;
  - the brick ("what changed") watch: a sighting no robot can claim that stays put BRICK_S is a new obstacle (a hand
    passing is not: it moves or goes); the image-level watch (brickwatch.py) does the same on the bridges' pictures.

Numbers: every number below says whether it was measured or chosen; the chosen ones are listed in the e09 report's
Sweep 2 with whether a DOE should set them.
"""
import itertools
import math
import os

VERSION = "tracking-layer-2026-11-07b"   # b: camera watch off by default (Nick 7 Nov: off until its false alarms are understood)

SOURCES_BY_TRACKING = {"lights": ("motion", "lock", "lights"), "tags": ("tags", "motion", "lock"),
                       "none": ()}
OLD_SOURCES = {"lights": ("motion", "lights"), "tags": ("tags",), "none": ()}     # before e09 (the switch)
ML_DETECTOR = os.environ.get("MIRLT_ML_DETECTOR", "0") == "1"     # the ML slot: OFF until measured
IDENTITY = os.environ.get("MIRLT_IDENTITY", "wiggle")              # "wiggle" (e09) | "predict" (before)
CAMERA_WATCH = os.environ.get("MIRLT_CAMERA_WATCH", "0") == "1"    # the camera watch (turn directions); OFF by default (Nick 7 Nov, false alarms in 2 of 12 e09 runs); 1: on

GATE_CM = 6.0            # as e02_multitrack.GATE_CM: a sighting this near a prediction can be that robot ...
GATE_GROW_CMS = 10.0     # ... plus this much per second since it was last seen for sure (e02_multitrack)
GATE_PER_CM = 0.15       # CHOSEN (e09): the gate grows 0.15 cm per cm the robot's own wheels drove since it was sure
GATE_ROOT_CM = 2.0       # ... and 2 cm x sqrt(seconds) (a robot pushed, a slow drift); DOE: yes, with GATE_MAX_CM
GATE_MAX_CM = 25.0       # CHOSEN: e02_multitrack's 18 cm, widened for the filter's longer blind stretches
CONTESTED_CM = 2.5       # as e02_multitrack.AMBIG_CM: two explanations this close: nobody is named
MERGE_AREA = 1.55        # as e02_multitrack.MERGE_AREA: an outline this many robot areas big is two robots merged
CLOSE_CM = 16.0          # CHOSEN: middles this close = a close pass (bodies within ~6.5 cm: where the 27 Oct and
                         # 5 Nov swaps happened); DOE: yes, against swaps caught vs wiggles asked per hour
PART_CM = 22.0           # CHOSEN: ... and parted again past this (hysteresis)
SWAP_WIN_S = 2.5         # CHOSEN: the motion since the pass compared over at most this long
SWAP_MARGIN_CM = 2.0     # CHOSEN: one explanation must beat the other by this (summed displacement error)
MIN_MOTION_CM = 3.0      # CHOSEN: a track that moved less than this cannot show who it is by moving: wiggle
WIGGLE_S = 2.5           # the wiggle's watch window (a quarter turn and back at 92 deg/s: ~2 s, as wiggle_bind's)
WIGGLE_STILL_CM = 1.5    # CHOSEN: the wiggling robot's outline stays within this of its spot
WIGGLE_NEAR_CM = 12.0    # CHOSEN: only outlines turning within this of either robot's spot count for the wiggle
WIGGLE_CLEAR = 0.5       # CHOSEN: the turning outline must be at most half as far from one robot as from the other
WIGGLE_RETRY_S = 6.0     # a wiggle that showed nothing is asked again after this
NO_WIGGLE_WAIT_S = 8.0   # CHOSEN: with no wiggle possible, how long the pair waits for a tag read or clear motion
PARKED_STILL_S = 2.0     # CHOSEN: wheels standing this long = parked (for the parked watch)
PARKED_MOVE_CM = 3.0     # CHOSEN: the camera's spot moved this far while parked = moved (3 camera sigmas)
PARKED_MAX_CM = 15.0     # CHOSEN: ... and at most this far (a push or a hand's nudge; a jump further is a misread or
                         # another robot, never a reason to move a robot's pose)
PARKED_EVERY_S = 10.0    # CHOSEN: at most one parked-move a robot this often
PARKED_REF_N = 10        # CHOSEN: sightings averaged for where it was parked ...
PARKED_NOW_N = 6         # ... and for where it is now
BRICK_S = 2.0            # CHOSEN: an unclaimed sighting standing BRICK_S = a new object (brickwatch: 3 checks at 4 Hz
                         # = 0.75 s after the hand leaves; here the fast simulator's sightings, 12 a second)
BRICK_STILL_CM = 3.0     # as brickwatch.STILL_CM
BRICK_R_CM = 6.0         # CHOSEN: the keep-out round a confirmed object (a brick's half width + a margin)
CLUSTER_CM = 3.5         # CHOSEN: sightings of different sources this close are ONE object (a robot's outline, its lights
                         # and its lock), matched as one: one robot's lights can then not be handed to another robot whose
                         # gate has grown (30 Oct: lights matched to the wrong robot twice). DOE: no, under a body's half
LOCK_SEED_STILL_CM = 1.0     # as e02_camera RELOCK_STILL_CM: re-lock on a robot the detector sees standing still
CAM_CHECK_CM = 6.0       # CHOSEN: a camera checkpoint every 6 cm of a robot's camera travel
CAM_TURN_DEG = 20.0      # CHOSEN: over three checkpoints its own heading turned at least this much: a vote
CAM_CHECK_N = 6          # CHOSEN: the last 6 votes (any robots) ...
CAM_BAD_N = 4            # ... 4 of them the other way: the picture is not trusted (one robot's slip or swap is not enough)
CAM_GOOD_N = 4           # CHOSEN: 4 agreeing in a row: trusted again
SOURCE_SIGMA = {"tags": 0.6, "motion": 1.0, "lock": 1.2, "lights": 1.5, "ml": 1.5}   # see fused_pose.SRC_SIGMA
SOURCE_RANK = {"tags": 0, "lock": 1, "motion": 2, "lights": 3, "ml": 4}   # several for one robot: the best one


def sources_for(tracking, old=False, ml=None):
    s = list((OLD_SOURCES if old else SOURCES_BY_TRACKING).get(tracking, ()))
    if (ML_DETECTOR if ml is None else ml) and "ml" not in s and not old:
        s.append("ml")
    return tuple(s)


class Det:
    """One sighting. name: set by tags (the id) and by a lock (the robot it was seeded on); None otherwise."""
    __slots__ = ("src", "t", "x", "y", "th", "name", "sigma", "score", "area", "spin", "lock_id")

    def __init__(self, src, t, x, y, th=None, name=None, sigma=None, score=1.0, area=1.0, spin=False, lock_id=None):
        self.src, self.t, self.x, self.y, self.th, self.name = src, t, float(x), float(y), th, name
        self.sigma = SOURCE_SIGMA.get(src, 1.0) if sigma is None else float(sigma)
        self.score, self.area, self.spin, self.lock_id = score, area, spin, lock_id


class Track:
    def __init__(self, name):
        self.name = name
        self.sure = None            # (t, x, y) last sure sighting
        self.own = []               # [(t, x, y, th, still)] own pose / filter (prediction and identity by motion)
        self.cam = []               # [(t, x, y)] the sightings given to it
        self.src_n = {}
        self.unsure = None          # why no fix now (close pass, contested, wiggle) or None
        self.parked_at = None       # (t0, x, y) camera spot when its wheels stopped
        self.lock_seeded = None     # t of the lock's last seed
        self.seen_t = None


class TrackingLayer:
    def __init__(self, names, sources=("motion", "lock", "lights"), identity=None, parked_watch=True,
                 brick_watch=True, log=None, lock_seed=None, wiggle=None, cluster=True, camera_watch=None):
        """log(kind, row); lock_seed(name, t, x, y): ask the lock-on to lock on that robot there (bridge / simulator);
        wiggle(name, why): ask the controller for an identity wiggle of that robot (None: no wiggle can be made, the
        robots keep driving on their filters until motion tells them apart)."""
        self.names = list(names)
        self.sources = tuple(sources)
        self.identity = identity or IDENTITY
        self.parked_watch = parked_watch
        self.brick_watch = brick_watch
        self.log = log or (lambda kind, row: None)
        self.lock_seed_fn = lock_seed
        self.wiggle_fn = wiggle
        self.cluster = cluster
        self.camera_watch = CAMERA_WATCH if camera_watch is None else bool(camera_watch)
        self.cam_ok = True           # the camera watch: False while the picture moves unlike the robots' own wheels
        self.cam_good = 0            # agreeing votes so far: the camera is PROVEN after CAM_GOOD_N (until then a camera
                                     # spot may not move a robot's own pose by itself: controller.camera_proven)
        self.cam_checks = []         # recent checks: True (agrees) / False
        self.cam_ref = {}            # name -> (t, cam x, y, own x, y) where the next check is measured from
        self.T = {n: Track(n) for n in self.names}
        self.pairs = {}             # frozenset(a, b) -> dict(t0, parted_t, state)
        self.wiggles = {}           # name -> dict(t0, pair, at)
        self.unclaimed = []         # [dict(t0, t, x, y, n)] for the brick watch
        self.obstacles = []
        self.counts = dict(frames=0, fixes=0, contested=0, merged=0, close_passes=0, kept=0, swapped=0, wiggles=0,
                           wiggle_kept=0, wiggle_swapped=0, wiggle_none=0, parked_moved=0, bricks=0, seeds=0,
                           tag_jumps=0, refused_close=0, lock_used=0, lock_checked=0)
        self.by_src = {}
        self.last_fix = {}          # name -> (t, src)

    # -- inputs from the robots --------------------------------------------------------------------------------------
    def own(self, name, t, x, y, th, still):
        tr = self.T.get(name)
        if tr is None:
            return
        tr.own.append((t, float(x), float(y), th, bool(still)))
        if len(tr.own) > 400:
            del tr.own[:100]
        if not still:
            tr.parked_at = None

    def _own_at(self, tr, t):
        h = tr.own
        if not h:
            return None
        if t <= h[0][0]:
            return h[0]
        if t >= h[-1][0]:
            return h[-1]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if h[m][0] <= t:
                lo = m
            else:
                hi = m
        a, b = h[lo], h[hi]
        f = (t - a[0]) / max(1e-6, b[0] - a[0])
        return (t, a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), b[3], b[4])

    def predict(self, name, t):
        """(x, y, gate) where the robot should be at t: the last sure sighting moved on by its own motion since."""
        tr = self.T[name]
        if tr.sure is None:
            o = self._own_at(tr, t)
            return None if o is None else (o[1], o[2], GATE_MAX_CM)
        ts, xs, ys = tr.sure
        a, b = self._own_at(tr, ts), self._own_at(tr, t)
        x, y = (xs + b[1] - a[1], ys + b[2] - a[2]) if (a is not None and b is not None) else (xs, ys)
        if self.identity == "predict" or a is None or b is None:
            gate = min(GATE_MAX_CM, GATE_CM + GATE_GROW_CMS * max(0.0, t - ts))      # e02_multitrack's, by time
        else:
            # (e09) the gate grows with what the robot's own wheels say it did since, not with the clock: a robot
            # standing still keeps a tight gate (a wrong spot near it is not taken), one driving blind a wide one
            trav = self._travel(tr, ts, t)
            gate = min(GATE_MAX_CM, GATE_CM + GATE_PER_CM * trav + GATE_ROOT_CM * math.sqrt(max(0.0, t - ts)))
        return (x, y, gate)

    def _travel(self, tr, ta, tb):
        d, prev = 0.0, None
        for q in tr.own:
            if q[0] < ta:
                prev = q
                continue
            if q[0] > tb:
                break
            if prev is not None:
                d += math.hypot(q[1] - prev[1], q[2] - prev[2])
            prev = q
        return d

    def seed(self, name, t, x, y, why="tag"):
        """A sure, named sighting (a tag read, a lights find, the start spot): the robot's track starts there and the
        lock-on is seeded on it (when the robot is not close to another: a seed must not land on a look-alike)."""
        tr = self.T.get(name)
        if tr is None:
            return
        tr.sure = (t, float(x), float(y))
        if "lock" in self.sources and self.lock_seed_fn is not None and not self._close_to_other(name, x, y) and (
                tr.lock_seeded is None or t - tr.lock_seeded > 1.0):
            tr.lock_seeded = t
            self.counts["seeds"] += 1
            self.lock_seed_fn(name, t, x, y)

    def _close_to_other(self, name, x, y, d=CLOSE_CM):
        for n, o in self.T.items():
            if n != name and o.sure is not None and math.hypot(o.sure[1] - x, o.sure[2] - y) < d:
                return True
        return False

    # -- one picture's sightings ---------------------------------------------------------------------------------------
    def feed(self, t, dets):
        """All sightings of one moment (any sources). Returns [dict(robot, t, x, y, th, sigma, src)]."""
        self.counts["frames"] += 1
        dets = [d for d in dets if d.src in self.sources]
        preds = {n: self.predict(n, t) for n in self.names}
        self._pairs(t, preds)
        out, named = {}, set()
        # 1. tags: the id is the name (a read far from the prediction is still believed: the tag knows; said)
        for d in dets:
            if d.src == "tags" and d.name in self.T:
                p = preds.get(d.name)
                if p is not None and math.hypot(d.x - p[0], d.y - p[1]) > GATE_MAX_CM:
                    self.counts["tag_jumps"] += 1
                    self.log("tracking", dict(t=round(t, 2), robot=d.name, what="tag read away from the prediction",
                                              d_cm=round(math.hypot(d.x - p[0], d.y - p[1]), 1)))
                out[d.name] = d
                named.add(d.name)
        # 2. the locks: a lock's name stands unless its robot is in a close pass or a wiggle (then it is checked as an
        #    unnamed sighting) or it is off its prediction's gate
        free = []
        for d in dets:
            if d.src == "tags":
                continue
            if d.src == "lock" and d.name in self.T and d.name not in named:
                p = preds.get(d.name)
                tr = self.T[d.name]
                if tr.unsure is None and p is not None and math.hypot(d.x - p[0], d.y - p[1]) <= p[2] and \
                        not self._in_pair(d.name):
                    out[d.name] = d
                    named.add(d.name)
                    self.counts["lock_used"] += 1
                    continue
                self.counts["lock_checked"] += 1
            free.append(d)
        # 3. unnamed (and checked locks): matched to predictions (several sources' sightings of one object: one)
        if self.cluster:
            free = self._clusters(free)
        want = [n for n in self.names if n not in named and preds.get(n) is not None]
        claimed = set()
        if want and free:
            asg = self._assign(t, want, preds, free)
            for n, d in asg.items():
                out[n] = d
                claimed.add(id(d))
        # 4. identity guard: robots in an unresolved close pass or wiggle take no camera fix
        fixes = []
        for n, d in out.items():
            tr = self.T[n]
            tr.sure = (t, d.x, d.y)          # the track goes on (who it is is checked when the pair parts) ...
            tr.seen_t = t
            tr.cam.append((t, d.x, d.y))
            if len(tr.cam) > 200:
                del tr.cam[:50]
            if (tr.unsure is not None or self._in_pair(n)) and d.src != "tags":
                self.counts["refused_close"] += 1    # ... but no fix: its filter drives on its own sensors meanwhile,
                continue                             # so its own motion stays clean evidence for the check
            if self.camera_watch and not self._camera_check(n, t, d) and d.src != "tags":
                self.counts["refused_camera"] = self.counts.get("refused_camera", 0) + 1
                continue
            tr.src_n[d.src] = tr.src_n.get(d.src, 0) + 1
            self.by_src[d.src] = self.by_src.get(d.src, 0) + 1
            if d.src == "tags":
                tr.unsure = None
                self.seed(n, t, d.x, d.y, "tag")
            elif d.src == "motion" and "lock" in self.sources and (tr.lock_seeded is None or t - tr.lock_seeded > 3.0):
                # the detector sees it sure and nobody near: (re)lock on to it
                if not self._close_to_other(n, d.x, d.y):
                    self.seed(n, t, d.x, d.y, "motion")
            self.last_fix[n] = (t, d.src)
            self._parked(n, t, d)
            fixes.append(dict(robot=n, t=t, x=d.x, y=d.y, th=d.th, sigma=d.sigma, src=d.src))
        self.counts["fixes"] += len(fixes)
        # 5. wiggles in progress, close passes that parted, things nobody claims
        self._wiggle_watch(t, dets)
        self._parted(t)
        if self.brick_watch:
            self._bricks(t, [d for d in free if id(d) not in claimed and d.src in ("motion", "ml")], preds)
        return fixes

    def _clusters(self, dets):
        groups = []
        for d in sorted(dets, key=lambda q: SOURCE_RANK.get(q.src, 5)):
            g = next((g for g in groups if any(math.hypot(d.x - e.x, d.y - e.y) < CLUSTER_CM for e in g)), None)
            if g is None:
                groups.append([d])
            else:
                g.append(d)
        out = []
        for g in groups:
            best = g[0]
            area = max(e.area for e in g)
            if area > best.area:
                best = Det(best.src, best.t, best.x, best.y, best.th, best.name, best.sigma, best.score, area,
                           best.spin or any(e.spin for e in g), best.lock_id)
            elif any(e.spin for e in g) and not best.spin:
                best.spin = True
            out.append(best)
        return out

    def _camera_check(self, n, t, d):
        """Observe, then trust (the camera itself). The camera's track of a robot must TURN the way the robot's own gyro
        says it turned (a frame-free check: a wrong camera name, a mirrored or swapped calibration turns the other way;
        positions cannot be compared, they are pulled to the camera). Every CAM_CHECK_CM of camera travel a checkpoint;
        over three checkpoints the camera's turn against its own heading's change, when that is over CAM_TURN_DEG.
        CAM_BAD_N of the last CAM_CHECK_N votes (any robots) the other way: the picture is not trusted (no camera
        fixes; the robots drive on their filters and the run narrows); trusted again after CAM_GOOD_N agreeing votes in a
        row (the votes go on while it is not trusted). Returns whether this fix may be used."""
        tr = self.T[n]
        o = tr.own[-1] if tr.own else None
        if o is None or o[3] is None:
            return self.cam_ok
        cp = self.cam_ref.setdefault(n, [])
        if cp and t - cp[-1][0] > 10.0:
            del cp[:]
        if not cp or math.hypot(d.x - cp[-1][1], d.y - cp[-1][2]) >= CAM_CHECK_CM:
            cp.append((t, d.x, d.y, float(o[3])))
            del cp[:-3]
            if len(cp) == 3:
                (_, x0, y0, h0), (_, x1, y1, _), (_, x2, y2, h2) = cp
                a1, a2 = math.atan2(y1 - y0, x1 - x0), math.atan2(y2 - y1, x2 - x1)
                cam_turn = math.degrees(math.atan2(math.sin(a2 - a1), math.cos(a2 - a1)))
                own_turn = ((h2 - h0 + 180.0) % 360.0) - 180.0
                if abs(own_turn) >= CAM_TURN_DEG and abs(cam_turn) >= 0.3 * CAM_TURN_DEG:
                    self._cam_vote(t, (cam_turn > 0) == (own_turn > 0), n)
                elif abs(own_turn) >= CAM_TURN_DEG and abs(cam_turn) < 0.15 * CAM_TURN_DEG:
                    pass                     # a straight camera track while it turned a lot: no vote (a turn on its spot)
        return self.cam_ok

    @property
    def proven(self):
        return (not self.camera_watch) or (self.cam_ok and self.cam_good >= CAM_GOOD_N)

    def _cam_vote(self, t, ok, robot="-"):
        if ok:
            self.cam_good += 1
        self.cam_checks.append(ok)
        del self.cam_checks[:-CAM_CHECK_N]
        self.cam_bad_by = (getattr(self, "cam_bad_by", []) + [robot if not ok else None])[-CAM_CHECK_N:]
        self.counts["camera_votes"] = self.counts.get("camera_votes", 0) + 1
        bad_robots = {r for r in self.cam_bad_by if r is not None}
        if self.cam_ok and self.cam_checks.count(False) >= CAM_BAD_N and \
                len(bad_robots) >= min(2, len(self.names)):
            self.cam_ok = False
            self.counts["camera_distrusted"] = self.counts.get("camera_distrusted", 0) + 1
            self.log("tracking", dict(t=round(t, 2), robot="-", what="camera not trusted",
                                      why="%d of the last %d checks: the picture turned the robots the other way from "
                                          "their own gyros" % (self.cam_checks.count(False), len(self.cam_checks))))
        elif not self.cam_ok and len(self.cam_checks) >= CAM_GOOD_N and all(self.cam_checks[-CAM_GOOD_N:]):
            self.cam_ok = True
            self.log("tracking", dict(t=round(t, 2), robot="-", what="camera trusted again", why="checks agree"))

    def _assign(self, t, want, preds, free):
        """Least total distance inside the gates; contested / merged sightings name nobody."""
        cand = {}
        for n in want:
            px, py, g = preds[n]
            cand[n] = [(math.hypot(d.x - px, d.y - py), d) for d in free if math.hypot(d.x - px, d.y - py) <= g]
        res = {}
        for d in free:
            near = sorted((math.hypot(d.x - preds[n][0], d.y - preds[n][1]), n) for n in want
                          if math.hypot(d.x - preds[n][0], d.y - preds[n][1]) <= preds[n][2])
            if len(near) >= 2 and d.area >= MERGE_AREA:
                self.counts["merged"] += 1
                for _, n in near[:2]:
                    self._mark(n, t, "merged outline")
                for _, n in near:
                    cand[n] = [c for c in cand[n] if c[1] is not d]
            elif len(near) >= 2 and near[1][0] - near[0][0] < CONTESTED_CM and self.identity != "predict_loose":
                self.counts["contested"] += 1
                for _, n in near[:2]:
                    cand[n] = [c for c in cand[n] if c[1] is not d]
        ns = [n for n in want if cand[n]]
        if not ns:
            return res
        best, best_cost = None, None
        # small numbers of robots: every assignment (3 robots x a few sightings)
        opts = [[None] + [c[1] for c in sorted(cand[n], key=lambda q: q[0])[:4]] for n in ns]
        for combo in itertools.product(*opts):
            used = [id(d) for d in combo if d is not None]
            if len(used) != len(set(used)):
                continue
            cost = 0.0
            for n, d in zip(ns, combo):
                cost += (preds[n][2] * 1.2) if d is None else math.hypot(d.x - preds[n][0], d.y - preds[n][1])
            # prefer better sources for the same robot at equal distance
            cost += 0.01 * sum(SOURCE_RANK.get(d.src, 5) for d in combo if d is not None)
            if best_cost is None or cost < best_cost:
                best, best_cost = combo, cost
        for n, d in zip(ns, best):
            if d is not None:
                res[n] = d
        # several sources may see the same robot: keep the best-ranked one per robot (already one per robot here)
        return res

    def _mark(self, n, t, why):
        self.log("tracking", dict(t=round(t, 2), robot=n, what="unsure", why=why))

    # -- close passes and wiggles ------------------------------------------------------------------------------------
    def _in_pair(self, n):
        return any(n in k and v["state"] in ("close", "wiggle") for k, v in self.pairs.items())

    def _pairs(self, t, preds):
        if self.identity == "predict":
            return
        for a, b in itertools.combinations(self.names, 2):
            pa, pb = preds.get(a), preds.get(b)
            if pa is None or pb is None:
                continue
            k = frozenset((a, b))
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            st = self.pairs.get(k)
            if d < CLOSE_CM and (st is None or st["state"] == "done"):
                self.pairs[k] = dict(t0=t, state="close", parted_t=None, a=a, b=b,
                                     own0={a: self._own_at(self.T[a], t), b: self._own_at(self.T[b], t)},
                                     cam0={a: self.T[a].sure, b: self.T[b].sure})
                self.counts["close_passes"] += 1
                self.log("tracking", dict(t=round(t, 2), robot="%s+%s" % (a, b), what="close pass",
                                          d_cm=round(d, 1)))
            elif st is not None and st["state"] == "close" and d > PART_CM and st["parted_t"] is None:
                st["parted_t"] = t

    def _parted(self, t):
        for k, st in list(self.pairs.items()):
            if st["state"] != "close" or st["parted_t"] is None:
                continue
            if t - st["parted_t"] < 0.6:
                continue                # a few sightings each after they parted
            a, b = st["a"], st["b"]
            verdict, ev = self._swap_check(t, st)
            if verdict == "kept":
                st["state"] = "done"
                self.counts["kept"] += 1
                for n in (a, b):
                    self.T[n].unsure = None
            elif verdict == "swapped":
                st["state"] = "done"
                self.counts["swapped"] += 1
                self._swap(a, b, t, "their motions since the close pass (%s)" % ev)
            else:
                # neither explains it: wiggle first (both take no fix until it is known)
                for n in (a, b):
                    self.T[n].unsure = "identity after a close pass"
                if self.wiggle_fn is None:
                    # nobody can wiggle (the school's runs, a replay): no fixes for either until a tag read names it or
                    # their motion tells them apart (checked again each picture), at most NO_WIGGLE_WAIT_S; then
                    # believed as predicted, said
                    if t - st["parted_t"] < NO_WIGGLE_WAIT_S:
                        continue
                    st["state"] = "done"
                    self.counts["wiggle_none"] += 1
                    for n in (a, b):
                        self.T[n].unsure = None
                    self.log("tracking", dict(t=round(t, 2), robot="%s+%s" % (a, b), what="names kept as predicted",
                                              why="no wiggle possible and their motion did not tell them apart"))
                    continue
                st["state"] = "wiggle"
                w = min((a, b), key=lambda n: self.names.index(n))
                self._ask_wiggle(w, st, t)

    def _ask_wiggle(self, w, st, t):
        tr = self.T[w]
        self.wiggles[w] = dict(t0=t, pair=st, at=self.predict(w, t), other=st["b"] if w == st["a"] else st["a"],
                               seen=[], tries=st.get("tries", 0) + 1)
        st["tries"] = self.wiggles[w]["tries"]
        self.counts["wiggles"] += 1
        self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle asked",
                                  why="who is who after the close pass with %s is not clear from their motion"
                                  % self.wiggles[w]["other"]))
        self.wiggle_fn(w, "identity after a close pass")
        tr.unsure = "identity wiggle"

    def wiggle_started(self, name, t):
        w = self.wiggles.get(name)
        if w is not None:
            w["t0"] = t
            w["at"] = self.predict(name, t)

    def _swap_check(self, t, st):
        a, b = st["a"], st["b"]
        t0 = max(st["t0"], t - SWAP_WIN_S - 1.0)
        res = {}
        for n in (a, b):
            tr = self.T[n]
            cam = [c for c in tr.cam if c[0] >= st["parted_t"] - 0.3]
            o0, o1 = self._own_at(tr, t0), self._own_at(tr, t)
            c0 = st["cam0"].get(n)
            if not cam or o0 is None or o1 is None or c0 is None:
                return "unknown", "no motion to compare"
            res[n] = dict(cam=(cam[-1][1] - c0[1], cam[-1][2] - c0[2]), own=(o1[1] - st["own0"][n][1],
                                                                             o1[2] - st["own0"][n][2]))
        ca, cb = res[a]["cam"], res[b]["cam"]
        oa, ob = res[a]["own"], res[b]["own"]
        keep = math.dist(ca, oa) + math.dist(cb, ob)
        # swapped: the track called a is b (it started where a was: its displacement from a's start spot equals b's own
        # displacement plus where b started relative to a)
        sa, sb = st["cam0"][a], st["cam0"][b]
        ca_abs = (sa[1] + ca[0], sa[2] + ca[1])
        cb_abs = (sb[1] + cb[0], sb[2] + cb[1])
        swap = math.dist((ca_abs[0] - sb[1], ca_abs[1] - sb[2]), ob) + math.dist((cb_abs[0] - sa[1], cb_abs[1] - sa[2]),
                                                                                  oa)
        moved = max(math.hypot(*oa), math.hypot(*ob))
        ev = "keep %.1f cm, swap %.1f cm" % (keep, swap)
        if moved < MIN_MOTION_CM:
            return "unknown", ev + ", too little motion"
        if keep + SWAP_MARGIN_CM < swap:
            return "kept", ev
        if swap + SWAP_MARGIN_CM < keep:
            return "swapped", ev
        return "unknown", ev

    def _swap(self, a, b, t, why):
        A, B = self.T[a], self.T[b]
        A.sure, B.sure = B.sure, A.sure
        A.cam, B.cam = B.cam, A.cam
        A.unsure = B.unsure = None
        A.lock_seeded = B.lock_seeded = None
        for n in (a, b):            # the locks follow the wrong robots: seed them again on their own
            tr = self.T[n]
            if tr.sure is not None:
                self.seed(n, t, tr.sure[1], tr.sure[2], "swap")
        self.log("tracking", dict(t=round(t, 2), robot="%s+%s" % (a, b), what="names swapped back", why=why))

    def _wiggle_watch(self, t, dets):
        for w, ws in list(self.wiggles.items()):
            st = ws["pair"]
            if t - ws["t0"] < 0.2:
                continue
            pw0, po0 = self.predict(w, t), self.predict(ws["other"], t)
            spins = [d for d in dets if d.src in ("motion", "lock", "ml") and d.spin and any(
                p is not None and math.hypot(d.x - p[0], d.y - p[1]) < WIGGLE_NEAR_CM for p in (pw0, po0))]
            ws["seen"] += [(d.x, d.y) for d in spins]
            if t - ws["t0"] < WIGGLE_S:
                continue
            other = ws["other"]
            pw, po = self.predict(w, t), self.predict(other, t)
            del self.wiggles[w]
            if not ws["seen"] or pw is None or po is None:
                self.counts["wiggle_none"] += 1
                if ws["tries"] < 3:
                    st["state"] = "close"
                    st["parted_t"] = t - 0.6 + WIGGLE_RETRY_S     # asked again later
                    self.T[w].unsure = "identity wiggle"
                    continue
                st["state"] = "done"
                for n in (w, other):
                    self.T[n].unsure = None
                self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle showed nothing",
                                          why="3 tries: names kept as predicted"))
                continue
            mx = sum(p[0] for p in ws["seen"]) / len(ws["seen"])
            my = sum(p[1] for p in ws["seen"]) / len(ws["seen"])
            near_w = math.hypot(mx - pw[0], my - pw[1])
            near_o = math.hypot(mx - po[0], my - po[1])
            st["state"] = "done"
            if not (near_w < WIGGLE_CLEAR * near_o or near_o < WIGGLE_CLEAR * near_w):
                # the turning outline is about as near both: no verdict, names kept as predicted (said)
                self.counts["wiggle_none"] += 1
                for n in (w, other):
                    self.T[n].unsure = None
                self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle unclear: names kept",
                                          why="the turning outline was %.1f / %.1f cm from the two" % (near_w, near_o)))
                continue
            if near_w <= near_o:
                self.counts["wiggle_kept"] += 1
                for n in (w, other):
                    self.T[n].unsure = None
                self.log("tracking", dict(t=round(t, 2), robot=w, what="wiggle: names kept",
                                          why="the turning outline was where %s was thought to be" % w))
            else:
                self.counts["wiggle_swapped"] += 1
                self._swap(w, other, t, "the wiggle: the turning outline was where %s was thought to be" % other)

    # -- parked robots and new objects -------------------------------------------------------------------------------
    def _parked(self, n, t, d):
        """A parked robot (its wheels standing PARKED_STILL_S) whose camera spot, averaged (the mean of its first
        PARKED_REF_N sightings standing against the mean of its last PARKED_NOW_N: single sightings are 1-2 cm noisy),
        moved PARKED_MOVE_CM to PARKED_MAX_CM: moved while parked."""
        if not self.parked_watch:
            return
        tr = self.T[n]
        o = tr.own[-1] if tr.own else None
        if o is None or not o[4]:
            tr.parked_at = None
            return
        k = len(tr.own) - 1
        while k > 0 and tr.own[k - 1][4]:
            k -= 1
        if t - tr.own[k][0] < PARKED_STILL_S:
            return
        if tr.parked_at is None:
            tr.parked_at = dict(ref=[], now=[])
        pa = tr.parked_at
        if len(pa["ref"]) < PARKED_REF_N:
            pa["ref"].append((d.x, d.y))
            return
        pa["now"].append((d.x, d.y))
        del pa["now"][:-PARKED_NOW_N]
        if len(pa["now"]) < PARKED_NOW_N:
            return
        rx = sum(p[0] for p in pa["ref"]) / len(pa["ref"])
        ry = sum(p[1] for p in pa["ref"]) / len(pa["ref"])
        nx = sum(p[0] for p in pa["now"]) / len(pa["now"])
        ny = sum(p[1] for p in pa["now"]) / len(pa["now"])
        mv = math.hypot(nx - rx, ny - ry)
        if mv > PARKED_MAX_CM or not self.proven or t - getattr(tr, "parked_ev_t", -1e9) < PARKED_EVERY_S or \
                self._close_to_other(n, nx, ny):
            return
        if mv >= PARKED_MOVE_CM:
            tr.parked_ev_t = t
            self.counts["parked_moved"] += 1
            self.log("tracking", dict(t=round(t, 2), robot=n, what="moved while parked", d_cm=round(mv, 1),
                                      x=round(nx, 1), y=round(ny, 1), src=d.src))
            tr.parked_at = dict(ref=list(pa["now"]), now=[])
            self.parked_events = getattr(self, "parked_events", [])
            self.parked_events.append(dict(t=t, robot=n, x=nx, y=ny, d_cm=mv))

    def _bricks(self, t, dets, preds):
        keep = []
        for u in self.unclaimed:
            if t - u["t"] < 1.0:
                keep.append(u)
        self.unclaimed = keep
        for d in dets:
            if any(p is not None and math.hypot(d.x - p[0], d.y - p[1]) < 2 * GATE_CM for p in preds.values()):
                continue
            if any(math.hypot(d.x - o["x"], d.y - o["y"]) < BRICK_R_CM for o in self.obstacles):
                continue
            u = next((u for u in self.unclaimed if math.hypot(d.x - u["x"], d.y - u["y"]) < BRICK_STILL_CM), None)
            if u is None:
                self.unclaimed.append(dict(t0=t, t=t, x=d.x, y=d.y, n=1))
                continue
            u["t"], u["n"] = t, u["n"] + 1
            if t - u["t0"] >= BRICK_S and u["n"] >= 5:
                ob = dict(x=round(u["x"], 1), y=round(u["y"], 1), r=BRICK_R_CM, t=round(t, 2), src="camera: new object")
                self.obstacles.append(ob)
                self.counts["bricks"] += 1
                self.unclaimed.remove(u)
                self.log("tracking", dict(t=round(t, 2), robot="-", what="new object", x=ob["x"], y=ob["y"]))
                self.new_obstacles = getattr(self, "new_obstacles", [])
                self.new_obstacles.append(ob)

    def take_obstacles(self):
        o = getattr(self, "new_obstacles", [])
        self.new_obstacles = []
        return o

    def take_parked(self):
        o = getattr(self, "parked_events", [])
        self.parked_events = []
        return o

    def summary(self):
        return dict(version=VERSION, sources=list(self.sources), identity=self.identity,
                    counts=dict(self.counts), by_source=dict(self.by_src),
                    per_robot={n: dict(tr.src_n) for n, tr in self.T.items()})
