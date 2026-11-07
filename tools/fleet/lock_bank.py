#!/usr/bin/env python3
"""The lock-on tracker for every robot at once, on the Pi's bridges (e09, Phase B1). The lock-on (tools/experiment/
e02_lock.py, 22-23 Oct: seen in 94 / 93 / 96 % of the evening losses' pictures against 41 / 40 / 72 % for the motion
detector alone) lived only in the old E02 runner (e02_camera.MotionTracker._lock_step); free roam's camera bridge and
the school's tag bridge never ran it (7 Nov audit, section 0).

Here: one LockTracker per robot, SEEDED by a sure, named sighting of that robot standing still (a roof tag read at
school; the motion detector's sure outline at home), with nobody else within SEED_CLEAR_CM (a seed on a look-alike would
follow the wrong robot). Each picture, a robot the other sources do not see is looked for by its lock (near where its
own pose says it is); its place goes out as a fix with src "<bridge>:lock" and the robot's name. The tracking layer
(tracking_layer.py) checks a lock's name like any sighting's whenever two robots are close.

Works with either camera model (e02_camera's, recognition.cameras.CameraModel): both have project, to_floor and
px_per_cm. Switch: the bridges' --no-lock (the tracking before e09).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "experiment"), TOOLS):
    if p not in sys.path:
        sys.path.append(p)

ROBOT_CM = 9.5              # the body's side (e02_camera.ROBOT_CM)
SEED_CLEAR_CM = 16.0        # tracking_layer.CLOSE_CM: no other robot this near when a lock is seeded
RESEED_S = 3.0              # CHOSEN: a lock that is locked is re-seeded (its look taken again) at most this often
LOCK_SIGMA_CM = 1.2         # tracking_layer.SOURCE_SIGMA["lock"]


class LockBank:
    def __init__(self, cam, z_cm, floor=None, enabled=True):
        self.cam = cam
        self.z = float(z_cm)
        self.floor = floor
        self.enabled = enabled
        self.locks = {}             # name -> LockTracker
        self.seeded = {}            # name -> t
        self.counts = dict(seeds=0, refused_seeds=0, steps=0, found=0, lost=0)

    def _rpx(self, x, y):
        try:
            return ROBOT_CM * float(self.cam.px_per_cm(x, y, self.z))
        except Exception:           # noqa: BLE001
            return None

    def set_floor(self, floor):
        self.floor = floor
        for lk in self.locks.values():
            try:
                lk.set_floor(floor)
            except Exception:       # noqa: BLE001
                pass

    def seed(self, name, img, t, x, y, others=()):
        """Lock on to `name` at (x, y) in this picture (it should stand still in it). others: [(x, y)] of the other
        robots: no seed with one of them within SEED_CLEAR_CM."""
        if not self.enabled or img is None:
            return False
        if any(math.hypot(x - ox, y - oy) < SEED_CLEAR_CM for ox, oy in others):
            self.counts["refused_seeds"] += 1
            return False
        lk0 = self.locks.get(name)
        if lk0 is not None and lk0.locked and t - self.seeded.get(name, -1e9) < RESEED_S:
            return False
        import e02_lock as LK
        rpx = self._rpx(x, y)
        if not rpx or rpx < 4:
            return False
        lk = LK.LockTracker(self.cam, rpx, self.z)
        if self.floor is not None:
            try:
                lk.set_floor(self.floor)
            except Exception:       # noqa: BLE001
                pass
        try:
            ok = lk.start(img, x, y, t)
        except Exception:           # noqa: BLE001  (a picture edge, a bad model: no lock, nothing else changes)
            ok = False
        if not ok:
            return False
        self.locks[name] = lk
        self.seeded[name] = t
        self.counts["seeds"] += 1
        return True

    def step(self, name, img, t, predict=None, still=False):
        """(x, y, score) where the lock finds `name` in this picture, or None."""
        lk = self.locks.get(name)
        if lk is None or not lk.locked or img is None:
            return None
        self.counts["steps"] += 1
        try:
            s = lk.step(img, t, predict=predict, still=still)
        except Exception:           # noqa: BLE001
            s = None
        if not lk.locked:
            self.counts["lost"] += 1
            self.locks.pop(name, None)
            return None
        if s is None:
            return None
        self.counts["found"] += 1
        return (s[1], s[2], s[3])

    def nudge(self, name, x, y, t):
        lk = self.locks.get(name)
        if lk is not None:
            lk.nudge(x, y, t)

    def drop(self, name):
        self.locks.pop(name, None)

    def summary(self):
        return dict(enabled=self.enabled, locked=sorted(n for n, lk in self.locks.items() if lk.locked),
                    **self.counts)
