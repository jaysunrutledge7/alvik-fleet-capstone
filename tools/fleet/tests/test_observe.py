"""observe.py: the shared 'observe, then trust' checks (5 Nov 2026)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import observe as OB  # noqa: E402


def test_heading_offset_turned_tags():
    for true_off in (0, 90, 180, -90):
        nose = 30.0
        tag = OB.wrap180(nose - true_off)          # the tag reads this when the nose points at 30 deg
        p0 = (100.0, 50.0, tag)
        p1 = (100 + 6 * math.cos(math.radians(nose)), 50 + 6 * math.sin(math.radians(nose)), tag)
        off, raw, mv = OB.heading_offset(p0, p1)
        assert abs(OB.wrap180(off - true_off)) < 1e-6, (true_off, off)
    assert OB.heading_offset((0, 0, 0), (1, 0, 0)) is None          # moved too little: nothing concluded
    off, raw, mv = OB.heading_offset((0, 0, 0), (-6, 0, 0), sign=-1)   # backing up along the tag's heading
    assert abs(off) < 1e-6


def test_snap_keeps_odd_offsets():
    assert OB.snap(176) == 180 or OB.snap(176) == -180
    assert OB.snap(45) == 45


def test_nudge_clear():
    edge = lambda a, b: ["edge"] if b[0] < 3 else []          # noqa: E731
    ok, base = OB.nudge_clear((20, 50, 0), edge)
    assert ok and base == 0
    ok, base = OB.nudge_clear((6, 50, 0), edge)                 # facing the edge: sideways instead
    assert ok and base == 90
    ok, base = OB.nudge_clear((6, 50, 45), edge)
    assert not ok


def _drive(w, start, nose_deg, tag_off, n=30, step=1.0, sign=1):
    x, y = start
    out = None
    evs = []
    for k in range(n):
        x += sign * step * math.cos(math.radians(nose_deg))
        y += sign * step * math.sin(math.radians(nose_deg))
        ev = w.feed(k, (x, y, OB.wrap180(nose_deg - tag_off)), own_heading=0.0, sign=sign)
        if ev:
            evs.append(ev)
    return evs


def test_heading_watch_relearns_a_turned_tag():
    w = OB.HeadingWatch(0.0)
    evs = _drive(w, (50, 50), 0.0, 180.0)
    kinds = [e["kind"] for e in evs]
    assert "heading_relearned" in kinds
    assert abs(OB.wrap180(w.offset - 180)) < 1e-6
    evs = _drive(w, (80, 50), 0.0, 180.0)
    assert all(e["kind"] == "heading_ok" for e in evs)


def test_heading_watch_one_bad_stretch_is_not_enough():
    w = OB.HeadingWatch(0.0)
    _drive(w, (50, 50), 0.0, 0.0, n=10)
    evs = _drive(w, (60, 50), 0.0, 180.0, n=9)        # one stretch only
    assert [e["kind"] for e in evs] == ["heading_bad"]
    assert w.offset == 0.0


def test_readmap_seed_and_evidence():
    rm = OB.ReadMap(396, 168)
    rm.seed(lambda x, y: 0.9 if x < 150 else 0.1, "camera model")
    assert rm.readable(40, 80)[0] and not rm.readable(300, 80)[0]
    for _ in range(20):
        rm.add(100, 80, False)                         # in room light the tag does not read at x 100
    ok, why = rm.readable(100, 80)
    assert not ok and "20 tries" in why
    for _ in range(20):
        rm.add(300, 80, True)                          # a second camera: reads there now
    assert rm.readable(300, 80)[0]
    rm2 = OB.ReadMap.from_json(rm.to_json())
    assert rm2.rate(100, 80) == rm.rate(100, 80)
    assert "#" in rm.text() and "." in rm.text()


def test_split_points_any_number_of_robots():
    pts = {"P%02d" % k: (10.0 * k, 50.0 + (k % 3) * 20) for k in range(1, 21)}
    ok = lambda x, y: (x <= 150, "x %.0f" % x)        # noqa: E731
    for n in (1, 2, 3):
        robots = {"R%d" % k: (40.0 * k, 50.0) for k in range(n)}
        orders, skipped, words = OB.split_points(pts, robots, ok)
        got = sorted(l for v in orders.values() for l in v)
        assert got == sorted(l for l in pts if pts[l][0] <= 150)
        assert set(skipped) == {l for l in pts if pts[l][0] > 150}
        assert max(len(v) for v in orders.values()) - min(len(v) for v in orders.values()) <= 1
    # each robot gets the run nearest it
    orders, _, _ = OB.split_points(pts, {"far": (150.0, 50.0), "near": (10.0, 50.0)}, ok)
    assert "P01" in orders["near"] and "P15" in orders["far"]


def test_split_points_halves_switch():
    pts = {"A": (10, 10), "B": (300, 10), "C": (20, 10)}
    orders, skipped, words = OB.split_points(pts, {"Alpha": (0, 0)}, None, mode="halves",
                                             halves={"Alpha": ["A", "C"], "Bravo": ["B"]})
    assert orders == {"Alpha": ["A", "C"]} and "B" in skipped and "halves" in words[0]


def test_pick_cameras():
    m, w = OB.pick_cameras([("/dev/video0", "USB 2.0 Camera")], {"overhead": "C920e", "side": ""})
    assert m == {"overhead": "/dev/video0"} and "observed" in w[0]
    m, w = OB.pick_cameras([("/dev/video0", "EMEET C950"), ("/dev/video2", "Logi C920e")],
                           {"overhead": "C920e", "side": "EMEET"})
    assert m == {"overhead": "/dev/video2", "side": "/dev/video0"}
    m, w = OB.pick_cameras([("/dev/video0", "A"), ("/dev/video2", "B")], {"overhead": "C920e", "side": "EMEET"})
    assert set(m.values()) == {"/dev/video0", "/dev/video2"} and len(w) == 2
    assert OB.pick_cameras([], {})[0] == {}


def test_edge_trip_kind():
    assert OB.edge_trip_kind(True, True, 50) == "edge"
    assert OB.edge_trip_kind(True, False, 60) == "surface_mark"         # the tape over the seam
    assert OB.edge_trip_kind(True, False, 5) == "edge"                   # the camera puts it at the edge
    assert OB.edge_trip_kind(True, False, None) == "unknown"             # not seen: cautious
    assert OB.edge_trip_kind(True, None, 60) == "surface_mark"


def test_restarted():
    assert OB.restarted({"boot": 1}, {"boot": 2})
    assert OB.restarted({"up_ms": 600000}, {"up_ms": 5000})
    assert not OB.restarted({"boot": 1, "up_ms": 100}, {"boot": 1, "up_ms": 2000})


class _D:
    def __init__(self, i, c):
        self.tag_id, self.corners, self.center = i, np.array(c, float), np.array(c, float).mean(0)


def test_detect_retry_downscaled():
    big = np.zeros((960, 1280), np.uint8)

    def detect(g):
        return [_D(1, [[10, 10], [20, 10], [20, 20], [10, 20]])] + ([_D(2, [[50, 50], [55, 50], [55, 55], [50, 55]])]
                                                                     if g.shape[1] < 1000 else [])
    dets, used = OB.detect_retry(detect, big, [1, 2], resize=lambda g, s: np.zeros((int(g.shape[0] * s),
                                                                                    int(g.shape[1] * s)), np.uint8))
    assert used == {2: 0.5}
    d2 = [d for d in dets if d.tag_id == 2][0]
    assert d2.corners[0].tolist() == [100, 100]
    dets, used = OB.detect_retry(detect, big, [1])
    assert used == {}


def test_pose_agree():
    w = OB.PoseAgree()
    assert w.feed(0, (0, 0), (10, 0), 0.1) is None
    assert w.feed(1, (0, 0), (10, 0), 0.1) is None
    ev = w.feed(2.1, (0, 0), (10, 0), 0.1)
    assert ev and ev["kind"] == "reseed"
    assert w.feed(3, (0, 0), (10, 0), 2.0) is None        # a stale camera says nothing


def test_tilt_watch_standing_tilt():
    w = OB.TiltWatch()
    t = 0.0
    for _ in range(OB.LEVEL_N):
        w.feed(t, 1.0, -0.5, 0.0)
        t += 0.02
    assert w.level == (1.0, -0.5)
    evs = []
    for _ in range(200):                      # Center: 3.4 deg, standing, 4 s
        e = w.feed(t, 4.4, -0.5, 0.0)
        t += 0.02
        if e:
            evs.append(e)
    assert len(evs) == 1 and evs[0]["dev_deg"] > 3
    w2 = OB.TiltWatch()
    for _ in range(OB.LEVEL_N):
        w2.feed(0, 0, 0, 0)
    assert not any(w2.feed(k * 0.02, 4.0, 0, 10.0) for k in range(300))    # driving: spikes are not a hazard


def test_done_too_fast():
    assert OB.done_too_fast(4.0, 0.05, 0.1, 14.0)          # Left's 4,289 reverses
    assert not OB.done_too_fast(4.0, 3.5, 0.1, 14.0)
    assert not OB.done_too_fast(4.0, 0.2, 12.0, 14.0)       # the camera saw it move: fine
    assert not OB.done_too_fast(1.0, 0.05, None, 1.0)


def test_reverse_room():
    rect = (0, 0, 122, 122)
    ok, w = OB.reverse_room((30, 61, 0), 14, rect)
    assert ok
    ok, w = OB.reverse_room((20, 61, 0), 14, rect)           # its back would reach the x = 0 edge
    assert not ok
    ok, w = OB.reverse_room((30, 61, 0), 14, rect, cam_age_s=5.0)
    assert not ok and "blind" in w
    ok, w = OB.reverse_room((60, 61, 0), 14, rect, others=[(40, 61)])
    assert not ok
