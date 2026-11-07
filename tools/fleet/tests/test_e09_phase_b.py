"""e09 (Phase B, 7 Nov): the one tracking layer, the filter in the loop, heading from the raw gyro, pictures."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (FLEET, os.path.dirname(FLEET)):
    if p not in sys.path:
        sys.path.insert(0, p)

import fused_pose as FP          # noqa: E402
import tracking_layer as TL      # noqa: E402
import freeroam_core as FC       # noqa: E402


# ------------------------------------------------------------------------------------------------- the filter (B2)
def drive(fp, t0, t1, v, w=0.0, gz_bias=0.5, dt=0.05, gz_scale=1.0):
    t = t0
    while t < t1 - 1e-9:
        t += dt
        still = abs(v) < 1e-6 and abs(w) < 1e-6
        fp.raw(t, v, w, gz_bias + w * gz_scale, still)
    return t


def test_filter_dead_reckons_and_takes_the_bias_at_a_standstill():
    fp = FP.FusedPose("A")
    fp.start(0.0, 10.0, 10.0, 0.0)
    t = drive(fp, 0.0, 2.0, 0.0)                    # standing: only the bias
    t = drive(fp, t, 3.0, 0.0, 40.0)                # a turn (the gyro's sign is learned from the wheels)
    t = drive(fp, t, 3.6, 0.0)                      # standing again: bias taken
    assert fp.gsign == 1.0
    assert abs(fp.gh.bias - 0.5) < 0.05
    x0, y0, _ = fp.pose(t)
    t = drive(fp, t, t + 5.0, 10.0)                 # 50 cm straight
    x1, y1, _ = fp.pose(t)
    assert abs(math.hypot(x1 - x0, y1 - y0) - 50.0) < 5.0


def test_a_late_fix_is_moved_on_by_the_motion_and_a_disagreement_widens_at_once():
    fp = FP.FusedPose("A")
    fp.start(0.0, 0.0, 0.0, 0.0, sxy=0.5)
    t = 0.0
    for k in range(20):                             # 2 s at 10 cm/s, the camera with it (at about x = 20)
        t = drive(fp, t, t + 0.1, 10.0)
        fp.fix(t, 10.0 * t, 0.0, None, 1.0, "motion", now=t)
    u0 = fp.unc_cm(t)
    # the camera saw it 0.5 s ago at x = 15: agrees once moved on by 5 cm of motion
    r = fp.fix(t - 0.5, 15.0, 0.0, None, 1.0, "motion", now=t)
    assert r["used"] and r["disagree_cm"] < 1.5
    # now a camera spot 6 cm off: used, and the uncertainty widened at once (Phase A's A1)
    r = fp.fix(t, fp.pose(t)[0], 6.0, None, 1.0, "motion", now=t)
    assert r["d2"] > FP.GATE_D2
    assert fp.unc_cm(t) > u0 + 1.0


def test_an_unproven_camera_does_not_pull_the_filter_and_a_jump_is_not_a_correction():
    fp = FP.FusedPose("A")
    fp.start(0.0, 50.0, 50.0, 0.0, sxy=0.5)
    t = drive(fp, 0.0, 1.0, 0.0)
    r = fp.fix(t, 60.0, 50.0, None, 1.0, "motion", now=t, hold_disagree=True)
    assert r["used"] is False and abs(fp.pose(t)[0] - 50.0) < 0.5
    r = fp.fix(t, 150.0, 50.0, None, 1.0, "motion", now=t)
    assert r.get("jump") and abs(fp.pose(t)[0] - 50.0) < 0.5


def test_the_pose_is_not_carried_on_when_the_raw_rows_stop():
    fp = FP.FusedPose("A")
    fp.start(0.0, 0.0, 0.0, 0.0)
    t = drive(fp, 0.0, 1.0, 10.0)
    x_now = fp.pose(t)[0]
    assert fp.pose(t + 10.0)[0] < x_now + 10.0 * FP.FusedPose.EXTRAP_S + 1.0     # not 100 cm on
    assert fp.unc_cm(t + 10.0) > fp.unc_cm(t)


def test_learned_trust_follows_a_noisier_source():
    rng = np.random.default_rng(1)
    fp = FP.FusedPose("A")
    fp.start(0.0, 0.0, 0.0, 0.0)
    t = 0.0
    for _ in range(300):
        t = drive(fp, t, t + 0.1, 0.0)
        fp.fix(t, rng.normal(0, 3.0), rng.normal(0, 3.0), None, None, "lights", now=t)
    assert fp.trust.get("lights", 1.0) > 2.0        # 3 cm noise against 1.5 cm assumed: trusted less


def test_gyro_heading_switch_is_off_by_default_and_integrates_with_the_standstill_bias():
    assert FP.GYRO_HEADING is False or os.environ.get("MIRLT_GYRO_HEADING") == "1"
    g = FP.GyroHeading()
    g.set(0.0)
    t = 0.0
    for _ in range(20):          # standing 1 s: bias 0.6
        t += 0.05
        g.feed(t, 0.6, True)
    for _ in range(40):          # turning 2 s at 45 deg/s (the wheels agree: sign +)
        t += 0.05
        g.feed(t, 0.6 + 45.0, False, 45.0)
    assert g.sign == 1.0 and abs(g.bias - 0.6) < 1e-6
    assert g.heading_deg() is not None and 60.0 < g.heading_deg() < 95.0


# ------------------------------------------------------------------------------------------- the tracking layer (B1)
def layer(**kw):
    log = []
    L = TL.TrackingLayer(["A", "B"], ("tags", "motion", "lock", "lights"), log=lambda k, r: log.append(r), **kw)
    L._log = log
    return L


def own_both(L, t, a, b, still=False):
    L.own("A", t, a[0], a[1], 0.0, still)
    L.own("B", t, b[0], b[1], 0.0, still)


def test_tags_name_robots_and_unnamed_sightings_go_to_the_nearest_prediction():
    L = layer()
    own_both(L, 0.0, (20, 20), (80, 80))
    f = L.feed(0.0, [TL.Det("tags", 0.0, 20, 20, 0.0, "A"), TL.Det("tags", 0.0, 80, 80, 0.0, "B")])
    assert {x["robot"] for x in f} == {"A", "B"}
    own_both(L, 0.1, (21, 20), (79, 80))
    f = L.feed(0.1, [TL.Det("motion", 0.1, 79.2, 80.1), TL.Det("motion", 0.1, 21.1, 20.0)])
    got = {x["robot"]: (round(x["x"]), round(x["y"])) for x in f}
    assert got == {"A": (21, 20), "B": (79, 80)}


def test_a_contested_or_merged_outline_names_nobody():
    L = layer(identity="predict")
    own_both(L, 0.0, (40, 40), (52, 40))
    L.seed("A", 0.0, 40, 40)
    L.seed("B", 0.0, 52, 40)
    f = L.feed(0.1, [TL.Det("motion", 0.1, 46.0, 40.0)])           # half way: contested
    assert f == []
    f = L.feed(0.2, [TL.Det("motion", 0.2, 45.0, 40.0, area=2.0)])  # one big outline: merged
    assert f == [] and L.counts["merged"] >= 1


def test_one_robots_lights_are_not_handed_to_another_whose_gate_grew():
    for cluster, want in ((True, ["B"]), (False, ["A", "B"])):
        L = layer(cluster=cluster)
        L.own("A", 0.0, 35.0, 40.0, 0.0, False)
        L.own("B", 0.0, 63.0, 40.0, 0.0, False)
        L.seed("A", 0.0, 35.0, 40.0)
        L.seed("B", 0.0, 63.0, 40.0)
        # A unseen for 2 s while its wheels drove 70 cm (out and back): its gate is wide
        for k in range(1, 21):
            t = k * 0.1
            L.own("A", t, 35.0 + (3.5 * k if k <= 10 else 35.0 - 3.5 * (k - 10)) + (10.0 if k == 20 else 0.0), 40.0,
                  0.0, False)
            L.own("B", t, 63.0, 40.0, 0.0, False)
        # B seen by its outline AND its lights 2 cm apart
        f = L.feed(2.0, [TL.Det("motion", 2.0, 63.0, 40.0), TL.Det("lights", 2.0, 61.0, 40.0)])
        assert sorted(x["robot"] for x in f) == want, (cluster, f)


def test_a_close_pass_is_checked_by_motion_and_swapped_names_are_put_back():
    L = layer()
    # after a close pass the CAMERA's track called A moved as B's wheels did, and the other way round
    L.seed("A", 0.0, 50.0, 50.0)
    L.seed("B", 0.0, 60.0, 50.0)
    for n, x in (("A", 50.0), ("B", 60.0)):
        L.own(n, 0.0, x, 50.0, 0.0, False)
    L._pairs(0.0, {n: L.predict(n, 0.0) for n in "AB"})
    st = L.pairs[frozenset("AB")]
    assert st["state"] == "close"
    for n, dx in (("A", -20.0), ("B", +20.0)):         # A's wheels: back 20 cm; B's: on 20 cm
        L.own(n, 2.0, (50.0 if n == "A" else 60.0) + dx, 50.0, 0.0, False)
    L.T["A"].cam = [(2.0, 80.0, 50.0)]                 # the track called A is where B went
    L.T["B"].cam = [(2.0, 30.0, 50.0)]
    L.T["A"].sure, L.T["B"].sure = (2.0, 80.0, 50.0), (2.0, 30.0, 50.0)
    st["parted_t"] = 1.0
    L._parted(2.0)
    assert L.counts["swapped"] == 1
    assert L.T["A"].sure[1] == 30.0 and L.T["B"].sure[1] == 80.0
    assert any(r.get("what") == "names swapped back" for r in L._log)


def test_wiggle_first_when_motion_cannot_tell_them_apart():
    asked = []
    L = layer(wiggle=lambda n, why: asked.append(n))
    L.seed("A", 0.0, 50, 50)
    L.seed("B", 0.0, 64, 50)
    t = 0.0
    for k in range(60):                  # both stand; the camera finds B's outline 9 cm further on (no wheel motion)
        t = k * 0.1
        own_both(L, t, (50, 50), (64, 50), still=True)
        L.feed(t, [TL.Det("motion", t, 50, 50), TL.Det("motion", t, 73, 50)])
        if asked:
            break
    assert asked == ["A"]
    assert L.T["A"].unsure is not None and L.T["B"].unsure is not None
    t0 = t
    for k in range(1, 40):               # A wiggles: its outline turns where A is thought to be
        t = t0 + k * 0.1
        own_both(L, t, (50, 50), (64, 50), still=True)
        L.feed(t, [TL.Det("motion", t, 50.2, 50.1, spin=True)])
    assert L.counts["wiggle_kept"] == 1
    assert L.T["A"].unsure is None and L.T["B"].unsure is None


def test_a_robot_pushed_while_parked_and_a_new_object_are_noticed():
    L = layer(camera_watch=False)
    L.seed("A", 0.0, 40, 40)
    L.seed("B", 0.0, 90, 90)
    for k in range(80):
        t = k * 0.1
        own_both(L, t, (40, 40), (90, 90), still=True)
        ax = 40.0 if k < 40 else 44.0                       # pushed 4 cm at 4 s, its wheels never turned
        L.feed(t, [TL.Det("lock", t, ax, 40.0, name="A"), TL.Det("lock", t, 90, 90, name="B"),
                   TL.Det("motion", t, 70.0, 20.0)])        # something new standing at (70, 20)
    ev = L.take_parked()
    assert ev and ev[0]["robot"] == "A" and ev[0]["d_cm"] >= 3.0
    ob = L.take_obstacles()
    assert ob and abs(ob[0]["x"] - 70.0) < 1.0


def test_the_camera_watch_distrusts_a_mirrored_picture():
    L = TL.TrackingLayer(["A"], ("motion",), identity="predict")
    t, x, y, h = 0.0, 60.0, 30.0, 90.0
    L.seed("A", t, x, y)
    for k in range(400):                 # it drives circles (turning left); the camera mirrors x: turns right
        t = k * 0.1
        h += 6.0
        x += math.cos(math.radians(h))
        y += math.sin(math.radians(h))
        L.own("A", t, x, y, h, False)
        L.sure_hack = None
        L.T["A"].sure = (t, 122.0 - x, y) if k else L.T["A"].sure
        L.feed(t, [TL.Det("motion", t, 122.0 - x, y)])
    assert L.counts.get("camera_distrusted", 0) >= 1 and not L.proven


def test_sources_by_site_and_the_ml_slot_is_off():
    assert TL.sources_for("tags") == ("tags", "motion", "lock")
    assert TL.sources_for("lights") == ("motion", "lock", "lights")
    assert TL.sources_for("lights", old=True) == ("motion", "lights")
    assert "ml" not in TL.sources_for("lights")
    assert "ml" in TL.sources_for("lights", ml=True)


# ------------------------------------------------------------------------------------------------- the controller
def ctrl(**kw):
    out, rows = [], []
    c = FC.Controller(["A"], (0, 0, 122, 122), lambda n, t: out.append(t), lambda tb, r: rows.append((tb, r)), **kw)
    return c, out, rows


def test_the_controller_plans_from_the_filter_while_raw_flows_and_falls_back_without():
    assert ctrl()[0].pose_source == FP.POSE_SOURCE
    c, out, rows = ctrl(pose_source="ekf")
    assert c.gyro_heading is False
    c.on_follow("A", 0.0, "idle", 60.0, 60.0, 0.0)
    c.on_fix("A", 0.0, 60.0, 60.0, 0.0, arrival=0.0)
    r = c.R["A"]
    t = 0.0
    for _ in range(20):
        t += 0.05
        c.t = t
        c.on_raw_full("A", t, 0.0, 0.0, 0.5, True)
        c.on_fix("A", t, 60.0, 60.0, None, arrival=t)
    assert c.ekf_live(r)
    assert abs(c.pos(r)[0] - 60.0) < 0.5
    c.t = t + 2.0            # RAW silent: the one-step blend's position again
    r.pc = None
    assert not c.ekf_live(r)


def test_blend_switch_keeps_the_old_pose():
    c, out, rows = ctrl(pose_source="blend")
    c.on_follow("A", 0.0, "idle", 60.0, 60.0, 0.0)
    c.on_raw_full("A", 0.0, 0.0, 0.0, 0.5, True)
    assert c.R["A"].fp is not None and not c.ekf_live(c.R["A"])     # running beside, not in charge


def test_unsure_near_an_edge_alone_asks_for_a_picture_and_camera_objects_are_kept_out():
    c, out, rows = ctrl()
    r = c.R["A"]
    c.want_picture(r, "unsure near an edge (test)")
    p = c.take_pictures()
    assert p and p[0]["robot"] == "A" and c.take_pictures() == []
    c.on_obstacle(dict(x=50.0, y=50.0, r=6.0))
    assert c.keepouts[-1]["confirmed"] and c.keepouts[-1]["src"].startswith("camera")


def test_the_identity_wiggle_turns_the_robot_on_its_spot():
    c, out, rows = ctrl()
    c.on_fw("A", "2026-11-05d")
    c.on_follow("A", 0.0, "idle", 60.0, 60.0, 0.0)
    c.on_fix("A", 0.0, 60.0, 60.0, 0.0, arrival=0.0)
    c.on_heard("A", 0.0)
    c.begin("A")
    c.R["A"].mode = "idle"
    c.t = 1.0
    c.identity_wiggle(c.R["A"], "test")
    assert c.counts["identity_wiggles"] == 1
    assert c.R["A"].move is not None and c.R["A"].move["kind"] == "wiggle"


# ------------------------------------------------------------------------------------------------- the simulator
def test_fastsim_detections_mode_tracks_through_the_layer_without_contact():
    import freeroam_fastsim as S
    s = S.Sim(3, 0.03, dead_robot=False, pi_freeze=False, sensors=True, camera="detections", dropouts=False,
              scene=dict(tracking="lights"))
    res = s.run()
    pb = res["phase_b"]
    assert res["contacts"] == 0
    assert min(pb["tracking"]["tracked_pct"].values()) > 90.0
    assert pb["tracking"]["swaps"] == 0
    assert all(v["median"] < 2.0 for v in pb["pos_err"].values())


def test_fastsim_school_board_far_half_is_tracked_by_the_lock_seeded_from_tags():
    import freeroam_fastsim as S
    s = S.Sim(1, 0.03, names=("Alpha", "Bravo"), board=(0, 0, 396, 168), dead_robot=False, pi_freeze=False,
              sensors=True, camera="detections", dropouts=False, scene=dict(tracking="tags", tag_x_max=100.0),
              starts={"Alpha": (60.0, 50.0, 0.0), "Bravo": (60.0, 120.0, 0.0)})
    res = s.run()
    tr = res["phase_b"]["tracking"]
    assert res["contacts"] == 0
    assert tr["far_frames"]["Alpha"] + tr["far_frames"]["Bravo"] > 0
    assert min(v for v in tr["far_tracked_pct"].values() if v is not None) > 80.0


# ------------------------------------------------------------------------------------------------- B4 pictures
def test_frames_snapshot_writes_each_cameras_newest_picture(tmp_path):
    import doe_loop as DL

    class FakeCam:
        cam = "overhead"
        latest = (1.0, b"\xff\xd8 not really a jpeg \xff\xd9")
    fr = DL.Frames.__new__(DL.Frames)
    fr.cams = [FakeCam()]
    paths = fr.snapshot(str(tmp_path), "120000_A_unsure")
    assert len(paths) == 1 and open(paths[0], "rb").read().startswith(b"\xff\xd8")


def test_lock_bank_refuses_a_seed_beside_another_robot():
    import lock_bank as LB

    class Cam:
        def px_per_cm(self, x, y, z):
            return 3.0
    lb = LB.LockBank(Cam(), 4.0)
    assert lb.seed("A", np.zeros((100, 100), np.uint8), 0.0, 50, 50, others=[(55, 50)]) is False
    assert lb.counts["refused_seeds"] == 1
