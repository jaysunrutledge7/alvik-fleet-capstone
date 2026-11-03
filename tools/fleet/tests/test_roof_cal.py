"""Roof tag height, tilt and offset measured in the auto calibration (2026-11-03, Nick 13:56): the tilt geometry, the
pose with a measured roof (exact under tilt and offset), the offset from a spin, the plain words and the switches, the
site fit measuring a 9 cm roof with the 6 cm placeholder (simulator pictures), the bridge using it. No robot, no Pi."""
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment"),
          os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import roof_cal as RC        # noqa: E402
import tag_cam as CAMS       # noqa: E402
import tag_geom as TG        # noqa: E402
import tagcore as TC         # noqa: E402


def _cam():
    return CAMS.TagCamera((100.0, 80.0, 200.0), (110.0, 85.0, 0.0), size=(1280, 960), hfov_deg=74.0, k1=0.02).true_model()


def _det(cam, P):
    u, v = cam.project(np.asarray(P))
    return TC.Detection(0, np.column_stack([u, v]), cam="c", t=0.0)


def test_tilt_vector_lean_directions():
    z = np.array([0, 0, 1.0])
    n = TG.rodrigues(TG.tilt_vec(30, 90)) @ z          # face towards the robot's left (+y in its frame)
    assert abs(math.degrees(math.acos(n[2])) - 30) < 1e-6 and n[1] > 0.49 and abs(n[0]) < 1e-9
    n = TG.rodrigues(TG.tilt_vec(10, 0)) @ z           # towards the nose
    assert n[0] > 0.17
    for t, l in ((12.0, 90.0), (5.0, 200.0), (30.0, 315.0)):
        t2, l2 = TG.tilt_of_vec(TG.tilt_vec(t, l))
        assert abs(t2 - t) < 1e-9 and abs(l2 - l) < 1e-9
    assert RC.lean_words(90) == "its left" and RC.lean_words(180) == "its back" and RC.lean_words(10) == "the nose"


def test_pose_exact_with_tilt_and_offset():
    """A 30 deg tag, 1.5 cm ahead and 1 cm right of the turning point, 7 cm up: the measured roof gives the turning point
    and heading exactly (noise-free corners); the flat model misses the heading and the offset."""
    cam = _cam()
    roof = {"height_cm": 7.0, "tilt_deg": 30.0, "lean_deg": 90.0, "offset_cm": [1.5, -1.0]}
    H, tv, off = TG.roof_of(roof)
    worst_new, worst_old = 0.0, 0.0
    for x, y, hd in ((40, 30, 0), (160, 120, 135), (100, 80, 250), (20, 150, 60)):
        P = TG.roof_corners_world(x, y, math.radians(hd), H, 8.0, tv, off)
        d = _det(cam, P)
        p = TG.pose_from_detection(cam, d, 6.0, 8.0, roof=roof)
        assert math.hypot(p.x - x, p.y - y) < 0.02 and abs(TG.wrap180(p.heading - hd)) < 0.05
        assert abs(p.size_ratio - 1.0) < 0.002
        q = TG.pose_from_detection(cam, d, 7.0, 8.0)          # flat model, right height: the middle, not the robot
        worst_new = max(worst_new, math.hypot(p.x - x, p.y - y))
        worst_old = max(worst_old, math.hypot(q.x - x, q.y - y))
        assert math.hypot(q.x - (p.mid[0]), q.y - p.mid[1]) < 0.02     # the middle is the same (diagonals' crossing)
    assert worst_old > 1.5 and worst_new < 0.02


def _spin_csv(path, off, centre=(150.0, 60.0), noise=0.15, seed=1):
    rng = np.random.default_rng(seed)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "cam", "robot", "id", "x", "y", "heading"])
        for k in range(60):
            th = math.radians(6.0 * k)
            c, s = math.cos(th), math.sin(th)
            x = centre[0] + c * off[0] - s * off[1] + rng.normal(0, noise)
            y = centre[1] + s * off[0] + c * off[1] + rng.normal(0, noise)
            w.writerow([10 + 0.2 * k, "overhead", "Alpha", 0, x, y, math.degrees(th)])
        for k in range(60):                    # driving straight afterwards: not a spin
            w.writerow([40 + 0.2 * k, "overhead", "Alpha", 0, centre[0] + 3 * k, centre[1], 0.0])


def test_offset_from_spin(tmp_path):
    p = str(tmp_path / "tag_poses.csv")
    _spin_csv(p, (1.5, -0.8))
    o = RC.offsets_from_poses(p)["Alpha"]
    assert abs(o["offset_cm"][0] - 1.5) < 0.1 and abs(o["offset_cm"][1] + 0.8) < 0.1 and o["sd_cm"] < 0.1
    _spin_csv(p, (0.0, 0.0))
    o = RC.offsets_from_poses(p)["Alpha"]
    assert math.hypot(*o["offset_cm"]) < 0.1


def _res(h=9.0, hsd=0.3, tilt=0.5, lean=0.0, two=6):
    tv = TG.tilt_vec(tilt, lean)
    return {"roofs": {"0": {"height_cm": h, "height_sd_cm": hsd, "tilt_deg": tilt, "tilt_sd_deg": 1.0, "lean_deg": lean,
                            "tilt_vec": [tv[0], tv[1]], "moments": 6, "two_camera_moments": two, "cameras": ["a"]}}}


def test_words_level_and_not_level():
    obs = {"tag_names": {"0": "Alpha"}, "tag_height_cm": 6.0}
    ok = RC.summarize(_res(), obs, typed_cm=None, placeholder=True)["robots"]["Alpha"]
    assert ok["level"] and ok["height_used"] and "level, OK" in ok["words"] and "placeholder 6.0" in ok["words"]
    bad = RC.summarize(_res(tilt=12.0, lean=90.0), obs)["robots"]["Alpha"]
    assert not bad["level"] and "NOT LEVEL" in bad["words"] and "its left" in bad["words"] and "wedge" in bad["words"]
    unsure = RC.summarize(_res(hsd=2.5, two=0), obs)["robots"]["Alpha"]
    assert not unsure["height_used"] and "only one camera" in unsure["words"]


def test_bridge_roofs_and_switches(tmp_path):
    obs = {"tag_names": {"0": "Alpha"}, "tag_height_cm": 6.0}
    rt = RC.summarize(_res(h=9.0, tilt=12.0, lean=90.0), obs)
    rt["robots"]["Alpha"].update(offset_cm=[1.2, 0.0], offset_sd_cm=0.1, offset_used=True)
    with open(tmp_path / "roof_tags.json", "w") as f:
        json.dump(rt, f)
    roofs, words = RC.roofs_for_bridge(str(tmp_path), {"height_cm": 6.0})
    assert roofs[0]["height_cm"] == 9.0 and roofs[0]["tilt_deg"] == 12.0 and roofs[0]["offset_cm"] == [1.2, 0.0]
    roofs, _ = RC.roofs_for_bridge(str(tmp_path), {"height_cm": 6.0, "tilt_correct": False})
    assert "tilt_deg" not in roofs[0] and roofs[0]["height_cm"] == 9.0
    roofs, _ = RC.roofs_for_bridge(str(tmp_path), {"height_cm": 6.0, "roof_measured": "off"})
    assert roofs[0]["height_cm"] == 6.0 and "offset_cm" not in roofs[0]
    roofs, words = RC.roofs_for_bridge(str(tmp_path), {"height_cm": 6.0, "roof_measured": False, "tilt_correct": False})
    assert roofs == {} and "switched off" in words[0]
    assert RC.roofs_for_bridge(str(tmp_path / "none"), {"height_cm": 6.0})[0] == {}


def test_bridge_uses_roof(tmp_path):
    """The bridge's pose for a robot with a measured roof: the turning point, not the tag's middle."""
    import tag_bridge as TB
    cam = _cam()
    roof = {"height_cm": 6.0, "offset_cm": [2.0, 0.0]}
    tags = {"family": "tag25h9", "tag_cm": 8.0, "height_cm": 6.0, "ids": {"Alpha": 0}}
    br = TB.Bridge(tags, {"c": cam}, lambda m: None, str(tmp_path), observe=True, roofs={0: roof})
    P = TG.roof_corners_world(100.0, 80.0, 0.0, 6.0, 8.0, None, (2.0, 0.0))
    p = TG.pose_from_detection(cam, _det(cam, P), 6.0, 8.0, roof=br.roofs[0])
    assert abs(p.x - 100.0) < 0.05 and abs(p.mid[0] - 102.0) < 0.05
    s = br.summary()
    br.close()
    assert s["roofs"]["Alpha"]["offset_cm"] == [2.0, 0.0]


def test_measure_finds_9cm_with_6cm_placeholder(tmp_path):
    """Simulator pictures (two half cameras, reference tags): both roofs truly 9 cm up, the setup told 6 (placeholder):
    the Measure fit says ~9 and uses it; a 15 deg tilt is found and named."""
    import roof_sim as RS
    truth = {"Alpha": {"height_cm": 9.0}, "Bravo": {"height_cm": 6.0, "tilt_deg": 15.0, "lean_deg": 90.0}}
    cfg = RS.config("two")
    cams = CAMS.cams_from_config(cfg)
    res = RS.measure(cfg, cams, truth, str(tmp_path), n_pics=10, say=lambda s: None)
    r = res["roof_tags"]["robots"]
    assert abs(r["Alpha"]["height_cm"] - 9.0) < 0.6 and r["Alpha"]["height_used"]
    assert r["Alpha"]["level"]
    assert abs(r["Bravo"]["tilt_deg"] - 15.0) < 5.0 and not r["Bravo"]["level"]
    assert os.path.exists(tmp_path / "tagcal" / "roof_tags.json")
    assert any("NOT LEVEL" in w for w in res["words"])


def test_one_camera_still_tilt_not_judged(tmp_path):
    """One camera, a still robot: a tilt reading is not trusted (no false NOT LEVEL), the bridge does not correct it;
    after a spin (headings round the circle) it is."""
    obs = {"tag_names": {"0": "Alpha"}, "tag_height_cm": 6.0}
    r = _res(tilt=20.0, lean=300.0, two=0)
    r["roofs"]["0"]["heading_spread_deg"] = 0.0
    d = RC.summarize(r, obs)["robots"]["Alpha"]
    assert d["level"] is None and "not checked yet" in d["words"] and "NOT LEVEL" not in d["words"]
    with open(tmp_path / "roof_tags.json", "w") as f:
        json.dump(RC.summarize(r, obs), f)
    assert "tilt_deg" not in RC.roofs_for_bridge(str(tmp_path), {"height_cm": 6.0})[0][0]
    r["roofs"]["0"]["heading_spread_deg"] = 330.0
    assert RC.summarize(r, obs)["robots"]["Alpha"]["level"] is False


def test_typed_height_per_robot(tmp_path):
    obs = {"tag_names": {"0": "Alpha"}, "tag_height_cm": 6.0, "roof_typed_cm": {"0": 8.4}}
    rt = RC.summarize(_res(h=7.0, hsd=3.0, two=0), obs)
    assert "typed 8.4 cm kept" in rt["robots"]["Alpha"]["words"]
    with open(tmp_path / "roof_tags.json", "w") as f:
        json.dump(rt, f)
    roofs, _ = RC.roofs_for_bridge(str(tmp_path), {"height_cm": 6.0, "heights_cm": {"Alpha": 8.4}})
    assert roofs[0]["height_cm"] == 8.4
