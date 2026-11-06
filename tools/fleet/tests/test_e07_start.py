"""Build e07 (6 Nov 2026, phase 2: Mission Control for someone who has never seen it): where am I, the Start screen's
runs, automatic mapping (tag edges, the tiles as the ruler, the far side unseen), the surface-mark, stop-lead and
reserved-recovery changes."""
import argparse
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sites as SI            # noqa: E402
import site_detect as SD      # noqa: E402


# ------------------------------------------------------------------------------------------------- where am I
REPO_SITES = [SI.load("home"), SI.load("school-conference")]     # (sites made on this computer left out)
@pytest.mark.parametrize("room,site", [("school", "school-conference"), ("home", "home"),
                                       ("school-misnamed", "school-conference")])
def test_the_room_decides_the_site(room, site):
    r = SD.decide(dict(SD.SIM_ROOMS[room]), REPO_SITES)
    assert r["site"] == site
    assert r["words"].startswith("This looks like")


def test_cameras_beat_the_computer_name_and_the_last_site():
    ev = dict(SD.SIM_ROOMS["school-misnamed"])        # home's Pi name, home's camera name, last site home
    r = SD.decide(ev, REPO_SITES)
    assert r["site"] == "school-conference" and "reference tags" in r["words"]


def test_a_new_area_and_no_evidence():
    assert SD.decide(dict(SD.SIM_ROOMS["new"]), REPO_SITES)["site"] == "new"
    r = SD.decide({"camera_names": [], "n_cameras": 0, "ref_ids_seen": None, "table_cm": None, "last": "home"}, REPO_SITES)
    assert r["site"] == "home" and r.get("no_evidence")


def test_pinned_site_wins():
    ev = dict(SD.SIM_ROOMS["home"], pinned="school-conference")
    assert SD.decide(ev, REPO_SITES)["site"] == "school-conference"


def test_new_site_is_made_with_points_from_the_measured_table(tmp_path, monkeypatch):
    monkeypatch.setattr(SD, "LOCAL_DIR", str(tmp_path))
    monkeypatch.setattr(SI, "local_dir", lambda: str(tmp_path))
    name = SD.make_new_site("school-conference", table_cm=(182.0, 91.0))
    s = SI.load(name)
    assert s["table"]["length_cm"] == 182.0 and s.get("measure_trust") == "cameras"
    assert "pins" not in (s.get("tags") or {}) and not s.get("keepouts")
    import sticker_points as SP
    pts, zones, _ = SP.points_cm(s)
    assert len(pts) >= 6
    assert all(SD.GRID_EDGE_CM - 0.1 <= x <= 182 - SD.GRID_EDGE_CM + 0.1 and
               SD.GRID_EDGE_CM - 0.1 <= y <= 91 - SD.GRID_EDGE_CM + 0.1 for x, y in pts.values())


def test_site_overrides_are_laid_over_the_file(tmp_path):
    site = dict(SI.load("school-conference"), conf_dir=str(tmp_path))
    SI.set_override(site, "tags.height_cm", 6.4)
    SI.set_override(site, "table", {"length_cm": 400.0, "width_cm": 170.0})
    d = SI.apply_overrides(dict(site, tags=dict(site["tags"])))
    assert d["tags"]["height_cm"] == 6.4 and d["table"]["length_cm"] == 400.0
    SI.set_override(site, "tags.height_cm", None)
    d = SI.apply_overrides(dict(site, tags=dict(site["tags"])))
    assert d["tags"]["height_cm"] == 6.19
    with pytest.raises(ValueError):
        SI.set_override(site, "robots", [])


# ------------------------------------------------------------------------------------------------- mapping
def test_edges_find_a_blurred_square_unbiased():
    import cv2
    import tagcore as TC
    ss, W = 16, 120
    for side in (20.0, 36.0):
        img = np.full((W * ss, W * ss), 230, np.float32)
        c0, a = np.array([60.3, 59.7]), 0.3
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        h = side / 2
        cor = np.array([c0 + R @ v for v in ([-h, -h], [h, -h], [h, h], [-h, h])])
        cv2.fillPoly(img, [np.round(((cor + 0.5) * ss - 0.5) * 16).astype(np.int32)], 20.0, cv2.LINE_AA, 4)
        small = cv2.GaussianBlur(cv2.resize(img, (W, W), interpolation=cv2.INTER_AREA), (0, 0), 0.9)
        rc = TC.refine_edges(small.astype(np.float32), cor + (c0 - cor) * 0.04)
        s_ = np.mean([np.linalg.norm(rc[k] - rc[(k + 1) % 4]) for k in range(4)])
        assert abs(s_ / side - 1.0) < 0.005 and np.abs(rc - cor).max() < 0.2


def test_simulator_draws_the_tag_its_printed_size():
    import tag_cam as CAMS
    import tagcore as TC
    import site_fit as SF
    cam = CAMS.TagCamera((100, 80, 150), (100.01, 80, 0), size=(1280, 960), hfov_deg=70, blur_px=0.6, noise=1.0,
                         seed=2, table=(0, 0, 300, 200), jpeg=0)
    r = dict(x=100, y=80, th=0.3, z=0.18, family="tag25h9", id=1, tag_cm=12.0, roof_cm=15.5)
    det = TC.TagDetector("tag25h9", ids=[1], refine=True)
    d = det.detect_full(cam.render([r]))[0]
    uv = cam.project(SF.tag_corners_world(100, 80, math.degrees(0.3), 0.18, 12.0))
    true_side = np.mean([np.linalg.norm(uv[k] - uv[(k + 1) % 4]) for k in range(4)])
    assert abs(d.size_px / true_side - 1.0) < 0.005


def test_measure_trust_auto_and_small_reads():
    import site_fit as SF
    assert SF.trust_of({"ref_tags": {"reads": {"overhead": {"20": [[0, 0]] * 4}}}}) == "cameras"
    assert SF.trust_of({}) == "typed"
    assert SF.trust_of({"measure_trust": "typed", "ref_tags": {"reads": {"o": {"20": 1}}}}) == "typed"
    big = [[0, 0], [40, 0], [40, 40], [0, 40]]
    small = [[0, 0], [14, 0], [14, 14], [0, 14]]
    g = [{"id": 0, "reads": {"a": big, "b": small}}, {"id": 1, "reads": {"a": small}}]
    out = SF.small_reads_out(g, 3.0)
    assert len(out) == 1 and list(out[0]["reads"]) == ["a"]
    assert SF.min_cell_for({"ref_tags": {"reads": {"o": {"20": 1}}}, "roof_fit": False}) == SF.MIN_CELL_PX  # inset fitted
    import importlib
    SF.INSET_FIT = False
    try:
        assert SF.min_cell_for({"ref_tags": {"reads": {"o": {"20": 1}}}, "roof_fit": False}) == SF.MIN_CELL_PX_TILES
    finally:
        SF.INSET_FIT = True
    two = {"a": {}, "b": {}}
    assert SF.min_cell_for({"ref_tags": {"reads": {"o": {"20": 1}}}, "cameras": two}) == SF.MIN_CELL_PX  # roofs fitted


def test_far_end_not_seen_is_held_at_a_guess_and_said():
    import site_fit as SF
    obs, truth, st = SF.simulate(seed=5, n_spots=8, refs=True, corners=False, typed=False, stickers=0,
                                 hide_ref=["21", "22", "24", "25"])
    keep = next(c for c, rd in obs["ref_tags"]["reads"].items() if "20" in rd and "23" in rd)   # one camera only
    obs["cameras"] = {keep: obs["cameras"][keep]}
    obs["corners"] = {keep: {}}
    obs["ref_tags"]["reads"] = {keep: {k: v for k, v in obs["ref_tags"]["reads"][keep].items() if k in ("20", "23")}}
    obs["tag_groups"] = [dict(g, reads={keep: g["reads"][keep]}) for g in obs["tag_groups"] if keep in g["reads"]]
    p = SF.Problem(obs)
    assert "length" in p.far_free
    res = SF.solve(obs, use_typed=False)
    assert res["far_unseen"] == ["length"] and any("far end" in w for w in res["words"])


# ------------------------------------------------------------------------------------------------- runs
def test_experiment_routes_for_any_number_of_robots():
    import experiments as EX
    import sticker_points as SP
    site = SI.load("school-conference")
    pts, zones, _ = SP.points_cm(site)
    labels = sorted(pts, key=EX.num)
    two = EX.halves(site, labels, zones, names=["Alpha", "Bravo"], pts=pts)
    assert two == EX.halves(site, labels, zones)                      # Jay-Sun's halves exactly
    homes = {"Alpha": [30, 80], "Bravo": [200, 80], "Charlie": [350, 80]}
    three = EX.halves(site, labels, zones, names=list(homes), pts=pts, homes_xy=homes)
    assert sorted(sum(three.values(), [])) == sorted(labels)
    xs = {n: np.mean([pts[l][0] for l in v]) for n, v in three.items()}
    assert xs["Alpha"] < xs["Bravo"] < xs["Charlie"]
    orders, meta = EX.experiment_orders(site, pts, zones, homes, "e1")
    assert set(orders) == set(homes) and "3 runs" in meta["robots_split"]
    one = EX.halves(site, labels, zones, names=["Alpha"], pts=pts, homes_xy={"Alpha": [10, 10]})
    assert len(one["Alpha"]) == len(labels)


def test_roam_orders_wander_without_repeating_a_point_twice_in_a_row():
    import mission_run as MR
    o = MR.roam_orders({"Alpha": ["P01", "P02", "P03"], "Bravo": ["P09"]}, 5.0, seed=3)
    a = o["Alpha"]
    assert len(a) == max(4, int(5 * 60 / MR.ROAM_S_PER_POINT)) and all(x != y for x, y in zip(a, a[1:]))
    assert o["Bravo"] == ["P09"]


def _rp(**kw):
    import run_plan as RP
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.args = argparse.Namespace(**kw)
    c.logged = []
    c.log = lambda *a, **k: c.logged.append((a, k))
    c.say = lambda *a, **k: None
    return c


def test_missions_send_forward_drives_their_stop_lead_short():
    c = _rp(stop_lead=["Alpha=5:0.3,10:0.5"])
    st = {"type": "drive", "from": [0.0, 0.0, 0.0], "to": [40.0, 0.0], "speed": 7.5}
    out = c.with_stop_lead("Alpha", st)
    assert out["to"][0] == pytest.approx(40.0 - 0.4) and out["stop_lead_cm"] == pytest.approx(0.4)
    assert c.with_stop_lead("Bravo", st) is st                                  # no calibration: as planned
    assert c.with_stop_lead("Alpha", dict(st, reverse=True)) == dict(st, reverse=True)
    assert c.with_stop_lead("Alpha", dict(st, to=[2.0, 0.0])) == dict(st, to=[2.0, 0.0])
    assert c.with_stop_lead("Alpha", dict(st, speed=20.0))["stop_lead_cm"] == pytest.approx(0.5)


def test_recovery_corridor_covers_the_way_back():
    import geometry as G   # noqa: F401  (the planner's own shapes)
    c = _rp()
    ob = c.recovery_corridor("Alpha", 7, 100.0, 50.0, 130.0, 50.0, 3.0)
    xs = [p[0] for p in ob["points"]]
    ys = [p[1] for p in ob["points"]]
    hw = 6.72 + 3.0 + 1.0
    assert min(xs) == pytest.approx(100.0 - hw) and max(xs) == pytest.approx(130.0 + hw)
    assert min(ys) == pytest.approx(50.0 - hw) and ob["source"] == "robot_out"
    import fleetmap as FM
    FM._norm_obstacle(ob, 0)                       # a source and shape the map accepts


def test_run_plan_defaults_surface_marks_logged_only_and_recovery_reserved():
    import run_plan as RP
    src = open(RP.__file__).read()
    assert 'choices=["log", "act", "auto", "off"], default="log"' in src
    assert '"--recover-reserve", choices=["on", "off"], default="on"' in src


# ------------------------------------------------------------------------------------------------- the Start screen
def test_start_screen_home_and_school(tmp_path, monkeypatch):
    import mission_control as MC
    import start_job as SJB
    monkeypatch.setenv("MIRLT_SITE", "home")
    s = MC.Session(str(tmp_path / "home"))
    st = SJB.Starter(s, where={"site": "home", "kind": "home", "words": "x"}, practice=True)
    b = {x["kind"]: x for x in st.buttons()}
    assert b["calibrate"]["ok"] and b["freeroam"]["ok"] and not b["visit"]["ok"] and "home" in b["visit"]["why"]
    state = st.state()
    assert state["practice"] and "Practice" in state["robots"]["words"]
    monkeypatch.setenv("MIRLT_SITE", "school-conference")
    s2 = MC.Session(str(tmp_path / "school"))
    st2 = SJB.Starter(s2, where={"site": "school-conference", "kind": "school", "words": "x"}, practice=True)
    assert all(x["ok"] for x in st2.buttons())
    assert st2.robots_words()[1] == ["Alpha", "Bravo"]
    st2.set_advanced({"practice_robots": "3", "sleep_when_done": "false", "experiment": "e3"})
    assert st2.adv["practice_robots"] == 3 and st2.adv["sleep_when_done"] is False and st2.next_experiment() == "e3"
    assert json.load(open(st2.adv_path))["practice_robots"] == 3
    with pytest.raises(ValueError):
        st2.set_advanced({"nonsense": 1})


def test_robots_sleep_when_a_real_run_ends_not_after_practice(tmp_path, monkeypatch):
    import mission_control as MC
    import start_job as SJB
    monkeypatch.setenv("MIRLT_SITE", "home")
    s = MC.Session(str(tmp_path / "h"))
    st = SJB.Starter(s, practice=False)
    sent = []
    st.sleep_robots = lambda why: sent.append(why)
    st.active, st.active_practice = "freeroam", False
    st.status()
    assert sent == ["the run ended"]
    st2 = SJB.Starter(s, practice=True)
    st2.sleep_robots = lambda why: sent.append(why)
    st2.active, st2.active_practice = "freeroam", True
    st2.status()
    assert sent == ["the run ended"]
    st3 = SJB.Starter(s, practice=False)
    st3.adv["sleep_when_done"] = False
    st3.sleep_robots = lambda why: sent.append(why)
    st3.active, st3.active_practice = "freeroam", False
    st3.status()
    assert len(sent) == 1


def test_attended_only_site_asks_for_jay_sun(tmp_path, monkeypatch):
    import mission_control as MC
    import start_job as SJB
    monkeypatch.setenv("MIRLT_SITE", "school-conference")
    s = MC.Session(str(tmp_path / "s"))
    st = SJB.Starter(s, practice=False)
    with pytest.raises(ValueError, match="Jay-Sun"):
        st.start("visit")


def test_simulator_room_for_a_small_new_table_does_not_hang():
    import tag_cam as CAMS
    cfg = CAMS.default_config(table=(182.0, 91.0), seed=3)          # (e07: 12 stickers 25 cm apart never fitted)
    assert 1 <= len(cfg["stickers"]) <= 12


def test_a_new_area_made_earlier_does_not_claim_the_school_tiles(tmp_path, monkeypatch):
    monkeypatch.setattr(SD, "LOCAL_DIR", str(tmp_path))
    monkeypatch.setattr(SI, "local_dir", lambda: str(tmp_path))
    name = SD.make_new_site("school-conference", table_cm=(182.0, 91.0))
    sites = REPO_SITES + [SI.load(name)]
    assert SD.decide(dict(SD.SIM_ROOMS["school-misnamed"]), sites)["site"] == "school-conference"
    assert SD.decide(dict(SD.SIM_ROOMS["new"]), sites)["site"] == name          # back in the same new area


def test_unclear_cameras_alone_keep_the_last_site():
    ev = {"camera_names": ["Some Webcam"], "n_cameras": 1, "ref_ids_seen": None, "table_cm": None, "host": "x",
          "last": "home"}
    r = SD.decide(ev, REPO_SITES)
    assert r["site"] == "home" and r.get("unsure")
