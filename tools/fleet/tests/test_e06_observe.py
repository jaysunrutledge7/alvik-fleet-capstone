"""Build e06 (5 Nov 2026, phase 1: multi-robot, observe then trust): the pieces wired into the runs."""
import json
import math
import os
import sys
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import observe as OB          # noqa: E402
import sites as SI            # noqa: E402


def test_camnames_one_camera_is_the_overhead_whatever_its_name():
    import camnames as C
    cams = [dict(dev=0, name="USB 2.0 Camera", port="1.1")]
    conf = {"overhead": {"name": "C920e", "port": ""}, "side": {"name": "", "port": ""}}
    dev, words = C.resolve("overhead", cams, conf)
    assert dev == 0 and "observed" in words
    assert C.resolve("side", cams, conf)[0] is None                  # camera 2 switched off stays off
    conf["observe"] = False
    assert C.resolve("overhead", cams, conf)[0] is None              # the old strict names, behind the switch


def test_two_cameras_wrong_names_both_used():
    import camnames as C
    cams = [dict(dev=0, name="A", port="1.1"), dict(dev=2, name="B", port="1.2")]
    conf = {"overhead": {"name": "C920e", "port": ""}, "side": {"name": "EMEET", "port": ""}}
    assert {C.resolve("overhead", cams, conf)[0], C.resolve("side", cams, conf)[0]} == {0, 2}


def test_auto_slots_any_number():
    import wiggle_bind as WB
    site = SI.load("school-conference")
    assert WB.auto_slots(site, 1) == ["Alpha"]
    assert WB.auto_slots(site, 3) == ["Alpha", "Bravo", "Charlie"]
    assert len(set(WB.auto_slots(site, 8))) == 8


def test_assign_slots_more_robots_than_homes():
    import wiggle_bind as WB
    homes = {"Alpha": (149.0, 82.0), "Bravo": (243.0, 82.0)}
    seen = {"AL-1": (240.0, 80.0), "AL-2": (150.0, 80.0), "AL-3": (40.0, 40.0)}
    out = WB.assign_slots(["Alpha", "Bravo", "Charlie"], list(seen), seen, homes)
    assert out == {"AL-2": "Alpha", "AL-1": "Bravo", "AL-3": "Charlie"}


def test_tags_with_binding_carries_heading_offsets():
    import wiggle_bind as WB
    rec = {"slots": {"Alpha": 2, "Bravo": 3}, "yaw_off": {"Alpha": 180.0}}
    t = WB.tags_with_binding({"ids": {"Alpha": 0}}, rec)
    assert t["yaw_off"] == {"2": 180.0} and t["ids"] == {"Alpha": 2, "Bravo": 3}


def test_idle_sleep_due():
    import mission_control as MC
    assert not MC.idle_sleep_due(True, 0.0, 10000.0, False, 30)
    assert MC.idle_sleep_due(False, 0.0, 30 * 60 + 1, False, 30)
    assert not MC.idle_sleep_due(False, 0.0, 30 * 60 + 1, True, 30)
    assert not MC.idle_sleep_due(False, 0.0, 1e9, False, 0)
    assert not MC.idle_sleep_due(False, None, 1e9, False, 30)


def _margs(**kw):
    a = argparse.Namespace(site_=SI.load("school-conference"), dry=True, cam_size="1280x960", seed=1, no_side=True,
                           no_refs=False, corner_clicks="auto", hide_ref="", experiment_=None, split="auto",
                           auto_robots=True, seen_=None, sim_homes_=None, tags_=None)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def test_one_camera_shares_only_what_it_reads(tmp_path):
    import mission_run as MR
    a = _margs()
    rm = MR.run_readmap(a, str(tmp_path))
    assert rm is not None
    assert rm.readable(60, 80)[0] and not rm.readable(350, 80)[0]      # camera 2 off: the far half is not read
    for n in (1, 2, 3):
        names = ["Alpha", "Bravo", "Charlie"][:n]
        a.sim_homes_ = MR.dry_homes(a, names)
        home_at, orders = MR.observed_split(a, names, str(tmp_path))
        got = [l for v in orders.values() for l in v]
        assert len(got) == len(set(got))                                 # every point once
        assert all(rm.readable(*home_at[k][:2])[0] for k in names)       # every robot starts where it is read
        assert a.skipped_ and all("does not read" in w for w in a.skipped_.values())
        assert not set(got) & set(a.skipped_)
        assert set(orders) == set(names)


def test_halves_switch_kept(tmp_path):
    import mission_run as MR
    a = _margs(split="halves", auto_robots=False)
    home_at, orders = MR.observed_split(a, ["Alpha", "Bravo"], str(tmp_path))
    assert orders is None                      # the site's fixed zones, as before


def test_safety_settings_per_site():
    assert OB.site_safe_edge(SI.load("school-conference")) == 2
    cmds = OB.safety_commands(2)
    assert "set safe_edge 2" in cmds and "set tilt_stop_deg 5" in cmds and "set tilt_ms 300" in cmds
    import mission_run as MR
    g = MR.guard_cmds(argparse.Namespace(site_=SI.load("school-conference")))
    assert "set safe_edge 2" in g and "set tilt_stop_deg 5" in g and "set safe_edge 3" not in g


def test_trip_helpers():
    assert OB.trip_kind("tilted 6.1 deg") == "tilt"
    assert OB.trip_kind("plywood edge at 1 2 3 bottom") == "drop"
    assert OB.trip_kind("plywood edge at 1 2 3 line") == "line"
    assert OB.trip_kind("obstacle 3 cm ahead") is None
    bx, by = OB.back_off_target(50, 50, 0)
    assert bx < 50 and abs(by - 50) < 1e-9                               # backwards, the heading kept
    ko = OB.trip_keepout(50, 50, 0)
    assert ko["center"][0] > 50


def test_run_plan_obstacles_from_trips_and_hazards_are_valid_map_obstacles():
    """(rehearsal s1 crashed at its end: a hazard obstacle's source was not a map source)"""
    import fleetmap as FM
    hz = {"id": "hazard-Alpha-3", "shape": "circle", "center": [130.0, 23.0], "radius": 6.0, "source": "user",
          "why": "hazard: moves failed here (11-05h)"}
    FM._norm_obstacle(hz, 0)
    tr = {"id": "edge-by-Alpha-1", "shape": "polygon", "points": [[0, 0], [10, 0], [10, 5], [0, 5]],
          "source": "edge_guard", "cue": "tilt", "why": "tilt trip (discovered edge)"}
    FM._norm_obstacle(tr, 1)
