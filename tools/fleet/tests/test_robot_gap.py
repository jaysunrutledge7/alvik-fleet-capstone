"""Robot gap fix (build e02-nov03-robot-gap.md, 2026-11-03): in the school 40-point simulator run (corners clicked,
seed 2) two robots came 6.6 cm apart, under the 8 cm net. Bravo carried on a step it had been sent into a held Alpha:

  1. past the re-plan cap (keep going) the obstacle went on the map, but the spaced re-plan later decided who carries on
     with no new obstacle, so nothing was checked against it;
  2. a robot held while every robot was held was never put on the map, so the robot that rejoined first was planned
     straight through it;
  3. a held robot's outline was its odometry pose + the net, without the drift since its last camera fix.

    python3 -m pytest tools/fleet/tests/test_robot_gap.py -q
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_keep_going import coord, production_plan   # noqa: E402  (sets the import paths too)

import planner as P                 # noqa: E402
import run_plan as RP               # noqa: E402


def _block_first_drive(c, n="Alpha"):
    """A circle obstacle on robot n's first drive (the 'held robot in its way')."""
    st = next(s for s in c.plan["robots"][n]["steps"] if s["type"] == "drive")
    x, y = st["to"][0], st["to"][1]
    return {"id": "out-Held", "shape": "circle", "center": [x, y], "radius": 6.0, "source": "robot_out"}


def _decide_spy(c, monkeypatch):
    seen = []
    real = P.obstacle_decide

    def spy(m, plan, states, news, **kw):
        dec = real(m, plan, states, news, **kw)
        seen.append(([o.get("id") for o in news], dict(dec["keep"])))
        return dec
    monkeypatch.setattr(P, "obstacle_decide", spy)
    c.handle_obstacle = RP.Coordinator.handle_obstacle.__get__(c)
    c.act_on_decision = lambda *a, **k: None
    c.halt_all = lambda: None
    return seen


def test_spaced_replan_past_the_cap_checks_the_obstacle_put_on_the_map(monkeypatch):
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.plan_obstacle_keys = {c.obstacle_key(o) for o in c.m.get("obstacles", [])}
    seen = _decide_spy(c, monkeypatch)
    ob = _block_first_drive(c)
    c.replans = [{}] * 30
    c.last_replan_t = RP.now()                  # past the cap, spaced: the obstacle goes on the map, no decision yet
    c.handle_obstacle(ob)
    assert c.pending_replan is not None and not seen
    c.last_replan_t = 0.0                       # the spaced re-plan comes: handle_obstacle(None)
    c.handle_obstacle(None)
    ids, keep = seen[-1]
    assert ids == ["out-Held"]                  # the obstacle on the map since the plan was made is checked
    assert keep["Alpha"] == 0                   # Alpha does not carry on into it
    c.planning["thread"].join(10)


def test_gap_fix_off_is_the_old_check(monkeypatch):
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.args.gap_fix = "off"
    c.plan_obstacle_keys = {c.obstacle_key(o) for o in c.m.get("obstacles", [])}
    seen = _decide_spy(c, monkeypatch)
    c.add_obstacle(_block_first_drive(c))
    c.handle_obstacle(None)
    ids, keep = seen[-1]
    assert ids == [] and keep["Alpha"] > 0      # before the fix: Alpha carried on into it
    c.planning["thread"].join(10)


def test_obstacles_the_plan_was_made_with_are_not_counted_again(monkeypatch):
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.add_obstacle({"id": "brick", "shape": "circle", "center": [200.0, 160.0], "radius": 3.0, "source": "camera"})
    c.plan_obstacle_keys = {c.obstacle_key(o) for o in c.m.get("obstacles", [])}
    seen = _decide_spy(c, monkeypatch)
    c.handle_obstacle(None)
    assert seen[-1][0] == []
    c.planning["thread"].join(10)


def test_every_robot_held_the_last_one_still_goes_on_the_map():
    for fix, expect in (("on", True), ("off", False)):
        mm, plan = production_plan()
        c = coord(mm, plan)
        c.args.gap_fix = fix
        c.retire(c.bots["Bravo"], "nothing heard from it for 4.0 s", kind="silent")
        c.retire(c.bots["Alpha"], "it drove 41 cm with no camera seeing it", kind="silent")
        assert c.abort is None and not c.names
        on_map = any(o.get("id") == "out-Alpha" for o in c.m["obstacles"])
        assert on_map is expect


def test_held_robot_outline_grows_by_its_odometry_drift_capped():
    mm, plan = production_plan()
    c = coord(mm, plan)
    b = c.bots["Alpha"]
    b.odo, b.acc_odo = 40.0, 0.0                # 40 cm since its last camera fix, no camera now
    ob = c.out_obstacle(b)
    assert abs(ob["odometry_pm_cm"] - 2.8) < 0.01 and abs(ob["gap_cm"] - (6.5 + 2.8)) < 0.01
    b.odo = 1000.0
    assert c.out_obstacle(b)["odometry_pm_cm"] == RP.Coordinator.HELD_PM_MAX_CM
    c.args.held_pm_max_cm = 0.0                 # off: the old outline
    ob = c.out_obstacle(b)
    assert ob["odometry_pm_cm"] == 0.0 and abs(ob["gap_cm"] - 6.5) < 0.01


def test_held_robot_seen_by_a_camera_gets_no_extra():
    mm, plan = production_plan()
    c = coord(mm, plan)
    b = c.bots["Alpha"]
    b.odo, b.acc_odo = 40.0, 0.0
    b.fix_log = [{"t": RP.now(), "x": b.pose[0], "y": b.pose[1], "unc": 1.9, "cams": ["side"]}]
    assert c.out_obstacle(b)["odometry_pm_cm"] == 0.0
