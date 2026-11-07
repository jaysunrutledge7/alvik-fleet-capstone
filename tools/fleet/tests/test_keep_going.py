"""Keep going and confidence at every point (build e02-nov03-keep-going-confidence.md, 2026-11-03; Nick 2 Nov 21:01-21:06:
"finishing the run with the highest confidence positional assessment it can make every step of the way, sometimes
that won't be 100%"; "The camera should be more trusted than the sensor if we're using April tags").

Each converted stop under on_problem keep_going (and that "stop" is unchanged), the reassignment of a lost robot's
points through the real planner, the skipped point and its dwell, the feel-out's camera trust, the site policy, and the
confidence numbers (camera fixes, odometry growth, a bumped camera).

    python3 -m pytest tools/fleet/tests/test_keep_going.py -q
"""
import json
import math
import os
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import feel_table as FT      # noqa: E402
import fleetmap as FM        # noqa: E402
import missions as MS        # noqa: E402
import planner as P          # noqa: E402
import production as PR      # noqa: E402
import run_plan as RP        # noqa: E402
import sites as SI           # noqa: E402

L, W = 396.2, 167.6


# ------------------------------------------------------------------------------------------------------------ helpers

def production_plan(orders=None, points=None):
    pts = points or {"1": [80.0, 120.0], "2": [320.0, 120.0], "3": [120.0, 50.0], "4": [280.0, 50.0],
                     "5": [60.0, 60.0], "6": [340.0, 60.0]}
    m, _ = PR.site_map((L, W), ["Alpha", "Bravo"], points=pts)
    prog = PR.program(orders or {"Alpha": ["1", "3", "5"], "Bravo": ["2", "4", "6"]})
    mm = MS.build(m, prog)
    over = {"separation_cm": 8.0}
    plan = MS.plan_synced(mm, over=over, t_limit_s=30.0)
    return mm, plan


class _Src:
    live = True

    def poll(self):
        return []


def coord(mm, plan, on_problem="keep_going", lost="reassign", self_correct=False):
    args = types.SimpleNamespace(on_problem=on_problem, lost_robot_points=lost, max_replans=30, retry_s=3.0,
                                 min_gap_cm=8.0, unconfirmed_s=15.0, cam_fit="", bridge_events="", improve_after_s=1.0,
                                 improve_budget_s=6.0, ahead=2, resend_s=0.3, fix_hz=10.0)
    c = RP.Coordinator.__new__(RP.Coordinator)
    c.args, c.source = args, _Src()
    c.m = FM.normalize(mm)
    c.plan = plan
    c.names = list(plan["robots"])
    c.bots = {n: RP.Bot(n) for n in c.names}
    c.plan_no, c.replans, c.deferred, c.limit = 0, [], [], {}
    c.planning, c.pending_replan, c.improve_at, c.abort = None, None, None, None
    c.n_sensor_obstacles = 0
    c.t_run = c.plan_start = RP.now() - 5.0
    c.self_correct = self_correct
    c.logged, c.said, c.sent, c.obstacles = [], [], [], []
    c.log = lambda ev, **k: c.logged.append((ev, k))
    c.say = c.said.append
    c.send = lambda n, t: c.sent.append((n, t))
    c.control = lambda b, t, ok=None: c.sent.append((b.name, t))
    c.handle_obstacle = lambda ob, cause=None, budget=None, improving=False: c.obstacles.append((ob, cause))
    c._install(plan, keep={}, first={}, old=None)
    for n in c.names:
        c.bots[n].pose = tuple(plan["robots"][n]["start"])
    return c


def replan(c):
    """What the run does next: the planner's own re-plan from the coordinator's plan (as handle_obstacle +
    finish_replan)."""
    states = c.states()
    dec = P.obstacle_decide(c.m, c.plan, states, [], over=dict(c.plan["settings"]), t_now=1.0)
    newplan, _ = P.obstacle_replan(dec)
    return newplan


def labels(plan, n):
    return [st.get("label") for st in plan["robots"][n]["steps"] if st.get("label")]


# ------------------------------------------------------------------------------------------------------------ sites

def test_site_policy_defaults_home_stop_school_keep_going():
    h, s = SI.load("home"), SI.load("school-conference")
    assert (h["on_problem"], h["feel_out_trust"]) == ("keep_going", "sensor")      # (e08, Q7): home keeps going too
    assert (s["on_problem"], s["feel_out_trust"], s["lost_robot_points"]) == ("keep_going", "camera", "reassign")
    assert "keep going" in SI.words(h)
    assert "keep going" in SI.words(s) and "trusted" in SI.words(s)


def test_site_policy_bad_word_stops_loudly(tmp_path, monkeypatch):
    d = json.load(open(os.path.join(SI.SITES_DIR, "school-conference.json")))
    d["on_problem"] = "keepgoing"
    monkeypatch.setattr(SI, "SITES_DIR", str(tmp_path))
    (tmp_path / "x.json").write_text(json.dumps(d))
    with pytest.raises(SI.SiteFileError) as e:
        SI.load("x")
    assert "on_problem" in str(e.value) and "keep_going" in str(e.value)


def test_mission_run_resolves_the_policy_from_the_site_and_the_override():
    import mission_run as MR
    a = MR.main.__globals__["argparse"].Namespace(site="school-conference", tracking="auto", robots="Left,Right",
                                                  on_problem="auto", lost_robot_points="auto", feel_trust="auto")
    MR.resolve_site(a)
    assert (a.on_problem_, a.lost_robot_points_, a.feel_out_trust_) == ("keep_going", "reassign", "camera")
    a2 = MR.main.__globals__["argparse"].Namespace(site="home", tracking="auto", robots="Left,Right",
                                                   on_problem="auto", lost_robot_points="auto", feel_trust="auto")
    MR.resolve_site(a2)
    assert a2.on_problem_ == "keep_going" and a2.feel_out_trust_ == "sensor"      # (e08, Q7)
    a3 = MR.main.__globals__["argparse"].Namespace(site="school-conference", tracking="auto", robots="Left,Right",
                                                   on_problem="stop", lost_robot_points="skip", feel_trust="sensor")
    MR.resolve_site(a3)
    assert (a3.on_problem_, a3.lost_robot_points_, a3.feel_out_trust_) == ("stop", "skip", "sensor")


# ----------------------------------------------------------------------------------- converted stops (run_plan)

def test_cam_guard_keep_going_snaps_the_pose_stop_still_stops():
    mm, plan = production_plan()
    for pol, abort in (("keep_going", False), ("stop", True)):
        c = coord(mm, plan, on_problem=pol)
        b = c.bots["Alpha"]
        t = RP.now()
        b.pose, b.pose_t = (100.0, 80.0, 90.0), t
        b.last_cam, b.cam_t = [110.0, 80.0, 90.0], t
        b.guard_since = t - 2.0
        c.halt_all = lambda: c.sent.append(("all", "halt"))
        c.cam_guard()
        if abort:
            assert c.abort and "disagree" in c.abort
        else:
            assert c.abort is None
            assert any(x[1].startswith("fpose 110.00 80.00") for x in c.sent)       # set to the camera's
            assert any(e[0] == "keep_going" and e[1]["kind"] == "cam_guard" for e in c.logged)
            assert any("KEEP GOING" in s for s in c.said)


def test_out_of_replans_keep_going_goes_on_spaced_stop_ends():
    mm, plan = production_plan()
    c = coord(mm, plan, on_problem="stop")
    c.handle_obstacle = RP.Coordinator.handle_obstacle.__get__(c)
    c.halt_all = lambda: None
    c.replans = [{}] * 30
    c.handle_obstacle(None)
    assert c.abort == "out of re-plans"
    c = coord(mm, plan)
    c.handle_obstacle = RP.Coordinator.handle_obstacle.__get__(c)
    c.halt_all = lambda: c.sent.append(("all", "halt"))
    c.replans = [{}] * 30
    c.last_replan_t = RP.now()                  # the last re-plan just now: this one waits its turn
    c.handle_obstacle({"id": "brick", "shape": "circle", "center": [200.0, 80.0], "radius": 5.0, "source": "camera"})
    assert c.abort is None and c.pending_replan is not None
    assert any(o.get("id") == "brick" for o in c.m["obstacles"])             # the obstacle is on the map meanwhile
    assert any("KEEP GOING" in s and "cap" in s for s in c.said)


def test_failed_replan_skips_the_robots_point_then_sets_it_out():
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.halt_all = lambda: None
    retired = []
    c.retire = lambda b, why, kind="out": retired.append((b.name, why, kind))
    vo0 = [v.get("label") for v in c.plan["robots"]["Alpha"]["visit_order"]]
    c.replan_trouble("no new plan (test)", None, "Alpha")
    vo1 = [v.get("label") for v in c.plan["robots"]["Alpha"]["visit_order"]]
    assert vo0[:2] == ["point 1", "at point 1"] and vo1[0] == "point 3"         # its first point and its dwell gone
    assert c.skipped[0]["point"] == "point 1" and c.pending_replan
    # the plan made after the skip goes on with point 3
    np_ = replan(c)
    assert labels(np_, "Alpha")[0] == "point 3"
    for _ in range(RP.Coordinator.REPLAN_FAIL_MAX):
        c.replan_trouble("no new plan (test)", None, None)                     # retries remember the robot
    assert [q["point"] for q in c.skipped] == ["point 1", "point 3", "point 5"]   # one point per failure
    assert retired and retired[0][0] == "Alpha" and retired[0][2] == "out"


def test_retire_keep_going_reassigns_points_through_the_planner():
    mm, plan = production_plan()
    c = coord(mm, plan)
    b = c.bots["Bravo"]
    b.stat[0]["status"] = "done"
    c.retire(b, "nothing heard from it for 5.0 s", kind="silent")
    vo = [v.get("label") for v in c.plan["robots"]["Alpha"]["visit_order"]]
    assert vo[-1] == "home" and {"point 2", "point 4", "point 6"} <= set(vo)
    assert vo.index("point 2") < vo.index("home") and vo.index("at point 2") == vo.index("point 2") + 1
    assert {r["point"] for r in c.reassigned} == {"point 2", "point 4", "point 6"}
    assert b.out_entry["visit_order"][0]["label"] == "home" and not b.out_entry["steps"]   # rejoins: only home
    assert any("points not done go to" in s for s in c.said)
    # the planner takes them: Alpha's new plan stands at every point, its own and Bravo's, then home
    np_ = replan(c)
    labs = labels(np_, "Alpha")
    for k in "123456":
        assert "at point %s" % k in labs
    assert labs[-1] == "home"


def test_points_already_stood_at_are_not_handed_over():
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.done_labels = {"at point 2"}
    c.retire(c.bots["Bravo"], "its follow run ended (lifted)", kind="restarted")
    assert {r["point"] for r in c.reassigned} == {"point 4", "point 6"}


def test_reassign_to_a_robot_that_already_finished():
    mm, plan = production_plan()
    c = coord(mm, plan)
    a = c.bots["Alpha"]
    for st in a.stat:
        st["status"] = "done"                       # Alpha is home
    a.pose = tuple(plan["robots"]["Alpha"]["steps"][-1]["to"])
    c.retire(c.bots["Bravo"], "nothing heard from it", kind="silent")
    pend = c.plan["robots"]["Alpha"]["pending"]
    assert [v["label"] for v in pend["visits"]][-1] == "home" and pend["visits"][0]["label"] == "point 2"
    np_ = replan(c)
    labs = labels(np_, "Alpha")
    assert "at point 2" in labs and "at point 6" in labs and labs[-1] == "home"


def test_skip_policy_and_stop_policy_do_not_reassign():
    mm, plan = production_plan()
    c = coord(mm, plan, lost="skip")
    c.retire(c.bots["Bravo"], "nothing heard", kind="silent")
    assert not getattr(c, "reassigned", [])
    assert any("are not done" in s for s in c.said)
    assert c.bots["Bravo"].out_entry["steps"]                                   # it rejoins with its own points
    c = coord(mm, plan, on_problem="stop")
    c.retire(c.bots["Bravo"], "nothing heard", kind="silent")
    assert not getattr(c, "reassigned", []) and not any("KEEP GOING" in s for s in c.said)


def test_every_robot_out_is_still_a_stop():
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.retire(c.bots["Bravo"], "nothing heard", kind="out")
    c.retire(c.bots["Alpha"], "nothing heard", kind="out")
    assert c.abort and c.abort.startswith("every robot is out")


def test_a_reassigned_point_lost_by_a_replan_goes_back_in():
    mm, plan = production_plan()
    c = coord(mm, plan)
    c.retire(c.bots["Bravo"], "nothing heard", kind="silent")
    # a re-plan computed before the hand-over comes in without them
    c.plan["robots"]["Alpha"]["visit_order"] = [v for v in c.plan["robots"]["Alpha"]["visit_order"]
                                                if str(v.get("label")) not in ("point 4", "at point 4")]
    c.pending_replan = None
    c.keep_reassigned()
    assert "point 4" in [v.get("label") for v in c.plan["robots"]["Alpha"]["visit_order"]]
    assert c.pending_replan is not None


def test_deadline_stretches_to_the_newest_plan_only_when_keeping_going():
    mm, plan = production_plan()
    c = coord(mm, plan, on_problem="stop")
    assert c.deadline(100.0) == pytest.approx(c.t_run + 100.0)
    c = coord(mm, plan)
    c.plan_start = c.t_run + 90.0
    c.plan = dict(c.plan, summary={"makespan_s": 60.0})
    assert c.deadline(100.0) == pytest.approx(c.t_run + 90.0 + 60.0 + RP.Coordinator.DEADLINE_TAIL_S)
    c.plan_start = c.t_run + 1000.0
    assert c.deadline(100.0) == pytest.approx(c.t_run + 300.0)                 # never past 3 x the limit


def test_edge_guard_end_keep_going_maps_the_spot():
    import session_checks as SC
    mm, plan = production_plan()
    for pol, n_new in (("keep_going", 1), ("stop", 0)):
        c = coord(mm, plan, on_problem=pol)
        b = c.bots["Alpha"]
        b.pose = (60.0, 150.0, 90.0)
        b.ended = "limited:plywood edge"
        n0 = len(c.m["obstacles"])
        ob = c.edge_end_strip(b)
        assert len(c.m["obstacles"]) == n0 + n_new
        if n_new:
            assert ob["source"] == "edge_guard" and any("no-go strip" in s for s in c.said)


# ---------------------------------------------------------------------------------- skipped points (planner)

def test_planner_skips_an_unreachable_point_and_its_dwell_and_names_it():
    pts = {"1": [80.0, 120.0], "2": [200.0, 80.0], "3": [120.0, 50.0]}
    m, _ = PR.site_map((L, W), ["Alpha", "Bravo"], points=pts)
    m["obstacles"] = [{"id": "box", "shape": "circle", "center": [200.0, 80.0], "radius": 15.0, "source": "camera"}]
    prog = PR.program({"Alpha": ["1", "2", "3"], "Bravo": []})
    mm = MS.build(FM.normalize(m), prog)
    plan = MS.plan_synced(mm, over={"separation_cm": 8.0, "skip_unreachable": True}, t_limit_s=30.0)
    labs = labels(plan, "Alpha")
    assert "point 2" not in labs and "at point 2" not in labs                   # no dwell somewhere else
    assert "at point 1" in labs and "at point 3" in labs
    w = [x for x in plan["warnings"] if "skips task" in x]
    assert any("(point 2)" in x for x in w)
    c = coord(mm, plan)
    c.note_skips(plan)
    assert [q["point"] for q in c.skipped] == ["point 2"]
    assert any("point 2 cannot be reached" in s for s in c.said)


# --------------------------------------------------------------------------------- feel-out: camera trusted

def _feel(tmp, trust):
    from test_feel_table import fake_session, table
    m = table([0, 0, L, W + 8.0])                    # the cameras' outline 8 cm too far out on the far long side
    sp = fake_session(str(tmp), m, [0, 0, L, W])
    return FT.analyze(m, str(tmp), sp, trust=trust)


def test_feel_out_camera_trust_reports_8cm_keeps_the_outline_no_hold(tmp_path):
    res = _feel(tmp_path, "camera")
    sd = res["sides"]["y_max"]
    assert sd["verdict"] == "camera kept" and sd["move_cm"] == 0.0 and sd["felt_cm"] == pytest.approx(-8.0, abs=0.6)
    assert not res["hold"] and not res["adjusted"]
    assert any("camera's outline is kept" in w for w in res["words"])
    cal = FT.save_cal(str(tmp_path / "conf"), "396x168", "x", res)
    assert cal["outline_used"] == cal["outline_camera"] and cal["trust"] == "camera"
    # its trips on that side teach no reach (it would be off by the 8 cm not resolved)
    reach = json.load(open(os.path.join(str(tmp_path / "conf"), "feel_table", "reach.json")))
    n_all = sum(len(v) for r in reach.values() for v in r.values())
    assert n_all == 6                                                          # 8 trips, 2 on the far long side left out


def test_feel_out_sensor_trust_unchanged(tmp_path):
    res = _feel(tmp_path, "sensor")
    assert res["sides"]["y_max"]["verdict"] == "adjusted" and res["adjusted"]["y_max"] == pytest.approx(-8.0, abs=0.6)


def test_feel_out_camera_trust_never_holds_even_far_off(tmp_path):
    from test_feel_table import fake_session, table
    m = table([0, 0, L, W + 13.0])
    sp = fake_session(str(tmp_path), m, [0, 0, L, W])
    assert FT.analyze(m, str(tmp_path), sp, trust="sensor")["hold"]
    assert not FT.analyze(m, str(tmp_path), sp, trust="camera")["hold"]


# ------------------------------------------------------------------------------------------- confidence numbers

def _cc(fit=None):
    mm, plan = production_plan()
    c = coord(mm, plan)
    c._cam_info = {"cal": {}, "moved": {}, "fit_t": 1e18, "ev_t": 1e18, "ev_pos": 0, "fit": fit}
    return c


FIT = {"rms_px": {"overhead": {"tags": 2.0}, "side": {"tags": 1.0}}, "sticker_sd_cm": {"1": 0.3, "2": 0.5}}


def test_camera_plus_minus_from_the_fit_two_cameras_tighter_than_one():
    c = _cc(FIT)
    one = c.cam_unc({"sigma": 0.3, "cams": ["overhead"], "apart": 0.0, "ppc": 4.0, "tagged": True})
    # overhead: max(0.4 sticker median, 2.0 px / 4 px per cm = 0.5) = 0.5; sqrt(0.3^2 + 0.5^2) = 0.583; x 2
    assert one == pytest.approx(1.17, abs=0.01)
    two = c.cam_unc({"sigma": 0.3, "cams": ["overhead", "side"], "apart": 0.2, "ppc": 4.0, "tagged": True})
    assert two < one
    far = c.cam_unc({"sigma": 0.3, "cams": ["overhead", "side"], "apart": 3.0, "ppc": 4.0, "tagged": True})
    assert far == pytest.approx(3.0, abs=0.01)                                # cameras 3 cm apart: at least half each


def test_a_bumped_camera_counts_in():
    c = _cc(FIT)
    f = {"sigma": 0.3, "cams": ["side"], "apart": 0.0, "ppc": 4.0, "tagged": True}
    before = c.cam_unc(f)
    c._cam_info["moved"]["side"] = 4.0
    assert c.cam_unc(f) == pytest.approx(before + 4.0, abs=0.01)             # the shift once, as a bias
    two = {"sigma": 0.3, "cams": ["overhead", "side"], "apart": 0.0, "ppc": 4.0, "tagged": True}
    c._cam_info["moved"].clear()
    b2 = c.cam_unc(two)
    c._cam_info["moved"]["side"] = 4.0
    assert c.cam_unc(two) == pytest.approx(b2 + 2.0, abs=0.01)                # half with the other camera in the fix


def test_a_bump_noticed_after_a_point_widens_that_point(tmp_path):
    # bump run, 3 Nov: the bridge noticed the move a few seconds after Bravo stood at point 4 (truth 3.1 cm off, said
    # +-1.2): a point stood at in the 12 s before the report gets the shift too
    c = _cc(FIT)
    c.outdir = str(tmp_path)
    b = c.bots["Bravo"]
    b.pose = (300.0, 60.0, 0.0)
    t0 = RP.now() - 4.0
    for k in range(30):
        c.note_fix(b, {"x": 300.0, "y": 60.0, "sigma": 0.3, "src": "tag_bridge:side", "ppc": 4.0}, "side")
        b.fix_log[-1]["t"] = t0 + 0.1 * k
    c.record_point(b, "point 4", t0, t0 + 3.0)
    before = c.points[-1]["plus_minus_cm"]
    c.bump_back("side", 4.1)
    q = c.points[-1]
    assert q["plus_minus_cm"] == pytest.approx(before + 4.1, abs=0.06)
    assert "cam side reported moved" in q["words"] and q["words"].startswith("point 4 (Bravo): \u00b1%.1f" % q["plus_minus_cm"])
    c.bump_back("overhead", 4.1)                                                # another camera: not this point
    assert c.points[-1]["plus_minus_cm"] == q["plus_minus_cm"]


def test_odometry_plus_minus_grows_0_07_per_cm_from_the_last_fix():
    c = _cc(FIT)
    b = c.bots["Alpha"]
    t = RP.now()
    c.note_fix(b, {"x": 100.0, "y": 50.0, "sigma": 0.3, "src": "tag_bridge:overhead+side", "apart": 0.1, "ppc": 4.0},
               "overhead+side")
    u0 = b.last_fix_unc
    c.odometer(b, t, 100.0, 50.0, "10")
    c.odometer(b, t + 0.1, 100.0, 50.0, "11")              # it took a fix here
    for k in range(1, 81):                                  # then 80 cm with no fix taken
        c.odometer(b, t + 0.1 + 0.1 * k, 100.0 + k, 50.0, "11")
    b.pose = (180.0, 50.0, 0.0)
    b.fix_log[-1]["t"] -= 10.0                              # the camera's last fix is old: odometry only
    cn = c.conf_now(b)
    assert cn["source"] == "odometry only" and cn["since_fix_cm"] == pytest.approx(80.0, abs=0.01)
    assert cn["plus_minus_cm"] == pytest.approx(u0 + 0.07 * 80.0, abs=0.02)


def test_point_record_two_cameras_and_odometry_words(tmp_path):
    c = _cc(FIT)
    c.outdir = str(tmp_path)
    a = c.bots["Alpha"]
    t0 = RP.now()
    a.pose = (80.0, 120.0, 90.0)
    for k in range(30):
        f = {"x": 80.0 + 0.05 * (k % 3), "y": 120.0, "sigma": 0.3, "src": "tag_bridge:overhead+side", "apart": 0.2,
             "ppc": 4.0}
        c.note_fix(a, f, "overhead+side")
        a.fix_log[-1]["t"] = t0 + 0.1 * k
    c.record_point(a, "point 7", t0, t0 + 3.0)
    q = c.points[-1]
    assert q["source"] == "two cameras" and q["n_fixes"] == 30 and q["plus_minus_cm"] < 1.5
    assert q["words"].startswith("point 7 (Alpha): ±") and q["words"].endswith("two cameras")
    # no camera during the stand: the robot's own pose and the odometry's +-
    b = c.bots["Bravo"]
    b.pose = (300.0, 60.0, 0.0)
    b.last_fix_unc, b.odo, b.acc_odo, b.acc_t = 1.0, 80.0, 0.0, t0 - 20.0
    c.record_point(b, "point 12", t0, t0 + 3.0)
    q = c.points[-1]
    assert q["source"] == "odometry only" and q["plus_minus_cm"] == pytest.approx(1.0 + 5.6, abs=0.05)
    assert "odometry for 80 cm since the last camera fix" in q["words"]
    rows = open(os.path.join(str(tmp_path), "points.csv")).read().splitlines()
    assert rows[0].startswith("robot,point,t_run,x,y,plus_minus_cm,source") and len(rows) == 3


def test_dwell_steps_make_points_and_done_labels():
    c = _cc(FIT)
    c.outdir = "/tmp"
    c.point_csv = lambda rec: None
    a = c.bots["Alpha"]
    a.pose = (80.0, 120.0, 90.0)
    t = RP.now()
    c.point_step(a, {"type": "wait", "label": "at point 1", "t_start": 0, "t_end": 3}, "started", t)
    c.point_step(a, {"type": "wait", "label": "at point 1", "t_start": 0, "t_end": 3}, "done", t + 3.0)
    assert "at point 1" in c.done_labels and c.points[-1]["point"] == "point 1"


# ------------------------------------------------------------------------------------------- Mission Control

def test_mission_control_tick_box_overrides_the_site():
    import mission_control as MC
    s = MC.Session.__new__(MC.Session) if hasattr(MC, "Session") else None
    if s is None:
        pytest.skip("no Session class")
    s.settings = {"run": dict(MC.RUN_DEFAULTS)}
    s.site = {"on_problem": "keep_going"}
    assert s.on_problem() == "keep_going"
    s.settings["run"]["on_problem"] = "stop"
    assert s.on_problem() == "stop"
    s.site = None
    s.settings["run"]["on_problem"] = ""
    assert s.on_problem() == "keep_going"                                       # (e08, Q7) no site: keep going


def test_feel_job_keep_going_starts_production_after_a_failed_feel_out():
    import feel_job as FJ
    j = FJ.FeelJob.__new__(FJ.FeelJob)
    import threading
    j.lock, j.words, j.conn = threading.Lock(), [], "sim-cameras"
    started = []
    j.s = types.SimpleNamespace(on_problem=lambda: "keep_going", site={"feel_out_trust": "camera"}, stop_asked=False)
    j._chain = lambda conn: started.append(conn)
    j._failed("feel-out run", 1)
    assert started == ["sim-cameras"] and "KEEP GOING" in j.err
    started.clear()
    j.s.stop_asked = True                                                       # Stop is still a stop
    j._failed("feel-out run", -2)
    assert not started
    j.s = types.SimpleNamespace(on_problem=lambda: "stop", site={}, stop_asked=False)
    j._failed("feel-out run", 1)
    assert not started and "not started" in j.err
