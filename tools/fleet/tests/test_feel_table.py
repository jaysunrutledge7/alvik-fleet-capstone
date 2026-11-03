"""Feel out the table (Nick, 2 Nov 2026 18:33): spots, analysis, decisions, storage, Mission Control's job, and one
simulator run (tools/fleet/feel_table.py, feel_job.py; tools/sim edge-miss, true floor and drop)."""
import json
import math
import os
import subprocess
import sys

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
import production as PR      # noqa: E402

L, W = 396.2, 167.6


def table(outline=None):
    m, _ = PR.site_map((L, W), ["Alpha", "Bravo"], outline=outline)
    return m


# ------------------------------------------------------------------------------------------------- synthetic sessions

def fake_session(tmp, m, true_rect, reach=10.8, miss=(), t0=1000.0):
    """A run_plan session as the feel-out would leave it: each crawl stops where the bottom zone (reach cm ahead)
    meets the TRUE edge, or at its planned end when missed; the camera sees the true place (+ a small lag offset)."""
    sp = FT.spots(m)
    ev, track = [], []
    t = t0
    for s in sp:
        side = s["side"]
        ox, oy = FT.OUTWARD[side]
        th = math.degrees(math.atan2(oy, ox))
        true_line = {"x_min": true_rect[0], "y_min": true_rect[1], "x_max": true_rect[2], "y_max": true_rect[3]}[side]
        if s["id"] in miss:
            x, y = s["end"]
            ev.append({"t": t, "event": "labelled_step", "robot": s["robot"], "label": FT.label(s), "phase": "probe",
                       "state": "done", "pose": [x, y, th], "cam": [x, y, th], "info": ""})
        else:
            if side.startswith("x"):
                x, y = true_line - ox * reach, s["along"]
            else:
                x, y = s["along"], true_line - oy * reach
            info = "plywood edge at %.1f %.1f %.1f bottom" % (x, y, th)
            ev.append({"t": t, "event": "step_report", "robot": s["robot"], "id": 1, "state": "failed", "x": x + 0.3,
                       "y": y + 0.3, "th": th, "info": info})
            ev.append({"t": t + 0.001, "event": "labelled_step", "robot": s["robot"], "label": FT.label(s),
                       "phase": "probe", "state": "failed", "pose": [x, y, th], "cam": [x, y, th], "info": info})
            for k in range(1, 5):
                track.append((t + 0.2 * k, s["robot"], x, y, th))
        t += 20.0
    os.makedirs(tmp, exist_ok=True)
    with open(os.path.join(tmp, "coordinator.jsonl"), "w") as f:
        for e in ev:
            f.write(json.dumps(e) + "\n")
    with open(os.path.join(tmp, "cam_track.csv"), "w") as f:
        f.write("t,robot,x_cm,y_cm,heading_deg\n")
        for r in track:
            f.write("%.3f,%s,%.2f,%.2f,%.1f\n" % r)
    return sp


# ------------------------------------------------------------------------------------------------------------ spots

def test_spots_eight_split_by_halves_on_every_side():
    m = table()
    sp = FT.spots(m)
    assert len(sp) == 8
    assert {s["side"] for s in sp} == set(FM.SIDES)
    assert all(len([s for s in sp if s["side"] == sd]) == 2 for sd in FM.SIDES)
    for s in sp:
        # each robot stays in its own half (neither crosses the other's path)
        if s["robot"] == "Alpha":
            assert s["start"][0] < L / 2 and s["end"][0] < L / 2
        else:
            assert s["start"][0] > L / 2 and s["end"][0] > L / 2
    assert len([s for s in sp if s["robot"] == "Alpha"]) == 4
    # a crawl starts 22 cm in and is planned to end 3 cm inside the cameras' line
    s = next(s for s in sp if s["side"] == "y_max")
    assert s["start"][1] == pytest.approx(W - FT.START_IN_CM) and s["end"][1] == pytest.approx(W - FT.END_IN_CM)


def test_plan_tasks_plans_and_keeps_the_net():
    import planner as P
    import missions as MS
    fm, sp = FT.plan_tasks(table())
    assert {n: len(v["visit"]) for n, v in fm["tasks"].items()} == {"Alpha": 4, "Bravo": 4}
    over = {}
    MS.gap_lock_planner(over, n_robots=2)
    plan = P.make_plan(fm, over=over)
    assert plan["summary"]["makespan_s"] < 200
    probes = [st for n in plan["robots"] for st in plan["robots"][n]["steps"] if st["phase"] == "probe"]
    assert probes and all(abs(st.get("speed", 3.0) - 3.0) < 1e-6 for st in probes if st["type"] == "drive")


# --------------------------------------------------------------------------------------------------------- analysis

def test_camera_outline_right_agrees(tmp_path):
    m = table()
    sp = fake_session(str(tmp_path), m, [0, 0, L, W])
    res = FT.analyze(m, str(tmp_path), sp)
    assert all(res["sides"][k]["verdict"] == "agree" for k in FM.SIDES)
    assert not res["adjusted"] and not res["hold"]
    assert res["reach"]["bottom"]["from"] == "learned" and res["reach"]["bottom"]["cm"] == pytest.approx(10.8, abs=0.5)


@pytest.mark.parametrize("err", [3.0, 8.0])
def test_outline_wrong_on_one_side_is_moved_and_said(tmp_path, err):
    # the cameras' outline err cm too far out (+) or in (-) on the far long side
    m = table([0, 0, L, W + err])
    sp = fake_session(str(tmp_path), m, [0, 0, L, W])
    res = FT.analyze(m, str(tmp_path), sp)
    sd = res["sides"]["y_max"]
    assert sd["verdict"] == "adjusted"
    assert sd["move_cm"] == pytest.approx(-err, abs=0.6)
    assert any(w.startswith("WARNING: far long side") for w in res["words"])
    for k in ("x_min", "x_max", "y_min"):
        assert res["sides"][k]["verdict"] == "agree"
    cal = {"outline_used": FT.adjusted_rect(res["outline"], res["adjusted"])}
    assert cal["outline_used"][3] == pytest.approx(W, abs=0.6)


def test_outline_too_small_keeps_the_camera_line(tmp_path):
    # the table 4 cm bigger than the cameras' outline on the far long side: the look-ahead feels it past the line, but
    # the outline is never moved OUT (the robots never drove past the cameras' line: the risk review)
    m = table([0, 0, L, W - 4.0])
    sp = fake_session(str(tmp_path), m, [0, 0, L, W])
    res = FT.analyze(m, str(tmp_path), sp)
    sd = res["sides"]["y_max"]
    assert sd["verdict"] == "further out" and sd["move_cm"] == 0.0 and sd["offset_cm"] == pytest.approx(4.0, abs=0.6)
    assert not res["adjusted"] and any("kept" in w for w in res["words"])


def _raw(tmp, rows):
    os.makedirs(tmp, exist_ok=True)
    cols = ["robot", "t_recv", "v_cms", "w_dps", "dbot_cm", "lineL", "lineC", "lineR"]
    with open(os.path.join(tmp, "raw.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r[c]) for c in cols) + "\n")
    with open(os.path.join(tmp, "coordinator.jsonl"), "w") as f:
        for n in ("Alpha", "Bravo"):
            f.write(json.dumps({"t": 99.0, "event": "labelled_step", "robot": n, "label": "surface check",
                                "state": "started"}) + "\n")
            f.write(json.dumps({"t": 104.0, "event": "labelled_step", "robot": n, "label": "surface check",
                                "state": "done"}) + "\n")


def test_surface_check_passes_a_quiet_tabletop_and_refuses_a_glossy_one(tmp_path):
    import random
    rng = random.Random(1)
    ok = [{"robot": n, "t_recv": 100 + 0.1 * k, "v_cms": 0, "w_dps": 0, "dbot_cm": round(7.9 + rng.gauss(0, 0.2), 2),
           "lineL": int(150 + rng.gauss(0, 5)), "lineC": int(145 + rng.gauss(0, 5)), "lineR": int(155 + rng.gauss(0, 5))}
          for n in ("Alpha", "Bravo") for k in range(30)]
    _raw(str(tmp_path / "a"), ok)
    r = FT.surface_check(str(tmp_path / "a"))
    assert r["ok"] and r["robots"]["Alpha"]["bottom_margin_cm"] > 2.0 and r["robots"]["Alpha"]["line_margin"] > 100
    # a glossy top: Bravo's look-ahead reads long now and then, and its line sensors flicker
    bad = [dict(x) for x in ok]
    for k, x in enumerate(bad):
        if x["robot"] == "Bravo" and k % 7 == 0:
            x["dbot_cm"], x["lineC"] = 10.4, 260
    _raw(str(tmp_path / "b"), bad)
    r = FT.surface_check(str(tmp_path / "b"))
    assert not r["ok"] and r["robots"]["Alpha"]["ok"] and not r["robots"]["Bravo"]["ok"]
    assert any("NOT run" in w for w in r["words"])
    # saved: the cameras' outline kept, later runs skip it until asked again
    conf = str(tmp_path / "conf")
    cal = FT.surface_cal(conf, "396x168", "s1", [0, 0, L, W], r)
    assert cal["outline_used"] == [0, 0, L, W] and cal["skipped_for_surface"]
    assert not FT.needed(conf, "396x168", "s1")[0]


def test_far_off_outline_holds_the_run_and_is_not_moved(tmp_path):
    m = table([0, 0, L, W + 13.0])
    sp = fake_session(str(tmp_path), m, [0, 0, L, W])
    res = FT.analyze(m, str(tmp_path), sp)
    assert res["sides"]["y_max"]["verdict"] == "suspect" and res["hold"]
    assert "y_max" not in res["adjusted"]
    assert any(w.startswith("STOP") for w in res["words"])


def test_a_missed_drop_is_named_and_the_side_still_answers(tmp_path):
    m = table()
    sp = fake_session(str(tmp_path), m, [0, 0, L, W], miss=("S7",))
    res = FT.analyze(m, str(tmp_path), sp)
    s7 = next(f for f in res["feels"] if f["id"] == "S7")
    assert s7["kind"] == "nothing"
    assert res["misses"] and res["misses"][0]["spot"] == "S7"
    assert res["sides"][s7["side"]]["verdict"] == "agree"     # the other feel on that side answers it


def test_reach_pools_old_data(tmp_path):
    m = table()
    sp = fake_session(str(tmp_path / "s"), m, [0, 0, L, W], reach=10.0)
    conf = str(tmp_path / "conf")
    os.makedirs(os.path.join(conf, "feel_table"))
    with open(os.path.join(conf, "feel_table", "reach.json"), "w") as f:
        json.dump({"Alpha": {"bottom": [9.0] * 20}}, f)
    res = FT.analyze(m, str(tmp_path / "s"), sp, conf_dir=conf)
    assert res["reach"]["bottom"]["n_old"] == 20 and res["reach"]["bottom"]["cm"] == pytest.approx(9.0)


# ------------------------------------------------------------------------------------------------------- storing

def test_saved_then_skipped_until_the_camera_setup_changes(tmp_path):
    m = table([0, 0, L, W + 8.0])
    sp = fake_session(str(tmp_path / "s"), m, [0, 0, L, W])
    conf, key = str(tmp_path / "conf"), FT.table_key(table())
    need, cal, why = FT.needed(conf, key, "setupA")
    assert need and cal is None and "first run" in why
    res, cal = FT.finish(m, str(tmp_path / "s"), sp, conf, key, "setupA", site="school-conference")
    assert os.path.exists(FT.cal_path(conf, key))
    assert cal["outline_used"][3] == pytest.approx(W, abs=0.6) and cal["outline_camera"][3] == pytest.approx(W + 8)
    need, cal2, why = FT.needed(conf, key, "setupA")
    assert not need and "skipped" in why
    assert FT.needed(conf, key, "setupB")[0]                # the corners clicked again
    assert FT.needed(conf, key, "setupA", force=True)[0]    # Nick's "Feel out again"
    with open(os.path.join(conf, "feel_table", "reach.json")) as f:
        assert len(json.load(f)["Alpha"]["bottom"]) == 4
    # the production map on the felt outline: homes at its middle, points kept off its edges
    m2, words = FT.apply_to_map(table(), cal)
    assert m2["floor"]["y_max"] == pytest.approx(W, abs=0.6)


def test_setup_id_follows_the_corner_clicks(tmp_path):
    d = tmp_path / "tagcal"
    d.mkdir()
    obs = {"cameras": {"overhead": {"size": [1280, 960]}}, "corners": {"overhead": {"1": [10.2, 20.1]}}}
    (d / "obs.json").write_text(json.dumps(obs))
    a = FT.setup_id(str(d))
    obs["tag_groups"] = [1, 2, 3]                         # a refit's new tag reads: the same setup
    (d / "obs.json").write_text(json.dumps(obs))
    assert FT.setup_id(str(d)) == a
    obs["corners"]["overhead"]["1"] = [14.0, 20.1]        # a corner clicked again
    (d / "obs.json").write_text(json.dumps(obs))
    assert FT.setup_id(str(d)) != a
    assert FT.setup_id(str(tmp_path / "none")) is None


def test_site_map_outline_moves_homes_and_points():
    m, words = PR.site_map(None, ["Alpha", "Bravo"], points={"1": [100, 2]}, outline=[0, 5, L, W])
    assert m["floor"]["y_min"] == 5
    assert m["spots"]["1"][1] == pytest.approx(5 + PR.EDGE_IN_CM)
    starts = {r["name"]: r["start"] for r in m["robots"]}
    assert starts["Alpha"][1] == pytest.approx((5 + W) / 2)


# ------------------------------------------------------------------------------------------------ the run's start

def test_mission_run_skips_the_feels_when_saved(tmp_path, monkeypatch):
    import mission_run as MR
    conf = str(tmp_path / "conf")
    m = table()
    sp = fake_session(str(tmp_path / "s"), m, [0, 0, L, W])
    FT.finish(m, str(tmp_path / "s"), sp, conf, FT.table_key(m), "simulator")
    args = MR.main.__globals__["argparse"].Namespace(
        feel_table="auto", dry=True, robots="Alpha,Bravo", map=None, table="396x168", feel_conf_dir=conf, site_=None,
        seed=1, timeout_s=60, sim_true_floor="", sim_arg=[], robot=[], no_frames=True)

    def boom(*a, **k):
        raise AssertionError("the feel-out ran although it was saved")
    monkeypatch.setattr(MR.subprocess, "call", boom)
    assert MR.feel_first(args, str(tmp_path / "out")) == 0
    assert args.feel_cal_ and args.feel_cal_["setup_id"] == "simulator"


# ---------------------------------------------------------------------------------------------- Mission Control

@pytest.fixture()
def mc_session(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("MIRLT_SITE", "school-conference")
    import mission_control as MC
    s = MC.Session(str(tmp_path / "mc"))
    return s


def test_mission_control_runs_the_feels_first_then_skips(mc_session, tmp_path, monkeypatch):
    s = mc_session
    assert s.feel is not None
    st = s.feel.status("sim-cameras")
    assert st["needed"] and "first run" in st["why"]
    started = []
    monkeypatch.setattr(s.feel, "start", lambda conn, chain=None, force=False: started.append((conn, chain)) or {})

    class R:
        def status(self):
            return {"kind": "feel"}

        def alive(self):
            return False
    s.run = None
    monkeypatch.setattr(s, "start_cam_run", lambda *a, **k: setattr(s, "run", R()))
    out = s.start_run({"kind": "mission", "conn": "sim-cameras"})
    assert started and started[0][0] == "sim-cameras" and started[0][1]["kind"] == "mission"
    assert "feel" in out
    # saved for this table: the next production run goes straight on
    m = s.feel.camera_map("sim-cameras")
    sp = fake_session(str(tmp_path / "s"), m, [0, 0, L, W])
    FT.finish(m, str(tmp_path / "s"), sp, s.feel.conf_dir("sim-cameras"), s.feel.key("sim-cameras"),
              s.feel.setup_id("sim-cameras"))
    started.clear()
    s.start_run({"kind": "mission", "conn": "sim-cameras"})
    assert not started
    assert s.feel.status("sim-cameras")["needed"] is False


def test_mission_control_feel_card_and_page(mc_session):
    st = mc_session.state()
    assert "feel" in st and st["feel"]["needed"]
    with open(os.path.join(FLEET, "mission_control.html")) as f:
        page = f.read()
    for k in ("function feelCard", "/api/feel/start", "Feel out the table before this run", "/api/feel/skip"):
        assert k in page


# ------------------------------------------------------------------------------------------------- the simulator

def _free(port, kind):
    import socket
    k = socket.socket(socket.AF_INET, kind)
    try:
        k.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        k.close()


def test_simulator_a_missed_drop_stops_on_the_planned_end(tmp_path):
    """run_plan's simulator (fixes from the simulator), the conference table's drop (74 cm), the cameras' outline
    right; Bravo's edge guard misses its first drop with BOTH cues: its crawl's planned end must keep it on the table,
    and the miss must be named."""
    import socket
    if not _free(5010, socket.SOCK_DGRAM):
        pytest.skip("port 5010 in use (another simulator run)")
    import planner as P
    m = table()
    fm, sp = FT.plan_tasks(m)
    FM.save(fm, str(tmp_path / "fm.json"))
    P.save_plan(P.make_plan(fm), str(tmp_path / "fp.json"))
    cmd = [sys.executable, os.path.join(FLEET, "run_plan.py"), "--map", str(tmp_path / "fm.json"), "--plan",
           str(tmp_path / "fp.json"), "--session", str(tmp_path / "run"), "--dry-run", "--yes", "--border-cm", "0",
           "--sim-arg=--drop-cm=74", "--sim-arg=--edge-miss=Bravo:1:both", "--timeout-s", "300"]
    for c in ("set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11"):
        cmd += ["--send", c]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=400, cwd=REPO)
    assert "Result: done" in r.stdout, r.stdout[-2000:]
    rep = json.load(open(str(tmp_path / "run" / "sim" / "sim_report.json")))
    assert rep["hung"] == []
    assert rep["edge_misses"]["Bravo"]                              # the miss happened...
    assert min(rep["min_com_open_edge_cm"].values()) > 0.5        # ...and every centre of mass stayed on the table
    res = FT.analyze(fm, str(tmp_path / "run"), sp)
    assert res["misses"] and res["misses"][0]["robot"] == "Bravo"  # ...and it was named
    assert all(res["sides"][k]["verdict"] == "agree" for k in FM.SIDES)


def test_progress_counts_a_crawl_once_at_its_end(tmp_path):
    # a crawl is cut into drive pieces; the first piece's "done" is not the crawl done (Mission Control showed it so)
    m = table()
    sp = FT.spots(m)
    s = next(x for x in sp if x["robot"] == "Alpha")
    ev = [{"t": 1, "event": "labelled_step", "robot": "Alpha", "label": FT.label(s), "phase": "probe",
           "state": "started", "to": [0, 0, 0]},
          {"t": 2, "event": "labelled_step", "robot": "Alpha", "label": FT.label(s), "phase": "probe",
           "state": "done", "to": [s["start"][0] + 1, s["start"][1], 0]}]
    with open(tmp_path / "coordinator.jsonl", "w") as f:
        f.write("\n".join(json.dumps(e) for e in ev) + "\n")
    assert FT.progress(str(tmp_path), sp)["Alpha"]["done"] == 0
    ev.append({"t": 3, "event": "labelled_step", "robot": "Alpha", "label": FT.label(s), "phase": "probe",
               "state": "failed", "info": "plywood edge at 1 2 3 bottom", "to": s["end"]})
    with open(tmp_path / "coordinator.jsonl", "w") as f:
        f.write("\n".join(json.dumps(e) for e in ev) + "\n")
    p = FT.progress(str(tmp_path), sp)["Alpha"]
    assert p["done"] == 1 and p["felt"] == 1
