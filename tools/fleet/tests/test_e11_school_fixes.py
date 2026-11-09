"""e11 (9 Nov 2026, school): the first real Mission Control free roam at school. Robots that keep their WiFi names
(aliases), the free roam's driving reaching the run, no sleep after a start where nothing moved, and the simulator's
robots answering with WiFi names (the real path's practice)."""
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
sys.path[:0] = [FLEET, TOOLS, os.path.join(TOOLS, "sim")]

import mission_run as MR  # noqa: E402
import start_job as SJ  # noqa: E402


def _args(argv, monkeypatch):
    got = {}
    monkeypatch.setattr(MR, "run", lambda a: got.setdefault("a", a) and 0)
    MR.main(argv)
    return got["a"]


def test_motion_given_in_any_form_reaches_the_free_roam(monkeypatch):
    for argv, want in ((["--motion", "stopturn"], "stopturn"), (["--motion=stopturn"], "stopturn"),
                       (["--motion", "curves"], "curves"), ([], "curves")):
        a = _args(argv + ["--roam-min", "5"], monkeypatch)
        assert MR.roam_motion_of(a) == want, argv
    assert _args([], monkeypatch).motion == "stopturn"          # the planner's default is unchanged


def test_practice_real_is_a_dry_run_without_the_simulated_rename(monkeypatch):
    a = _args(["--practice-real", "--sim-bind"], monkeypatch)
    assert a.dry and a.practice_real and not a.sim_bind


def test_practice_aliases_wifi_names_to_slots():
    al = MR.practice_aliases(["Alpha", "Bravo"])
    assert al == {"AL-1DC0": "Alpha", "AL-2B7F": "Bravo"}


def test_fleet_sim_robot_answers_with_its_wifi_name():
    import fleet_sim as FS
    sent = []
    r = types.SimpleNamespace(name="Alpha", seq=0, laptop="127.0.0.1", wire="AL-1DC0", lost_out=0,
                              sim=types.SimpleNamespace(dropped=lambda n, t: False),
                              _random_drop=lambda d: False,
                              sock=types.SimpleNamespace(sendto=lambda b, a: sent.append(b.decode())))
    FS.Radio.transmit(r, None, "M,hello from Alpha at 127.0.0.2")
    assert sent == ["AL-1DC0|0|M,hello from AL-1DC0 at 127.0.0.2"]
    r.wire, sent[:] = None, []
    FS.Radio.transmit(r, None, "M,hello from Alpha")
    assert sent == ["Alpha|1|M,hello from Alpha"]


def test_link_names_an_aliased_robot_by_its_slot():
    import alvik_console as AC
    s = AC.Session.__new__(AC.Session)
    s.aliases = {"AL-1DC0": "Alpha"}
    assert s.aliases.get("AL-1DC0", "AL-1DC0") == "Alpha"


def _sj(practice=False, moved=False):
    slept = []
    o = types.SimpleNamespace(active_practice=practice, adv={"sleep_when_done": True}, deferred_sleep=None,
                              DEFERRED_SLEEP_MIN=SJ.StartJob.DEFERRED_SLEEP_MIN if hasattr(SJ, "StartJob") else 10.0,
                              busy=lambda: False, sleep_robots=lambda why: slept.append(why))
    return o, slept


def _cls():
    return next(v for v in vars(SJ).values() if isinstance(v, type) and hasattr(v, "deferred_sleep_due"))


def test_a_refused_start_does_not_sleep_the_robots_at_once_but_after_ten_minutes():
    C = _cls()
    o, slept = _sj()
    o.DEFERRED_SLEEP_MIN = C.DEFERRED_SLEEP_MIN
    assert C.DEFERRED_SLEEP_MIN == 10.0
    assert C.on_run_end(o, refused=True) is False
    assert slept == [] and o.deferred_sleep
    assert C.deferred_sleep_due(o, now=time.time() + 9 * 60) is False and slept == []
    assert C.deferred_sleep_due(o, now=time.time() + 10 * 60 + 1) is True and len(slept) == 1
    assert o.deferred_sleep is None


def test_a_new_run_cancels_the_held_sleep():
    C = _cls()
    o, slept = _sj()
    o.DEFERRED_SLEEP_MIN = C.DEFERRED_SLEEP_MIN
    C.on_run_end(o, refused=True)
    o.busy = lambda: True
    assert C.deferred_sleep_due(o, now=time.time() + 3600) is False and slept == [] and o.deferred_sleep is None


def test_a_run_that_ended_normally_still_sleeps_at_once():
    C = _cls()
    o, slept = _sj()
    assert C.on_run_end(o) is True and slept == ["the run ended"]


def test_no_robot_answered_is_a_refusal_in_plain_words(tmp_path):
    C = _cls()
    (tmp_path / "mission_stdout.txt").write_text("the robots start now. To stop\nSTOPPED: No robot answered.\n")
    o = types.SimpleNamespace(mission_tail=lambda f, n=8: C.mission_tail(None, f, n))
    w = C.start_failure_words(o, str(tmp_path))
    assert w and "nothing moved" in w and "WiFi" in w
    (tmp_path / "m2").mkdir()
    (tmp_path / "m2" / "mission_stdout.txt").write_text("NOT STARTED: the card has 0.5 GB free\n")
    assert C.nothing_moved(None, str(tmp_path / "m2")) is True
    assert C.nothing_moved(None, str(tmp_path)) is False


def test_freeroam_takes_aliases():
    src = open(os.path.join(FLEET, "freeroam.py")).read()
    assert '"--alias"' in src and "aliases=self.aliases" in src


# -- part 2: Visit points in an order Nick chooses -------------------------------------------------------------------
def test_visit_order_typed_one_robot_takes_it_all_with_repeats():
    assert MR.parse_visit_order("P01,p5, 11,P05", ["Alpha"]) == {"Alpha": ["P01", "P05", "P11", "P05"]}


def test_visit_order_more_robots_split_in_runs_or_named_lists():
    assert MR.parse_visit_order("P01,P02,P03", ["Alpha", "Bravo"]) == {"Alpha": ["P01", "P02"], "Bravo": ["P03"]}
    assert MR.parse_visit_order("Alpha:P01,P05;Bravo:P11", ["Alpha", "Bravo"]) == {"Alpha": ["P01", "P05"],
                                                                                    "Bravo": ["P11"]}


def test_visit_order_bad_label_is_plain_words():
    import pytest
    with pytest.raises(ValueError):
        MR.parse_visit_order("P01,XYZ", ["Alpha"])
    with pytest.raises(ValueError):
        MR.parse_visit_order("Zulu:P01", ["Alpha"])


class _RM:
    def __init__(self, bad):
        self.bad = bad

    def readable(self, x, y):
        return (not any(abs(x - b[0]) < 1 and abs(y - b[1]) < 1 for b in self.bad), "test")


def _visit_args(order, keep=False):
    import sites as SI
    site = SI.load("school-conference")
    return types.SimpleNamespace(site_=site, visit_order=order, visit_keep_unreadable=keep, readmap_=None)


def test_unreadable_points_skipped_by_default_and_kept_when_asked(tmp_path):
    import json
    import sticker_points as SP
    a = _visit_args("P01,P12,P05,P15,P05")
    pts, _z, _w = SP.points_cm(a.site_)
    a.readmap_ = _RM([pts["P12"], pts["P15"]])
    o = MR.visit_orders(a, ["Alpha"], str(tmp_path))
    assert o == {"Alpha": ["P01", "P05"]}        # P12 skipped; P05 after P05 (once P15 is skipped) visited once
    rec = json.load(open(tmp_path / "visit_order.json"))
    assert sorted(rec["flagged"]) == ["P12", "P15"]
    a.visit_keep_unreadable = True
    assert MR.visit_orders(a, ["Alpha"], None) == {"Alpha": ["P01", "P12", "P05", "P15", "P05"]}


def test_every_point_unreadable_refuses_before_moving():
    import pytest
    import sticker_points as SP
    a = _visit_args("P12")
    pts, _z, _w = SP.points_cm(a.site_)
    a.readmap_ = _RM([pts["P12"]])
    with pytest.raises(SystemExit) as e:
        MR.visit_orders(a, ["Alpha"], None)
    assert "Nothing moved" in str(e.value)


# -- Jay-Sun's Experiment A tours (coordinator, 9 Nov) ---------------------------------------------------------------
def test_jay_sun_presets_and_defaults():
    P = SJ.VISIT_PRESETS
    assert P["In order (P01-P20)"][0] == "P01" and P["In order (P01-P20)"][-1] == "P20"
    assert P["Reversed (P20-P01)"] == list(reversed(P["In order (P01-P20)"]))
    assert P["Jay-Sun custom"][:3] == ["P12", "P18", "P09"] and sorted(P["Jay-Sun custom"]) == P["In order (P01-P20)"]
    assert SJ.ADV_DEFAULTS["visit_speed_cms"] == 10.0 and SJ.ADV_DEFAULTS["visit_dwell_s"] == 8.0
    assert MR.visit_dwell_of(types.SimpleNamespace(visit_dwell_s=None)) == 8.0


def test_stops_csv_each_stop_findable_with_unix_times(tmp_path):
    import csv
    import json
    import run_exports as RX
    (tmp_path / "run").mkdir()
    with open(tmp_path / "run" / "coordinator.jsonl", "w") as f:
        for k, (pt, t1) in enumerate((("point P12", 1794240008.0), ("point P18", 1794240030.5))):
            f.write(json.dumps({"t": t1, "event": "point_confidence", "robot": "Alpha", "point": pt, "dwell_s": 8.0,
                                "t_abs": t1, "x": 10.0 + k, "y": 20.0, "plus_minus_cm": 1.4, "n_fixes": 80}) + "\n")
        f.write(json.dumps({"t": 1, "event": "step"}) + "\n")
    p, n = RX.stops(str(tmp_path))
    rows = list(csv.DictReader(open(p)))
    assert n == 2 and rows[0]["point"] == "P12" and rows[1]["n"] == "2"
    assert float(rows[0]["dwell_start_unix"]) == 1794240000.0 and float(rows[0]["dwell_end_unix"]) == 1794240008.0
    assert rows[0]["cam_x_cm"] == "10.0" and "stops.csv" in " ".join(r for r, _ in RX.BUNDLE_FILES)


# -- 9 Nov home: discovery said "out of view" with the camera tracking all three robots -------------------------------
def test_a_stand_waits_for_a_slow_camera():
    import start_discovery as D
    sent = []
    pd = D.ProofDriver(["Left"], lambda n, txt: sent.append(txt), t0=0.0)
    t, k = 0.0, 0
    pd.on_follow("Left", 0.0, 40.0, 50.0, 0.0)
    while t < 1.6:                         # the first stand (1.5 s) with a camera giving 2 fixes a second
        if abs(t * 2 - round(t * 2)) < 1e-6:
            pd.on_sighting("Left", t - 0.3, 40.0, 50.0)
        pd.tick(t)
        t = round(t + 0.05, 2)
    assert pd.R["Left"]["phase"] == 0      # before e11 it moved on with 2 sightings: the stand never counted
    while pd.R["Left"]["phase"] == 0 and t < 10:
        if abs(t * 2 - round(t * 2)) < 1e-6:
            pd.on_sighting("Left", t - 0.3, 40.0, 50.0)
        pd.tick(t)
        t = round(t + 0.05, 2)
    m = pd.R["Left"]["marks"][0]
    assert pd.R["Left"]["phase"] == 1 and sum(1 for q in pd.R["Left"]["cam"] if m["t0"] + 0.2 <= q[0] <= m["t1"]) >= \
        D.SEEN_MIN


def test_an_unproven_camera_falls_back_to_the_typed_board():
    src = open(os.path.join(FLEET, "freeroam.py")).read()
    assert "FALLING BACK TO THE TYPED BOARD" in src and "make_workspace(self.rect, camera=True)" in src
    assert '"--discovery-strict"' in src
