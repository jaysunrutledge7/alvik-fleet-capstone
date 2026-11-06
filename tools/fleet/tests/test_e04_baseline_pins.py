"""Build e04 (Nick's answers, 5 Nov 11:00): the first calibration of a robot starts from the defaults (uncalibrated
baseline), later ones from its saved values; robot <-> roof tag <-> slot pinned (the wiggle becomes a check); names in
the run only (no renaming on the robot at school); runs refuse a robot on older firmware; park spots P11, P01, P09;
the Pi's WiFi flashing helper. No robot, no Pi."""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import calib_run as CR                                     # noqa: E402
import sites as SI                                         # noqa: E402
from test_school_calibration import FakeEye, FakeRobot     # noqa: E402


# ------------------------------------------------------------------------------------------------- pins
def test_pinned_robot_does_the_short_check_and_a_moved_tag_is_rebound_loudly():
    eye = FakeEye({2: 0.0, 3: 40.0})
    a, b = FakeRobot("AL-1DC0", eye, [2]), FakeRobot("AL-2B7F", eye, [3])
    turns = []
    a.leg = (lambda f: (lambda kind, value, **kw: (turns.append(value), f(kind, value))[1]))(a.leg)
    notes, said = {}, []
    out = CR.bind_by_wiggle([a, b], eye, pause_s=0, log=said.append, expect={"AL-1DC0": 2}, check_deg=45, notes=notes)
    assert out == {"AL-1DC0": 2, "AL-2B7F": 3}
    assert turns == [45, -45]                                   # the pinned robot: the 45 deg check, not 90
    assert notes["AL-1DC0"].startswith("pin checked") and notes["AL-2B7F"].startswith("bound by the wiggle")
    # the tags swapped between the robots: re-bound, said loudly
    eye = FakeEye({2: 0.0, 3: 40.0})
    a, b = FakeRobot("AL-1DC0", eye, [3]), FakeRobot("AL-2B7F", eye, [2])
    notes, said = {}, []
    out = CR.bind_by_wiggle([a, b], eye, pause_s=0, log=said.append, expect={"AL-1DC0": 2, "AL-2B7F": 3},
                            check_deg=45, notes=notes)
    assert out == {"AL-1DC0": 3, "AL-2B7F": 2}
    assert all("PIN CHECK FAILED" in notes[n] for n in out) and any(s.startswith("!!!") for s in said)
    # pinned, but nothing turned: refused, and the words say the pin failed
    eye = FakeEye({2: 0.0})
    with pytest.raises(CR.BindError) as e:
        CR.bind_by_wiggle([FakeRobot("AL-1DC0", eye, [])], eye, pause_s=0, log=lambda s: None, expect={"AL-1DC0": 2},
                          check_deg=45)
    assert "PIN CHECK FAILED" in str(e.value)


def test_pins_file_merges_site_pins_and_records_tag_and_slot(tmp_path):
    p = str(tmp_path / CR.PINS_FILE)
    site = {"tags": {"pins": {"AL-9999": {"tag": 5, "slot": "Bravo"}}}}
    assert CR.load_pins(site, p) == {"AL-9999": {"tag": 5, "slot": "Bravo", "source": "site"}}
    CR.update_pins(p, {}, {"AL-1DC0": 2}, {"AL-1DC0": "Alpha"})
    pins = CR.load_pins(site, p)
    assert pins["AL-1DC0"]["tag"] == 2 and pins["AL-1DC0"]["slot"] == "Alpha" and pins["AL-9999"]["tag"] == 5
    CR.update_pins(p, pins, {"AL-1DC0": 2})                      # a check: the slot is kept
    assert CR.load_pins({}, p)["AL-1DC0"]["slot"] == "Alpha"


# ------------------------------------------------------------------------------------------------- firmware
def test_firmware_versions_compare_and_refusal_says_what_to_do():
    assert CR.firmware_ok("2026-11-05a", "2026-11-05a") and CR.firmware_ok("2026-11-06a", "2026-11-05a")
    assert not CR.firmware_ok("2026-10-29b", "2026-11-05a") and not CR.firmware_ok("", "2026-11-05a")
    assert CR.firmware_ok("2026-10-29b", "") and CR.firmware_ok("", None)   # switched off: no check
    w = CR.firmware_refusal("AL-1DC0", "2026-10-29b", "2026-11-05a")
    assert "2026-10-29b" in w and "2026-11-05a or newer" in w and "Reflash" in w
    m = CR.FW_RX.search("M,status AL-1DC0 (firmware 2026-11-05a, boot 1A2B): idle, selected e02, battery 90%")
    assert m and m.group(1) == "2026-11-05a"


# ------------------------------------------------------------------------------------------------- the site
def test_school_site_switches_and_spots():
    site = SI.load("school-conference")
    tg, cal = site["tags"], site["calibration"]
    assert tg["rename_to_slot"] is False and tg["pin"] is True and tg["unclaimed"] == "refuse"
    assert cal["min_firmware"] == "2026-11-05a" and cal["old_firmware_save"] == "off" and cal["baseline"] == "auto"
    assert tuple(cal["park_spots"]) == ("P09", "P01", "P06")     # Nick 5 Nov 21:38: the site's spots (they win)
    assert CR.PARK_SPOTS == ("P11", "P01", "P09")               # the code's fallback, used only without a site list
    assert CR.PARK_SPOTS_E03 == ("P11", "P05", "P01")           # kept for the record
    S = CR.Site(site)
    for p in CR.PARK_SPOTS:
        x, y = S.points[p]
        assert S.edge_dist((x, y)) >= S.edge + 8 and S.keepout_dist((x, y))[0] > CR.KEEPOUT_CLEAR_CM + 10
        assert x < S.view_x
    # the spots far enough apart that a parked robot never covers another's spot
    for i, p in enumerate(CR.PARK_SPOTS):
        for q in CR.PARK_SPOTS[i + 1:]:
            d = ((S.points[p][0] - S.points[q][0]) ** 2 + (S.points[p][1] - S.points[q][1]) ** 2) ** 0.5
            assert d > 2 * (CR.PARKED_R_CM + CR.KEEPOUT_CLEAR_CM)


# ------------------------------------------------------------------------------------------------- baseline
class _A:
    def __init__(self, state_dir, baseline="auto"):
        self.state_dir, self.baseline = state_dir, baseline


class _Rb:
    name = "AL-1DC0"


def _run(tmp_path, baseline="auto"):
    r = CR.CalibRun.__new__(CR.CalibRun)
    r.a, r.rb, r.wifi, r.folder = _A(str(tmp_path), baseline), _Rb(), {"AL-1DC0": "AL-1DC0"}, str(tmp_path / "s")
    return r


def test_first_calibration_starts_from_defaults_then_from_saved(tmp_path):
    r = _run(tmp_path)
    mode, words = r.baseline_mode()
    assert mode == "defaults" and "first calibration" in words
    fit = {"saved": {"ok": True, "how": "save"}, "new": {"wheel_scale": 0.97}, "baseline": "defaults"}
    r.record_calibrated(fit)
    rec = json.load(open(tmp_path / CR.CALIBRATED_FILE))
    assert rec["AL-1DC0"]["how"] == "save" and rec["AL-1DC0"]["values"] == {"wheel_scale": 0.97}
    assert r.baseline_mode()[0] == "saved"
    assert _run(tmp_path, "defaults").baseline_mode()[0] == "defaults"      # --baseline-defaults
    assert _run(tmp_path / "x", "saved").baseline_mode()[0] == "saved"      # --from-saved
    # not saved (switched off, or the save failed): not counted as calibrated
    r2 = _run(tmp_path / "y")
    r2.record_calibrated({"saved": {"ok": False, "how": "save"}})
    r2.record_calibrated({"saved": {"ok": True, "how": "off"}})
    assert not os.path.exists(tmp_path / "y" / CR.CALIBRATED_FILE)


def test_fit_uses_the_values_phase_a_ran_with():
    # phase A at the defaults: belief = raw; a robot whose real gyro is 1.06 turns 1.06 x what it believes
    turns = [(90.0, 1.5, 95.4), (-90.0, 1.5, -95.4), (170.0, 3.0, 180.2), (-170.0, 3.0, -180.2)]
    still = [(0.0, 60.0)]
    rolls = [(20.0, 18.6), (45.0, 41.85), (70.0, 65.1), (20.0, 18.6), (45.0, 41.85), (70.0, 65.1)]
    f = CR.fit_values(turns, still, rolls, dict(CR.BASELINE_DEFAULTS))
    assert abs(f["gyro_scale"] - 1.06) < 0.002 and abs(f["wheel_scale"] - 0.93) < 0.002


# ------------------------------------------------------------------------------------------------- names in the run
def test_console_session_aliases_name_a_robot_by_its_slot(tmp_path):
    import alvik_console as AC
    s = AC.Session(str(tmp_path), False, on_new_robot=lambda n: None)
    s.quiet = True
    s.aliases = {"AL-1DC0": "Alpha"}
    s.handle("AL-1DC0|1|M,hello", "192.168.1.20")
    s.handle("AL-2B7F|1|M,hello", "192.168.1.21")
    assert sorted(s.robots) == ["AL-2B7F", "Alpha"] and s.robots["Alpha"]["ip"] == "192.168.1.20"
    s.close()


def test_run_plan_and_mission_run_carry_the_aliases(tmp_path, monkeypatch):
    import argparse
    import mission_run as MR
    import wiggle_bind as WB
    src = open(os.path.join(FLEET, "run_plan.py")).read()
    assert '"--alias"' in src and "aliases=aliases" in src
    site = SI.load("school-conference")
    a = argparse.Namespace(site_=site, binding="auto", dry=False, robots="Alpha,Bravo", stream="127.0.0.1:8000",
                           calib="", robot=[], tags_=dict(site["tags"]), binding_file="")
    rec = {"slots": {"Alpha": 3, "Bravo": 2}, "empty_slots": [], "robots": [], "renamed": [],
           "aliases": {"AL-2B7F": "Alpha", "AL-1DC0": "Bravo"}}
    monkeypatch.setattr(WB, "run_for_mission", lambda *x, **k: rec)
    assert MR.wiggle_first(a, str(tmp_path)) == 0 and a.aliases_ == rec["aliases"]
    p = tmp_path / "b.json"
    p.write_text(json.dumps(rec))
    a2 = argparse.Namespace(**dict(vars(a), tags_=dict(site["tags"]), binding_file=str(p)))
    assert MR.wiggle_first(a2, str(tmp_path)) == 0 and a2.aliases_ == rec["aliases"]      # a feel-out child


# ------------------------------------------------------------------------------------------------- WiFi flashing
def test_ota_push_checks_the_file_and_reads_the_start_up_line(tmp_path):
    import ota_push as OP
    assert "no file" in OP.check_bin(str(tmp_path / "nothing.bin"))
    good = tmp_path / "home_lab.ino.bin"
    good.write_bytes(b"\xe9" + b"\0" * (300 * 1024))
    assert OP.check_bin(str(good)) == ""
    merged = tmp_path / "home_lab.ino.merged.bin"
    merged.write_bytes(b"\xe9" + b"\0" * (300 * 1024))
    assert "not the app image" in OP.check_bin(str(merged))
    small = tmp_path / "x.bin"
    small.write_bytes(b"\xe9" + b"\0" * 1000)
    assert "0.2-4 MB" in OP.check_bin(str(small))
    bad = tmp_path / "y.bin"
    bad.write_bytes(b"\x00" * (300 * 1024))
    assert "0xE9" in OP.check_bin(str(bad))
    assert OP.ota_state(["M,home_lab 2026-11-05a ready", "M,AL-1DC0: WiFi firmware updates on"]) == "on"
    assert OP.ota_state(["M,AL-1DC0: WiFi firmware updates off (add OTA_PASSWORD to secrets.h to turn them on)"]) == "off"
    assert OP.ota_state(["M,home_lab 2026-11-05a ready"]) == ""
    ota = open(os.path.join(os.path.dirname(TOOLS), "firmware", "home_lab", "ota.h")).read()
    assert OP.ON in ota and OP.OFF in ota       # the words the firmware says
