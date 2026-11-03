"""The six fixes from the pre-session risk review (2026-11-03): the emergency stop reaches every robot, the Pi's own
address, a broken site file stops loudly, the refit runs in the background and is reused, disk floors scale with the
pictures, and the new Pi's self-check. No robot, no Pi."""
import csv
import json
import os
import socket
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import sites as SI           # noqa: E402
import stop_all as SA        # noqa: E402
import pi_addr as PA         # noqa: E402
import diskguard as DG       # noqa: E402


def _listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(0.3)
    return s, s.getsockname()[1]


def _drain(s):
    got = []
    while True:
        try:
            got.append(s.recvfrom(64)[0])
        except socket.timeout:
            return got


# ------------------------------------------------------------------------------------------------- 1. stop all
def test_stopall_reaches_robots_heard_on_the_wifi(tmp_path, monkeypatch, capsys):
    heard = tmp_path / "heard.json"
    SA.remember("Alpha", "127.0.0.1", path=str(heard))
    monkeypatch.setattr(SA, "HEARD_FILE", str(heard))
    monkeypatch.setenv("MIRLT_SITE", "school-conference")      # no fixed addresses at school
    rx, port = _listener()
    SA.main(["--no-tmux", "--mc-url", "http://127.0.0.1:9/x", "--port", str(port), "--listen-s", "0.1"])
    out = capsys.readouterr().out
    got = _drain(rx)
    assert got.count(b"stop") == SA.STOP_REPEATS               # the heard robot, by its address
    assert "3b. Broadcast" in out and "Alpha at 127.0.0.1" in out
    assert "site school-conference: no fixed addresses" in out


def test_stopall_old_way_kept_behind_a_switch(tmp_path, monkeypatch, capsys):
    heard = tmp_path / "heard.json"
    SA.remember("Alpha", "127.0.0.1", path=str(heard))
    monkeypatch.setattr(SA, "HEARD_FILE", str(heard))
    rx, port = _listener()
    SA.main(["--no-tmux", "--mc-url", "http://127.0.0.1:9/x", "--port", str(port), "--only-listed",
             "--robot", "127.0.0.2"])
    out = capsys.readouterr().out
    assert _drain(rx) == []                                    # the heard robot is not reached the old way
    assert "only these: --only-listed" in out and "Broadcast" not in out


def test_heard_file_keeps_a_day_and_survives_garbage(tmp_path):
    p = str(tmp_path / "h.json")
    SA.remember("Alpha", "10.0.0.5", path=p, now=1000.0)
    SA.remember("Bravo", "10.0.0.6", path=p, now=1000.0 + 3600 * 30)
    assert SA.heard_from_file(p, now=1000.0 + 3600 * 30) == {"Bravo": "10.0.0.6"}
    with open(p, "w") as f:
        f.write("{not json")
    assert SA.heard_from_file(p) == {}


def test_console_link_remembers_robots(tmp_path):
    import alvik_console as AC
    p = str(tmp_path / "h.json")
    AC.remember_robot("Alpha", "192.168.4.20", path=p)
    assert SA.heard_from_file(p) == {"Alpha": "192.168.4.20"}


def test_stopall_never_blocked_by_a_broken_site(monkeypatch):
    monkeypatch.setenv("MIRLT_SITE", "no-such-site")
    ips, words = SA.site_ips()
    assert ips == [] and "could not be read" in words


# ------------------------------------------------------------------------------------------------- 2. the Pi's address
def test_pi_address_detected_tailscale_then_lan_then_name():
    def fake(outs):
        return lambda cmd, timeout=2.0: outs.get(cmd[0], "")
    assert PA.detect(fake({"tailscale": "100.101.5.9\n", "hostname": "192.168.1.30 100.101.5.9"}), pin="")[0] == \
        "100.101.5.9"
    assert PA.detect(fake({"hostname": "192.168.1.30 100.101.5.9 fe80::1"}), pin="")[0] == "192.168.1.30"
    a, how = PA.detect(fake({}), pin="")
    assert a.endswith(".local") and "name" in how
    assert PA.detect(fake({"tailscale": "100.101.5.9"}), pin="10.1.1.1")[0] == "10.1.1.1"      # the old way, pinned


def test_school_site_never_shows_home_address():
    s = SI.load("school-conference")
    assert not (s.get("pi") or {}).get("use_site_address")
    sh = open(os.path.join(REPO, "run-fleet-pi.sh")).read()
    assert 'PIADDR="${MIRLT_PI_ADDR:-100.122.222.75}"' not in sh
    assert "tmux new-session -d -s mc" in sh and "mc here" in sh          # Mission Control in tmux; old way kept


# ------------------------------------------------------------------------------------------------- 3. broken site file
def _site(tmp_path, monkeypatch, text):
    d = tmp_path / "sites"
    d.mkdir()
    (d / "home.json").write_text(open(os.path.join(FLEET, "sites", "home.json")).read())
    (d / "broken.json").write_text(text)
    monkeypatch.setattr(SI, "SITES_DIR", str(d))
    monkeypatch.setenv("MIRLT_SITE", "broken")


def test_broken_site_names_file_line_and_problem(tmp_path, monkeypatch):
    txt = open(os.path.join(FLEET, "sites", "school-conference.json")).read().replace('"height_cm": null',
                                                                                    '"height_cm": 6,2')
    _site(tmp_path, monkeypatch, txt)
    with pytest.raises(SI.SiteFileError) as e:
        SI.load()
    msg = str(e.value)
    assert "broken.json" in msg and "line 13" in msg and "6.2, not 6,2" in msg and "Nothing was started" in msg


def test_wrong_kind_of_value_is_named(tmp_path, monkeypatch):
    _site(tmp_path, monkeypatch, json.dumps({"tracking": "tags", "robots": [{"name": "Alpha"}],
                                             "tags": {"height_cm": "6.2", "ids": {"Alpha": 0}}}))
    with pytest.raises(SI.SiteFileError) as e:
        SI.load()
    assert "tags.height_cm should be a number" in str(e.value)


def test_missing_site_is_loud_and_mission_run_does_not_fall_back(monkeypatch):
    import mission_run as MR
    monkeypatch.setenv("MIRLT_SITE", "no-such-site")
    a = MR.argparse.Namespace(site="", tracking="auto")
    with pytest.raises(SystemExit) as e:
        MR.resolve_site(a)
    assert SI.is_site_file_error(e.value)


def test_launcher_stops_on_a_broken_site_but_stopall_still_runs():
    env = dict(os.environ, MIRLT_SITE="no-such-site", FLEET_DRY="1")
    r = subprocess.run(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "check"], capture_output=True, text=True,
                       env=env, timeout=60)
    assert r.returncode == 1 and "No site called 'no-such-site'" in r.stderr and "STOPPED" in r.stderr
    r = subprocess.run(["bash", "-c", "grep -n 'site|stopall|selfcheck' %s" % os.path.join(REPO, "run-fleet-pi.sh")],
                       capture_output=True, text=True)
    assert r.returncode == 0


# ------------------------------------------------------------------------------------------------- 4. refit
def _refit_inputs(tmp_path, n=10):
    import site_fit as SF
    obs, truth, st = SF.simulate(seed=4, n_spots=n, stickers=6)
    groups = obs["tag_groups"]
    obs["tag_groups"] = groups[:4]
    ob = tmp_path / "obs.json"
    ob.write_text(json.dumps(obs))
    run = tmp_path / "run"
    (run / "bridge").mkdir(parents=True)
    with open(run / "bridge" / "tag_obs.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "cam", "id", "u0", "v0", "u1", "v1", "u2", "v2", "u3", "v3"])
        for k, g in enumerate(groups[4:]):
            for c, cs in g["reads"].items():
                w.writerow([100 + k, c, g["id"]] + [v for p in cs for v in p])
    return str(ob), str(run)


def test_refit_runs_in_the_background_and_the_button_reuses_it(tmp_path):
    import site_fit as SF
    import tag_job as TJ
    ob, run = _refit_inputs(tmp_path)
    out = os.path.join(run, "tagcal_refit")
    t0 = time.time()
    st = SF.start_refit_bg(ob, os.path.join(run, "bridge", "tag_obs.csv"), out)
    assert time.time() - t0 < 3 and st["state"] == "running"         # returns at once
    for _ in range(300):
        if SF.refit_status(out)["state"] != "running":
            break
        time.sleep(0.5)
    st = SF.refit_status(out)
    assert st["state"] == "done" and st["size_cm"] and st["words"]

    class Sess:
        dir = str(tmp_path)
    site = {"conf_dir": str(tmp_path / "conf"), "tags": {"tag_cm": 8.0, "height_cm": 6.0, "ids": {"A": 0}},
            "robots": [], "capture": {"width": 1280, "height": 960}, "cameras": {}}
    tj = TJ.TagJob(Sess(), site)
    mtime = os.path.getmtime(os.path.join(out, "site_fit.json"))
    t0 = time.time()
    r = tj.refit(run)
    assert r["reused"] and time.time() - t0 < 2 and r["size_cm"] == st["size_cm"]
    assert os.path.getmtime(os.path.join(out, "site_fit.json")) == mtime          # nothing computed again
    r = tj.refit(run, use=True)
    assert r["used"] and os.path.exists(os.path.join(site["conf_dir"], "tagcal", "site_fit.json"))
    assert not os.path.exists(os.path.join(site["conf_dir"], "tagcal", "status.json"))


def test_refit_failed_process_is_said(tmp_path):
    import site_fit as SF
    out = str(tmp_path / "r")
    SF._write_status(out, state="running", pid=999999, started=time.time() - 5)
    st = SF.refit_status(out)
    assert st["state"] == "failed" and "ended without a result" in st["why"]


def test_mission_control_starts_the_refit_in_the_background_by_default():
    src = open(os.path.join(FLEET, "mission_control.py")).read()
    assert '"--refit-bg"' in src and 'auto_refit' in src
    assert "--refit-bg" in open(os.path.join(FLEET, "mission_run.py")).read()


# ------------------------------------------------------------------------------------------------- 5. disk
def test_disk_floors_scale_with_picture_size_and_cameras():
    import camrec
    assert camrec.disk_floors([(640, 480)] * 2, 25) == (1.0, pytest.approx(0.3))              # home: unchanged
    warn, stop = camrec.disk_floors([(1280, 960)] * 2, 15)
    assert warn == pytest.approx(2.4) and stop == pytest.approx(0.72)
    lines = []
    camrec.disk_precheck("/tmp", 2, 0, log=lines.append, free=2.0, width=1280, height=960, fps=15)
    assert lines[0].startswith("! LOW DISK SPACE") and "1280x960" in lines[0]
    lines = []
    camrec.disk_precheck("/tmp", 2, 0, log=lines.append, free=2.0)                             # 640 x 480: fine
    assert lines[0].startswith("Disk:")


def test_diskguard_scales_frames_floors_with_the_site_capture():
    home = {"MIRLT_CAPTURE": "--width 640 --height 480", "MIRLT_CAPTURE_FPS": "25"}
    school = {"MIRLT_CAPTURE": "--width 1280 --height 960", "MIRLT_CAPTURE_FPS": "15"}
    assert DG.capture_scale(home) == 1.0 and DG.capture_scale(school) == 4.0
    assert DG.rec_gb_per_h(school) == pytest.approx(14.4) and DG.rec_gb_per_h(home) == pytest.approx(6.0)
    g = DG.Guard(frames_dir="/tmp/x", scale=1.0, fake={"card": 10, "drive": 10})
    assert (g.card_frames_gb, g.drive_frames_gb) == (DG.CARD_FRAMES_GB, DG.DRIVE_FRAMES_GB)
    g = DG.Guard(frames_dir="/tmp/x", scale=4.0, fake={"card": 3.0, "drive": 10})
    assert g.card_frames_gb == pytest.approx(5.0) and g.drive_frames_gb == pytest.approx(20.0)
    assert not g.state()["frames_ok"]                       # 3 GB on the card: frames off at 1280 x 960 x 2


# ------------------------------------------------------------------------------------------------- 6. the new Pi
def test_selfcheck_never_prints_the_token(tmp_path, monkeypatch, capsys):
    import selfcheck as SC
    tok = tmp_path / "github_token"
    tok.write_text("github_pat_SECRETSECRET\n")
    os.chmod(tok, 0o644)
    monkeypatch.setattr(SC, "TOKEN_PATH", str(tok))
    R = SC.Report()
    SC.check_token(R, net=False)
    out = capsys.readouterr().out
    assert "SECRET" not in out and "chmod 600" in out


def test_selfcheck_runs_and_names_every_area(capsys):
    import selfcheck as SC
    rc = SC.main(["--no-net"])
    out = capsys.readouterr().out
    for what in ("Python", "numpy", "cv2", "tag detector", "repository", "site", "GitHub token", "Tailscale", "clock",
                 "SD card", "address"):
        assert what in out
    assert rc in (0, 1)


def test_school_site_packages_are_the_listed_ones():
    import selfcheck as SC
    assert {m for m, _, _ in SC.PACKAGES} == {"numpy", "scipy", "cv2", "matplotlib"}


def test_by_id_names(tmp_path, monkeypatch):
    import camnames as CN
    d = tmp_path / "by-id"
    d.mkdir()
    (tmp_path / "video2").write_text("")
    (tmp_path / "video3").write_text("")
    os.symlink(str(tmp_path / "video2"), str(d / "usb-Cam_ABC_123-video-index0"))
    os.symlink(str(tmp_path / "video3"), str(d / "usb-Cam_ABC_123-video-index1"))
    assert CN.by_id_names(str(d)) == {2: "usb-Cam_ABC_123-video-index0"}
