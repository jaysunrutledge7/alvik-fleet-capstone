"""5 Nov 2026 (the school Pi): `rec` looked for home's overhead camera ("USB 2.0 Camera") because the school site's folder
had no cameras.json, and warned it used home's roam1 floor calibration. A non-home site never gets home's camera names
or home's floor calibration; an existing cameras.json (Nick's camnames.py --set) always wins."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(PI))


def _sysfs(tmp, names):
    for k, n in enumerate(names):
        d = tmp / "sys" / ("video%d" % (2 * k))
        d.mkdir(parents=True)
        (d / "name").write_text(n + "\n")
        (d / "index").write_text("0\n")
    return str(tmp / "sys")


def _run(args, env):
    e = dict(os.environ, **env)
    return subprocess.run([sys.executable] + args, capture_output=True, text=True, cwd=REPO, env=e, timeout=60)


def test_school_without_file_uses_site_names_never_home(tmp_path):
    env = dict(MIRLT_SITE="school-conference", MIRLT_CONF_DIR=str(tmp_path / "conf"),
               MIRLT_V4L_SYSFS=_sysfs(tmp_path, ["Logi Webcam C920e"]))
    r = _run(["tools/pi/camnames.py", "--dev", "overhead"], env)
    assert r.returncode == 0 and r.stdout.strip() == "0", r.stderr
    r = _run(["tools/pi/camnames.py"], env)
    assert "USB 2.0 Camera" not in r.stdout and "OFF: the side camera" in r.stdout


def test_existing_file_wins_and_init_keeps_it(tmp_path):
    conf = tmp_path / "conf"
    conf.mkdir()
    (conf / "cameras.json").write_text(json.dumps({"overhead": "Webcam"}))
    env = dict(MIRLT_SITE="school-conference", MIRLT_CONF_DIR=str(conf),
               MIRLT_V4L_SYSFS=_sysfs(tmp_path, ["Logi Webcam C920e"]))
    r = _run(["tools/pi/camnames.py", "--init"], env)
    assert "kept" in r.stdout
    assert json.loads((conf / "cameras.json").read_text()) == {"overhead": "Webcam"}


def test_init_writes_site_names(tmp_path):
    conf = tmp_path / "conf"
    env = dict(MIRLT_SITE="school-conference", MIRLT_CONF_DIR=str(conf))
    r = _run(["tools/pi/camnames.py", "--init"], env)
    assert r.returncode == 0, r.stdout
    d = json.loads((conf / "cameras.json").read_text())
    assert d["overhead"] == "C920e" and d["side"] == ""


def test_site_without_names_gets_none(tmp_path):
    sys.path.insert(0, PI)
    import camnames as CN
    d, words = CN.site_defaults({"name": "elsewhere"})
    assert d == {"overhead": "", "side": ""} and "home's are not used" in words
    d, _ = CN.site_defaults({"name": "home"})
    assert d == CN.DEFAULTS


def test_setup_conf_no_roam1_for_school(tmp_path):
    env = dict(MIRLT_SITE="school-conference", MIRLT_CONF_DIR=str(tmp_path / "conf"))
    r = _run(["tools/pi/setup_conf.py", "calib"], env)
    assert r.returncode == 0 and r.stdout.strip() == ""
    assert "roam1 is NOT used" in r.stderr and "tagcal" in r.stderr
    r = _run(["tools/pi/setup_conf.py", "calib"], dict(env, MIRLT_SITE="home"))
    assert "roam1" in r.stdout
