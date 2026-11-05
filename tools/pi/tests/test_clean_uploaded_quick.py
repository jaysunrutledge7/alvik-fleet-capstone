"""clean_uploaded.py quick mode, auto-clean after an upload, `uploads` status, the refusal while a run goes
(2026-11-05, Nick: "a much simpler way of checking that uploads are being done, even if its not airtight").

GitHub is faked (no network); nothing on a Pi or robot is touched.
    python3 -m pytest tools/pi/tests/test_clean_uploaded_quick.py -q
"""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
TOOLS = os.path.dirname(PI)
for p in (PI, os.path.join(TOOLS, "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

import clean_uploaded as C   # noqa: E402
import upload_session as US  # noqa: E402
import diskguard as DG       # noqa: E402

NAME = "20261105-143500-freeroam"


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("MIRLT_DRIVE", str(tmp_path / "no-drive"))
    monkeypatch.setattr(US, "STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setattr(DG, "rec_out", lambda: str(tmp_path / "home" / "recordings"))
    monkeypatch.setattr(C, "running", lambda: [])
    monkeypatch.setattr(US, "token", lambda: None)
    monkeypatch.setattr(C, "zip_size", lambda *a, **k: pytest.fail("the quick check must not rebuild any zip"))
    return tmp_path


def _session(root, name=NAME):
    s = root / "logs" / name
    (s / "frames" / "overhead" / "track").mkdir(parents=True)
    (s / "moves.csv").write_text("t,robot\n1,Left\n")
    (s / "report.html").write_text("<p>report</p>")
    (s / "frames" / "overhead" / "track" / "drive01.avi").write_bytes(b"v" * 5000)
    (s / "frames" / "overhead" / "snap.jpg").write_bytes(b"j" * 300)
    (s / "frames" / "overhead" / "events.csv").write_text("t,kind\n")
    return s, s / "frames"


def _record(env, name, tag, assets, **kw):
    d = env / "state" / "uploads"
    d.mkdir(parents=True, exist_ok=True)
    m = dict(tag=tag, logs=True, exclude=["frames"], assets=assets)
    m.update(kw)
    (d / (name + ".json")).write_text(json.dumps(m))


def _gh(rels):
    def get(url, tok):
        return [dict(tag_name=t, assets=[dict(name=n, size=s, state="uploaded") for n, s in a.items()])
                for t, a in rels.items()]
    return get


def _run(env, rels, *args):
    out = []
    code = C.main(["--logs", str(env / "logs")] + list(args), get=_gh(rels), say=out.append)
    return code, "\n".join(out)


def test_quick_check_with_the_uploads_record(env):
    sess, fd = _session(env)
    sent = {"logs-%s.zip" % NAME: 1234, "rec-frames-small.zip": 999}
    _record(env, NAME, "freeroam-" + NAME, sent, recording=str(fd))
    # one file missing on GitHub: not confirmed, kept
    code, out = _run(env, {"freeroam-" + NAME: {"logs-%s.zip" % NAME: 1234}}, "--yes")
    assert sess.exists() and "missing 1 of the 2 files" in out and "rec-frames-small.zip" in out
    # a file at another size: kept
    code, out = _run(env, {"freeroam-" + NAME: dict(sent, **{"rec-frames-small.zip": 998})}, "--yes")
    assert sess.exists()
    # everything the upload sent is there: listed, deleted only with --yes
    rels = {"freeroam-" + NAME: dict(sent)}
    code, out = _run(env, rels)
    assert code == 0 and "DELETE" in out and "quick check" in out and sess.exists()
    code, out = _run(env, rels, "--yes")
    assert code == 0 and not sess.exists()


def test_quick_check_without_a_record_needs_a_release_with_a_file(env):
    sess, fd = _session(env)
    code, out = _run(env, {"freeroam-" + NAME: {}}, "--yes")
    assert sess.exists() and "has no files" in out
    code, out = _run(env, {}, "--yes")
    assert sess.exists() and "no release on GitHub holds it" in out
    code, out = _run(env, {"freeroam-%s-2" % NAME: {"anything.zip": 5}}, "--yes")
    assert not sess.exists()


def test_frames_not_in_the_upload_are_kept(env):
    sess, fd = _session(env)
    sent = {"logs-%s.zip" % NAME: 1234}
    _record(env, NAME, "freeroam-" + NAME, sent)        # logs only, frames left out, no recording sent
    code, out = _run(env, {"freeroam-" + NAME: dict(sent)}, "--yes")
    assert sess.exists() and fd.exists() and "frames folder was not part of the upload" in out


def test_refuses_while_a_robot_run_is_going_unless_force(env, monkeypatch):
    sess, fd = _session(env)
    rels = {"freeroam-" + NAME: {"x.zip": 1}}
    monkeypatch.setattr(C, "running", lambda: ["tmux freeroam"])
    code, out = _run(env, rels, "--yes")
    assert code == 1 and sess.exists() and "tmux freeroam" in out and "CPU" in out and "--force" in out
    assert "Asking GitHub" not in out
    code, out = _run(env, rels, "--yes", "--force")
    assert code == 0 and not sess.exists()


def test_status_works_while_a_run_is_going(env, monkeypatch):
    _session(env)
    _session(env, "20261105-100000-doe")
    _session(env, "20261105-090000-freeroam-dryrun")
    monkeypatch.setattr(C, "running", lambda: ["tmux freeroam"])
    code, out = _run(env, {"freeroam-" + NAME: {"x.zip": 1}}, "--status")
    lines = [l for l in out.splitlines() if NAME in l or "100000-doe" in l or "dryrun" in l]
    assert code == 0 and len(lines) == 3
    assert lines[0].strip().startswith("✓") and NAME in lines[0]               # newest first
    assert lines[1].strip().startswith("✗") and "no release" in lines[1]
    assert "dry run" in lines[2]


def test_robot_programs_from_proc(tmp_path):
    def proc(pid, *argv):
        d = tmp_path / str(pid)
        d.mkdir()
        (d / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")
    proc(101, "python3", "tools/fleet/mission_run.py", "--table", "61x122")     # Mission Control's run
    proc(102, "python3", "tools/fleet/freeroam.py", "--dry", "--hours", "1")   # simulator: not counted
    proc(103, "python3", "tools/fleet/freeroam.py", "status")                  # a quick look: not counted
    proc(104, "bash", "-c", "python3 tools/fleet/doe_loop.py run")             # a shell line: not counted
    proc(105, "python3", "-u", "tools/fleet/calib_run.py")
    got = C.robot_programs(str(tmp_path))
    assert sorted(got) == ["calib_run.py (pid 105)", "mission_run.py (pid 101)"]


def test_tmux_busy_without_tmux_is_false(monkeypatch):
    def boom(*a, **k):
        raise OSError("no tmux")
    monkeypatch.setattr(C.subprocess, "run", boom)
    assert C._tmux_busy("freeroam") is False


def test_free_after_upload_keeps_logs_and_small_files(env):
    sess, fd = _session(env)
    sent = {"logs-%s.zip" % NAME: 100, "rec-frames-small.zip": 200}
    out = []
    freed = C.free_after_upload(str(sess), str(fd), "freeroam-" + NAME, sent, dict(sent), logs_up=True,
                                exclude=["frames"], roots=[str(sess)], say=out.append)
    assert freed == 5300
    assert not (fd / "overhead" / "track").exists() and not (fd / "overhead" / "snap.jpg").exists()
    assert (fd / "overhead" / "events.csv").exists()
    assert (sess / "moves.csv").exists() and (sess / "report.html").exists()
    assert "5300" not in "\n".join(out) and "freed" in "\n".join(out)


def test_free_after_upload_deletes_nothing_when_github_lacks_a_file(env):
    sess, fd = _session(env)
    sent = {"logs-%s.zip" % NAME: 100, "rec-frames-small.zip": 200}
    out = []
    assert C.free_after_upload(str(sess), str(fd), "t", sent, {"logs-%s.zip" % NAME: 100}, roots=[str(sess)],
                               say=out.append) == 0
    assert (fd / "overhead" / "track" / "drive01.avi").exists() and "nothing deleted" in "\n".join(out)


def test_free_after_upload_frames_inside_the_logs_zip_and_outside_roots(env, tmp_path):
    sess, fd = _session(env)
    sent = {"logs-%s.zip" % NAME: 100}
    C.free_after_upload(str(sess), "", "t", sent, dict(sent), logs_up=True, exclude=(), roots=[str(sess)],
                        say=lambda m: None)
    assert not (fd / "overhead" / "track").exists() and (fd / "overhead" / "events.csv").exists()
    other = tmp_path / "elsewhere"
    other.mkdir()
    (other / "big.avi").write_bytes(b"x" * 10)
    out = []
    C.free_after_upload(str(sess), str(other), "t", sent, dict(sent), roots=[str(sess)], say=out.append)
    assert (other / "big.avi").exists() and "skipped" in "\n".join(out)


def test_auto_clean_switch(monkeypatch):
    monkeypatch.setenv("MIRLT_SITE", "home")
    monkeypatch.delenv("MIRLT_AUTO_CLEAN", raising=False)
    assert C.auto_clean_on() is True
    monkeypatch.setenv("MIRLT_SITE", "school-conference")
    assert C.auto_clean_on() is False
    monkeypatch.setenv("MIRLT_AUTO_CLEAN", "1")
    assert C.auto_clean_on() is True
    monkeypatch.setenv("MIRLT_SITE", "home")
    monkeypatch.setenv("MIRLT_AUTO_CLEAN", "0")
    assert C.auto_clean_on() is False


def _upload(tmp_path, monkeypatch, *extra):
    """A whole upload_session run against a fake GitHub that keeps what it is sent."""
    sess = tmp_path / "logs" / "e02-20261105-090000"
    sess.mkdir(parents=True)
    (sess / "e02_log.txt").write_text("hello")
    rec = tmp_path / "rec" / "20261105-090010_E02" / "cam0"
    rec.mkdir(parents=True)
    (rec / "frames.csv").write_text("frame,t\n")
    (rec / "video.avi").write_bytes(os.urandom(3000))
    monkeypatch.setattr(US, "PART_BYTES", 1000)
    monkeypatch.setattr(US, "SMALL_BYTES", 500)
    monkeypatch.setattr(US, "token", lambda: "t")
    monkeypatch.setattr(US, "STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setattr(US, "PROGRESS", US.Progress(str(tmp_path / "state" / "upload.json")))
    monkeypatch.setenv("MIRLT_DRIVE", str(tmp_path / "no-drive"))
    gh = {}

    def fake_api(method, url, tok, data=None, ctype=None, length=None):
        if method == "GET":
            return dict(upload_url="https://up/x{?name}", url="https://api/rel/1",
                        assets=[dict(name=n, size=s, state="uploaded") for n, s in gh.items()])
        if data is not None and hasattr(data, "read"):
            n = 0
            while True:
                c = data.read(400)
                if not c:
                    break
                n += len(c)
            gh[url.split("name=")[1]] = n
        return {}

    monkeypatch.setattr(US, "api", fake_api)
    monkeypatch.setattr(sys, "argv", ["x", "--recordings", str(tmp_path / "rec"), "--session", str(sess)] + list(extra))
    US.main()
    return sess, rec


def test_upload_then_auto_clean_frees_the_video_keeps_logs(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("MIRLT_AUTO_CLEAN", "1")
    sess, rec = _upload(tmp_path, monkeypatch)
    out = capsys.readouterr().out
    assert not (rec / "video.avi").exists() and (rec / "frames.csv").exists()
    assert (sess / "e02_log.txt").exists()
    assert "Auto-clean: deleted 1 big file(s)" in out and "freed" in out


def test_upload_keep_local_or_switch_off_deletes_nothing(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("MIRLT_AUTO_CLEAN", "1")
    sess, rec = _upload(tmp_path, monkeypatch, "--keep-local")
    assert (rec / "video.avi").exists() and "Auto-clean is off" in capsys.readouterr().out
