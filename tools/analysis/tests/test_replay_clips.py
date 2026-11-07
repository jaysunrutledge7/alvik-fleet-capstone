"""Offline tests for tools/analysis/replay_clips.py (no robots, Pis, cameras or network).

    python3 -m pytest -q tools/analysis/tests/test_replay_clips.py

A synthetic run: two robots on a 122 cm board, logs in the free-roam formats, and an overhead video drawn from a
known board -> pixel mapping, so the fitted mapping, the moments, the clips and the page can be checked.
"""
import csv
import json
import math
import os
import shutil
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "pi"))

import replay_clips as R  # noqa: E402

cv2 = pytest.importorskip("cv2")

T0 = 1794017000.0          # 2026-11-06 21:03:20 at -05:00
TZ = "-05:00"
H_TRUE = np.array([[2.4, 0.15, 170.0], [0.1, -2.35, 400.0], [0.0, 0.0, 1.0]])   # board cm -> px (y up -> down)


def _px(x, y):
    v = H_TRUE @ (x, y, 1.0)
    return v[0] / v[2], v[1] / v[2]


def _paths(t):
    """Left drives a circle; Center drives toward the x = 0 edge and stops 6 cm from it at t = 30 s."""
    a = 0.25 * t
    left = (80 + 22 * math.cos(a), 70 + 22 * math.sin(a), math.degrees(a) + 90)
    cx = max(6.0, 60 - 1.8 * t)
    center = (cx, 30.0 + 0.2 * t, 180.0)
    return {"Left": left, "Center": center}


def make_run(root, video=True, seconds=40.0, fps=11.0, clips_only=False):
    d = os.path.join(root, "20261106-210320-freeroam")
    os.makedirs(d)
    json.dump(dict(version="test", robots=["Left", "Center"], rect=[0, 0, 122, 122]), open(os.path.join(d, "run.json"), "w"))
    json.dump(dict(start="2026-11-06T21:03:20" + TZ, start_unix=T0), open(os.path.join(d, "session.json"), "w"))
    with open(os.path.join(d, "camera_track.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "robot", "x_cm", "y_cm", "th_deg", "arrived"])
        t = 0.0
        while t <= seconds:
            for n, (x, y, _) in _paths(t).items():
                w.writerow(["%.3f" % (T0 + t), n, "%.2f" % x, "%.2f" % y, "", "%.3f" % (T0 + t + 0.45)])
            t += 0.1
    with open(os.path.join(d, "follow.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow("robot,t_recv,run,ms,step_id,state,queued,last_done,last_failed,x_cm,y_cm,th_deg,pos_sd_cm,"
                   "th_sd_deg,fixes_used,fixes_rejected,front_cm,travel_cm,quiet_ms".split(","))
        t, k = 0.0, 0
        while t <= seconds:
            for n, (x, y, th) in _paths(t).items():
                off = 6.0 if (n == "Center" and 28.0 <= t <= 32.0) else 0.0      # own pose 6 cm off for 4 s
                rej = k // 4 if n == "Center" and t > 28 else 0
                w.writerow([n, "%.3f" % (T0 + t + 0.05), "x", 0, 1, "drive", 0, 0, 0, "%.2f" % (x + off), "%.2f" % y,
                            "%.1f" % th, 0.6, 1.0, k, rej, 50, 0, 20])
            t += 0.1
            k += 1
    rows = [(T0 + 30.0, "Center", "tilt stop", "its own tilt stop ended the step (tilted 6.1 deg)"),
            (T0 + 30.05, "Center", "trophy break", "tilted and no room to back away: stands until it can back off"),
            (T0 + 4.0, "Left", "front stop", "its own front sensor stopped the step: obstacle 5.0 cm ahead"),
            (T0 + 4.0, "Left", "keep-out", "front sensor felt something 5.0 cm ahead at 90.0, 80.0 (32 cm inside)")]
    with open(os.path.join(d, "events.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "robot", "kind", "text"])
        for r in sorted(rows):
            w.writerow(["%.2f" % r[0], r[1], r[2], r[3]])
    with open(os.path.join(d, "moves.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow("t,robot,n,kind,cell,turn_deg,length_cm,speed,x0,y0,x1,y1,est_s,dur_s,result,end_err_cm,"
                   "own_vs_cam_cm,cam_x,cam_y,style,curvature".split(","))
        k = -0.05
        L = 30.0
        th0 = math.radians(180.0)
        x1 = 15 + (math.sin(th0 + k * L) - math.sin(th0)) / k
        y1 = 35 - (math.cos(th0 + k * L) - math.cos(th0)) / k
        w.writerow(["%.2f" % (T0 + 27.5), "Center", 46, "menu", "", math.degrees(k * L), L, 8, 15, 35, "%.2f" % x1,
                    "%.2f" % y1, 3, 2.5, "failed: tilted 6.1 deg", "", "", "", "", "curve", k])
    with open(os.path.join(d, "keepouts.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["x", "y", "r", "t", "robot", "src", "edge_cm", "d_cm", "what"])
        w.writerow([90.0, 80.0, 2.0, "%.2f" % (T0 + 4.0), "Left", "front sensor", 32, 5.0, "found"])
    if video:
        vd = os.path.join(root, "rec")
        sub = os.path.join(vd, "frames", "overhead", "clips" if clips_only else "track")
        os.makedirs(sub)
        name = (os.path.join(sub, "210320_other-Center.avi") if clips_only
                else os.path.join(vd, "overhead_track_210320_freeroam.avi"))
        csvp = name[:-4] + ".csv" if clips_only else os.path.join(sub, "210320_freeroam.csv")
        vw = cv2.VideoWriter(name, cv2.VideoWriter_fourcc(*"MJPG"), 12, (640, 480))
        with open(csvp, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["i", "t", "pi_frame"])
            i, t = 0, 0.0
            while t <= seconds:
                img = np.full((480, 640, 3), 60, np.uint8)
                quad = np.int32([_px(0, 0), _px(122, 0), _px(122, 122), _px(0, 122)])
                cv2.fillConvexPoly(img, quad, (215, 215, 215))
                for n, (x, y, _) in _paths(t).items():
                    u, v = _px(x, y)
                    cv2.rectangle(img, (int(u - 11), int(v - 11)), (int(u + 11), int(v + 11)), (25, 25, 25), -1)
                cv2.rectangle(img, (int(_px(60, 90)[0]) - 10, int(_px(60, 90)[1]) - 10),
                              (int(_px(60, 90)[0]) + 10, int(_px(60, 90)[1]) + 10), (30, 30, 30), -1)   # an obstacle
                vw.write(img)
                w.writerow([i, "%.3f" % (T0 + t), i])
                i += 1
                t += 1.0 / fps
        vw.release()
        return d, vd
    return d, None


@pytest.fixture
def run(tmp_path):
    return make_run(str(tmp_path))


def test_arc_reaches_logged_end():
    """A curve's drawn path starts at x0, y0 and ends at the logged end (6 Nov move 46's numbers)."""
    m = dict(x0=18.6, y0=12.5, x1=27.0, y1=32.4, turn=-199.7, length=38.2, k=-0.09123, kind="menu", style="curve")
    p = R.move_path(m)
    assert len(p) > 2
    assert math.dist(p[0], (18.6, 12.5)) < 0.01 and math.dist(p[-1], (27.0, 32.4)) < 0.01
    # it swings out: some point well away from the chord's side
    assert min(x for x, _ in p) < 12.0


def test_reverse_is_a_chord():
    p = R.move_path(dict(x0=5, y0=18, x1=9, y1=11, turn=0, length=8, k=None, kind="reverse", style="stopturn"))
    assert p == [(5, 18), (9, 11)]


def test_pick_finds_kinds_and_spreads(run):
    d, _ = run
    S = R.Session(d)
    assert S.tz == -5.0
    moms = R.pick(S, 8)
    kinds = {m["kind"] for m in moms}
    assert "tilt" in kinds and "front" in kinds
    for m in moms:
        assert m["explain"] and len(m["explain"]) < 700
        a, b = m["span"]
        assert b - a <= R.MAX_SPAN_S + 1e-6 and a <= m["t"] <= b
    ts = sorted(m["t"] for m in moms)
    assert all(t2 - t1 >= R.MIN_GAP_S - 1e-6 for t1, t2 in zip(ts, ts[1:]))


def test_tilt_explanation_uses_camera_at_plan_time(run):
    d, _ = run
    S = R.Session(d)
    m = [x for x in R.pick(S, 8) if x["kind"] == "tilt"][0]
    assert "move 46" in m["explain"] and "camera put it" in m["explain"] and "worked out from the logs" in m["explain"]


def test_forced_moment_with_kind(run):
    d, _ = run
    S = R.Session(d)
    moms = R.pick(S, 1, at=[R._at_time("21:03:24/front", S)])
    assert len(moms) == 1 and moms[0]["kind"] == "front" and moms[0].get("forced")


def test_fix_status_from_counters(run):
    d, _ = run
    S = R.Session(d)
    early = S.cam_near("Left", T0 + 2.0)
    assert S.fix_status("Left", early) == "used"
    # Center's rejected counter rises after 28 s while its used counter also rises: used wins; lost = refused
    S.lost_iv["Center"] = [(T0 + 15.0, T0 + 16.0)]
    assert S.fix_status("Center", S.cam_near("Center", T0 + 15.5)) == "refused"


def test_pose_disagreement_found(run):
    d, _ = run
    S = R.Session(d)
    dis = R.pose_disagreements(S)
    assert any(n == "Center" and 5.5 < sp[2] < 6.5 and 3.0 < sp[1] - sp[0] < 4.5 for n, sp in dis)


def test_mapping_fitted_from_frames(run):
    d, vd = run
    S = R.Session(d)
    srcs = R.find_sources([vd], S)
    assert srcs and srcs[0]["kind"] == "track"
    m = R.fit_mapping(S, srcs, log=lambda *_: None)
    assert m is not None and m["median_err_px"] < 3.0
    H = np.array(m["H"])
    for x, y in [(10, 32), (100, 90), (60, 60), (100, 50)]:        # where the robots went: within 3 px
        v = H @ (x, y, 1.0)
        assert math.dist((v[0] / v[2], v[1] / v[2]), _px(x, y)) < 3.0
    for x, y in [(0, 0), (122, 0), (122, 122), (0, 122)]:           # the corners no robot visited: within 10 px
        v = H @ (x, y, 1.0)
        assert math.dist((v[0] / v[2], v[1] / v[2]), _px(x, y)) < 10.0
    assert abs(srcs[0]["lag"]) <= 0.1


def test_derived_timing_without_csv(tmp_path):
    d, vd = make_run(str(tmp_path))
    os.remove(os.path.join(vd, "frames", "overhead", "track", "210320_freeroam.csv"))
    S = R.Session(d)
    s = R.find_sources([vd], S)[0]
    ts = R.source_times(s)
    assert "derived" in s["timing"] and abs(ts[0] - T0) < 1.0


@pytest.mark.skipif(not R.ffmpeg_ok(), reason="no ffmpeg")
def test_make_and_page(run, tmp_path):
    d, vd = run
    out = str(tmp_path / "out")
    res = R.make(d, [vd], out, top=3, log=lambda *_: None)
    assert res["moments"] and res["mapping_err_px"] < 3.0
    for m in res["moments"]:
        assert m["clip"] and os.path.getsize(os.path.join(out, m["clip"])) <= R.MAX_BYTES
        assert os.path.exists(os.path.join(out, m["poster"]))
        md = json.load(open(os.path.join(out, m["data"])))
        assert md["samples"] and md["t0"] <= m["t"] <= md["t1"] + 0.1
        assert {"Left", "Center"} <= set(md["samples"][0]["r"])
    page = R.build_page([out], str(tmp_path / "page"), log=lambda *_: None)
    html = open(page).read()
    assert "<title>Robot Replay</title>" in html and "/*__DATA__*/" not in html
    assert os.path.getsize(page) < 16 * 1024 * 1024
    assert "prefers-color-scheme: dark" in html and ':root[data-theme="dark"]' in html
    data = json.loads(html.split('<script id="replay-data" type="application/json">')[1].split("</script>")[0])
    assert data["runs"][0]["moments"][0]["clip"].startswith("r1_")


@pytest.mark.skipif(not R.ffmpeg_ok(), reason="no ffmpeg")
def test_event_clips_used_when_no_full_video(tmp_path):
    d, vd = make_run(str(tmp_path), clips_only=True)
    res = R.make(d, [vd], str(tmp_path / "out"), top=2, log=lambda *_: None)
    assert any(m["clip"] for m in res["moments"])
    assert any("210320_other-Center" in (m["source"] or "") for m in res["moments"])


def test_logs_only_run_still_gives_moments(tmp_path):
    d, _ = make_run(str(tmp_path), video=False)
    res = R.make(d, [], str(tmp_path / "out"), top=3, log=lambda *_: None)
    assert res["moments"] and all(m["clip"] is None for m in res["moments"])
    assert all(os.path.exists(os.path.join(str(tmp_path / "out"), m["data"])) for m in res["moments"])


def test_auto_switch_off(tmp_path, monkeypatch):
    d, vd = make_run(str(tmp_path), video=False)
    monkeypatch.setenv("MIRLT_REPLAY_CLIPS", "0")
    assert R.auto(d, [], log=lambda *_: None) == 0
    assert not os.path.exists(R.replay_dir(d))


def test_auto_fails_softly(tmp_path):
    """A broken session folder never raises and leaves no busy marker behind."""
    d = str(tmp_path / "nothing-here")
    os.makedirs(d)
    open(os.path.join(d, "events.csv"), "w").write("garbage\x00\n")
    assert R.auto(d, [str(tmp_path / "missing")], log=lambda *_: None) == 0
    assert not os.path.exists(os.path.join(R.replay_dir(d), ".busy"))


def test_auto_cap_skips_clips(tmp_path):
    d, vd = make_run(str(tmp_path))
    R.auto(d, [vd], cap_s=0.0, log=lambda *_: None)
    res = json.load(open(os.path.join(R.replay_dir(d), "moments.json")))
    assert res["moments"] and all(m["clip"] is None for m in res["moments"])
    assert os.path.exists(os.path.join(R.replay_dir(d), "page", "index.html"))


def test_upload_cleanup_waits_for_replay(tmp_path, monkeypatch):
    import upload_session as U
    sess = str(tmp_path / "s")
    os.makedirs(sess + "-replay")
    t = time.time()
    U.wait_for_replay(sess, lambda m: None, poll_s=0.01)       # no marker: straight on
    assert time.time() - t < 0.5
    open(sess + "-replay/.busy", "w").write("1")
    monkeypatch.setenv("MIRLT_REPLAY_WAIT_S", "0.2")
    said = []
    U.wait_for_replay(sess, said.append, poll_s=0.05)          # waits, up to the cap
    assert said
    old = time.time() - 7200
    os.utime(sess + "-replay/.busy", (old, old))                # a crashed run's marker: ignored
    t = time.time()
    U.wait_for_replay(sess, lambda m: None, poll_s=0.05)
    assert time.time() - t < 0.2
