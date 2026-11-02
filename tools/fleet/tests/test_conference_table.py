"""Conference-table build (2026-11-02): sites, AprilTags (detector, geometry, simulator camera, bridge, facing check),
the site fit, production runs, sync marks, the pose timeline, the external comparison, the camera check-out, the tag
sheet. Simulator pictures only; no robot, no Pi."""
import csv
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment"),
          os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import sites as SI            # noqa: E402
import tagcore as TC          # noqa: E402
import tag_geom as TG         # noqa: E402
import tag_cam as CAMS        # noqa: E402
import production as PR       # noqa: E402
import missions as MS         # noqa: E402
import sync_tools as ST       # noqa: E402
import merge_external as ME   # noqa: E402

TAGS = {"family": "tag25h9", "tag_cm": 8.0, "roof_cm": 9.5, "height_cm": 6.0, "ids": {"Alpha": 0, "Bravo": 1}}


def robot(x, y, hd, i, **kw):
    return dict(x=x, y=y, th=math.radians(hd), z=6.0, family="tag25h9", id=i, tag_cm=8.0, roof_cm=9.5, **kw)


# ------------------------------------------------------------------------------------------------- sites
def test_home_site_is_unchanged(monkeypatch):
    monkeypatch.delenv("MIRLT_SITE", raising=False)
    s = SI.load("home")
    assert SI.robot_args(s) == "--robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253"
    assert s["conf_dir"] == os.path.expanduser("~/.config/mirlt")
    assert s["tracking"] == "lights" and s["mc_dir"].endswith(os.path.join("logs", "mission-control"))


def test_school_site_tags_and_own_folders():
    s = SI.load("school-conference")
    assert s["tracking"] == "tags" and SI.tag_ids(s) == {"Alpha": 0, "Bravo": 1}
    assert s["conf_dir"].endswith(os.path.join("sites", "school-conference"))
    assert SI.robot_args(s) == ""                    # found on the WiFi by their hello
    env = "\n".join(SI.env_lines(s))
    assert "MIRLT_CONF_DIR=" in env and "--width 1280 --height 960" in env


def test_site_name_rules(tmp_path, monkeypatch):
    monkeypatch.setattr(SI, "SITES_DIR", str(tmp_path))
    (tmp_path / "bad.json").write_text(json.dumps({"robots": [{"name": "far too long a name"}]}))
    with pytest.raises(ValueError):
        SI.load("bad")


# ------------------------------------------------------------------------------------------------- the detector
@pytest.mark.parametrize("fam", ["tag16h5", "tag25h9", "aruco4x4"])
def test_detector_reads_drawn_tags(fam):
    det = TC.TagDetector(fam)
    img = np.full((300, 400), 150, np.uint8)
    t = TC.tag_image(fam, 1, px_per_cell=6, quiet_cells=1)
    img[50:50 + t.shape[0], 60:60 + t.shape[1]] = t
    ds = det.detect(img)
    assert [d.id for d in ds] == [1]


def test_window_search_reads_small_tag_the_whole_picture_misses():
    cam = CAMS.TagCamera((100, 80, 190), (100.01, 80, 6), size=(1280, 720),
                         hfov_deg=2 * math.degrees(math.atan(640 / (1.7 * 184))), blur_px=0.8, noise=3, seed=4,
                         table=(0, 0, 300, 200))
    img = cam.render([robot(100, 80, 30, 0)])
    det = TC.TagDetector("tag25h9", ids=[0, 1])
    u, v = cam.true_model().project((100, 80, 6.0))
    assert det.detect_window(img, u + 3, v - 2, 30, upscale=2.0)


# ------------------------------------------------------------------------------------------------- geometry
def test_pose_from_tag_corrects_the_height_parallax():
    cam = CAMS.TagCamera((100, 80, 190), (100.01, 80, 0), size=(1280, 960), hfov_deg=70, blur_px=0.0, noise=0,
                         jpeg=0, table=(0, 0, 300, 200))
    m = cam.true_model()
    import site_fit as SF
    P = SF.tag_corners_world(220.0, 140.0, 35.0, 6.0, 8.0)
    u, v = m.project(P)
    d = TC.Detection(0, np.column_stack([u, v]))
    p = TG.pose_from_detection(m, d, 6.0, 8.0)
    assert math.hypot(p.x - 220.0, p.y - 140.0) < 0.05 and abs(TG.wrap180(p.heading - 35.0)) < 0.2
    assert abs(p.size_ratio - 1.0) < 0.005
    flat = TG.pose_from_detection(m, d, 0.0, 8.0)          # read on the table's plane instead
    assert math.hypot(flat.x - 220.0, flat.y - 140.0) > 3.0


def test_simulated_picture_reads_back_the_pose():
    cam = CAMS.TagCamera((100, 80, 190), (100.01, 80, 0), size=(1280, 960), hfov_deg=70, seed=2, table=(0, 0, 300, 200))
    img = cam.render([robot(140, 60, -50, 1)])
    ds = TC.TagDetector("tag25h9", ids=[0, 1]).detect_full(img)
    assert len(ds) == 1
    p = TG.pose_from_detection(cam.true_model(), ds[0], 6.0, 8.0)
    assert math.hypot(p.x - 140, p.y - 60) < 0.6 and abs(TG.wrap180(p.heading + 50)) < 2.0


def test_fuse_weights_the_closer_camera():
    a = TG.TagPose(x=10.0, y=0.0, heading=0.0, sigma_cm=0.1, sigma_deg=1.0, size_ratio=1.0, cam="a", t=1.0, id=0,
                   px_per_cm=5.0)
    b = TG.TagPose(x=11.0, y=0.0, heading=10.0, sigma_cm=1.0, sigma_deg=1.0, size_ratio=1.0, cam="b", t=1.0, id=0,
                   px_per_cm=2.0)
    f, spread = TG.fuse([a, b])
    assert 10.0 < f.x < 10.05 and spread == pytest.approx(1.0) and 4.0 < f.heading < 6.0


# ------------------------------------------------------------------------------------------------- the site fit
def test_site_fit_two_cameras_over_the_table():
    import site_fit as SF
    obs, truth, st = SF.simulate(seed=2, n_spots=14, stickers=8)
    res = SF.full(obs)
    cmp_ = SF.compare(res, truth, st)
    assert abs(res["size_cm"][0] - 396.24) < 2.0 and abs(res["size_cm"][1] - 167.64) < 2.0
    assert max(v["worst_cm"] for v in cmp_.values() if isinstance(v, dict)) < 2.5
    assert max(v["median_cm"] for v in cmp_.values() if isinstance(v, dict)) < 1.0
    assert all(r["ok"] for r in res["checks"])


def test_site_fit_names_a_wrong_typed_size():
    import site_fit as SF
    obs, truth, st = SF.simulate(seed=3, n_spots=14, stickers=8)
    obs["table"] = {"length_cm": 396.24, "width_cm": 167.64 * 1.12}          # a tape 12 % off on the width
    res = SF.full(obs)
    assert any(not r["ok"] for r in res["checks"])
    assert any("off" in w or "%" in w for w in res["words"])


# ------------------------------------------------------------------------------------------------- production
def test_homes_equidistant_from_the_middle():
    h = PR.homes(396.0, 168.0, ["Alpha", "Bravo"])
    assert h["Alpha"][1] == h["Bravo"][1] == 84.0
    assert (198.0 - h["Alpha"][0]) == pytest.approx(h["Bravo"][0] - 198.0) == pytest.approx(PR.HOME_GAP_CM / 2)


def test_points_near_an_edge_are_moved_in_and_said():
    pts, words = PR.clamp_points({"1": [5, 80], "2": [200, 90]}, 396, 168)
    assert pts["1"][0] == PR.EDGE_IN_CM and pts["2"] == [200, 90]
    assert len(words) == 1 and "point 1" in words[0]


def test_production_program_plans_points_then_home_with_a_sync_spin():
    import tag_job as TJ
    m, _ = PR.site_map((396, 168), ["Alpha", "Bravo"], points={"1": [60, 40], "2": [330, 120], "3": [120, 130]})
    prog = TJ.add_sync_spin(PR.program({"Alpha": ["1", "3"], "Bravo": ["2"]}, dwell_s=2))
    mm = MS.build(m, prog)
    va = mm["tasks"]["Alpha"]["visit"]
    assert [v.get("label") for v in va[:2]] == ["sync spin", "sync spin"]
    assert va[-1]["label"] == "home" and va[-1]["goto"][:2] == mm["robots"][0]["start"][:2]
    assert sum(1 for v in va if v.get("wait_s") == 2) == 2
    plan = MS.plan_synced(mm, over={"separation_cm": 8.0}, t_limit_s=60)
    assert plan["summary"]["makespan_s"] > 0


def test_orders_naming_a_missing_point_are_refused():
    assert PR.check_orders({"Alpha": ["9"]}, {"1": [0, 0]})


# ------------------------------------------------------------------------------------------------- sync
def test_marks_and_timeline(tmp_path):
    f = str(tmp_path)
    ST.mark(f, "start", session="MIRLT-x", clock={"words": "test"})
    ST.mark(f, "end", session="MIRLT-x", clock={"words": "test"})
    assert [m["what"] for m in ST.read_marks(f)] == ["start", "end"]
    os.makedirs(os.path.join(f, "bridge"))
    os.makedirs(os.path.join(f, "run"))
    with open(os.path.join(f, "bridge", "fixes.csv"), "w") as fh:
        fh.write("t,robot,x,y,heading,sigma_cm,sigma_deg,cams,cams_apart_cm\n100.0,Alpha,1,2,90,0.3,1,overhead,0\n")
    with open(os.path.join(f, "run", "follow.csv"), "w") as fh:
        fh.write("robot,t_recv,run,ms,step_id,state,queued,last_done,last_failed,x_cm,y_cm,th_deg,pos_sd_cm,"
                 "th_sd_deg,fixes_used,fixes_rejected,front_cm,travel_cm,quiet_ms\n"
                 "Alpha,100.05,r,1,0,drive,0,0,0,1.1,2.1,89,0.4,1,0,0,100,0,0\n")
    p, counts = ST.export_timeline(f)
    rows = list(csv.DictReader(open(p)))
    assert [r["source"].split(" ")[0] for r in rows] == ["camera", "robot's"]
    assert ST.session_id("school-conference").startswith("MIRLT-")


def test_clock_status_reads_timedatectl_and_chrony():
    def fake(cmd):
        return "NTPSynchronized=yes\nNTP=yes\n" if cmd[0] == "timedatectl" else \
            "System time     : 0.000250000 seconds slow of NTP time\nLeap status     : Normal\n"
    import shutil
    real = shutil.which
    shutil.which = lambda n: "/usr/bin/" + n
    try:
        c = ST.clock_status(run=fake)
    finally:
        shutil.which = real
    assert c["ntp_synced"] is True and c["offset_ms"] == pytest.approx(-0.25)


# ------------------------------------------------------------------------------------------------- comparison
def test_similarity_fit_finds_a_mirror_rotation_and_units():
    rng = np.random.default_rng(1)
    ours = rng.uniform(0, 300, (6, 2))
    a = math.radians(70)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    theirs = ((ours @ R.T) + [40, -20]) * [1, -1]
    T = ME.fit_similarity(theirs, ours)
    assert T["mirror"] and T["rms"] < 1e-6 and T["s"] == pytest.approx(1.0)


def test_compare_with_synthetic_external_data(tmp_path):
    f = tmp_path / "run"
    os.makedirs(f / "run" / "sim")
    t0 = 1000.0
    rows = []
    with open(f / "run" / "sim" / "truth.csv", "w") as fh:
        fh.write("t,robot,x_cm,y_cm,heading_deg\n")
        for k in range(600):
            t = t0 + 0.1 * k
            for n, ph in (("Alpha", 0.0), ("Bravo", 2.0)):
                x, y = 200 + 80 * math.cos(0.05 * k + ph), 84 + 50 * math.sin(0.05 * k + ph)
                fh.write("%.2f,%s,%.3f,%.3f,0\n" % (t, n, x, y))
                rows.append((t, n, x, y))
    with open(f / "pose_timeline.csv", "w") as fh:
        w = csv.writer(fh)
        w.writerow(ST.TIMELINE_COLS)
        for t, n, x, y in rows:
            w.writerow([t, "", n, x, y, 0, "camera tag (overhead)", 0.3, 1])
    ST.mark(str(f), "start", clock={})
    ST.mark(str(f), "end", clock={})
    ms = ST.read_marks(str(f))
    for m in ms:                                   # marks inside the run's time
        m["t_unix"] = t0 + (5.0 if m["what"] == "start" else 50.0)
    with open(f / "sync_marks.jsonl", "w") as fh:
        fh.write("\n".join(json.dumps(m) for m in ms) + "\n")
    pts = {"1": (50.0, 30.0), "2": (350.0, 40.0), "3": (200.0, 150.0), "4": (100.0, 120.0)}
    th, rf = str(tmp_path / "theirs.csv"), str(tmp_path / "refs.csv")
    ME.synth_external(str(f), th, rf, pts, truth_csv=str(f / "run" / "sim" / "truth.csv"),
                      our_marks=[(m["t_unix"], m["what"]) for m in ms], noise_cm=3.0, outlier_pct=0)
    cfg = {"time": {"column": "timestamp", "format": "unix"},
           "robot": {"column": "tag", "map": {"tag-Alpha": "Alpha", "tag-Bravo": "Bravo"}},
           "x": {"column": "pos_x_ft"}, "y": {"column": "pos_y_ft"}, "units": "ft",
           "event": {"column": "note", "sync_values": ["start", "end"]}}
    s = ME.run(str(f), th, cfg, refs_csv=rf, our_points=pts, out=str(tmp_path / "cmp"), chart=False)
    assert s["clock_offset_s"] == pytest.approx(1503.25, abs=0.01)
    assert s["transform"]["mirror"] and all(r["median_cm"] < 6 for r in s["robots"].values())


# ------------------------------------------------------------------------------------------------- check-out
def test_checkout_passes_a_good_camera_and_fails_a_washed_out_slow_one():
    import camera_checkout as CO
    good = CAMS.TagCamera((100, 80, 190), (100.01, 80, 0), size=(1280, 960), hfov_deg=60, seed=3, table=(0, 0, 300, 200))
    m = good.true_model()
    pics = [(k / 12.0, good.render([robot(100, 80, 10, 0)])) for k in range(12)]
    r = CO.check_camera("overhead", pics, tags=TAGS, model=m, area=(0, 0, 200, 160), want_size=[1280, 960])
    assert {i["what"]: i["verdict"] for i in r["items"]}["picture rate"] == "PASS"
    assert r["numbers"]["tags_read_share"] == 1.0
    bad = CAMS.TagCamera((100, 80, 190), (100.01, 80, 0), size=(1280, 960), hfov_deg=60, seed=3, gain=2.6,
                         table=(0, 0, 300, 200))
    pics = [(k / 3.0, bad.render([robot(100, 80, 10, 0)])) for k in range(4)]
    r = CO.check_camera("overhead", pics, tags=TAGS, model=m, area=(0, 0, 200, 160), want_size=[1280, 960])
    v = {i["what"]: i["verdict"] for i in r["items"]}
    assert v["picture rate"] == "FAIL" and v["exposure"] in ("WARN", "FAIL")


# ------------------------------------------------------------------------------------------------- bridge, facing
def test_bridge_fuses_both_cameras_and_sends_fixes(tmp_path):
    import tag_bridge as TB
    cams = CAMS.overhead_pair(size=(1280, 960), height_cm=200.0, hfov_deg=74.0)
    models = {"overhead": cams[0].true_model(), "side": cams[1].true_model()}
    sent = []
    br = TB.Bridge(dict(TAGS), models, sent.append, str(tmp_path))
    rs = [robot(190, 84, 90, 0), robot(220, 84, 90, 1)]
    for k in range(6):
        t = 10.0 + 0.11 * k
        br.on_picture("overhead", cams[0].render(rs), t)
        br.on_picture("side", cams[1].render(rs), t)
    br.summary()
    br.close()
    a = [m for m in sent if m["robot"] == "Alpha"]
    assert a and all(abs(m["x"] - 190) < 1.0 and abs(m["y"] - 84) < 1.0 and abs(m["th"] - 90) < 3 for m in a)
    assert br.counts["Alpha"]["both"] >= 1


def test_facing_check_by_tags():
    import facing as FC
    cam = CAMS.TagCamera((100, 80, 190), (100.01, 80, 0), size=(1280, 960), hfov_deg=70, seed=6, table=(0, 0, 300, 200))
    img = cam.render([robot(90, 70, 180, 0), robot(130, 90, 92, 1)])

    class Grab:
        def between(self, c, t0, t1):
            return [img] * 3
    FC.TAG_LOOK_S = 0.0
    res = FC.tag_check(["Alpha", "Bravo"], Grab(), {"overhead": cam.true_model()}, TAGS,
                       assumed={"Alpha": 90.0, "Bravo": 90.0}, say=lambda s: None)
    ra, rb = res["robots"]["Alpha"], res["robots"]["Bravo"]
    assert abs(TG.wrap180(ra["meas"]["heading"] - 180)) < 2 and not ra["verdict"]["ok"]
    assert rb["verdict"]["ok"] and math.hypot(rb["meas"]["centre"][0] - 130, rb["meas"]["centre"][1] - 90) < 1.0


def test_tag_sheet_pdf(tmp_path):
    import tag_sheet as TS
    out = TS.make(str(tmp_path / "s.pdf"), [0, 1, 2], names={0: "Alpha", 1: "Bravo"})
    assert os.path.getsize(out) > 2000
    b = TS.bits("tag25h9", 0)
    assert b.shape == (7, 7) and b[0].all() and b[:, 0].all()
