"""Reference AprilTags on the table (2026-11-03): the layout, the site fit from the tags alone (no clicks), a corner
tag hidden, a paper tag turned on its square, swapped ids, the simulated cameras drawing them, the setup through the
simulator, a bumped camera, keep-out tiles, the corner-click switch, the paper PDF, the STL files and the print sheet.
Simulator pictures only; no robot, no Pi."""
import json
import math
import os
import struct
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment"),
          os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import ref_tags as RT        # noqa: E402
import site_fit as SF        # noqa: E402
import sites as SI           # noqa: E402
import tag_cam as CAMS       # noqa: E402

L, W = 396.24, 167.64


def _worst(res, truth, st):
    cmp_ = SF.compare(res, truth, st)
    return max(v["worst_cm"] for v in cmp_.values() if isinstance(v, dict))


# ------------------------------------------------------------------------------------------------- layout
def test_layout_corner_one_is_the_origin_and_ids_are_not_robots():
    p = RT.world_poses(RT.default_config(), L, W)
    assert p["20"][:2] == (RT.OFFSET_CM, RT.OFFSET_CM) and p["20"][2] == 90.0
    assert p["22"][:2] == pytest.approx((L - RT.OFFSET_CM, W - RT.OFFSET_CM))
    assert p["24"][1] == RT.OFFSET_CM and p["25"][1] == pytest.approx(W - RT.OFFSET_CM)
    s = SI.load("school-conference")
    rids = set(RT.ids(s["ref_tags"]))
    assert rids == {20, 21, 22, 23, 24, 25}
    assert not rids & (set(s["tags"]["ids"].values()) | set(s["tags"]["spares"]))
    assert s["corner_clicks"] is False and SI.load("home").get("corner_clicks") is True


def test_site_refuses_a_reference_id_that_is_a_robot(tmp_path, monkeypatch):
    d = json.load(open(os.path.join(FLEET, "sites", "school-conference.json")))
    d["ref_tags"]["corners"]["1"] = 0
    (tmp_path / "x.json").write_text(json.dumps(d))
    monkeypatch.setattr(SI, "SITES_DIR", str(tmp_path))
    with pytest.raises(ValueError) as e:
        SI.load("x")
    assert "also robot tags" in str(e.value)


# ------------------------------------------------------------------------------------------------- the fit
def test_fit_from_reference_tags_alone_no_clicks():
    obs, truth, st = SF.simulate(seed=1, n_spots=8, refs=True, corners=False)
    assert not any(obs["corners"].values())
    # (e07) the default measure_trust "auto" lets the cameras win when the tiles are read: in this geometric simulator
    # the typed size is the exact truth, so the tape-trusted fit stays the tighter one here (< 0.6 cm); the cameras'
    # own (the tiles' printed size the ruler) within 1 cm. With real blur the cameras win (measure_bench, e07 report)
    res_t = SF.full(dict(obs, measure_trust="typed"))
    assert _worst(res_t, truth, st) < 0.6
    res = SF.full(obs)
    assert abs(res["size_cm"][0] - L) < 0.5 and abs(res["size_cm"][1] - W) < 0.5
    assert _worst(res, truth, st) < 1.0
    al = res["ref_tags"]["along_cm"]
    for i, v in obs["ref_tags"]["truth_along"].items():
        assert abs(al[i] - v) < 0.5
    assert any("placed from" in w and "reference tag" in w for w in res["words"])
    assert "height_check" in res


def test_fit_with_corner_one_hidden_keeps_the_frame():
    obs, truth, st = SF.simulate(seed=2, n_spots=8, refs=True, corners=False, hide_ref=(20,))
    res = SF.full(obs)
    assert _worst(res, truth, st) < 0.8
    assert abs(res["ref_tags"]["poses"]["20"][0] - RT.OFFSET_CM) < 0.5        # where corner 1's tag would be


def test_fit_without_robots_or_typed_size():
    # flat tags alone, nothing typed: the table right to about 1 cm; the worst point (at the roofs' 6 cm height) is
    # looser, since flat tags cannot tell a straight-down camera's height from its zoom (the roof tags do)
    obs, truth, st = SF.simulate(seed=3, n_spots=0, refs=True, corners=False, typed=False)
    res = SF.full(obs)
    assert abs(res["size_cm"][0] - L) < 1.5 and _worst(res, truth, st) < 2.0


def test_paper_tag_turned_on_its_square_is_read():
    obs, truth, st = SF.simulate(seed=4, n_spots=6, refs=True, corners=False)
    # tag 23's paper turned a quarter: its corners come one place round
    for c, rd in obs["ref_tags"]["reads"].items():
        if "23" in rd:
            rd["23"] = rd["23"][1:] + rd["23"][:1]
    res = SF.full(obs)
    assert res["ref_tags"]["quarters"]["23"] in (1, 3)
    assert _worst(res, truth, st) < 0.8
    assert any("turned" in w for w in res["words"])


def test_swapped_corner_tiles_are_named():
    obs, truth, st = SF.simulate(seed=5, n_spots=6, refs=True, corners=False)
    for c, rd in obs["ref_tags"]["reads"].items():          # 21 and 22 swapped on the table
        a, b = rd.pop("21", None), rd.pop("22", None)
        if a is not None:
            rd["22"] = a
        if b is not None:
            rd["21"] = b
    res = SF.full(obs)
    assert any("do not sit where their ids say" in w for w in res["words"])


def test_old_corner_clicks_still_fit_without_reference_tags():
    obs, truth, st = SF.simulate(seed=2, n_spots=14, stickers=8)
    assert "ref_tags" not in obs
    assert _worst(SF.full(obs), truth, st) < 2.5


# ------------------------------------------------------------------------------------------------- simulator
def test_simulated_cameras_draw_reference_tags_and_hide_some():
    cfg = CAMS.default_config(height_cm=175.0, hide_refs=["20"])
    cams = CAMS.cams_from_config(cfg)
    det = RT.detector()
    seen = set()
    for c in cams:
        seen |= {d.id for d in det.detect_full(c.render([]))}
    assert 20 not in seen and {22, 23, 24, 25} <= seen
    assert CAMS.default_config(refs=False).get("ref_tags") is None


def test_setup_through_the_simulator_tags_only(tmp_path):
    import tag_setup as TSU
    rc = TSU.main(["--dry", "--no-corner-clicks", "--out", str(tmp_path)])
    assert rc == 0
    tc = json.load(open(tmp_path / "tagcal" / "truth_check.json"))
    assert max(tc["overhead"]["worst_cm"], tc["side"]["worst_cm"]) < 0.8
    assert all(abs(v) < 0.5 for v in tc["side_tags_along_off_cm"].values())
    obs = json.load(open(tmp_path / "tagcal" / "obs.json"))
    assert not any(obs["corners"].values()) and obs["ref_tags"]["reads"]


# ------------------------------------------------------------------------------------------------- bumped camera
def test_ref_watch_still_then_bumped():
    import tag_bridge as TB
    cfg = CAMS.default_config(height_cm=175.0)
    cam = CAMS.cams_from_config(cfg)[0]
    exp = {}
    for i, (x, y, th) in cfg["ref_tags"]["poses"].items():
        uv = cam.project(SF.tag_corners_world(x, y, th, 0.18, RT.TAG_CM))
        if np.all(np.isfinite(uv)) and np.all((uv[:, 0] > 2) & (uv[:, 0] < cam.W - 2) & (uv[:, 1] > 2)
                                              & (uv[:, 1] < cam.H - 2)):
            exp[i] = uv
    w = TB.RefWatch("overhead", exp, RT.detector(), every_s=0.0)
    g = cam.render([])
    med, sh, v = w.check(g, 1.0)
    assert v == "ok" and med < 1.0 and len(sh) == len(exp)
    cam.bump(0.6)
    g = cam.render([])
    assert w.check(g, 2.0)[2] == "over"
    med, sh, v = w.check(g, 3.0)
    assert v == "moved" and med > RT.BUMP_PX


def test_bridge_reads_expected_reference_tags_from_the_setup(tmp_path):
    from cameras import CameraModel
    cfg = CAMS.default_config(height_cm=175.0)
    cams = CAMS.cams_from_config(cfg)
    fit = {"ref_tags": {"poses": cfg["ref_tags"]["poses"], "tag_cm": 12.0, "z_cm": 0.18,
                        "seen": {"overhead": ["20", "23", "24", "25"]}}}
    p = tmp_path / "site_fit.json"
    p.write_text(json.dumps(fit))
    import tag_bridge as TB
    exp = TB.ref_expected({"overhead": cams[0].true_model(), "side": cams[1].true_model()}, str(p))
    assert set(exp["overhead"]) == {"20", "23", "24", "25"} and exp.get("side")


# ------------------------------------------------------------------------------------------------- the map
def test_tiles_are_keep_outs_and_points_move_off_them():
    import production as PR
    poses = RT.world_poses(RT.default_config(), L, W)
    kos = RT.keepouts(poses)
    assert len(kos) == 6
    m, words = PR.site_map((L, W), ["Alpha", "Bravo"], points={"1": [L / 2, 18.0], "2": [100, 80]}, keepouts=kos)
    assert len(m["keepouts"]) == 6
    assert m["spots"]["1"][1] > 2 * RT.OFFSET_CM + RT.KEEPOUT_MARGIN_CM + 5
    assert m["spots"]["2"] == [100.0, 80.0]
    assert any("reference tile" in w for w in words)


# ------------------------------------------------------------------------------------------------- the switch
def test_corner_clicks_switch_per_site_and_on_the_page(tmp_path):
    import tag_job as TJ
    import mission_run as MR

    class Sess:
        dir = str(tmp_path)
    s = SI.load("school-conference")
    s = dict(s, conf_dir=str(tmp_path / "conf"))
    tj = TJ.TagJob(Sess(), s)
    assert tj.corner_clicks_on() is False and tj.ref_cfg
    tj.set_corner_clicks(True)
    assert TJ.TagJob(Sess(), s).corner_clicks_on() is True
    a = MR.argparse.Namespace(corner_clicks="auto", site_=s)
    assert MR.corner_clicks(a) is False
    a.site_ = SI.load("home")
    assert MR.corner_clicks(a) is True
    a.corner_clicks = "off"
    assert MR.corner_clicks(a) is False


# ------------------------------------------------------------------------------------------------- print
def test_paper_pdf_prints_tags_at_scale(tmp_path):
    import subprocess
    import cv2
    pdf = RT.pdf(str(tmp_path / "r.pdf"))
    r = subprocess.run(["pdftoppm", "-r", "100", "-f", "1", "-l", "1", "-gray", "-png", pdf, str(tmp_path / "p")],
                       capture_output=True)
    if r.returncode != 0:
        pytest.skip("pdftoppm not available")
    img = cv2.imread(str(tmp_path / "p-1.png"), cv2.IMREAD_GRAYSCALE)
    d = [x for x in RT.detector().detect_full(img) if x.id == 20]
    assert d and abs(d[0].size_px - 12.0 / 2.54 * 100) < 4          # 12 cm at 100 dpi = 472 px


def _stl_size(path):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    if data[:5] == b"solid" and len(data) != 84 + 50 * n:
        import re
        v = np.array([[float(t) for t in m.split()] for m in re.findall(rb"vertex\s+([^\n]+)", data)])
    else:
        a = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        v = a["v"].reshape(-1, 3).astype(float)
    return v.max(0) - v.min(0)


def test_stl_files_and_print_sheet():
    stl = os.path.join(REPO, "hardware", "dock", "stl")
    for i in (20, 21, 22, 23):
        assert np.allclose(_stl_size(os.path.join(stl, "ref_corner_%d.stl" % i)), [158, 158, 13.8], atol=0.1)
    for i in (24, 25):
        assert np.allclose(sorted(_stl_size(os.path.join(stl, "ref_side_%d.stl" % i))), [13.8, 155, 158], atol=0.1)
    for i in (0, 1):
        assert np.allclose(_stl_size(os.path.join(stl, "roof_tag_%d.stl" % i)), [91, 91, 1.8], atol=0.1)
    from pypdf import PdfReader
    r = PdfReader(os.path.join(REPO, "hardware", "PRINT_SHEET.pdf"))
    assert len(r.pages) == 2 and "ref_corner_20" in r.pages[1].extract_text()


def test_printed_raised_cells_read_like_paper():
    import ref_printcheck as RP
    cfg = CAMS.default_config(height_cm=175.0, refs=False)
    cam = CAMS.cams_from_config(cfg)[1]
    x, y, th = RT.world_poses(RT.default_config(), *cfg["table"])["22"]
    truth = cam.project(SF.tag_corners_world(x, y, th, 0.18, RT.TAG_CM))
    ppc = cam.px_per_cm(x, y, 0.18)
    for mode in ("flat", "nozzle", "raised_shadow30"):
        d = [q for q in RT.detector().detect_full(RP.render(cam, dict(id=22, x=x, y=y, th=th), mode)) if q.id == 22]
        assert d, mode
        assert np.linalg.norm(d[0].center - truth.mean(0)) / ppc < 0.2, mode       # under 2 mm on the table
