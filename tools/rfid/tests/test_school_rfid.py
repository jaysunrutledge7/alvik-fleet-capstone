"""RFID pipeline (build e02-nov03-rfid-pipeline, 2026-11-03): their capture's columns, the clock (lower envelope and
the shift to the Pi), which tag is on which robot, the labels, the model's tests and the sticker comparison, end to end
on a made-up session and a fake capture (simulate_jay_csv.py). Offline only.

    python3 -m pytest tools/rfid/tests -q
"""
import csv
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
RFID = os.path.dirname(HERE)
for p in (RFID, os.path.join(os.path.dirname(RFID), "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

import rfid_model as RM            # noqa: E402
import school_rfid as R            # noqa: E402
import simulate_jay_csv as G       # noqa: E402

COLS = ["host_ts_unix", "host_ts_iso", "session_id", "condition", "point_x", "point_y", "orientation_deg", "epc",
        "robot_id", "antenna_id", "channel_index", "frequency_khz", "rssi_impinj_raw", "rssi_impinj_dbm",
        "rssi_llrp_dbm", "phase_raw", "phase_deg", "doppler_raw", "reader_ts_us", "tag_seen_count",
        "reader_first_ts_us"]


def _write(path, rows, cols=COLS):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})


# ------------------------------------------------------------------ a made-up session
POINTS = {"Alpha": [(40, 30), (90, 120), (60, 60), (150, 40), (30, 140), (120, 90), (170, 140), (80, 20),
                    (20, 90), (140, 130), (100, 50), (60, 150)],
          "Bravo": [(250, 40), (330, 130), (220, 100), (370, 30), (280, 150), (360, 90), (240, 20), (300, 70),
                    (380, 140), (210, 60), (340, 50), (260, 120)]}
T0 = 1793700000.0


def make_session(folder, speed=12.0, dwell=3.0, seed=0, wiggle=False):
    """A session folder as mission_run leaves it (the parts the pipeline reads): pose_timeline.csv (camera poses
    with a little noise, 10 a second), run/coordinator.jsonl (point_confidence at each 3 s stop), run/sim/truth.csv."""
    rng = np.random.default_rng(seed)
    os.makedirs(os.path.join(folder, "run", "sim"), exist_ok=True)
    truth, tl, ev = [], [], []
    wig = {}
    for rob, pts in POINTS.items():
        t, (x, y) = T0, (198.0 + (-15 if rob == "Alpha" else 15), 84.0)
        h = 90.0
        seq = [(t, x, y, h)]
        if wiggle:
            # the sync wiggle (3 Nov): 8 s still at home, quarter turn (1.5 s), 1 s stop, back, 1 s stop
            w0 = t + 8.0
            seq += [(w0, x, y, h), (w0 + 1.5, x, y, h + 90), (w0 + 2.5, x, y, h + 90), (w0 + 4.0, x, y, h),
                    (w0 + 5.0, x, y, h)]
            wig[rob] = {"start": w0, "end": w0 + 5.0}
            t = w0 + 5.0
        for k, (px, py) in enumerate(pts):
            d = math.hypot(px - x, py - y)
            h = math.degrees(math.atan2(py - y, px - x))
            n = max(2, int(d / speed / 0.1))
            for i in range(1, n + 1):
                seq.append((t + i * d / speed / n, x + (px - x) * i / n, y + (py - y) * i / n, h))
            t += d / speed
            x, y = px, py
            seq.append((t + dwell, x, y, h))
            ev.append({"t": t + dwell, "event": "point_confidence", "robot": rob, "point": "P%02d" % (
                k + 1 + (0 if rob == "Alpha" else 12)), "t_abs": t + dwell, "dwell_s": dwell, "x": x + 0.2,
                "y": y - 0.1, "plus_minus_cm": 1.1, "source": "two cameras"})
            t += dwell
        seq = np.array(seq)
        ts = np.arange(seq[0, 0], seq[-1, 0], 0.1)
        X, Y = np.interp(ts, seq[:, 0], seq[:, 1]), np.interp(ts, seq[:, 0], seq[:, 2])
        H = np.interp(ts, seq[:, 0], seq[:, 3])
        for a, b, c, d in zip(ts, X, Y, H):
            truth.append((a, rob, b, c, d))
            tl.append((a, rob, b + rng.normal(0, 0.3), c + rng.normal(0, 0.3), d))
    with open(os.path.join(folder, "run", "sim", "truth.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "x_cm", "y_cm", "heading_deg"])
        w.writerows(sorted(truth))
    with open(os.path.join(folder, "pose_timeline.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_unix", "t_iso", "robot", "x_cm", "y_cm", "heading_deg", "source", "sigma_cm", "sigma_deg"])
        for a, rob, b, c, d in sorted(tl):
            w.writerow(["%.4f" % a, "", rob, "%.2f" % b, "%.2f" % c, "%.2f" % d, "camera tag (overhead)", "0.5", "1"])
    with open(os.path.join(folder, "run", "coordinator.jsonl"), "w") as f:
        for e in sorted(ev, key=lambda e: e["t"]):
            f.write(json.dumps(e) + "\n")
    with open(os.path.join(folder, "sync_marks.jsonl"), "w") as f:
        f.write(json.dumps({"kind": "sync_mark", "what": "start", "t_unix": T0, "clock": {"ntp_synced": True}}) + "\n")
    if wig:
        with open(os.path.join(folder, "mission_summary.json"), "w") as f:
            json.dump({"sync_wiggle": wig}, f)
    with open(os.path.join(folder, "stickers.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "x_in", "y_in"])
        for rob, pts in POINTS.items():
            for k, (px, py) in enumerate(pts):
                lab = k + 1 + (0 if rob == "Alpha" else 12)
                dx = 6.0 if lab == 5 else 0.0          # one sticker typed 6 cm off
                w.writerow(["P%02d" % lab, (px + dx) / 2.54, py / 2.54])
    return folder


@pytest.fixture(scope="module")
def e2e(tmp_path_factory):
    d = tmp_path_factory.mktemp("rfid")
    ses = make_session(str(d / "session"))
    cap = str(d / "capture.csv")
    key = G.simulate(G.read_truth(ses), cap, mac_off_s=1.37, seed=11, n_still=6)
    out = str(d / "out")
    summ = R.run(ses, cap, out=out, search_s=20.0, doe=False, log=lambda *a: None)
    return ses, cap, key, out, summ


# ------------------------------------------------------------------ their columns
def test_load_capture_columns_and_fallbacks(tmp_path):
    p = str(tmp_path / "c.csv")
    _write(p, [
        {"host_ts_unix": "100.4", "epc": "e1", "antenna_id": "2", "channel_index": "7", "rssi_impinj_dbm": "-51.5",
         "phase_deg": "90", "reader_ts_us": "50000000", "tag_seen_count": "3", "reader_first_ts_us": "49800000"},
        {"host_ts_unix": "100.9", "epc": "E1", "antenna_id": "3", "channel_index": "8", "rssi_impinj_raw": "-6000",
         "phase_raw": "2048", "reader_ts_us": "50500000", "tag_seen_count": "", "reader_first_ts_us": ""},
        {"host_ts_unix": "101.0", "epc": "E2", "antenna_id": "1", "reader_ts_us": ""},       # no reader time
    ])
    c = R.load_capture(p)
    assert list(c["epc"]) == ["E1", "E1"]
    assert c["rssi"][0] == -51.5 and c["rssi"][1] == -60.0
    assert c["phase"][0] == pytest.approx(math.pi / 2) and c["phase"][1] == pytest.approx(math.pi)
    assert c["r_mid"][0] == pytest.approx(49.9) and c["span"][0] == pytest.approx(0.2)
    assert c["r_mid"][1] == pytest.approx(50.5)          # no first sighting: the last one
    assert c["n"][1] == 1 and c["dropped"] == 1 and c["has_first"] == 0.5


def test_load_capture_refuses_other_files(tmp_path):
    p = str(tmp_path / "x.csv")
    _write(p, [{"t": 1}], cols=["t", "robot"])
    with pytest.raises(SystemExit):
        R.load_capture(p)


def test_lower_envelope_finds_reader_offset():
    rng = np.random.default_rng(1)
    reader = np.sort(rng.uniform(0, 600, 5000))
    host = reader + 1500.25 + rng.uniform(0.003, 0.5, len(reader))
    host[100] -= 3.0                         # one stray row early: the median of block minima ignores it
    e = R.lower_envelope(host, reader)
    assert e["offset_s"] == pytest.approx(1500.25, abs=0.01)
    assert abs(e["drift_ms_per_min"]) < 1.0


# ------------------------------------------------------------------ our side
def test_pose_at_gap_and_own_fallback():
    cam = np.array([[0.0, 0, 0, 0, 0.5], [0.2, 2, 0, 0, 0.5], [2.0, 20, 0, 0, 0.5], [2.1, 21, 0, 0, 0.5]])
    own = np.array([[t, 10 * t, 1, 0, 1.0] for t in (0.0, 1.0, 2.0, 3.0)])
    pa = R.PoseAt({"cam": cam, "own": own})
    x, y, pm, src, _ = pa([0.1, 1.0, 2.05, 5.0])
    assert x[0] == pytest.approx(1.0) and src[0] == 1 and pm[0] == pytest.approx(1.0)
    assert src[1] == 2 and y[1] == pytest.approx(1.0)      # camera gap 1.8 s > 0.5 s: the robot's own pose
    assert src[2] == 1
    assert src[3] == 0 and math.isnan(x[3])


def test_labels_zones_and_names():
    assert R.norm_label("point 7") == "P07" == R.norm_label("P7") == R.norm_label("7")
    assert list(RM.zone_of([0, 38.9 * 2.54, 39 * 2.54, 100 * 2.54, 156 * 2.54])) == [1, 1, 2, 3, 4]


def test_knn_predicts_neighbours():
    X = np.array([[0.0], [1.0], [10.0], [11.0]])
    Y = np.array([[0, 0], [0, 0], [100, 50], [100, 50]])
    p = RM.knn_predict(X, Y, np.array([[10.5]]), k=2)
    assert p[0] == pytest.approx([100, 50])


def test_sticker_compare_and_frame_check():
    st = {"P01": (10.0, 10.0), "P02": (100.0, 20.0), "P03": (50.0, 80.0), "P04": (200.0, 60.0)}
    stands = [{"robot": "Alpha", "point": k, "x": v[0] + 5.0, "y": v[1], "pm": 1.0, "source": "two cameras"}
              for k, v in st.items()]
    rows, fit = R.sticker_compare(stands, st, used={"P02": (100.0, 40.0)})
    assert all(r["dist_cm"] == pytest.approx(5.0) for r in rows)
    assert fit["mean_offset_cm"][0] == pytest.approx(5.0) and abs(fit["turn_deg"]) < 0.01
    assert rows[1]["site_override"] and not rows[0]["site_override"]


# ------------------------------------------------------------------ end to end on a fake capture
def test_e2e_finds_tags_and_clock(e2e):
    ses, cap, key, out, s = e2e
    assert s["epc_to_robot"] == key["epc_to_robot"]
    true_shift = key["reader_to_pi_s"] - s["reader_to_laptop"]["offset_s"]
    assert s["reader_to_pi_s"] == pytest.approx(key["reader_to_pi_s"], abs=0.3)
    lo, hi = s["shift_95_s"]
    assert lo - 0.1 <= true_shift <= hi + 0.1
    assert abs(s["motion_check_shift_s"] - true_shift) < 1.0
    for r, v in s["tag_scores"].items():
        assert v["score"] > v["next_best"] + 0.05


def test_e2e_labels_and_outputs(e2e):
    ses, cap, key, out, s = e2e
    for f in ("report.md", "summary.json", "labelled_rows.csv", "windows_1s.csv", "windows_3s.csv",
              "stickers_vs_camera.csv", "clock.png", "errors.png", "stickers.png"):
        assert os.path.exists(os.path.join(out, f)), f
    rows = list(csv.DictReader(open(os.path.join(out, "labelled_rows.csv"))))
    states = {r["state"] for r in rows}
    assert {"stand", "moving"} <= states
    # standing rows sit on their stop (the made-up camera noise is 0.3 cm)
    pts = {"P%02d" % (k + 1 + (0 if rob == "Alpha" else 12)): xy for rob, v in POINTS.items() for k, xy in enumerate(v)}
    d = [math.hypot(float(r["x_cm"]) - pts[r["point"]][0], float(r["y_cm"]) - pts[r["point"]][1])
         for r in rows if r["state"] == "stand"]
    assert np.median(d) < 1.5
    # the typed-off sticker is listed
    rep = open(os.path.join(out, "report.md")).read()
    assert "| P05 | Alpha |" in rep
    assert s["stickers"]["n"] == 24
    # nothing written into the session apart from the output folder (here outside it)
    assert sorted(os.listdir(ses)) == ["pose_timeline.csv", "run", "stickers.csv", "sync_marks.jsonl"]


def test_e2e_model_beats_table_centre_on_points_held_out(e2e):
    s = e2e[4]
    q = s["model"]["by_window"]["3s"]["schemes"]["points"]
    assert q["model"]["all"]["median_cm"] < q["centre"]["all"]["median_cm"]
    assert set(s["model"]["by_window"]) == {"1s", "3s"}
    assert set(s["model"]["by_window"]["3s"]["schemes"]) == {"zones", "points"}


def test_latest_picks_newest_run_with_a_log(tmp_path, monkeypatch):
    base = tmp_path / "logs" / "mission-control-school-conference" / "runs"
    for name, has_log in (("20261103-1000-production", True), ("20261103-1100-production", True),
                          ("20261103-1200-feel", False)):
        d = base / name / "run"
        d.mkdir(parents=True)
        if has_log:
            (d / "coordinator.jsonl").write_text('{"event": "point_confidence"}\n')
        else:
            (d / "coordinator.jsonl").write_text('{"event": "labelled_step"}\n')     # a feel-out: no stops
        os.utime(str(base / name), (1e9 + len(name) + int(name[9:13]), 1e9 + int(name[9:13])))
    monkeypatch.setattr(R, "REPO", str(tmp_path))
    assert os.path.basename(R.latest_run()) == "20261103-1100-production"


# ------------------------------------------------------------------ the sync wiggle as the clock (3 Nov)
def test_wiggle_anchor_recovers_the_clock_at_least_as_well(tmp_path):
    ses = make_session(str(tmp_path / "session"), wiggle=True)
    errs = {"wiggle": [], "route": []}
    for seed, mac in ((11, 1.37), (12, -2.4), (13, 0.6)):
        cap = str(tmp_path / ("capture%d.csv" % seed))
        key = G.simulate(G.read_truth(ses), cap, mac_off_s=mac, seed=seed, n_still=6)
        s = R.run(ses, cap, out=str(tmp_path / ("w%d" % seed)), search_s=10.0, doe=False, charts=False,
                  log=lambda *a: None)
        assert s["clock_anchor"] == "sync wiggle" and set(s["wiggle_per_robot_s"]) == {"Alpha", "Bravo"}
        errs["wiggle"].append(abs(s["reader_to_pi_s"] - key["reader_to_pi_s"]))
        s2 = R.run(ses, cap, out=str(tmp_path / ("r%d" % seed)), search_s=10.0, doe=False, charts=False,
                   log=lambda *a: None, use_wiggle=False)
        assert s2["clock_anchor"] == "route fit"
        errs["route"].append(abs(s2["reader_to_pi_s"] - key["reader_to_pi_s"]))
        lo, hi = s["shift_95_s"]
        true_shift = key["reader_to_pi_s"] - s["reader_to_laptop"]["offset_s"]
        assert lo - 0.05 <= true_shift <= hi + 0.05
    assert max(errs["wiggle"]) <= max(max(errs["route"]), 0.15), errs
