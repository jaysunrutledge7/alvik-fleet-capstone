"""(11-05h) The school calibration of 5 Nov 19:34 (AL-1DC0, firmware 2026-10-29b): the wheel_scale fit was right and a
fixed overrun per leg read as '+1 cm/m after the fit'. These tests refit phase A of the real legs and predict phase C."""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import calib_run as CR      # noqa: E402
import calib_report as CRP  # noqa: E402

REAL = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "docs", "results",
                    "calib-school-2026-11-05-1934", "legs.csv")


def _legs(phase):
    out = []
    for r in csv.DictReader(open(REAL)):
        if r["kind"] != "go" or r["result"] != "done" or r["phase"] != phase or r["interrupted"] == "1":
            continue
        v = float(r["commanded"])
        sp = CR.FINAL_CMS if r["target"].endswith("final") else CR.SPEED_CMS
        out.append(dict(bel=float(r["bel_len_cm"]), cam=float(r["cam_len_cm"]), v=v, speed=sp,
                        along=float(r["along_err_cm"]), target=r["target"]))
    return out


def test_real_fit_scale_matches_phase_c():
    A, C = _legs("A"), _legs("C")
    rolls = [(q["bel"], q["cam"]) for q in A if q["v"] >= 15]
    over = [(q["bel"], q["cam"], q["speed"], q["v"]) for q in A if q["v"] >= 4]
    f = CR.fit_values([], [], rolls, dict(CR.BASELINE_DEFAULTS), overrun_rows=over)
    assert abs(f["wheel_scale"] - 0.9759) < 0.001          # the real run's fit, unchanged
    # phase C's own legs (driven with 0.9759) say the true scale is 0.9765: the fit did not overshoot
    b = np.array([q["bel"] / 0.9759 for q in C if q["v"] >= 15])
    c = np.array([q["cam"] for q in C if q["v"] >= 15])
    k = np.polyfit(b, c, 1)[0]
    assert abs(k - f["wheel_scale"]) < 0.002
    # the overrun per speed predicted from phase A matches what phase C measured
    for sp in (5.0, 10.0):
        meas = np.mean([q["along"] for q in C if q["speed"] == sp])
        assert abs(f["overrun_cm"]["%g" % sp] - meas) < 0.06, (sp, f["overrun_cm"], meas)


def test_report_splits_scale_and_offset():
    rows = [r for r in csv.DictReader(open(REAL))]
    g = CRP.groups(rows)
    mc = CRP.metrics([r for k in g if k.startswith("C") for r in g[k]])
    ma = CRP.metrics(g["A1"])
    assert mc["dist_err_signed_cm_per_m_mean"] > 0.8            # what looked like an overshoot ...
    assert abs(mc["dist_scale_cm_per_m"]) < 0.3                 # ... has no scale left after the fit
    assert 0.3 < mc["dist_offset_cm"] < 0.7                     # ... only a fixed overrun per leg
    assert ma["dist_scale_cm_per_m"] < -1.8                     # phase A: the real -2.4 % scale


def test_lead_value():
    lead = {5.0: 0.32, 10.0: 0.49}
    assert CR.lead_value(50.0, 10.0, lead) == 49.51
    assert CR.lead_value(5.0, 5.0, lead) == 4.68
    assert CR.lead_value(0.5, 5.0, lead) == 0.5          # tiny leg: as asked
    assert CR.lead_value(50.0, 10.0, None) == 50.0
    assert CR.lead_value(50.0, 10.0, {10.0: 3.0}) == 50.0  # not believed


def test_lead_removes_the_offset_on_real_c_legs():
    """Each phase C leg re-driven with the lead: real end = (sent - shortfall) + coast(v); the along error is gone."""
    A, C = _legs("A"), _legs("C")
    over = [(q["bel"], q["cam"], q["speed"], q["v"]) for q in A if q["v"] >= 4]
    f = CR.fit_values([], [], [(q["bel"], q["cam"]) for q in A if q["v"] >= 15], dict(CR.BASELINE_DEFAULTS),
                      overrun_rows=over)
    lead = {float(k): v for k, v in f["overrun_cm"].items()}
    errs = []
    for q in C:
        sent = CR.lead_value(q["v"], q["speed"], lead)
        errs.append(q["along"] - (q["v"] - sent))      # the same overrun, the leg sent shorter
    assert abs(np.mean(errs)) < 0.05
