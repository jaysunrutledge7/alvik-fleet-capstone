#!/usr/bin/env python3
"""e09 (Phase B2/B3): the live filter (tools/fleet/fused_pose.py) replayed on a REAL free-roam session's logs, against
the way positions are kept today. Logs only: reads raw.csv, follow.csv and camera_track.csv of a session folder.

For each robot and each camera-gap length L, gaps of L seconds are cut out of the camera's track (one every L + 30 s,
starting where the robot drives and the camera has it); at each gap's end, where each method puts the robot is compared
with where the camera saw it then (the camera is the reference: ~1 cm, worse at open edges):

  fused        the filter, fed every RAW row (wheels, raw gyro rate, accelerometer, line sensors) all along and the
               camera's fixes except in the gap (fused_pose.FusedPose)
  blend        today: the camera's last spot before the gap moved on by the ROBOT'S OWN pose (re-made from RAW the
               firmware's way: wheel distance x wheel_scale along the library yaw x gyro_scale, from the robot's own
               heading at the gap's start: FOLLOW th_deg). The real own pose cannot be used as it is: in the real run it
               kept taking the camera's fixes through the "gap"
  gyro (B3)    the same wheel distance along the heading integrated from the raw gyro rate minus the bias taken at the
               latest standstill, from the same start heading as blend

and, for the uncertainty: how often the fused error falls inside the filter's own K_SIGMA x sigma (its coverage).

    python3 tools/analysis/fused_replay.py <session folder> [--minutes 90] [--gaps 5,10,20,40] [--out f.json]
"""
import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "fleet"), TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np   # noqa: E402

import fused_pose as FP   # noqa: E402
import raw_units as RU    # noqa: E402

WHEEL_R_CM = 1.7          # the library's wheel radius (raw_units.robot_columns)


def fnum(v, d=float("nan")):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def load(folder, minutes=None, robots=None):
    """{robot: dict(raw=[...], cam=[(t, x, y, th)], fol=[(t, th)])} on the Pi's clock."""
    out = {}
    t_end = None
    cam_rows = list(csv.DictReader(open(os.path.join(folder, "camera_track.csv"))))
    t0 = min(fnum(r["t"]) for r in cam_rows)
    if minutes:
        t_end = t0 + minutes * 60.0
    for r in cam_rows:
        t = fnum(r["t"])
        if t_end and t > t_end:
            continue
        if robots and r["robot"] not in robots:
            continue
        out.setdefault(r["robot"], dict(raw=[], cam=[], fol=[]))["cam"].append(
            (t, fnum(r["x_cm"]), fnum(r["y_cm"]), fnum(r["th_deg"], None)))
    offs = {}
    keep = ("ms", "t_recv", "v_cms", "w_dps", "gz", "yaw_deg", "wl_deg", "wr_deg", "wl_rpm", "wr_rpm", "ax", "lineL",
            "lineC", "lineR", "dC_cm")
    with open(os.path.join(folder, "raw.csv")) as fh:
        for r in csv.DictReader(fh):
            n = r["robot"]
            if n not in out:
                continue
            ms, tr = fnum(r["ms"]), fnum(r["t_recv"])
            o = tr - ms / 1000.0
            offs[n] = o if n not in offs else min(offs[n], o)       # lower envelope: the quickest packet
            if t_end and tr > t_end + 5:
                continue
            out[n]["raw"].append({k: fnum(r.get(k)) for k in keep})
    with open(os.path.join(folder, "follow.csv")) as fh:
        for r in csv.DictReader(fh):
            n = r["robot"]
            if n in out and r.get("state") not in ("nopose",):
                out[n]["fol"].append((fnum(r["t_recv"]), fnum(r["th_deg"])))
    for n, d in out.items():
        o = offs.get(n, 0.0)
        for row in d["raw"]:
            row["t"] = row["ms"] / 1000.0 + o
        d["raw"].sort(key=lambda q: q["t"])
        d["cam"].sort()
    return out


def params(folder):
    """gyro_scale / wheel_scale each robot used (param.csv), else the 5 Nov values (1.075, 0.979)."""
    p = {}
    f = os.path.join(folder, "param.csv")
    if os.path.exists(f):
        for r in csv.DictReader(open(f)):
            if r["name"] in ("gyro_scale", "wheel_scale"):
                p.setdefault(r["robot"], {})[r["name"]] = fnum(r["value"])
    return p


def replay(d, L, gs=1.075, ws=0.979, every=None, k_sigma=FP.K_SIGMA, gz_scale=1.0):
    raw, cam = d["raw"], d["cam"]
    if len(raw) < 100 or len(cam) < 50:
        return []
    every = every or (L + 30.0)
    t_first = cam[0][0] + 20.0
    gaps = []
    t = t_first
    while t + L < cam[-1][0]:
        gaps.append((t, t + L))
        t += every
    fp = FP.FusedPose("r", learn=True)
    gh = FP.GyroHeading(scale=gz_scale)
    ci, gi = 0, 0
    res = []
    active = None          # dict for the gap in progress
    prev = None
    fol = d["fol"]
    fi = 0
    fj = 0

    def own_th(t):
        nonlocal fi
        while fi + 1 < len(fol) and fol[fi + 1][0] <= t:
            fi += 1
        return fol[fi][1] if fol and fol[fi][0] <= t + 0.2 else None

    def cam_at(t, tol=0.12):
        lo, hi = 0, len(cam) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if cam[m][0] <= t:
                lo = m
            else:
                hi = m
        best = min((cam[lo], cam[hi]), key=lambda c: abs(c[0] - t))
        return best if abs(best[0] - t) <= tol else None

    for row in raw:
        t = row["t"]
        # camera fixes up to t (except inside a gap)
        while ci < len(cam) and cam[ci][0] <= t:
            c = cam[ci]
            ci += 1
            if active is not None or any(a <= c[0] < b for a, b in gaps[gi:gi + 1]):
                continue
            fp.fix(c[0], c[1], c[2], None, 1.0, "motion")
        while fj < len(fol) and fol[fj][0] <= t:      # the robot's own heading's turn (FOLLOW), as live
            if fp.started and fol[fj][1] == fol[fj][1]:
                fp.own_turn(fol[fj][0], fol[fj][1])
            fj += 1
        v, w = RU.speed_cms(row["v_cms"]), RU.turn_dps(row["w_dps"])
        still = abs(row["wl_rpm"]) < FP.STILL_RPM and abs(row["wr_rpm"]) < FP.STILL_RPM
        fp.raw(t, v, w, row["gz"], still, row["ax"], row["dC_cm"], (row["lineL"], row["lineC"], row["lineR"]))
        gh.feed(t, row["gz"], still, w)
        if prev is not None and active is not None:
            dwl, dwr = row["wl_deg"] - prev["wl_deg"], row["wr_deg"] - prev["wr_deg"]
            ds = (dwl + dwr) / 2.0 * math.pi / 180.0 * WHEEL_R_CM * ws
            if abs(ds) < 5.0:
                dy = math.atan2(math.sin(math.radians(row["yaw_deg"] - prev["yaw_deg"])),
                                math.cos(math.radians(row["yaw_deg"] - prev["yaw_deg"])))
                a = active
                a["own_th"] += math.degrees(dy) * gs
                a["ox"] += ds * math.cos(math.radians(a["own_th"]))
                a["oy"] += ds * math.sin(math.radians(a["own_th"]))
                gth = gh.heading_deg()
                if gth is not None and a.get("g0") is not None:
                    hd = a["g_th0"] + FP.wrap180(gth - a["g0"])
                else:
                    hd = a["own_th"]
                a["gx"] += ds * math.cos(math.radians(hd))
                a["gy"] += ds * math.sin(math.radians(hd))
                a["dist"] += abs(ds)
                a["turn"] += abs(math.degrees(dy) * gs)
        prev = row
        # gap bookkeeping
        if active is None and gi < len(gaps) and t >= gaps[gi][0]:
            c0 = cam_at(gaps[gi][0], 0.3)
            th0 = own_th(t)
            if c0 is None or th0 is None or not fp.started:
                gi += 1
                continue
            active = dict(t0=t, t1=gaps[gi][1], c0=c0, own_th=th0, ox=c0[1], oy=c0[2], gx=c0[1], gy=c0[2],
                          g0=gh.heading_deg(), g_th0=th0, dist=0.0, turn=0.0)
            gh.set(th0)
            active["g0"] = gh.heading_deg()
        elif active is not None and t >= active["t1"]:
            c1 = cam_at(active["t1"], 0.25)
            p = fp.pose(t)
            if c1 is not None and p is not None and active["dist"] >= 1.0:
                ef = math.hypot(p[0] - c1[1], p[1] - c1[2])
                res.append(dict(L=L, t=round(active["t0"], 2), dist_cm=round(active["dist"], 1),
                                turn_deg=round(active["turn"], 1), fused=round(ef, 2),
                                blend=round(math.hypot(active["ox"] - c1[1], active["oy"] - c1[2]), 2),
                                gyro=round(math.hypot(active["gx"] - c1[1], active["gy"] - c1[2]), 2),
                                unc=round(fp.unc_cm(t) or 0, 2), sigma=round(fp.sigma_xy(t) or 0, 2)))
            active = None
            gi += 1
    res_sum = fp.summary()
    for r in res:
        r["_sum"] = res_sum
    return res


def table(rows):
    out = {}
    for L in sorted({r["L"] for r in rows}):
        rs = [r for r in rows if r["L"] == L]
        if not rs:
            continue
        out[L] = dict(n=len(rs), fused=round(float(np.median([r["fused"] for r in rs])), 2),
                      blend=round(float(np.median([r["blend"] for r in rs])), 2),
                      gyro=round(float(np.median([r["gyro"] for r in rs])), 2),
                      fused_p90=round(float(np.percentile([r["fused"] for r in rs], 90)), 2),
                      blend_p90=round(float(np.percentile([r["blend"] for r in rs], 90)), 2),
                      gyro_p90=round(float(np.percentile([r["gyro"] for r in rs], 90)), 2),
                      covered=round(100.0 * sum(1 for r in rs if r["fused"] <= r["unc"]) / len(rs), 1),
                      dist_cm=round(float(np.median([r["dist_cm"] for r in rs])), 1),
                      turn_deg=round(float(np.median([r["turn_deg"] for r in rs])), 1))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--minutes", type=float, default=None)
    ap.add_argument("--gaps", default="5,10,20,40")
    ap.add_argument("--robots", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--gz-scale", type=float, default=1.0, help="B3: the raw gyro rate times this")
    a = ap.parse_args(argv)
    data = load(a.folder, a.minutes, [x for x in a.robots.split(",") if x] or None)
    pr = params(a.folder)
    rows, sums = [], {}
    for n, d in sorted(data.items()):
        g = pr.get(n, {})
        for L in [float(x) for x in a.gaps.split(",")]:
            rs = replay(d, L, g.get("gyro_scale", 1.075), g.get("wheel_scale", 0.979), gz_scale=a.gz_scale)
            for r in rs:
                sums[n] = r.pop("_sum")
                r["robot"] = n
            rows += rs
            print("%s L=%.0f s: %d gaps" % (n, L, len(rs)), flush=True)
    tab = table(rows)
    for L, t in tab.items():
        print("gap %4.0f s: n %3d  fused %.2f (p90 %.2f)  blend %.2f (p90 %.2f)  gyro-heading %.2f (p90 %.2f)  "
              "fused inside its uncertainty %.0f %%  (median %.0f cm, %.0f deg in the gap)" % (
                  L, t["n"], t["fused"], t["fused_p90"], t["blend"], t["blend_p90"], t["gyro"], t["gyro_p90"],
                  t["covered"], t["dist_cm"], t["turn_deg"]))
    print("filters:", json.dumps(sums))
    if a.out:
        json.dump(dict(folder=os.path.basename(a.folder.rstrip("/")), minutes=a.minutes, table=tab, rows=rows,
                       filters=sums, version=FP.VERSION), open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
