#!/usr/bin/env python3
"""A fake RFID capture for testing school_rfid.py offline (build e02-nov03-rfid-pipeline, 2026-11-03).

Makes a CSV in the external recorder's column layout (one row per bundle of sightings of one tag on one antenna within
one ~0.5 s report; reader clock in microseconds; laptop arrival time lagging the reader) from the TRUE robot paths of
one of our simulator runs (run/sim/truth.csv), so the pipeline can be checked against known answers:

  - a reader clock far from the Pi's (READER_OFF_S) and a laptop clock a little off the Pi's (--mac-off-s): the
    pipeline must find the sum (reader -> Pi) to within its stated confidence;
  - two robot tags with made-up EPCs, randomly assigned to Alpha / Bravo (seeded): the pipeline must find which is which;
  - distractor tags with the same made-up prefix: still ones round the room, and one that wanders (a badge on someone
    walking about);
  - signal model (everything here is GUESSED, nothing measured): free-space path loss from antennas at guessed spots
    (6 in outside each table corner, 50 cm above the table), a forward-facing antenna pattern, a dipole tag whose gain
    turns with the robot's heading, a spatial fading field per antenna (period about half a wavelength), a fixed
    offset per antenna and channel, read noise, 0.5 dB steps; phase from the round-trip distance per channel.

None of the numbers or EPCs below come from the other team's code or data; the prefix is invented.

    python3 tools/rfid/simulate_jay_csv.py <our sim session folder> --out fake_rfid.csv [--mac-off-s 1.7] [--seed 1]
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import sys

import numpy as np

IN = 2.54
# Guessed antenna spots (Nick's message, 3 Nov), inches in the table frame (origin = corner by A3, x along the long edge)
ANTENNAS_IN = {1: (-6.0, 72.0), 2: (162.0, 72.0), 3: (-6.0, -6.0), 4: (162.0, -6.0)}
ANT_HEIGHT_CM = 50.0       # GUESSED: antenna centre above the tabletop (the tag is about 10 cm up on the robot)
TAG_HEIGHT_CM = 10.0       # GUESSED
TABLE_CM = (396.2, 167.6)
READER_OFF_S = 1503.217    # reader clock behind the laptop's (made up; the real one is estimated per session)
REPORT_S = 0.5             # one report every 0.5 s (the reader's report timeout)
STEP_S = 0.025             # one read attempt per tag every 25 ms on the antenna that is on
HOP_S = 0.2                # channel dwell
N_CH = 50
F0_KHZ, DF_KHZ = 902750, 500
PREFIX = "E2003412DC03"    # invented prefix (not any real tag's)
SENS_DBM = -68.0           # reads become likely above this
C = 299792458.0


def fake_epc(rng):
    return PREFIX + "".join(rng.choice(list("0123456789ABCDEF"), 12))


def read_truth(session):
    p = os.path.join(session, "run", "sim", "truth.csv")
    tr = {}
    with open(p) as f:
        for r in csv.DictReader(f):
            tr.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"]),
                                                  float(r["heading_deg"])))
    return {k: np.array(sorted(v)) for k, v in tr.items()}


class Field:
    """A smooth random fading field (dB) over the table: a sum of plane waves, period about half a wavelength."""

    def __init__(self, rng, sd_db=3.0, n=24, period_cm=17.0):
        ang = rng.uniform(0, 2 * math.pi, n)
        k = 2 * math.pi / (period_cm * rng.uniform(0.8, 1.6, n))
        self.kx, self.ky = k * np.cos(ang), k * np.sin(ang)
        self.ph = rng.uniform(0, 2 * math.pi, n)
        self.a = sd_db * math.sqrt(2.0 / n)

    def __call__(self, x, y):
        return self.a * np.sum(np.cos(np.outer(x, self.kx) + np.outer(y, self.ky) + self.ph), axis=1)


def simulate(truth, out_csv, mac_off_s=1.7, seed=1, n_still=8, wander=True, ants=None, t_pad_s=20.0,
             session_id="SIM-RFID", reader_off_s=READER_OFF_S):
    """Write the fake capture. Returns the answer key: {epc_to_robot, reader_to_pi_s, mac_off_s, ...}."""
    rng = np.random.default_rng(seed)
    ants = ants or {a: (x * IN, y * IN) for a, (x, y) in ANTENNAS_IN.items()}
    aid = sorted(ants)
    A = np.array([ants[a] for a in aid])
    robots = sorted(truth)
    epcs = [fake_epc(rng) for _ in robots]
    order = rng.permutation(len(robots))          # which EPC goes on which robot: random, recorded in the key
    epc_of = {robots[i]: epcs[order[i]] for i in range(len(robots))}
    rid_of = {epcs[j]: "tag%s" % "ABCDEFGH"[j] for j in range(len(epcs))}
    t0 = min(v[0, 0] for v in truth.values()) - t_pad_s
    t1 = max(v[-1, 0] for v in truth.values()) + t_pad_s
    # still distractors: round the room (shelves, walls) and two on the table's ends
    still = []
    for k in range(n_still):
        if k < 2:
            still.append((rng.uniform(5, 30) if k == 0 else TABLE_CM[0] - rng.uniform(5, 30), rng.uniform(10, 150)))
        else:
            ang = rng.uniform(0, 2 * math.pi)
            still.append((TABLE_CM[0] / 2 + 260 * math.cos(ang), TABLE_CM[1] / 2 + 160 * math.sin(ang)))
    tags = [("robot", r, epc_of[r]) for r in robots] + [("still", k, fake_epc(rng)) for k in range(n_still)]
    if wander:
        tags.append(("wander", 0, fake_epc(rng)))
    fields = {(a, j): Field(rng) for a in aid for j in range(len(tags))}
    ch_off = rng.normal(0, 1.5, (len(aid), N_CH))
    ph_off = rng.uniform(0, 2 * math.pi, (len(aid), N_CH))
    tag_ph = rng.uniform(0, 2 * math.pi, len(tags))
    tag_gain = rng.normal(0, 1.0, len(tags))
    hops = rng.integers(0, N_CH, int((t1 - t0) / HOP_S) + 2)
    ts = np.arange(t0, t1, STEP_S)
    # every tag's position and heading at every step (Pi time)
    X = np.zeros((len(tags), len(ts)))
    Y = np.zeros_like(X)
    H = np.zeros_like(X)
    for j, (kind, who, _) in enumerate(tags):
        if kind == "robot":
            v = truth[who]
            X[j], Y[j] = np.interp(ts, v[:, 0], v[:, 1]), np.interp(ts, v[:, 0], v[:, 2])
            H[j] = np.degrees(np.interp(ts, v[:, 0], np.unwrap(np.radians(v[:, 3]))))
        elif kind == "still":
            X[j], Y[j], H[j] = still[who][0], still[who][1], rng.uniform(0, 360)
        else:   # someone walking round the table's far side at about 30 cm/s, to and fro
            s = np.cumsum(rng.normal(0, 1.0, len(ts))) * 0.6
            X[j] = TABLE_CM[0] / 2 + 150 * np.sin(2 * math.pi * (ts - t0) / 90.0) + s * 0.1
            Y[j], H[j] = TABLE_CM[1] + 80 + 20 * np.sin((ts - t0) / 7.0), (ts - t0) * 10 % 360
    rows = []
    rep_t = t0 + REPORT_S
    pending = {}
    for i, t in enumerate(ts):
        if t >= rep_t:
            host = rep_t + mac_off_s + rng.uniform(0.004, 0.040)     # arrival at the laptop, laptop clock
            for (j, a), s in sorted(pending.items(), key=lambda kv: kv[1]["first"]):
                rows.append((host, j, a, s))
            pending = {}
            rep_t += REPORT_S
        a_i = int((t - t0) / STEP_S) % len(aid)
        ch = int(hops[int((t - t0) / HOP_S)])
        f = (F0_KHZ + DF_KHZ * ch) * 1e3
        lam = C / f * 100.0
        dx, dy = X[:, i] - A[a_i, 0], Y[:, i] - A[a_i, 1]
        d = np.sqrt(dx * dx + dy * dy + (ANT_HEIGHT_CM - TAG_HEIGHT_CM) ** 2)
        # antenna pattern: facing the table's middle, cos^2 roll-off down to -12 dB
        ax, ay = TABLE_CM[0] / 2 - A[a_i, 0], TABLE_CM[1] / 2 - A[a_i, 1]
        cosb = (dx * ax + dy * ay) / (np.hypot(dx, dy) * math.hypot(ax, ay) + 1e-9)
        g_ant = 10 * np.log10(np.clip(0.5 * (1 + cosb), 0.06, 1.0))
        # tag dipole across the robot (left-right): gain by the angle to the antenna in the floor plane
        ang_ant = np.degrees(np.arctan2(-dy, -dx))
        g_tag = 10 * np.log10(np.clip(np.sin(np.radians(ang_ant - H[:, i])) ** 2, 0.08, 1.0))
        fade = np.array([fields[(aid[a_i], j)](X[j:j + 1, i], Y[j:j + 1, i])[0] for j in range(len(tags))])
        rssi = (-47.0 - 20 * np.log10(d / 100.0) + g_ant + g_tag + fade + ch_off[a_i, ch] + tag_gain
                + rng.normal(0, 0.7, len(tags)))
        p = 0.75 / (1 + np.exp(-(rssi - SENS_DBM) / 1.5))
        hit = rng.random(len(tags)) < p
        for j in np.nonzero(hit)[0]:
            phase = (4 * math.pi * d[j] / lam + ph_off[a_i, ch] + tag_ph[j] + rng.normal(0, 0.1)) % (2 * math.pi)
            key = (j, aid[a_i])
            r_us = int(round((t + mac_off_s - reader_off_s) * 1e6))          # reader clock
            s = pending.get(key)
            q = round(rssi[j] * 2) / 2.0
            if s is None:
                pending[key] = dict(first=r_us, last=r_us, n=1, ch=ch, rssi=q, phase=phase)
            else:
                s.update(last=r_us, n=s["n"] + 1, rssi=max(s["rssi"], q), phase=phase)
    with open(out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["host_ts_unix", "host_ts_iso", "session_id", "condition", "point_x", "point_y", "orientation_deg",
                    "epc", "robot_id", "antenna_id", "channel_index", "frequency_khz", "rssi_impinj_raw",
                    "rssi_impinj_dbm", "rssi_llrp_dbm", "phase_raw", "phase_deg", "doppler_raw", "reader_ts_us",
                    "tag_seen_count", "reader_first_ts_us"])
        for host, j, a, s in rows:
            epc = tags[j][2]
            raw = int(round(s["rssi"] * 100))
            praw = int(s["phase"] / (2 * math.pi) * 4096) % 4096
            w.writerow(["%.6f" % host, dt.datetime.fromtimestamp(host, tz=dt.timezone.utc).isoformat(), session_id,
                        "mounted_moving", "", "", "", epc, rid_of.get(epc, ""), a, s["ch"] + 1,
                        F0_KHZ + DF_KHZ * s["ch"], raw, "%.2f" % (raw / 100.0), int(round(s["rssi"])), praw,
                        "%.4f" % (praw * 360.0 / 4096), int(rng.integers(-200, 200)), s["last"], s["n"], s["first"]])
    return {"epc_to_robot": {epc_of[r]: r for r in robots}, "reader_to_pi_s": reader_off_s - mac_off_s,
            "mac_off_s": mac_off_s, "reader_off_s": reader_off_s, "rows": len(rows),
            "distractors": [t[2] for t in tags if t[0] != "robot"], "antennas_cm": {str(a): ants[a] for a in aid},
            "seed": seed}


def main(argv=None):
    import rfid_gate
    argv = rfid_gate.enable_from_argv(sys.argv[1:] if argv is None else argv)
    rfid_gate.require("simulate_jay_csv.py")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="our simulator session folder (with run/sim/truth.csv)")
    ap.add_argument("--out", required=True, help="the fake capture CSV to write")
    ap.add_argument("--mac-off-s", type=float, default=1.7, help="laptop clock minus the Pi's (s), the residual to find")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--still", type=int, default=8, help="still distractor tags")
    ap.add_argument("--no-wander", action="store_true", help="no wandering distractor")
    a = ap.parse_args(argv)
    key = simulate(read_truth(a.session), a.out, a.mac_off_s, a.seed, a.still, not a.no_wander)
    kp = os.path.splitext(a.out)[0] + "_key.json"
    with open(kp, "w") as f:
        json.dump(key, f, indent=1)
    print("%s: %d rows; answer key %s" % (a.out, key["rows"], kp))
    return 0


if __name__ == "__main__":
    sys.exit(main())
