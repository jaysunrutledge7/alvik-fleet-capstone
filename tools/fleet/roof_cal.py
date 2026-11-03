#!/usr/bin/env python3
"""Each robot's roof tag measured by the cameras (2026-11-03, Nick 13:56: "checking for height and angle should be
part of the auto calibration process").

The roof tags at school are paper on the robots' RFID mounts, which lean about 30 deg; a paper wedge is meant to make
each one level and centred over the wheel-axle midpoint, by hand. So each robot's tag gets measured, every Measure and
every refit after a run:

  height   the tag middle's height above the table (cm, +-). The site fit (site_fit.solve, roof_fit) fits it per robot
           with the cameras; two cameras seeing the same tag pin it (their rays cross at the tag), one camera alone
           only through the tag's printed size, which the blur also changes (its +- says so).
  tilt     the angle between the tag's face and the table, and which way the face points in the robot's frame
           (towards the nose, its left, ...): a tilted square reads narrower one way.
  offset   the tag middle's distance from the robot's turning point (ahead, left; cm). Only a turn shows it: a tag
           off-centre draws a circle when the robot spins on the spot. The robots stand still at Measure, so it is
           measured after a run that starts with the sync spin (the refit), from tag_poses.csv.

What is done with them (tag_bridge.py, switches in the site's tags):
  tags.roof_measured  (default on)  the measured height (and offset, when measured well) replace the typed or
                      placeholder tags.height_cm for that robot; off: the typed/placeholder height for every robot.
  tags.tilt_correct   (default on)  the measured tilt goes into the pose (tag_geom: each corner met at its own height);
                      off: the tag is taken as flat.
Over TILT_OK_DEG the setup's words and Mission Control say it plainly, with what it costs.

Saved as roof_tags.json beside the camera models (site_fit.save).
"""
import csv
import json
import math
import os

import numpy as np

TILT_OK_DEG = 5.0        # a tilt at or under this (plus twice its +-) is "level" (OK). Chosen: the simulator measured flat tags at 0.5-3 deg
                         # (the check's own noise), and 5 deg costs under 0.3 cm (report table). DOE note: a design over
                         # wedge angles (0, 5, 10 deg) on the real table, the tilt read against a phone's level, would set it
HEIGHT_SD_USE_CM = 1.5   # the measured height replaces the typed/placeholder one only when its +- is this or better.
                         # Chosen from the simulator: two cameras said +- 0.1-1.0 cm (true errors 0.0-0.3 cm, so the +-
                         # is cautious); one camera said +- 3.5-5.7 cm (true errors up to 9 cm). 1.5 sits between. DOE
                         # note: Measure 5 times with the robots moved, the heights' spread against their +-
OFFSET_SD_USE_CM = 0.5   # the measured offset is used only when its +- is this or better and the spin covered
                         # SPIN_MIN_DEG. Chosen; DOE note: the sync spin repeated 5 times, the offsets' spread
TILT_VIEWS_DEG = 90.0    # a tilt is trusted only when the tag was seen from two cameras, or from one camera at headings
                         # covering this much of the circle (a turn). One camera looking nearly straight down at a still
                         # tag cannot tell a small tilt (it only changes the tag's look to second order): the simulator's
                         # flat tags read 18-30 deg that way. Chosen; DOE note: the simulator's one-camera cases at
                         # 0, 5, 15, 30 deg with the robot still and after the spin (report)
SPIN_MIN_DEG = 270.0     # a spin must turn at least this much (seen by one camera) for the offset. Chosen: a circle
                         # needs most of its round to fix its middle. No DOE: geometry
SPIN_MAX_DRIFT_CM = 6.0  # ... while the tag stays within this of its middle (a spin, not a drive). Chosen: the offset is
                         # at most ~2 cm, the wheels slip ~1 cm in a spin. No DOE
SPIN_WINDOW_S = 25.0     # a spin is looked for in windows this long. Chosen: the sync spin (two half turns) takes ~10 s
OFFSET_SAY_CM = 0.5      # an offset under this is said as "centred". Chosen: the placement's 2 mm spec plus the +-

# What a tilt left uncorrected costs, measured in the simulator (report e02-nov03-roof-tag-height-tilt, table 2: the
# setup and tracking as before this build, height right): (tilt deg, position cm added to the flat tag's worst,
# heading deg, 95th percentile). 45 deg is extrapolated, not simulated. Interpolated linearly.
TILT_ERR = [(0.0, 0.0, 1.5), (5.0, 0.0, 2.0), (15.0, 0.4, 5.0), (30.0, 0.8, 12.0), (45.0, 1.5, 20.0)]

DIRS = ["the nose", "the nose-left corner", "its left", "the back-left corner", "its back", "the back-right corner",
        "its right", "the nose-right corner"]


def lean_words(lean_deg):
    """Which way the face points, in the robot's frame: 'its left' etc. (lean 0 = the nose, anticlockwise)."""
    return DIRS[int(round((lean_deg % 360.0) / 45.0)) % 8]


def tilt_err_cm(tilt_deg):
    xs, ys, _ = zip(*TILT_ERR)
    return float(np.interp(abs(tilt_deg), xs, ys))


def tilt_err_deg(tilt_deg):
    xs, _, ys = zip(*TILT_ERR)
    return float(np.interp(abs(tilt_deg), xs, ys))


def tilt_trusted(r):
    """A roof's tilt is judged only when two cameras saw the tag, or one saw it turned round (TILT_VIEWS_DEG)."""
    return r.get("two_camera_moments", 0) > 0 or r.get("heading_spread_deg", 0.0) >= TILT_VIEWS_DEG


def names_of(obs):
    return {str(k): v for k, v in (obs.get("tag_names") or {}).items()}


def summarize(res, obs, typed_cm=None, placeholder=False):
    """The per-robot verdicts and words from a site fit made with roof_fit (res["roofs"]). typed_cm: the site's
    tags.height_cm (None: not typed, placeholder). Returns {robots: {name: {...}}, words: [...]}."""
    roofs = res.get("roofs") or {}
    nm = names_of(obs)
    out = {"robots": {}, "words": [], "typed_height_cm": typed_cm, "placeholder": bool(placeholder),
           "assumed_height_cm": float(obs.get("tag_height_cm", 6.0)), "tilt_ok_deg": TILT_OK_DEG}
    for i, r in sorted(roofs.items(), key=lambda kv: int(kv[0])):
        name = nm.get(str(i), "tag %s" % i)
        h, hsd = r["height_cm"], r["height_sd_cm"]
        use_h = hsd <= HEIGHT_SD_USE_CM
        t, tsd, lean = r["tilt_deg"], r["tilt_sd_deg"], r["lean_deg"]
        trusted = tilt_trusted(r)
        # level unless clearly over: TILT_OK_DEG plus twice its +- (a 5.2 +- 0.9 deg read of a true 5 deg is not a warning)
        level = (t <= TILT_OK_DEG + 2.0 * tsd) if trusted else None
        d = dict(r, id=int(i), name=name, height_used=use_h, level=level, tilt_trusted=trusted,
                 lean_words=lean_words(lean), tilt_err_cm=round(tilt_err_cm(t), 1), tilt_err_deg=round(tilt_err_deg(t)),
                 offset_cm=None, offset_sd_cm=None, offset_used=False)
        own = (obs.get("roof_typed_cm") or {}).get(str(i))
        ref = ("typed %.1f cm" % own if own is not None else "typed %.1f cm" % typed_cm if typed_cm is not None
               else "placeholder %.1f cm" % out["assumed_height_cm"])
        d["height_in_use_cm"] = round(h if use_h else (own if own is not None else typed_cm if typed_cm is not None
                                                       else out["assumed_height_cm"]), 2)
        if use_h:
            hw = "height %.1f cm (+- %.1f; %s): used for %s" % (h, hsd, ref, name)
        else:
            hw = ("height not measurable here (the cameras' +- is %.1f cm): %s kept; %s" % (
                hsd, ref, "only one camera sees it, and one camera cannot tell the tag's height from its size: "
                "measure from the table to the tag's middle with a ruler and type it (tags.heights_cm)"
                if r.get("two_camera_moments", 0) == 0 else "more reads needed: drive it about and Refit"))
        if level is None:
            tw = ("tilt not checked yet: one camera looking nearly straight down at a still tag cannot tell a tilt; it is "
                  "checked after a run's sync spin (Refit). Meanwhile look at the tag from the side: it should sit flat")
        elif level:
            tw = "tilt %.1f deg (+- %.1f): level, OK" % (t, tsd)
        else:
            tw = ("tilt %.0f deg (+- %.1f), leaning towards %s: NOT LEVEL. Level it with the wedge and Measure again: "
                  "uncorrected, a tilt like this costs about %.1f cm of position and %.0f deg of heading, and the size "
                  "check refuses some reads at the table's ends (the tilt is corrected in the pose while "
                  "tags.tilt_correct is on)" % (t, tsd, lean_words(lean), tilt_err_cm(t), tilt_err_deg(t)))
        d["words"] = "%s's roof tag: %s; %s; offset from the turning point: not measured at Measure (the robots stand " \
                     "still; it is measured from the sync spin after a run)." % (name, hw, tw)
        out["robots"][name] = d
        out["words"].append(d["words"])
    return out


def add_offsets(roof_tags, tag_poses_csv, prev=None):
    """The offsets from a run's spins (tag_poses.csv: every read with its tag-middle x, y and heading). prev: the
    roof_tags.json in use (its offsets kept for a robot this run did not spin). Changes roof_tags in place."""
    found = offsets_from_poses(tag_poses_csv) if tag_poses_csv and os.path.exists(tag_poses_csv) else {}
    for name, d in (roof_tags.get("robots") or {}).items():
        f = found.get(name)
        if f and f["sd_cm"] <= OFFSET_SD_USE_CM:
            d.update(offset_cm=f["offset_cm"], offset_sd_cm=f["sd_cm"], offset_used=True, offset_spins=f["spins"],
                     offset_from="this run's spins")
        elif prev and ((prev.get("robots") or {}).get(name) or {}).get("offset_used"):
            p = prev["robots"][name]
            d.update(offset_cm=p["offset_cm"], offset_sd_cm=p["offset_sd_cm"], offset_used=True,
                     offset_from=p.get("offset_from", "an earlier refit"))
        if d.get("offset_used"):
            a, l_ = d["offset_cm"]
            ow = ("offset %.1f cm (%.1f ahead, %.1f to its left; +- %.1f, from %s): used" % (
                math.hypot(a, l_), a, l_, d["offset_sd_cm"], d.get("offset_from"))
                  if math.hypot(a, l_) >= OFFSET_SAY_CM else "offset under %.1f cm: centred, OK" % OFFSET_SAY_CM)
        elif f:
            ow = "offset %.1f cm (+- %.1f: not sure enough, not used)" % (math.hypot(*f["offset_cm"]), f["sd_cm"])
        else:
            ow = "offset not measured (no spin on the spot seen in this run: start runs with the sync spin)"
        d["words"] = d["words"].split("; offset from")[0] + "; " + ow + "."
    roof_tags["words"] = [d["words"] for d in roof_tags["robots"].values()]
    return roof_tags


def offsets_from_poses(path):
    """{robot: {offset_cm [ahead, left], sd_cm, spins}} from tag_poses.csv: windows where one camera saw the robot turn
    SPIN_MIN_DEG while its tag stayed within SPIN_MAX_DRIFT_CM; then tag = c + R(heading) . offset fitted (linear) over
    every such window (each its own c)."""
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                # the tag's middle (mid_x, mid_y, 2026-11-03 bridges) when there: x, y have an offset in use taken off
                mx, my = (r.get("mid_x") or r["x"]), (r.get("mid_y") or r["y"])
                rows.setdefault((r["robot"], r["cam"]), []).append(
                    (float(r["t"]), float(mx), float(my), math.radians(float(r["heading"]))))
            except (KeyError, ValueError):
                continue
    by_robot = {}
    for (rob, cam), lst in rows.items():
        lst.sort()
        i = 0
        while i < len(lst):
            j = i
            while j + 1 < len(lst) and lst[j + 1][0] - lst[i][0] <= SPIN_WINDOW_S:
                j += 1
            seg = lst[i:j + 1]
            if len(seg) >= 12:
                th = np.unwrap([s[3] for s in seg])
                xy = np.array([[s[1], s[2]] for s in seg])
                turned = float(np.degrees(th.max() - th.min()))
                drift = float(np.max(np.linalg.norm(xy - np.median(xy, 0), axis=1)))
                if turned >= SPIN_MIN_DEG and drift <= SPIN_MAX_DRIFT_CM:
                    by_robot.setdefault(rob, []).append((xy, th))
                    i = j + 1
                    continue
            i += max(1, (j - i) // 4)
    out = {}
    for rob, segs in by_robot.items():
        n = len(segs)
        A, b = [], []
        for k, (xy, th) in enumerate(segs):
            for (x, y), t in zip(xy, th):
                c, s = math.cos(t), math.sin(t)
                row1 = [0.0] * (2 * n) + [c, -s]
                row2 = [0.0] * (2 * n) + [s, c]
                row1[2 * k], row2[2 * k + 1] = 1.0, 1.0
                A += [row1, row2]
                b += [x, y]
        A, b = np.array(A), np.array(b)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        res = b - A @ sol
        dof = max(1, len(b) - len(sol))
        s2 = float(res @ res) / dof
        cov = s2 * np.linalg.pinv(A.T @ A)
        sd = float(math.sqrt(max(cov[-2, -2], cov[-1, -1])))
        out[rob] = {"offset_cm": [round(float(sol[-2]), 2), round(float(sol[-1]), 2)], "sd_cm": round(sd, 2),
                    "spins": n, "rms_cm": round(math.sqrt(s2), 2)}
    return out


def load(cal_dir):
    p = os.path.join(cal_dir, "roof_tags.json")
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def roofs_for_bridge(cal_dir, tags):
    """{tag id: roof dict for tag_geom.pose_from_detection} from roof_tags.json, by the site's switches; {} when off or
    nothing is measured (every robot: the flat model at tags.height_cm, as before). Also the words said at start."""
    rt = load(cal_dir) if cal_dir else None
    on_h = _on(tags.get("roof_measured", True))
    on_t = _on(tags.get("tilt_correct", True))
    if not rt or not (on_h or on_t):
        return {}, ["roof tags: %s" % ("switched off (tags.roof_measured and tags.tilt_correct): the typed/placeholder "
                                       "height for every robot, flat" if rt else "not measured yet (Measure)")]
    out, words = {}, []
    for name, d in (rt.get("robots") or {}).items():
        tid = int(d["id"])
        H0 = float((tags.get("heights_cm") or {}).get(name) or tags.get("height_cm") or 6.0)
        h = float(d["height_cm"]) if (on_h and d.get("height_used")) else H0
        roof = {"height_cm": h}
        if on_t and d.get("level") is False:
            roof.update(tilt_deg=float(d["tilt_deg"]), lean_deg=float(d["lean_deg"]))
        if on_h and d.get("offset_used"):
            roof["offset_cm"] = list(d["offset_cm"])
        out[tid] = roof
        words.append("%s: tag %.1f cm up%s%s" % (
            name, h, ", tilt %.0f deg corrected" % roof["tilt_deg"] if "tilt_deg" in roof else "",
            ", offset %.1f/%.1f cm taken off" % tuple(roof["offset_cm"]) if "offset_cm" in roof else ""))
    return out, words


def _on(v):
    return str(v).lower() not in ("0", "false", "off", "no", "none")
