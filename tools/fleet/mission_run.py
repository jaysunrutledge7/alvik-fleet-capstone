#!/usr/bin/env python3
"""Run a mission from one command, with or without Mission Control (one system, 2026-10-28; Nick 18:36: "also runnable
from one command without the page", the safety net for Monday). Mission Control's Run screen starts this too, so the
page and the command are the same run.

    python3 tools/fleet/mission_run.py --table 61x122                        # Monday's default on the robots (Pi)
    python3 tools/fleet/mission_run.py --map logs/mc/map.json --program logs/mc/program.json
    python3 tools/fleet/mission_run.py --table 70x130 --dry                  # the simulator, both simulated cameras
    python3 tools/fleet/mission_run.py --table 61x122 --dry --washout --drop-pct 5 --dead Right@60

What it does, in order (each step logged in the session folder, logs/<date-time>-mission[-dry]/):
  1. the workspace: --map (Mission Control's, with the warm-up's sides and zones) or a blank table of --table size
     (every side "not felt yet": kept away from like an open edge);
  2. the mission: --program (blocks) or Monday's default for --robots (missions.monday_program), built for this table
     and planned (refused if the plan's closest approach is under 8 cm);
  3. the cameras: the camera bridge (cam_bridge.py; --farm: each camera tracked in its own process) on the recorder's
     pictures (real) or the simulator's two cameras (dry);
  4. the run: run_plan.py with every engine check (battery, restarts, settings read back, silent robots, the 8 cm net),
     each robot straightened at the start: its measured pose (camera position, the facing check's heading) against
     its start pose; off: a turn on the spot or a drive to its spot, then the facing check again (straighten.py);
  5. the record (item 6): run_plan's logs (both cameras' positions, every move and its timing, battery, messages),
     the Pi's temperature and throttling (cpu.csv, perf.csv: the heat tracker), the pictures at the tracking rate and
     a clip 5 s either side of every stop (frames: on the USB drive, <drive>/fleet-frames/<session>/frames, since
     2026-10-28; no drive: <session>/frames), the line check, mission_summary.json; --upload: a release with a tag of
     its own (upload_session.py --unique-tag), the frames as their own asset (rec-frames-small.zip).
Tracking during the run (2026-11-03, Nick: "I want to be able to run with or without tags"): --tracking tags (roof
AprilTags, tag bridge), lights (the robots' lights and outlines, camera bridge; home's way) or none (no camera at all:
each robot starts on its home by hand and drives on its wheels and gyro; no bridge, no facing check, no feel-out; the
+- at each point grows 0.07 cm per cm driven). auto = the site's run_tracking, else its tracking. A site with typed
production points (sticker_points.py) gives the map, homes, keep-outs and default orders in every mode.
Ctrl+C stops every robot (run_plan's own stop). Free space (diskguard.py, 2026-10-28): not started when the card is
under its floor; frame saving stops under the frames' floor; under the card's floor during the run, run_plan is sent
Ctrl+C (every robot stops) and the run ends as it would on Ctrl+C, with the upload.
"""
import argparse
import datetime as dt
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "live"),
          os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import fleetmap as FM        # noqa: E402
import missions as MS        # noqa: E402
import planner as P          # noqa: E402
import workspace as WS       # noqa: E402
import engine_checks as EC   # noqa: E402
import diskguard as DG       # noqa: E402

PY = sys.executable
EDGE_GUARD = ["set safe_edge 3", "set edge_dir 0", "set edge_thr 150", "set edge_n 3", "set edge_bot_cm 11",
              "set bumper_cm 0"]    # the robots' edge guard, as run-fleet-pi.sh / ladder.SAFE (lost at every power-off)
PLAN_LIMIT_S = 60.0     # planning time allowed for the mission (Monday plans in 4-5 s here). No DOE: a limit
DROP_CM = 74.0          # the simulator's conference table: the floor this far below the top past an open edge (a
                        # typical conference table's height; the site's table.drop_cm when given). No DOE: a model
BRIDGE_START_S = 5.0    # the camera bridge learns the empty floor round the robots before run_plan starts (as
                        # run-fleet-pi.sh). No DOE: the bridge needs 5 pictures


def table_map(size, robots):
    """A blank table: 'WxL' or 'LxW' cm (the longer side along x)."""
    a, b = (float(v) for v in size.lower().split("x"))
    L, W = max(a, b), min(a, b)
    return WS.blank_map((0, 0, L, W), [(n, [20 + 30 * k, 15, 90]) for k, n in enumerate(robots)])


def say(s):
    print(time.strftime("%H:%M:%S ") + s, flush=True)


def uses_refs(args):
    """Reference tags on the simulated table (2026-11-03): on unless --no-refs or the site switches them off."""
    if getattr(args, "no_refs", False):
        return False
    rt = (getattr(args, "site_", None) or {}).get("ref_tags")
    return True if rt is None else bool(rt.get("use", True))


def corner_clicks(args):
    """Corner clicking (a switch per site, 2026-11-03): --corner-clicks on/off, else the site's corner_clicks, else on."""
    v = getattr(args, "corner_clicks", "auto") or "auto"
    if v != "auto":
        return v == "on"
    return bool((getattr(args, "site_", None) or {}).get("corner_clicks", True))


def build(args, out):
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    cal = getattr(args, "feel_cal_", None)
    if args.map:
        m = FM.load(args.map)
        m["tasks"] = {}
        if cal and getattr(args, "tracking_", "lights") == "tags":
            # the table's calibration from feeling it out (2026-11-02): the outline the robots felt
            import feel_table as FTB
            m, _w = FTB.apply_to_map(m, cal)
            for w_ in _w:
                say(w_)
    elif typed_site(args):
        # a site with typed production points (2026-11-03, Jay's sticker sheet): its points, homes, raised box and
        # default orders, in every tracking mode
        import production as PR
        m, _w, sprog, _o = PR.site_production(args.site_, robots, outline=(cal or {}).get("outline_used"),
                                              experiment=getattr(args, "experiment_", None) or None,
                                              swap=getattr(args, "swap_", False),
                                              wiggle={"on": True, "off": False}.get(getattr(args, "sync_wiggle", "auto")))
        args.site_program_ = sprog
        for w_ in _w:
            say(w_)
    elif getattr(args, "tracking_", "lights") == "tags":
        # the AprilTag site (2026-11-02): the table with its production points and the homes near the middle
        import production as PR
        a_, b_ = (float(v) for v in args.table.lower().split("x"))
        pts = {}
        import tag_cam as TCAM
        dcfg = TCAM.default_config(table=(max(a_, b_), min(a_, b_)), seed=args.seed + 4, refs=uses_refs(args))
        if not args.program:
            # no program given (a dry run from the command line): the simulator's stickers as the points
            pts = dcfg["stickers"]
        kos = None
        if dcfg.get("ref_tags"):
            # the reference tiles (2026-11-03): no robot drives over them (the simulator's tiles where they lie)
            import ref_tags as RT
            kos = RT.keepouts(dcfg["ref_tags"]["poses"], dcfg["ref_tags"].get("config"))
        m, _w = PR.site_map((max(a_, b_), min(a_, b_)), robots, points=pts,
                            outline=(cal or {}).get("outline_used"), keepouts=kos)
        for w_ in _w:
            say(w_)
    else:
        m = table_map(args.table, ["Left", "Center", "Right"])
    if args.program:
        with open(args.program) as f:
            prog = json.load(f)
    elif getattr(args, "site_program_", None):
        prog = args.site_program_                      # the typed site's default production mission
    elif getattr(args, "tracking_", "lights") == "tags":
        # a production run over the map's points (half each, in order), then home (2026-11-02)
        import production as PR
        labs = sorted((m.get("spots") or {}), key=lambda k: (len(k), k))
        orders = {n: labs[k::len(robots)][:args.points_each] for k, n in enumerate(robots)}
        prog = PR.program(orders, dwell_s=args.dwell_s)
    else:
        prog = MS.monday_program(tuple(robots))
    sets = dict(kv.split("=", 1) for kv in args.setting)
    # the 8 cm lock (2026-10-29): no 2-robot mission under the net from the command line either (the simulator's
    # safety-net DOE: --dry --below-net-doe)
    raised = MS.gap_lock_settings(dict((k, float(v)) for k, v in sets.items() if k in MS.GAP_KEYS_MISSION),
                                  where="--setting")
    for w in raised:
        say("8 cm LOCK: " + w)
    for k in MS.GAP_KEYS_MISSION:
        if k in sets and not MS._BELOW_NET_DOE[0]:
            sets[k] = str(max(float(sets[k]), MS.gap_floor()))
    if "chicken_min_cm" in sets:
        # the program's own chicken min_cm (Monday's says 8) gives way to an explicit --setting (safety-net DOE)
        for bl in prog.get("blocks", []):
            if bl.get("block") == "chicken":
                bl["min_cm"] = float(sets["chicken_min_cm"])
    if not MS._BELOW_NET_DOE[0]:
        for bl in prog.get("blocks", []):
            # a program's own Chicken min_cm (Mission Control's program.json, a saved edit) is held to the net too
            if bl.get("block") == "chicken" and "min_cm" in bl and float(bl["min_cm"]) < MS.gap_floor():
                say("8 cm LOCK: the program's Chicken min_cm %.1f cm raised to %.1f cm" % (float(bl["min_cm"]),
                                                                                        MS.gap_floor()))
                bl["min_cm"] = MS.gap_floor()
    mm = MS.build(m, prog, s=sets or None)
    over = {"separation_cm": max(float(((mm.get("mission") or {}).get("planner") or {}).get("separation_cm", 8.0)),
                                 args.min_gap_cm)}
    if typed_site(args):
        # same speed in every experiment (Jay-Sun, 3 Nov): the site's production speeds pinned in the planner (re-plans
        # keep the plan's settings), recorded in the summary
        import experiments as EX
        sp = dict(((prog.get("production") or {}).get("experiment") or {}).get("speed") or EX.conf(args.site_)["speed"])
        over.update(sp)
        args.speed_ = sp
        say("speeds pinned for this site: drive %.0f cm/s, turn %.0f deg/s, final approach %.0f cm/s" % (
            sp["cruise_cms"], sp["turn_dps"], sp["confirm_cms"]))
    t0 = time.time()
    if getattr(args, "on_problem_", "stop") == "keep_going":
        over["skip_unreachable"] = True     # keep going (2026-11-03): a point no route reaches is skipped and listed,
        #                                     not a plan refused
    plan = MS.plan_synced(mm, over=over, t_limit_s=PLAN_LIMIT_S, log=say)
    gap = EC.plan_gap(plan, mm, gate_cm=args.min_gap_cm)
    bad = EC.gate_words(gap, args.min_gap_cm)
    mp, pp = os.path.join(out, "mission.json"), os.path.join(out, "plan.json")
    with open(mp, "w") as f:
        json.dump(mm, f, indent=1)
    P.save_plan(plan, pp)
    with open(os.path.join(out, "program.json"), "w") as f:
        json.dump(prog, f, indent=1)
    f_ = mm["floor"]
    say("mission %s on a %.0f x %.0f cm table, robots %s: planned in %.1f s, done in %.0f s, closest %.1f cm"
        % (prog.get("name"), f_["x_max"] - f_["x_min"], f_["y_max"] - f_["y_min"], ", ".join(plan["robots"]),
           time.time() - t0, plan["summary"]["makespan_s"], gap["closest_cm"]))
    for n in mm["mission"].get("notes", []):
        say("  " + n)
    for w in plan.get("warnings") or []:
        if " skips task " in str(w) and "(at point" not in str(w):
            say("KEEP GOING: %s (skipped; listed in the summary)" % w)
    if bad:
        raise SystemExit("Plan refused: %s" % bad)
    return mm, plan, mp, pp


def start_bridge(args, out, mp, cal, scal, stream, side_stream):
    if getattr(args, "tracking_", "lights") == "tags":
        # the AprilTag robots (2026-11-02): tag_bridge.py, both cameras, the tags' own ids and heights
        cmd = ["nice", "-n", "5", PY, os.path.join(HERE, "tag_bridge.py"), "--tags", args.tags_json,
               "--cam", "overhead=%s,%s" % (stream, cal), "--out", os.path.join(out, "bridge"), "--wait-calib-s", "90",
               "--robots", args.robots]
        if scal:
            cmd += ["--cam", "side=%s,%s" % (side_stream, scal)]
        f = open(os.path.join(out, "bridge.txt"), "w")
        return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO), f
    cmd = ["nice", "-n", "5", PY, os.path.join(HERE, "cam_bridge.py"), "--map", mp, "--calib", cal, "--stream", stream,
           "--out", os.path.join(out, "bridge"), "--wait-calib-s", "40"]
    if scal:
        cmd += ["--side-calib", scal, "--side-stream", side_stream]
    if args.farm:
        cmd += ["--farm"]
        if args.rings:
            cmd += ["--overhead-ring", "mirlt-overhead", "--side-ring", "mirlt-side"]
        if args.half:
            cmd += ["--half"]
    f = open(os.path.join(out, "bridge.txt"), "w")
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO), f


def space_watch(guard, child, said, stop_evt):
    """While run_plan drives: the card under its floor -> Ctrl+C to run_plan (every robot stops), once."""
    while not stop_evt.wait(DG.CHECK_S * 2):
        st = guard.state()
        if st["stop"] and child.poll() is None:
            said.append(st["why"])
            say("THE PI'S CARD IS NEARLY FULL: %s. Stopping every robot (as Ctrl+C); the logs are closed and "
                "uploaded next. Before the next run: ./run-fleet-pi.sh space" % st["why"])
            child.send_signal(signal.SIGINT)
            return


def bridge_watch(events_path, stop_evt, said):
    """While the run goes (2026-11-03): the tag bridge's camera_moved events (a bumped camera, its reference tags off
    where the setup saw them) said on this screen (Mission Control shows these lines) and kept for the summary. The
    run is not stopped by this (Nick decides: the robots' own sensors and the other camera still work)."""
    pos = 0
    while not stop_evt.wait(1.0):
        try:
            with open(events_path) as f:
                f.seek(pos)
                for ln in f:
                    if not ln.endswith("\n"):
                        break
                    pos += len(ln.encode())
                    try:
                        ev = json.loads(ln)
                    except ValueError:
                        continue
                    if ev.get("kind") == "camera_moved":
                        said.append(ev)
                        say("WARNING: camera %s has MOVED (bumped?): its reference tags are %.1f px, about %.1f cm on "
                            "the table, from where the setup saw them. Its positions are off by about that much. Stop "
                            "the run and Measure again (Cameras & table): with the tags it takes a minute." % (
                                ev.get("cam"), ev.get("median_px", 0), ev.get("about_cm", 0)))
                    elif ev.get("kind") == "camera_back":
                        say("camera %s: its reference tags are back where the setup saw them" % ev.get("cam"))
        except OSError:
            continue


def gap_args(args):
    """The 8 cm lock on the command line: --min-gap-cm under the net is raised for a 2-robot mission (the simulator's
    safety-net DOE: --dry --below-net-doe)."""
    if args.below_net_doe:
        if not args.dry:
            raise SystemExit("--below-net-doe is for the simulator only (--dry): nothing was moved")
        MS.below_net_doe(True)
        say("SIMULATOR DOE: robot gaps under the 8 cm safety net allowed for this dry run only")
        return
    n = len([r for r in args.robots.split(",") if r.strip()])
    if n >= 2 and args.min_gap_cm < MS.gap_floor():
        say("8 cm LOCK: --min-gap-cm %.1f raised to %.1f cm (no 2-robot mission runs under the safety net)"
            % (args.min_gap_cm, MS.gap_floor()))
        args.min_gap_cm = MS.gap_floor()


def typed_site(args):
    """The site's production points are typed (sticker_points.py; the site's points.use "typed")."""
    import sticker_points as SP
    return bool(getattr(args, "site_", None)) and SP.typed_on(args.site_)


def resolve_site(args):
    """--site / the site in use: its tracking (lights or tags), its tags and calibration folder (2026-11-02). Home (the
    default) changes nothing: lights, as before."""
    import sites as SI
    site = SI.load(args.site) if (args.site or args.tracking != "lights") else None
    if site is None:
        try:
            site = SI.load()
        except SystemExit as e:
            if SI.is_site_file_error(e):
                raise            # a broken or missing chosen site stops loudly (2026-11-03), never home in silence
            site = None
    tr = args.tracking if args.tracking != "auto" else ((site or {}).get("run_tracking") or (site or {}).get(
        "tracking") or "lights")
    args.tracking_ = tr
    args.site_ = site
    # the keep-going policy (2026-11-03): this run's --on-problem / --lost-robot-points / --feel-trust, else the site's,
    # else home's (stop, as before)
    import sites as SI_
    for k, a in (("on_problem", "on_problem"), ("lost_robot_points", "lost_robot_points"),
                 ("feel_out_trust", "feel_trust")):
        v = getattr(args, a, "auto") or "auto"
        setattr(args, k + "_", v if v != "auto" else ((site or {}).get(k) or SI_.POLICY_DEFAULTS[k]))
    if site and not os.environ.get("MIRLT_CONF_DIR"):
        # the site's own folder for the day's calibration records and stored corrections (run_plan inherits it)
        os.environ["MIRLT_CONF_DIR"] = site["conf_dir"]
    if tr in ("tags", "none", "lights") and args.robots == "Left,Right" and site and site.get("tracking") == "tags":
        args.robots = ",".join(r["name"] for r in site["robots"])        # the site's robots, not home's
    if tr == "tags":
        tags = dict((site or {}).get("tags") or {})
        if not tags.get("ids"):
            tags.update({"family": "tag25h9", "tag_cm": 8.0, "roof_cm": 9.5,
                         "ids": {n: k for k, n in enumerate(r.strip() for r in args.robots.split(",") if r.strip())}})
        if tags.get("height_cm") is None:
            tags["height_cm"] = 6.0
            say("NOTE: the roof's height is not measured yet: 6.0 cm used (PLACEHOLDER; tags.height_cm in the site)")
        args.tags_ = tags
    return site


def sync_mark(out, what, args, extra=None):
    """A sync mark in the session folder (the Pi's time and clock status; Mission Control's buttons add more)."""
    try:
        import sync_tools as ST
        m = ST.mark(out, what, session=getattr(args, "session_id", None), extra=extra)
        say("SYNC MARK %s: %s (Pi clock: %s)" % (what, m["t_iso"], m["clock"].get("words")))
    except Exception as e:     # noqa: BLE001  (a mark never stops a run)
        say("(sync mark %s not written: %s)" % (what, e))


def feel_conf_dir(args, out):
    """Where the table calibrations live: --feel-conf-dir, else the site's folder (robots), else the session's own
    (a dry run: nothing kept unless a folder is given)."""
    if args.feel_conf_dir:
        return os.path.abspath(os.path.expanduser(args.feel_conf_dir))
    if not args.dry and args.site_:
        return args.site_["conf_dir"]
    return os.path.join(out, "feel_conf")


def feel_first(args, out):
    """Feel out the table first (Nick, 2 Nov 18:33): on the first run on a table (per site, per table, per camera
    setup), before the production run: a run of its own (calibration, the crawls), then the comparison, saved as the
    table's calibration; later runs use the saved one. --feel-table: auto (default for the robots), always, skip
    (default for a dry run). Returns an exit code to stop with, or 0 to go on to the production run."""
    import feel_table as FTB
    import production as PR
    mode = args.feel_table or ("skip" if args.dry or (args.site_ or {}).get("feel_out", True) is False else "auto")
    args.feel_cal_ = None
    if mode == "skip":
        return 0
    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    if args.map:
        m = FM.load(args.map)
        m["tasks"] = {}
    elif typed_site(args):
        m = PR.site_production(args.site_, robots)[0]       # the site's homes (2026-11-03: at points P11 / P22)
        m["tasks"] = {}
    else:
        a_, b_ = (float(v) for v in args.table.lower().split("x"))
        m, _ = PR.site_map((max(a_, b_), min(a_, b_)), robots)
    conf = feel_conf_dir(args, out)
    tb_ = (args.site_ or {}).get("table") or {}
    # the table's name: the site's typed size when it has one (a refit or a moved side never renames it)
    key = FTB.table_key(table=(tb_["length_cm"], tb_["width_cm"])) if tb_.get("length_cm") and tb_.get("width_cm") \
        else FTB.table_key(m)
    sid = "simulator" if args.dry else FTB.setup_id(os.path.join(conf, "tagcal"))
    need, cal, why = FTB.needed(conf, key, sid, force=(mode == "always"))
    say("FEEL OUT THE TABLE: %s" % why)
    if not need:
        args.feel_cal_ = cal
        for w in (cal or {}).get("adjusted") and cal.get("words") or []:
            say("  (saved) " + w)
        return 0
    fdir = os.path.join(out, "feel")
    kg = args.on_problem_ == "keep_going"
    # the feel-out's own runs: the same policy; a lost robot's crawls are not handed on (they are its half's edges)
    extra = ["--robots", ",".join(robots), "--seed", str(args.seed), "--timeout-s", str(args.timeout_s),
             "--on-problem", args.on_problem_, "--lost-robot-points", "skip"]
    if args.dry and args.sim_true_floor:
        extra += ["--sim-true-floor", args.sim_true_floor]
    for a in args.sim_arg:
        extra += ["--sim-arg=%s" % a]
    for ip in args.robot:
        extra += ["--robot", ip]
    if args.no_frames:
        extra += ["--no-frames"]
    site_name = (args.site_ or {}).get("name")
    # 1. the calibration, then each robot standing still at home: the surface check (the risk review, 2 Nov)
    sdir = os.path.join(out, "surface")
    smp, spp = FTB.prepare_surface(m, sdir)
    say("feel-out, 1 of 2: each robot calibrated, then standing still %.0f s: is the tabletop clearly apart from the "
        "edge sensors' thresholds?" % FTB.SURFACE_HOLD_S)
    code = subprocess.call([PY] + FTB.mission_run_args(smp, spp, sdir, site=site_name, dry=args.dry, extra=extra,
                                                       calibrate=True), cwd=REPO)
    if code and kg:
        say("KEEP GOING: the surface check did not finish (exit %s, see %s): no crawls; the production run uses the "
            "cameras' outline as it is" % (code, sdir))
        return 0
    if code:
        say("THE SURFACE CHECK DID NOT FINISH (exit %s): the production run is not started. Look at %s" % (code, sdir))
        return code or 1
    surf = FTB.surface_check(os.path.join(sdir, "run"))
    for w in surf["words"]:
        say("  " + w)
    if not surf["ok"]:
        cal = FTB.surface_cal(conf, key, sid, FTB.rect(FM.normalize(m)), surf, site=site_name)
        say("WARNING: no crawls: the production run uses the camera's outline as it is (saved: %s)"
            % FTB.cal_path(conf, key))
        args.feel_cal_ = cal
        return 0
    # 2. the crawls (the calibration is kept: same power-on)
    mp, pp, sp, mk = FTB.prepare(m, fdir)
    say("feel-out, 2 of 2: %d spots (%s), about %.0f s" % (
        len(sp), ", ".join("%s %d" % (n, len([s for s in sp if s["robot"] == n])) for n in robots), mk))
    cmd = [PY] + FTB.mission_run_args(mp, pp, fdir, site=site_name, dry=args.dry, extra=extra)
    code = subprocess.call(cmd, cwd=REPO)
    if code and not kg:
        say("THE FEEL-OUT DID NOT FINISH (exit %s): the production run is not started. Look at %s" % (code, fdir))
        return code or 1
    if code:
        say("KEEP GOING: the feel-out did not finish (exit %s, see %s): what it felt is used; the production run "
            "starts" % (code, fdir))
    try:
        res, cal = FTB.finish(FM.load(mp), os.path.join(fdir, "run"), sp, conf, key, sid, site=site_name, surface=surf,
                              trust=args.feel_out_trust_)
    except Exception as e:     # noqa: BLE001
        if not kg:
            raise
        say("KEEP GOING: the feel-out's result could not be read (%s: %s): the cameras' outline is used" % (
            type(e).__name__, e))
        return 0
    for w in res["words"]:
        say("  " + w)
    say("saved as this table's calibration: %s" % FTB.cal_path(conf, key))
    if res.get("hold") and kg:
        say("KEEP GOING: the outline is too far off on a side (above) and was not moved; the production run starts "
            "on the cameras' outline")
        args.feel_cal_ = cal
        return 0
    if res.get("hold"):
        say("NOT STARTED: the production run waits (the outline is too far off). Do the camera setup again.")
        return 4
    args.feel_cal_ = cal
    return 0


def resolve_experiment(args):
    """--experiment / --swap-halves (2026-11-03): args.experiment_, args.swap_, args.swap_words_. The shakedown is the
    no-camera run unless a tracking was chosen."""
    args.experiment_, args.swap_, args.swap_words_ = None, False, ""
    if not args.experiment:
        return
    if not typed_site(args):
        raise SystemExit("--experiment needs a site with typed production points (school-conference). Nothing moved.")
    import experiments as EX
    args.experiment_ = args.experiment
    if args.experiment == "shakedown":
        args.swap_, args.swap_words_ = False, "shakedown: halves as usual"
        if args.tracking == "auto":
            args.tracking_ = "none"
            say("SHAKEDOWN: tracking none (odometry only), the site's default orders")
    else:
        args.swap_, args.swap_words_ = EX.swap_for(args.site_, args.swap_halves)
    say("EXPERIMENT %s: %s; %s" % (args.experiment, EX.KINDS[args.experiment]["name"], args.swap_words_))


def run(args):
    gap_args(args)
    resolve_site(args)
    resolve_experiment(args)
    tag = "mission" + ("-dry" if args.dry else "")
    out = os.path.abspath(args.session or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") +
                                                       "-" + tag))
    os.makedirs(out, exist_ok=True)
    frames_dir = None if args.no_frames else DG.frames_dir_for(out)[0]
    fake = None
    if args.dry and args.sim_free:
        fake = dict((k, float(v)) for k, v in (kv.split("=") for kv in args.sim_free.split(",")))
    guard = DG.Guard(card=out, frames_dir=frames_dir, fake=fake)
    say(guard.words(frames_on=not args.no_frames))
    st0 = guard.state()
    if st0["stop"]:
        say("NOT STARTED: %s. Nothing moved. Free space first: ./run-fleet-pi.sh space, then "
            "./run-fleet-pi.sh clean-uploaded" % st0["why"])
        return 3
    if args.tracking_ == "tags":
        args.tags_json = os.path.join(out, "tags.json")
        with open(args.tags_json, "w") as f:
            json.dump(args.tags_, f, indent=1)
    args.session_id = args.session_id or None
    with open(os.path.join(out, "mission_cmd.json"), "w") as f:
        json.dump({"argv": sys.argv, "args": {k: v for k, v in vars(args).items() if k != "site_"}}, f, indent=1,
                  default=str)
    if args.tracking_ == "tags" and not args.mission and not args.plan_only:
        code_ = feel_first(args, out)
        if code_ or args.feel_only:
            return code_
    if args.mission:
        # a mission made elsewhere (Mission Control's warm-up): its map and plan as they are
        mp, pp = os.path.abspath(args.mission), os.path.abspath(args.plan)
    else:
        mm, plan, mp, pp = build(args, out)
    if args.plan_only:
        return 0
    sess = os.path.join(out, "run")
    os.makedirs(sess, exist_ok=True)
    rp = [os.path.join(HERE, "run_plan.py"), "--map", mp, "--plan", pp, "--session", sess, "--fixes", "udp",
          "--poses-to", "127.0.0.1:5011", "--replan-from-camera", "--timeout-s", str(args.timeout_s),
          "--border-cm", str(args.border_cm), "--min-gap-cm", str(args.min_gap_cm),
          "--place-tol-cm", "4"]
    if args.tracking_ == "none":
        # no camera at all (2026-11-03): run_plan's own "none" source (each robot gets its start pose once, then wheels
        # and gyro); nothing waits for a camera (no facing check, no turn pause, no re-plan from the camera)
        rp[rp.index("--fixes") + 1] = "none"
        rp.remove("--replan-from-camera")
        args.no_facing_check = True
        args.no_turn_pause = True
        say("TRACKING: none (odometry only). Each robot must stand on its home sticker, facing +y (away from the "
            "A3-A4 edge), before Run: its start pose is taken as exact. The +- at each point grows 0.07 cm per cm "
            "driven (29 Oct logs, measured to 40 cm only); no camera checks anything during the run.")
    if not args.no_turn_pause:
        rp += ["--turn-pause"]         # after each turn, stand until the cameras agree (2026-10-29)
    # the keep-going policy (2026-11-03): the site's on_problem (home: stop, as before)
    rp += ["--on-problem", args.on_problem_, "--lost-robot-points", args.lost_robot_points_]
    say("on a problem: %s" % ("KEEP GOING (said once, logged, worked round; a lost robot's points %s)" % (
        "go to the robots still running" if args.lost_robot_points_ == "reassign" else "are listed as not done")
        if args.on_problem_ == "keep_going" else "stop the run (as before)"))
    if args.tracking_ == "tags":
        # confidence at every point: a bumped camera (the bridge's events) counts in the +-
        rp += ["--bridge-events", os.path.join(out, "bridge", "events.jsonl")]
    if args.no_mirror:
        rp += ["--no-mirror"]          # mirrored legs at their planned speed, no waits (the points still measured)
    if args.below_net_doe:
        rp += ["--below-net-doe"]
    for c in EDGE_GUARD:
        rp += ["--send", c]
    for ip in args.robot:
        rp += ["--robot", ip]
    rp += list(args.rp_arg)
    cond = {}
    tag_setup_thread = None
    if args.dry and args.tracking_ == "tags":
        # the AprilTag robots in the simulator (2026-11-02): two tag cameras (tools/sim/tag_cam.py) over the table, the
        # setup (corner and sticker clicks from the truth + noise, the tags read, site_fit) run as Nick would run it,
        # and the bridge and the facing check on the FITTED cameras, not the true ones
        import tag_cam as TCAM
        import tag_setup as TSU
        mmap = FM.load(mp)
        fl = mmap["floor"]
        L_, W_ = fl["x_max"] - fl["x_min"], fl["y_max"] - fl["y_min"]
        if args.sim_true_floor:
            # the real table where the map's outline is wrong (feel out the table, 2026-11-02): the tag cameras draw it
            # and the edge sensors meet it; the planner keeps the map's outline
            tf_ = [float(v) for v in args.sim_true_floor.split(",")]
            L_, W_ = tf_[2] - tf_[0], tf_[3] - tf_[1]
            rp += ["--sim-arg=--true-floor=%s" % args.sim_true_floor]
        # the conference table's drop (2026-11-02): past an open edge the floor is far below, not home's plywood step
        drop_ = (((args.site_ or {}).get("table") or {}).get("drop_cm") or DROP_CM)
        if not any(str(a).startswith("--drop-cm") for a in args.sim_arg):
            rp += ["--sim-arg=--drop-cm=%g" % float(drop_)]
        w_, h_ = (int(v) for v in args.cam_size.split("x"))
        tcfg = TCAM.default_config(table=(L_, W_), size=(w_, h_), tags=args.tags_, seed=args.seed + 4,
                                   refs=uses_refs(args), hide_refs=[x for x in args.hide_ref.split(",") if x.strip()])
        for b_ in args.bump:
            # a camera knocked mid-run (2026-11-03): CAM@T:DEG[:AXIS], T s after the plan starts
            c_, _, rest = b_.partition("@")
            t_, _, rest = rest.partition(":")
            deg_, _, ax_ = rest.partition(":")
            tcfg.setdefault("bumps", []).append({"cam": c_, "t": float(t_), "deg": float(deg_ or 1.0),
                                                 "axis": ax_ or "x"})
        if mmap.get("spots"):
            tcfg["stickers"] = {k: v[:2] for k, v in mmap["spots"].items()}
        for g in args.tag_gap:
            n_, _, rest = g.partition("@")
            t_, _, d_ = rest.partition(":")
            tcfg.setdefault("gaps", []).append({"robot": n_, "t": float(t_), "dur": float(d_ or 10)})
        for lr in args.cam_low_res:
            c_, _, wh = lr.partition("=")
            tcfg.setdefault("low_res", {})[c_] = [int(v) for v in wh.split("x")]
        tcp = os.path.join(out, "tag_cams.json")
        with open(tcp, "w") as f:
            json.dump(tcfg, f, indent=1)
        tagcal = os.path.join(out, "tagcal")
        cal = os.path.join(tagcal, "overhead.json")
        scal = os.path.join(tagcal, "side.json") if not args.no_side else ""
        stream, side_stream = "127.0.0.1:8765", "127.0.0.1:8766"
        rp += ["--dry-run", "--yes", "--seed", str(args.seed), "--sim-arg=--fix-hz=0", "--sim-arg=--tag-cams=%s" % tcp]
        import threading as _th

        def _setup():
            try:
                TSU.sim_setup(os.path.join(sess, "sim"), {"overhead": stream, "side": side_stream}, tcfg, tagcal,
                              typed=not args.no_typed, say=say, corner_clicks=corner_clicks(args))
            except Exception as e:     # noqa: BLE001
                say("SETUP FAILED: %s" % e)
        tag_setup_thread = _th.Thread(target=_setup, daemon=True)
    if args.dry:
        if args.tracking_ == "none":
            cal = scal = ""
            stream, side_stream = args.stream, args.side_stream
            rp += ["--dry-run", "--yes", "--seed", str(args.seed)]
        elif args.tracking_ != "tags":
            cal = os.path.join(sess, "sim", "table_camera.json")
            scal = os.path.join(sess, "sim", "side_camera.json") if not args.no_side else ""
            stream, side_stream = "127.0.0.1:8765", "127.0.0.1:8766"
            rp += ["--dry-run", "--yes", "--seed", str(args.seed), "--sim-arg=--fix-hz=0",
                   "--sim-arg=--serve-live=8765", "--sim-arg=--live-calib-out=%s" % cal]
            if scal:
                rp += ["--sim-arg=--live-side=8766", "--sim-arg=--live-side-calib-out=%s" % scal]
            if args.washout:
                rp += ["--sim-arg=--live-washout"]
        if args.drop_pct:
            cond["drop_pct"] = args.drop_pct
        if args.dead:
            n, _, t = args.dead.partition("@")
            cond["robot_out"] = {"name": n, "t": float(t or 60), "mode": "dead"}
        if args.sim_cond:
            with open(args.sim_cond) as f:
                cond.update(json.load(f))
        rp += ["--sim-arg=%s" % a for a in args.sim_arg]
        if cond:
            cp = os.path.join(out, "sim_cond.json")
            with open(cp, "w") as f:
                json.dump(cond, f)
            rp += ["--sim-arg=--sweep-cond=%s" % cp]
        launcher = [PY, os.path.join(HERE, "sweeps", "rp_shim.py")]     # sim_shim's conditions (drops, a dead robot)
    elif args.tracking_ == "none":
        cal = scal = ""                        # no camera models needed (odometry only)
        stream, side_stream = args.stream, args.side_stream
        launcher = [PY]
    elif args.tracking_ == "tags":
        # the AprilTag site on the Pi: the camera models the tag setup saved (Mission Control's Setup, Measure)
        tc = os.path.join((args.site_ or {}).get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
        cal = args.calib or os.path.join(tc, "overhead.json")
        scal = "" if args.no_side else (args.side_calib or os.path.join(tc, "side.json"))
        if scal and not os.path.exists(scal):
            say("no model for camera 2 (%s): camera 1 alone" % scal)
            scal = ""
        if not os.path.exists(cal):
            raise SystemExit("No camera model at %s: run the setup first (Mission Control, Setup, Measure). Nothing "
                             "moved." % cal)
        stream, side_stream = args.stream, args.side_stream
        launcher = [PY]
    else:
        import setup_conf as SCF
        # setup_conf returns (path, words): take the path, print the words (2026-10-29 20:46 on the Pi: the tuple
        # went to the camera bridge and the robots' run never started)
        if args.calib:
            cal = args.calib
        else:
            cal, words = SCF.overhead_calib(relative=False)
            say(words)
            if not cal:
                raise SystemExit("No lights floor calibration for this site (above). Nothing moved.")
        if args.no_side:
            scal = ""
        elif args.side_calib:
            scal = args.side_calib
        else:
            scal, words = SCF.side_calib()
            scal = scal or ""
            say(words)
        stream, side_stream = args.stream, args.side_stream
        launcher = [PY]
    if args.tracking_ == "tags" and cal:
        # confidence at every point (2026-11-03): each camera's fit error from the setup's site_fit.json
        rp += ["--cam-fit", os.path.join(os.path.dirname(os.path.abspath(cal)), "site_fit.json")]
    # the facing check (2026-10-29): run_plan measures which way each robot faces from its lights before anything
    # moves, through both cameras (facing.py)
    if not args.no_facing_check:
        rp += ["--facing-check", "--facing-calib", cal, "--facing-stream", stream,
               "--facing-tol-deg", str(args.facing_tol_deg)]
        if args.tracking_ == "tags":
            rp += ["--facing-tags", args.tags_json]
        if not args.no_straighten:
            # start straightening (Nick, 2026-10-29 15:41): a robot off its start pose is turned or driven to its spot,
            # the camera watching, and checked again, instead of a stop (straighten.py)
            rp += ["--straighten"]
        if scal:
            rp += ["--facing-side-calib", scal, "--facing-side-stream", side_stream]
    # the record: cpu.csv, frames (tracked pictures and clips)
    cpu = frames = tail = None
    try:
        import e02_cpulog as CL
        cpu = CL.CpuLog(out, procs={"mission": os.getpid()},
                        find={"recorder": "camrec.py", "bridge": "cam_bridge.py", "run_plan": "run_plan.py"})
        cpu.start()
    except Exception as e:     # noqa: BLE001
        say("(cpu log not started: %s)" % e)
    br, brf = (None, None) if args.tracking_ == "none" else start_bridge(args, out, mp, cal, scal, stream, side_stream)
    if args.tracking_ == "none":
        say("no camera bridge (tracking none)")
    elif args.tracking_ == "tags":
        say("tag bridge starting (each robot found by its roof tag, wherever it is)")
    else:
        say("camera bridge starting (it learns the empty floor round the robots: keep hands out of the picture)")
    if args.tracking_ == "none":
        pass                                   # no camera pictures to save
    elif not args.no_frames and not st0["frames_ok"]:
        say("FRAMES NOT SAVED: %s" % st0["why_frames"])
    elif not args.no_frames:
        try:
            import doe_loop as DL
            if (args.site_ or {}).get("frames_part_s"):
                # (2026-11-03) the tracking-rate pictures are held in memory until each .avi part is written: 5 min
                # parts took the simulator's mission_run to 1.2 GB, 3 GB while writing (the 450 s experiments). A site
                # may cut shorter parts (school: 60 s); home keeps doe_loop's 300 s
                DL.PART_S = float(args.site_["frames_part_s"])
                say("frames: .avi parts of %.0f s (the site's frames_part_s)" % DL.PART_S)
            streams = [("overhead", stream, 12.0)] + ([("side", side_stream, 10.0)] if scal else [])
            if args.dry:
                time.sleep(0.1)
            frames = DL.Frames(out, streams, frames_dir=frames_dir, guard=guard)
            tail = DL.Tail(os.path.join(sess, "coordinator.jsonl"), frames)
            tail.start()
            frames.active(True, "mission")
        except Exception as e:     # noqa: BLE001
            say("(frame saving not started: %s)" % e)
    time.sleep(BRIDGE_START_S)
    say("the robots start now. To stop them at any moment: Ctrl+C")
    code = 1
    # (rp_shim.py stands in for run_plan.py on a dry run and takes run_plan's arguments)
    try:
        child = subprocess.Popen(launcher + (rp[1:] if args.dry else rp), cwd=REPO)
    except BaseException:
        if br is not None:
            br.send_signal(signal.SIGINT)    # never leave the bridge running on its own (2026-11-02: one did)
        raise
    if tag_setup_thread is not None:
        tag_setup_thread.start()
    sync_mark(out, "start", args)
    exp_meta = None
    try:
        with open(os.path.join(out, "program.json")) as f:
            exp_meta = ((json.load(f).get("production") or {}).get("experiment"))
    except (OSError, ValueError):
        pass
    if exp_meta and not args.dry and args.site_:
        # the run counter (swap halves every other run): one more production run with an experiment on the robots
        import experiments as EX
        d_ = EX.counter_add(args.site_, {"experiment": exp_meta.get("id"), "swap_halves": exp_meta.get("swap_halves"),
                                         "session": os.path.basename(out), "session_id": args.session_id})
        say("run counter at this site: %d (next run's halves: %s)" % (d_["runs"], "swapped" if d_["runs"] % 2 else
                                                                        "as usual"))
    import threading
    disk_said, disk_stop = [], threading.Event()
    threading.Thread(target=space_watch, args=(guard, child, disk_said, disk_stop), daemon=True).start()
    moved_said = []
    if args.tracking_ == "tags":
        threading.Thread(target=bridge_watch, args=(os.path.join(out, "bridge", "events.jsonl"), disk_stop,
                                                    moved_said), daemon=True).start()
    try:
        code = child.wait()
    except KeyboardInterrupt:
        say("stopping every robot")
        child.send_signal(signal.SIGINT)
        try:
            code = child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            child.kill()
    finally:
        disk_stop.set()
        for p, f in ((br, brf),) if br is not None else ():
            try:
                p.send_signal(signal.SIGINT)
                p.wait(timeout=10)
            except (subprocess.TimeoutExpired, OSError):
                p.kill()
            f.close()
        if tail is not None:
            tail.stop()
        if frames is not None:
            frames.active(False)
            frames.close()
        if cpu is not None:
            try:
                cpu.stop()
            except Exception:     # noqa: BLE001
                pass
    sync_mark(out, "end", args)
    summ = summarise(out, sess, frames, cond)
    try:
        # the experiment (2026-11-03): id, orders, swap, seed, pinned speeds; the sync wiggle's times per robot
        import experiments as EX
        if exp_meta:
            summ["experiment"] = dict(exp_meta, swap_words=getattr(args, "swap_words_", ""),
                                      tracking=args.tracking_)
        if getattr(args, "speed_", None):
            summ["speed_pinned"] = args.speed_
        wg = EX.wiggle_events(os.path.join(sess, "coordinator.jsonl"))
        if wg:
            summ["sync_wiggle"] = wg
            for n_, w_ in sorted(wg.items()):
                say("SYNC WIGGLE %s: %.3f -> %.3f (Pi clock, %.1f s)" % (n_, w_["start"], w_["end"],
                                                                         w_["end"] - w_["start"]))
        with open(os.path.join(sess, "experiment.json"), "w") as f:
            json.dump({"experiment": exp_meta, "speed_pinned": getattr(args, "speed_", None), "sync_wiggle": wg,
                       "tracking": args.tracking_}, f, indent=1)
        if not args.no_run_exports:
            import run_exports as RX
            summ["exports"] = RX.export_all(out, say=say)
    except Exception as e:     # noqa: BLE001  (an export never stops the run's end)
        say("(experiment record / exports not made: %s)" % e)
    with open(os.path.join(out, "mission_summary.json"), "w") as f:
        json.dump(summ, f, indent=1, default=str)
    if args.tracking_ == "tags":
        summ["camera_moved"] = [{k: e.get(k) for k in ("cam", "t", "median_px", "about_cm")} for e in moved_said]
    try:
        import sync_tools as ST
        p_, counts = ST.export_timeline(out)
        summ["pose_timeline"] = {"file": p_, "rows": counts}
        say("pose timeline: %s (%s)" % (p_, ", ".join("%s %d" % kv for kv in counts.items())))
        if args.tracking_ == "tags" and getattr(args, "refit_bg", False):
            import site_fit as SF
            ob = os.path.join(out, "tagcal", "obs.json") if args.dry else os.path.join(
                (args.site_ or {}).get("conf_dir") or "", "tagcal", "obs.json")
            to = os.path.join(out, "bridge", "tag_obs.csv")
            if os.path.exists(ob) and os.path.exists(to):
                st = SF.start_refit_bg(ob, to, os.path.join(out, "tagcal_refit"), python=PY, cwd=REPO)
                summ["refit"] = {"background": True, "folder": os.path.join(out, "tagcal_refit"), "state": st["state"]}
                say("refit started in the background (its own process, a few minutes; the run is over and nothing waits "
                    "for it): %s" % os.path.join(out, "tagcal_refit"))
        if args.tracking_ == "tags" and args.refit and not getattr(args, "refit_bg", False):
            import site_fit as SF
            ob = os.path.join(out, "tagcal", "obs.json") if args.dry else os.path.join(
                (args.site_ or {}).get("conf_dir") or "", "tagcal", "obs.json")
            to = os.path.join(out, "bridge", "tag_obs.csv")
            if os.path.exists(ob) and os.path.exists(to):
                rf = SF.refit(ob, to, os.path.join(out, "tagcal_refit"))
                summ["refit"] = {"size_cm": rf["size_cm"], "rms_px": rf["rms_px"], "words": rf["words"]}
                say("refit from this run's tag reads: table %.1f x %.1f cm (models in %s)" % (
                    rf["size_cm"][0], rf["size_cm"][1], os.path.join(out, "tagcal_refit")))
        with open(os.path.join(out, "mission_summary.json"), "w") as f:
            json.dump(summ, f, indent=1, default=str)
    except Exception as e:     # noqa: BLE001
        say("(pose timeline / refit not made: %s)" % e)
    if disk_said:
        summ["ended_by"] = "the card nearly full: " + disk_said[0]
        try:
            with open(os.path.join(out, "mission_summary.json"), "w") as f:
                json.dump(summ, f, indent=1, default=str)
        except OSError:
            pass
    if args.upload and not args.dry:
        up = [PY, os.path.join(TOOLS, "pi", "upload_session.py"), "--recordings", args.recordings, "--session", out,
              "--tag", "mission-" + os.path.basename(out), "--unique-tag", "--logs-only"]
        if frames_dir and os.path.isdir(frames_dir):
            # the frames as their own asset (rec-frames-small.zip), as the DOE loop's; not inside the logs zip
            up += ["--recording-dir", frames_dir, "--exclude", "frames"]
        say("uploading: " + " ".join(up[1:]))
        subprocess.call(up, cwd=REPO)
    for q in summ.get("points") or []:
        say("POINT %s%s" % (q.get("words"), "" if q.get("truth_err_cm") is None else
                            "  [simulator truth: %.1f cm off, %s]" % (q["truth_err_cm"], "inside" if q.get("within")
                                                                     else "OUTSIDE its \u00b1")))
    for q in summ.get("points_skipped") or []:
        say("SKIPPED %s (%s): %s" % (q["point"], q["robot"], q["why"]))
    for q in summ.get("points_reassigned") or []:
        say("REASSIGNED %s: %s -> %s" % (q["point"], q["frm"], q["to"]))
    if summ.get("on_problem") == "keep_going":
        say("keep going: %d problem(s) worked round without stopping the run" % len(summ.get("kept_going") or []))
    say("result: %s; saved in %s" % (summ.get("result"), out))
    return code


def point_truth(points, truth_csv, sess):
    """Simulator only: each point's estimate against the simulator's true pose at the point's time (the median over
    the dwell's last second): truth_err_cm and whether it fell inside its +- (within)."""
    import csv as _csv
    rows = {}
    try:
        with open(truth_csv) as f:
            for r in _csv.DictReader(f):
                rows.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
        with open(os.path.join(sess, "coordinator.jsonl")) as f:
            t_run = None
            for ln in f:
                e = json.loads(ln)
                if e.get("event") == "message" and str(e.get("text", "")).startswith("plan ") and "started" in e["text"]:
                    t_run = e["t"]
                    break
    except (OSError, ValueError, KeyError):
        return
    if t_run is None:
        return
    for q in points:
        if q.get("x") is None or q.get("t_run") is None or q["robot"] not in rows:
            continue
        t1 = t_run + q["t_run"]
        near = [(x, y) for t, x, y in rows[q["robot"]] if t1 - 1.0 <= t <= t1]
        if not near:
            continue
        import statistics as _st
        tx, ty = _st.median([p[0] for p in near]), _st.median([p[1] for p in near])
        q["truth_err_cm"] = round(((q["x"] - tx) ** 2 + (q["y"] - ty) ** 2) ** 0.5, 2)
        q["within"] = q["truth_err_cm"] <= (q.get("plus_minus_cm") or 0)


def summarise(out, sess, frames, cond):
    """mission_summary.json: what the run did against the plan, the simulator's truth (dry), the line check."""
    s = {"folder": out, "conditions": cond}
    try:
        with open(os.path.join(sess, "run_plan_summary.json")) as f:
            rs = json.load(f)
    except (OSError, ValueError):
        rs = {}
    s.update(result=rs.get("result"), duration_s=rs.get("duration_s"), planned_s=rs.get("planned_makespan_s"),
             replans=len([r for r in rs.get("replans", []) if r.get("source") not in ("improve",)]),
             robots_out=rs.get("robots_out"), plans_refused=rs.get("plans_refused"),
             sensor_stops=len(rs.get("sensor_stops") or []), confirm_checks=rs.get("confirm_checks"),
             battery=rs.get("battery"), start_checks=rs.get("start_checks"))
    # keep going and confidence at every point (2026-11-03)
    s.update(on_problem=rs.get("on_problem"), kept_going=rs.get("kept_going") or [], points=rs.get("points") or [],
             points_skipped=rs.get("points_skipped") or [], points_reassigned=rs.get("points_reassigned") or [])
    tru = os.path.join(sess, "sim", "truth.csv")
    if s["points"] and os.path.exists(tru):
        point_truth(s["points"], tru, sess)
    mir = rs.get("mirror") or {}
    if mir.get("on"):
        s["mirror"] = {k: mir.get(k) for k in ("lane_ends", "all_points", "holds", "retimed_legs", "target_s",
                                                "target_cm")}
        le = mir.get("lane_ends") or {}
        if le.get("time_s"):
            say("mirroring: %d lane ends, time apart max %.2f s (median %.2f), position max %.1f cm (median %.1f); "
                "%d within %.1f s and %.0f cm" % (le["time_s"]["n"], le["time_s"]["max"], le["time_s"]["median"],
                                                   le["pos_cm"]["max"], le["pos_cm"]["median"], le["within_target"],
                                                   mir.get("target_s") or 0.5, mir.get("target_cm") or 3.0))
    sim = rs.get("simulator") or {}
    if sim:
        s["sim"] = {k: sim.get(k) for k in ("min_body_gap_cm", "min_body_gap_at", "collision", "min_open_edge_cm",
                                            "min_open_edge_at", "hung")}
    try:
        import line_check as LC
        tru = os.path.join(sess, "sim", "truth.csv")
        lc = LC.check(sess, os.path.join(out, "bridge"), truth=tru if os.path.exists(tru) else None,
                      png=os.path.join(out, "line_check.png"))
        s["line_check"] = {n: dict(p95_cm=r["camera"].get("p95_cm"), within_pct=r["camera"].get("within_gate_pct"),
                                   misses=r["camera"].get("misses"), sure_share_pct=r["sure_share_pct"],
                                   camera_vs_truth=r.get("camera_vs_truth"))
                           for n, r in lc["robots"].items()}
        s["closest_by_camera"] = lc.get("closest_pair_camera")
    except Exception as e:     # noqa: BLE001
        s["line_check"] = "not made: %s" % e
    try:
        with open(os.path.join(out, "bridge", "bridge_summary.json")) as f:
            bs = json.load(f)
        s["camera"] = {"frames": bs.get("frames"), "robots": bs.get("robots"), "farm": bs.get("farm")}
    except (OSError, ValueError):
        pass
    if frames is not None:
        s["frames"] = frames.words()
    # labels the robots reached (blocks done), from the step reports
    done = {}
    try:
        with open(os.path.join(sess, "coordinator.jsonl")) as f:
            plan_steps = {}
            for ln in f:
                e = json.loads(ln)
                if e.get("event") == "labelled_step" and e.get("state") == "done":
                    lab = str(e.get("label", "")).split(" lane")[0]
                    done.setdefault(e["robot"], [])
                    if lab not in done[e["robot"]]:
                        done[e["robot"]].append(lab)
        s["reached"] = done
    except (OSError, ValueError):
        pass
    try:
        with open(os.path.join(out, "mission_summary.json"), "w") as f:
            json.dump(s, f, indent=1, default=str)
    except OSError as e:
        say("(mission_summary.json not written: %s)" % e)
    return s


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--table", default="61x122", help="the table's size in cm, e.g. 61x122 (without --map)")
    ap.add_argument("--map", help="Mission Control's workspace map (sides, zones from the warm-up)")
    ap.add_argument("--program", help="a block program (Mission Control's program.json); default: Monday's")
    ap.add_argument("--robots", default="Left,Right", help="Monday's two robots (default Left,Right)")
    ap.add_argument("--mission", help="a mission map made elsewhere (Mission Control's warm-up), with --plan")
    ap.add_argument("--plan", help="with --mission: its plan")
    ap.add_argument("--session", help="the session folder (default logs/<date-time>-mission)")
    ap.add_argument("--plan-only", action="store_true", help="build and plan, then stop (nothing moves)")
    ap.add_argument("--min-gap-cm", type=float, default=EC.MIN_GAP_CM)
    ap.add_argument("--border-cm", type=float, default=6.0)
    ap.add_argument("--timeout-s", type=float, default=600.0)
    ap.add_argument("--robot", action="append", default=[], help="a robot's IP (default: found on the WiFi)")
    ap.add_argument("--calib", help="the overhead's floor calibration (default: what the setup page saved)")
    ap.add_argument("--side-calib", help="the side camera's (default: what the setup page saved)")
    ap.add_argument("--no-side", action="store_true", help="the overhead camera alone")
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--side-stream", default="127.0.0.1:8001")
    ap.add_argument("--farm", action="store_true", help="each camera tracked in its own process (trackfarm.py)")
    ap.add_argument("--rings", action="store_true", help="--farm: the recorder shares its pictures (camrec.py --shm)")
    ap.add_argument("--half", action="store_true", help="--farm: track at half size")
    ap.add_argument("--no-facing-check", action="store_true",
                    help="skip the facing check (which way each robot faces, from its lights, before anything moves)")
    ap.add_argument("--facing-tol-deg", type=float, default=15.0,
                    help="the facing check: a robot more than this off its start heading stops the start")
    ap.add_argument("--no-straighten", action="store_true",
                    help="a robot off its start pose stops the start (before 2026-10-29 15:41); default: it is turned "
                         "on the spot or driven to its spot, then checked again (run_plan --straighten)")
    ap.add_argument("--no-frames", action="store_true", help="no tracked pictures or clips")
    ap.add_argument("--upload", action="store_true", help="upload the session (a release with a tag of its own)")
    ap.add_argument("--recordings", default=os.path.expanduser("~/recordings"))
    g = ap.add_argument_group("simulator (dry run)")
    g.add_argument("--dry", action="store_true")
    g.add_argument("--washout", action="store_true", help="the overhead's board washed out (27 Oct 11:20)")
    g.add_argument("--drop-pct", type=float, default=0.0, help="lost packets, %% each way")
    g.add_argument("--dead", help="NAME@T: that robot dies T s into the run (silent, still)")
    g.add_argument("--seed", type=int, default=1)
    g.add_argument("--sim-free", default="", help="simulated free space (GB), e.g. card=0.8,drive=50: the guard's "
                                                  "refusal / frames stop")
    g.add_argument("--sim-cond", help="a JSON file of sim_shim.py conditions (sweeps/sim_shim.py), added to the above")
    g.add_argument("--sim-arg", action="append", default=[], metavar="ARG",
                   help="one more fleet_sim.py argument, e.g. --sim-arg=--turn-err=Left=7.5 (repeatable)")
    ap.add_argument("--rp-arg", action="append", default=[], metavar="ARG",
                    help="one more run_plan.py argument, e.g. --rp-arg=--send --rp-arg='set gyro_scale 1.075'")
    ap.add_argument("--no-turn-pause", action="store_true",
                    help="drive straight on after each turn (before 2026-10-29); default: stand until the cameras "
                         "agree with the robot's own pose (run_plan --turn-pause)")
    g.add_argument("--below-net-doe", action="store_true",
                   help="simulator only: allow robot gaps under the 8 cm safety net (the safety-net DOE)")
    ap.add_argument("--setting", action="append", default=[], metavar="NAME=VALUE",
                    help="a mission setting (missions.SETTINGS), e.g. separation_cm=6.5 (repeatable)")
    ap.add_argument("--site", default="", help="the site profile (default: the one in use; home: lights as before)")
    ap.add_argument("--tracking", choices=["auto", "lights", "tags", "none"], default="auto",
                    help="how the robots are followed: the site's run_tracking (auto), their lights, their roof AprilTags, or none (odometry only, no camera)")
    ap.add_argument("--session-id", default="", help="the session id both teams write down (Mission Control's)")
    ap.add_argument("--refit", action="store_true", help="tags: after the run, fit the cameras again with the run's "
                                                          "tag reads (saved beside the run, not used until chosen); "
                                                          "the run's end waits for it (the old way)")
    ap.add_argument("--refit-bg", action="store_true", help="tags: the same refit, started in the background as its own "
                                                             "process (2026-11-03): the run ends at once; its folder's "
                                                             "tagcal_refit/status.json says when it is done")
    g.add_argument("--cam-size", default="1280x960", help="tags: the simulated cameras' picture size")
    g.add_argument("--tag-gap", action="append", default=[], metavar="NAME@T:DUR",
                   help="tags: that robot's tag covered (unreadable) from T s after the plan starts for DUR s")
    g.add_argument("--cam-low-res", action="append", default=[], metavar="CAM=WxH",
                   help="tags: that camera sends smaller pictures, e.g. side=640x480")
    g.add_argument("--no-typed", action="store_true", help="tags: the simulated setup types no table size")
    g.add_argument("--no-refs", action="store_true", help="tags: no reference tags on the simulated table")
    g.add_argument("--hide-ref", default="", metavar="IDS", help="tags: these reference tags not on the simulated "
                                                                  "table (e.g. 20: corner 1's)")
    g.add_argument("--bump", action="append", default=[], metavar="CAM@T:DEG[:AXIS]",
                   help="tags: that simulated camera knocked T s after the plan starts, turned DEG degrees")
    ap.add_argument("--corner-clicks", choices=["auto", "on", "off"], default="auto",
                    help="tags: the setup's corner clicks (auto: the site's corner_clicks switch; on without one)")
    g.add_argument("--sim-true-floor", default="", metavar="X0,Y0,X1,Y1",
                   help="tags: the real table when the map's outline is wrong (the feel-out's tests)")
    ap.add_argument("--feel-table", choices=["auto", "always", "skip"], default="",
                    help="tags: feel out the table first (its edges felt by the robots against the cameras' outline): "
                         "auto = when this table has no saved calibration for this camera setup (default for the "
                         "robots), always, skip (default for --dry)")
    ap.add_argument("--feel-only", action="store_true", help="tags: feel out the table (with --feel-table) and stop: "
                                                               "no production run")
    ap.add_argument("--feel-conf-dir", default="", help="where table calibrations are saved (default: the site's folder; "
                                                         "--dry: the session's)")
    ap.add_argument("--on-problem", choices=["auto", "keep_going", "stop"], default="auto",
                    help="this run only (2026-11-03): keep_going = a problem that used to stop the whole run is said "
                         "once, logged and worked round (only a full card, Stop / Ctrl+C or every robot out end it); "
                         "stop = as before; auto = the site's on_problem (home: stop; school: keep_going)")
    ap.add_argument("--lost-robot-points", choices=["auto", "reassign", "skip"], default="auto",
                    help="with keep_going: a robot that is out hands its points not done to the others (reassign) or "
                         "they are listed as not done (skip); auto = the site's lost_robot_points")
    ap.add_argument("--feel-trust", choices=["auto", "camera", "sensor"], default="auto",
                    help="tags, feel-out: camera = a felt-vs-camera difference is reported and logged, the cameras' "
                         "outline stands and nothing waits; sensor = a side moved in up to 10 cm, held past 10 (as "
                         "built 2 Nov); auto = the site's feel_out_trust")
    ap.add_argument("--experiment", default="", choices=["", "shakedown", "e1", "e2", "e3", "e4", "default"],
                    help="a typed site's experiment routes (experiments.py, Jay-Sun 3 Nov): shakedown (no camera, default "
                         "orders), e1 in order, e2 reversed, e3 zig-zag, e4 shuffled, default; none = the site's default "
                         "orders as before")
    ap.add_argument("--swap-halves", default="auto", choices=["on", "off", "auto"],
                    help="with --experiment: the robots' halves exchanged; auto = every other run (the site's run "
                         "counter; simulator runs read it but never count)")
    ap.add_argument("--sync-wiggle", default="auto", choices=["on", "off", "auto"],
                    help="the sync wiggle at the start (both robots a quarter turn and back at their homes); auto = the "
                         "site's experiments.sync_wiggle.use (school on, home off)")
    ap.add_argument("--no-run-exports", action="store_true",
                    help="no camera_track.csv / odometry_raw.csv in the run folder at the end")
    ap.add_argument("--points-each", type=int, default=3, help="tags, no --program: points per robot")
    ap.add_argument("--dwell-s", type=float, default=3.0, help="tags, no --program: seconds at each point")
    ap.add_argument("--no-mirror", action="store_true",
                    help="Left and Right are not re-timed to mirror each other (before 2026-10-29 20:06); default: the "
                         "late one speeds up and the leader slows down (5-10 cm/s) or waits (run_plan's mirror sync)")
    args = ap.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
