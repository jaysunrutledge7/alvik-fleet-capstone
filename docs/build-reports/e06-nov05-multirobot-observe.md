# Phase 1: multi-robot runs that observe, then trust (5 Nov evening, simulator and tests only)

## Premise

The first real school runs (5 Nov, AL-1DC0, one C920e at the TV end) went wrong wherever the code trusted a setting
instead of what it could see: the roof tag was stuck on 180 deg off (legs ran backwards, 1 cm from an edge); the camera
names defaulted to home's; the site assumed two cameras and gave a second robot zones nobody can see; the line sensors
took the seam's tape for an edge; the tag did not read at P11 or far up the table and the run ended instead of
narrowing; a downscaled frame read a tag the full frame missed; a robot woken from sleep restarted and refused to turn;
tag height was typed per site robot name; the run slept the robot at the end, wiping values it could not save. Nick's
decisions (5 Nov 20:28) for phase 1 of 3: one run for however many robots are on the table, found by the wiggle, each
tag's heading found by its first drive and checked all run long; the table shared over the area where tags actually
read; straight lines with turns on the spot for point visits; Measure's cameras over typed values; self-checks for each
of the evening's settings; a straight-line lost-tag recovery in missions with the others' paths reserved; a one-button
Visit points on Mission Control's Production screen; a simulator that breaks the guesses on purpose.

Added during the build by the coordinator: (a) the real 19:34 calibration's wheel fit "overshot" (find why with the real
legs); (b) Nick 20:46: every run type's robot-side safety layers (tilt stop, bottom-zone drop) switched on and read
back, a tilt or drop trip kept out and backed away from, never forward; (c) the 4-hour home free-roam analysis (Center
tilted 3.4 deg at the x = 0 edge for 64 min): pose disagreement re-seeds, a standing-tilt rule, moves "done" without
moving, reverses only when shown safe, cpu.csv in every run; (d) Nick 21:11: those last two replaced by evidence fusion
(3'), never repeating a failed action unchanged (3b) and swept-space evidence (4'); (e) Nick 21:14: a hard side-edge
invariant (the robot's possible-locations circle never overlaps the perimeter along any planned path) in every run
type, and home has no walls either.

**Measured, and where from.** Real data: the 19:34 calibration's legs.csv / fit.json (copied to
docs/results/calib-school-2026-11-05-1934/). Everything else is the simulator (fleet_sim robots, the tag picture
simulator with the real tag bridge, the point-fix camera) and unit tests; nothing touched a robot or a Pi. Firmware
2026-11-05d passed the stand-in syntax check (tools/fwcheck), as every firmware build here; no real Arduino compile.

## Data

### 1. The wheel fit did not overshoot: a fixed overrun per leg read as "cm per metre"

![wheel fit](../screenshots/2026-11-05/e06-problem-wheel-fit-looked-overshot-but-fixed-overrun-per-leg.png)

Each dot is one straight leg at school: grey before the fit, blue after; the grey line slopes (a scale error the fit
removed), the blue line is flat about 0.43 cm above zero (every leg ends the same few millimetres long).

| | phase A (wheel_scale 1) | phase C (0.9759) | source |
|---|---|---|---|
| along error = slope x length + offset | -2.34 cm/m x L + 0.43 cm | +0.15 cm/m x L + 0.43 cm | legs.csv, legs of 10 cm or more |
| true wheel scale (camera / counted) | fitted 0.9759 | C's own legs say 0.9765 | through the counted lengths |
| overrun at 5 cm/s / 10 cm/s | predicted from A: 0.32 / 0.49 cm | measured in C: 0.31 / 0.52 cm | stop_overrun (speed-split) |
| "signed cm/m" (the old report's figure) | -0.89 (-1.12 on drives only) | +1.31 | the report divided the fixed overrun by short legs |

**Why.** The robot stops when its own count reaches the asked length (minus a 0.15 cm tolerance), then rolls on:
about 0.3 cm at 5 cm/s and 0.5 cm at 10 cm/s. That is the same few millimetres on every leg, so "error per metre" is
large on a 5-50 cm leg and looks like a scale. Not double counting (the fit's coast term already separated it: 0.63 cm),
not wheel_scale applied twice (C's slope is flat), not the 5 cm finals (they were not in the scale fit). **Fixed:**
calib_run 2026-11-05h measures the overrun per speed in phase A and from phase B on sends each leg that much short
("stop lead", `--stop-lead off` keeps the old way); on the real C legs it takes the mean along error from +0.43 to
0.00 cm (test_stop_lead). The report now splits the distance error into its scale part (cm/m) and its fixed part (cm
per leg). The lead is kept on the Pi (stop_leads.json): firmware 2026-10-29b cannot save it. **Simulator case** that
reproduces it (`--sim-coast 0.08`, true wheel 0.976): fit 0.9787, C slope -0.15 cm/m with a fixed +0.23 cm per leg
and "+0.44 cm/m" in the old figure, the same pattern (smaller: the simulated coast is a pure wheel run-down).

### 2. What was built

| Part | Where | In one line |
|---|---|---|
| observe.py (new, shared) | tools/fleet/observe.py | every check below, pure functions, 20 tests |
| Robots found however many | wiggle_bind `--robots auto`, mission_run `--robots auto` | the camera counts the roof tags, the run waits for that many robots; slots Alpha, Bravo, Charlie ... |
| Tag heading | calib_run / wiggle_bind heading_nudge, tag bridge and facing check yaw_off, HeadingWatch in calib_run and run_plan | a 6 cm nudge (or a quarter turn first, if only sideways is clear) after each wiggle; every straight stretch checked after; two disagreeing stretches re-learn it |
| Readability | observe.ReadMap; mission_run run_readmap; run_plan learns it, saves readmap.json, the site keeps it | seeded from the camera models (whole roof in the picture, 14 px), earlier runs at half weight, learned from every robot's reads |
| Table sharing | observe.split_points; mission_run `--split auto` (production.split "halves" kept) | readable points only, cut along the table into equal runs, each to the robot standing nearest; homes where the robots were seen |
| Lost tag in missions | run_plan recover_blind | turn to face where it was last read and drive straight there FORWARDS (edge guard on), path checked against the edges (the invariant) and the others' next steps; points learned unreadable skipped |
| Safety layers | observe.safety_settings; calib_run arm_safety, wiggle_bind, mission_run guard_cmds, freeroam, run-fleet-pi.sh | tilt_stop_deg 5, tilt_ms 300, safe_lift 1, edge_bot_cm 11, safe_edge from the site (school 2) set and read back; not read back = not moved |
| Trips | calib_run after_trip, run_plan trip_back_off, freeroam tilt_trip | a keep-out at the nose, a straight reverse over its own trail or camera-shown room, else hold; firmware 2026-11-05d allows only that |
| Firmware 2026-11-05d | motion.h, pose.h, follow.h, e02.h (+ simulators) | a tilt ends the follow step (not the run); forward drives and turns refused while tilted; a straight backward drive allowed |
| Four protections (3', 3b, 4', invariant) | observe.move_outcome / FailMemory / Trail / back_out_plan / path_ok; freeroam, run_plan, calib_run | below |
| Restarts | freeroam on_boot, calib_run (refused leg), run_plan (existed), wiggle_bind / calib (existed) | a new boot id: safety layers and compass check again, automatically |
| Stay awake | calib_run (`--sleep-at-end` kept), freeroam (`--sleep-parked` kept), Mission Control idle sleeper | robots sleep after 30 min with no run |
| Cameras | camnames.resolve | one plugged camera is the overhead whatever its name; two with wrong names both used, said |
| Detection | tagcore.WindowTracker | a wanted tag missing after the full picture: searched again at half size |
| Surface marks | observe.edge_trip_kind; run_plan | a line-sensor trip with the camera well inside the table: a surface mark, logged; that robot's line cue off (bottom zone stays) |
| Measure | site_fit measure_trust | switch built; default left at "typed" (section 5) |
| Mission Control | Production screen | a Visit points card: one button, a simulator rehearsal, a Stop that turns red |
| cpu.csv | calib_run, freeroam (missions had it) | the heat tracker works for every run |
| Simulator breakers | fleet_sim `--tape`, `--stop-coast`; tag_cam tag_yaw / dark / clock_skew_s; robot.py wheel-over-edge tilt; mission_run `--sim-bind --sim-tag-yaw --sim-dark --sim-clock-skew --sim-robots` | the real binding runs in dry runs too |

![visit points](../screenshots/2026-11-05/e06-feature-visit-points-one-button-card.png)

The new card on Mission Control's Production screen: one button runs every robot on the table; Stop is always there.

### 3. The four protections (Nick 21:11 / 21:14) as built

- **3' Move outcome by evidence.** move_outcome votes with the wheels (weight 1: they can spin), the accelerometer
  (1.5), the gyro (1.5), the front range against the expected change (2), the line sensors (0.5) and the camera only
  with a fresh fix (3). Physical impossibility (faster than 30 cm/s) decides alone. Own sensors agreeing with the camera
  blind: accepted, bubble grown. Wired: free roam (RAW rows: ax, dC_cm, lineC), run_plan (own pose, RAW, camera),
  calib_run (its LEG record, camera).
- **3b Never repeat unchanged.** FailMemory per robot: the same kind, direction and length (within 25 %) from within
  5 cm of a failure is not planned again; 3 failures within 8 cm = a hazard (keep-out for everyone, re-seed from the
  camera or a find-me, a full stop said).
- **4' Swept space by evidence.** back_out_plan: (a) back along its own breadcrumb trail (no camera), (c) a fresh camera
  fix showing room behind, (d) turn on the spot (only if its circle is clear) so the front sensors check, then forward,
  (e) hold. Free roam's back-off / reverse rungs and the trip back-offs use it; a tilted robot cannot turn, so (d) is
  not offered then.
- **Side-edge invariant.** path_ok: the circle of half diagonal (6.72) + uncertainty (camera error, bubble, the
  camera's staleness x speed) + 1 cm never crosses a side or keep-out at any sample of a planned move (turns included:
  the circle covers any heading); a robot already overlapping may only shrink it. Free roam: every move (feasible,
  curve_ok, back_move). run_plan: the blind watch stops a robot whose circle crosses; the recovery is checked with it;
  planned paths keep 11 cm (planner margins) > 8.7 cm by construction. Calibration: the legs' safety check keeps 13 cm +
  the leg's error, and every leg is aimed from a fresh camera pose. Every side is a drop at home too (site walls false).

{{FREEROAM_CASES}}

### 4. Rehearsals (simulator)

{{REHEARSALS}}

### 5. Measure: cameras over typed values (built, not switched on)

In site_fit's own simulator (seed 2, 14 tag spots, 8 stickers), "cameras" (typed values as a 5 cm seed, the tags'
printed size as the scale) made the table 394.0 x 166.0 cm (true 396.2 x 167.6) and its worst sticker 4.4 cm off; the
tape-trusted fit made 396.3 x 167.5 and 2.4 cm. The blur that reads a tag 1.5-3.5 % small (2 Nov report) is why: the
tape fixes the scale. Observe-then-trust says the evidence wins, so the switch exists (site `measure_trust`:
"cameras" / "typed") but the default stays "typed", and every typed value is still checked against the cameras and a
disagreement over 1.5 % said (as before). Question 1.

## Conclusion

{{CONCLUSION}}

## What's next

{{NEXT}}

## Not built as asked

{{NOTBUILT}}

## Rigid assumptions found, and what replaced them

| Assumption (where) | Replaced by | Switch kept |
|---|---|---|
| The roof tag faces the nose (every tag reader) | the nudge's measured offset per tag, re-learned by the heading watch; bridge, facing check, calibration Eye | none needed (offset 0 when it agrees) |
| The run's robots are the site's two (mission_run `--robots`, wiggle_bind slots) | as many as the camera counts tags and as answer | `--robots Alpha,Bravo` still works |
| Zones 1-2 / 3-4 split at x = 78 in (production.zones) | readable points split between the robots found | production.split "halves" / `--split halves` |
| The camera reads to x 110 cm (`--view-x-max`, VIEW_X_MAX_CM) | ReadMap (camera model, earlier runs, this run); the nudge ignores the typed view limit where the robot is seen | `--view-x-max` kept |
| Camera 2 present (site cameras) | site's side.use false read; missing model = one camera | cameras.side.use |
| Camera names (camnames) | what is plugged in | cameras.json "observe": false |
| A line trip is an edge (edge guard) | camera distance from the edge decides; surface mark logged, line cue off for that robot | site safe_edge |
| Robots wake ready (calibration, free roam, missions) | boot id watched: safety layers and compass check again | - |
| Sleep at the end | stay awake, idle sleeper 30 min | `--sleep-at-end`, `--sleep-parked`, site idle_sleep_min |
| The full frame finds every tag | downscaled retry | tagcore.DOWNSCALE_RETRY 0 |
| A move reported done happened | evidence fusion | - |
| Reversing near an edge is fine after a front stop | trail / camera evidence or turn-then-forward or hold | - |
| The board's sides at home are walls | every side a drop (site walls false) | site "walls": true |
| The fixed stop overrun is a scale (calibration report) | split into scale and fixed parts; the stop lead | `--stop-lead off` |
| Measure's typed size is right | checked, switch built, default kept typed on evidence | site measure_trust |

**Left for phase 3's full audit and motion fusion**: tag height per site robot name (still typed: AL-1DC0 got Alpha's
6.19; the roof check measures it with two cameras only); the free roam's start (ladder L0/L1) still assumes home's
three names; the planner's fixed margins (11 cm) are not yet scaled by live uncertainty (the invariant is checked on
top); the accelerometer threshold and tilt numbers have no bench data; the camera clock is aligned in the free roam
and checked in missions by the camera guard, not in the calibration; feel-out still assumes rectangular tables; the
experiments (e1-e4) still use the fixed halves (an experiment's design); wheel slip is judged per move, not mapped
(discover-not-map: own_vs_cam rows exist); one fused pose (tag + odometry + gyro + accelerometer) is still several
estimators side by side.

## Sweep 1: new tests that may now be necessary

{{SWEEP1}}

## Sweep 2: possible inaccuracies in what this relied on

{{SWEEP2}}

## Questions for Nick

{{QUESTIONS}}
