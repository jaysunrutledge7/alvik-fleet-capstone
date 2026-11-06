# Calibration run, zones 1-2 (SIMULATOR): AL-2B7F

Written automatically by tools/fleet/calib_run.py 2026-11-05h at the run's end. Robot: **AL-2B7F** (WiFi name AL-2B7F), roof tag 3. Result: **finished**.

## Premise
Nick (4 Nov): get enough data without calibration as a base level, use the camera only for safety (edges, keep-outs, the camera's view, parking), then fit the robot's corrections and run a series of tours with them. Every leg is driven on the robot's wheels and gyro alone; the one school camera measures where it actually stopped, and the gap is that leg's odometry error. Speed as in the experiments: 10 cm/s, 60 deg/s turns, last 5 cm at 5 cm/s.

## Data
**Binding** (wiggle (experiments.py sync wiggle, 90 deg and back; pinned robots a 45 deg check)): AL-2B7F (WiFi AL-2B7F, 127.0.0.3) = tag 3 [bound by the wiggle (no pin yet)], AL-1DC0 (WiFi AL-1DC0, 127.0.0.2) = tag 2 [bound by the wiggle (no pin yet)].

**The robot's values before** (read with `get`, saved in robot_values_before.json): wheel_scale 1.0000, gyro_scale 1.0000, gyro_drift_dps 0.0000.

**Phase A started from the defaults (wheel_scale 1, gyro_scale 1, gyro_drift_dps 0)**: asked for (--baseline-defaults / calibration.baseline defaults).

**Fit from phase A** (18 turns, 1 still periods, 12 straight legs; 95 % +- = 1.96 x SE):

| value | before | fitted | +- (95 %) | enough = +- under | verdict | sent | read back | true (simulator) |
|---|---|---|---|---|---|---|---|---|
| wheel_scale | 1.0000 | 0.9755 | 0.0070 | 0.008 | ok | 0.9755 | 0.976 OK | 0.976 |
| gyro_scale | 1.0000 | 1.0921 | 0.0021 | 0.005 | ok | 1.0921 | 1.092 OK | 1.086 |
| gyro_drift_dps | 0.0000 | -0.0772 | 0.0410 | 0.030 | not pinned down: +-0.0410 (need +-0.0300) | 0.0000 | 0.000 OK | 0.000 |

**Stop overrun** (how far past the asked length a leg ends with the new wheel_scale, from phase A; the robot's usual stop shortfall 0.18 cm taken off): +0.37 cm at 5 cm/s (9 legs), +0.37 cm at 10 cm/s (12 legs). Phase C sends each leg short by this (the stop lead).

**Tag offset from the turning point** (from the in-place turns): 0.0 cm ahead, 0.1 cm left (0.1 cm; circle fit +- 0.1 cm rms). Taken off every camera position in this run.

**Per-leg odometry error, before (A: the old values) vs after (C: the fitted values)**, tour legs only (the last 10 cm at an edge point is driven from the camera and left out):

| tour | legs (go / turn) | distance error, cm per m (median / p95) | signed mean, cm per m | scale part, cm per m | fixed part per leg, cm | sideways, cm per m (median) | heading error per turn, deg (median / p95) |
|---|---|---|---|---|---|---|---|
| A (before) | 8 / 8 | 1.80 / 2.79 | -1.95 | -2.11 | +0.08 | 0.30 | 7.35 / 12.30 |
| C1 (after) | 9 / 8 | 0.38 / 0.79 | -0.17 | +0.66 | -0.39 | 0.92 | 0.38 / 1.25 |
| C2 (after) | 9 / 9 | 0.12 / 0.59 | -0.13 | +0.17 | -0.14 | 0.94 | 0.41 / 0.80 |
| C3 (after) | 10 / 10 | 0.21 / 0.65 | +0.02 | +0.20 | -0.08 | 0.34 | 0.37 / 1.71 |

**Gyro drift standing still** (the robot's own heading change while the camera saw no turn): A0 -0.0626 deg/s over 20 s (camera -0.22 deg); C1 -0.0957 deg/s over 10 s (camera +1.33 deg); C2 -0.0760 deg/s over 10 s (camera +0.80 deg); C3 -0.0355 deg/s over 10 s (camera -0.15 deg).

**Safety**: closest the robot's middle came to an edge 20.8 cm (margin 13 cm), to a keep-out or tile 12.3 cm (wanted: the body's half diagonal + 2 cm = 8.7 cm, less 0.5 cm of measuring), furthest x 177.6 cm (the camera reads to 198.1 cm); the camera corrected the pose before 14 legs; holds for a lost tag: 0; points skipped: none.

**Saved on the robot** (Nick, 5 Nov: kept over power-off): saved on the robot with `save` (firmware 2026-11-05a or newer) and checked: wheel_scale 0.9755 (saved), gyro_scale 1.0921 (saved), gyro_drift_dps 0.0000 (saved).

**End**: parked at P01, 0.3 cm from the sticker; `sleep 10` sent (no answer seen). Battery 91 % at the start, 88 % at the end.

**Four-point check** (Jay-Sun re-checks these tomorrow): how far the camera puts the robot with and without the tag-height correction (tag 6.19 cm up), inches, away from the camera:

| point | Jay-Sun (no correction) | model, camera 145 cm above the table | model, 229 cm | sticker (cm) | this run, phase C stops: with correction (cm) | the same read without (cm) | difference (in) |
|---|---|---|---|---|---|---|---|
| P01 | 2.4 | 2.7 | 1.7 | 30.5, 31.8 | 29.9, 30.8 | 36.3, 28.4 | 2.7 |
| P05 | 2.7 | 2.4 | 1.5 | 15.9, 126.4 | not visited | - | - |
| P11 | 4.5 | 4.6 | 2.9 | 149.9, 82.5 | not visited | - | - |
| P15 | 5.2 | 5.3 | 3.3 | 179.7, 150.5 | not visited | - | - |

![chart_leg_errors.png](chart_leg_errors.png)

Each dot is one leg of a P01-P20 tour: how far the robot's own wheels and gyro were off where the camera saw it stop, before the fit (A, the robot's old values) and after it (C tours, the new values); measured by the one school camera, from legs.csv.

![chart_four_points.png](chart_four_points.png)

How far the robot's tag is put away from the camera when its 6.19 cm height is ignored, at Jay-Sun's four points: his tape measurements (3 Nov) against the tag-height model for the two readings of the camera's height and this run's camera.

![chart_tracks.png](chart_tracks.png)

One P01-P20 tour before and one after the fit: where the camera saw the robot stop after each leg (blue) against where the robot's own wheels and gyro put it, added up from the tour's start with no camera correction (orange); table outline and the camera's x = 78 in limit (dashed); from legs.csv.

## Conclusion
- Fitted and read back on the robot: wheel_scale 0.9755, gyro_scale 1.0921; kept old: gyro_drift_dps (not pinned down: +-0.0410 (need +-0.0300)).
- (Claude's conclusion) Distance error per metre went from 1.80 to 0.21 cm (median), heading error per turn from 7.35 to 0.40 deg: the new values are better.
- (Claude's conclusion) The tag-height model with the camera 145 cm above the table (7.5 ft above the floor) gives 2.7/2.4/4.6/5.3 inches at P01/P05/P11/P15, within 0.3 in of Jay-Sun's 2.4/2.7/4.5/5.2; with 229 cm it gives 1.7/1.5/2.9/3.3, so the camera's 7.5 ft is above the floor.

## What's next
- Jay-Sun re-checks P01, P05, P11 and P15 with a tape (the four-point table above).
- The values are saved on the robot and come back at every power-on and wake. If the after-tours are NOT better: `unsave` (firmware 2026-11-05a) or `corr reset` puts it back to 1 / 0 / 1.
- Zones 3-4 need the second camera (EMEET C950 at the door end) before they can be calibrated or run.

## Sweep 1: new tests that may now be needed
- A repeat of this run on another day (do the values hold across a battery and a power cycle?).
- The same tours at 5 and 13 cm/s (does wheel_scale depend on speed?).
- Turn rate: the camera's measured turn speed against the 60 deg/s asked (turn_max_rpm was derived, not measured).
- The four points re-measured by tape, with the height correction on (Jay-Sun, tomorrow).
- LOST_HOLD_S (1 s) against the camera's real dropout lengths in camera_track.csv.

## Sweep 2: possible inaccuracies in what this relied on
- The camera's model comes from Measure with only tags 20, 23, 24, 25 (21 and 22 too small to read): its error grows towards zones 2's far side; every 'camera measured' number carries it.
- Tag height 6.19 cm is typed (Jay-Sun's tape), not measured by the camera (one camera cannot).
- The tag offset from the turning point is fitted from 13 resting reads between the in-place turns; a tilted roof would look like an offset.
- turn_max_rpm 26.5 is worked out from the nominal wheel size for 60 deg/s, not measured.
- Chosen numbers (no DOE yet): LOST_HOLD_S 1 s, MEASURE_S 1.2 s, the still periods 60 / 30 s, 12 turns, rolls 20/45/70 cm, the 'enough' +- targets, the 1 cm edge aim, the 10 cm camera-driven edge final.
- The fit assumes the drift is constant through the run; a gyro bias that wanders (the simulator's does) shows as a different drift in each still period.
- Distances per leg are from the robot's own odometry record (LEG start/done poses); a lost record means the leg is left out.
- SIMULATOR: the camera here is a pinhole model with chosen noise (0.08 cm + 0.12 % of range), not rendered pictures; the robot is tools/sim with its own physics.
