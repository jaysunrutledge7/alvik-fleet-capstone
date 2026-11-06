# Calibration run, zones 1-2: 20261105-193455-calib

Written automatically by tools/fleet/calib_run.py 2026-11-05g at the run's end. Robot: **AL-1DC0** (WiFi name AL-1DC0), roof tag 2. Result: **finished**.

## Premise
Nick (4 Nov): get enough data without calibration as a base level, use the camera only for safety (edges, keep-outs, the camera's view, parking), then fit the robot's corrections and run a series of tours with them. Every leg is driven on the robot's wheels and gyro alone; the one school camera measures where it actually stopped, and the gap is that leg's odometry error. Speed as in the experiments: 10 cm/s, 60 deg/s turns, last 5 cm at 5 cm/s.

## Data
**Binding** (wiggle (experiments.py sync wiggle, 90 deg and back; pinned robots a 45 deg check)): AL-1DC0 (WiFi AL-1DC0, 192.168.1.134) = tag 2 [pin checked: its pinned tag 2 turned with it].

**The robot's values before** (read with `get`, saved in robot_values_before.json): wheel_scale 1.0000, gyro_scale 1.0000, gyro_drift_dps 0.0000.

**Phase A started from the defaults (wheel_scale 1, gyro_scale 1, gyro_drift_dps 0)**: the first calibration of AL-1DC0 on this site (not in calibrated_robots.json): an uncalibrated baseline.

**Fit from phase A** (41 turns, 1 still periods, 21 straight legs; 95 % +- = 1.96 x SE):

| value | before | fitted | +- (95 %) | enough = +- under | verdict | sent | read back |
|---|---|---|---|---|---|---|---|
| wheel_scale | 1.0000 | 0.9759 | 0.0053 | 0.008 | ok | 0.9759 | 0.976 OK |
| gyro_scale | 1.0000 | 1.0863 | 0.0015 | 0.005 | ok | 1.0863 | 1.086 OK |
| gyro_drift_dps | 0.0000 | 0.0025 | 0.0163 | 0.030 | ok | 0.0025 | 0.002 OK |

**Tag offset from the turning point** (from the in-place turns): 0.1 cm ahead, -0.2 cm left (0.2 cm; circle fit +- 0.1 cm rms). Taken off every camera position in this run.

**Per-leg odometry error, before (A: the old values) vs after (C: the fitted values)**, tour legs only (the last 10 cm at an edge point is driven from the camera and left out):

| tour | legs (go / turn) | distance error, cm per m (median / p95) | signed mean, cm per m | sideways, cm per m (median) | heading error per turn, deg (median / p95) |
|---|---|---|---|---|---|
| A (before) | 15 / 20 | 1.37 / 2.17 | -1.12 | 1.35 | 7.44 / 14.69 |
| C1 (after) | 14 / 14 | 0.87 / 1.65 | +0.95 | 2.31 | 0.46 / 1.91 |
| C2 (after) | 9 / 7 | 0.95 / 1.66 | +1.03 | 1.06 | 1.35 / 4.99 |
| C3 (after) | 14 / 14 | 1.53 / 2.88 | +1.55 | 2.14 | 0.68 / 3.74 |

**Gyro drift standing still** (the robot's own heading change while the camera saw no turn): A0 +0.0032 deg/s over 60 s (camera -0.02 deg); C1 +0.0013 deg/s over 30 s (camera +0.07 deg); C2 -0.0017 deg/s over 30 s (camera +0.21 deg); C3 -0.0027 deg/s over 30 s (camera -0.08 deg).

**Safety**: closest the robot's middle came to an edge 13.3 cm (margin 13 cm), to a keep-out or tile 11.2 cm (wanted: the body's half diagonal + 2 cm = 8.7 cm, less 0.5 cm of measuring), furthest x 118.9 cm (the camera reads to 198.1 cm); the camera corrected the pose before 78 legs; holds for a lost tag: 0; points skipped: P11 (the edge of the camera's view (x 112 cm)), P12 (the edge of the camera's view (x 114 cm)), P13 (the edge of the camera's view (x 106 cm)), P14 (the edge of the camera's view (x 108 cm)), P15 (the edge of the camera's view (x 112 cm)), P16 (the edge of the camera's view (x 108 cm)), P17 (the edge of the camera's view (x 110 cm)), P18 (the edge of the camera's view (x 110 cm)), P19 (the edge of the camera's view (x 109 cm)), P20 (the edge of the camera's view (x 104 cm)), P11 (the edge of the camera's view (x 111 cm)), P12 (the edge of the camera's view (x 113 cm)), P13 (the edge of the camera's view (x 111 cm)), P14 (the edge of the camera's view (x 112 cm)), P15 (the edge of the camera's view (x 111 cm)), P16 (the edge of the camera's view (x 110 cm)), P17 (the edge of the camera's view (x 112 cm)), P18 (the edge of the camera's view (x 110 cm)), P19 (the edge of the camera's view (x 117 cm)), P20 (the edge of the camera's view (x 113 cm)), P20 (the edge of the camera's view (x 122 cm)), P19 (the edge of the camera's view (x 115 cm)), P18 (the edge of the camera's view (x 115 cm)), P17 (the edge of the camera's view (x 112 cm)), P15 (the edge of the camera's view (x 124 cm)), P14 (the edge of the camera's view (x 119 cm)), P13 (the edge of the camera's view (x 121 cm)), P12 (the edge of the camera's view (x 125 cm)), P11 (the edge of the camera's view (x 122 cm)), P10 (the edge of the camera's view (x 115 cm)), P09 (the edge of the camera's view (x 111 cm)), P08 (the edge of the camera's view (x 111 cm)), P07 (the edge of the camera's view (x 115 cm)), P06 (the edge of the camera's view (x 112 cm)), P18 (zone 1 outlet plate (7.1 cm)), P14 (zone 1 outlet plate (5.0 cm)), P13 (the edge of the camera's view (x 113 cm)), P15 (the edge of the camera's view (x 125 cm)), P19 (the edge of the camera's view (x 116 cm)), P17 (the edge of the camera's view (x 112 cm)), P20 (the edge of the camera's view (x 111 cm)), P16 (zone 1 outlet plate (0.0 cm)), P12 (the edge of the camera's view (x 119 cm)), P11 (the edge of the camera's view (x 114 cm)).

**Saved on the robot** (Nick, 5 Nov: kept over power-off): this robot's firmware has no `save` (it needs 2026-11-05a: reflash it to get it); NOT saved (calibration.old_firmware_save off): the values last until it sleeps or is switched off (NOT confirmed).

**End**: parked at P09, 0.3 cm from the sticker; `sleep 10` answered. Battery 87 % at the start, 81 % at the end.

**Four-point check** (Jay-Sun re-checks these tomorrow): how far the camera puts the robot with and without the tag-height correction (tag 6.19 cm up), inches, away from the camera:

| point | Jay-Sun (no correction) | model, camera 145 cm above the table | model, 229 cm | sticker (cm) | this run, phase C stops: with correction (cm) | the same read without (cm) | difference (in) |
|---|---|---|---|---|---|---|---|
| P01 | 2.4 | 2.7 | 1.7 | 30.5, 31.8 | not visited | - | - |
| P05 | 2.7 | 2.4 | 1.5 | 15.9, 126.4 | not visited | - | - |
| P11 | 4.5 | 4.6 | 2.9 | 149.9, 82.5 | not visited | - | - |
| P15 | 5.2 | 5.3 | 3.3 | 179.7, 150.5 | not visited | - | - |

![chart_leg_errors.png](chart_leg_errors.png)

Each dot is one leg of a P01-P20 tour: how far the robot's own wheels and gyro were off where the camera saw it stop, before the fit (A, the robot's old values) and after it (C tours, the new values); measured by the one school camera, from legs.csv.

![chart_four_points.png](chart_four_points.png)

How far the robot's tag is put away from the camera when its 6.19 cm height is ignored, at Jay-Sun's four points: his tape measurements (3 Nov) against the tag-height model for the two readings of the camera's height.

![chart_tracks.png](chart_tracks.png)

One P01-P20 tour before and one after the fit: where the camera saw the robot stop after each leg (blue) against where the robot's own wheels and gyro put it, added up from the tour's start with no camera correction (orange); table outline and the camera's x = 78 in limit (dashed); from legs.csv.

## Conclusion
- Fitted and read back on the robot: wheel_scale 0.9759, gyro_scale 1.0863, gyro_drift_dps 0.0025.
- (Claude's conclusion) Distance error per metre went from 1.37 to 1.02 cm (median), heading error per turn from 7.44 to 0.71 deg: the new values are better.
- (Claude's conclusion) The tag-height model with the camera 145 cm above the table (7.5 ft above the floor) gives 2.7/2.4/4.6/5.3 inches at P01/P05/P11/P15, within 0.3 in of Jay-Sun's 2.4/2.7/4.5/5.2; with 229 cm it gives 1.7/1.5/2.9/3.3, so the camera's 7.5 ft is above the floor.

## What's next
- Jay-Sun re-checks P01, P05, P11 and P15 with a tape (the four-point table above).
- The values are NOT kept over power-off (this robot's firmware has no `save` (it needs 2026-11-05a: reflash it to get it); NOT saved (calibration.old_firmware_save off): the values last until it sleeps or is switched off): reflash the robot with firmware 2026-11-05a and run this again, or keep them for today only.
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
