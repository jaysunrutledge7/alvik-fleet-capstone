# Audit C: findings and insights left behind (read-only, repo at 85f5e99, 6 Nov 07:27)

Scope: every build report in docs/build-reports (recognition-handoff-1, 20 Oct, to e07, 6 Nov), docs/results,
docs/session-notes, docs/experiments, docs/summary, presentation-notes, recognition-next-build, home-experiments,
doe-backlog, open-questions. Each report's next steps, sweeps, questions and "not built" sections were checked against
later reports and the current code (tools/fleet, firmware/home_lab).

Labels: **VERIFIED** means a grep or code read confirms the status (the evidence is quoted). **INFERRED** means it is my
reading of the reports, with no code proof either way.

"Current builds" means the fleet chain: Mission Control, mission_run, run_plan, calib_run, freeroam/freeroam_core,
observe, and the two bridges. tag_bridge.py is the school's roof-tag tracker. cam_bridge.py is home's lights and
outline tracker. The "old E02 runner" (tools/experiment/e02.py and its modules) is where most of the Oct 22-26 work
lives. The fleet chain does not import most of it.

---

## 0. Motion detection, in one place (Nick's headline concern)

| What was proven | Where | Status now | Evidence |
|---|---|---|---|
| Motion detector (floor memory) hands over to a lock-on tracker, and the wheels and gyro vouch for it in turns. Seen 94 / 93 / 96 % of pictures on the three evening losses, against 41 / 40 / 72 % for the detector alone. All within 7 cm of the camera fixes. Off-the-shelf trackers: CSRT drifted 24 cm without noticing it; MIL was too slow. This is headline 1 of the summary outline. | e02-oct22-build.md, "morning of 2026-10-23"; summary/outline.md headline 1; tests-to-run.md #8 | **LOST at school, PARTLY kept at home** | VERIFIED. tag_bridge.py has no motion, change or outline detector (it imports only tagcore and tag_geom). At home, cam_bridge uses e02_multitrack.MultiTracker (the floor-memory detector, with identity by prediction), but MultiTracker overrides `push` and never calls `_lock_step`. The lock-on (`USE_LOCK`, e02_lock) lives only in e02_camera.MotionTracker, which only the old e02 runner uses. `grep -n "e02_lock\|LockTracker" tools/fleet/*.py` returns nothing. |
| "Made hard on purpose" #4: no markers. The robot is found by its motion and followed by its own look. | summary/hard-on-purpose.md row 4 | **Reversed at school.** Roof AprilTags and corner tag tiles are the only tracker there. | VERIFIED (tag_bridge docstring). e07 "Not done": home still tracks by lights, and corner finding is not automatic. |
| Brick watch: the camera's "what changed" detector (hands, bricks, a pushed robot), with hands ignored | e02-oct27-school-engine; brickwatch.py | **Home run_plan only.** Not in free roam, tag_bridge or mission_run. | VERIFIED: `grep -i brick` finds nothing in freeroam.py, freeroam_core.py, tag_bridge.py or mission_run.py. e04 sweep, collision risk 5: "the camera bridge's brick watch is not on in the free roam". |
| Parked-robot watch: a parked robot that is pushed or moved is noticed, and the moment is saved as a clip | e02-oct25-tracker-process, conclusion 2 | **LOST** in both bridges | VERIFIED: `grep -ci parked` gives 0 for cam_bridge.py and for tag_bridge.py. |
| Motion fusion (tag pose plus motion in the picture, as CLAUDE.md "Observe, then trust" item 3 requires) | e06 "Left for phase 3 ... motion fusion"; e07 next-3 | **Promised, not started** ("Phase 3, not started, as asked") | VERIFIED (e07 text). |

Why it matters, with numbers. School tags read only about 150 cm of the 396 cm table from the TV-end camera (e07
Start screen), and stop reading at P11, the far corners and dark patches (e06 premise). A moving body is a far bigger
target than a tag's cells. A detector would let the robots stay seen in the unreadable area, instead of driving blind
there. INFERRED: in e07's half-table rehearsal, two robots came within 6.6 cm of each other while driving blind
(65 re-plans, 25 recoveries).

A known limit to design around: motion alone never sees a robot that has not moved yet. On roam1, 19 % of still
robots were found against 93 % of moving ones (session-notes/2026-10-21 §2). The tag, or the lights, has to seed it.
This is exactly the handover design of 22-23 Oct.

---

## 1. LOST INSIGHTS (measured, not used by the current code), most important first

1. **The firmware's heading comes from the library yaw times gyro_scale. Integrating the raw gyro rate, with its bias
   taken whenever the wheels stand still, measured far better.**
   - Source: e02-oct25-real-fusion, conclusion 2 and recommendation R1.
   - Measured: on uncalibrated (U) routes the robot's own pose ended 15.9 cm off. The same data integrated the
     fusion's way ended 2.0 cm off. On K routes: 18.0 against 4.2 cm.
   - Status: **LOST (VERIFIED).** motion.h:200-214 `yawTrack()` still accumulates `alvik.get_orientation` yaw times
     `P("gyro_scale")`. There is no gz or standstill-bias path.
   - Why it matters: every blind stretch (free roam, school recoveries, no-camera runs) starts from this heading. It
     also removes the dependence on the gyro-scale fit.

2. **The fused estimate (wheels, gyro and camera, EKF in tools/fusion) beat every single source on real data.**
   - Source: e02-oct25-real-fusion, gap table.
   - Measured: error after a camera gap of 2 / 5 / 10 / 20 / 40 s was 1.4 / 2.2 / 3.4 / 3.8 / 5.1 cm fused. Wheels
     and gyro alone: 1.5 / 2.7 / 4.2 / 4.6 / 7.3. The robot's own pose: 2.5 / 3.9 / 4.6 / 7.5 / 19.2. Routes driven
     blind after 10 s: 2.3 cm fused against 8.6 cm own pose. Near boards, the fused estimate was closer to the
     board-touch truth (3.6 cm) than the camera itself (5.1 cm).
   - Status: **LOST (VERIFIED).** No file in tools/fleet imports tools/fusion. e02_fusesteer ("built and OFF",
     tracker-process report) was never run on the board. e06 calls it "one fused pose ... still several estimators
     side by side". e07 lists it for phase 3.
   - Why it matters: this is the project's stated objective (README "Goal"; experiments/INDEX T3 "fusion gain"). The
     current chain has a weighted vote (observe.move_outcome) and side-by-side checks, not fusion.

3. **The camera spot rides 5-9 cm outward at open edges.** Within 15 cm of a board it jumps 5-8 cm.
   - Source: e02-oct27-session-edges, conclusion 4; e02-oct25-real-fusion, "board-side camera"; R3 (widen the camera
     noise near edges and boards); the e04 11-05c table ("the camera reads 5-9 cm outward at an open edge"); the 4 h
     analysis (Left "3 cm past x = 122" while on the board).
   - Status: **LOST (VERIFIED).** cam_bridge.py:56 has `FIX_SIGMA_CM = 1.0` flat. Its comment says the spot "rides
     5-9 cm onto the nose AT an open edge; Phase 0's routes stay 25 cm in". Free roam now plans end spots 16 cm in
     (corners 9.3 cm) and trusts fixes there at 1 cm sigma. No edge-dependent sigma or bias correction appears in
     freeroam_core or observe.
   - Why it matters: this ties directly to Nov 6. Near the edge, the camera and the odometry disagree by the bias
     itself, which triggers 2 s of disagreement handling (see Repeated fault 1).

4. **Learned per-robot uncertainty, and the turning term.**
   - Measured: the logs DOE refit the turning term to 0.014 per degree, against 0.005 chosen. After 50 cm and 180° the
     sd is 3.4-3.7 cm, not 2.2 cm. The online learner e02_unc.py adapts per robot after 30 corrections. In the
     simulator, 85-88 % of corrections fell inside 2.45 sd.
   - Source: e02-oct25-logs-doe §1; e02-oct25-backing-unc D.
   - Status: **LOST in the fleet (VERIFIED).** No tools/fleet file references e02_unc. freeroam_core:93-95 uses
     BLIND_BASE 2.0 and BLIND_PER_CM 0.07 (measured 29 Oct, straights only, up to 40 cm), plus BLIND_TURN_FRAC 0.03
     (chosen). keep-going says the 0.07 cm/cm is "measured only to 40 cm ... turns not in it". The no-camera run
     extrapolated it to 740 cm (sticker-points report).
   - Why it matters: the invariant circle and the bubble are only as honest as this model, and blind driving near
     edges depends on it.

5. **Sensor trust: each sensor judged against its own history first.**
   - What it is: e02_trust.py gives a per-sensor k and flags. It follows Nick's 25 Oct rule: "don't compare one
     robot's wheels to another's until each is calibrated independently". The healthy-session false flags fell to
     zero at MIN_N 5.
   - Source: e02-oct25-trust-fix; logs-doe §9-10.
   - Status: **LOST in the fleet (VERIFIED).** e02_trust is not imported by tools/fleet. observe.move_outcome uses
     fixed, chosen weights (wheels 1, accelerometer 1.5, gyro 1.5, front 2, line 0.5, camera 3; e06 sweep 2 lists them
     as chosen). The recommended MIN_N 3 → 5 was never applied even in e02_trust.py (still
     `wheels=3, gyro=3`, line 81).
   - Why it matters: CLAUDE.md says "a disagreement between sources is information" and "fuse every available
     signal". The weights never learn which sensor is degraded (the "degraded sensors" experiment Nick wanted on
     24 Oct).

6. **Camera drift watch (ECC alignment to the session's first picture).**
   - Measured: the overhead image crept up to 2 px (about 1 cm at the top board) over a day (oct24 analyses §1).
   - Status: **LOST at home (VERIFIED).** CameraShiftWatch exists only in e02.py (lines 10031-10182). cam_bridge has
     no shift or bump check. School has the ref_tags bump check (tag_bridge), so it is **used there**.
   - Why it matters: the 5 Nov 4 h run found a uniform 4 % wheel-against-camera scale gap and could not tell whether
     the camera (calibrated 30 Oct) or the wheels caused it.

7. **Tracker-jump cut at 3 cm before fitting.** In stage 2, 66 % of the stretches held a jump. Cutting them brought
   every wheel fit under 1.5 cm.
   - Source: oct24 night analyses B6; logs-doe §3.
   - Status: **USED** where e02_fit is imported (engine_checks, ladder, run_plan: `JUMP_CM = 3.0`, e02_fit.py:112).
     Not checked whether calib_run's own fit (school) cuts tag-read jumps; INFERRED not needed (tag reads, not
     outlines).

8. **Gyro drift band 0 (drift held at the measured standing value).** This was 0.12-0.15 cm better at p < 0.001.
   - Status: **USED** (e02_fit.py:43 `DRIFT_BAND_DPS = 0.0`). calib_run fits drift itself (school), and e04 kept an
     unpinned drift at the saved value.

9. **The fixed stop overrun of 0.3 cm at 5 cm/s and 0.5 cm at 10 cm/s, removed by a "stop lead"** (mean along-error
   +0.43 → 0.00 cm on the real school legs).
   - Source: e06 §1.
   - Status: **PARTLY.** It is used in calib_run (e06) and in missions/run_plan (e07). **Not in free roam (VERIFIED:**
     0 hits for stop_lead in freeroam_core.py and freeroam.py). It was measured on one robot (AL-1DC0), one run.

10. **The front sensor reads 10-15 cm short beyond about 30 cm on every robot** (45 cm board: 30 / 24 / 37 cm).
    - Source: e02-oct25-right-front-and-rpm §1; robot-differences F2.
    - Status: **PARTLY.** The old runner limited front checks to 50 cm and 45°. observe.move_outcome's front-range
      vote (weight 2) has no range cap (VERIFIED, observe.py:707-708: it only needs `front_expect_cm >= 2`).
      INFERRED: long-range votes are biased.

11. **Robots drive about 7.5 % slower than commanded; wl_rpm reads 8 % high** (motor-board timing). v_cms is mm/s.
    - Status: **USED** for units (tools/raw_units.py). The per-robot rpm-against-angle check was **never built**.
      INFERRED: it affects free roam's move-time estimates (moves took 0.87 of the estimate in e04).

12. **Center range zone = mean of CL and CR** (derived, R² 1.000), so it should not be counted as an independent
    sixth zone.
    - Source: discovery replay, conclusion 2.
    - Status: unchecked in the fleet code (INFERRED: no fusion counts zones today, so it is moot until fusion exists).

13. **Battery: 11-16 %/h standing, 17-19 %/h in free roam whether driving or not. The reading is smoothed (driving =
    standing).**
    - Source: night analyses C; 4 h analysis §5.
    - Status: **USED** for planning (free roam ends at 4 h). The **sleep-saving test** (night analyses, next) and the
      "battery-rate learner" were never measured or ported (INFERRED).

14. **The camera picture takes 0.42-0.47 s from capture to fix at home, with a spike at each 5-minute video file
    rollover** (45 of 45 spike windows).
    - Source: e04 11-05c; 4 h analysis §4.
    - Status: the delay is **USED** (measured online in freeroam). The rollover fix ("smooth the video rollover",
      4 h next 4) is **NOT done** (VERIFIED: camrec has no change; only the bridge is `nice -n 5`).

15. **The overhead loses robots on a sunlit strip (x = 30-45 at home). Lighting labels per camera. Lock-on hold
    correlates with the robot's visible detail (r = 0.73).** The "camera-quality map" plan raises camera uncertainty
    in poor cells.
    - Source: oct24 analyses §2; daytime-sessions; rules audit rec 5.
    - Status: **PARTLY.** The school has observe.ReadMap (where tags read). Home's cam_bridge has no per-cell quality
      map (INFERRED from grep: ReadMap is only in calib_run, mission_run, observe and run_plan).

---

## 2. UNDONE FOLLOW-UPS (promised and never done), most important first

1. **An independent position reference (tape marks across the board, or a tape-measured drive).**
   - Requested in: logs-doe §4 ("the fixed prior predicts as well as the session's own fit: we need a reference that
     isn't the camera"), robot-differences C3/W7, open-questions 34 and 80, corners-only-setup (cameras saw crawls
     7.8 % longer than the wheels), the 4 h analysis next 3 (a tape 100 cm drive for the 4 % gap), and e07 sweep 2
     (the measure bench's tape is "exact truth (a real tape is not)").
   - Status: **NEVER DONE (VERIFIED: no report holds a taped result).** The wheel-against-camera gap has been 8 %
     (28-29 Oct), 4 % (5 Nov home) and 2.4 % (school wheel 0.976). Every accuracy claim, including "self-calibration
     1.4 against 16.4 cm", is measured against the same camera that calibrates the robots.

2. **The tilt and edge bench DOE** (a wheel over a 1 / 2 / 3 cm drop, standing and driving; sliding sideways off an
   edge; braking pitch).
   - Asked for in: oct25-2114 sweep 1; oct26 runner-fixes next 3; oct26 0955 fix 6 ("include a robot sliding
     sideways: Center 3.3° pitch, 1.7° roll"); the 4 h analysis next 3; e06 sweep 1 and Q3; doe-backlog lines 248, 258
     and 277, all open.
   - Status: **NEVER DONE.** Nick declined it on 5 Nov at 21:38. STAND_TILT 2.5° / 2 s and ACCEL_JOLT 0.03 g remain
     chosen.
   - Why it matters: the Nov 6 tipping was caught by the tilt stop. The thresholds still rest on no bench data.

3. **The DOE backlog is about 5 % executed.** The backlog has 362 table rows: 203 open, 19 done or partly done. All 19
   done rows are logs or simulator rows from 25-28 Oct. **No board or bench row has ever been run.**
   - VERIFIED by tallying the status column of doe-backlog.md.
   - The backlog stopped growing at e04 (5 Nov). e05, e06 and e07 put "DOE notes in the code", but those numbers
     (LOOP_R, K_BINS, V_MIN_AVOID, POSE_AGREE, EDGE_PROFILE_CELLS, site_detect weights and others) have no rows (the
     last section is "Home free roam (2026-11-05)").

4. **The experiment index (T0-T9) was never updated.** experiments/INDEX.md was last edited 18 Oct. Every row says
   "planned", including T3 fusion gain, T4 failures, T5 delay budget, T8 speed against accuracy and T9 the comparison
   with the lab's AprilTag system.
   - Status: the real results exist (E02: C 1.4 cm against U 16.4 cm on 23 Oct; real-fusion gaps 25 Oct) but are not
     recorded against the project's own test plan (VERIFIED: git log date).

5. **The latency test** (inject delay; camera-steered K against our C/KC). This was the summary's "What's next" and
   tests-to-run #1.
   - Status: **NEVER RUN** on real data. The free-roam fastsim has delay injection (simulator only, VERIFIED by
     grep). It is the core argument of summary headline 4.

6. **The degraded-sensor and lights-off runs** ("driving without the lights", `set blink_codes 0`; "the system learns
   how much to trust each sensor").
   - Source: next-week-notes, 24 Oct.
   - Status: simulator grid only (24-25 Oct). **No board run.** The trust system they would test is not in the fleet.

7. **Home free-roam follow-ups.**
   - e04 next 3: the camera bridge should re-find a robot whose name it gave away, and speed up (it skipped 16,938 of
     about 27,600 pictures as "behind").
   - Sweep: a camera that swaps two robots' names while both drive (not modelled).
   - Status: INFERRED **not done** (no later report mentions it; cam_bridge only logs `skipped_behind`).

8. **The calibration "look" reads the floor as a wall.** Seen on 19 Oct (room 8.3-10.9 cm in all 8 directions, so
   the roll never runs) and again on 30 Oct ("most likely the sensor read the floor all round").
   - Fix proposed twice: in firmware, ignore readings at the floor's distance.
   - Status: **NOT DONE (VERIFIED: look.h has no floor handling; params.h only has `tof_min_cm 2`).** Open
     question 1 of robots-anywhere was never answered.

9. **Firmware heading R1 (raw gyro rate with standstill bias)** and the compass-check fix ("take the compass
   direction from the gyro rate or the wheels", oct25-1545 next 3).
   - Status: **NOT DONE (VERIFIED: motion.h).**

10. **Open questions 8-87.** open-questions.md was last edited 30 Oct, and later questions were never added.
    - Unanswered items with measurement value: 10 (Pi cooling; the heat tracker still says "unknown"), 12 (speed
      sweep), 34 (taped marks), 66 (Center's lights 24° off its crawl), 79 (light spacing by ruler: the home setup's
      scale ruler), 80 (tape `go 10`), 87 (Right's long turns 20-27° short).
    - The oct30 daytime finding "Right drives 10-27° off course in about 1 drive in 6" was never followed up
      (INFERRED).

11. **School-specific promises still open** (e07 sweep 1):
    - a real camera with printed tags of known size (EDGE_PROFILE_CELLS);
    - Measure with nothing typed;
    - site detection on both Pis;
    - the red STOP reaching every robot within 1 s;
    - sleep-when-done.
    - These are tests of the 6 Nov build and are fairly new (INFERRED pending, not lost).

12. **Lights mode at school** (motion-based tracking without tags) was "not tested end to end" on real cameras
    (sticker-points self-review). It is still untested (INFERRED).

---

## 3. DROPPED TECHNIQUES (worked once, absent later), most important first

1. **The lock-on tracker plus motion detector, the brick watch and the parked watch** (see section 0). This is the
   biggest drop. It worked on real footage, it is the project's "no markers" claim, and it is absent from the school
   chain (VERIFIED).

2. **Explore: the robots calibrate the camera themselves** (hard-on-purpose #10, "the robot itself is the only thing
   needed to calibrate the camera").
   - What happened: it was fixed to pass 18 of 18 in the simulator (oct26 explore reliability) but never succeeded on
     the real board (15:10 and 09:55 failed). It was replaced by corner clicks (29 Oct) and corner tag tiles (3 Nov).
   - Status: **DROPPED (VERIFIED:** no tools/fleet file references e02_explore or survey). e07 "Not done": home corners
     are still clicked.
   - Why it matters: the "it calibrates itself" emphasis that Nick wanted carried through everything (next-week-notes,
     24 Oct).

3. **Reverse find and the back-switch attachment** (firmware 25b, simulated 20/20 finds within 0.22 cm; the dock
   enters backwards).
   - Status: INFERRED **dropped**. No later report uses `--switches-back`. Free roam sends `bumper_cm 0`. Board finds
     (touching boards to reset the pose to under 0.5 cm) are not part of the fleet chain at all.
   - Why it matters: it was the only reference that is independent of the camera (the find corner). The dock
     self-calibration idea (19 Oct) depends on it.

4. **Unmeasured headings are never written into a robot; a heading push happens when the measured heading is more
   than 8° off.**
   - Proven: the heading diagnosis traced 100 of 112 errors of 30° or more to unmeasured headings. Fixes 1-4 were
     built in e02 and tested.
   - Status: **e02 only (VERIFIED:** no heading_push in tools/fleet). The fleet chain has its own HeadingWatch (e06),
     but restarts still left headings wrong: 105° off in an e06 rehearsal, and 20° off with every fix refused in e07
     phase-3 item (c). (See Repeated fault 3.)

5. **The discovery stack** (tools/discovery: blind sensor naming 93 %, 1 wrong in 1,374 accepted; the calibration
   battery spec with a square drive, a one-wheel bump and an open-edge approach).
   - Status: **DROPPED.** tools/fleet never imports it. The printed bump part was never made.
   - Why it matters: this is "Discover, don't pre-map" in its purest form (CLAUDE.md, 5 Nov). A new robot's sensors
     could be identified and calibrated by itself.

6. **The recovery ladder of 22 Oct** (another robot sweeps for a lost one, it finds itself against the boards, it
   drives out blind, and a person is the last step).
   - It worked in the simulator and on the real board ("in the afternoon two robots swept for a lost one and found
     it", summary outline headline 3).
   - Status: INFERRED **dropped**. The fleet has recover_blind (drive straight to the last read spot) and a reserved
     corridor. There is no robot-assisted search and no board find.

7. **The Live lab's per-robot filter with the camera start rule** (restarts cut from 1,233 to 14 an hour, 95th
   percentile 2.7 cm).
   - Status: built in e02_live (VERIFIED by the live-page report). It is not part of Mission Control's fleet path
     (INFERRED).

8. **Fusion steering (`--fusion-steer`)**. Built off and never run on the board (tracker-process report). Absent in
   the fleet (VERIFIED).

9. **Learned front-sensor offset** (e02_front: each robot's offset learned from its history and replaced by the
   session's own).
   - Status: **PARTLY.** fleetmap, ladder and warmup reference FRONT_OFFSET. The floor check (`follow_look_cm` per
     robot) is used (VERIFIED in run_plan and front_stop).

10. **Grain (SIFT) calibration of a second camera** (side camera at 0.42 cm leave-one-out, no markers).
    - Status: superseded by corner clicks and tags (INFERRED). It is still useful for a "new area" with no tiles.

---

## 4. REPEATED FAULTS (the same failure in several reports), most important first

1. **The robot is planned or driven from a stale or wrong pose while the camera and odometry disagree. This is
   Nov 6's fault.**

   Earlier occurrences:

   | Date | Report | What happened |
   |---|---|---|
   | 24 Oct | oct24 analyses §3 | A camera heading fix 2 s before a find made 13-24 cm find errors |
   | 25 Oct | heading report | Fixes wrote unmeasured headings: 17 % of stretches ended 30° or more wrong, with corrections of 22 cm median |
   | 25 Oct | oct25-1545 | A stale "held" tracker spot was sent as a fix (15 cm wrong). Left ended up at the edge |
   | 26 Oct | 0955 | The session ended on a 20-minute-old position. A blind rescue drove into the edge |
   | 5 Nov 14:35 | e04 | Each fix was compared with the pose "now", not at the picture's time. Jumps of up to 45 cm |
   | 5 Nov, 4 h run | 4 h analysis | Center was planned from the camera and executed from its own pose, which was 17 cm off. It drove into the x = 0 edge and tilted 3.4° for 64 min |
   | 5 Nov | e06 rehearsals | A rejoin on a 36 cm stale spot came to 5.3 cm of another robot. A restarted robot was 105° off and refused every fix |
   | 6 Nov | e07 (c) | 20° off after a restart, with every fix refused |

   - Common cause: when the two sources disagree, one is believed for a fixed time instead of the doubt being added at
     once.
   - VERIFIED in the code: observe.py:579-580 has `POSE_AGREE_CM = 5.0` and `POSE_AGREE_S = 2.0`. freeroam_core
     `pose_agree` (1183-1200) re-seeds only after 2 s. `unc()` (893-899) inflates only for camera age and for the
     bubble when lost, not for a live own-against-camera gap. A curve piece is a 2-8 s commitment (ARC_LEN_BINS
     comment).
   - The lesson in the reports (real-fusion, CLAUDE.md item 3): a disagreement is information. It should widen the
     possible-locations circle immediately, so the invariant refuses moves toward an edge until the sources agree.

2. **Robots end at, over or hanging on open edges.**

   | Date | What happened |
   |---|---|
   | 22 Oct | Center fell after learning the floor with its sensors over the edge |
   | 23 Oct | Right and Center went off at a slant; the bottom zone did not trip |
   | 25 Oct 15:10 | The centre stopped 3.8-5.5 cm past the stored line at every line-sensor stop |
   | 25 Oct 15:45 | Left drove blind forward at the edge |
   | 25 Oct 21:14 | Left turned on the spot at the edge and hung at the corner |
   | 26 Oct 09:55 | Center hung over the right edge at a 3.3°/1.7° tilt, under the 5° limit |
   | 27 Oct | Center was hanging at the start of E2 |
   | 30 Oct | Crawls walked a robot to the plywood edge |
   | 5 Nov | Center sat at 3.4° for 64 min; Right's nose was over y = 0 for 16 min; Left was past x = 122 |
   | 6 Nov | Tipping, caught by the tilt stop |

   - Recurring mechanisms:
     - the edge guard only looks forward ("backing up is blind");
     - a shallow or sideways approach puts a side wheel over before the nose;
     - the bottom zone reads 10.0-10.3 cm with the nose over the drop, under the 11 cm limit (21:14);
     - tape strips read 135-140 against a trip level of 150 (27 Oct);
     - the camera's outward bias at edges (Lost insight 3).
   - Curves make it worse: in e05, bodies came within 3 cm of an edge against 11 cm for stop-turn, and the stress
     simulator "does not model the robots' own 6 cm border" (e05 sweep 2).
   - The bench DOE that would set the limits was never run (Undone 2).

3. **Heading lost after restarts or sleep.**
   - 27 Oct: robots restarted from sleep came back uncalibrated, and the ladder did not notice.
   - 28 Oct: a restarted robot failed to start (Q56).
   - 4 Nov: a robot woken from sleep refused to turn.
   - e06: 105° off after a restart.
   - e07: 20° off, every fix refused, the point reported 5 cm off. In-run recalibration rolled a body to 4.9 cm from
     an edge.
   - The restart re-check loops on "camera's position is N s old" for a turned tag.
   - Status: partly fixed each time (boot id watch, compass re-check). It is still listed open in e07 phase 3
     (VERIFIED by the text).

4. **"Out of re-plans" and "no plan" endings.**
   - 29 Oct: 5 "no plan" endings came from phantom stops, fixed by unconfirmed obstacles.
   - 3 Nov: keep going ground on for 10 min with both robots held.
   - e06: the home Monday program ran "out of re-plans" after a seam-tape crossing, and the same happened with
     surface marks off.
   - e07: it is still listed as a phase-3 fault.
   - Status: **OPEN (VERIFIED by the e07 text).**

5. **Front-sensor phantom stops from the floor reading** (Left's centre-right zone at about 11 cm, standing after
   turns; 180 false stops in 236 drives).
   - Status: fixed by 3 readings and a per-robot floor check (VERIFIED in front_stop.py and run_plan). The same floor
     reading also breaks the calibration look (Undone 8), which is still open.

6. **Done without moving, and loops** (Left: 4,289 instant reverses; wheels counting while the body did not move in
   explore, oct26; creep steps stalled in 12 of 23).
   - Status: addressed by move_outcome and FailMemory (e06). Untested on real slip ("no real slip has been recorded",
     e06 sweep 2).

7. **WiFi silence and lost commands** (21:14: 8-11 % loss; 2 Nov: all three robots silent within 24 s; 5 Nov
   watchdog stops).
   - Numbered and acknowledged commands exist (firmware 26a). The Pi network log ("ping the router and each robot
     into perf.csv") was promised twice (2 Nov, low-battery DOE and fixes) and **never built** (INFERRED: no later
     mention).

8. **Name and identity swaps between close robots** (22 Oct: Right taken for Left; 27 Oct: Left swapped with a parked
   Right; 30 Oct: lights matched to the wrong robot twice; 5 Nov: the camera put Center on Right's spot).
   - Fixed piecewise each time. The fast simulator still "never names the wrong robot" (e04 sweep), so free roam's
     stress results cannot show this fault.

---

## Notes for the main session

- Nothing here came from memory. All of it comes from the repo.
- The Nov 6 run is not in the repo (the last commit is 6 Nov 07:27). The link above to POSE_AGREE_S = 2 s and the
  missing edge-bias sigma is my code-level inference from the description given in the task.
