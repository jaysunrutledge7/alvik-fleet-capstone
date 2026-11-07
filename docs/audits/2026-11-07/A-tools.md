# A. Tools and features inventory: what each pipeline really uses, and what got left behind

Repo: clone at `peek/`, HEAD `85f5e99` (6 Nov 2026), 714 commits. Read-only audit; nothing was run against robots or Pis.

**How this was checked.** Import closures were built with an AST script over `tools/` (every `import`, including the
lazy ones inside functions) starting from each entry point, plus every `subprocess` call and every `--flag` that
`run-fleet-pi.sh`, `mission_run.py`, `freeroam.py` and `start_job.py` actually pass. **V** = verified in code (grep or
read, with file:line). **I** = inferred (from docstrings, reports or commit messages, not traced at runtime).

## 1. The two current pipelines

**Home free roam** (`run-fleet-pi.sh freeroam` -> tmux -> `tools/fleet/freeroam.py run`) (V):
- start: `ladder.py` L0 (robots found by their lights flashing: `ladder.Eye.find_lights`, a two-camera tracker
  `e02_twocam`), L1 (firmware `calibrate`, look.h, plus the `e02_fit.PRIOR` corrections, or `~/.config/mirlt/corrections.json`), a heading crawl.
- tracking: subprocess `cam_bridge.py` -> `e02_multitrack.MultiTracker` (built on `e02_camera.MotionTracker`: frame
  differencing plus floor memory, identity by prediction), `e02_twocam` (side camera when calibrated), lights as markers
  (on unless `--no-lights`), travel heading `e02_steer`.
- control: `freeroam_core.Controller` (menu, reservations, reflex, curves, dead reckoning, battery park). It does **not** use
  `run_plan.py` or `planner.py`.
- support: `observe.py` (only `check_safety`, `restarted`, `safety_*`, `site_safe_edge`), `diskguard`, `doe_loop.Frames`
  (frame saving), `calib_run.start_cpu_log` (cpu.csv for the heat tracker), `freeroam_report`, recorder `tools/pi/camrec.py`.
- board: **typed**, `HOME_BOARD = (0, 0, 122.24, 121.92)` (freeroam.py:52); floor calibration = the static roam1 file
  from the corner-click setup page (`setup_conf.py calib`).

**School Mission Control** (`run-fleet-pi.sh mc` -> `site_detect.py` -> `mission_control.py`; Start screen `start_job.py`) (V):
- Calibrate: `calib_run.py` (tag site) / ladder L0+L1 (home). Free roam at a tag site: `mission_run --roam-min`
  (planner + run_plan, curves); at home: it starts `freeroam.py`. Visit points and experiments: `mission_run.py`.
- tracking: `mission_run.start_bridge` -> `tag_bridge.py` when tracking is `tags` (school site `tracking`/`run_tracking` =
  `tags`), else `cam_bridge.py` (lights), or none (odometry only). `tag_bridge` closure: tagcore, tag_geom, ref_tags,
  roof_cal, site_fit, sites, e02_camera (camera model only), sources. **No MotionTracker, no lights, no brick watch.**
- driving: `run_plan.py` (planner, engine_checks, front_stop, straighten, facing, observe: HeadingWatch, TiltWatch,
  ReadMap, FailMemory, Trail, edge_trip_kind / surface marks).
- setup: ref_tags (corner tiles), site_fit with `auto_refit background`, roof_cal, wiggle_bind, feel_table / feel_job,
  sticker_points, camera_checkout, camera_focus; side camera `use: false` (sites/school-conference.json).

## 2. Capability table

Columns: Home = home free roam; School = Mission Control at the school (tag site). "Off" = present but switched off by
default. Status: in use / switched off / orphaned (built, nothing in either pipeline calls it) / E02-only (lives only in
the older E02 runner `tools/experiment/e02.py`, still started by the Pi start page `tools/pi/launcher.py` through
`run-e02-pi.sh`) / deleted.

### Camera and recognition

| Capability | Where in code | Home | School | Switch / default | Status | Evidence it worked |
|---|---|---|---|---|---|---|
| Motion in the picture (frame differencing + floor memory, outline tracker) | `experiment/e02_camera.py` MotionTracker (MOTION_THR, l.337, 659), `e02_multitrack.py` | yes (cam_bridge) | **no** in tags mode; yes only with `--tracking lights` | site `tracking`; school = tags | home: in use; school: switched off by site choice | real footage 93-100 % of moving robots found (docs/session-notes/2026-10-21-real-footage.md:159); used every home run since 27 Oct (V) |
| Multi-robot identity by prediction (pose echo) | `e02_multitrack.py`, `cam_bridge.py` | yes | no (the tag id gives identity) | - | home in use | 4 h free roam 5 Nov (docs/results/freeroam-2026-11-05-4h-analysis.md) |
| Lock-on tracker (template lock, detector checks it) | `e02_lock.py`, `e02_camera.USE_LOCK` | indirect (imported by cam_bridge closure; I: not active in MultiTracker) | no | e02 `--no-lock` | E02-only (I) | commits 80a01fd..9ad4e43 (22-25 Oct) |
| Lights as markers (brightest spot near the prediction) | `e02_twocam.py` l.25, `cam_bridge --no-lights` | yes | no (lights hidden under the roof tag) | on; `--no-lights` | home in use | e02-oct27-two-camera-tracking.md |
| Find by lights flashing, facing check from left/right LED | `ladder.Eye.find_lights` (l.459), `facing.py` | yes (L0) | no (tags replace it; `run_plan --facing-tags`) | run_plan `--facing-check` opt-in | home in use | e02-oct29-facing-check.md, commit ffa41fd |
| Blink-code naming (rhythm identity) | `tools/blink/blink.py`, `calib.py`, `tools/live/track_live.py`, `livecore.py` | no | no | - | E02-only (NamingWatch, record only, e02.py:10365) | 17 commits 17-18 Oct; "named Left and Center too rarely on real footage" (cam_bridge.py docstring) |
| Light patterns (SK6812 plate codes) | `tools/lights/*`, `hardware/dock/build_light_plate.py` | no | no | - | orphaned (sim only) | commits 5dbac9e, 80b4b46 (18 Oct), simulated scenes only |
| Roof AprilTags (pose + heading + id) | `fleet/tagcore.py`, `tag_geom.py`, `tag_bridge.py` | no | yes | site `tracking: tags` | school in use | school runs 5 Nov; e06/e07 rehearsals |
| ML detector (CentreNet, "MIRLT model"), ONNX on the Pi | `tools/detector/*` (`infer.py`, `benchmark.py`) | no | no | - | orphaned from both pipelines (no runtime import; V grep) | trained/tested in the studio; real-footage numbers not found in this slice |
| Recognition studio (label, train, use model offline) | `tools/recognition/*`, `tools/label/*`, `start-recognition.bat`, `track-with-model.bat` | - | - | AI helper off by default | standalone offline tool (meets "usable offline") | recognition-stage2.md, commits 90548b6..be58a31 |
| Motion-based labelling detector | `tools/label/motion.py` | no | no | - | studio only | recognition-stage2.md |
| Brick watch ("what changed", obstacles from the camera) | `fleet/brickwatch.py`, `cam_bridge --brick-watch` | **no** (freeroam never passes it) | **no** (mission_run never passes it) | opt-in flag | used only by `run-fleet-pi.sh watch/brick` and ladder L6 (V) | commit 10852a9 (sim brick drop) |
| Two-camera geometry | `e02_twocam.py` (home); `tag_bridge` inverse-variance pair (school) | yes when side calibrated | supported, **side camera switched off** (`cameras.side.use: false`) | site file | home in use; school switched off | e02-oct27-two-camera-tracking.md; commits 0f1e998, c0b3b28 |
| Tracking spread over cores | `fleet/trackfarm.py` | no (freeroam does not pass `--farm`) | no (`mission_run --farm` opt-in; tags path ignores it) | `--farm` off | switched off | bench_trackfarm.py |
| Lighting check / brightness watch | `fleet/lighting.py`; `tag_bridge.light_check` | I: ladder | yes (tag_bridge) | - | in use | - |
| Bumped-camera watch | `ref_tags.BUMP_*` in tag_bridge; run_plan `camera_moved` | **no** | yes | site ref_tags.use | school only | commit 4ee0e85 |

### Calibration and discovery

| Capability | Where in code | Home | School | Switch / default | Status | Evidence |
|---|---|---|---|---|---|---|
| Camera floor calibration by corner clicks | `pi/camera_setup.py`, `fleet/corner_setup.py`, `setup_job.py` | yes (static roam1 file) | fallback only | `corner_clicks`: home true, school false, Start `auto` | in use (home) | e02-oct27-camera-setup.md |
| Corner AprilTag tiles + site fit (no clicks), auto refit | `ref_tags.py`, `site_fit.py`, `hardware/dock/build_ref_tiles.py` | **no** | yes | `ref_tags.use` school true; home site has no block | school only | commit 4ee0e85; measure_bench e07 |
| Robots calibrate the camera + map the board (explore) | `experiment/e02_explore.py`, `explore_check.py` | **no** | **no** | e02 default setup (`--classic-setup` opt-out) | E02-only | sim 18/18 after fixes but real 09:55 board 11 % small (e02-oct26-explore-reliability.md, commit 3451b1d) |
| Calibration from the runs (camera vs robot logs, no clicking) | `tools/floorcal_runs.py` | no | no | - | orphaned (only calibrate_floor.py) | - |
| Camera scale from the robot's own size | `label/size_check.py` | no | no | - | orphaned (mark_robot page only) | commit 9eff890 |
| Checker pieces / charuco tie between cameras | `pi/checker_pieces.py`, `charuco_calib.py` | yes (setup page) | no (ref tags) | - | home setup | e02-oct28-checker-pieces.md |
| Roof tag height / tilt / offset measured | `fleet/roof_cal.py`, `sim/roof_sim.py` | n/a | yes | `tags.roof_measured` | school in use | e02-nov03-roof-tag-height-tilt.md |
| Feel out the table (edge found by the edge guard vs camera line) | `fleet/feel_table.py`, `feel_job.py` | **no** | yes (tag sites, first run per table) | Mission Control tick-box | school only | commit 729c572 |
| Board edge find by touch (any two boundaries), perimeter mapping | `e02_edgefind.py`, `e02_map.py` | no | no | e02 `--edge-find`, `--map` | E02-only | commits b1eae42, b1e6cd6 |
| Gyro/wheel self-calibration fitted from camera | `e02_fit.py` (E02); `calib_run.py` fit phases (school) | **prior only** (`e02_fit.PRIOR` + corrections.json; ladder.py:181-200) | yes (fitted) | - | school in use; home uses a fixed prior | real L2 27 Oct: +5..+10 deg per quarter turn on factory values (ladder.py:181) |
| Calibration saved on the robot | firmware `corr.h` `save`/`corr save`; `calib_run.py:1961` | only the curve values (`freeroam_core.arc_save`, `--no-arc-save`) | yes (`calibration.save_on_robot: true`) | - | school full; home partial | e04-nov05-baseline-pins-ota.md, commit 60f4f77 |
| Calibration room (spin room check) | `calib_room.py` | yes (L1) | yes | - | both | e02-oct29-calibration-room.md |
| Heading check: crawl / travel heading | ladder crawl; `cam_bridge` + `e02_steer.heading_from_track` | yes | - | `--no-travel-heading` | home in use | - |
| Heading check: tag vs motion, all run | `observe.heading_offset`, `HeadingWatch` (run_plan.py:1018) | **no** | yes | - | school only | AL-1DC0 tag 180 deg off, 5 Nov (observe.py docstring) |
| Straighten at start | `straighten.py` | no | yes (mission_run) | - | school | e02-oct29-start-straighten-blind-strip.md |
| Wiggle naming (robot <-> tag) | `wiggle_bind.py` | n/a | yes | `tags.binding: wiggle` | school | commit 60f4f77 |
| Surface marks (seam tape vs real drop) | `observe.edge_trip_kind`, run_plan `--surface-marks log` | **no** (freeroam logs own-vs-camera per move but builds no map; freeroam_core.py:1326) | yes (log only) | `log` default; `act/auto/off` | school logs only; nobody maps seams | commit 18fe4e6, 51aab2e |
| Readability map (where tags read) | `observe.ReadMap` | n/a | yes | - | school | observe.py: tags read to x ~100 cm only |
| Blind sensor discovery (which RAW column is which) | `tools/discovery/*` | no | no | - | orphaned (logs only) | 93 % held-out typed right, 0 wrong of 234 accepted (commit b8dc764) |
| Front sensor offset learned per session | `e02_front.py` | no | no | e02 `--no-front-learn` | E02-only | commit 23bb170 |
| Site detection (home / school / new area) | `site_detect.py` | via mc | yes | `site auto` / pin | in use | e07 |

### Fusion and sensors

| Capability | Where in code | Home | School | Switch / default | Status | Evidence |
|---|---|---|---|---|---|---|
| On-robot wheel + gyro pose, camera fix blended (one-step Kalman, latency compensated) | firmware `pose.h`, `follow.h` | yes | yes | - | both | every run since 27 Oct |
| Multi-sensor EKF (wheels, gyro, camera, lights, heading, landmark ranges) | `tools/fusion/ekf.py`, `engine.py` | **no** | **no** | - | E02 Live lab (display only) + offline | 15-30 % closer than wheels+gyro after camera gaps on real sessions (e02-oct25-real-fusion.md; e02_fusesteer.py docstring) |
| Steering on the fused estimate | `e02_fusesteer.py` | no | no | `--fusion-steer`, off | switched off, never run on hardware | commit 9ad4e43 |
| Factor-graph smoother | `fusion/graph.py` | no | no | - | offline only | commit 45809f0 |
| Fusion tracking rules (possible area, look-alikes, pick-up) | `fusion/engine.py` | no | no | - | offline only | fusion README |
| Learned sensor trust | `e02_trust.py` | no | no | e02 `--trust-file` | E02-only | e02-oct25-trust-fix.md |
| Learned odometry uncertainty | `e02_unc.py` | no (freeroam uses fixed BLIND_* from 29 Oct logs, freeroam_core.py:93) | no | e02 `--no-unc-learn` | E02-only | commit 8a7297b |
| Accelerometer: standing tilt watch | `observe.TiltWatch`, run_plan.py:519 | **no** | yes | - | school only, though built after a HOME free roam incident (Center tilted 64 min, run_plan.py:520) | commit e6140fe |
| Accelerometer: jolt / bump detection | firmware `e02.h` (bump_jolt_g) | no | no | - | E02-only | e02-oct25-reverse-find.md |
| Front ToF: stop rule | firmware follow.h; `front_stop.py` | yes (firmware front stop, `follow_look_cm`) | yes (+ laptop rule) | - | both | front_stop_replay.py |
| Front ToF: board approach fused with pose | follow.h `fuse_from_cm` (A step) | no | only planner board approaches | - | run_plan only | - |
| Front ToF: landmark ranging, site survey, change check | `fusion/landmarks.py`, `tof_calib.py`, `tools/site/*`, run_plan `--site-map` | no | no (nothing passes `--site-map`) | off | orphaned, sim only (E00d "planned (simulation done)") | commit f5ec925 |
| Line / cliff sensors: edge guard | firmware `edge.h`, `safe_edge` | yes (3 = both cues) | yes (2 = bottom zone only) | site `calibration.safe_edge` | both | 15 own stops, no falls (commit a002b4d) |
| Virtual border | firmware `border.h` | yes | yes | - | both | commit 10852a9 |
| Bumper switches / touch probe | firmware `switches.h`, `dock.h` P step | no (`set bumper_cm 0`) | no (`bumper_cm 0`, mission_run.py EDGE_GUARD) | e02 `--switches-back` | E02-only / orphaned | sim only |
| Dock / self-charging | firmware `dock.h` K step, run_plan `--home`, `fusion/dockfix.py`, `tools/dock/*`, `hardware/dock/*` | no | no | `--home` never passed (V) | orphaned; E00e "planned (simulation done)" | charge levels live only in e02.py:4316 |
| Battery drain learning | `tools/dock/battery_rates.py`, `charge_levels.py` | no (freeroam parks at a fixed 10 %) | no | - | E02-only | commit 79324f9 |
| Heartbeat watchdog | firmware follow.h `follow_wd_ms` | yes | I: yes | off unless set | in use (home) | commit ad52186 |

### Operation, records, experiments

| Capability | Where in code | Home | School | Switch / default | Status | Evidence |
|---|---|---|---|---|---|---|
| Recorder (video, /live, shm) | `pi/camrec.py` | yes | yes | - | both | - |
| Frames at the tracking rate | `doe_loop.Frames`, `e02_framesaver.py` | yes | yes | `--no-frames` | both | - |
| CPU / heat log | `e02_cpulog`, `calib_run.start_cpu_log`; `analysis/heat_tracker.py` | yes | yes | tracker run by hand at analysis | both | docs/results/heat-tracker.md |
| Live page with the robots on the picture | `e02_view.py` (8090, EKF Live lab); MC engineer panels; `mapview.py` | **no** (freeroam has tmux status only; no page) | MC screens; mapview only in the old Warm-up screen, not on the Start screen | - | partial | e02-oct25-live-page.md |
| Warm-up (side feels, wheel/gyro vs camera) | `fleet/warmup.py` | no | in MC, not on the Start screen's four buttons (start_job.py: 0 refs) | - | likely stale (I) | commit 10852a9 |
| DOE loop (unattended out-and-backs) | `fleet/doe_loop.py` | `run-fleet-pi.sh doe` (separate) | no | - | home only | e02-oct27-doe-loop.md |
| Fast stress simulator | `freeroam_fastsim.py` | yes (dev) | no equivalent for run_plan roam | - | home only | commit ad52186 |
| Simulators that break the guesses | `sim/fleet_sim.py`, `tag_cam.py`, `table_cam.py`, `e02_sim.py` | yes | yes | - | both | e06/e07 rehearsals |
| Dataset packaging for publication | `tools/dataset/package.py` | no | no | - | orphaned (manual) | publication plan 25 Oct |
| Old roam (camera-steered random spots) | `fleet/roam.py` | no | no | - | orphaned (superseded by freeroam; nothing imports it) | - |
| Jay-Sun data exports / sync | `run_exports.py`, `sync_tools.py`, `merge_external.py` | no | yes | - | school | not RFID inputs |
| RFID tools | `tools/rfid/*` behind `rfid_gate.py` | no | no | off unless `--jay-requested` | switched off (correct per rule) | commit a95a8f5 |

**Deleted in git history (V):** almost nothing. `git log --diff-filter=D` shows only two STLs (nose bumper, 5909728),
an experiment README (7f587f1) and placeholders (9258879); one rename (`discovery/score.py` -> `disc_score.py`). So the
"left behind" problem is **not deletion**: whole working subsystems sit in the repo with no caller in the two current
pipelines. The pattern (V from the commit dates): each new pipeline (E02 runner 21-26 Oct -> fleet/ladder 27 Oct ->
Mission Control and tags 2 Nov -> free roam 5 Nov) was built beside the last one, not on it, and carried over only what
its build needed.

## 3. Gaps

**Built and proven, but neither current pipeline uses it (V):** the fusion EKF and fused steering; learned trust and
uncertainty; explore / edge find / perimeter map (robots map the board and fit the camera); front-sensor landmark
ranging and the site survey; brick watch during runs; blink and light-pattern identity; the ML detector; blind sensor
discovery; battery-drain learning; dock and touch probe; the jolt bump cue; the E02 live page.

**One pipeline has it, the other lacks it (V):**
- School only, home lacks it: TiltWatch (accelerometer), HeadingWatch, surface-mark classification, FailMemory and
  Trail (`observe.py` via run_plan), fitted calibration saved on the robot (calib_run), feel out the table, corner
  reference tiles and auto refit, bumped-camera watch, ReadMap.
- Home only, school lacks it: motion/outline tracking, lights as markers, two cameras (switched off at school), the
  reflex / reservation / continuous-curves controller (`freeroam_core`; school free roam is planner + run_plan), the
  fast stress simulator, the DOE loop.

## 4. Left behind, ranked (most important first)

1. **Motion in the picture at school.** The tag bridge has no motion/outline source, and tags read only to about x 100 cm in
   room light (observe.py TAG_PX_MIN note), so points past that are skipped. A robot whose tag drops runs blind. Matters
   for the objectives "fuse every signal", "degrade gracefully" and the demo. To bring back: run `e02_multitrack.MultiTracker`
   inside `tag_bridge` as a second source, seeded and identified by the last tag read (tags give identity, motion gives
   position when tags fail). Both bridges already send the same fix messages. Medium effort: 1-2 days plus a CPU check
   at 1280x960 (trackfarm exists for that).
2. **Real sensor fusion in the loop.** The proven EKF (15-30 % better after camera gaps) is used by neither pipeline.
   Today the robot does only a one-step blend of camera into wheels and gyro. To bring back: run `fusion/ekf.py` per robot in
   both bridges / freeroam (inputs: fixes plus FOLLOW/RAW rows already logged), and use it for lost mode and reflex. Turn on
   the `e02_fusesteer` choose() rule. Medium to large; the code and its replay tests exist.
3. **The two "free roams" have split.** Home lacks TiltWatch, which was written because of a home free-roam tilt (run_plan.py:520).
   It also lacks HeadingWatch, surface marks, FailMemory and Trail. School lacks the reflex/curves controller. To bring
   back: feed RAW roll/pitch and the observe.* watches into `freeroam_core` (small: these are pure functions), and decide
   whether the school's roam should be `freeroam_core` on tag fixes (medium).
4. **Home board and camera are typed, not discovered.** `HOME_BOARD` is hard-coded, and the camera uses a static corner-click
   file. That breaks the "Discover, don't pre-map" rule. Explore, feel_table, the ref tiles and floorcal_runs all exist.
   To bring back: give the home site a `ref_tags` block or run feel_table at the start of freeroam (small to medium). Explore needs
   real-board fixes first (11 % scale error on its last real run).
5. **Home calibration is a prior, not a measurement saved on the robot.** calib_run fits and saves at school; home's L1 uses
   `e02_fit.PRIOR` + a hand file, and the values are lost at power-off. To bring back: run calib_run's fit/save at home,
   or save L1's result (small).
6. **Learned trust and uncertainty.** The fleet uses chosen constants (FIX_SIGMA_CM 1.0 "chosen", fixed BLIND_*), while
   `e02_trust` and `e02_unc` learned these per session. That fits "self-calibrating". Medium.
7. **Camera obstacle detection (brick watch) is off in every real run.** Freeroam and mission_run never pass `--brick-watch`.
   Small to switch on for lights tracking; needs motion at school (item 1).
8. **No live view for free roam.** The demo and the "engineer view / mapping visuals" objective need one. The E02 view
   (8090) and mapview exist. Medium.
9. **ML detector (MIRLT model) never in the loop.** It runs on the Pi via ONNX (`benchmark.py`). It could be a third camera
   source and gives the publishable story. Medium; the model's real accuracy was not verified in this slice.
10. **ToF landmark ranging and site survey** (`tools/site`, `fusion/landmarks`, `tof_calib`): front sensor used only as a stop.
    Sim only, never on hardware. Large.
11. **Blind sensor discovery** (93 % / 0 wrong on logs): the plug-and-play objective. Not in any start-up. Medium.
12. **Dock, self-charging, bumper switches, touch probe, battery-drain learning**: complete in firmware, sim, CAD and
    policy, never tested on hardware, no caller. Large (hardware).
13. **Second camera at school** switched off by the site file; the tag bridge already fuses two cameras. Small (site switch +
    calibration).
14. **Blink / light-pattern identity** (superseded by prediction and wiggle), the **jolt bump cue** (E02 firmware only), and
    **old roam.py** (orphan). Low; keep switched off.

**Possibly outdated (I):** the Pi start page (`tools/pi/launcher.py`) still builds `run-e02-pi.sh` arguments (E02
runner) as its main Start, beside the Mission Control tab. The E02-only features above are reachable only from there.
Worth deciding whether the E02 runner is still a supported pipeline or a parts bin.
