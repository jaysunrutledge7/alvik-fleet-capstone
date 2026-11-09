# e11: school free roam fixes, the real-path practice, discovery fix, Visit points in your order (9 Nov, simulator and logs only)

## Premise

This morning's first real Mission Control free roam at school (AL-1DC0 as "Alpha", one camera, 0.8 GB card) failed
three ways after the two crash fixes already pushed: no robot answered, the motion setting did not reach the run, and
a refused start still put the robot to sleep. All e10 rehearsals had used the dry path, which differs from the real one.
Mid-build, the first real home free roam on e10 froze for 9 minutes in start discovery (coordinator's message), and
Jay-Sun's Experiment A tours were added to part 2. Nothing here touched a robot, a Pi or a camera.

## Data

### Fixes (each old way kept behind a switch)

| Problem | Cause found | Fix | Switch / old way |
|---|---|---|---|
| "No robot answered" (school) | mission_run built the free-roam command afresh and dropped the run's `--alias AL-1DC0=Alpha`; freeroam.py had no alias option, so it waited for "Alpha" while the robot answered as "AL-1DC0" | `freeroam.py --alias WIFI=SLOT` into its robot link (the same mechanism as run_plan): link, fixes, logs, status and the identity wiggle all use the slot name; mission_run passes the aliases; the run's robots come from the map | - |
| Motion "curves" when straight lines were wanted | Every link in the chain passes it in this code (Start screen Advanced -> start_job -> mission_control -> mission_run -> freeroam). The likeliest cause is that Mission Control was still running code from before this morning's c82674e (it must be restarted to load new code); a second weak point: mission_run read `--motion` only if the exact text "--motion" was in its command line | `--motion` resolved after parsing (any form); the run says "FREE ROAM DRIVING: ..." at the start; the Free roam button says the driving it will use; the report adds "Motion actually used" per robot (from events.csv) | - |
| Refused start slept the robots | `on_run_end` slept the robots on every end | A start that was refused, failed, or never reached "the robots start now": robots stay awake, slept after 10 min idle with no new run; a new run cancels it; the page says why it was refused and when they will sleep. "No robot answered" now shows as a plain refusal | Advanced "sleep when a run ends" unchanged |
| Home free roam froze 9 min (9 Nov, 12:35 run) | The proof drive's stands (1.2-1.5 s) ended on the clock alone. The busy home Pi gave 4-5 camera fixes a second, so a stand got 3-4 sightings, under the 4 needed: "out of view" for all three. The run then switched the camera OFF (that is why camera_track.csv stops after about 30 s) and kept a 0 cm2 workspace, so every robot was "over the edge" | A stand ends when the camera has given it 4 sightings (waits up to 6 s). A camera discovery cannot prove now falls back to the typed board with the camera on, said in capitals | `--discovery-strict` (e10's narrowing); `MIRLT_DISCOVERY=0` as before |
| (found in passing) | freeroam's simulated camera delay and its held robots shared one variable (TypeError in `--sim-cam-delay` runs) | separate variables | - |

### The real-path practice (part 1's main fix)

`mission_run --practice-real` (the Start screen's practice uses it by default; Advanced "Practice runs take" = real /
dry): simulated robots answer on the WiFi as AL-1DC0, AL-2B7F ... (`fleet_sim --wifi-name`) and the run names them by
slot through aliases; the same free-roam launcher and arguments as a real run (freeroam.py not marked dry); the
readability map seeded from real `CameraModel`s, projected one point at a time as on the Pi.

| Rehearsal (real path unless said) | Result | Source |
|---|---|---|
| School free roam, 1 robot, stopturn, 1.5 min | Robot answered as AL-1DC0, ran as Alpha; discovery "check out"; moved 95 %; 1,287 camera-track rows; report "Motion style: stopturn", "Motion actually used: Alpha stop-turn-drive" | scratch `reh/r1` |
| School Visit points (shared out), 1 robot | Done, 20 points, 0 problems; every point within 0.1 cm of the simulator's truth | `reh/v1` |
| Jay-Sun's custom order (first 10), 10 cm/s, 8 s stops | P04 and P19 flagged and skipped before the run (the simulated camera cannot read them); the 8 others driven in order; 8 rows in stops.csv, each dwell 8.0 s | `reh/vo1` |
| Home dry, camera at 4.5 fixes/s, e10 code | **Reproduced** the home freeze: all "out of view", 0 cm2, every robot "-2.5 cm from the edge" | `reh/home_slow` |
| Same, e11 code | "Camera, board and clock check out"; robots moving 92-94 % | `reh/home_slow2` |
| Same, camera at 0.5 fixes/s (cannot be proven) | Loud typed-board fallback; workspace 14,400 cm2; robots moving 91-93 % | `reh/home_vslow` |

Not rehearsed on the real path: the calibration run (calib_run has its own path), and Mission Control's Start screen
end to end through the browser (its pieces were run separately: the API, the mission_run command it builds).

Tests: `tools/fleet/tests/test_e11_school_fixes.py` (19). Fleet suite: 754 passed, 9 skipped before the discovery
fix; the files that fix touches re-run after it (70 passed).

### Part 2: Visit points in your order (on; Advanced "Visit points in your order" hides it)

- **Entry:** typed (P01,P05,P11,P05), clicked on the map, or clicked in the overhead camera's picture (the nearest
  sticker, through the site's camera model). The order shows as numbers on both; repeats show as "2,4".
- **Presets** (Jay-Sun): In order (P01-P20), Reversed (P20-P01), "Jay-Sun custom" (P12 P18 P09 P14 P15 P13 P17 P11 P04
  P19 P08 P01 P03 P20 P07 P05 P10 P16 P02 P06). "Save this order as..." keeps more, in the site's Mission Control folder.
- **Speed and stop** on the card: 10 cm/s and 8 s by default.
- **Unreadable points** are listed in a box before the run ("Will be SKIPPED: ..."), skipped by default; "try
  unreadable points anyway (this run)" keeps them. The run checks again and writes `visit_order.json`.
- **One robot takes the list**; with more, each takes the next run of points, or `Alpha:P01,P05;Bravo:P11`.
- **Logs:** camera_track.csv, odometry_raw.csv, the event log as before, plus new **run/stops.csv**: per stop the
  arrival, dwell start and end in Unix seconds on the Pi's clock, ISO times, and the camera's median position during
  the dwell. Arrival = dwell start (the wait step starts as the drive to the point ends).
- **Command line:** `./run-fleet-pi.sh visit P01,P05,P11 [keep] [dry]`; `mission_run --visit-order ...`.
- **jay-bundle** (rehearsed on `reh/vo1`, 28 files): run/camera_track.csv, odometry_raw.csv, raw.csv, follow.csv,
  step.csv, points.csv, **stops.csv**, **visit_order.json**, clock_sync.csv, param.csv, runinfo.csv,
  coordinator.jsonl, experiment.json, run_plan_summary.json, plan_given.json, messages.csv, bridge/tag_poses.csv,
  bridge/fixes.csv, bridge/track.csv, mission_summary.json, program.json, mission.json, sync_marks.jsonl,
  pose_timeline.csv, tags.json, site/school-conference.json, site/school-conference-points.csv, README.txt.
  `latest` finds Mission Control runs only; a `./run-fleet-pi.sh visit` run needs its folder given.

Pictures: `docs/screenshots/2026-11-09/` (5, index lines added).

## Conclusion

Part 1 and part 2 are pushed and rehearsed in the simulator. **My conclusions (not yet seen on the robots):** the alias
fix is what the school run needed, since the practice now reproduces the exact naming the real run had; the motion
problem was most likely stale Mission Control code, which the restart in the steps below removes; the home freeze was
discovery's timing against a slow camera, not lost fixes.

## Next

- The 3:00 steps below. Watch the first lines of the free roam for "Names in this run only: AL-1DC0 is Alpha" and
  "FREE ROAM DRIVING: straight lines".
- The per-site card-space floor (tonight's build, not this one).
- Real-path rehearsal of the calibration run.

## The 3:00 run steps (school Pi)

Type each command in the **terminal window connected to the school Pi**, then press Enter.

1. Go to the project folder:
   ```
   cd ~/alvik-fleet-capstone
   ```
2. Get the new code. You should see a list of changed files (or "Already up to date" if it was pulled already):
   ```
   git pull
   ```
3. Close Mission Control (it must restart to load the new code). You should see "Mission Control is closing":
   ```
   ./run-fleet-pi.sh mc-stop
   ```
4. Start it again. You should see the address to open in the laptop's browser:
   ```
   ./run-fleet-pi.sh mc
   ```
5. Check the card-space setting is still there. You should see `MIRLT_CARD_STOP_GB=0.55`:
   ```
   tmux show-environment -g MIRLT_CARD_STOP_GB
   ```
   If you see `-MIRLT_CARD_STOP_GB` or "unknown variable", type this, then repeat steps 3 and 4:
   ```
   tmux set-environment -g MIRLT_CARD_STOP_GB 0.55
   ```
6. In the **laptop's browser**, open the Mission Control address and reload the page (Ctrl+R). On the Start screen open
   **Advanced** and check: **Free roam driving** = "Straight lines: stop and turn on the spot"; **Visit points in your
   order** = "Shown on the Start screen". The **Free roam** button should now read "...in straight lines...".
7. **Free roam:** press **Free roam**, confirm Jay-Sun is in the room. Within about a minute the status box should show
   lines with "Names in this run only ... AL-1DC0 is Alpha", "FREE ROAM DRIVING: straight lines", then "Alpha joins at
   ...". If it is refused, the box says why and that the robot stays awake (asleep after 10 min with no run).
8. **Jay-Sun's tours:** in "Visit points in your order", press **In order (P01-P20)**, **Reversed** or **Jay-Sun custom**.
   Check the red box "Will be SKIPPED" (expected: P12 and P15). Tick "try unreadable points anyway" only if he wants
   them. Speed 10, stop 8. Press **Visit points**.
9. After a run, his files. You should see "bundle for Jay-Sun: ... .zip":
   ```
   ./run-fleet-pi.sh jay-bundle
   ```

## Sweep 1: new tests that may now be necessary

- On the Pi, click three stickers in the live picture and check each number lands on its sticker (the simulator's
  practice-room picture shows its own sticker set, so alignment was checked only by construction).
- Calibration run through the real-path practice.
- A real-path practice with two robots and a restart mid-run (the alias after a reboot).
- Mission Control's Start screen end to end in a browser with `--practice-real`, free roam and visit.
- Discovery with the camera at 2-3 fixes a second (both cameras and frame saving on a Pi 4).

## Sweep 2: possible inaccuracies in what the work relied on

- **The motion cause is inferred**, not proven: the school Pi's Mission Control log was not available.
- **10 minutes** before the deferred sleep: Nick's number. **STAND_MAX_S 6 s** and the 4-sighting minimum: chosen, not
  derived; a DOE over camera rates (1-12 fixes/s) on the fast simulator would set them properly.
- **Arrival = dwell start** in stops.csv: true for the planner's goto-then-wait steps; a re-plan that inserts a step
  between them would make arrival earlier than logged.
- **Unreadable points** come from the camera model's read-rate seed plus earlier runs' readmap.json. If P12 and P15's
  failures this morning were not saved there, the model alone decides; at x about 150 cm they may show as readable.
- **The readable area in practice** is the simulator's camera, not the school's (the practice skipped P04 and P19, not
  P12 and P15).
- Practice WiFi names other than AL-1DC0 (AL-2B7F, ...) are placeholders.

## Questions for Nick

**Substantive**
1. When discovery cannot prove the camera, the run now uses the typed board (as before e10). Do you want it to ask on
   the page instead when someone is watching?
2. With two robots and one order, the list is split into runs (first half, second half). Would you rather each robot
   ran the whole list, one after the other?

**Other**
3. Should `jay-bundle latest` also find runs started from the command line (`./run-fleet-pi.sh visit`)?
4. Keep "Practice runs take: real path" as the default?
