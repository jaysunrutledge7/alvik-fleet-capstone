# Full audit, 7 Nov 2026: what got left behind, and which rules are out of date

Repo at `85f5e99` (6 Nov). Read-only: nothing was run on robots or Pis. Evidence and line references are in the four
slice reports in `docs/audits/2026-11-07/` (A tools, B rules, C findings, D chat) and the reviewer's check (E). **V** = checked in the code or docs;
**I** = Claude's reading, not proven.

## Premise

Nick found that motion detection, a core part of the project, was missing from the school build, and asked whether
smaller parts had been dropped too. The audit judges everything against the long-term objective first (a
plug-and-play, self-calibrating system that fuses every signal, degrades gracefully, and is publishable, with a live
December demo), then the immediate one (reliable school runs with Jay-Sun, and home runs that stay on the board).

## Data: how it was checked

| Slice | Sources | Result |
|---|---|---|
| A. Tools | Imports and subprocess calls traced from each entry point; all 714 commits for deletions | 60 capabilities tabled by pipeline |
| B. Rules | CLAUDE.md, docs, site files, 319 dated rule comments in the code, Nick's saved decisions | 16 outdated, 9 contradictions, 14 not built, ~20 chosen numbers |
| C. Findings | Every build report, results and session notes (20 Oct to 6 Nov) | 15 measured insights, 12 undone follow-ups, 10 dropped techniques, 8 repeated faults |
| D. Chat | This chat (6-7 Nov), 45 build briefs (28 Oct-6 Nov), saved decisions | Each decision marked built / partly / lost / superseded |

## Conclusion

### 1. Why things got left behind

Almost nothing was deleted (V: in all history, only two STL files, two pictures, an old secrets template, a README, placeholders and five early data sheets). Instead, each new
pipeline was **built beside the last one, not on it**, and carried over only what its own build needed: the E02 runner
(21-26 Oct), then the fleet ladder (27 Oct), then Mission Control with roof tags (2 Nov), then free roam (5 Nov). Whole
working subsystems now have no caller. The same split explains why the two current pipelines lack each other's
safeguards.

### 2. Left behind: major (most important first)

| # | What | Proven result | Home now | School now |
|---|---|---|---|---|
| M1 | **Motion detection** with the lock-on tracker, plus the "what changed" (brick) watch and the parked-robot watch | Robots kept in view 94 / 93 / 96 % of pictures, against 41 / 40 / 72 % for the detector alone (22-23 Oct, three evening losses) | Motion detector yes; lock-on and parked watch no; brick watch built in but no real run turns it on | **None.** Tags only, and tags read to about x 100 cm, so the far half is driven blind |
| M2 | **Sensor fusion (EKF)** in the loop | Error after 40 s with no camera: 5.1 cm fused, 7.3 wheels and gyro, 19.2 the robot's own pose (25 Oct, real data; the 40 s row has only 7 gaps) | No (camera blended once into wheels and gyro on the robot) | No |
| M3 | **Heading from the raw gyro rate**, bias taken at every standstill | Uncalibrated routes ended 2.0 cm off instead of 15.9 cm (25 Oct) | No: firmware still uses the library's yaw | No |
| M4 | **One set of safety watches for both pipelines** | Each pipeline gained checks the other never got | Has the tilt watch and fail memory; lacks the heading watch, stop lead and surface marks | Lacks the free-roam controller (reflex, reservations, continuous curves) and motion tracking |
| M5 | **Camera reads 5-9 cm outward at open edges** | Measured 25 and 27 Oct | Ignored: camera trusted at 1 cm near edges (widened only for side views and lights) | n/a (tags) |
| M6 | **Learned trust per sensor and learned uncertainty** | Turn error 0.014 per degree measured, not the 0.005 chosen; false sensor flags went to zero | Fixed chosen weights and blind-driving numbers measured only to 40 cm | Same |
| M7 | **The system discovering its own setup** | Blind sensor naming 93 % right, 0 wrong of 234; feel-out-the-table built 2 Nov | Board size typed in code; corners clicked; robot calibration a fixed prior, lost at power-off | Feel-out exists but the new Start screen never calls it; tag height and table length typed |
| M8 | **A measurement not taken by the camera** (tape marks or a tape-measured drive) | Never used to grade a robot: the 18 Oct tape rig (E00) only checked its own precision. Wheel-vs-camera gap read 8 %, then 4 %, then 2.4 % | - | - |

M8 matters for publication: every accuracy result so far, including "self-calibration 1.4 cm against 16.4 cm", is
graded by the same camera that calibrates the robots.

### 3. Left behind: smaller

- Camera drift watch (home), front-sensor range cap (it reads 10-15 cm short past 30 cm), the 5-minute video-file
  delay spike, a network ping log (promised twice), robot-assisted search for a lost robot, reverse find with the back
  switches, the dock and self-charging (hardware never tested), the ML detector as a camera source, the second school
  camera (switched off in the site file), a live view for free roam.
- Never run: the latency test, the lights-off and degraded-sensor runs, the tilt/edge bench test (asked for in five
  reports), and about 200 of the roughly 300 DOE backlog rows (no board or bench row ever). The e05-e07 chosen numbers have no backlog rows.
- The calibration "look" still reads the floor as a wall (seen 19 and 30 Oct; fix proposed twice).

### 4. Repeated faults

| Fault | Times seen | Root cause (V unless marked) |
|---|---|---|
| Planning from a stale or wrong position | 8 (24 Oct to 6 Nov; the 6 Nov case is from this week's logs, not yet in the repo) | When camera and wheels disagree, the code waits 2 s before correcting, and doesn't widen the robot's uncertainty meanwhile |
| Robots at, over or hanging on an edge | ~10 (22 Oct to 6 Nov) | Edge guard looks forward only; the camera's outward edge bias; curves swing wide (3 cm from edges in simulation); tilt limits never set from data |
| Heading lost after a restart or sleep | 5 | Patched each time; still open |
| "Out of re-plans" endings | 4 | Still open |
| Two close robots' names swapped | 4 | Patched each time; the fast simulator never swaps names, so its results can't show it |

### 5. Outdated rules (retire or rewrite)

Each kept as a switch where it is code (keep-features rule).
1. Home stops the run on a problem (`home.json`, defaults in `sites.py`, `run_plan`, Mission Control), against "no
   stops, warn and log" (30 Oct) and "stop only when safety can't be kept" (5 Nov).
2. Free roam puts a robot out for the whole run after 120 s silent, or after a failed heading crawl (Right, 6 Nov),
   against "no robot permanently out".
3. Free roam keeps every felt obstacle for the whole run; the 29 Oct rule makes sensor-only obstacles fade after 15 s.
4. Five edge distances (17 cm missions, 16 free roam, 13 school points, 25 DOE and ladder, 12/21 calibration room), plus the 6 cm virtual border, answer one question.
5. Two battery floors (25 % "one source of truth", and free roam's 10 %).
6. The DOE loop parks robots unless keep-going is passed.
7. The Monday 2 Nov demo machinery and its reasons (still wired into Mission Control and `run-fleet-pi.sh monday`).
8. "Robots stay awake, sleep after 30 min" in `calib_run` and its runbook (6 Nov: sleep right away).
9. Corner clicking as the active home setup (5-6 Nov: automatic, corners only if necessary).
10. Docs that describe an older system: `pi-run.md` (E02 runner as the main path), README (8x8 tape grid), DESIGN.md,
    the 19 Oct decision (nose switches, two docks), the experiment index (18 Oct), `open-questions.md` (30 Oct), and
    four one-night runbooks.
11. "Markerless / one camera / no clicks" claims in the summary and the publication plan: true only at home.

Saved memory notes now outdated: RFID "integrate the programs" (now never an input), "OK with robots going over the
edge" (26 Oct), "fixed light combination" identity (wiggle first since 5 Nov), dock nose switches (back attachment
since 25 Oct), "ML detector, no markers" and "EKF on each robot" (goals, not the current system).

### 6. What this means for the next build (Claude's recommendation)

The left-behind items share one cause (pipelines built side by side), so patching each pipeline again would repeat it.
The recommended next build makes **one pipeline**, whatever the site:
1. **One tracking layer** that takes every camera source the site has (roof tags, motion with lock-on, lights, later
   the ML detector), with the brick and parked watches on.
2. **The EKF in the loop**, fed by those sources plus wheels, raw gyro rate, accelerometer, front and line sensors,
   with learned trust and uncertainty. A disagreement widens the uncertainty at once (fixes the stale-position fault).
3. **One controller** (free roam's) with all of `observe.py`'s watches, used for free roam, visit points and experiments.
4. **Discovery at start**: feel out the board, proven workspace, calibration fitted and saved on the robot.
5. Edge safety rebuilt on 1-3, with the camera's edge bias and a derived edge distance.

## What's next

1. Nick decides the questions in the decision panel (one build or patches; the E02 runner; the outdated rules).
2. Logs-only checks that need no board time (run without asking): tilt limits and keep-out build-up from the 5 Nov
   and 6 Nov home runs; how often the virtual border has ever fired; battery behaviour below 25 %.
3. The re-scoped phase 3, with a time estimate once the decisions are in.
4. A tape-measured reference (taped marks or a taped 100 cm drive) before any accuracy claim is submitted.

## Sweep 1: new tests that may now be needed

- A simulator case for 6 Nov: a position 4.6 cm wrong just before a wide curve near an edge.
- A tilted robot at an edge with "no room to back away".
- A robot that fails its heading crawl and should rejoin, not sit out.
- Motion tracking at school seeded by the tag (simulator first, then real tiles in room light).
- Fading versus permanent keep-outs over 4 h with injected false stops.
- The fused estimate replayed on the 5 Nov 4 h log against the current blend.

## Sweep 2: possible inaccuracies in what this audit relied on

- Chats before 28 Oct were not reachable; earlier decisions are judged through Nick's saved notes and Claude's briefs,
  not his own words.
- "Not used" findings rest on traced imports and greps; code loaded another way could be missed.
- Many rule dates and the word "Nick" in code comments were written by Claude builds, not checked against the chat.
- The 6 Nov edge diagnosis uses the first run's logs only; the second run's logs had not uploaded.
- Proven results (M1-M3) come from single sessions in late October, on the home board and older firmware.
- The motion result (94 / 93 / 96 %) was measured on three evening losses, not over whole runs.
- An independent reviewer checked this report against the code (`2026-11-07/E-verify.md`). It found one wrong claim (home free
  roam does have the tilt watch and fail memory) and six overstated ones; all are corrected above.
