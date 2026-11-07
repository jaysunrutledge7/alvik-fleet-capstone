# E-verify: claims in docs/audits/2026-11-07-full-audit.md checked against the repo (HEAD 85f5e99)

Format: claim | verdict | evidence | suggested correction. Paths are relative to the peek clone.

## WRONG

1. M4 home "Lacks tilt watch ... fail memory" | WRONG |
   - freeroam_core.py:1225 `w = r.__dict__.setdefault("twatch", OB.TiltWatch())` in `on_raw`, fed live from freeroam.py:252
     (`c.on_raw(name, t, float(row["roll_deg"]), ...)`). Tilt trips handled at :1238 `tilt_trip`.
   - freeroam_core.py:1397 `fm = r.__dict__.setdefault("fmem", OB.FailMemory())`, :1402 `OB.FailMemory.ESCALATE_N`,
     :1386 `OB.move_outcome(...)` (in `fast_done`, :1355). Also `OB.Trail` (:618) and `OB.PoseAgree` (:1189).
   - D-chat is right ("used in freeroam_core"); A-tools sec. 3 item 3 and the "school only" list are wrong on TiltWatch,
     FailMemory and Trail, and the report copied A. The report's own premise (tilt watch "written after a home run") is
     the reverse of "home lacks it": the 5 Nov commit 18fe4e6 added it to freeroam_core.
   - Still true for home: no HeadingWatch, no stop lead (`with_stop_lead` is run_plan.py:923 only), no surface-mark
     classification/`edge_trip_kind` (run_plan.py:2636-2668 only). grep of freeroam.py + freeroam_core.py returns 0 hits.
   - Correction: home "Lacks heading watch, stop lead, surface marks, ReadMap; has tilt watch, fail memory, trail,
     move-outcome vote". Also fix Sweep text and "What this means" (item 3 says "all of observe.py's watches" - fine).

2. Section 4, fault 1 "8 (24 Oct to 6 Nov, including last night)" | UNVERIFIABLE as stated (the "last night" part) |
   - C-insights.md:397-398: "The Nov 6 run is not in the repo ... my code-level inference from the description given in
     the task." The 8 table rows end at "6 Nov e07 (c) 20 deg off after a restart", a different fault (heading after
     restart). The "last night" 4.6 cm / 2 s case is from the chat (D-chat.md:44, "Nov 7 12:36Z ... assistant's log
     analysis"), not from any repo file. The report marks the root cause "V unless marked" - the 2 s rule and `unc()`
     are V (observe.py:579-580; freeroam_core.py:893-899, no live own-vs-camera term), but the last night's incident is I.
   - Correction: "8 in the reports (24 Oct to 6 Nov), plus last night's (from the chat, logs not in the repo)".

3. Sec. 2 M5 "Measured 25 and 27 Oct, again 5 Nov" | WRONG for 5 Nov |
   - docs/build-reports/e04-nov05-home-free-roam.md:278: "The camera's outward bias near edges (5-9 cm) is from
     cam_bridge's notes, not measured on this run." 5 Nov only quotes it.
   - Measured: e02-oct25-session-2114.md:228/244 (5 cm at the corner), e02-oct27-session-edges.md:74,186 ("5-9 cm").
   - Correction: "Measured 25 and 27 Oct; quoted, not re-measured, on 5 Nov".

4. Sec. 1 "only two STL files, a README and placeholders in all history" | UNDERSTATED / WRONG list |
   - `git log --diff-filter=D --name-only`: 12 files. Besides 2 STL + README: 2 PNG previews (nose_bumper_*.png),
     five E01-odometry-precision `measurements.csv` data files (commit 7f587f1; real tape-measured data, replaced by a
     new E01-baseline-dataset), firmware/secrets.example.h and tools/.gitkeep (9258879). One rename (discovery/score.py).
   - Correction: "12 files in 714 commits: 2 STLs and their 2 previews, the superseded E01 odometry README and its
     5 data sheets, and 2 placeholders". The conclusion (almost nothing deleted) still holds.

## OVERSTATED

5. M1 home "brick ... watches no" | OVERSTATED |
   - cam_bridge.py:20,330,354,609 implements `--brick-watch`; ladder.py:2494 `bridge_extra=["--brick-watch"]` (rung L6) and
     run-fleet-pi.sh:22-24,525 (`brick` test) turn it on in attended tests. Only the real runs (freeroam.py, mission_run.py)
     never pass it (grep: 0 hits). Parked watch: confirmed absent from fleet (only e02_alltrack.py:164/243 `WATCH_PARKED`).
   - Correction: "brick watch exists in cam_bridge and ladder L6 but is off in every real run; parked watch not wired".

6. M2 "5.1 / 7.3 / 19.2 cm ... (25 Oct, real data)" | numbers CORRECT, headline OVERSTATED |
   - e02-oct25-real-fusion.md:59 is the 40 s row: n = 7 gaps ("only 7 gaps (few runs last 50 s)", :62). The 20 s row (n = 100)
     reads 3.8 / 4.6 / 7.5. Use the 2-20 s rows or say n = 7. Also "no camera" = camera gap, not no camera in the run.
   - Report lists these as the "Proven result"; Sweep 2 says single sessions but not n = 7.

7. M8 "Never taken" | OVERSTATED |
   - docs/experiments/E00-measurement-check/README.md:74-104: Nick tape-measured 10 placements on 18 Oct (spacing SD 0.08 cm);
     conclusion says the camera is checked "later ... by the camera against the tape" - never done in the fleet era.
   - C-insights.md:167 "no report holds a taped result" is true of build reports, not of docs/experiments.
   - Correction: "No tape reference has been used to grade the camera or fleet results since the 18 Oct rig check".
   - Numbers 8 % / 4 % / 2.4 %: 7.8 % is in e02-oct29-corners-only-setup.md:131; the wheel scale 0.976 (e06-nov05-multirobot-observe.md:43)
     matches 2.4 %; the 4 % home figure I could not find in a build report (C-insights.md:165 cites the "4 h analysis"
     only) - UNVERIFIED.

8. Sec. 3 "203 of 222 DOE backlog rows" | OVERSTATED / unclear denominator |
   - docs/doe-backlog.md has 298 data rows (362 is the count of all `|` lines, headers and separators included).
     My tally of the Status column: 197 starting "open", 22 "ruled out", ~39 mention done/built. "222" is open + done only
     (C-insights.md:180), not the backlog size.
   - Correction: "about 200 of ~300 backlog rows still open; no board or bench row run" (the second clause holds).

9. Sec. 5 item 4 "Six different edge distances (17, 16, 13, 25, 12/21 cm and the 6 cm border)" | slightly WRONG |
   - B-rules.md C4 lists six as 17 (mission), 13 (school), 16 (free roam EDGE_CC), 17 (new-area), 25 (DOE/ladder), 12/21 (calib room);
     the 6 cm border is not in B's six (C-insights.md:339 mentions the robots' own 6 cm border separately). Code: sites/ and
     freeroam_core.py:110 PARK_EDGE_CC etc. Nit: list the six as sourced.

10. Sec. 5 item 2 / "Right, 6 Nov failed heading crawl" | UNVERIFIED |
   - OUT_SILENT_S = 120 at freeroam_core.py:105,2441-2443 is V. The "failed heading crawl (Right, 6 Nov)" case is only in freeroam.py:618
     ("no heading ... stays out of the run") as code; the Right/6 Nov incident is from e07 text in B/C slices, not re-found.

## CORRECT (checked)

- M1 numbers: 94/93/96 vs 41/40/72 % (e02-oct22-build.md:157; tests-to-run.md:30; summary/outline.md:23). They are for "three evening losses" (as Sweep 2 says).
- M1 tag_bridge has no motion/outline tracking (grep: only tagcore.WindowTracker); cam_bridge uses `e02_multitrack.MultiTracker` (docstring :11) and imports no `e02_lock` (e02_lock used only in tools/experiment/e02_alltrack.py:34, e02_camera.py:416,744); trackfarm/twocam use MultiTracker too.
- M1 "tags read to about x 100 cm": observe.py:12,52 ("the 8 cm tag read to x ~100 cm (~18 px)").
- M2 no import of tools/fusion anywhere in tools/fleet, tools/pi, run-fleet-pi.sh (grep of `import ekf|engine|replay|...`, `tools/fusion`, `fusion.`: only DESIGN.md prose and comments).
- M3 firmware/home_lab/motion.h:184-207: heading = yaw track x `P("gyro_scale")`; raw gyro (`get_imu` gx..gz) only logged (rawlog.h) or used in dock.h hold-still. 2.0 vs 15.9 cm: e02-oct25-real-fusion.md:145; open-items-2026-10-25.md:34.
- M4 school claims: HeadingWatch (run_plan.py:1018; calib_run.py:2283), TiltWatch (run_plan.py:533), FailMemory (:577), Trail (:839), stop lead (:923), surface marks (:2636-2668) are run_plan-only. TiltWatch history comment: run_plan.py:520 "home 5 Nov: Center stood tilted 3.4 deg ... 64 min".
- M5 `FIX_SIGMA_CM = 1.0` "chosen" (cam_bridge.py:56) with the 5-9 cm note at :56-60. Caveat: it is not strictly flat everywhere: cam_bridge.py:398 `sig = FIX_SIGMA_CM * max(1.0, st.get("sigma") or 1.0)` stretches it for side views and lights (comment "lights: 2 cm"). "Flat 1 cm everywhere" is true for the overhead camera; say "1 cm (2 cm lights)". No edge-dependent term found in freeroam_core/run_plan.
- M6 0.014 vs 0.005 per degree (e02-oct25-logs-doe.md:26; backing-unc.md:25); false flags "3 in 4.8 healthy hours at 3, none at 5" (logs-doe.md:34) - "went to zero" is correct only for the MIN_N change on a fresh file.
- M7 "93 %, 0 wrong of 234" = e02-oct25-discovery-replay.md:32,95 (378/407 streams named right; 234 labels accepted, 0 wrong; held-out replay on logs). start_job.py has no feel-out call; table length and tag height are config values (start_job.py:560-561; HOME_BOARD freeroam.py:52).
- Sec. 4 counts: edge ~10 rows, heading 5, re-plan 4, swaps 4 match C-insights.md:300-396. "2 s before correcting" and "doesn't widen uncertainty": observe.py:579-580, 600-621; freeroam_core.py:893-899 (`uncertainty_cm` takes camera age and lost-bubble only).
- Sec. 5: item 1 (home.json:17 `"on_problem": "stop"`, sites.py POLICY_DEFAULTS, run_plan.py:2309); item 2 OUT_SILENT_S; item 5 battery floors engine_checks.py:29 (25) vs freeroam_core.py:110 (10); item 8 calib_run.py:2777 "stay awake" vs start_job.py:38 `sleep_when_done` True; item 9, 10, 11 match B-rules O3-O16. Calib_run "sleep after 30 min" wording not verified in code (only "stay awake" default).
- "714 commits" correct (`git log | wc -l` = 714; 725 counting remote branches).

## OMITTED by the report that the slices found (worth adding)

- C-insights "Lost insight" and "Undone" items not in the report: front-sensor phantom-stop fix status (fault 5, fixed), WiFi silence / the ping log (fault 7) - the ping log is in "smaller" but the fault table drops faults 5-8 (WiFi silence, done-without-moving). Report table lists 5 faults, C lists 8.
- A-tools: E02-only features still started from the Pi start page `tools/pi/launcher.py` (E02 runner as a parts bin, "possibly outdated (I)") appears only as a decision line, not a finding.
- B-rules: Oct 29 rule 10 (room rules 25/30 -> 20/25) never answered; 9 contradictions (report cites only a few of them in section 5); ~20 chosen numbers with no DOE row (N14: e05-e07 numbers) - report mentions only the last as one line.
- D-chat: no conflict found; but D's "PARTLY" on the possible-locations circle (Nov 5) is exactly the report's fault 1 and is flagged "no fix committed (HEAD is still Nov 6)" - the report does not say the fix is not built.
- The report cites slice reports at `docs/audits/2026-11-07/`; that folder does not exist in the clone (only the report file, untracked). Add the slices or change the pointer.
