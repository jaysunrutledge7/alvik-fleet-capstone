# D. Chat decisions vs the repo (audit slice D)

**Premise.** Every robot/software decision Nick stated in chat, checked against the capstone repo at `peek` (HEAD 85f5e99, Nov 6; no commits since).

**Sources and their reach (important):**
- The main transcript (`fff6dc9c….jsonl`) holds only **Nov 6 02:47Z to Nov 7 12:48Z**. Nov 5 is reachable only through the two compaction summaries inside it, which paraphrase Nick's messages.
- The 45 subagent briefs (`subagents/*.jsonl`, first message each) relay Nick's decisions from **Oct 28 to Nov 6**, often quoting him with times. They are Claude's relay, not Nick's own words.
- Nothing before Oct 28 is reachable except the memory notes.
- The 41 pasted-text attachments are terminal output, not decisions.

**Legend:**
- **V** = verified in code or docs (file:line cited), or in Nick's own pasted output.
- **I** = inferred (from a build report's own claim, a grep that found nothing, or a summary's paraphrase).
- Dates are UTC as logged. Nick's local time is UTC-4 before Nov 2 and UTC-5 after.

---

## 1. Decision table, by topic

### Motion detection and sensor fusion

| Date | Nick's decision / request | Status | Evidence |
|---|---|---|---|
| Nov 5 (summary) | "Is it also using our motion detecting tools? … note it for later" (school run) | **LOST** | V: `tag_bridge.py` uses `e02_camera` only to load camera models (line 532); there is no motion or difference tracking at school. The home `cam_bridge.py` uses `e02_multitrack` (floor memory). Deferred to "phase 3" in e06 and e07 (e07 report line 170); never built. |
| Nov 5 (summary) | "Why wasn't the program written with motion detection as part of its data fusion?" → rule "Observe, then trust": fuse every signal (tag pose, motion in picture, odometry, gyro, accel, front, line) | **PARTLY** | V: the rule is in CLAUDE.md. Accel, gyro, front and line feed only a move's *outcome* (`observe.move_outcome`, `freeroam_core.py:1350-1386`, `run_plan.py:546-570`), not the position. Motion in the picture is fused nowhere. Tracking is one source per site: `sites.py:15-27` "lights" OR "tags" OR "none". `tools/fusion` (EKF / factor graph) is called by no current pipeline (grep of tools/fleet, tools/pi and run-fleet-pi.sh found no import). |
| Nov 5 | A disagreement between sources is information, not a reason to drop one | **PARTLY** | V: `freeroam.py:618` drops a robot with "no heading" from the whole run. That happened live Nov 7 (Right excluded: "Free roam: Center, Left"; the assistant's reply 02:08Z traced it to a camera jump, not the wheels). |

### Observe, then trust: self-checks

| Date | Decision | Status | Evidence |
|---|---|---|---|
| Nov 5 20:28 (brief e06) | Every run type finds the tag heading by its first drive and keeps checking it | **PARTLY** | V: `HeadingWatch` is used in `run_plan.py:1018` and `calib_run.py:2283`, but not in `freeroam_core.py`. Free roam uses the ladder's heading crawl and parks a robot that fails it (`freeroam.py:608-620`). |
| Nov 5 | Single plugged camera = overhead, whatever its name | IMPLEMENTED | V: `observe.pick_cameras` (`observe.py:395`), `camnames.py:188` |
| Nov 5 | Tag detection retries a downscaled pass | IMPLEMENTED | V: `observe.detect_retry` (`observe.py:467`) |
| Nov 5 | Tag heights and scale solved by the program; look for any chance to fix scale | **NOT IMPLEMENTED** | I (e06/e07 reports): the tag height is still a typed starting value (6.19 cm, school site). e07 fixed *table* scale from the corner tiles, not tag height. "Self-solved tag height" is listed as phase 3. |
| Nov 5 | Cameras win over typed measurements | IMPLEMENTED with a limit | V: `measure_trust` auto (`site_fit.py`, `site_detect.py`). I (e07 report): the cameras win only when the corner tiles are seen; the school table *length* is still typed (far tiles unreadable). |
| Nov 5 | Keep refusing old firmware; keep the `--min-firmware` override | IMPLEMENTED | V: `calibration.min_firmware` in the school site; `run_plan.py`, `calib_run.py`, `wiggle_bind.py`, `ladder.py` |
| Nov 5 | The simulator deliberately breaks guesses | IMPLEMENTED | I: e06/e07 rehearsal tables (tags turned, dark band, clock skew, restart) |

### Edge safety

| Date | Decision | Status | Evidence |
|---|---|---|---|
| Nov 5 | Possible-locations circle must never overlap the workspace perimeter; home board has no walls or rim | **PARTLY** | V: the dead-reckoning bubble is in `freeroam_core.py` (`uncertainty_cm`, `PoseAgree`). Broken in practice Nov 7: the assistant's log analysis (12:36Z) found planned move ends 11.6 / 12.5 / 15.4 cm from the edge against the stated 16 cm. The camera pose is adopted only after 2 s of disagreement, so a 200° curve was planned from a pose 4.6 cm wrong. Two robots tipped. No fix committed (HEAD is still Nov 6). |
| Nov 5 | Adapted rule 3': no single source decides a move's outcome; never repeat a failed action unchanged | IMPLEMENTED | V: `observe.move_outcome`, `FailMemory`, used in `freeroam_core` and `run_plan` |
| Nov 5 | Adapted rule 4': swept space shown safe (breadcrumb reverse, front, camera, look-before-backing, otherwise hold) | **PARTLY** | V: `back_out_plan` and `Trail` are used in freeroam, run_plan and calib. Nov 7: a tilted Center "stands until it can back off" and its one backward drive "did NOT move" (Nick's paste, 02:26Z). The tilt stop (firmware 11-05d) has no exit when the planner finds no room. |
| Nov 5 | Tilt/drop safety in every run; no tilt bench test if it means catching robots | IMPLEMENTED | V: `TiltWatch` in all three pipelines; firmware 11-05d `gTiltBackOK` |
| Nov 6 Q3 | Re-pick standing-tilt (2.5°, 2 s) and accel (0.03 g) from real logs, unless that risks false starts | **NOT YET** (waiting on data) | V: `observe.py:583` `STAND_TILT_DEG = 2.5`; `freeroam_core.py:1350` `ACCEL_JOLT_G = 0.03`, unchanged. The Nov 6 and Nov 7 home runs are the first real tilt data. |
| Nov 7 Q4 | Operate only inside proven workspace (limit to what's visible, or send a robot ahead with cliff detection + odometry corrections to map unseen areas) | **NOT IMPLEMENTED** | V: no code (grep "proven" finds nothing). Left-behind tool that fits: Feel-out-the-table (`feel_table.py`, `feel_job.py`, Nov 2), which crawls robots to the edges. It is only reachable through `mission_run`; the e07 Start screen (`start_job.py`) never calls it (I). |

### Motion style, free roam and "no stops"

| Date | Decision | Status | Evidence |
|---|---|---|---|
| Oct 30 (memory, briefs) | No stops or robots sitting out: warn and log instead | **PARTLY, and contradicted by home** | V: `sites/home.json:17` `"on_problem": "stop"`, shown in Nick's own paste Nov 7 01:48Z ("on a problem: stop the run (as before)"). School is `keep_going` (`school-conference.json:108`). |
| Nov 5 | Directional solutions instead of stops; continuous simultaneous movement = "the trophy" | IMPLEMENTED (home) | V: firmware 11-05c `R` pieces, `farc`, dodge (home_lab.ino changelog); `freeroam_report.chart_reflex`. Nov 5: 99% moving for 160 min. |
| Nov 5 16:06 | Curves "where velocity never drops to 0"; reverse only if absolutely necessary; minimum speed chosen by simulator | IMPLEMENTED | V: `freeroam_core.py:141` (minimum-speed evidence note) |
| Nov 5 | Both motion styles always available; calibration for smooth turns | **PARTLY** | V: e05 report line 182 says the arc calibration phase was "not built as asked"; arc scales are learned during the free roam instead. Its question (e05:228) was never answered. |
| Nov 5 20:28 | Motion per run type: curves for free roam; straight lines with turns on the spot for points and experiments | IMPLEMENTED | I: e07 report; `start_job.py:9` |
| Nov 5 | Balanced menu with speed in the menu; reservations; speed adjustments documented; dead reckoning instead of freeze-on-lost; battery to 10%; keeps running after logout; uploads | IMPLEMENTED | V: `freeroam_core.py` (Traj reservations, bubble), `freeroam_report.py`, run-fleet-pi.sh `freeroam*`. Nick's paste Nov 7 02:01Z confirms "each robot parks at 10 %". |
| Nov 5 | Front stops become discovered keep-outs | IMPLEMENTED | V: `freeroam_core.py:960`. Keep-outs last "for the rest of the run". |
| Nov 7 Q1 | "Shouldn't our program be able to implement free roam anywhere?" | **NOT IMPLEMENTED** | V: `freeroam.py:630` always launches `cam_bridge.py` (lights). The school "free roam" is a 10-minute wander between points (`start_job.py:30` `ROAM_MIN = 10.0`). |
| Nov 5 | System starts with however many robots; detect and label them by calling one out and seeing which moves | **PARTLY** | V: wiggle binding exists on tag sites (`wiggle_bind.py`). Home free roam uses the site's fixed robot/IP list plus lights (Nov 7 status lists Left/Center/Right from `home.json`); no wiggle there (`freeroam.py:262` mentions a wiggle only for re-follow). |

### Mission Control

| Date | Decision | Status | Evidence |
|---|---|---|---|
| Nov 5 | One-button runs: Calibrate, Free roam, Visit points, Experiment routes | IMPLEMENTED | V: `start_job.py:28-29` |
| Nov 5 | One Mission Control that auto-detects home, school or a new area | IMPLEMENTED | V: `site_detect.py`; e07 commit 5c6f8be |
| Nov 5 | Corner selection only if absolutely necessary, then obvious and simple | **PARTLY** | V: `home.json:15` `corner_clicks: true`. I (e07): home corners are not found automatically; the saved clicks are reused. |
| Nov 6 Q1 | Never type values; typed values under Advanced; mapping automatic | IMPLEMENTED (mostly) | V: e07 Start screen. School table length and tag height are still typed (see above). |
| Nov 6 Q2 | Reserve a lost robot's recovery path in the other robots' plans | IMPLEMENTED | V: `run_plan.py:1481-1495` |
| Nov 6 Q4 | Missions use each robot's stop lead | IMPLEMENTED | V: `stop_lead` in `mission_run.py`, `run_plan.py`, `start_job.py` |
| Nov 6 Q5 | Extra robots named Charlie, Delta, Echo within the run | IMPLEMENTED | I: e07 report |
| Nov 6 Q6 | Surface marks: log only, don't act | IMPLEMENTED | V: `run_plan.py:2636` `surface_marks` default "log", `acted=False` |
| Nov 6 Q7 | Sleep right away after a run, with an on/off option | IMPLEMENTED in Mission Control; **stale elsewhere** | V: `start_job.py:504-509` (`sleep_when_done`, default True). See superseded item 1. |
| Nov 5 | Start runs without PowerShell or Claude (Oct 26 memory: from the live lab page) | **PARTLY** | V: Mission Control and :8091 launcher exist, but the Nov 6/7 runs were all started over ssh with `run-fleet-pi.sh`. Nick asked to run "quickly, not through mission control" (Nov 6 11:11Z), so Mission Control does not yet cover home free roam conveniently (I). |
| Nov 7 (pending) | Q5 smaller defaults: sleep with no timer, e1-e4 cycling, 10-minute school roam, practice mode behind a blue bar | **AWAITING NICK** | V: asked at 12:36Z, no answer in the transcript |

### School site, Jay-Sun and data

| Date | Decision | Status | Evidence |
|---|---|---|---|
| Nov 5 | Park spots P09, P01, P06 | IMPLEMENTED | V: `park_spots` in the school site |
| Nov 5 | Attended-only at school; second camera or Sodhi's OK before the half table | IMPLEMENTED | V: `--attended` in `mission_run`, `calib_run`, `start_job`; half-table tick in `mission_control.py` |
| Nov 3 / Nov 5 | Raw odometry never camera-corrected; sync wiggle; per-frame camera track; Jay bundle | IMPLEMENTED | V: run-fleet-pi.sh `jay-bundle`; I: e02-nov03-experiments report |
| Nov 4 | RFID tools off unless Jay asks; RFID never an input | IMPLEMENTED | V: `tools/rfid/rfid_gate.py:1,29` |
| Nov 5 | Lighting held fixed and logged | IMPLEMENTED | V: `tag_bridge.py:343` `LIGHT_EVERY_S` |
| Nov 5 | Jay-Sun able to watch the camera from his laptop ("yes but maybe not now") | NOT IMPLEMENTED | I: no sharing feature found; listed as "noted for later" |
| Nov 5 | Save calibration on the robot; first calibration from defaults; rename off at school; pin robot↔tag; OTA through the Pi | IMPLEMENTED | V: firmware `save`; `ota_push.py` (commit dbc351d per summary); `park_spots` and pins tests (`test_e04_baseline_pins.py`) |
| Nov 5 | Simpler upload check; auto-clean at home | IMPLEMENTED | V: `uploads` and `clean-uploaded` commands; `clean_uploaded.py:5` |
| Nov 5 | "Is there a way to check how this run's latency has been affected?" | IMPLEMENTED (free roam) | V: `freeroam_report.py:354-371` (camera delay median and p90) |
| Nov 5 | Full video for the continuous run (GIF asked, "don't build now"); USB stick at school and its latency effect | **PARTLY** | V: `full-video` option exists. GIF and school USB recording were not built or answered (I). |

### Earlier decisions relayed in the briefs (Oct 28 to Nov 3), checked only where still relevant

| Date | Decision | Status | Evidence |
|---|---|---|---|
| Oct 29-30 | Monday-demo mission: lanes 13 cm, turn pause, step-away, 8 cm lock, mirroring by speed matching, Chicken pass | **SITUATION GONE** (Nov 2 demo) | V: "monday" appears 13× in run-fleet-pi.sh, 35× in missions.py, 18× in mission_control.py. Candidate to retire or switch off. |
| Oct 29 | Facing check, start straightening, blind-strip cap 40 cm, 3 readings in a row, fading obstacles, silent robot rejoins, floor check | IMPLEMENTED | I (lights-site path) |
| Oct 30 | User only clicks corners 1-4; robots self-correct; board size measured | **SUPERSEDED** Nov 5/6 (no clicks unless necessary) | V: still the home default (`home.json:15`) |
| Nov 2 21:01 | "Pillows … okay if they drop … finish the run with the highest-confidence position"; camera trusted over sensors inside the camera barrier | IMPLEMENTED (school) | V: `on_problem keep_going`, `feel_out_trust` in the school site |
| Nov 2 | Feel out the table on a new table | **ORPHANED from the new flow** | I: `start_job.py` has no feel-out call; still in `mission_run.py:213-219` |
| Nov 3 | Tag height and tilt measured during auto-calibration | **PARTLY** | I: roof_cal / site_fit estimate it, but e06/e07 still describe the height as a typed starting value. A fitted height is only possible where two cameras see the tag. |
| Nov 3 | Run with or without tags (tags / lights / none) | IMPLEMENTED | V: `sites.py:27` `run_tracking` |

---

## 2. LOST or not-incorporated items, ranked (most important first)

1. **Motion detection and true multi-signal fusion** (V). The school tracker has no motion-in-picture. The home floor-memory tracker is not shared with school. No pipeline fuses the robots' own sensors into position (`tools/fusion` is orphaned). This is Nick's headline objective and the Nov 5 rule. It was deferred twice (e06, e07).
2. **Free-roam edge safety** (V, Nov 7 incident). It needs four changes. The free roam currently adopts the camera pose only after 2 s of disagreement, plans curves that swing past the 16 cm margin, and has no exit from a tilt stop. Robots went off the board in both Nov 6 runs; the last good run was Nov 5 on firmware 10-28a. Nothing has been committed since.
   - Plan each move from the camera pose whenever it is fresh.
   - Actually keep the 16 cm margin, and keep wide curves away from the edges.
   - Back a tilted robot out in the direction the camera confirms is inward.
   - Treat a robot whose backward drive "did NOT move" as a tilt to recover from.
3. **Free roam at any site** (Nov 7 Q1, V not built). Free roam is hard-wired to `cam_bridge`, the lights tracker.
4. **Proven-workspace operation** (Nov 7 Q4, V not built). Feel-out-the-table (Nov 2) and the virtual-border and explore tools are the existing pieces to reuse.
5. **Self-solved tag height and scale** (Nov 5, I not built). The 6.19 cm and the school table length are still typed. This is also against "Discover, don't pre-map".
6. **Heading check, continuous watch and "adapt, don't exclude" in free roam** (V). A robot failing the heading crawl is parked for the whole run (Right, Nov 7). `HeadingWatch` is not used in free roam.
7. **Seam / surface-feature discovery** (open since e06, V). The home seam-tape rehearsal still ends "out of re-plans". Discovery of slippery patches or bumps from odometry-vs-camera data is not built, though the Nov 5 4-hour analysis computed the spatial odometry error.
8. **Arc (curve) calibration phase** (Nov 5, V replaced by in-run learning; question unanswered).
9. **Wiggle-based identification at home** (Nov 5 "call out to a robot, see which moves"; I). Home relies on the fixed list plus lights.
10. **Re-pick tilt and accel thresholds from real logs** (Nov 6 Q3). The Nov 6 home runs now give real tilt data.
11. **Home corners found automatically** (e07 not done; V `corner_clicks: true`).
12. **e07 open faults** (V, e07 report "Left for phase 3"). These are not lost, but they wait on phase 3.
    - 6.6 cm robot gap when driving blind with the half table.
    - A robot with no room for its heading check.
    - A restarted robot whose heading stays 20° off.
    - The recalibration roll that brought a body to 4.9 cm from an edge.
    - The practice simulator left running.
13. **Recorder start "pictures missing"** (V, Nick's pastes Nov 6 11:16Z and Nov 7 02:01Z). It recurs at every free-roam start; the Oct 28 fix (start_recorder checks pictures) restarts it rather than preventing it.
14. **Small items noted for later and never picked up** (I).
    - Live camera stream for Jay-Sun.
    - GIF of a run.
    - School USB-stick recording and its latency effect.
    - SSH keys.
    - 4–7 cm/s minimum-speed comparison.
    - Camera-stream speed-up and name mix-ups.

---

## 3. SUPERSEDED decisions still in code or docs

1. **"Robots stay awake at the end; sleep after 30 minutes idle"** (e06 brief, Nick 5 Nov 20:28) was superseded by **Nov 6 10:19Z Q7: "sleep right away, with an on/off option"**. It survives in three places:
   - V: `docs/how-to/school-calibration-tonight.md:27-28` still says robots stay awake and sleep after 30 minutes.
   - V: `calib_run.py:2457-2461, 2777` still defaults to staying awake when run from `./run-fleet-pi.sh calib` (only Mission Control's `start_job` sleeps right away).
   - V: `observe.py:58` `IDLE_SLEEP_MIN = 30.0` (kept as a switch; that part is fine).
2. **Home `on_problem: "stop"`** (Nov 3 build choice: "home stops on a problem, as before") conflicts with **Oct 30 "no stops, warn and log"** and **Nov 5 "directional solutions instead of stops"**. V: `sites/home.json:16-17`; Nick saw it printed Nov 7 01:48Z.
3. **A robot excluded from free roam for a failed heading crawl** (`freeroam.py:618-619`, the ladder's park rule from Oct) conflicts with **Oct 30 "no robots sitting out"** and **Nov 5 "evidence wins: adapt, log, carry on"**. V, live Nov 7.
4. **Corner clicking as the home setup** (Oct 30 "user only clicks corners") was superseded by **Nov 5/6 "automatic mapping, corners only if absolutely necessary"**. V: `home.json:15`. It is kept as a switch per the keep-features rule, but it is still the *active* home method.
5. **Typed table length and tag height at school** were superseded by **Nov 5 "tag heights and scale solved by the program"**, **Nov 6 "never type values"** and **Nov 7 Q4 "proven workspace"**. V/I: `school-conference.json` `tags.heights_cm 6.19`; the e07 report says the length is still typed.
6. **e06 surface-mark behaviour (line sensors off for the rest of the run)** was superseded by **Nov 6 Q6 "log only"**. Code is updated (V `run_plan.py:2636`). The runbook line `school-calibration-tonight.md:510` ("that robot's drop check stays on") still describes the e06 wording (I; ambiguous rather than wrong).
7. **Fixed halves and lengthwise strips for Experiment routes** (Nov 3 Jay-Sun "swap halves") versus **Nov 5 "table sharing automatic from what's seen"**. Auto sharing is built for Visit points. Experiment routes still cut strips (e07 Q3; Nick: no preference). Leave it until Jay-Sun answers.
8. **Monday-demo mission machinery** (Oct 28-30: lanes, mirroring, Chicken pass, 8 cm lock): the situation ended Nov 2, but it is still wired into run-fleet-pi.sh `monday` and Mission Control (V counts above). This is a candidate for a switch-off, not deletion.

---

## 4. Memory notes now outdated or wrong (against `audit/memory-capstone.md`)

1. **"Camera: ML detector, no printed markers"** and **"no tape/markers on robots"**: superseded by AprilTags at school (Nov 2). The note says this partly. Home tracks lights. The ML detector (`tools/detector`) is used by neither pipeline (I, grep).
2. **"fusion by sliding-window factor graph + EKF, running on each robot"**: this describes a design, not the current system. The robots blend timestamped camera fixes, and `tools/fusion` is not wired in (I). Keep it as a goal, not a fact.
3. **"Oct 27: would much rather a robot stand still than collide or go over the edge"** and **"Oct 26: OK with robots going over the edge"** contradict each other. Both are overtaken by Oct 30 "no stops", Nov 2 "pillows, okay if they drop", and the **Nov 5 adapted rule 4'** plus "circle never overlaps the perimeter".
4. **"Oct 30: setup needs nothing beyond clicking corners"**: superseded by Nov 5/6 (automatic mapping; corners only if absolutely necessary).
5. **"Oct 19: docks with nose switches as touch probes"**: superseded by Oct 25 "one attachment at the back with bumper switches". Both lines are still in memory.
6. **"Nov 6 answers: … operate only within proven workspace …; free roam should work at any site"**: these two were answered on **Nov 7 12:34Z** (Q1 and Q4 of phase 2's questions), not Nov 6. Only the date is wrong.

Decisions from the chat that are **missing from memory** (worth saving):
- Nov 5 adapted rules 3' and 4'.
- "The possible-locations circle never overlaps the perimeter"; the home board has no walls or rim.
- No tilt bench test if it means catching robots.
- Motion per run type (curves for free roam; straight lines with turns on the spot for points and experiments).
- Both motion styles always available; reverse only if absolutely necessary.
- "Continuous simultaneous movement is the trophy."
- Cameras win over typed values.
- Keep refusing old firmware.
- Jay-Sun constraints: attended-only, raw odometry never camera-corrected, lighting fixed, second camera or Sodhi's OK, one console.
- Nov 7 Q2: no home grid.
- The Nov 2 low-battery floor of 5% (DOE loop).

---

## 5. Sweeps

**Tests that may now be needed:**
- A simulator case reproducing Nov 7: a pose 4.6 cm wrong just before a wide curve near an edge, under the 2 s snap.
- A tilted robot at an edge with "no room to back away".
- A free-roam robot failing its heading crawl that should adapt, not be excluded.
- A runbook-vs-default consistency check (the sleep wording).
- Home `on_problem` behaviour under keep_going.

**Possible inaccuracies in this slice:**
- Nov 5 and earlier decisions come from Claude's compaction summaries and briefs, not Nick's verbatim words. Wording and dates may be off by minutes.
- "Not wired in" findings rest on import and subprocess greps; dynamic loading could be missed.
- Items marked I rely on build reports' own claims.
- The Nov 7 edge-incident cause is the assistant's log reading (first run only); the second run's logs had not uploaded.
- Chats before Oct 28 were not reachable, so earlier decisions are judged only through memory.
