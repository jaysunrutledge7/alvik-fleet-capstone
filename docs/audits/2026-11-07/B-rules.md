# B. Rules and decisions audit (MIRLT capstone, repo at 85f5e99, 6 Nov 07:27)

Read-only. Sources: CLAUDE.md and its 5 commits, README.md, tools/*/README.md, tools/fleet/DESIGN.md, docs/decisions,
docs/build-plans, docs/build-reports (Oct 29 rules audit through e07), docs/session-plans, docs/how-to,
docs/open-questions.md, docs/doe-backlog.md, docs/notes, docs/hardware, docs/pi-run.md, docs/summary, site files
(tools/fleet/sites/*.json, sites.py), and the 319 code comments that cite a dated Nick decision (list kept at
`audit/_b_nickrules.txt`). Judged against `audit/memory-capstone.md` (Nick's saved decisions).

**[V]** = verified in code or doc text. **[I]** = inferred (reasoned from dates or context, not proven by a run).

Baseline: the Oct 29 rules audit (`docs/build-reports/e02-oct29-rules-audit.md`, ~70 rules) was largely acted on. Recs
1-5, 7, 9 are built (3-in-a-row stop, unconfirmed obstacles expiring at 15 s, bridge "moved" by own motion, per-camera
lighting, silent robot rejoins, facing heading). Rec 10 (room rules 25/30 -> 20/25) was never answered or changed.
This audit does not repeat its table; it covers what changed after it (Oct 30 "no stops", Nov 2-6 builds, the three
Nov 5 CLAUDE.md rules, Nov 6 answers).

---

## 1. OUTDATED (retire or rewrite)

| # | Rule / decision | Where | Date, source | Why outdated | Action |
|---|---|---|---|---|---|
| O1 | Home "stops on a problem, as before" | `sites/home.json:17` (`on_problem: stop`), `sites.py:67` POLICY_DEFAULTS, `run_plan.py:5432` `--on-problem` default `stop`, `mission_run.py:1595` auto -> home stop, `mission_control.py:616` fallback `stop` | 3 Nov build ("home keeps exactly what it did before") | Nick 30 Oct 10:08: "no stops or robots sitting out ... rather a warning and a logged failure than a stop"; 2 Nov 21:01 keep going; CLAUDE.md 5 Nov "stop only when safety can't be kept". The default was kept at `stop` only to leave home unchanged. **[V]** | Make `keep_going` the default everywhere (home.json and POLICY_DEFAULTS); keep `stop` as the switch (Keep-features rule). |
| O2 | DOE loop parks a robot "out of the loop for good" unless `--keep-going` is passed | `doe_loop.py:627-640` (park), `:1684` flag is opt-in; `run-fleet-pi.sh doe ... [keep-going]` | 27-28 Oct | Same Oct 30 stance. `retry_outs` (Oct 30) re-tries parked robots each loop, so the docstring "for good" is also stale. **[V]** | Default keep-going on; keep normal mode as the switch; fix the docstring. |
| O3 | E02 runner's holds: "virtual border ... stops every robot", "lost robot stops every robot", session waits for Nick's letter (c/a/o/s) | `docs/pi-run.md:164-189`, `run-e02-pi.sh`, `tools/experiment/e02.py` | 24-26 Oct | "One system" decision 27 Oct: e02 runner gets no fixes; Oct 30 no stops / no human input. Still the first runbook a reader meets (README links pi-run). **[V]** | Mark pi-run.md's E02 sections as legacy (or move to an appendix); keep the runner (Keep-features) but not as the documented path. |
| O4 | "OK with robots going over the edge" to collect more data | memory 26 Oct; `tools/pi/session_plan.sh:34-48` (boards taken off for edge data) | 26 Oct | Overruled 27 Oct ("much rather stand still than go over the edge") and by the 4x4 ft home board with a drop on every side (Nov). **[I]** that nothing still relies on it; session_plan.sh is the e02-era plan. | Retire; say so in memory (see section 7). |
| O5 | "Rather stand still than collide or go over the edge" (as a general stance) | `docs/open-questions.md:202` (27 Oct 07:08) | 27 Oct | Narrowed by Oct 30 ("no stops") and CLAUDE.md 5 Nov: standing still is now only the last resort when safety cannot be kept; normal answers are slow / steer / back off / recover (freeroam curves, recovery reservation). **[V]** both texts. | Keep only as the safety floor wording of "Observe, then trust" (2); retire the 27 Oct form. |
| O6 | Monday (2 Nov) demo rules and their justifications | `docs/session-plans/2026-11-02-monday-run-sheet.*`, 2026-10-27/28/29 plans; open-questions "School demo (Monday)" Q18-Q78; `missions.py:979` "Monday's default"; `facing.py:45` TOL 15 deg justified by "Monday's timed mission refuses" a re-plan; `engine_checks.py:72` "narrow it this week" | 27-29 Oct | Demo did not happen (memory); Left/Right/Center at school were replaced by Jay-Sun's robots and roof tags; school tables differ (66 x 156 in, not 2 x 4 ft). **[V]** | Archive the session plans / run sheet; close or re-word Monday Qs in open-questions; re-justify facing TOL without the timed-mission argument (straighten auto now corrects instead of refusing). |
| O7 | open-questions.md as the live question list | `docs/open-questions.md` (last commit 30 Oct) | 27 Oct rule: "keep track of unanswered questions" | No update since 30 Oct; Nov 2-6 questions live only in build reports (e03-e07 each have their own "Questions for Nick"). Many listed Qs are answered by builds (Q52 L0 stop -> no stops; Q65 facing stop -> straighten; Q71 -> built; Q86 firmware long past). **[V]** | Re-sync: close answered, move Monday Qs out, pull e03-e07 open Qs in. |
| O8 | README goal and setup | `README.md` | 4-18 Oct | Says "8x8 tape-grid testbed", "overhead AprilTag camera", E1-E8 vs 5 cm target; replaced by the Oct 18 plug-and-play objective, Oct 19 self-calibration test plan, T0-T9 list. "Andrew's code is not copied ... only with his permission" predates Andrew's "free to use anything" (memory, Oct 20). Time tracking says TIME.md in the tower-defense-rpg repo, but this repo keeps `docs/time-tracker.md`. **[V]** | Rewrite README goal/layout; settle one time-tracker location. |
| O9 | "Made hard on purpose" claims: no markers (#4), one camera (#9), no clicks / no checkerboard (#10), two open edges (#8), robots find themselves against boards (#7) | `docs/summary/hard-on-purpose.md`, `docs/summary/outline.md:38`, `docs/build-plans/publication-plan.md:27,39,148` ("markerless") | 23-25 Oct | School now uses roof AprilTags (2 Nov), printed corner reference tags (3 Nov), typed sticker points; home clicks corners; home board has no boards/walls and four drops. Claims are true only for the home lights chain. **[V]** | Re-state per site ("markerless at home; markers welcome as extra data at school", matching the doc's own closing argument); fix the publication claim before any abstract. |
| O10 | Oct 19 decision: two docks near the middle, posts on three sides, nose switches as touch probes, landmarks clicked at setup | `docs/decisions/2026-10-19-self-calibration-test-plan.md` | 19 Oct | 25 Oct: one dock; one attachment at the back, calibrate by backing in. 30 Oct: setup = corners only. 5 Nov: discover, don't pre-map. **[V]** | Add a "superseded by" header to the decision. |
| O11 | DESIGN.md: board position "is a tape measurement", docking with the front distance sensor facing the board, laptop-side EKF | `tools/fleet/DESIGN.md` (18 Oct, never updated) | 18 Oct | Back attachment (25 Oct), boards gone at home, tags at school, observe-then-trust fusion. Message protocol section still useful. **[V]** text; **[I]** whether any code still follows the front-dock path. | Mark historical; point to data-formats.md for the protocol. |
| O12 | Experiment index: T0-T9 list as the plan | `docs/experiments/INDEX.md` (18 Oct) | 18 Oct | Oct 19 decision made self-calibration the core; nothing logged there since. **[V]** | Freeze with a pointer to build-reports, or update. |
| O13 | Firmware comment "Where the boards stand ... pressed against the side of the plywood" and explore / edge-find that push into boards | `firmware/home_lab/e02.h:659`, `tools/experiment/e02_explore.py` | 25 Oct | Home board now has no walls (memory, Nov). **[I]** explore is only reachable through the frozen e02 runner. | Keep (switched-off legacy), note that home has no boards. |
| O14 | Lights identity "fixed light combination, 6 lights" and blink codes as the who's-who | memory 19 Oct; `ladder.py:621`, tools/blink | 19-27 Oct | 5 Nov: wiggle first, then bind (wiggle_bind.py, every run with roof tags). At home (lights chain) the facing/lights path is still used. **[V]** wiggle built; **[I]** home still uses lights. | Keep lights as home's method and fallback; record wiggle as the rule. |
| O15 | Memory plan "then integrate components of the two programs" (RFID) | memory 2 Nov; `tools/rfid/*` | 2 Nov | 4 Nov: Jay-Sun asked for no models/reports/clock alignment from his data -> `rfid_gate.py` off; CLAUDE.md 5 Nov: RFID never an input. **[V]** | Memory should say RFID is off unless Jay-Sun asks (see section 7). |
| O16 | `docs/notes/open-items-2026-10-25.md`, `docs/summary/next-week-notes.md`, `docs/how-to/morning-nov03.md`, `school-experiments.md` ("Tonight ... 3 Nov") | those files | 24 Oct-4 Nov | One-day runbooks and to-do lists; superseded by the e07 Start screen how-to (`mission-control-start.md`). School-calibration-tonight already points there. **[V]** | Archive folder; keep one current runbook per site. |

## 2. CONTRADICTIONS (pick one)

| # | Rule A | Rule B | Notes | Recommend |
|---|---|---|---|---|
| C1 | "No robot is ever permanently OUT" (Nick 30 Oct 10:08; `doe_loop.py:588`); run_plan holds and rejoins a silent robot (29 Oct, `run_plan.py:1477`) | Free roam: silent 120 s = "out for the rest of the run" (`freeroam_core.py:105`, `:2441-2443`) | Same fleet, opposite rule. **[V]** | Free roam: hold as an obstacle, rejoin on a fresh camera fix (as run_plan). |
| C2 | Sensor-only obstacles are "unconfirmed" and expire after 15 s unless confirmed (Nick 29 Oct, `front_stop.py:60`, `run_plan.py:1535`) | Free roam: every felt point (front stop, edge guard) is a keep-out "for the run" (`freeroam_core.py:458`, KEEP_R 2 cm); trip spots are keep-outs (`observe.py:507`, `trip_keepout`) with no expiry | The 29 Oct reason (phantoms accumulating and ending runs) applies more to a 4 h unattended roam. The Nov 5 home run had 515 front stops in a row at one spot. **[V]** code; **[I]** effect on long runs. | One rule: unconfirmed expire, confirmed (camera / second moving stop / drop-trip) stay; real drops stay permanent. |
| C3 | "THE battery floor (one source of truth)": 25 % (`engine_checks.py:29,37`) | Free roam drives to 10 % (`freeroam_core.py:110`, "Nick") | Both cite Nick (25 Oct ladder sag at 20 %; 5 Nov free roam). **[V]** | Fold free roam's 10 % into `battery_floor()` as a per-run-type value so the "one source" claim is true. Worth a logs check: did any robot sag below 15 % on 5 Nov? |
| C4 | Edge room 17 cm "everywhere in the mission" (Nick 28 Oct 21:27; `missions.py:44,56`; `production.py:47`; site default `edge_in_cm` 17) | School `edge_in_cm` 13 (chosen, `school-conference.json` `_note_edge`); free roam `EDGE_CC` 16 (chosen); new-area runs 17 with points 30 cm in (e07); DOE/ladder 25 cm (Nick 27 Oct; `ladder.py:69`, `doe_loop.py:71`); calib room 12/21 (geometry) | Six edge distances for one physical question ("how close may the centre get to a drop"). None derived from the robots' own trip data. **[V]** | One derived rule: half diagonal + the robot's current position uncertainty + a measured stop distance, per robot and surface (CLAUDE.md "discover"); keep the old numbers as switches. |
| C5 | Feel-out trusts the camera over the edge sensors on AprilTag sites (Nick 2 Nov 21:06) | `POLICY_DEFAULTS feel_out_trust: sensor` and home.json `sensor` | Nick's words were conditional ("if we're using AprilTags"), so home = sensor is defensible; but "Observe, then trust" (3) says fuse both and treat disagreement as information. **[V]** | Replace the either/or switch with fusion (log disagreement, weight by measured noise); keep the switch. |
| C6 | Nov 6 answer (memory): "re-pick tilt limits from real logs" | e07 report lines 15, 157: Nick's answer was "standing-tilt and accelerometer numbers left alone" | Two records of the same answer disagree. **[V]** both texts. | Ask Nick. Either way the 5 Nov 4 h home log (Center 3.4-4.1 deg for 64 min) is already on disk, so a logs-only re-pick can run without board time. |
| C7 | "Setup needs nothing beyond clicking corners" (30 Oct) | "Discover, don't pre-map" (5 Nov) and e07 "nothing typed"; home still `corner_clicks: true`, tracking `lights` (`home.json:13,15`) | Clicking corners is hand-mapping. e07 made clicks `auto` (only when no reference tile reads), but home has no tiles. **[V]** | Give home reference tiles (or derive the board from robot trips + camera); keep clicks as the fallback. |
| C8 | 8 cm safety net (Nick 27 Oct; `engine_checks.py:72`) | Free roam HARD_CC 13.9 cm / PLAN_CC 17.4 cm; Chicken "never under 8" | Not a true conflict (different layers), but the 8 cm net's "narrow it this week" (Q45) was never closed. **[V]** | Close Q45; state the net per layer. |
| C9 | School: robots drive only when Jay-Sun is in the room (`attended_only`, 5 Nov) | Nick 30 Oct: start and run whenever robots are on and on the board; long-term 24/7 | Site-specific, so it is a scope note, not an error. **[V]** | Keep; say in the Oct 30 rule "except where a site requires attendance". |

## 3. NOT IMPLEMENTED (or only partly)

| # | Decision | Date | Status | Where it should go |
|---|---|---|---|---|
| N1 | No stops by default (O1, O2) | 30 Oct | Built only as an option (school default). **[V]** | sites.py defaults, doe_loop default |
| N2 | Free roam works at any site | 6 Nov (memory) | School "free roam" is a wander through the mission planner with half-second stops; freeroam_core runs only on home's lights chain (e07 Q1). **[V]** | Port freeroam_core to the tag bridge (tag_bridge -> freeroam.py) |
| N3 | Operate only within the proven workspace; or send a robot ahead with cliff detection + odometry to map unseen areas | 6 Nov (memory) | First half built (readable-area learning, `allow_half_table`, `--view-x-max`). The "send a robot ahead to map unseen area" half not found. **[V]** first half; **[I]** second half absent (grep found nothing) | site_fit / experiments.py: an explore-the-unreadable step |
| N4 | Never type values; mapping automatic | 6 Nov | e07 moved typed values into Advanced and fits from the cameras, but school still reads typed table length (far tiles unreadable, e07 Q4), typed sticker points (`points.use: typed`), typed keep-outs (outlets), typed tag height seed 6.19 cm, typed antenna guesses. **[V]** | Points and keep-outs from the camera (tile-like detection of stickers/outlets) or robot discovery; typed only as seeds (CLAUDE.md allows typed as a stand-in) |
| N5 | Discover surface features (seams, bumps, slippery patches) | 5 Nov CLAUDE.md | Data logged (moves.csv own-vs-camera; surface marks "log"); no learner yet. School `safe_edge 2` was a hand fix for the seam tape (`school-conference.json` `_safe_edge_note` says so). **[V]** | A discovery pass over moves.csv / trip logs -> per-site surface map |
| N6 | Long-term: eliminate the virtual border | 25 Oct | Border still on in every run (`run_plan.py:83` 6 cm, `bd_on 1`; free roam layer 5). **[V]** | Planned only; nothing measures what the border still catches (Oct 29: it never fired). Track its fire count per run as the evidence to remove it |
| N7 | Calibration settings "Fresh / From scratch / New system from scratch" | 25 Oct | calib_run `baseline auto / defaults / saved` covers the first two. "New system from scratch" (unknown sensors, guessed and confirmed) exists only as a plan (`docs/build-plans/sensor-array-self-calibration.md`). **[V]** | Long-term build |
| N8 | Learned trust per sensor; swap a sensor for a bad one and have it caught | 24 Oct | Built for the e02 runner (`e02_trust.py`); the fleet code uses fixed evidence weights (`observe.py` move_outcome: wheels 1, accel 1.5, gyro 1.5, front 2, line 0.5, camera 3, chosen). **[V]** | observe.py: learn the weights from disagreement history (the e02_trust method) |
| N9 | Lift the home session time limit (work toward 24/7) | 26 Oct, 25 Oct | Free roam 4 h cap (`freeroam.py:696`), DOE `--hours 4`, one-button roam 10 min. Battery is the real limit now (robots park at 10 % and sleep). **[I]** the cap no longer protects anything the battery park does not. | Make the cap a switch, default off at home once the self-charging dock exists |
| N10 | Re-pick tilt limits from real logs (if C6 resolves that way) | 6 Nov | Not done: firmware tilt 5 deg / 300 ms, standing tilt 2.5 deg / 2 s still chosen (`observe.py:506-525`). Driving at 3.4 deg on a wheel-over-edge is still under the firmware stop. **[V]** | Logs-only analysis of 5 Nov 4 h run |
| N11 | DOE-loop room rules 25 / 30 -> 20 / 25 (Oct 29 rec 10, Q73) | 29 Oct | Unanswered, unchanged (`ladder.py:69`, `doe_loop.py:71,104`). **[V]** | Nick's call (they are his numbers); or replace with C4's derived rule |
| N12 | System handbook (PryMax-style) | 19 Oct | No handbook in docs/. **[I]** could live elsewhere | docs/handbook |
| N13 | Live lab shows the Kalman filter's corrections and weights | 21 Oct | Not checked in this slice. **[I]** | (slice for UI audit) |
| N14 | DOE backlog rows for e05-e07 chosen numbers | standing rule | Backlog's last section is 5 Nov free roam; e05 (K_BINS, LOOP_R, ARC_*), e06 (POSE_AGREE 5 cm / 2 s, STAND_TILT, BACK_OFF 10, TRIP_KEEPOUT 5, EDGE_MARGIN 1, evidence weights, FailMemory, NUDGE 6, TAG_PX_MIN 14, READ_OK 0.6), e07 (EDGE_PROFILE_CELLS, site_detect weights, GRID 30 / 30, ROAM_MIN 10) are listed only in the reports' sweeps. **[V]** (grep: 0 hits for e05/e06/e07 in the backlog) | Add rows to docs/doe-backlog.md |

## 4. CHOSEN NUMBERS (no derivation; DOE?)

Only numbers that gate safety or a run's outcome. All **[V]** in code with a "chosen" comment.

| Number | Value | Where | DOE? |
|---|---|---|---|
| Free roam end-spot edge margin EDGE_CC | 16 cm | `freeroam_core.py:50` | Yes (backlog row open): 14 / 16 / 20 vs edge trips; better: derive (C4) |
| School edge_in_cm | 13 cm | `school-conference.json` `_note_edge` | Yes; chosen so every sticker is reachable (P04 at 13.3) - a convenience, not safety |
| Edge room in missions | 17 cm | `missions.py:44,56`, `production.py:47` | Nick's number; lane margin vs closest body-to-edge DOE noted |
| DOE / ladder edge refuse, robot room | 25 / 30 cm | `ladder.py:69`, `doe_loop.py:71,104` | Nick's; Oct 29 suggested 20/25 (N11) |
| Virtual border | 6 cm | `run_plan.py:83`, `mission_run.py:1502` | Never fired in 236 drives (Oct 29); collect fire counts before deciding to remove (N6) |
| Tilt stop (firmware) | 5 deg, 300 ms | `params.h:155`, `observe.py:506` | Yes; logs show a hung robot at 3.4-4.1 deg below it (N10) |
| Standing tilt hazard | 2.5 deg, 2 s | `observe.py` (four protections) | Yes; Nick 5 Nov 21:38 declined a bench test - logs-only is possible |
| Own pose vs camera disagreement stop | 5 cm, 2 s | `observe.py` POSE_AGREE | Yes: 3 / 5 / 8 cm vs false stops (from logs) |
| Back-off after a trip | 10 cm | `observe.py` BACK_OFF_CM | Bench DOE noted |
| Trip keep-out | 5 cm disc 5 cm ahead, permanent | `observe.py` trip_keepout | Pair with C2's expiry decision |
| Unconfirmed obstacle expiry | 15 s | `front_stop.py:60` | Nick's "about 15 s"; simulator sweep with injected phantoms (Oct 29 rec 2) never run **[I]** |
| Silent-out in free roam | 120 s | `freeroam_core.py:105` | Policy, not DOE (C1) |
| Facing tolerance | 15 deg | `facing.py:45` | Justification is Monday-specific (O6); re-derive with straighten auto |
| Free roam reflex / plan gaps | 2.5 / 4 cm, 2 s horizon | `freeroam_core.py` | Yes, backlog rows open (sim) |
| Bubble cap (find-me) | 8 cm | `freeroam_core.py` BUBBLE_CAP | Yes, row open |
| Minimum avoiding speed | 5 cm/s | `freeroam_core.py` V_MIN_AVOID | Evidence-based (4 seeds); finer DOE suggested |
| Session caps | free roam 4 h, school roam 10 min, DOE 4 h | `freeroam.py:696`, `start_job.py` ROAM_MIN | Not a DOE: policy (N9) |
| Lost recovery max blind | 40 cm | `calib_run.py:98`, run_plan BLIND_MAX | Nick 29 Oct; the 0.07 cm/cm growth is measured |
| Uncalibrated edge allowance | 10 % (fitted 4 %) | calib_run SAFE_FRAC_UNCAL | Yes, noted |
| Evidence weights (move outcome) | wheels 1, accel 1.5, gyro 1.5, front 2, line 0.5, camera 3 | `observe.py` | Should be learned, not a DOE (N8) |
| Lost-robot points | reassign | `school-conference.json` `_note_keep_going` | "Claude's assumption, Nick did not answer" - a policy question for Nick |
| Frames part length (school) | 60 s | `school-conference.json` | Memory-driven; not a DOE |

## 5. CURRENT and implemented (brief)

All **[V]**:
- CLAUDE.md: screenshots per build; heat tracker per real session; keep features, switch them off (many examples:
  corner_clicks, side camera `use: false`, keep-outs `use: false`, old_firmware_save, lost-mode hold, sleep timer,
  stop-turn motion); observe-then-trust shared checks (`observe.py`), simulator breaks guesses (practice_breaks,
  e06/e07 rehearsals).
- RFID off unless Jay-Sun asks (`rfid_gate.py`, 4 Nov) and never an input (CLAUDE.md 5 Nov).
- Wiggle-first binding, pins as first guesses with a check wiggle (5 Nov); calibrate every robot found; saved on robot;
  min firmware 2026-11-05a refused below.
- School: one camera (side `use: false`), attended only, hourly sleep check-in, P16 missing said, park spots P09 / P01 /
  P06 (Nick 5 Nov 21:38), safe_edge 2 (seam tape), surface marks logged only (6 Nov), recovery path reserved (6 Nov),
  stop lead in missions (6 Nov), sleep right after a run (6 Nov, timer kept off).
- Oct 29 audit recs 1-5, 7, 9 (3 readings, floor check, Left never under 9 cm, unconfirmed obstacles in run_plan,
  bridge moved by own motion, per-camera lighting, rejoin, facing heading to crawl).
- Oct 30: straighten / auto start with warnings (`run_plan.py:5076`), retry parked robots (`doe_loop.py:588`), facing
  re-measure instead of giving up.
- Free roam: curves default with stop-turn kept (5 Nov 16:06 "both options always available"), 10 % battery park,
  never-stop trophy logging (zeros.csv).
- One attachment at the back, backing in (25 Oct), robots sleep (24-25 Oct), MIT licence.

## 6. What's next (proposed order)

1. Nick: C6 (tilt answer), O1/N1 (make keep_going the default at home), C1/C2 (free roam silent-out and permanent
   keep-outs), C4 (one derived edge rule), N11.
2. Logs-only, no board time (runnable without asking): tilt re-pick and keep-out accumulation from the 5 Nov 4 h home
   run; virtual-border fire count across all saved runs; battery sag below 15 % on 5 Nov (C3).
3. Docs clean-up (no code): open-questions re-sync, README, hard-on-purpose / publication "markerless" wording (the
   IISE abstract deadline in publication-plan.md is 8 Nov - tomorrow, unconfirmed), archive Monday plans and
   one-night runbooks, superseded headers on the Oct 19 decision and DESIGN.md, DOE backlog rows for e05-e07.

## 7. Memory items that look outdated (for the main session; source: audit/memory-capstone.md and the capstone memory note)

- 2 Nov "integrate components of the two programs" (RFID): superseded 4 Nov (rfid_gate) and 5 Nov (never an input).
- 26 Oct "OK with robots going over the edge": superseded by 27 Oct and the all-drop home board.
- Oct "no tape/markers on robots" and "ML detector with no printed markers": already flagged as superseded at school;
  home still runs markerless.
- 19 Oct "fixed light combination" identity: wiggle-first binding is the rule since 5 Nov.
- 6 Nov "re-pick tilt limits from real logs" disagrees with the e07 report ("left alone"): needs Nick's confirmation.

## Sweep 1: new tests that may now be necessary

- Free roam with expiring unconfirmed keep-outs vs permanent (fastsim with injected phantoms, 4 h).
- Free roam silent robot: hold-and-rejoin vs out at 120 s (fastsim, a robot dropping WiFi 3 min).
- Home missions under `keep_going` (rehearsal in the simulator) before flipping the default.
- One derived edge-margin rule vs the six fixed ones, replayed on every logged trip and closest-edge approach.
- Battery: lowest per-robot voltage/percentage behaviour between 10 and 25 % from the 5 Nov run.

## Sweep 2: possible inaccuracies in what this relied on

- Rule dates and authorship come from code comments and commit messages; many commits under Claude wrote "Nick"
  for decisions taken in chat. Not checked against the chats themselves.
- "Not found" items (N3 second half, N12, N13) rest on greps for likely names; a differently named implementation
  would be missed.
- Line numbers are at commit 85f5e99 and will drift.
- The memory file is a summary of Nick's notes, not his words; C6 may be a summarising difference rather than a real
  disagreement.
- I did not read every line of the 667-line doe-backlog or the 1,410-line data-formats.md; rule text inside them that
  is not in a "chosen" row may be missed.
- The firmware rules (border, edge, params) were checked only where the Oct 29 audit or Nov builds changed them.
