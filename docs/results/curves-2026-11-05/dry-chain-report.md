# Home free roam: freeroam-curves-dry1

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.1 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. **Simulator run** (tools/sim/fleet_sim.py), not the robots. Seed 5.

Motion style: **curves** (5 Nov 16:06: curves = robots on firmware 2026-11-05c drive continuous curving pieces from a menu of curvature x length x speed and never stop on purpose; robots on older firmware, or the stopturn choice, stop, turn on the spot and drive; the menu chart below counts the stop-turn moves only).

## Data

**The trophy: moving at the same time** (4 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 99.8 % (4 min) | 98.6 % (4 min) |
| all 3 moving | 96.5 % (3 min) | 68.7 % (2 min) |
| every robot still in the run moving | 96.5 % | 68.7 % |

| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |
|---|---|---|---|---|---|
| Center | 4 | 98.1 % | 97.2 % | 3.0 s | 1 |
| Left | 4 | 98.1 % | 98.1 % | 3.5 s | 1 |
| Right | 4 | 99.8 % | 71.7 % | 0.5 s | 0 |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Never stops** (motion style: curves; speed > 0 = the robot's own wheels show 1 cm/s or more over 0.6 s, every 0.5 s, motion.csv; speed-zero moments and why: zeros.csv).

| robot | style | speed > 0 | speed-zero moments | per hour | the main reasons |
|---|---|---|---|---|---|
| Center | curve | 97.2 % | 2 | 33 | the start 1, find-me wiggle 1 |
| Left | curve | 98.1 % | 2 | 33 | the start 1, trophy break 1 |
| Right | stopturn | 71.7 % | 38 | 634 | stop-turn 38 |

Speed > 0 at the same time: at least 2 robots 98.6 %, all 3 68.7 % of the time.

**Moves**: 163 (163 finished). By kind: menu 138, free 19, off-menu 2, loop 2, escape 1, wiggle 1.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (41 of 112 cells used; fewest 0, most 2).

**Fallbacks** (no menu move fitted; fallbacks.csv): loop 6, off-menu curve 1, escape 1, off-menu 1, find-me wiggle 1.

**Reflex events** (reflex.csv): none; outcomes: -. Full stops (trophy breaks, breaks.csv): 2: Left lost near an edge: stands until seen (4 s); Center lost near an edge: stands until seen (2 s).

**Camera timing** (camera.csv, measured during the run): fixes arrive 0.48 s after the picture (median; 90 % within 0.50 s); a robot counts as lost when no fix newer than 1.20 s; the camera's clock matches this one (alignment +0.00 s, inside the deadband).

**Lost by the camera** (lost.csv): 2 episodes, 31 s in all; the odometry's jump when seen again: median 1.6 cm, largest 1.6 cm; 2 of 2 inside the bubble.

![lost](chart_lost.png)

Each dot is one time the camera lost a robot: how far it drove unseen, and how far its own idea of where it was turned out to be off; dots under the line were inside the safety bubble's straight-line rule.

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: still roaming at the end. The run ended: the time cap (0.1 h).

## Conclusion

- At least two robots moved at the same time for 4 minutes (99.8 % of the run); all three for 3 minutes (96.5 %), counting turns on the spot; driving alone: 98.6 % and 68.7 %.
- Full stops (trophy breaks): 2. Reflex changes: 0 (speed 0, steer 0, new move 0).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
