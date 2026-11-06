# Home free roam: fr1

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.1 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. **Simulator run** (tools/sim/fleet_sim.py), not the robots. Seed 32856.

Motion style: **curves** (5 Nov 16:06: curves = robots on firmware 2026-11-05c drive continuous curving pieces from a menu of curvature x length x speed and never stop on purpose; robots on older firmware, or the stopturn choice, stop, turn on the spot and drive; the menu chart below counts the stop-turn moves only).

## Data

**The trophy: moving at the same time** (6 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 99.9 % (6 min) | 99.9 % (6 min) |
| all 3 moving | 97.4 % (6 min) | 96.8 % (6 min) |
| every robot still in the run moving | 97.4 % | 96.8 % |

| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |
|---|---|---|---|---|---|
| Center | 6 | 97.4 % | 97.4 % | 9.0 s | 1 |
| Left | 6 | 99.9 % | 99.9 % | 0.5 s | 0 |
| Right | 6 | 99.9 % | 99.3 % | 0.5 s | 0 |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Never stops** (motion style: curves; speed > 0 = the robot's own wheels show 1 cm/s or more over 0.6 s, every 0.5 s, motion.csv; speed-zero moments and why: zeros.csv).

| robot | style | speed > 0 | speed-zero moments | per hour | the main reasons |
|---|---|---|---|---|---|
| Center | curve | 97.4 % | 2 | 20 | the start 1, trophy break 1 |
| Left | curve | 99.9 % | 1 | 10 | the start 1 |
| Right | curve | 99.3 % | 2 | 20 | the start 1, find-me wiggle 1 |

Speed > 0 at the same time: at least 2 robots 99.9 %, all 3 96.8 % of the time.

**Moves**: 298 (298 finished). By kind: menu 229, free 34, loop 26, off-menu 4, escape 4, wiggle 1.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (0 of 112 cells used; fewest 0, most 0).

**Fallbacks** (no menu move fitted; fallbacks.csv): loop 26, escape curve 6, off-menu curve 4, find-me wiggle 1.

**Reflex events** (reflex.csv): speed 2; outcomes: kept 2. Full stops (trophy breaks, breaks.csv): 1: Center lost near an edge: stands until seen (9 s).

| id | time s | robot | other | change | predicted before | after | closest seen | method | outcome |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1793932903.26 | Center | Left | speed x1.35 | 15.69 | 21.79 | 24.64 | fspd | kept |
| 2 | 1793932903.36 | Left | Center | speed x0.6 | 15.67 | 22.22 | 24.64 | fspd | kept |

![reflex](chart_reflex.png)

Each mark is one time the reflex changed a robot's speed or direction; marks above the dashed line never let the bodies' circles touch.

**Camera timing** (camera.csv, measured during the run): fixes arrive 0.18 s after the picture (median; 90 % within 0.20 s); a robot counts as lost when no fix newer than 0.90 s; the camera's clock matches this one (alignment -0.08 s, inside the deadband).

**Lost by the camera** (lost.csv): 3 episodes, 38 s in all; the odometry's jump when seen again: median 1.0 cm, largest 1.1 cm; 3 of 3 inside the bubble.

![lost](chart_lost.png)

Each dot is one time the camera lost a robot: how far it drove unseen, and how far its own idea of where it was turned out to be off; dots under the line were inside the safety bubble's straight-line rule.

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: still roaming at the end. The run ended: the time cap (0.1 h).

## Conclusion

- At least two robots moved at the same time for 6 minutes (99.9 % of the run); all three for 6 minutes (97.4 %), counting turns on the spot; driving alone: 99.9 % and 96.8 %.
- Full stops (trophy breaks): 1. Reflex changes: 2 (speed 2, steer 0, new move 0).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
