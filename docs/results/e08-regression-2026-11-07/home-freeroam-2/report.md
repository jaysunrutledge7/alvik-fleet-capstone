# Home free roam: 20261107-102915-freeroam-sim

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.1 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. **Simulator run** (tools/sim/fleet_sim.py), not the robots. Seed 65356.

Motion style: **curves** (5 Nov 16:06: curves = robots on firmware 2026-11-05c drive continuous curving pieces from a menu of curvature x length x speed and never stop on purpose; robots on older firmware, or the stopturn choice, stop, turn on the spot and drive; the menu chart below counts the stop-turn moves only).

## Data

**The trophy: moving at the same time** (3 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 42.6 % (1 min) | 42.1 % (1 min) |
| all 3 moving | 0.0 % (0 min) | 0.0 % (0 min) |
| every robot still in the run moving | 98.1 % | 96.8 % |

| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |
|---|---|---|---|---|---|
| Center | 3 | 99.7 % | 99.7 % | 0.5 s | 0 |
| Left | 1 | 98.1 % | 96.8 % | 1.0 s | 0 |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Never stops** (motion style: curves; speed > 0 = the robot's own wheels show 1 cm/s or more over 0.6 s, every 0.5 s, motion.csv; speed-zero moments and why: zeros.csv).

| robot | style | speed > 0 | speed-zero moments | per hour | the main reasons |
|---|---|---|---|---|---|
| Center | curve | 99.7 % | 1 | 20 | the start 1 |
| Left | curve | 96.8 % | 2 | 40 | the start 1, trophy break 1 |

Speed > 0 at the same time: at least 2 robots 42.1 %, all 3 0.0 % of the time.

Curves' calibration found in the run (arcfit.csv): Left curvature x1.1784, length x0.9726 of what was asked (7 pieces).

**Moves**: 75 (75 finished). By kind: menu 63, free 7, loop 2, escape 2, park 1.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (0 of 112 cells used; fewest 0, most 0).

**Fallbacks** (no menu move fitted; fallbacks.csv): loop 4, escape curve 2.

**Reflex events** (reflex.csv): none; outcomes: -. Full stops (trophy breaks, breaks.csv): 1: Left lost near an edge: stands until seen (1 s).

**Camera timing** (camera.csv, measured during the run): fixes arrive 0.18 s after the picture (median; 90 % within 0.20 s); a robot counts as lost when no fix newer than 0.90 s; the camera's clock matches this one (alignment +0.00 s, inside the deadband).

**Lost by the camera** (lost.csv): 1 episodes, 13 s in all; the odometry's jump when seen again: median 1.0 cm, largest 1.0 cm; 1 of 1 inside the bubble.

![lost](chart_lost.png)

Each dot is one time the camera lost a robot: how far it drove unseen, and how far its own idea of where it was turned out to be off; dots under the line were inside the safety bubble's straight-line rule.

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: Left: parked at 105.6, 75.0 and stopped (battery 9 %). The run ended: the time cap (0.1 h).

## Conclusion

- At least two robots moved at the same time for 1 minutes (42.6 % of the run); all three for 0 minutes (0.0 %), counting turns on the spot; driving alone: 42.1 % and 0.0 %.
- Full stops (trophy breaks): 1. Reflex changes: 0 (speed 0, steer 0, new move 0).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
