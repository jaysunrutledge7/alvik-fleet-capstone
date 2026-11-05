# Home free roam: 20261105-rehearsal2-freeroam-dryrun

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.4 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. **Simulator run** (tools/sim/fleet_sim.py), not the robots. Seed 12.

## Data

**The trophy: moving at the same time** (19 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 98.9 % (19 min) | 84.6 % (16 min) |
| all 3 moving | 95.4 % (18 min) | 43.7 % (8 min) |
| every robot still in the run moving | 97.7 % | 44.9 % |

| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |
|---|---|---|---|---|---|
| Center | 19 | 99.0 % | 76.3 % | 4.5 s | 2 |
| Left | 19 | 99.5 % | 76.4 % | 3.0 s | 1 |
| Right | 18 | 99.1 % | 76.8 % | 7.5 s | 1 |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Moves**: 526 (526 finished). By kind: menu 462, free 50, wiggle 9, park 3, escape 1, off-menu 1.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (112 of 112 cells used; fewest 1, most 7).

**Fallbacks** (no menu move fitted; fallbacks.csv): find-me wiggle 9, escape 1, off-menu 1.

**Reflex events** (reflex.csv): replan 1; outcomes: kept 1. Full stops (trophy breaks, breaks.csv): 1: Center lost near an edge: stands until seen (3 s).

| id | time s | robot | other | change | predicted before | after | closest seen | method | outcome |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 1793900851.16 | Left | Center | new move | 26.7 |  | 37.6 | halt+new move | kept |

![reflex](chart_reflex.png)

Each mark is one time the reflex changed a robot's speed or direction; marks above the dashed line never let the bodies' circles touch.

**Lost by the camera** (lost.csv): 10 episodes, 103 s in all; the odometry's jump when seen again: median 0.9 cm, largest 2.0 cm; 10 of 10 inside the bubble.

![lost](chart_lost.png)

Each dot is one time the camera lost a robot: how far it drove unseen, and how far its own idea of where it was turned out to be off; dots under the line were inside the safety bubble's straight-line rule.

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: Right: parked at 68.0, 104.6 and stopped (battery 9 %); Left: parked at 75.3, 19.6 and stopped (battery 10 %); Center: parked at 18.3, 38.9 and stopped (battery 9 %). The run ended: every robot is out (battery 10 %, or silent).

## Conclusion

- At least two robots moved at the same time for 19 minutes (98.9 % of the run); all three for 18 minutes (95.4 %), counting turns on the spot; driving alone: 84.6 % and 43.7 %.
- Full stops (trophy breaks): 1. Reflex changes: 1 (speed 0, steer 0, new move 1).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
