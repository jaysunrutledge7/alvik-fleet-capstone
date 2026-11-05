# Home free roam: 20261105-drill-freeroam-dryrun

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.2 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. **Simulator run** (tools/sim/fleet_sim.py), not the robots. Seed 13.

## Data

**The trophy: moving at the same time** (10 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 99.9 % (10 min) | 85.2 % (9 min) |
| all 3 moving | 99.0 % (10 min) | 39.5 % (4 min) |
| every robot still in the run moving | 99.0 % | 39.5 % |

| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |
|---|---|---|---|---|---|
| Center | 10 | 99.4 % | 73.5 % | 1.0 s | 0 |
| Left | 10 | 99.8 % | 75.4 % | 0.5 s | 0 |
| Right | 10 | 99.6 % | 73.1 % | 1.0 s | 0 |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Moves**: 300 (300 finished). By kind: menu 175, drill 90, free 21, escape 9, wiggle 4, holding 1.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (109 of 112 cells used; fewest 0, most 8).

**Fallbacks** (no menu move fitted; fallbacks.csv): escape 10, find-me wiggle 4, holding turn 1.

**Reflex events** (reflex.csv): speed 27, replan 13; outcomes: kept 40. Full stops (trophy breaks, breaks.csv): 0.

| id | time s | robot | other | change | predicted before | after | closest seen | method | outcome |
|---|---|---|---|---|---|---|---|---|---|
| 2 | 1793899931.38 | Left | Center | speed x0.6 | 15.64 | 18.46 | 22.59 | fspd | kept |
| 3 | 1793899932.18 | Left | Center | speed x0.35 | 15.13 | 18.07 | 19.11 | fspd | kept |
| 1 | 1793899931.03 | Center | Left | speed x0.6 | 15.8 | 17.91 | 19.11 | fspd | kept |
| 4 | 1793899933.18 | Left | Center | speed x0.15 | 15.83 | 18.0 | 17.15 | fspd | kept |
| 5 | 1793899934.34 | Left | Center | new move | 15.21 |  | 17.15 | halt+new move | kept |
| 6 | 1793899939.26 | Right | Left | speed x0.6 | 15.7 | 16.97 | 20.54 | fspd | kept |
| 7 | 1793899939.31 | Left | Right | speed x0.6 | 15.62 | 17.63 | 18.56 | fspd | kept |
| 8 | 1793899949.21 | Right | Center | speed x0.6 | 15.51 | 20.88 | 28.47 | fspd | kept |
| 9 | 1793899949.71 | Right | Center | speed x0.35 | 15.94 | 21.06 | 25.73 | fspd | kept |
| 10 | 1793899970.07 | Center | Left | new move | 15.65 |  | 31.74 | halt+new move | kept |
| 11 | 1793900030.7 | Left | Right | new move | 5.7 |  | 27.3 | halt+new move | kept |
| 12 | 1793900031.4 | Right | Left | speed x0.6 | 7.84 | 19.9 | 24.71 | fspd | kept |
| 13 | 1793900031.45 | Left | Right | speed x0.35 | 9.79 | 19.48 | 24.71 | fspd | kept |
| 15 | 1793900085.43 | Center | Right | speed x0.6 | 15.21 | 19.19 | 25.79 | fspd | kept |
| 16 | 1793900086.03 | Center | Right | speed x0.35 | 14.17 | 16.75 | 24.9 | fspd | kept |
| 17 | 1793900086.43 | Center | Right | speed x0.15 | 15.86 | 17.93 | 23.1 | fspd | kept |
| 18 | 1793900086.98 | Center | Right | new move | 15.9 |  | 23.1 | halt+new move | kept |
| 14 | 1793900085.08 | Right | Center | speed x0.6 | 15.9 | 19.93 | 20.99 | fspd | kept |
| 19 | 1793900087.99 | Right | Center | new move | 15.84 |  | 20.99 | halt+new move | kept |
| 20 | 1793900211.7 | Left | Center | new move | 15.32 |  | 24.53 | halt+new move | kept |
| 21 | 1793900226.71 | Left | Center | speed x0.6 | 14.17 | 17.54 | 23.26 | fspd | kept |
| 22 | 1793900227.11 | Left | Center | speed x0.35 | 15.66 | 18.04 | 19.5 | fspd | kept |
| 24 | 1793900229.87 | Right | Center | new move | 7.93 |  | 21.58 | halt+new move | kept |
| 23 | 1793900227.61 | Left | Center | speed x0.15 | 15.7 | 17.11 | 16.15 | fspd | kept |
| 25 | 1793900229.87 | Left | Center | new move | 15.48 |  | 16.15 | halt+new move | kept |
| 26 | 1793900229.94 | Center | Right | new move | 14.32 |  | 21.23 | halt+new move | kept |
| 27 | 1793900230.89 | Center | Right | new move | 14.79 |  | 16.27 | halt+new move | kept |
| 28 | 1793900231.94 | Center | Left | speed x0.6 | 13.89 | 19.2 | 17.94 | fspd | kept |
| 29 | 1793900231.99 | Left | Center | speed x0.35 | 13.81 | 17.47 | 17.94 | fspd | kept |
| 30 | 1793900334.47 | Center | Left | speed x0.6 | 15.18 | 26.62 | 31.99 | fspd | kept |
| 31 | 1793900335.92 | Center | Left | speed x0.35 | 15.76 | 22.29 | 24.73 | fspd | kept |
| 32 | 1793900337.27 | Center | Left | speed x0.15 | 15.68 | 20.48 | 24.13 | fspd | kept |
| 34 | 1793900370.51 | Left | Right | speed x0.6 | 15.92 | 21.12 | 31.21 | fspd | kept |
| 35 | 1793900371.37 | Left | Right | speed x0.35 | 15.6 | 19.33 | 25.71 | fspd | kept |
| 36 | 1793900372.17 | Left | Right | speed x0.15 | 14.09 | 17.41 | 23.56 | fspd | kept |
| 37 | 1793900372.62 | Left | Right | new move | 15.08 |  | 23.56 | halt+new move | kept |
| 33 | 1793900370.16 | Right | Left | speed x0.6 | 15.58 | 19.99 | 20.62 | fspd | kept |
| 38 | 1793900373.69 | Right | Left | new move | 15.34 |  | 20.62 | halt+new move | kept |
| 39 | 1793900466.91 | Left | Center | speed x0.6 | 15.48 | 27.08 | 32.54 | fspd | kept |
| 40 | 1793900468.21 | Left | Center | new move | 15.4 |  | 32.54 | halt+new move | kept |

![reflex](chart_reflex.png)

Each mark is one time the reflex changed a robot's speed or direction; marks above the dashed line never let the bodies' circles touch.

**Lost by the camera** (lost.csv): 5 episodes, 47 s in all; the odometry's jump when seen again: median 0.8 cm, largest 2.0 cm; 5 of 5 inside the bubble.

![lost](chart_lost.png)

Each dot is one time the camera lost a robot: how far it drove unseen, and how far its own idea of where it was turned out to be off; dots under the line were inside the safety bubble's straight-line rule.

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: still roaming at the end. The run ended: the time cap (0.2 h).

## Conclusion

- At least two robots moved at the same time for 10 minutes (99.9 % of the run); all three for 10 minutes (99.0 %), counting turns on the spot; driving alone: 85.2 % and 39.5 %.
- Full stops (trophy breaks): 0. Reflex changes: 40 (speed 27, steer 0, new move 13).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
