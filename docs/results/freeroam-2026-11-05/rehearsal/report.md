# Home free roam: 20261105-rehearsal-freeroam-dryrun

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.8 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. **Simulator run** (tools/sim/fleet_sim.py), not the robots. Seed 11.

## Data

**The trophy: moving at the same time** (37 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 98.7 % (37 min) | 85.0 % (32 min) |
| all 3 moving | 94.5 % (35 min) | 43.5 % (16 min) |
| every robot still in the run moving | 96.1 % | 44.3 % |

| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |
|---|---|---|---|---|---|
| Center | 37 | 98.3 % | 76.7 % | 36.7 s | 1 |
| Left | 37 | 98.1 % | 75.5 % | 38.8 s | 1 |
| Right | 37 | 99.6 % | 77.1 % | 1.0 s | 0 |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Moves**: 1047 (1047 finished). By kind: menu 932, free 92, wiggle 16, escape 3, park 3, off-menu 1.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (112 of 112 cells used; fewest 3, most 11).

**Fallbacks** (no menu move fitted; fallbacks.csv): find-me wiggle 16, escape 3, off-menu 1.

**Reflex events** (reflex.csv): none; outcomes: -. Full stops (trophy breaks, breaks.csv): 0.

**Lost by the camera** (lost.csv): 17 episodes, 160 s in all; the odometry's jump when seen again: median 0.6 cm, largest 1.7 cm; 17 of 17 inside the bubble.

![lost](chart_lost.png)

Each dot is one time the camera lost a robot: how far it drove unseen, and how far its own idea of where it was turned out to be off; dots under the line were inside the safety bubble's straight-line rule.

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: Right: parked at 48.3, 102.4 and stopped (battery 10 %); Left: parked at 103.8, 24.9 and stopped (battery 10 %); Center: parked at 101.6, 67.8 and stopped (battery 10 %). The run ended: every robot is out (battery 10 %, or silent).

## Conclusion

- At least two robots moved at the same time for 37 minutes (98.7 % of the run); all three for 35 minutes (94.5 %), counting turns on the spot; driving alone: 85.0 % and 43.5 %.
- Full stops (trophy breaks): 0. Reflex changes: 0 (speed 0, steer 0, new move 0).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
