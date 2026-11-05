# Home free roam: 20261105-chain-freeroam-dryrun

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.1 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. Seed 21.

## Data

**The trophy: moving at the same time** (9 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 99.9 % (9 min) | 86.8 % (8 min) |
| all 3 moving | 99.9 % (9 min) | 43.9 % (4 min) |
| every robot still in the run moving | 99.9 % | 43.9 % |

| robot | minutes in the run | any motion | translating |
|---|---|---|---|
| Center | 9 | 99.9 % | 77.0 % |
| Left | 9 | 99.9 % | 75.9 % |
| Right | 9 | 99.9 % | 76.0 % |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Moves**: 264 (264 finished). By kind: menu 234, free 30.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (110 of 112 cells used; fewest 0, most 4).

**Fallbacks** (no menu move fitted; fallbacks.csv): none.

**Reflex events** (reflex.csv): none; outcomes: -. Full stops (trophy breaks, breaks.csv): 0.

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: still roaming at the end. The run ended: the time cap (0.1 h).

## Conclusion

- At least two robots moved at the same time for 9 minutes (99.9 % of the run); all three for 9 minutes (99.9 %), counting turns on the spot; driving alone: 86.8 % and 43.9 %.
- Full stops (trophy breaks): 0. Reflex changes: 0 (speed 0, steer 0, new move 0).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
