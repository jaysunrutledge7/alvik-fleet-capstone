# Home free roam: run

## Premise

Three robots move at the same time, all the time, on the home board until each battery reaches 10 % (or 0.1 h). Moves come from a balanced menu of turn x length x speed; every move is planned as a reservation in space and time; a reflex on every camera frame changes speed or steers before anything stops. Seed 79678.

Motion style: **curves** (5 Nov 16:06: curves = robots on firmware 2026-11-05c drive continuous curving pieces from a menu of curvature x length x speed and never stop on purpose; robots on older firmware, or the stopturn choice, stop, turn on the spot and drive; the menu chart below counts the stop-turn moves only).

## Data

**The trophy: moving at the same time** (3 min of roaming; source motion.csv).

| | any motion (wheels turning, turns on the spot count) | translating (driving) |
|---|---|---|
| at least 2 robots moving | 32.2 % (1 min) | 32.2 % (1 min) |
| all 3 moving | 0.0 % (0 min) | 0.0 % (0 min) |
| every robot still in the run moving | 32.2 % | 32.2 % |

| robot | minutes in the run | any motion | translating | longest stand-still | stand-stills over 3 s |
|---|---|---|---|---|---|
| Alpha | 3 | 32.2 % | 32.2 % | 0.5 s | 0 |
| Bravo | 3 | 99.7 % | 99.7 % | 0.5 s | 0 |

![motion](chart_motion.png)

Dark green is the share of each minute with all three robots moving; the paler bands are fewer.

**Never stops** (motion style: curves; speed > 0 = the robot's own wheels show 1 cm/s or more over 0.6 s, every 0.5 s, motion.csv; speed-zero moments and why: zeros.csv).

| robot | style | speed > 0 | speed-zero moments | per hour | the main reasons |
|---|---|---|---|---|---|
| Alpha | curve | 32.2 % | 2 | 40 | the start 1, unexplained 1 |
| Bravo | curve | 99.7 % | 1 | 20 | the start 1 |

Speed > 0 at the same time: at least 2 robots 32.2 %, all 3 0.0 % of the time.

**Moves**: 63 (63 finished). By kind: menu 51, free 11, escape 1.

![menu](chart_menu.png)

Each box counts finished moves of one turn, length and speed; even shading means an even menu (0 of 112 cells used; fewest 0, most 0).

**Fallbacks** (no menu move fitted; fallbacks.csv): escape curve 1.

**Reflex events** (reflex.csv): none; outcomes: -. Full stops (trophy breaks, breaks.csv): 0.

**Camera timing** (camera.csv, measured during the run): fixes arrive 0.06 s after the picture (median; 90 % within 0.10 s); a robot counts as lost when no fix newer than 0.80 s; the camera's clock is +0.70 s from this one (applied).

![battery](chart_battery.png)

Each line is one robot's charge; it parks when its line reaches the dashed 10 % mark.

**How each robot finished**: still roaming at the end. The run ended: the time cap (0.1 h).

## Conclusion

- At least two robots moved at the same time for 1 minutes (32.2 % of the run); all three for 0 minutes (0.0 %), counting turns on the spot; driving alone: 32.2 % and 0.0 %.
- Full stops (trophy breaks): 0. Reflex changes: 0 (speed 0, steer 0, new move 0).
- Closest approaches: no reflex event let the bodies' circles touch.

## Next

- Compare this run's estimated and real move times (moves.csv est_s against dur_s) and set the reservation slack from them.
- Lost-tag jumps against the bubble rule: if any fell outside, widen BLIND_TURN_FRAC or BUBBLE_CAP.
- The DOE rows in docs/doe-backlog.md (edge margin, gaps, free share, reflex horizon) from this run's logs.
