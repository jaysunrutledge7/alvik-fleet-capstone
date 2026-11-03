# Tonight at school: the shakedown, experiments 1-4, and Jay-Sun's raw files

For 3 Nov, 5 pm, with Jay-Sun. Everything before the Production screen is as in
[morning-nov03.md](morning-nov03.md) section B (pack, flash Alpha and Bravo, the new Pi, cameras, corner tiles, name
the robots, Mission Control open in the laptop's browser). This page replaces **B7 step 6** and **B8**, and adds the
export at the end.

What is new tonight (build report: [e02-nov03-experiments-routes-data](../build-reports/e02-nov03-experiments-routes-data.md)):

- An **Experiment** card on the Production screen: Shakedown, Experiment 1 (in order), 2 (reversed), 3 (zig-zag),
  4 (shuffled), the default orders, or Custom (your own lists).
- **Swap halves**: every other run, Alpha does Bravo's half and Bravo does Alpha's. The robots still start and end on
  their own home stickers (Alpha P11, Bravo P22). On a swapped run they cross the middle at the start and at the end.
- A **sync wiggle** at the start of every run. Both robots turn a quarter turn, stop, turn back and stop, at the same
  moment, at their homes. Jay-Sun's RFID software uses it to line up the clocks.
- **Same speed** in every run: drive 10 cm/s, turn 60 deg/s, final approach 5 cm/s.
- At the end of each run there are two new files in the run's folder: **camera_track.csv** (every camera fix) and
  **odometry_raw.csv** (wheels and gyro only, never corrected by a camera). There is also a command that zips one
  run's raw files for Jay-Sun.

The order tonight: **Shakedown first** (Jay-Sun: not one of the four experiments), then **Experiment 1, 2, 3, 4**.

---

## 1. Shakedown (no camera, about 4 min)

**In the laptop's browser, Mission Control, screen 3 Production:**

1. In the **Experiment** card, set **Run** to `Shakedown (no camera, default orders)`.
   You should see: `Halves as usual`, and the line `The shakedown runs with no camera: Plan sets Tracking to No camera.`
   Alpha's list fills with `P11, P20, P19, ...` and Bravo's with `P22, P21, P28, ...`.
2. Leave **sync wiggle at the start** ticked.
3. Press **Plan**. You should see the green note `Shakedown: tracking set to No camera (odometry only) on the Run
   screen` and `Planned: everyone done in about 3:10`.
4. Press **Continue to run**.

**At the table:**

5. Put **Alpha exactly on sticker P11** and **Bravo exactly on sticker P22**. Each robot's middle goes over its
   sticker, with the nose pointing at the whiteboard (away from the A3-A4 edge) and the robot square to the table's
   edge. With no camera, this is the only position the robots ever get.

**On screen 4 Run:**

6. Check that the card **Tracking during the run** shows **No camera**.
7. **Recording in step**: read the session id to Jay-Sun. Press **Start mark** at the moment he presses his.
8. Press **Run (robots)**. In the first seconds after start-up, both robots turn a quarter turn left, stop, turn back
   and stop. **That is the sync wiggle; it is expected.**
9. When it ends, press **End mark** with Jay-Sun.

## 2. Experiments 1 to 4 (each about 4-8 min)

Before Experiment 1: put the roof tags on and check the cameras (morning-nov03 B7 steps 2-5: **Check the cameras**,
**Take pictures**, **Measure**).

For each experiment, in order 1, 2, 3, 4:

**Screen 3 Production, in the Experiment card:**

1. Set **Run** to `Experiment 1: in order`. Use `Experiment 2: reversed`, `Experiment 3: zig-zag across zones` or
   `Experiment 4: shuffled` for the next ones.
2. Leave **Swap halves** at `every other run (auto)`. The bold words underneath say what this run does:
   `Halves SWAPPED` or `Halves as usual`, plus `Runs counted at this site: N`. With the shakedown as run 1,
   Experiments 1 and 3 should show **SWAPPED** and Experiments 2 and 4 **as usual**.
   - The count goes up by one each time a run starts on the robots. If a run was stopped and you run it again, set
     **Swap halves** by hand to what it should be (`swapped` or `not swapped`). Writing down what each run did is
     enough; it is also saved in the run's summary.
3. The card also shows **Legs over the middle (x = 78 in)** for each robot. For Experiments 2-4 that is 6 or more
   per robot; for Experiment 1 it is 0, or 2 when swapped.
4. Press **Plan**. If tracking was No camera, the note `Tracking back to the site's (tags) on the Run screen` appears.
   You should see `Planned: everyone done in ...` (between about 3 and 8 min; Experiment 3 swapped is the longest).
5. Press **Continue to run**.

**Screen 4 Run:**

6. Check that the card **Tracking during the run** shows **Roof tags**, and that **Keep going when something goes
   wrong** is ticked.
7. Press **Start mark** with Jay-Sun, then **Run (robots)**. The robots do the start-up, then the **sync wiggle**,
   then their points. On a swapped run, each robot first drives to the far half, round the raised box.
8. When it ends, press **End mark** with Jay-Sun. **Review** shows the points.

## 3. Jay-Sun's raw files (after each run, or all at the end)

**In the Pi window (PowerShell, logged into the Pi), one command:**

```
./run-fleet-pi.sh jay-bundle
```

You should see `the newest run: .../logs/mission-control-school-conference/runs/<date-time>-run-robots`, then
`camera track: ... (Alpha N, Bravo N)`, `wheel+gyro odometry, no camera corrections: ...`, and finally
`bundle for Jay-Sun: .../jay-raw-<date-time>-run-robots.zip (24 files, ...)`.

For an older run, give its folder (Review shows the folder name):

```
./run-fleet-pi.sh jay-bundle logs/mission-control-school-conference/runs/<folder name>
```

The zip holds our files only: the camera track, the raw odometry, the robots' logs, the event log, the summary with the
experiment and the sync-wiggle times, the site file, the points, and a README of the columns. **Nothing from Jay-Sun's
RFID capture goes in.** It is never added to git (logs/ is ignored). To get it to Jay-Sun, upload the run as usual
(it is inside the run's folder) or copy it to a USB stick.

## If something is different

| You see | What it means | Do |
|---|---|---|
| `every robot held 3 times ...: the run ends here` | Keep going stopped looping: both robots were held again and again (new cap) | Note which points are listed as not done. Run the same experiment again with Swap halves set by hand |
| `SYNC WIGGLE` lines missing in the event log | The wiggle was switched off | Tick **sync wiggle at the start** and Plan again |
| A robot bumps the raised box when crossing | The box is bigger than the 12 x 12 in we assumed | Stop. Measure the box and tell me |

**Still needs you:** P09 and P10's true positions, the raised box's size, and the roof tag's height (morning-nov03 B).
