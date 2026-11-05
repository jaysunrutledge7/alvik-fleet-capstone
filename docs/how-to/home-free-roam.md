# Home free roam: three robots moving at once until their batteries run down

Left, Center and Right drive around the home board **at the same time, all the time**, each choosing its own moves,
until each battery reaches **10 %** (it then parks near an edge and goes to sleep) or **4 hours** pass. You start it
from the Pi and walk away. It keeps running when you close the window, log out of SSH, or switch Tailscale to another
account. At the end it writes its report and uploads everything to GitHub by itself.

Every box is **one command**: type it exactly, press **Enter**, and wait for what "You should see" says before the next
box. Three kinds of window are used, and each step says which one:

- **PowerShell on your PC**: the Windows key, type `powershell`, press Enter.
- **The Pi window**: the same PowerShell window after you have logged in to the Pi (step 3). Its lines end in `$`.
- **Browser**: Chrome or Edge on your PC.

**The one command that stops everything** (the Pi window, at any time):

```
./run-fleet-pi.sh stopall
```

If a robot still moves: pick it up and switch it off.

---

## 1. Reflash the robots (for curves; once; about 5 minutes per robot)

The new firmware is **2026-11-05c** (5 Nov evening). With it the robots drive **continuous curves**: no stopping to
turn on the spot, they curve from one move into the next, slow down (never below 3 cm/s) or bend their curve to keep
apart, and curve away from something in front instead of stopping. It keeps everything 2026-11-05b had (speed and
steer changes while driving, the 1.5 s watchdog).

A robot that is **not** reflashed (your robots ran **2026-10-28a** this afternoon) still works: it simply drives the
old way (stop, turn on the spot, drive). The start-up lines say which way each robot drives, and the report counts
the two kinds apart. So reflashing is what makes them curve; skipping it is safe.

On your PC, for each robot (Left, Center, Right):

1. **Browser**: open `https://github.com/nfmaxfield-cmd/alvik-fleet-capstone`, click the green **Code** button, then
   **Download ZIP**. Open the downloaded file and drag the `alvik-fleet-capstone-main` folder to your Desktop.
   (Only once, not once per robot.)
2. Copy **your own** `secrets.h` (the one you flashed with last time) into
   `alvik-fleet-capstone-main\firmware\home_lab\`. It is never in the download.
3. Plug the robot into the PC with the USB-C **data** cable. Switch the robot **on**.
4. Arduino IDE: **File, Open**, `alvik-fleet-capstone-main\firmware\home_lab\home_lab.ino`. **Tools, Board**:
   `Arduino Nano ESP32`. **Tools, Port**: the one that appeared when you plugged it in. Press **Upload** (the right
   arrow). Wait for `Done uploading`.
5. **Tools, Serial Monitor**, set `115200`. You should see `home_lab 2026-11-05c ready, battery NN%`.
6. Unplug it and do the next robot.

## 2. Before you start (5 minutes)

- [ ] All three robots **charged to 100 %** and switched **on**.
- [ ] On the plywood, anywhere, **30 cm from every edge and 40 cm from each other**, nothing else on the board.
- [ ] The overhead camera (and the side camera) plugged into the Pi and not moved since the last run. If one moved:
      `./run-fleet-pi.sh setup` first (camera setup page).
- [ ] The room lit as for the DOE runs (it runs for hours: avoid sun across the board if you can).
- [ ] **Charge the robots straight after the run.** They end at about 10 % (asleep).

## 3. Into the Pi (PowerShell on your PC)

```
ssh st1maxfini@100.122.222.75
```

Type the Pi's password (nothing shows while you type), press **Enter**. You should see a line ending in `$`. From now
on this is **the Pi window**.

```
cd ~/alvik-fleet-capstone
```

```
git pull
```

You should see a list of changed files, or `Already up to date.`

```
./run-fleet-pi.sh site home
```

You should see `Site now: home`.

## 4. Check the robots (the Pi window)

```
./run-fleet-pi.sh check
```

You should see `Robots heard: Center, Left, Right`, then one line per robot with its firmware (`2026-11-05b` after
step 1) and `battery 100%` or close to it.

- **A robot missing:** switch it off and on, wait 20 seconds, type the box again.
- **`Port 5005 is in use`:** another robot program is running. Type `./run-fleet-pi.sh stopall`, then the box again.

## 5. Start it (the Pi window)

```
./run-fleet-pi.sh freeroam
```

You should see `Free roam started in tmux session 'freeroam' (time cap 4 h; each robot parks at 10 % battery)` and
`You can close this window and switch Tailscale: it keeps running on the Pi.`

**Curves or the old way.** Curves are the default (for robots on 2026-11-05c). For tonight's run with full video,
type this instead of the box above (one command):

```
./run-fleet-pi.sh freeroam 4 full-video
```

To make every robot drive the old way (stop, turn on the spot, drive), add the word `stopturn`:

```
./run-fleet-pi.sh freeroam 4 stopturn full-video
```

When it starts (watch it with `tmux attach -t freeroam`; leave with **Ctrl+B**, then **D**) you should see one line
per robot ending in `motion: CURVES (never stops on purpose)` for a reflashed robot, or
`motion: stop-turn-drive (its firmware is older than 2026-11-05c: reflash for curves)` for one that is not.

**The curves' own calibration** happens during the run: each robot's curves are measured by the camera, and when it
parks at 10 % the values are saved on it (you see `arc calibration ... saved on the robot` in the events). Nothing
to do. (To leave them unsaved, a helper can add `--no-arc-save`.)

What happens next, by itself: the robots are found by their lights and calibrated one at a time (about 1 minute each;
a robot calibrated earlier today on the same power-on keeps it). Then all three start moving together.

## 6. Is it going? (the Pi window, 5 minutes after step 5)

```
./run-fleet-pi.sh freeroam-status
```

You should see `Free roam ...: N min`, then one line per robot: `move`, its battery and how many moves it has made.
Then the run's last screen lines.

(Since 5 Nov evening, after the first real run) At the start the run also prints a line beginning `Board assumed:`:
the board size it plans on (from the camera setup calibration). If a robot's own front sensor or edge sensor finds a
wall or object inside that board, the screen says `keep-out` or `wall found` and the run keeps away from it from then
on: nothing to do. A robot that its front sensor stopped now **backs off a few cm** (it drives backwards slowly) before
it turns; a robot near an edge with no room to turn backs away from the edge. Both are expected.

To watch the robots: **Browser**, `http://100.122.222.75:8000` (the overhead camera).

You can now close the window, log out of SSH, and switch Tailscale to the other account. **None of that stops it**: it
runs inside the Pi (tmux session `freeroam`).

## 7. Stop it early (only if you want to; the Pi window)

```
./run-fleet-pi.sh freeroam-stop
```

You should see `Sent Ctrl+C to the free roam: every robot stops; it then makes its report and uploads`. Every robot
stops where it is.

## 8. When it is over

It ends by itself when every robot has parked at 10 %, or at 4 hours. It then uploads the run folder (logs, the camera
track, every event, the report and its charts) to GitHub as a release named `freeroam-<date-time>-freeroam`, as the
DOE loop does. `./run-fleet-pi.sh freeroam-status` then says `ENDED: ...` and why.

- **Charge the robots now.** A parked robot is asleep: switch it off and on to wake it.
- The report is `report.md` in the run folder: how long two and three robots moved at once (the trophy), how evenly the
  moves were spread, every avoidance (speed change, steer, new move, full stop), every time the camera lost a robot,
  and the battery curves.

### If something looks wrong while it runs

| You see | What it means | What to do |
|---|---|---|
| A robot standing still for a long time, others moving | its battery reached 10 % and it parked, or it stopped answering | nothing: the others go on round it |
| A robot turning a little left and right on the spot | the camera lost it; it is wiggling so the camera finds it again | nothing |
| A robot near the edge | it keeps 9 cm between its corners and the edge when it plans; its own edge sensors stop it at the edge | if it is over the edge: `./run-fleet-pi.sh stopall`, pick it up |
| A robot backing up a few cm | its own front sensor felt something (or it was too close to an edge to turn): it backs away, then turns | nothing |
| `camera disagrees` on the screen | the camera put a robot somewhere its wheels say it cannot be (5 Nov: Center put on Right's spot); it is not believed | nothing; if it repeats for one robot for minutes, note the time |
| Two robots touching | should not happen (none in 30 simulated hours of stress tests and 4 rehearsals) | `./run-fleet-pi.sh stopall`, tell me the time |
