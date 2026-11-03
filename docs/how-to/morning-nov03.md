# Morning, 3 Nov: a home run, then the school run

Two parts. **A** starts the home run before you leave (about 10 minutes of your time; the run then goes on by itself
and uploads when it ends). **B** is the school run on the new Pi with Alpha and Bravo.

Every box is one command: type it exactly, press **Enter**, wait for what "You should see" says before the next box.

**The one command that stops everything** (the Pi window, at any time, home or school):

```
./run-fleet-pi.sh stopall
```

At school the red **Stop** button on Mission Control's Run screen comes first. If a robot still moves: pick it up and
switch it off.

---

## A. The home run (home Pi, Left .250, Center .252, Right .253)

What runs: the DOE loop (`./run-fleet-pi.sh doe`), the home run of the last week. It runs unattended in its own
session on the Pi, so you can close the window and leave; it uploads by itself when it ends. **Nothing about it changed
in last night's build**: home stops on a problem exactly as before (the site's `on_problem` is `stop`).

### A1. Robots on the board (2 min)

1. Switch on **Left**, **Center** and **Right**. Put them on the plywood, anywhere, **30 cm from every edge and 40 cm
   from each other** (the loop then does not need to move them first).
2. Each should be charged to **60 % or more**. The loop sets a robot aside under 25 %.

### A2. Into the Pi (laptop, PowerShell)

1. Press the **Windows key**, type `powershell`, press **Enter**. A blue window opens: this is the **Pi window** once
   you are in.
2. Type:

```
ssh st1maxfini@100.122.222.75
```

Type the Pi's password (nothing shows while you type), press **Enter**. You should see a line ending in `$`.

```
cd ~/alvik-fleet-capstone
```

```
git pull
```

You should see a list of changed files (last night's build) or `Already up to date.`

```
./run-fleet-pi.sh site home
```

You should see `Site now: home` and, among its lines, `on a problem: stop the run (as before)`.

### A3. Wake the robots (they may be asleep: 1-3 min)

```
python3 tools/fleet/wake_loop.py --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253 --minutes 5
```

You should see a line starting with the time and `sending`, then `wake`, `every 2 s to 192.168.1.250, ...`, then one
line per robot: `192.168.1.250 answered: stays awake now`
(and .252, .253). It ends by itself when all three answered.
- **A robot never answers in 5 minutes:** switch it off and on, then type the same box again.
- **`Port 5005 is in use`:** another robot program is running. Type `./run-fleet-pi.sh stopall`, then this box again.

### A4. Check them

```
./run-fleet-pi.sh check
```

You should see `Robots heard: Center, Left, Right`, then one line each with its firmware (`2026-10-29b`) and battery.
- **A robot missing:** the wake box (A3) again.
- **A battery under 60 %:** charge it, or start without it (the loop sets it aside by itself).

### A5. Start it

```
./run-fleet-pi.sh doe 2
```

(`2` = at most 2 hours; any number works.) You should see `DOE loop started in tmux session 'doe' (time cap 2 h ...)`
and `You can close this window: it keeps running on the Pi.`

### A6. Is it going? (1 min after)

```
./run-fleet-pi.sh doe-status
```

You should see the loop's last lines: robots found, then `calibrating alone (about 65 s) ...` for each robot. That is
it: close the window and leave.

- To stop it early: the Pi window, `./run-fleet-pi.sh doe-stop` (every robot stops, then it uploads).
- **Do not start a home Mission Control run while it goes** (the robots and the cameras are busy).

---

## B. The school run (new Pi, Alpha and Bravo)

What runs: Mission Control on the new Pi, site `school-conference`. Last night's build is on for this site:
- **Keep going.** A problem that used to stop the whole run (camera and a robot disagree, no route, a robot lost) is
  said once in the event log, recorded and worked round. If a robot drops out, its points not done go to the robot
  still running. Only **Stop**, a full card, or both robots out end the run.
- **The camera is trusted over the edge sensors.** The feel-out still crawls (3 cm inside the cameras' line, as
  before); where the robots feel an edge apart from the cameras' outline, the card says so, but the outline is not
  moved and the production run starts anyway.
- **Every production point gets a position and a ±**, e.g. `point 7 (Alpha): ±0.9 cm, two cameras` or
  `point 12 (Bravo): ±6.4 cm, odometry for 80 cm since the last camera fix`.
- **New this morning (3 Nov build): Jay's 40 stickers are typed in.** The production points come from Jay-Sun's
  spreadsheet; you do **not** click each sticker. Alpha does zones 1-2 (P01-P20), Bravo zones 3-4 (P21-P40), each in a
  short order, already loaded in Mission Control. **Homes are stickers: Alpha on P11, Bravo on P22**, both facing the
  whiteboard (away from the A3-A4 edge). The raised box in the middle is kept out (a 12 x 12 in square until you
  measure it).
- **You can run with or without AprilTags** (B7c): roof tags, the robots' lights, or no camera at all.

**Two things to check at the table with Jay (assumed until you say):** sticker **P09** is at (17, 57.5) in and **P10** at
(33, 37.5) in, as his drawing shows (his spreadsheet says 37.5 and 27.5). Measure both from the A3 corner. If the
spreadsheet is right, tell me (or delete the two `overrides` lines in `tools/fleet/sites/school-conference.json`).
And measure the raised box: its edges, in inches from the A3 corner.

### B1. Pack

- [ ] The new Pi, its power supply, the USB drive (plug it in **before** switching the Pi on)
- [ ] Both new cameras, their stands, USB cables
- [ ] The six corner tiles (20-25) or the paper tags (`docs/hardware/reference-tags-letter.pdf`)
- [ ] The laptop and charger; the USB-C **data** cable (for flashing Alpha and Bravo)
- [ ] Tape measure; pillows for round the table

### B2. Flash Alpha and Bravo (laptop, Arduino IDE; skip if done)

1. File Explorer: open the repo's `firmware\home_lab` folder. Open **your** `secrets.h` (not in the repo) in Notepad.
   Change `#define WIFI_SSID2 "..."` to the school router's network name and `#define WIFI_PASSWORD2 "..."` to its
   password. Save. (Never paste the password anywhere else.)
2. Plug Alpha's board into the laptop with the data cable. Switch Alpha **on**.
3. Arduino IDE: **File, Open**, `firmware\home_lab\home_lab.ino`. **Tools, Board**: `Arduino Nano ESP32`.
   **Tools, Port**: the one that appeared. Press **Upload** (the right arrow). Wait for `Done uploading`.
4. **Tools, Serial Monitor**, set `115200`. Within 15 s you should see `This robot is AL-xxxx`. Write the AL-xxxx down
   on a sticky note on Alpha.
5. The same for Bravo (steps 2-4).

### B3. The new Pi: in, and its self-check (laptop, PowerShell)

1. Plug the cameras and the USB drive into the Pi, then the power. Wait 2 minutes.
2. PowerShell (Windows key, type `powershell`, **Enter**):

```
ssh nick@mirlt-school.local
```

Password, **Enter**. You should see a line ending in `nick@mirlt-school:~ $`. (`Could not resolve hostname`: wait 2
more minutes and try again; if the laptop is not on the router's WiFi, use the Pi's Tailscale address:
`ssh nick@100.x.y.z`.)

```
cd alvik-fleet-capstone
```

```
git pull
```

```
./run-fleet-pi.sh selfcheck
```

You should see one line per item and, last, `... 0 FAIL. Ready.` Any FAIL line says what to type; do it, then this
box again.

```
./run-fleet-pi.sh site school-conference
```

You should see `Site now: school-conference` and among its lines `on a problem: keep going ...` and
`feel-out: the cameras' outline is trusted ...`.

### B4. The cameras (Pi window)

1. Mount each camera straight down over its half of the table, about 175 cm up, both seeing the middle.

```
python3 tools/pi/camnames.py
```

You should see each camera's name and its USB port (the last part of `usb-...-1.4`, here `1.4`). The camera over
corner 1's half is `overhead`, the other is `side`. Type, with **the name and port it printed** for the camera over
corner 1's half:

```
python3 tools/pi/camnames.py --set overhead "USB 2.0 Camera" 1.4
```

You should see `Saved: the overhead camera is the one named ...`. The same with `side` and the other camera's name and
port.

```
./run-fleet-pi.sh rec
```

You should see `Recorder running.`

```
./run-fleet-pi.sh camcheck
```

You should see each camera with PASS lines (size 1280 x 960, sharpness, exposure, coverage, tag size). Do what any
WARN or FAIL line says.

### B5. The corner tiles and (0, 0) (at the table, with Jay)

1. **(0, 0) is the table corner nearest antenna A3 (the TV end, near side)**: Jay's stickers are measured from it, x
   along the 156 in edge toward the door end (A4), y toward the whiteboard (A1).
2. Tile **20** on that corner (corner 1). Then going round **counter-clockwise seen from above**: **21** on the far end
   of that long side (corner 2), **22** opposite 20 (corner 3), **23** the last corner (corner 4). **24** in the middle
   of the long side from 20 to 21, **25** in the middle of the other long side.
3. Each tile's lip pressed against the table's edge, the tile flat. (Paper tags: the sides marked EDGE flush with the
   table's edges.)
4. Pillows on the floor round the table.

### B6. Name the robots (Pi window; once per robot)

Switch Alpha and Bravo on, on the table. With the AL-xxxx you wrote for Alpha:

```
python3 tools/fleet/robot_cmd.py --to AL-xxxx --wait-s 3 "name Alpha"
```

You should see `AL-xxxx is now called Alpha (kept after power-off)`. The same for Bravo (its AL-xxxx, `"name Bravo"`).

### B7. Mission Control (Pi window, then the laptop's browser)

```
./run-fleet-pi.sh mc
```

You should see `Mission Control is open: http://...:8095 in the laptop's browser`. Type that address in Chrome on the
laptop. You may close the PowerShell window: Mission Control keeps running.

1. **Connect**: click **Robots**.
2. **Cameras & table**: put Alpha and Bravo near the middle, about 80 cm apart, where both cameras see them. Press
   **Check the cameras**: every line PASS (do what a WARN or FAIL line says).
3. Press **Take pictures**. You should see orange outlines `20 corner 1`, `21 corner 2` ... in the pictures: every
   tile in at least one picture, 24 and 25 in both. Do **not** click the corners.
4. **Do not click the stickers.** They are typed in from Jay's spreadsheet. (The old way, clicking each sticker, is
   still there: it is only needed if you switch `points` `use` to `clicked` in the site file.)
5. Press **Measure**. You should see the table near 396.2 x 167.6 cm, **40 production points**, the line
   `Production points: the 40 typed stickers (school-conference-points.csv); their spots are not clicked`, and `Every
   typed measurement agrees`. A tile not seen: its line says which; move it so a camera sees it, Measure again. A
   line starting `CHECK: sticker` means the cameras saw that sticker somewhere else: tell me its number.
6. **Production**: Alpha's and Bravo's lists are already filled in (Alpha starts `P11, P20, P19, ...`, Bravo
   `P22, P21, P28, ...`). You may change them (type, or click points on the table). Press **Plan**. You should see
   `Planned: everyone done in about 3 min` and two lines about P12 and P21 moved just clear of the raised box. Press
   **Continue to run**.
7. **Upload at the end (once):** on the Run screen, **More**, set **upload** to `true`, press **Save run settings**.

### B7c. With or without AprilTags (Run screen, card "Tracking during the run")

The Run screen's first card on the right has three choices. Pick one **before** pressing Run:

| Choice | Needs | What you do differently |
|---|---|---|
| **Roof tags** (the site's setting) | roof tags on Alpha and Bravo; corner tiles (or corner clicks) | B7-B8 as written |
| **Robots' lights** | no tags at all; the robots' lights must be visible to the cameras (Jay's RFID tag on top may cover them: check in the camera pictures) | a **Setup** step appears in the top bar: click the table's 4 corners in each camera picture there, press its Measure. No feel-out, no refit |
| **No camera (odometry only)** | nothing: cameras may be off | Put **Alpha exactly on sticker P11** and **Bravo exactly on P22**, each robot's middle over its sticker, nose pointing at the whiteboard (away from the A3-A4 edge), square to the table's edge. You may skip B4 and B7 steps 2-5 (cameras, Measure): the points are typed. No feel-out, no refit |

With **no camera**: each point's ± grows with distance driven (about ±5 cm after the first 40 cm, ±60 cm by the end
of the run): the event log says `odometry for N cm since the start (no camera this run)`. That is the honest
number, not a fault. The edge guard still stops a robot at a drop. Keep the pillows round the table.

To test without any tags **at first** (Nick's plan): pick **No camera**, run once; then **Robots' lights**; then put
the tags on and pick **Roof tags**.

### B8. The run: feel-out, then production (Run screen)

1. Check the two tick-boxes on the right: **Keep going when something goes wrong** (ticked: the school's setting) and
   **Feel out the table before this run** (ticked the first time on this table).
2. **Recording in step**: read the session id to Jay; both write it down. Press **Start mark** as he presses his.
3. Press **Run (robots)**. The feel-out runs first: `1 of 2: surface check` (about 1 min 40 s: each robot calibrates,
   then stands still), then `2 of 2: crawling` (about 2 min), then the production run starts **by itself**.
4. The feel-out card: lines starting `NOTE:` say a side was felt apart from the cameras' outline; it is logged, the
   outline is kept, nothing waits. That is expected with the cameras trusted.
5. During the production run each robot's card shows `Position (x, y) ±... cm, two cameras` (green under 2 cm, amber
   under 5 cm, red over); **Confidence at each point** fills in, one line per point. Lines starting `KEEP GOING:` in
   the right column (and the Event log) are problems worked round without stopping.
6. When it ends, press **End mark** with Jay. **Review** shows the points with their ±, any skipped or handed-over
   points, and the problems kept going through.
7. **Cameras & table**: **Refit from the last run**; if its words say the checks agree, **Use the refit**.

### B9. Stop

- **Stop** (red, Run screen): every robot halts, the run ends. Keep going never overrides it.
- Pi window: `./run-fleet-pi.sh stopall` (reaches robots with no fixed address too).
- Last resort: pick the robot up and switch it off.

### B10. Upload the session

If you set **upload** to `true` in B7.7, the run uploads by itself when it ends (the Event log's last lines say
`uploading: ...`). If you forgot: Run screen, **More**, **upload** `true`, **Save run settings**, then run once more,
or tell me the run's folder name (Review shows it) and I will give you the upload line.

When the robots are done: switch them off. Pi window:

```
./run-fleet-pi.sh mc-stop
```

---

**Still needs you:** P09 and P10's true positions and the raised box's size (above, start of B). The roof tag's height above the table (`tags.height_cm` in the site file; 6.0 cm is a
placeholder). Measure it at school with a ruler and tell me, or edit it on GitHub.
