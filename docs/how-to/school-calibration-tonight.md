# Tonight at school: the calibration run (zones 1-2)

For 4 Nov, at the school conference table. One robot, **Alvik 3** (WiFi name AL-1DC0, address 192.168.1.134), is
already standing on sticker **P11**, switched on and **asleep** (it wakes for 20 s every 10 minutes). Its roof tag is
id 2. Nobody needs to type which tag is on which robot: the run finds out by itself (the robot wiggles; the tag that
wiggles with it is its tag).

What the run does, about 40 minutes, all at experiment speed (drive 10 cm/s, turn 60 deg/s, last 5 cm at 5 cm/s):

1. finds the robot on the WiFi and wakes it, reads its current settings and saves them;
2. binds its roof tag (a quarter turn and back);
3. **phase A**: turns on the spot at P11, straight rolls out and back, one minute standing still, then a tour of
   P01-P20, all on its own wheels and gyro; the camera only keeps it off the edges, the outlets and the edge of its
   view;
4. **phase B**: works out its new wheel_scale, gyro_scale and gyro_drift_dps, sends them and reads them back;
5. **phase C**: three more tours of P01-P20 with the new settings (in order, reversed, shuffled);
6. parks on P11, sends `sleep 10`, and writes the report.

Build report: [e02-nov04-school-calibration-run](../build-reports/e02-nov04-school-calibration-run.md).

Three places you type or click, always named at the start of each step:

- **Mac Terminal**: the Terminal app on your Mac (Finder, Applications, Utilities, Terminal).
- **Pi window**: the same Terminal window after step 2, when it is logged in to the Pi (the line starts `nick@rfidthesis`).
- **Browser**: Chrome or Safari on your Mac.

Type each box as one line, then press **Enter**.

---

## 1. Tailscale on the joincapstonedb account (Mac)

1. **Mac**, top right of the screen: click the **Tailscale** icon in the menu bar.
2. Click the account name at the top of its menu. Choose **joincapstonedb** (the network of
   capstonedatabaseuri@gmail.com). If it is not listed: **Add account** and sign in with that Google account.
3. You should see a tick next to that account and `Connected`.

## 2. Log in to the Pi (Mac Terminal)

**Mac Terminal:**

```
ssh nick@100.72.183.28
```

The first time it asks `Are you sure you want to continue connecting (yes/no)?`: type `yes`, press **Enter**. Then it
asks for nick's password on the Pi: type it (nothing shows while you type) and press **Enter**. You should see a line
ending in `nick@rfidthesis:~ $`. From here on this window is the **Pi window**.

## 3. Get our repository onto the Pi (Pi window)

**Pi window:**

```
git clone https://github.com/nfmaxfield-cmd/alvik-fleet-capstone.git ~/alvik-fleet-capstone
```

You should see `Cloning into ...` and `done.`

**Pi window:**

```
cd ~/alvik-fleet-capstone
```

Only if the clone said `already exists` (it was cloned before), get tonight's version. **Pi window:**

```
git pull
```

You should see `Fast-forward` or `Already up to date.`

## 4. Set the Pi up (Pi window, about 2-10 minutes)

**Pi window:**

```
./run-fleet-pi.sh school-setup
```

It checks the programs and Python packages, installs what is missing (it may ask for nick's password once: type it,
press **Enter**), sets the camera's focus, picks the school site, and runs the self-check. You should see, near the end:

- `PASS  camera focus: /dev/video0 autofocus off, focus_absolute 0`
- a list of `PASS` / `WARN` lines, and
- `School setup DONE: no FAIL lines.`

`WARN` lines for the GitHub token (uploads), `roof height` (6.19 cm is typed per robot; the run uses it) and `video
space` are fine tonight. A `FAIL` line: see **If something is different** below.

## 5. Wake the robot (Pi window, up to 10 minutes)

**Pi window:**

```
python3 tools/pi/robot_sleep.py wake 192.168.1.134
```

You should see `Waking 192.168.1.134: sending wake every 2 s ...` and, within 10 minutes, `192.168.1.134: awake`. Then
the prompt comes back. (If you skip this step, the run itself keeps waking it for up to 12 minutes.)

## 6. Start Mission Control (Pi window)

**Pi window:**

```
./run-fleet-pi.sh mc
```

You should see `Mission Control is open: http://100.72.183.28:8095 in the laptop's browser` and `It keeps running if
this window closes.` It also sets the camera's focus again (`PASS  camera focus ...`).

## 7. Open the page and Measure (Browser)

1. **Browser**: go to `http://100.72.183.28:8095`
2. Click **Connect**, then **Robots**.
3. Click **Cameras & table** in the top bar. Press **Check the cameras**: every line PASS (a WARN or
   FAIL about camera 2 / "side" is expected tonight: only one camera is plugged in).
4. Press **Take pictures**. You should see orange outlines on tags **20, 23, 24 and 25** (the two corners at the TV end
   and the two side tags). **21 and 22 are expected to be missing** (too small from the TV end). Do **not** click
   corners or stickers.
5. Press **Measure**. You should see the table near **396.2 x 167.6 cm** and a line like `overhead placed from 4
   reference tags (20, 23, 24, 25)`.

## 8. Run the calibration (Browser)

1. Click **Production** in the top bar.
2. In the card **Calibration (zones 1-2)**, press **Start calibration on the robot**, then **OK**.
3. You should see, within a minute: `Finding the robot`, then `Binding its roof tag`, then `Robot: AL-1DC0, roof tag 2`.
   The robot turns a quarter turn left and back: **that is the wiggle, expected.**
4. Then `Phase A ...`. The robot turns on the spot about 12 times, rolls out and back towards the TV end, stands still
   for a minute, then visits P01-P20. Then `Phase B`, then `Phase C` (three tours), then `Parking at P11`, then
   `Done`.
5. You may close the browser; the run goes on. Open the address again to see where it is.

**To stop the robot at any moment** (it heads for an edge; pillows are on the floor), any one of:

- **Browser**: press **Stop** in the Calibration card.
- **Pi window** (open a second Mac Terminal and do step 2 again if the first is busy), first:

```
cd ~/alvik-fleet-capstone
```

  then:

```
./run-fleet-pi.sh stopall
```

After a stop, the robot stays where it is (it is not driven back). Put it back on P11 by hand, nose towards the
whiteboard, before starting again.

## 9. Read the result (Browser)

When the card says **Done**: read the plain-words line under it (the new settings and whether they read back), then
press **Show the report**. It has the before and after errors per leg, the fitted settings, Jay-Sun's four points
(P01, P05, P11, P15) with and without the height correction, the battery used, and the two closing lists.

## 10. Send the files (Pi window): start this straight away

Start this as soon as the run says **Done**, so it uploads while you read the report. **Pi window:**

```
./run-fleet-pi.sh calib-upload
```

You should see `Uploading .../calib/<date-time>` and, at the end, the release's address on GitHub.

If it says there is no GitHub token, copy the folder to your Mac instead. First, **Pi window**, to see the folder's
name:

```
ls -td ~/alvik-fleet-capstone/logs/mission-control-school-conference/calib/* | head -1
```

Then **Mac Terminal** (a new window, not the Pi window; put the folder it printed in place of FOLDER):

```
scp -r nick@100.72.183.28:FOLDER ~/Desktop/
```

You should see the files copied, then the folder on your Desktop.

The robot is asleep at the end (`sleep 10`). To wake it later from home: steps 1 and 2 again, then **Pi window:**

```
cd ~/alvik-fleet-capstone
```

and step 5 again.

---

## If something is different

| You see | What it means | Do |
|---|---|---|
| Step 5 never says `awake` (10 minutes) | The robot is off, flat, or not on AGV_SWARM | Ask Jay-Sun to look at the robot: lights on? If it was switched off and on, it is awake now: go on with step 6 |
| `STOPPED: no robot answered on the WiFi` on the card | Same as above, during the run | As above, then press **Start calibration** again |
| `no roof tag is seen by the camera` | The camera does not see a tag where the robot stands | Check the robot is on P11 with its tag on top, face up. Look at the camera picture (Cameras & table, Take pictures): is the robot in it? |
| `no roof tag moved when AL-1DC0 did its wiggle` | The tag on the robot was not read while it turned (blur, light), or the robot did not turn | Watch the robot during step 8.3: did it turn? If yes, check focus (row below) and start again |
| `2 tags moved` | Someone moved a second tag, or a second robot moved | Keep other tags and robots still; start again |
| Step 7: corner tags 20 or 23 not outlined, or Measure says a corner is missing | The camera does not see that end of the table | Ask Jay-Sun to check the tripod has not moved; tags 20 and 23 must be in the picture. Measure again |
| The picture looks blurry, or `WARN camera focus` | The camera's focus reset (unplugged / Pi restarted) | **Pi window:** `python3 tools/pi/camera_focus.py` You should see `PASS camera focus`. Then Take pictures again |
| The robot heads for an edge | Its own odometry is off (that is what phase A measures), and the camera should step in before the edge | Press **Stop** (or `stopall`). Note the time and the point it was going to. Put it back on P11 |
| `HOLDING: the camera lost the roof tag` | The tag was not read for over 1 s while driving: the robot stopped (cautious) | Nothing: it carries on when the tag is seen again. Over 30 s: the run ends where it is. Check nothing covers the robot |
| `LOW BATTERY` | Under 25 % | It parks at P11 and stops the series. Charge it; the report shows what was done |
| A line with `NOT OK` after the new values | A setting did not read back after two tries: the old value was put back | Fine: the report says which one |
| `port 5005 is in use` | Another program talks to the robots (robot_sleep.py still running, or a Mission Control run) | Wait for robot_sleep.py to finish, or stop the other run, then start again |
