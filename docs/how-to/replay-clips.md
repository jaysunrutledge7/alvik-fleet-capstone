# Replay clips: a run's interesting moments, as the program saw them

`tools/analysis/replay_clips.py` picks the interesting moments of a free-roam run from its logs (edge breaks, tilt
stops, front stops and keep-outs, near-misses between robots, camera losses, camera-vs-own-pose disagreements,
reflexes, stuck robots, and the e08 / e09 edge and identity checks), cuts a short clip of each from the overhead
video at half speed, and draws on it what the program saw and decided:

- **green dot**: a camera fix the robot used; **red X**: a fix it refused (or the program had it lost);
- **coloured ring and tick**: where the robot itself believed it was, and its heading; a dashed circle around it is
  its own uncertainty (2 x pos_sd) when that is big enough to see;
- **coloured line**: the move it planned (a curve from its curvature, dashed for a reverse);
- **amber band**: too near the edge; **red circles**: keep-outs;
- **bar underneath**: the clock time, the robot, and the log line behind the decision.

It also writes a replay page (`index.html`) with the clips, the program's map beside the video, and the log in step.
It only reads logs and video files: it never contacts a robot, the Pi's cameras or the robots' WiFi.

## After every free-roam run: nothing to type

`./run-fleet-pi.sh freeroam-finish` (the end of every free roam) starts it by itself, in the background, beside the
upload. It never stops or slows the upload: it runs at low priority, stops making clips after 15 minutes, and if
anything goes wrong it writes why in `logs/<session>-replay.log` and the upload carries on as before. When the run's
upload has finished, it adds **`replay-<session>.zip`** to the same GitHub release (the page and its clips: unzip it
and open `replay/index.html`).

If the run kept no full video (the default keeps only event stills and short event clips), it uses the short event
clips; a moment with no video at all still gets the map replay on the page.

To switch it off, in the Pi window (after logging in, step 1 below):

```
touch ~/.config/mirlt/replay_clips_off
```

To switch it back on, in the Pi window:

```
rm ~/.config/mirlt/replay_clips_off
```

## Making clips by hand on the Pi

Use this for a run whose clips you want again, or with moments of your own choosing.

**1. Log in to the Pi.** In **PowerShell on your PC** (Windows key, type `powershell`, press Enter), type:

```
ssh st1maxfini@100.122.222.75
```

You should see a line ending in `$`. This is **the Pi window** from now on.

**2. Go to the project folder.** In the Pi window:

```
cd ~/alvik-fleet-capstone
```

**3. Find the run's folder.** In the Pi window (this shows the newest free-roam folder):

```
ls -td logs/*-freeroam | head -1
```

You should see something like `logs/20261106-210129-freeroam`. Use that name in the next boxes in place of
`logs/20261106-210129-freeroam`.

**4. See which moments it would pick** (takes a few seconds; nothing is written). In the Pi window:

```
python3 tools/analysis/replay_clips.py pick logs/20261106-210129-freeroam
```

You should see one line per moment: the time, the kind, the robot and a short explanation.

**5. Make the clips** (about 3-6 minutes on the Pi). In the Pi window:

```
python3 tools/analysis/replay_clips.py make logs/20261106-210129-freeroam --video "$(python3 tools/fleet/diskguard.py where logs/20261106-210129-freeroam)" --out logs/20261106-210129-freeroam-replay
```

You should see `mapping: homography, median ... px`, one line per moment ending in `clip`, then `wrote ...moments.json`.
If a moment says `no clip`, its video was already deleted after the upload (the page still replays its map).

**6. Make the page.** In the Pi window:

```
python3 tools/analysis/replay_clips.py page logs/20261106-210129-freeroam-replay --to logs/20261106-210129-freeroam-replay/page
```

You should see `page: .../index.html (... MB page, 1 runs, ... moments)`.

**7. Copy the page to your PC.** Open a **new PowerShell window on your PC** (not the Pi window) and type:

```
scp -r st1maxfini@100.122.222.75:alvik-fleet-capstone/logs/20261106-210129-freeroam-replay/page $HOME\Desktop\replay
```

**8. Open it.** In File Explorer, open `Desktop\replay` and double-click `index.html`. Your browser shows the
moments on the left; click one, then press **Play**.

## Choosing your own moments

Add `--at` with the clock time (and, if you like, the kind and the robot) to step 5, once per moment:

```
python3 tools/analysis/replay_clips.py make logs/20261106-210129-freeroam --video "$(python3 tools/fleet/diskguard.py where logs/20261106-210129-freeroam)" --out logs/20261106-210129-freeroam-replay --at 21:09:11/tilt --at 21:06:43/front/Center
```

The kinds: `edge`, `tilt`, `recovery`, `front`, `keepout`, `backout`, `pass`, `lost`, `pose`, `reflex`, `stuck`,
`silent`, `unsure`, `identity`, `parked`. `--top 12` keeps more moments (8 by default). `--notes notes.json` replaces
the explanation of a moment with your own (`{"21:09:11": "what happened"}`).

## How it lines the drawing up with the picture

No calibration file is needed. It finds dark robot-sized shapes on the bright board in a few hundred pictures,
pairs them with the camera fixes taken at the same moment, and fits the board-to-picture mapping from the pairs that
agree (written to `mapping.json`, with its error in pixels). The fit is to the robots' tops, so the drawn board
outline sits a little outside the plywood's edge in the picture (the camera sees a robot's top farther out than its
wheels); the program's numbers are drawn as the program had them.

Times: each camera fix is drawn on the picture it was taken from (its `t`), not when it arrived (about 0.45 s later).
When a video has no per-picture time file, the times are spread evenly from the file's start (its name) to the next
file's start, and the tool measures how far off that is (on 5 Nov the names ran 0.7-1.4 s ahead of the pictures)
and corrects it.
