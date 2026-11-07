# alvik-fleet-capstone

ISE 401 capstone (SWARM project, MIRLT): a fleet of Arduino Alvik robots that sets itself up and drives
accurately on a table it has never seen.

This builds on Andrew Muszynski's work. His repositories are the reference
implementation:

| Repo | What it is |
| --- | --- |
| [AGV-Line-Following-Factory](https://github.com/Andrew-Muszynski/AGV-Line-Following-Factory/tree/VLM-integration) (`VLM-integration` branch) | Current fleet stack: AprilTag localization, micro-ROS firmware, navigator, dashboard |
| [Alvik-Factory](https://github.com/Andrew-Muszynski/Alvik-Factory) | Earlier ROS 2 / micro-ROS line-following AGV |
| [SwarmSLAM](https://github.com/Andrew-Muszynski/SwarmSLAM) | Earlier overhead-camera swarm with a Flask server |

Andrew has said anything in his repos may be used here.

## Time tracking
Every task expected to run over 10 minutes: estimate + questions to Nick first, and a row in
[docs/time-tracker.md](docs/time-tracker.md) when it ends.

## On the Pi: one address

**http://100.122.222.75:8091** (Tailscale; on the home WiFi also `http://<the Pi>.local:8091`) is the one address to
bookmark. It is the start page service (always on; `tools/pi/launcher.py`, PIN), now with three tabs: **Live lab**
(an E02 session's live page and its questions; a DOE loop's or Monday mission's own screen, with Stop, while one runs),
**Mission Control** (opened from the tab on demand) and **Start/Stop**. The old addresses still work: 8090 (live page
during an E02 session), 8095 (Mission Control), 8000 / 8001 (camera pictures). After a `git pull` on the Pi the page
runs the new code only once restarted: `sudo systemctl restart mirlt-launcher`. Details: [docs/pi-run.md](docs/pi-run.md).

## Goal

Plug and play: put robots on an unknown table and point a camera at it, and the system calibrates itself (robots,
camera, table), fuses every signal it has (camera, wheels, gyro, accelerometer, front and line sensors), and degrades
gracefully when a source is lost instead of stopping. The aim is a publishable result and a live demo in December.

Two sites today:
- **Home**: 4 x 4 ft plywood with a drop on every side; robots tracked by their lights (no markers); corners clicked
  for now.
- **School** (with Jay-Sun): the conference table; roof AprilTags on the robots, printed corner tiles, one camera for
  now.

**Mission Control** (Start screen) runs Calibrate, Free roam, Visit points and Experiment routes for any number of
robots, and finds by itself which site it is at: [docs/how-to/mission-control-start.md](docs/how-to/mission-control-start.md).
Home free roam: [docs/how-to/home-free-roam.md](docs/how-to/home-free-roam.md).

**The plan** (Nick, 7 Nov): one pipeline for every site, built in phases: one tracking layer for every camera source,
the fused estimate in the loop, one controller, discovery at start, and edge safety from one derived rule. The old
E02 runner stays working as a "parts bin". Accuracy is graded by the camera only for now. Why and what got left
behind: [docs/audits/2026-11-07-full-audit.md](docs/audits/2026-11-07-full-audit.md).

## Layout

```
firmware/home_lab/     the robots' firmware (driving, curves, safety layers, sleep, settings)
firmware/sensor_explorer/  print every sensor reading over USB
tools/fleet/           Mission Control, the planner and run_plan, free roam, calibration, site files (sites/)
tools/pi/              on the Pi: start page (launcher), camera recorder, camera names, uploads, robot sleep
tools/experiment/      the E02 runner and its parts (the "parts bin")
tools/sim/, tools/fusion/, tools/analysis/, ...   simulators, fusion, analyses (most have a README)
hardware/              printed parts (dock, attachments, reference tiles) and the print sheet
run-fleet-pi.sh        start any fleet run on the Pi
docs/how-to/           current runbooks       docs/build-reports/   one report per build
docs/audits/           full audits            docs/archive/         one-night runbooks and the 2 Nov demo plans
docs/open-questions.md   questions for Nick   docs/doe-backlog.md   chosen numbers waiting for a DOE
```

## Secrets

This repo is public. WiFi credentials, agent IPs and other lab details go in
files that are git-ignored:

- `firmware/home_lab/secrets.h`: copy `secrets.example.h` in that folder and fill it in
- `.env`: for Python tools

Never commit real credentials.
