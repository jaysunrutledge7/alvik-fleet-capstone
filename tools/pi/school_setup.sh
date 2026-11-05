#!/usr/bin/env bash
# The school Pi, one command (2026-11-04, Nick: "one command each"): ./run-fleet-pi.sh school-setup
# From a fresh clone in ~/alvik-fleet-capstone as user nick (sudo asks for nick's password once, if anything is missing).
#   1. programs: tmux, v4l2-ctl (v4l-utils), git            -> sudo apt-get install -y the missing ones
#   2. Python packages: numpy, scipy, OpenCV (cv2), matplotlib -> sudo apt-get install -y python3-...
#      the tag detector: OpenCV's aruco with AprilTag 25h9, else pupil-apriltags (pip, --user)
#   3. the camera's focus: autofocus off, focus_absolute 0 (tools/pi/camera_focus.py; it resets at every unplug or
#      restart, so Mission Control's start sets it again too)
#   4. the site: school-conference (tools/fleet/sites.py use)
#   5. the self-check (tools/pi/selfcheck.py): one PASS / WARN / FAIL line each
# SCHOOL_SETUP_DRY=1: says what it would install, installs nothing (how it was tested off the Pi).
set -u
cd "$(dirname "$0")/../.."
DRY="${SCHOOL_SETUP_DRY:-0}"
say() { echo; echo "=== $* ==="; }
APT=()

say "1. Programs"
for pair in tmux:tmux v4l2-ctl:v4l-utils git:git; do
  prog="${pair%%:*}"; pkg="${pair##*:}"
  if command -v "$prog" >/dev/null 2>&1; then echo "ok       $prog"; else echo "MISSING  $prog (apt package $pkg)"; APT+=("$pkg"); fi
done

say "2. Python packages"
for pair in numpy:python3-numpy scipy:python3-scipy cv2:python3-opencv matplotlib:python3-matplotlib; do
  mod="${pair%%:*}"; pkg="${pair##*:}"
  if python3 -c "import $mod" >/dev/null 2>&1; then echo "ok       $mod"; else echo "MISSING  $mod (apt package $pkg)"; APT+=("$pkg"); fi
done

if [ "${#APT[@]}" -gt 0 ]; then
  echo; echo "Installing: ${APT[*]}"
  if [ "$DRY" = 1 ]; then echo "(SCHOOL_SETUP_DRY=1: would run) sudo apt-get update && sudo apt-get install -y ${APT[*]}"
  else
    echo "(sudo may ask for nick's password: type it and press Enter; nothing shows while you type)"
    sudo apt-get update && sudo apt-get install -y "${APT[@]}" || { echo "STOPPED: the install did not finish (internet? see the lines above)."; exit 1; }
  fi
else
  echo; echo "Nothing to install."
fi

# the tag detector: OpenCV's aruco module with the AprilTag 25h9 family, else pupil-apriltags
if python3 -c "import cv2, sys; sys.exit(0 if hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'DICT_APRILTAG_25h9') else 1)" >/dev/null 2>&1; then
  echo "ok       tag detector (OpenCV aruco, AprilTag 25h9)"
elif python3 -c "import pupil_apriltags" >/dev/null 2>&1; then
  echo "ok       tag detector (pupil-apriltags)"
else
  echo "MISSING  tag detector: installing pupil-apriltags (pip, for user $(whoami))"
  if [ "$DRY" = 1 ]; then echo "(SCHOOL_SETUP_DRY=1: would run) python3 -m pip install --user pupil-apriltags"
  else python3 -m pip install --user pupil-apriltags || python3 -m pip install --user --break-system-packages pupil-apriltags || echo "WARNING: pupil-apriltags did not install: the self-check below says what is missing."
  fi
fi

say "3. Camera focus (autofocus off, focus_absolute 0)"
python3 tools/pi/camera_focus.py || true

say "4. Site"
python3 tools/fleet/sites.py use school-conference

say "5. Self-check"
python3 tools/pi/selfcheck.py "$@"
rc=$?
echo
if [ $rc -eq 0 ]; then echo "School setup DONE: no FAIL lines. Next: wake the robot (runbook step 5)."
else echo "School setup: some lines say FAIL (above). Read them; the runbook says what to do."; fi
exit $rc
