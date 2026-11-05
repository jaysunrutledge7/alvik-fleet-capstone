#!/usr/bin/env python3
"""The school camera's focus (Jay-Sun 3-4 Nov: Logitech C920e, autofocus off, focus_absolute 0). Both settings RESET
when the camera is unplugged or the Pi restarts, so ./run-fleet-pi.sh school-setup, every Mission Control start
(`./run-fleet-pi.sh mc`) and every calibration run (calib_run.py) set them again, then read them back.

    python3 tools/pi/camera_focus.py                 # the site's camera_hw.device (default /dev/video0)
    python3 tools/pi/camera_focus.py --dev /dev/video2 --focus 0
Prints one line: PASS (set and read back), WARN (no camera / no v4l2-ctl: nothing set). Exit 0 either way, so a start
never stops on it; exit 1 only when the camera is there and does not take the setting."""
import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "fleet"))

AUTO_NAMES = ("focus_automatic_continuous", "focus_auto")   # the control's name differs between kernels


def site_device():
    try:
        import sites as S
        hw = (S.load() or {}).get("camera_hw") or {}
        return hw.get("device") or "/dev/video0"
    except Exception:  # noqa: BLE001
        return "/dev/video0"


def ctrl_names(dev):
    r = subprocess.run(["v4l2-ctl", "-d", dev, "--list-ctrls"], capture_output=True, text=True, timeout=5)
    return {ln.split()[0] for ln in r.stdout.splitlines() if ln.strip()}, r.returncode


def set_focus(dev, focus=0, log=print):
    if not shutil.which("v4l2-ctl"):
        log("WARN  camera focus: v4l2-ctl is not installed (./run-fleet-pi.sh school-setup installs v4l-utils): "
            "focus NOT set")
        return None
    if not os.path.exists(dev):
        log("WARN  camera focus: no camera at %s (plugged in?): focus NOT set" % dev)
        return None
    names, rc = ctrl_names(dev)
    auto = next((n for n in AUTO_NAMES if n in names), None)
    if rc != 0 or "focus_absolute" not in names:
        log("WARN  camera focus: %s has no focus_absolute control (not the C920e?): nothing set" % dev)
        return None
    cmds = (["%s=0" % auto] if auto else []) + ["focus_absolute=%d" % focus]
    for c in cmds:     # autofocus off FIRST (focus_absolute is refused while it is on)
        subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl=%s" % c], capture_output=True, timeout=5)
    want = ([auto] if auto else []) + ["focus_absolute"]
    r = subprocess.run(["v4l2-ctl", "-d", dev, "--get-ctrl=%s" % ",".join(want)], capture_output=True, text=True,
                       timeout=5)
    got = {}
    for ln in r.stdout.splitlines():
        k, _, v = ln.partition(":")
        try:
            got[k.strip()] = int(v.strip())
        except ValueError:
            pass
    ok = got.get("focus_absolute") == focus and (not auto or got.get(auto) == 0)
    log("%s  camera focus: %s autofocus %s, focus_absolute %s (wanted off, %d)" % (
        "PASS" if ok else "FAIL", dev, {0: "off", 1: "ON"}.get(got.get(auto), "?") if auto else "(no control)",
        got.get("focus_absolute"), focus))
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dev", default="")
    ap.add_argument("--focus", type=int, default=0)
    a = ap.parse_args(argv)
    ok = set_focus(a.dev or site_device(), a.focus)
    return 1 if ok is False else 0


if __name__ == "__main__":
    sys.exit(main())
