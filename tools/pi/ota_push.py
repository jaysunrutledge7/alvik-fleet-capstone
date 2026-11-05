#!/usr/bin/env python3
"""Firmware over WiFi from the Pi (5 Nov 2026): for a robot on the Pi's WiFi when the PC with the Arduino IDE is NOT on
that WiFi (the school robots, Nick at home). The PC compiles (Arduino IDE: Sketch > Export Compiled Binary), the .bin
goes to the Pi (scp over Tailscale), and the Pi sends it to the robot's WiFi update port with espota.py.

    python3 tools/pi/ota_push.py check --name AL-1DC0                       # are WiFi updates on? (about 1-2 min)
    python3 tools/pi/ota_push.py push --name AL-1DC0 --bin ~/home_lab.ino.bin   # send it (asks the update password)

check: wakes the robot, sends `sleep 1` (it restarts a minute later, as a wake from sleep is a fresh start), and
listens to its start-up lines: firmware/home_lab/ota.h says "WiFi firmware updates on" only when the secrets.h it was
built with defines OTA_PASSWORD, else "WiFi firmware updates off". Nothing is changed on the robot.

push: checks the file (an ESP32 app image, 0.2-4 MB; not the .merged.bin, not the bootloader), finds the robot at
whatever address it answers from, makes sure it is awake and idle (`wake`, `stop`, `status`), asks for the update
password (typed, never shown, never written anywhere; it is the OTA_PASSWORD of the secrets.h the robot was LAST built
with), runs espota.py, then waits for the robot to come back and prints its new firmware version.

espota.py is NOT in this repo (the arduino-esp32 file carries no licence header of its own; the project is LGPL-2.1):
the runbook downloads it to the Pi's home folder (about 15 KB). It needs only Python's standard library.
"""
import argparse
import getpass
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import robot_sleep as RS   # noqa: E402

OTA_PORT = 3232            # ArduinoOTA's port on the ESP32 (arduino-esp32 default; ota.h does not change it)
ON, OFF = "WiFi firmware updates on", "WiFi firmware updates off"
FW_RX = re.compile(r"home_lab (\S+) ready|\(firmware ([^,)]+)")


def check_bin(path):
    """'' if the file looks like an ESP32 app image to send, else why not (plain words)."""
    if not os.path.isfile(path):
        return "no file at %s (copy it to the Pi first: runbook, Reflash over WiFi, step 2)" % path
    n = os.path.getsize(path)
    base = os.path.basename(path)
    if "merged" in base or "bootloader" in base or "partitions" in base:
        return "%s is not the app image: send the file that ends in .ino.bin (not .merged.bin / .bootloader.bin / " \
               ".partitions.bin)" % base
    if not 200 * 1024 <= n <= 4 * 1024 * 1024:
        return "%s is %d KB: an app image is 0.2-4 MB (is it the right file?)" % (base, n // 1024)
    with open(path, "rb") as f:
        if f.read(1) != b"\xe9":
            return "%s does not start like an ESP32 image (first byte 0xE9)" % base
    return ""


def ota_state(lines):
    """From a robot's start-up lines: 'on', 'off' or '' (not said)."""
    for ln in lines:
        if ON in ln:
            return "on"
        if OFF in ln:
            return "off"
    return ""


def listen_for(rx, tx, name, addr, secs, want, wake=True):
    """Lines from that robot (by name or address) for up to secs, sending `wake` at its check-in; stops when want(lines)."""
    out = []
    end = time.time() + secs
    while time.time() < end:
        try:
            data, (src, _) = rx.recvfrom(2048)
        except socket.timeout:
            continue
        n, line = RS._split(data)
        if n != name and src != addr:
            continue
        out.append(line)
        if wake and "check-in" in line:
            tx.sendto(b"wake", (src, RS.CMD_PORT))
        if want(out):
            break
    return out


def find_espota(path):
    for p in ([path] if path else []) + [os.path.expanduser("~/espota.py")]:
        if p and os.path.isfile(p):
            return p
    return ""


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["check", "push"])
    ap.add_argument("--name", required=True, help="the robot's name as it answers (its WiFi name, e.g. AL-1DC0)")
    ap.add_argument("--bin", default="", help="push: the compiled app image (home_lab.ino.bin)")
    ap.add_argument("--espota", default="", help="push: espota.py (default ~/espota.py)")
    ap.add_argument("--max-min", type=float, default=12.0, help="wait this long for the robot to answer")
    a = ap.parse_args(argv)
    if a.what == "push":
        why = check_bin(os.path.expanduser(a.bin))
        if why:
            print("NOT SENT: " + why)
            return 2
        esp = find_espota(os.path.expanduser(a.espota) if a.espota else "")
        if not esp:
            print("NOT SENT: espota.py is not in your home folder. Get it first (runbook, Reflash over WiFi, step 3).")
            return 2
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    rx = RS._bind_rx()
    got = RS.wake_heard(a.max_min, tx, rx, name=a.name)
    if not got:
        print("NOT DONE: %s did not answer in %.0f min (switched off, flat, or not on the WiFi)." % (a.name, a.max_min))
        return 1
    addr = got[a.name][0]
    if a.what == "check":
        print("Sending %s `sleep 1`: it restarts in about a minute and says at start-up whether WiFi updates are on ..."
              % a.name)
        tx.sendto(b"sleep 1", (addr, RS.CMD_PORT))
        lines = listen_for(rx, tx, a.name, addr, 150, lambda L: ota_state(L) and any("ready" in x for x in L))
        st = ota_state(lines)
        ver = next((m.group(1) or m.group(2) for x in lines for m in [FW_RX.search(x)] if m), "")
        if st == "on":
            print("WiFi firmware updates are ON on %s (firmware %s, at %s): it can be flashed from the Pi with "
                  "`push`, using the OTA_PASSWORD of the secrets.h it was built with." % (a.name, ver or "?", addr))
            return 0
        if st == "off":
            print("WiFi firmware updates are OFF on %s (firmware %s): the secrets.h it was built with has no "
                  "OTA_PASSWORD line. It can only be flashed with the USB cable this time (add an OTA_PASSWORD line "
                  "to that secrets.h first, so the next flash can go over WiFi)." % (a.name, ver or "?"))
            return 3
        print("NOT SURE: %s restarted%s but its start-up line about WiFi updates was not heard (a lost packet). Run "
              "check again." % (a.name, " (firmware %s)" % ver if ver else ""))
        return 1
    # push
    tx.sendto(b"stop", (addr, RS.CMD_PORT))
    tx.sendto(b"status", (addr, RS.CMD_PORT))
    st = listen_for(rx, tx, a.name, addr, 4, lambda L: any(x.startswith("M,status ") for x in L), wake=False)
    stl = next((x for x in st if x.startswith("M,status ")), "")
    if stl and " running" in stl.split(":", 1)[-1][:12]:
        print("NOT SENT: %s is running something (updates are refused while it runs). Stop it, then push again." % a.name)
        return 1
    old = (FW_RX.search(stl).group(2) if FW_RX.search(stl) else "?") if stl else "?"
    size = os.path.getsize(os.path.expanduser(a.bin))
    print("%s at %s, firmware %s, idle. Sending %s (%d KB) to its WiFi update port %d." % (
        a.name, addr, old, os.path.basename(a.bin), size // 1024, OTA_PORT))
    pw = getpass.getpass("The robot's WiFi update password (OTA_PASSWORD in the secrets.h it was last built with; "
                         "nothing shows while you type), then Enter: ")
    tx.sendto(b"wake", (addr, RS.CMD_PORT))            # keep it awake (a check-in lasts only 20 s)
    r = subprocess.run([sys.executable, esp, "-i", addr, "-p", str(OTA_PORT), "-a", pw, "-f", os.path.expanduser(a.bin),
                        "-r", "-t", "15"])
    pw = None
    if r.returncode != 0:
        print("NOT FLASHED (espota.py ended with %d). Most likely: the password is not the robot's, WiFi updates are "
              "off on it (`check`), or it went to sleep. The robot still runs its old firmware %s." % (r.returncode, old))
        return 1
    print("Sent. Waiting for %s to restart with the new firmware (up to 90 s) ..." % a.name)
    back = listen_for(rx, tx, a.name, addr, 90, lambda L: any("ready" in x for x in L), wake=True)
    ver = next((m.group(1) for x in back for m in [FW_RX.search(x)] if m and m.group(1)), "")
    if ver:
        print("DONE: %s now runs firmware %s (was %s)." % (a.name, ver, old))
        return 0
    print("Sent, but %s's restart line was not heard (it may have a new address). Check with: "
          "python3 tools/pi/robot_sleep.py wake --name %s" % (a.name, a.name))
    return 1


if __name__ == "__main__":
    sys.exit(main())
