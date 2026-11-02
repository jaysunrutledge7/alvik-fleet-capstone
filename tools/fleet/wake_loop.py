#!/usr/bin/env python3
"""Keep sending `wake` to sleeping robots until each one answers, so a robot is caught in its short check-in
window (the firmware's sleep: it wakes for sleep_awake_s, 20 s, then sleeps again) whenever that comes.

Why (Nick, 2 Nov 2026 10:11): robot_cmd.py waits until every robot is heard before it sends anything; robots that
check in at slightly different moments went back to sleep while it waited for the last one. This sends straight
to each robot's address every few seconds, needs nothing heard first, and stops sending to a robot once it says
it is awake.

    python3 tools/fleet/wake_loop.py --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253 --minutes 130

Run it inside tmux so it keeps going when the SSH window closes. Afterwards: ./run-fleet-pi.sh check
"""
import argparse
import socket
import time

LOG_PORT = 5005   # robot -> Pi (tools/alvik_console.py)
CMD_PORT = 5006   # Pi -> robot


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", action="append", required=True, help="robot IP (repeatable)")
    ap.add_argument("--minutes", type=float, default=130, help="give up after this long")
    ap.add_argument("--every-s", type=float, default=2.0, help="send `wake` this often to each robot not yet awake")
    a = ap.parse_args(argv)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("", LOG_PORT))
    except OSError as e:
        raise SystemExit("Port %d is in use: another robot program is running (stop it first). %s" % (LOG_PORT, e))
    rx.settimeout(0.2)

    waiting = set(a.robot)
    end = time.time() + a.minutes * 60
    print(time.strftime("%H:%M:%S"), "sending `wake` every %.0f s to %s until each answers (up to %.0f min)"
          % (a.every_s, ", ".join(sorted(waiting)), a.minutes), flush=True)
    next_send = 0.0
    while waiting and time.time() < end:
        now = time.time()
        if now >= next_send:
            for ip in waiting:
                tx.sendto(b"wake", (ip, CMD_PORT))
            next_send = now + a.every_s
        try:
            data, (ip, _port) = rx.recvfrom(4096)
        except socket.timeout:
            continue
        text = data.decode("utf-8", "replace")
        if ip in waiting and ("awake (it stays awake" in text or "stays awake now" in text):
            waiting.discard(ip)
            print(time.strftime("%H:%M:%S"), "%s is awake and staying awake" % ip, flush=True)
        elif ip in waiting and "check-in" in text:
            print(time.strftime("%H:%M:%S"), "%s checked in; `wake` on its way" % ip, flush=True)
    if waiting:
        print(time.strftime("%H:%M:%S"), "gave up; never heard awake from: %s" % ", ".join(sorted(waiting)))
        return 1
    print(time.strftime("%H:%M:%S"), "all awake. Next: ./run-fleet-pi.sh check")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
