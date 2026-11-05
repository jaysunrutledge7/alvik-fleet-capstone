#!/usr/bin/env python3
"""Put robots to sleep, or wake them, from the Pi or the laptop (firmware 2026-10-25a; Nick, 2026-10-24: robots
left switched on drain about 14% an hour). Not while a session runs (it uses the same WiFi port).

    python3 tools/pi/robot_sleep.py sleep 30 192.168.1.250 192.168.1.252    # sleep, checking in every 30 min
    python3 tools/pi/robot_sleep.py wake 192.168.1.250                        # waits for its next check-in, wakes it
    python3 tools/pi/robot_sleep.py wake any                                  # wakes the first robot heard, any address
    python3 tools/pi/robot_sleep.py wake any --count 2                        # ... the first two robots heard
    python3 tools/pi/robot_sleep.py wake --name AL-1DC0                       # that robot, whatever address it has now

A sleeping robot hears nothing until its check-in (it wakes every N minutes for sleep_awake_s, 20 s): `wake` keeps
sending until the robot answers, for up to --max-min minutes.

`wake any` / `wake --name` (5 Nov 2026, the school Pi: `wake 192.168.1.134` waited 10+ min with no answer; only that
exact address was asked and listened to, and the robot can come back from a check-in on another address): `wake` goes
to the WiFi broadcast every 2 s, and every robot heard (its check-in, its hello, anything) from ANY address is sent
`wake` straight to the address it was heard from. When it answers, its name, address and battery are printed."""
import argparse
import re
import socket
import sys
import time

CMD_PORT = 5006   # laptop -> robot (tools/alvik_console.py)
LOG_PORT = 5005   # robot -> laptop
BROADCAST = "255.255.255.255"
AWAKE = b"awake (it stays awake"


def _bind_rx():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("", LOG_PORT))
    except OSError:
        sys.exit("Port %d is in use: a session (or the console, or the calibration run) is running. Stop it first."
                 % LOG_PORT)
    rx.settimeout(2.0)
    return rx


def _split(data):
    """'name|seq|line' -> (name, line)."""
    txt = data.decode("utf-8", "replace").strip()
    name, _, rest = txt.partition("|")
    _seq, _, line = rest.partition("|")
    return name, line


def battery_of(line):
    m = re.search(r"battery (-?\d+)\s*%", line)
    return int(m.group(1)) if m else None


def wake_ips(ips, max_min, tx, rx):
    """The old way, kept: `wake` to these addresses only. Also says which robots were heard from other addresses."""
    left = set(ips)
    end = time.time() + max_min * 60
    others = {}
    print("Waking %s: sending `wake` every 2 s until each answers (up to %.0f min) ..." % (", ".join(ips), max_min))
    while left and time.time() < end:
        for ip in list(left):
            try:
                tx.sendto(b"wake", (ip, CMD_PORT))
            except OSError as e:
                print("  (could not send to %s: %s)" % (ip, e))
        try:
            data, (src, _) = rx.recvfrom(2048)
        except socket.timeout:
            continue
        if src in left and AWAKE in data:
            print("%s: awake" % src)
            left.discard(src)
        elif src not in ips:
            name, _ = _split(data)
            if others.get(src) != name:
                others[src] = name
                print("(heard %s at %s, not one of the addresses asked: to wake whatever address it has, use "
                      "`wake --name %s` or `wake any`)" % (name, src, name))
    if left:
        print("Not answered: %s (switched off, flat, asleep longer than --max-min, or on another address: try "
              "`python3 tools/pi/robot_sleep.py wake any`)" % ", ".join(sorted(left)))
        return 1
    return 0


def wake_heard(max_min, tx, rx, name=None, count=1, bcast=BROADCAST, also=()):
    """Wake robots by what they say, from any address: `wake` to the broadcast (and `also`) every 2 s, and to every
    address a robot is heard from. name: only that robot (its WiFi name, e.g. AL-1DC0, or its saved name); else the
    first `count` robots heard. Returns {name: (address, battery or None)}."""
    end = time.time() + max_min * 60
    heard = {}        # name -> address (the newest)
    awake = {}        # name -> (address, battery)
    battery = {}
    who = ("%s" % name) if name else ("any robot" if count == 1 else "the first %d robots" % count)
    print("Waking %s, from any address: `wake` to the WiFi broadcast every 2 s, and to each robot as it is heard "
          "(up to %.0f min; a sleeping robot answers at its check-in) ..." % (who, max_min))
    last = 0.0
    said = time.time()
    while time.time() < end:
        if time.time() - last >= 2.0:
            last = time.time()
            for a in [bcast] + list(also):
                try:
                    tx.sendto(b"wake", (a, CMD_PORT))
                except OSError:
                    pass
            for n, a in heard.items():
                if n not in awake and (name is None or n == name):
                    tx.sendto(b"wake", (a, CMD_PORT))
        if time.time() - said > 60:
            said = time.time()
            print("  nothing yet: still listening, %.0f min left (a robot asleep with `sleep 10` checks in every "
                  "10 min)" % ((end - time.time()) / 60))
        try:
            data, (src, _) = rx.recvfrom(2048)
        except socket.timeout:
            continue
        n, line = _split(data)
        if not n:
            continue
        b = battery_of(line)
        if b is not None:
            battery[n] = b
        if name is not None and n != name:
            if heard.get(n) != src:
                print("  (heard %s at %s: not %s, left as it is)" % (n, src, name))
            heard[n] = src
            continue
        if heard.get(n) != src:
            print("  heard %s at %s%s: sending it wake" % (n, src, ", checking in" if "check-in" in line else ""))
            heard[n] = src
            tx.sendto(b"wake", (src, CMD_PORT))
        if AWAKE in data and n not in awake:
            tx.sendto(b"status", (src, CMD_PORT))
            t0 = time.time()
            while battery.get(n) is None and time.time() - t0 < 3.0:     # its battery from `status`
                try:
                    d2, (s2, _) = rx.recvfrom(2048)
                except socket.timeout:
                    break
                n2, l2 = _split(d2)
                if battery_of(l2) is not None:
                    battery[n2] = battery_of(l2)
            awake[n] = (src, battery.get(n))
            print("%s: awake at %s, battery %s" % (n, src, "%d %%" % battery[n] if battery.get(n) is not None
                                                     else "not reported"))
            if name is not None or len(awake) >= count:
                return awake
    return awake


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["sleep", "wake"])
    ap.add_argument("args", nargs="*", help="sleep: minutes then robot IPs; wake: robot IPs, or `any`")
    ap.add_argument("--max-min", type=float, default=35.0, help="wake: give up after this long")
    ap.add_argument("--name", default="", help="wake: this robot by its name (e.g. AL-1DC0), from any address")
    ap.add_argument("--count", type=int, default=1, help="wake any: stop once this many robots are awake")
    ap.add_argument("--broadcast", default=BROADCAST, help=argparse.SUPPRESS)       # tests: the simulator's address
    ap.add_argument("--also", action="append", default=[], help=argparse.SUPPRESS)  # tests: more addresses to ask
    a = ap.parse_args(argv)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    if a.what == "sleep":
        if len(a.args) < 2:
            ap.error("sleep: minutes then robot IPs")
        minutes, ips = int(a.args[0]), a.args[1:]
        for ip in ips:
            tx.sendto(("sleep %d" % minutes).encode(), (ip, CMD_PORT))
            print("%s: sleep %d sent" % (ip, minutes))
        return 0
    if a.name or a.args == ["any"]:
        rx = _bind_rx()
        got = wake_heard(a.max_min, tx, rx, name=a.name or None, count=max(1, a.count), bcast=a.broadcast,
                         also=a.also)
        if not got:
            print("No robot answered in %.0f min (%s). Is it switched on, charged, and on the school WiFi? Its "
                  "check-in is every 10 min after `sleep 10`: run this again, or switch it off and on." % (
                      a.max_min, "named %s" % a.name if a.name else "any address"))
            return 1
        if not a.name and len(got) < a.count:
            print("Only %d of %d robots answered." % (len(got), a.count))
            return 1
        return 0
    if not a.args:
        ap.error("wake: robot IPs, `any`, or --name NAME")
    return wake_ips(a.args, a.max_min, tx, _bind_rx())


if __name__ == "__main__":
    sys.exit(main())
