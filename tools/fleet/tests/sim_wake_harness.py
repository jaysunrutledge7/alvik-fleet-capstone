#!/usr/bin/env python3
"""Run inside `unshare -rn` (its own network, so the robots' ports 5005/5006 are free): the school table in fleet_sim
with robots asleep, then robot_sleep.py with the given arguments. Prints robot_sleep's words, then RC=<exit code>.
Used by test_save_wiggle_multi.py (5 Nov 2026). Not a tool for the Pi.

    python3 sim_wake_harness.py <folder> <robot spec NAME:MIN:FRAC ...> -- <robot_sleep.py arguments>
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "pi")):
    if p not in sys.path:
        sys.path.insert(0, p)


def cmds(args):
    """`cmds IP CMD ...`: each command sent to that robot, every line heard for 1.5 s after it printed (CMD 'wait:S'
    waits S seconds, printing what is heard)."""
    import socket
    ip, todo = args[0], args[1:]
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("", 5005))
    rx.settimeout(0.2)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for c in todo:
        dur = 1.5
        if c.startswith("wait:"):
            dur = float(c[5:])
        else:
            tx.sendto(c.encode(), (ip, 5006))
            print(">> " + c, flush=True)
        end = time.time() + dur
        while time.time() < end:
            try:
                d, _ = rx.recvfrom(4096)
            except socket.timeout:
                continue
            print(d.decode("utf-8", "replace"), flush=True)
    return 0


def main():
    argv = sys.argv[1:]
    k = argv.index("--")
    folder, specs, rs_args = argv[0], argv[1:k], argv[k + 1:]
    import calib_run as CR
    import production as PR
    import sites as SI
    CR.loopback_up()
    S = CR.Site(SI.load("school-conference"))
    names = [s.split(":")[0] for s in specs if not s.startswith("--")]
    os.makedirs(os.path.join(folder, "sim"), exist_ok=True)
    m, _w = PR.site_map((S.L, S.W), names, points={kk: list(v) for kk, v in S.points.items()}, keepouts=S.keepouts)
    mp = os.path.join(folder, "sim", "map.json")
    PR.save_map(m, mp)
    starts = {n: list(S.points[p]) + [90.0] for n, p in zip(names, ("P11", "P05", "P22"))}
    with open(os.path.join(folder, "sim", "starts.json"), "w") as f:
        json.dump(starts, f)
    cmd = [sys.executable, os.path.join(TOOLS, "sim", "fleet_sim.py"), "--map", mp, "--out",
           os.path.join(folder, "sim"), "--fix-hz", "0", "--starts", os.path.join(folder, "sim", "starts.json"),
           "--max-s", "300", "--drop-cm", "74", "--quiet"]
    for s in specs:
        cmd += ([s] if s.startswith("--") else ["--asleep", s])
    log = open(os.path.join(folder, "sim", "fleet_sim.txt"), "w")
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=REPO)
    time.sleep(2.5)
    import robot_sleep as RS
    try:
        if rs_args[:1] == ["cmds"]:
            rc = cmds(rs_args[1:])
        else:
            rc = RS.main(rs_args)
    finally:
        p.terminate()
        p.wait(timeout=10)
    print("RC=%d" % rc, flush=True)


if __name__ == "__main__":
    main()
