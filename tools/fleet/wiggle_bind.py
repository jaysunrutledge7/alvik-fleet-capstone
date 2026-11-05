#!/usr/bin/env python3
"""Wiggle-first naming before EVERY run (Nick, 5 Nov 2026: "wiggle first then attach the robot that wiggled to its
corresponding number"). Before a run with roof tags:

  1. find    every robot on the WiFi, at whatever address it answers from (`wake` + `status` to the broadcast and to
             each robot heard; a sleeping robot answers at its check-in), its compass check if it restarted;
  2. wiggle  each robot in turn does the sync wiggle (a quarter turn and back); the roof tag that turns with it is its
             tag (calib_run.bind_by_wiggle: refused in plain words when no tag or two tags turn);
  3. slots   each robot is given one of the run's robot slots (Alpha / Bravo, Left / Right ...): a robot the site pins
             (robots[].wifi = its WiFi name) gets that slot; the others by where the camera sees them, the slot whose
             home is nearest (tags.slot_by "position", the default) or keeping a name that already is a slot
             (tags.slot_by "name");
  4. name    a robot whose name is not its slot is renamed with `name <slot>` (kept on the robot; temporary names first
             when two robots swap); tags.rename_to_slot false: not renamed, the run then stops if a slot has no robot
             by that name;
  5. record  <conf_dir>/robot_binding.json and the session's binding.json: slot -> tag id, WiFi name, address. The run's
             tag bridge then follows those tag ids (instead of the site's typed tags.ids).

The site's tags.binding switch: "wiggle" (the default since 5 Nov) or "fixed" (the typed tags.ids, the old way, kept).

    python3 tools/fleet/wiggle_bind.py --robots Alpha,Bravo                    # on the Pi (mission_run calls it too)
    python3 tools/fleet/wiggle_bind.py --robots Alpha,Bravo --dry              # the simulator
"""
import argparse
import datetime as dt
import itertools
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import calib_run as CR   # noqa: E402

VERSION = "2026-11-05a"
say = CR.say


def binding_mode(site, override="auto"):
    """'wiggle' or 'fixed': --binding, else the site's tags.binding, else wiggle."""
    if override and override != "auto":
        return override
    return str(((site or {}).get("tags") or {}).get("binding") or "wiggle")


def slot_homes(site, S, slots):
    """{slot: (x, y)} from the site's homes (a production point per slot), where known."""
    hp = (site or {}).get("homes") or {}
    out = {}
    for sl in slots:
        lab = hp.get(sl)
        if isinstance(lab, str) and lab in S.points:
            out[sl] = tuple(S.points[lab])
    return out


def assign_slots(slots, robots, seen, homes, pins=None, by="position"):
    """{robot name: slot}. robots: names as heard; seen: {name: (x, y)} from the camera; homes: {slot: (x, y)};
    pins: {robot name: slot} from the site. Raises CR.BindError in plain words when it cannot."""
    pins = dict(pins or {})
    if len(robots) > len(slots):
        raise CR.BindError("%d robots answered (%s) but the run has %d robot slots (%s): switch the extra one off or "
                           "add a slot" % (len(robots), ", ".join(robots), len(slots), ", ".join(slots)))
    out = {r: pins[r] for r in robots if pins.get(r) in slots}
    free_s = [s for s in slots if s not in out.values()]
    free_r = [r for r in robots if r not in out]
    if by == "name":
        for r in list(free_r):
            if r in free_s:
                out[r] = r
                free_s.remove(r)
                free_r.remove(r)
    if free_r and all(s in homes for s in free_s) and all(r in seen for r in free_r):
        best = None
        for perm in itertools.permutations(free_s, len(free_r)):
            c = sum(math.hypot(seen[r][0] - homes[s][0], seen[r][1] - homes[s][1]) for r, s in zip(free_r, perm))
            if best is None or c < best[0]:
                best = (c, perm)
        out.update(dict(zip(free_r, best[1])))
    else:
        for r in free_r:                       # no homes or no positions: keep a slot name, else in order
            s = r if r in free_s else free_s[0]
            out[r] = s
            free_s.remove(s)
    return out


def rename_steps(current, assign):
    """[(ip, old name, new name)] so that no two robots ever share a name: robots whose new name another robot still
    has are first given a temporary name (t-<last part of the address>)."""
    move = {r: s for r, s in assign.items() if r != s}
    if not move:
        return []
    steps = []
    names = dict(current)        # name -> ip
    clash = [r for r, s in move.items() if s in names and s != r]
    tmp = {}
    for r in list(move):
        if clash:
            t = "t-%s" % current[r].split(".")[-1]
            steps.append((current[r], r, t))
            tmp[r] = t
    for r, s in move.items():
        steps.append((current[r], tmp.get(r, r), s))
    return steps


def rename(link, steps, log=say):
    """Send each `name`, waiting for its answer ("<old> is now called <new>"). Returns the steps that answered."""
    done = []
    for ip, old, new in steps:
        ok = None
        for _ in range(3):
            k = link.mark()
            link.send(ip, "name %s" % new)
            ok = link.wait_for(lambda n, l: l if "is now called %s" % new in l else None, k, 3.0)
            if ok:
                break
        if not ok:
            raise CR.BindError("the robot at %s did not answer `name %s` (it was %s): not renamed; nothing ran"
                               % (ip, new, old))
        log("  %s at %s is now called %s" % (old, ip, new))
        done.append((ip, old, new))
    return done


def bind_for_run(site, slots, link, eye, ips=(), wake_min=3.0, dry=False, folder=None, log=say, by=None,
                 rename_ok=None):
    """Steps 1-5 with a Link and an Eye already open. Returns the record (dict): slots {slot: tag}, robots [...]."""
    S = CR.Site(site)
    tg = (site or {}).get("tags") or {}
    by = by or str(tg.get("slot_by") or "position")
    rename_ok = bool(tg.get("rename_to_slot", True)) if rename_ok is None else rename_ok
    found = CR.discover(link, list(ips), wake_min, dry, count=len(slots))
    if not found:
        raise CR.BindError("no robot answered on the WiFi (wake and status to %s for %.0f min). Is it switched on and "
                           "on the school WiFi? `python3 tools/pi/robot_sleep.py wake any` wakes one from any address."
                           % (", ".join(ips) or "the broadcast", wake_min))
    log("Found: %s" % ", ".join("%s at %s" % kv for kv in found.items()))
    robots = [CR.Robot(link, n, ip) for n, ip in found.items()]
    for rb in robots:
        link.send(rb.ip, "wake")
        for c in CR.EDGE_GUARD:
            link.send(rb.ip, c)
            time.sleep(0.05)
    for rb in robots:
        CR.compass_check(link, rb, log=log)
    import experiments as EX
    w = EX.conf(site)["sync_wiggle"]
    time.sleep(1.0)
    b = CR.bind_by_wiggle(robots, eye, wiggle_deg=float(w["deg"]), pause_s=float(w["pause_s"]), log=log)
    seen = {}
    for n, t in b.items():
        m = eye.measure(t, 0.6)
        if m:
            seen[n] = (m[0], m[1])
    pins = {}
    for r in (site or {}).get("robots") or []:
        if r.get("wifi") and r["wifi"] in found:
            pins[r["wifi"]] = r["name"]
    assign = assign_slots(list(slots), list(found), seen, slot_homes(site, S, slots), pins, by)
    log("Slots (%s): %s" % ("pinned in the site, then nearest home" if by == "position" else
                            "pinned, then names kept", ", ".join("%s -> %s" % kv for kv in assign.items())))
    steps = rename_steps(found, assign)
    renamed = []
    if steps and not rename_ok:
        raise CR.BindError("these robots are not called by their slot and renaming is switched off "
                           "(tags.rename_to_slot false): %s" % ", ".join("%s should be %s" % (r, s) for r, s in
                                                                         assign.items() if r != s))
    if steps:
        renamed = rename(link, steps, log=log)
    wifi = {}
    for rb in robots:            # the WiFi name (firmware 2026-11-05a `id`; older: the name it answered with)
        k = link.mark()
        link.send(rb.ip, "id")
        r = link.wait_for(lambda n, l: l if l.startswith("M,id ") else None, k, 1.5)
        m = re.search(r" wifi (\S+)", r or "")
        wifi[rb.ip] = m.group(1) if m else (rb.name if rb.name.startswith("AL-") else None)
    rec = {"how": "wiggle (wiggle_bind.py %s)" % VERSION, "time": dt.datetime.now().isoformat(timespec="seconds"),
           "slots": {assign[n]: int(t) for n, t in b.items()},
           "robots": [{"slot": assign[rb.name], "name_before": rb.name, "wifi_name": wifi.get(rb.ip), "ip": rb.ip,
                       "tag_id": b.get(rb.name), "seen_cm": seen.get(rb.name)} for rb in robots],
           "renamed": [{"ip": ip, "from": o, "to": n} for ip, o, n in renamed],
           "empty_slots": [s for s in slots if s not in assign.values()]}
    for p in ([os.path.join(folder, "binding.json")] if folder else []) + \
            ([os.path.join(site["conf_dir"], "robot_binding.json")] if (site.get("conf_dir") and not dry) else []):
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            json.dump(rec, f, indent=1)
    log("Bound: %s%s" % (", ".join("%s = tag %d" % kv for kv in sorted(rec["slots"].items())),
                         ("; no robot for %s" % ", ".join(rec["empty_slots"])) if rec["empty_slots"] else ""))
    return rec


def tags_with_binding(tags, rec):
    """The run's tags block with the wiggle's ids (slot -> tag) in place of the typed tags.ids."""
    t = dict(tags or {})
    t["ids_typed"] = dict(t.get("ids") or {})
    t["ids"] = dict(rec["slots"])
    t["binding"] = "wiggle"
    return t


def run_for_mission(site, slots, folder, stream="127.0.0.1:8000", calib="", ips=(), log=say):
    """mission_run's step before its bridge (real robots): a tag bridge of its own on the recorder's pictures with
    every listed and spare tag id, the binding, the bridge stopped. Returns the record."""
    import argparse
    os.makedirs(folder, exist_ok=True)
    a = argparse.Namespace(calib=calib, stream=stream)
    bp, _cal, _h = CR.start_bridge(a, site, folder)
    link = eye = None
    try:
        link = CR.Link(folder)
        eye = CR.Eye(folder, CR.FIX_PORT)
        time.sleep(2.0)
        return bind_for_run(site, slots, link, eye, ips=ips, folder=folder, log=log)
    finally:
        for o in (eye, link):
            if o is not None:
                o.close()
        if bp.poll() is None:
            bp.send_signal(signal.SIGINT)
            try:
                bp.wait(timeout=10)
            except subprocess.TimeoutExpired:
                bp.kill()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default="")
    ap.add_argument("--robots", default="", help="the run's robot slots (default: the site's robots)")
    ap.add_argument("--robot", action="append", default=[], help="an address to ask too (besides the broadcast)")
    ap.add_argument("--wake-min", type=float, default=12.0)
    ap.add_argument("--slot-by", choices=["position", "name"], default=None)
    ap.add_argument("--session", default="")
    ap.add_argument("--stream", default="127.0.0.1:8000")
    ap.add_argument("--calib", default="")
    g = ap.add_argument_group("simulator")
    g.add_argument("--dry", action="store_true")
    g.add_argument("--sim", action="append", default=[], metavar="NAME:TAG:POINT[:HEADING]",
                   help="a simulated robot (default AL-1DC0:2:P14 and AL-2B7F:3:P11)")
    g.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    import sites as SI
    if a.site:
        os.environ["MIRLT_SITE"] = a.site
    site = SI.load(a.site or None)
    slots = [s.strip() for s in (a.robots or ",".join(r["name"] for r in site.get("robots") or [])).split(",")
             if s.strip()]
    folder = a.session or os.path.join(REPO, "logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-bind" +
                                       ("-dry" if a.dry else ""))
    os.makedirs(folder, exist_ok=True)
    if not a.dry:
        try:
            rec = run_for_mission(site, slots, folder, a.stream, a.calib, a.robot)
        except CR.BindError as e:
            say("NOT BOUND: %s" % e)
            return 1
        print(json.dumps(rec["slots"]))
        return 0
    # the simulator: fleet_sim robots named by their WiFi, each with its tag, standing on stickers
    CR.loopback_up()
    S = CR.Site(site)
    sims = a.sim or ["AL-1DC0:2:P14", "AL-2B7F:3:P11"]
    first = sims[0].split(":")
    sa = argparse.Namespace(sim_name=first[0], sim_also=sims[1:], seed=a.seed, sim_max_s=600, sim_start=first[2],
                            sim_true="1,1,0", sim_true_also="", sim_sleep="0.2:0.05", sim_fw=[])
    sp, ips = CR.start_sim(sa, S, folder)
    link = CR.Link(folder)
    cam = CR.SchoolCamera()
    tags = {q.split(":")[0]: int(q.split(":")[1]) for q in sims}
    eye = CR.SimEye(folder, CR.FIX_PORT + 100, cam, tags, seed=a.seed)
    try:
        time.sleep(1.0)
        rec = bind_for_run(site, slots, link, eye, ips=ips, wake_min=2.0, dry=True, folder=folder, by=a.slot_by)
        print(json.dumps(rec["slots"]))
        return 0
    except CR.BindError as e:
        say("NOT BOUND: %s" % e)
        return 1
    finally:
        eye.close()
        link.close()
        sp.terminate()
        sp.wait(timeout=10)


if __name__ == "__main__":
    sys.exit(main())
