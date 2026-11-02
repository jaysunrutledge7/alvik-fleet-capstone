#!/usr/bin/env python3
"""Site profiles (conference-table build, Nick 2026-11-02): everything that differs between the home lab and a school
room, chosen by one word instead of edits in the code. Home stays the default and is exactly what ran before.

A site (tools/fleet/sites/<name>.json) says:
  robots        each robot's name (the name saved on the robot with the console's `name` command) and, optionally,
                its fixed WiFi address; robots without an address are found on the WiFi by their own hello (broadcast)
  pi            the Pi's addresses to show on screens (its Tailscale and WiFi names)
  conf_dir      where this site's calibrations, camera names, stored driving corrections and day records live
                (home: ~/.config/mirlt, as before; a school: ~/.config/mirlt/sites/<name>), so nothing measured at
                school ever overwrites home's and the reverse
  mc_dir        Mission Control's folder (workspace, production points, programs, runs)
  cameras       each camera role (the recorder's camera 0 is "overhead", camera 1 "side") with a label for the screens
  capture       the recorder's picture size and rate
  tracking      "lights" (home: robots' lights and outlines, cam_bridge.py) or "tags" (AprilTags on the roofs,
                tag_bridge.py)
  tags          family, printed size, roof size and height, each robot's tag id, spares
  table         the table's size when it is known (typed: a check on the measured one), else null

Which site is used: MIRLT_SITE in the environment, else the one saved by `sites.py use <name>` (the file
~/.config/mirlt/site), else home.

    python3 tools/fleet/sites.py                 # the site in use, in words
    python3 tools/fleet/sites.py list            # every site
    python3 tools/fleet/sites.py use school-conference
    python3 tools/fleet/sites.py get robot_args  # e.g. "--robot 192.168.1.250 ..." (for the launchers)
    python3 tools/fleet/sites.py env             # export lines for the launchers (MIRLT_CONF_DIR, ...)

No WiFi password is ever stored here or anywhere in this repo: the robots' networks are typed by Nick into the
firmware's secrets.h (git-ignored) on his laptop.
"""
import copy
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SITES_DIR = os.path.join(HERE, "sites")
BASE_CONF = "~/.config/mirlt"
DEFAULT_SITE = "home"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,10}$")    # the firmware keeps at most 10 characters (net.h setName)

ROBOT_COLOURS = ["#2952CC", "#C2410C", "#7A3EC8", "#12805C", "#B54708", "#0E7490"]


def base_conf_dir():
    return os.path.expanduser(BASE_CONF)


def chosen_file():
    return os.path.join(base_conf_dir(), "site")


def names():
    out = []
    for f in sorted(os.listdir(SITES_DIR)):
        if f.endswith(".json"):
            out.append(f[:-5])
    return out


def current_name():
    env = os.environ.get("MIRLT_SITE", "").strip()
    if env:
        return env
    try:
        with open(chosen_file()) as f:
            v = f.read().strip()
        if v:
            return v
    except OSError:
        pass
    return DEFAULT_SITE


def _check(d, name):
    probs = []
    seen_ids = {}
    for r in d.get("robots", []):
        if not NAME_RE.match(str(r.get("name", ""))):
            probs.append("robot name %r: 1-10 letters, digits, - or _ (the firmware keeps 10)" % r.get("name"))
        ip = r.get("ip")
        if ip and not re.match(r"^\d{1,3}(\.\d{1,3}){3}$", str(ip)):
            probs.append("robot %s: address %r is not like 192.168.1.40" % (r.get("name"), ip))
    tags = d.get("tags") or {}
    for n, i in (tags.get("ids") or {}).items():
        if i in seen_ids.values():
            probs.append("tag id %s used twice" % i)
        seen_ids[n] = i
    if d.get("tracking") == "tags":
        for r in d.get("robots", []):
            if r.get("name") not in (tags.get("ids") or {}):
                probs.append("robot %s has no tag id (tags.ids)" % r.get("name"))
    if probs:
        raise ValueError("site %s: %s" % (name, "; ".join(probs)))


def load(name=None):
    """The site as a dict, with defaults filled in and paths expanded (conf_dir, mc_dir absolute)."""
    name = name or current_name()
    path = os.path.join(SITES_DIR, name + ".json")
    if not os.path.exists(path):
        raise SystemExit("No site called %r (sites: %s). Make one: copy tools/fleet/sites/school-conference.json to "
                         "tools/fleet/sites/%s.json and edit it." % (name, ", ".join(names()), name))
    with open(path) as f:
        d = json.load(f)
    d = copy.deepcopy(d)
    d["name"] = name
    d.setdefault("title", name)
    d.setdefault("tracking", "lights")
    d.setdefault("robots", [])
    d.setdefault("pi", {})
    d.setdefault("cameras", {"overhead": {"label": "overhead camera"}, "side": {"label": "side camera"}})
    d.setdefault("capture", {"width": 640, "height": 480, "fps": 25})
    d.setdefault("table", None)
    cd = d.get("conf_dir") or (BASE_CONF if name == DEFAULT_SITE else BASE_CONF + "/sites/" + name)
    d["conf_dir"] = os.path.expanduser(cd)
    md = d.get("mc_dir") or ("logs/mission-control" if name == DEFAULT_SITE else "logs/mission-control-" + name)
    d["mc_dir"] = md if os.path.isabs(md) else os.path.join(REPO, md)
    for k, r in enumerate(d["robots"]):
        r.setdefault("colour", ROBOT_COLOURS[k % len(ROBOT_COLOURS)])
    _check(d, name)
    return d


def use(name):
    load(name)       # it must exist and be valid
    os.makedirs(base_conf_dir(), exist_ok=True)
    with open(chosen_file(), "w") as f:
        f.write(name + "\n")
    return name


def robot_names(site=None):
    s = site or load()
    return [r["name"] for r in s["robots"]]


def robot_args(site=None):
    """'--robot IP ...' for the robots with a fixed address ('' when none: they are found by their hello)."""
    s = site or load()
    return " ".join("--robot %s" % r["ip"] for r in s["robots"] if r.get("ip"))


def tag_ids(site=None):
    s = site or load()
    return dict((s.get("tags") or {}).get("ids") or {})


def colours(site=None):
    s = site or load()
    return {r["name"]: r["colour"] for r in s["robots"]}


def env_lines(site=None):
    """Shell export lines the launchers eval: the site's folders and robots."""
    s = site or load()
    out = ["export MIRLT_SITE=%s" % s["name"], "export MIRLT_CONF_DIR=%s" % _q(s["conf_dir"]),
           "export MIRLT_MC_DIR=%s" % _q(s["mc_dir"]), "export MIRLT_TRACKING=%s" % s["tracking"],
           "export MIRLT_ROBOTS=%s" % _q(",".join(robot_names(s))),
           "export MIRLT_ROBOT_ARGS=%s" % _q(robot_args(s)),
           "export MIRLT_CAPTURE=%s" % _q("--width %d --height %d" % (s["capture"]["width"], s["capture"]["height"])),
           "export MIRLT_CAPTURE_FPS=%d" % int(s["capture"].get("fps", 25)),
           "export MIRLT_PI_ADDR=%s" % _q(s["pi"].get("address", ""))]
    return out


def _q(v):
    v = str(v)
    return "'" + v.replace("'", "'\"'\"'") + "'"


def words(site=None):
    s = site or load()
    rb = ", ".join("%s%s" % (r["name"], " at " + r["ip"] if r.get("ip") else " (found on the WiFi)") for r in s["robots"])
    lines = ["Site: %s (%s)" % (s["name"], s["title"]),
             "  robots: %s" % (rb or "none listed"),
             "  tracking: %s" % ("AprilTags on the roofs" if s["tracking"] == "tags" else "the robots' lights and outlines"),
             "  cameras: %s" % "; ".join("%s = %s" % (k, v.get("label", k)) for k, v in s["cameras"].items()),
             "  pictures: %d x %d at %d a second" % (s["capture"]["width"], s["capture"]["height"],
                                                    s["capture"].get("fps", 25)),
             "  calibrations and camera names: %s" % s["conf_dir"],
             "  Mission Control's folder: %s" % os.path.relpath(s["mc_dir"], REPO)]
    if s.get("table"):
        lines.append("  table (typed): %s x %s cm" % (s["table"].get("length_cm"), s["table"].get("width_cm")))
    if s["tracking"] == "tags":
        t = s.get("tags") or {}
        lines.append("  tags: %s, %.1f cm black square on a %.1f cm roof, %s cm above the table; ids %s, spares %s" % (
            t.get("family"), t.get("tag_cm", 0), t.get("roof_cm", 0),
            t.get("height_cm") if t.get("height_cm") is not None else "NOT MEASURED YET",
            ", ".join("%s=%s" % kv for kv in (t.get("ids") or {}).items()), t.get("spares")))
    return "\n".join(lines)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    cmd = argv[0] if argv else "show"
    if cmd == "show":
        print(words())
    elif cmd == "list":
        cur = current_name()
        for n in names():
            s = load(n)
            print("%s %-20s %s" % ("*" if n == cur else " ", n, s["title"]))
    elif cmd == "use":
        if len(argv) < 2:
            raise SystemExit("Which site? python3 tools/fleet/sites.py use <name> (sites: %s)" % ", ".join(names()))
        use(argv[1])
        print("Site now: %s. Every launcher and Mission Control opened from now on uses it." % argv[1])
        print(words(load(argv[1])))
    elif cmd == "get":
        key = argv[1] if len(argv) > 1 else ""
        s = load()
        if key == "robot_args":
            print(robot_args(s))
        elif key == "robots":
            print(",".join(robot_names(s)))
        elif key in ("conf_dir", "mc_dir", "tracking", "name"):
            print(s[key])
        else:
            print(json.dumps(s.get(key), indent=1))
    elif cmd == "env":
        print("\n".join(env_lines()))
    else:
        raise SystemExit(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main())
