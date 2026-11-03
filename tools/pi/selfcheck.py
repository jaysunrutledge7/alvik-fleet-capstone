#!/usr/bin/env python3
"""Self-check for a Pi set up from a blank SD card (2026-11-03, risk review #5): everything the school site's tools
need, tested, one line each: PASS, WARN (works, but look at it) or FAIL (fix it before the session).

    ./run-fleet-pi.sh selfcheck              (the same as the line below)
    python3 tools/pi/selfcheck.py
    python3 tools/pi/selfcheck.py --no-net   (skip the checks that need the internet: GitHub, the token)

What it checks, in order: the system (Raspberry Pi OS, Python), the programs (git, tmux, v4l2-ctl, Tailscale), every
Python package the school site's tools import (found by scanning their imports, 3 Nov: numpy, scipy, OpenCV with its
aruco module, matplotlib; pandas for the heat tracker; pupil-apriltags only when OpenCV lacks aruco), the tag detector
on a drawn tag (the robots' tag25h9 and the reference tags), the repository (a git clone, its remote, reachable), the
site in use (loads, roof height measured, reference tags), the GitHub token for uploads (there, only you can read it,
GitHub accepts it: the token itself is never printed), Tailscale, the clock (NTP), the cameras (found by name, the
site's picture size offered, their stable /dev/v4l/by-id names), the disk (USB drive, hours of video that fit), and
power and heat (vcgencmd). Exit 1 when any line FAILs. Changes nothing.
"""
import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

TOKEN_PATH = os.path.expanduser("~/.config/mirlt/github_token")
# the Python packages the school site's tools import (scanned 2026-11-03 from mission_control, mission_run, run_plan,
# tag_bridge, camrec, camera_checkout, site_fit, tag_setup, feel_table, sync_tools, merge_external, upload_session and
# what they import): (import name, apt package, needed for)
PACKAGES = [
    ("numpy", "python3-numpy", "everything"),
    ("scipy", "python3-scipy", "the site fit, the planner"),
    ("cv2", "python3-opencv", "cameras, tags, the recorder"),
    ("matplotlib", "python3-matplotlib", "charts (feel-out, comparison)"),
]
OPTIONAL = [
    ("pandas", "python3-pandas", "the heat tracker (analysis)"),
]


class Report:
    def __init__(self, out=print):
        self.rows, self.out = [], out

    def add(self, verdict, what, words):
        self.rows.append((verdict, what, words))
        self.out("%-4s  %-22s %s" % (verdict, what, words))

    def counts(self):
        return {v: sum(1 for r in self.rows if r[0] == v) for v in ("PASS", "WARN", "FAIL")}


def run(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except (OSError, subprocess.SubprocessError) as e:
        return 127, str(e)


def os_release():
    try:
        with open("/etc/os-release") as f:
            d = dict(l.strip().split("=", 1) for l in f if "=" in l)
        return d.get("PRETTY_NAME", "?").strip('"')
    except OSError:
        return "?"


def check_system(R):
    R.add("PASS" if sys.version_info >= (3, 9) else "FAIL", "Python", "%s (%s)" % (sys.version.split()[0], os_release()))
    for prog, apt, why in (("git", "git", "the repository"), ("tmux", "tmux", "runs that survive a closed window"),
                           ("v4l2-ctl", "v4l-utils", "camera sizes and focus")):
        R.add("PASS" if shutil.which(prog) else "FAIL", prog, "found" if shutil.which(prog) else
              "missing: sudo apt install -y %s (%s)" % (apt, why))


def check_packages(R):
    for mod, apt, why in PACKAGES + OPTIONAL:
        try:
            m = __import__(mod)
            R.add("PASS", mod, "version %s (%s)" % (getattr(m, "__version__", "?"), why))
        except Exception as e:      # noqa: BLE001
            R.add("FAIL" if (mod, apt, why) in PACKAGES else "WARN", mod,
                  "cannot import (%s): sudo apt install -y %s (%s)" % (str(e).splitlines()[0][:60], apt, why))


def check_tags(R):
    try:
        import cv2
    except ImportError:
        R.add("FAIL", "tag detector", "no OpenCV")
        return
    import tagcore as TC
    ver = cv2.__version__
    has_aruco = hasattr(cv2, "aruco")
    major_minor = tuple(int(x) for x in ver.split(".")[:2] if x.isdigit())
    note = ("tested: 4.6 (Raspberry Pi OS Bookworm) and 5.0; 4.7+ uses the newer ArucoDetector call"
            if major_minor not in ((4, 6), (5, 0)) else "a tested version")
    R.add("PASS" if has_aruco else "WARN", "OpenCV aruco", "OpenCV %s, aruco %s (%s)" % (
        ver, "present" if has_aruco else "MISSING: the fallback is pupil-apriltags: "
        "pip3 install --break-system-packages pupil-apriltags", note))
    try:
        import numpy as np
        fam = TC.TAG_FAMILY
        ok = []
        for tid in (0, 1, 20):           # the robots' tags and a reference tag
            img = TC.tag_image(fam, tid, px_per_cell=10, quiet_cells=2)
            img = cv2.copyMakeBorder(img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
            det = TC.TagDetector(fam)
            got = [d.id for d in det.detect(img)]
            ok.append(tid in got)
        R.add("PASS" if all(ok) else "FAIL", "tag detector", "%s ids 0, 1 (robots) and 20 (a reference tag) read from "
              "drawn tags with the %s backend" % (fam, det.backend) if all(ok) else
              "drawn tags not read (%s): the detector does not work here" % ok)
    except Exception as e:      # noqa: BLE001
        R.add("FAIL", "tag detector", "error: %s" % str(e).splitlines()[0][:100])


def check_repo(R, net=True):
    rc, out = run(["git", "-C", REPO, "rev-parse", "--short", "HEAD"])
    if rc != 0:
        R.add("FAIL", "repository", "%s is not a git clone: git clone https://github.com/nfmaxfield-cmd/"
              "alvik-fleet-capstone.git" % REPO)
        return
    head = out.strip()
    rc, url = run(["git", "-C", REPO, "remote", "get-url", "origin"])
    R.add("PASS", "repository", "%s at %s (origin %s)" % (REPO, head, url.strip() or "?"))
    if net:
        rc, out = run(["git", "-C", REPO, "ls-remote", "--heads", "origin", "main"], timeout=20)
        R.add("PASS" if rc == 0 and out.strip() else "WARN", "GitHub reachable",
              "git pull will work" if rc == 0 and out.strip() else
              "could not reach GitHub (%s): no internet, or the repo went private (then the clone needs a token: "
              "see the setup steps)" % out.strip().splitlines()[-1][:80] if out.strip() else "no answer")


def check_site(R):
    try:
        import sites as SI
        s = SI.load()
    except BaseException as e:      # noqa: BLE001
        R.add("FAIL", "site", str(e).splitlines()[0][:160])
        return None
    R.add("PASS", "site", "%s: %s, %dx%d pictures, robots %s" % (
        s["name"], s["tracking"], s["capture"]["width"], s["capture"]["height"],
        ", ".join(r["name"] for r in s["robots"]) or "none"))
    if s["tracking"] == "tags":
        h = (s.get("tags") or {}).get("height_cm")
        R.add("PASS" if h is not None else "WARN", "roof height", "%s cm" % h if h is not None else
              "not measured yet (6.0 cm placeholder): measure table to tag face, send it to Claude")
        rt = s.get("ref_tags") or {}
        import ref_tags as RT
        R.add("PASS" if rt.get("use", True) and rt else "WARN", "reference tags",
              "%s ids %s, %s cm; corner clicking %s" % (rt.get("family", "tag25h9"),
                                                      ", ".join(str(i) for i in RT.ids(rt)), rt.get("tag_cm"),
                                                      "ON" if s.get("corner_clicks", True) else "off")
              if rt else "none in the site profile (corners are clicked by hand)")
    return s


def check_token(R, net=True):
    if not os.path.exists(TOKEN_PATH):
        R.add("WARN", "GitHub token", "none at %s: uploads will not work (runs still do). Make one: "
              "python3 tools/pi/upload_session.py --set-token" % TOKEN_PATH)
        return
    mode = os.stat(TOKEN_PATH).st_mode & 0o777
    if mode & 0o077:
        R.add("WARN", "GitHub token", "the file can be read by other users (%o): chmod 600 %s" % (mode, TOKEN_PATH))
    if not net:
        R.add("PASS", "GitHub token", "saved (not checked online: --no-net)")
        return
    rc, out = run([sys.executable, os.path.join(HERE, "upload_session.py"), "--check-token"], timeout=30)
    line = (out.strip().splitlines() or ["?"])[-1]
    if "github_pat_" in line or "ghp_" in line:
        line = "(answer hidden)"          # never print anything that looks like a token
    R.add("PASS" if rc == 0 else "FAIL", "GitHub token", line[:160])


def check_tailscale(R):
    if not shutil.which("tailscale"):
        R.add("WARN", "Tailscale", "not installed: the laptop must be on the same WiFi as the Pi (or install it: "
              "curl -fsSL https://tailscale.com/install.sh | sh)")
        return
    rc, out = run(["tailscale", "ip", "-4"])
    ip = out.strip().split()[0] if rc == 0 and out.strip() else ""
    R.add("PASS" if ip.startswith("100.") else "WARN", "Tailscale", ("up: %s" % ip) if ip.startswith("100.") else
          "installed but not logged in or not up: sudo tailscale up")


def check_clock(R):
    rc, out = run(["timedatectl", "show", "-p", "NTPSynchronized", "-p", "Timezone"])
    if rc != 0:
        R.add("WARN", "clock", "timedatectl not available")
        return
    d = dict(l.split("=", 1) for l in out.strip().splitlines() if "=" in l)
    sync = d.get("NTPSynchronized") == "yes"
    R.add("PASS" if sync else "WARN", "clock", "NTP synchronized %s, time zone %s" % (
        "yes" if sync else "NO (the marks with the RFID team rely on it: check the internet)", d.get("Timezone", "?")))


def check_cameras(R, site=None):
    import camnames as CN
    cams = CN.scan()
    if not cams:
        R.add("FAIL", "cameras", "no USB camera found: plug them in (the blue USB 3 ports work best)")
        return
    rep = CN.report(cams=cams)
    want = (site or {}).get("capture") or {"width": 640, "height": 480}
    byid = CN.by_id_names()
    for c in cams:
        fmt_ok = None
        if shutil.which("v4l2-ctl"):
            rc, out = run(["v4l2-ctl", "-d", "/dev/video%d" % c["dev"], "--list-formats-ext"])
            fmt_ok = "%dx%d" % (want["width"], want["height"]) in out
        name = byid.get(c["dev"])
        R.add("PASS" if fmt_ok is not False else "WARN", "camera /dev/video%d" % c["dev"],
              "%s, USB port %s%s; %s%s" % (c["name"], c["port"] or "?", (" <- " + c["role"]) if c.get("role") else "",
                                           "offers %dx%d" % (want["width"], want["height"]) if fmt_ok else
                                           "does NOT list %dx%d (look: v4l2-ctl -d /dev/video%d --list-formats-ext)" % (
                                               want["width"], want["height"], c["dev"]) if fmt_ok is False else
                                           "sizes not checked (no v4l2-ctl)",
                                           "; stable name /dev/v4l/by-id/%s" % name if name else ""))
    for r, v in rep["roles"].items():
        R.add("PASS" if v["ok"] else "FAIL", "camera role %s" % r, v["words"] if v["ok"] else
              v["words"] + ". Name it: python3 tools/pi/camnames.py --set %s \"<name>\" <port>" % r)


def check_disk(R, site=None):
    import diskguard as DG
    d = DG.drive_mount()
    env = dict(os.environ)
    if site and not env.get("MIRLT_CAPTURE"):
        # run by hand (not through the launcher): the site's picture size and rate, as the recorder will use them
        cap = site.get("capture") or {}
        env["MIRLT_CAPTURE"] = "--width %d --height %d" % (cap.get("width", 640), cap.get("height", 480))
        env["MIRLT_CAPTURE_FPS"] = str(cap.get("fps", 25))
    c = DG.free_gb(os.path.join(REPO, "logs"))
    R.add("PASS" if (c or 0) >= 5 else "WARN" if (c or 0) >= DG.CARD_STOP_GB else "FAIL", "SD card",
          "%.1f GB free" % (c or 0))
    line, low = DG.rec_words(d, env)
    R.add("WARN" if (low or not d) else "PASS", "video space", line)


def check_power(R):
    if not shutil.which("vcgencmd"):
        R.add("WARN", "power and heat", "vcgencmd not found (not a Raspberry Pi?)")
        return
    rc, t = run(["vcgencmd", "measure_temp"])
    rc2, th = run(["vcgencmd", "get_throttled"])
    th = th.strip()
    ok = th.endswith("=0x0")
    R.add("PASS" if ok else "WARN", "power and heat", "%s, %s%s" % (t.strip(), th, "" if ok else
          " (not 0x0: under-voltage or heat since boot: the official 5.1 V 3 A supply and a fan case)"))


def check_address(R):
    import pi_addr as PA
    a, how = PA.detect()
    R.add("PASS", "address", "the laptop's browser: http://%s:8095 for Mission Control (%s)" % (a, how))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-net", action="store_true", help="skip the checks that need the internet")
    a = ap.parse_args(argv)
    R = Report()
    print("Self-check of this Pi for the MIRLT tools (%s)" % REPO)
    for f in (check_system, check_packages, check_tags):
        try:
            f(R)
        except Exception as e:      # noqa: BLE001
            R.add("FAIL", f.__name__[6:], "the check itself failed: %s" % str(e).splitlines()[0][:100])
    try:
        check_repo(R, net=not a.no_net)
    except Exception as e:      # noqa: BLE001
        R.add("FAIL", "repository", str(e)[:100])
    site = None
    try:
        site = check_site(R)
    except Exception as e:      # noqa: BLE001
        R.add("FAIL", "site", str(e)[:100])
    for f in (lambda: check_token(R, net=not a.no_net), lambda: check_tailscale(R), lambda: check_clock(R),
              lambda: check_cameras(R, site), lambda: check_disk(R, site), lambda: check_power(R), lambda: check_address(R)):
        try:
            f()
        except Exception as e:      # noqa: BLE001
            R.add("FAIL", "check", "a check failed to run: %s" % str(e).splitlines()[0][:100])
    n = R.counts()
    print("")
    print("%d PASS, %d WARN, %d FAIL. %s" % (n["PASS"], n["WARN"], n["FAIL"], "Ready." if not n["FAIL"] else
                                             "Fix every FAIL line (each says how), then run this again."))
    return 1 if n["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main())
