#!/usr/bin/env python3
"""Free the Pi's card (2026-10-28; quick by default since 2026-11-05): delete session folders whose upload to GitHub
is confirmed.

QUICK (the default, Nick 5 Nov: "a much simpler way of checking that uploads are being done, even if its not
airtight"): a session counts as uploaded when its GitHub release exists and has every file its upload recorded
(~/.local/state/mirlt/uploads/<session>.json: names and sizes); with no record, when a release for it exists with at
least one file. Nothing is zipped or read from the folders (seconds, not ~1 min a GB), and it runs at low priority.
A run that is going (tmux freeroam / doe / calib / monday / practice / e02 / ladder, or a robot program such as
Mission Control's run) makes it refuse to start (it would take CPU from the run) unless --force.

    python3 tools/pi/clean_uploaded.py --status        # the last 10 sessions: uploaded or not, and their size here
    python3 tools/pi/clean_uploaded.py --thorough ...  # the exact check below (the old default), kept behind a switch

THOROUGH (--thorough, the old way) - "confirmed" means: a release on GitHub holds this session, and every file the upload made from it is there with
exactly the size the folder makes today. The zips are not kept on the Pi, so each one is rebuilt in memory (nothing
written; the same code as the upload, tools/pi/upload_session.py zip_folder) and only its size is compared: a file
added, removed or changed since the upload changes the size, and the folder is then NOT deleted. Big files (> 50 MB)
went up as parts: every part must be there, adding up to the file's size.

What it looks at:
  - every session folder under logs/ (logs/<date-time>-..., logs/e02-...);
  - its frames folder (frames_where.txt: the USB drive's fleet-frames/<session>/frames; or <session>/frames);
  - the upload's own zip copies in .upload-tmp (on the drive, or ~/.upload-tmp): a copy whose name and exact size are
    on GitHub.
Anything not confirmed is listed with the reason and never deleted (dry runs are never uploaded, so never deleted).

    python3 tools/pi/clean_uploaded.py                 # the list only (nothing deleted)
    python3 tools/pi/clean_uploaded.py --yes           # delete what the list marks DELETE
    python3 tools/pi/clean_uploaded.py --frames-only   # keep every logs folder; only frames and zip copies
run-fleet-pi.sh clean-uploaded [--yes] [--frames-only] [--thorough] [--force] runs this; run-fleet-pi.sh uploads runs
--status. After each upload, upload_session.py calls free_after_upload() here (auto-clean, on at home). The GitHub token (~/.config/mirlt/github_token) is
used if it is there (more requests an hour); the repository is public, so it also works without one. It is never shown.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

import upload_session as US   # noqa: E402
import diskguard as DG        # noqa: E402

SESSION_RE = re.compile(r"^(\d{8}-\d{6}-.+|e02-.+)$")
RUNNING = ("freeroam", "doe", "calib", "monday", "practice", "e02", "ladder", "ladder2")   # tmux sessions of runs
TEXT_EXT = (".csv", ".json", ".jsonl", ".txt", ".md", ".html", ".log", ".yaml", ".yml", ".tsv")


class Sink:
    """A file that only counts: zipfile writes into it (seeking back to fix headers) and size is the zip's size."""

    def __init__(self):
        self.pos = self.size = 0

    def write(self, b):
        self.pos += len(b)
        self.size = max(self.size, self.pos)
        return len(b)

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else (self.pos + off if whence == 1 else self.size + off)
        return self.pos

    def seekable(self):
        return True

    def flush(self):
        pass


def zip_size(folder, small_only=False, exclude=()):
    s = Sink()
    US.zip_folder(folder, s, small_only=small_only, exclude=tuple(exclude))
    return s.size


def _get(url, tok):
    req = urllib.request.Request(url)
    if tok:
        req.add_header("Authorization", "Bearer " + tok)
    req.add_header("Accept", "application/vnd.github+json")
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read() or b"[]")


def releases(tok=None, get=None):
    """{tag: {asset name: size}} of every release (only assets GitHub reports as fully uploaded)."""
    get = get or _get
    out, page = {}, 1
    while True:
        rows = get("https://api.github.com/repos/%s/releases?per_page=100&page=%d" % (US.REPO_SLUG, page), tok)
        for r in rows:
            out[r["tag_name"]] = {a["name"]: int(a["size"]) for a in r.get("assets", [])
                                  if a.get("state", "uploaded") == "uploaded"}
        if len(rows) < 100:
            return out
        page += 1


def manifest(name):
    try:
        with open(os.path.join(US.STATE_DIR, "uploads", name + ".json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def tags_for(name, rels, man=None):
    """The releases that may hold this session: the manifest's tag first, then any tag ending in the session's name
    (doe-<name>, mission-<name>, p0-<name>, e02-session-<date-time>), with or without -2, -3 (--unique-tag)."""
    core = name[4:] if name.startswith("e02-") else name
    pat = re.compile(r"(^|-)%s(-\d+)?$" % re.escape(core))
    out = []
    if man and man.get("tag") in rels:
        out.append(man["tag"])
    out += sorted(t for t in rels if pat.search(t) and t not in out)
    return out


def parts_ok(folder, assets):
    """Every file over the small-files size went up as <path>.part-aa, -ab, ...: all there, sizes adding up."""
    for root, _, files in os.walk(folder):
        for fn in files:
            p = os.path.join(root, fn)
            size = os.path.getsize(p)
            if size <= US.SMALL_BYTES:
                continue
            stem = os.path.relpath(p, folder).replace(os.sep, "_")
            k, got = 0, 0
            for start in range(0, size, US.PART_BYTES):
                part = "%s.part-%s%s" % (stem, chr(97 + k // 26), chr(97 + k % 26))
                if part not in assets or assets[part] != min(US.PART_BYTES, size - start):
                    return False, "%s not all on GitHub" % stem
                got += assets[part]
                k += 1
            if got != size:
                return False, "%s's parts do not add up" % stem
    return True, ""


def asset_size(assets, name):
    """The size GitHub has for name: the asset itself, or (2026-10-29: zips over 1.9 GB go up in parts) the sum of
    name.part-aa, -ab, ... when every part there is, in order. None when it is not there."""
    if name in assets:
        return assets[name]
    total, k = 0, 0
    while True:
        part = "%s.part-%s%s" % (name, chr(97 + k // 26), chr(97 + k % 26))
        if part not in assets:
            break
        total += assets[part]
        k += 1
    return total if k else None


def check_frames(fd, assets, say=print):
    """(confirmed, why) for a frames folder against one release's assets."""
    name = "rec-%s-small.zip" % os.path.basename(fd.rstrip("/"))
    have = asset_size(assets, name)
    if have is None:
        return False, "%s is not in the release" % name
    raw = DG.folder_bytes(fd)
    if raw > 3 * have + 1e6 or raw < have / 3:
        return False, "%s on GitHub is %d bytes; the folder (%d bytes) cannot make that" % (name, have, raw)
    say("    rebuilding %s in memory to compare its size (%.1f GB: about a minute a GB on the Pi) ..." % (
        name, raw / 1e9))
    z = zip_size(fd, small_only=True)
    if z != have:
        return False, "%s: GitHub has %d bytes, the folder makes %d now (changed since the upload)" % (
            name, have, z)
    return parts_ok(fd, assets)


def check_session(name, sess, fd, rels, say=print):
    """What can go for this session: dict(logs, frames, tag, why). logs: its logs folder is confirmed; frames: its
    frames folder is confirmed (True when it has none). Each part may be confirmed by a different release."""
    man = manifest(name)
    tags = tags_for(name, rels, man)
    has_frames = fd is not None and os.path.isdir(fd)
    frames_inside = has_frames and sess is not None and os.path.dirname(os.path.normpath(fd)) == os.path.normpath(sess)
    logs_ok, frames_ok, used, why = False, not has_frames, [], []
    if not tags:
        why.append("no release on GitHub holds it")
    for tag in tags:
        assets = rels[tag]
        if sess is not None and not logs_ok:
            lz = "logs-%s.zip" % name
            lz_size = asset_size(assets, lz)
            if lz_size is None:
                why.append("%s is not in %s" % (lz, tag))
            else:
                excl = [tuple(man["exclude"])] if man.get("tag") == tag and "exclude" in man else [(), ("frames",)]
                for ex in excl:
                    if zip_size(sess, exclude=ex) == lz_size:
                        logs_ok = True
                        used.append(tag)
                        if frames_inside and "frames" not in ex:
                            frames_ok = True          # the frames went up inside the logs zip
                        break
                if not logs_ok:
                    why.append("%s: GitHub has %d bytes, the folder makes another size now (changed since the "
                               "upload)" % (lz, lz_size))
        if has_frames and not frames_ok:
            ok, w = check_frames(fd, assets, say)
            if ok:
                frames_ok = True
                if tag not in used:
                    used.append(tag)
            else:
                why.append(w)
        if (logs_ok or sess is None) and frames_ok:
            break
    return dict(logs=logs_ok, frames=frames_ok, tag=", ".join(used), why="; ".join(why))


def recorded_missing(recorded, assets):
    """The files an upload recorded ({name: size}) that GitHub does not have at that size: [name]."""
    out = []
    for n, size in sorted((recorded or {}).items()):
        if n not in assets or (size is not None and int(assets[n]) != int(size)):
            out.append(n)
    return out


def _inside(p, root):
    rp, rr = os.path.realpath(p), os.path.realpath(root)
    return rp != rr and rp.startswith(rr + os.sep)


def quick_session(name, sess, fd, rels, say=print):
    """The quick check (2026-11-05): dict(logs, frames, tag, why) as check_session, from GitHub's list of files only.
    With the upload's record: its release has every file it sent, at the size sent; a frames folder counts only when
    the upload included it. Without a record: a release for this session exists with at least one file."""
    man = manifest(name)
    tags = tags_for(name, rels, man)
    has_frames = fd is not None and os.path.isdir(fd)
    if not tags:
        return dict(logs=False, frames=not has_frames, tag="", why="no release on GitHub holds it")
    if man.get("tag") in rels:
        tag = man["tag"]
        assets = rels[tag]
        rec = man.get("assets") or {}
        miss = recorded_missing(rec, assets)
        if miss:
            return dict(logs=False, frames=not has_frames, tag="", why="%s is missing %d of the %d files its upload "
                        "sent (or has them at another size): %s" % (tag, len(miss), len(rec), ", ".join(miss[:3])))
        if not assets:
            return dict(logs=False, frames=not has_frames, tag="", why="its release %s has no files" % tag)
        frames_ok = not has_frames
        if has_frames:
            mr = man.get("recording") or ""
            frames_ok = bool(mr and os.path.realpath(mr) == os.path.realpath(fd)) or \
                asset_size(assets, "rec-%s-small.zip" % os.path.basename(fd.rstrip("/"))) is not None or \
                bool(sess and _inside(fd, sess) and man.get("logs") and "frames" not in (man.get("exclude") or []))
        return dict(logs=True, frames=frames_ok, tag=tag,
                    why="" if frames_ok else "its frames folder was not part of the upload (%s)" % tag)
    for t in tags:
        if rels[t]:
            return dict(logs=True, frames=True, tag=t, why="")
    return dict(logs=False, frames=not has_frames, tag="", why="its release (%s) has no files" % ", ".join(tags))


def auto_clean_on():
    """(2026-11-05) Free the big local files after a session's own upload? MIRLT_AUTO_CLEAN=1/0 decides; else the
    site file's "auto_clean_after_upload": true/false; else on at home, off at other sites."""
    v = os.environ.get("MIRLT_AUTO_CLEAN", "").strip().lower()
    if v in ("1", "on", "yes", "true"):
        return True
    if v in ("0", "off", "no", "false"):
        return False
    name = "home"
    try:
        import sites as ST
        name = ST.current_name()
        with open(os.path.join(ST.SITES_DIR, name + ".json")) as f:
            d = json.load(f)
        if isinstance(d.get("auto_clean_after_upload"), bool):
            return d["auto_clean_after_upload"]
    except Exception:     # noqa: BLE001  (no site file: decided by the name alone)
        pass
    return name == "home"


def prune_big(folder, say=print):
    """Delete a folder's big files (videos, pictures, zips, anything over 50 MB); keep its small text files (CSV,
    JSON, TXT, ...) and the folder. Returns (bytes freed, files deleted). Links are never followed or deleted."""
    freed = n = 0
    for root, dirs, files in os.walk(folder, topdown=False):
        for fn in files:
            p = os.path.join(root, fn)
            if os.path.islink(p):
                continue
            try:
                size = os.path.getsize(p)
            except OSError:
                continue
            if fn.lower().endswith(TEXT_EXT) and size <= US.SMALL_BYTES:
                continue
            try:
                os.remove(p)
                freed += size
                n += 1
            except OSError as e:
                say("  could not delete %s: %s" % (p, e))
        if root != folder:
            try:
                os.rmdir(root)          # only when empty
            except OSError:
                pass
    return freed, n


def free_after_upload(sess, rec, tag, sent, assets, logs_up=True, exclude=(), roots=(), say=print):
    """(2026-11-05, auto-clean) Right after a session's own upload: when GitHub's release `tag` has every file the
    upload sent (`sent` {name: size} against `assets` {name: size}), delete the big files of what went up: the
    recording / frames folder it sent (rec), and the session's own frames folder when the logs zip carried it. The
    session's logs, CSVs and report stay. Only inside `roots`. Returns the bytes freed."""
    miss = recorded_missing(sent, assets)
    if not sent or miss:
        say("Auto-clean: nothing deleted on the Pi (%s)." % (
            "the upload recorded no files" if not sent else
            "%d file(s) not on GitHub at the size sent yet: %s" % (len(miss), ", ".join(miss[:3]))))
        return 0
    targets = []
    if rec and os.path.isdir(rec):
        targets.append(rec)
    if sess and logs_up and "frames" not in tuple(exclude):
        fd = DG.where_frames(sess)
        if os.path.isdir(fd) and _inside(fd, sess) and fd not in targets:
            targets.append(fd)
    freed = 0
    for t in targets:
        if os.path.islink(t) or not any(_inside(t, r) for r in roots if r):
            say("Auto-clean: skipped %s (outside the folders it may touch)." % t)
            continue
        b, n = prune_big(t, say)
        freed += b
        say("Auto-clean: deleted %d big file(s) in %s, %s freed (on GitHub in %s; its small CSV/JSON/TXT files "
            "stay)." % (n, t, _gb(b), tag))
    if not targets:
        say("Auto-clean: no big local folder to free for this upload.")
    elif freed:
        where = sess or targets[0]
        say("Auto-clean: %s freed in all. The card now has %.1f GB free." % (_gb(freed), DG.free_gb(where) or 0.0))
    return freed


def zip_copies(rels):
    """The upload's zip copies whose name and exact size are an asset on GitHub: [(path, bytes, tag)]."""
    dirs = {os.path.expanduser("~/.upload-tmp"), os.path.join(os.path.dirname(DG.rec_out().rstrip("/")), ".upload-tmp")}
    d = DG.drive_mount()
    if d:
        dirs.add(os.path.join(d, ".upload-tmp"))
    by = {}
    for tag, assets in rels.items():
        for n, s in assets.items():
            by.setdefault((n, s), tag)
            if n.endswith(".part-aa"):                  # a zip sent in parts: its copy is the parts' sum
                base = n[:-len(".part-aa")]
                by.setdefault((base, asset_size(assets, base)), tag)
    out = []
    for dd in sorted(dirs):
        if not os.path.isdir(dd):
            continue
        for fn in sorted(os.listdir(dd)):
            p = os.path.join(dd, fn)
            if os.path.isfile(p) and not os.path.islink(p):
                s = os.path.getsize(p)
                out.append((p, s, by.get((fn, s))))
    return out


SHELLS = ("bash", "sh", "zsh", "dash", "tmux", "")
# (2026-11-05) the programs that drive robots, whoever started them (Mission Control's run is one of them)
RUN_SCRIPTS = ("freeroam.py", "calib_run.py", "mission_run.py", "run_plan.py", "doe_loop.py", "e02.py", "ladder.py")
QUICK_LOOKS = ("status", "--status", "report", "--help", "-h")


def _tmux_busy(s):
    """The tmux session s has a program running in it (not just its shell, e.g. waiting at 'Press Enter')."""
    try:
        r = subprocess.run(["tmux", "list-panes", "-s", "-t", s, "-F", "#{pane_current_command}"],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return False
    if r.returncode != 0:
        return False
    return any(c.strip() not in SHELLS for c in r.stdout.splitlines())


def robot_programs(proc="/proc"):
    """['mission_run.py (pid 123)', ...]: robot programs running now (simulator runs and quick looks not counted)."""
    out = []
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit() and int(p) != os.getpid()]
    except OSError:
        return out
    for p in pids:
        try:
            with open(os.path.join(proc, p, "cmdline"), "rb") as f:
                argv = [x.decode("utf-8", "replace") for x in f.read().split(b"\0") if x]
        except OSError:
            continue
        if not argv or not os.path.basename(argv[0]).startswith("python"):
            continue
        for a in argv[1:3]:
            b = os.path.basename(a)
            if b in RUN_SCRIPTS:
                if "--dry" in argv or "--dry-run" in argv or any(q in argv for q in QUICK_LOOKS):
                    break
                out.append("%s (pid %s)" % (b, p))
                break
    return out


def running():
    """What is driving robots now, in words: tmux sessions of runs (with a program going in them) and robot programs.
    Never stops anything."""
    out = ["tmux " + s for s in RUNNING if _tmux_busy(s)]
    return out + robot_programs()


def _gb(n):
    return "%.2f GB" % (n / 1e9) if n >= 1e8 else "%.0f MB" % (n / 1e6)


def sessions(logs_dir):
    """[(name, session folder or None, frames folder or None)] of every session here, sorted by name."""
    drive = DG.drive_mount()
    ff = os.path.join(drive, "fleet-frames") if drive else None
    names = set()
    if os.path.isdir(logs_dir):
        names |= {n for n in os.listdir(logs_dir) if SESSION_RE.match(n) and os.path.isdir(os.path.join(logs_dir, n))}
    if ff and os.path.isdir(ff):
        names |= {n for n in os.listdir(ff) if os.path.isdir(os.path.join(ff, n))}
    out = []
    for name in sorted(names):
        sess = os.path.join(logs_dir, name)
        sess = sess if os.path.isdir(sess) else None
        fd = DG.where_frames(sess) if sess else None
        if (fd is None or not os.path.isdir(fd)) and ff and os.path.isdir(os.path.join(ff, name, "frames")):
            fd = os.path.join(ff, name, "frames")
        if fd is not None and not os.path.isdir(fd):
            fd = None
        out.append((name, sess, fd))
    return out


def is_dry(name):
    return name.endswith("-dryrun") or name.endswith("-dry")


def plan(logs_dir, rels, frames_only=False, say=print, thorough=False):
    """[(path, bytes, where, what)] to delete, and [(name, why)] kept. thorough: the exact check (zips rebuilt)."""
    dele, kept = [], []
    for name, sess, fd in sessions(logs_dir):
        if is_dry(name):
            kept.append((name, "a dry run: never uploaded"))
            continue
        if thorough:
            say("  checking %s ..." % name)
            r = check_session(name, sess, fd, rels, say)
        else:
            r = quick_session(name, sess, fd, rels, say)
        fd_inside = fd is not None and sess is not None and os.path.dirname(os.path.normpath(fd)) == \
            os.path.normpath(sess)
        if sess and r["logs"] and r["frames"] and not frames_only:
            dele.append((sess, DG.folder_bytes(sess), "card", "%s (logs%s; on GitHub as %s)" % (
                name, " and frames" if fd_inside else "", r["tag"])))
            if fd is not None and not fd_inside:
                dele.append((fd, DG.folder_bytes(fd), "drive", "%s frames (on GitHub as %s)" % (name, r["tag"])))
            continue
        if fd is not None and r["frames"]:
            dele.append((fd, DG.folder_bytes(fd), "card" if fd_inside or DG.same_fs(fd, logs_dir) else "drive",
                         "%s frames only (on GitHub as %s)" % (name, r["tag"])))
            if sess and frames_only and r["logs"]:
                kept.append((name, "its logs folder is confirmed on GitHub (%s), kept because of --frames-only"
                             % r["tag"]))
            elif sess:
                kept.append((name, "its logs folder: " + (r["why"] or "not confirmed")))
            continue
        if frames_only and r["logs"]:
            kept.append((name, "its logs folder is confirmed on GitHub (%s), kept because of --frames-only" % r["tag"]))
            continue
        kept.append((name, r["why"] or "not confirmed"))
    for p, s, tag in zip_copies(rels):
        if tag:
            dele.append((p, s, "card" if DG.same_fs(p, logs_dir) else "drive",
                         "upload copy %s (the same size is on GitHub in %s)" % (os.path.basename(p), tag)))
        else:
            kept.append((os.path.basename(p), "an upload copy GitHub does not have at this size (an upload that "
                                              "did not finish: run the upload again)"))
    return dele, kept


def status(logs_dir, rels, n=10, say=print):
    """(2026-11-05) `run-fleet-pi.sh uploads`: the newest n sessions, uploaded or not (the quick check), and their
    size on this Pi. Nothing is deleted."""
    rows = sorted(sessions(logs_dir), key=lambda r: r[0][4:] if r[0].startswith("e02-") else r[0])[-n:][::-1]
    if not rows:
        say("No sessions on this Pi.")
    for name, sess, fd in rows:
        size = (DG.folder_bytes(sess) if sess else 0) + (DG.folder_bytes(fd) if fd and not (sess and _inside(fd, sess))
                                                         else 0)
        if is_dry(name):
            mark, words = "-", "a dry run (never uploaded)"
        else:
            r = quick_session(name, sess, fd, rels, say)
            ok = (r["logs"] or sess is None) and r["frames"]
            mark, words = ("\u2713", "uploaded (%s)" % r["tag"]) if ok else ("\u2717", "NOT uploaded: " + r["why"])
        say("  %s  %-34s %9s here   %s" % (mark, name, _gb(size), words))
    say("(\u2713 = its GitHub release has every file its upload sent; quick check, not byte-exact. Free the space: "
        "./run-fleet-pi.sh clean-uploaded)")
    return 0


def low_priority():
    """Lowest CPU and disk priority for this process, so a robot run is never slowed by it."""
    try:
        os.nice(10)
    except (OSError, AttributeError):
        pass
    try:
        subprocess.call(["ionice", "-c", "3", "-p", str(os.getpid())], stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)
    except OSError:
        pass


def safe_path(p, logs_dir):
    """Only inside logs/, a fleet-frames folder or an .upload-tmp folder, never a link."""
    rp = os.path.realpath(p)
    if os.path.islink(p):
        return False
    roots = [os.path.realpath(logs_dir)]
    d = DG.drive_mount()
    if d:
        roots += [os.path.realpath(os.path.join(d, "fleet-frames")), os.path.realpath(os.path.join(d, ".upload-tmp"))]
    roots += [os.path.realpath(os.path.expanduser("~/.upload-tmp")),
              os.path.realpath(os.path.join(os.path.dirname(DG.rec_out().rstrip("/")), ".upload-tmp"))]
    return any(rp != r and rp.startswith(r + os.sep) for r in roots)


def main(argv=None, get=None, say=print):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yes", action="store_true", help="delete what the list marks DELETE")
    ap.add_argument("--frames-only", action="store_true", help="keep every logs folder")
    ap.add_argument("--thorough", action="store_true", help="the exact check: rebuild each zip in memory and compare "
                    "sizes (about a minute a GB on the Pi; the default until 2026-11-05)")
    ap.add_argument("--force", action="store_true", help="run even while a robot run is going (it takes CPU)")
    ap.add_argument("--status", action="store_true", help="only list the newest sessions: uploaded or not")
    ap.add_argument("-n", type=int, default=10, help="--status: how many sessions (default 10)")
    ap.add_argument("--logs", default=os.path.join(REPO, "logs"))
    a = ap.parse_args(argv)
    if get is None:
        low_priority()
    if not a.status:
        busy = running()
        if busy and not a.force:
            say("NOT STARTED: a robot run is going (%s). The clean would take CPU from it%s. Run it again after the "
                "run ends, or add --force to run it now anyway." % (
                    ", ".join(busy), " (the thorough check zips every folder again)" if a.thorough else ""))
            return 1
    say("Asking GitHub which sessions are uploaded ...")
    try:
        rels = releases(US.token(), get=get)
    except (urllib.error.URLError, OSError, ValueError) as e:
        say("Could not read the releases from GitHub (%s): nothing can be confirmed, so nothing is deleted. Check the "
            "WiFi and try again." % e)
        return 2
    say("%d releases on GitHub." % len(rels))
    if a.status:
        return status(a.logs, rels, a.n, say)
    dele, kept = plan(a.logs, rels, a.frames_only, say, thorough=a.thorough)
    say("")
    say("KEPT (not confirmed on GitHub; never deleted):")
    for n, why in kept:
        say("  keep    %s: %s" % (n, why))
    if not kept:
        say("  (none)")
    say("")
    say("CONFIRMED on GitHub (%s):" % ("exact sizes match" if a.thorough else
                                       "its release has every file its upload sent; quick check"))
    tot = {"card": 0, "drive": 0}
    for p, s, where, what in dele:
        tot[where] += s
        say("  DELETE  %9s  %-5s  %s   [%s]" % (_gb(s), where, what, p))
    if not dele:
        say("  (none)")
    say("")
    say("That frees %s on the card and %s on the USB drive." % (_gb(tot["card"]), _gb(tot["drive"])))
    if not a.yes:
        say("Nothing was deleted. To delete exactly the DELETE lines above: ./run-fleet-pi.sh clean-uploaded --yes")
        return 0
    freed = 0
    for p, s, where, what in dele:
        if not safe_path(p, a.logs):
            say("  skipped (outside the folders this may touch): %s" % p)
            continue
        try:
            if os.path.isdir(p):
                shutil.rmtree(p)
            else:
                os.remove(p)
            freed += s
            say("  deleted %s" % p)
        except OSError as e:
            say("  could not delete %s: %s" % (p, e))
    say("Freed %s. The card now has %.1f GB free." % (_gb(freed), DG.free_gb(a.logs) or 0.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
