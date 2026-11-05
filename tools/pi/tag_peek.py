#!/usr/bin/env python3
"""Which AprilTags the live camera reads right now (Nick, 2026-11-05: "can we check if it can currently read the April
tags?" with the room's overhead light on a timer). Reads N pictures from the recorder's /live (the recorder must be
running: ./run-fleet-pi.sh rec), looks for every tag25h9 id in each, and prints per id: in how many pictures it was
read, its size in pixels, and the picture's brightness. Changes nothing.

    python3 tools/pi/tag_peek.py                 # 20 pictures from http://127.0.0.1:8000/live
    python3 tools/pi/tag_peek.py --n 50 --url http://127.0.0.1:8000/live
"""
import argparse
import os
import sys
import time
import urllib.request

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "fleet"))
import tagcore as T  # noqa: E402

NAMES = {0: "robot Alpha", 1: "robot Bravo", 2: "robot (spare id 2)", 3: "robot (spare id 3)", 4: "robot (spare id 4)",
         5: "robot (spare id 5)", 20: "corner 1", 21: "corner 2", 22: "corner 3", 23: "corner 4",
         24: "side tag", 25: "side tag"}


def frames(url, n, timeout=10):
    """JPEG pictures cut out of the recorder's MJPEG stream (start 0xFFD8, end 0xFFD9)."""
    r = urllib.request.urlopen(url, timeout=timeout)
    buf, got = b"", 0
    while got < n:
        chunk = r.read(65536)
        if not chunk:
            break
        buf += chunk
        while True:
            a = buf.find(b"\xff\xd8")
            b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
            if a < 0 or b < 0:
                break
            jpg, buf = buf[a:b + 2], buf[b + 2:]
            g = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
            if g is not None:
                got += 1
                yield g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8000/live")
    ap.add_argument("--n", type=int, default=20)
    a = ap.parse_args()
    det = T.TagDetector("tag25h9")
    seen, sizes, bright, shape, k = {}, {}, [], None, 0
    t0 = time.time()
    try:
        for g in frames(a.url, a.n):
            k += 1
            shape = g.shape
            bright.append(float(g.mean()))
            for d in det.detect_full(g):
                seen[d.id] = seen.get(d.id, 0) + 1
                sizes.setdefault(d.id, []).append(d.size_px)
    except OSError as e:
        sys.exit("No pictures from %s (%s). Is the recorder running? ./run-fleet-pi.sh rec" % (a.url, e))
    if not k:
        sys.exit("No pictures from %s. Is the recorder running? ./run-fleet-pi.sh rec" % a.url)
    print("%d pictures, %dx%d, in %.0f s; brightness %.0f of 255 (under ~40 is dark)" % (
        k, shape[1], shape[0], time.time() - t0, float(np.mean(bright))))
    if not seen:
        print("NO tags read.")
    for i in sorted(seen):
        print("tag %2d  %-20s read in %2d of %d pictures, %4.1f px across" % (
            i, NAMES.get(i, "other"), seen[i], k, float(np.median(sizes[i]))))
    for i in (20, 21, 22, 23, 24, 25):
        if i not in seen:
            print("tag %2d  %-20s NOT read" % (i, NAMES[i]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
