"""(e10) A stalled drive never pauses the live pictures (port 8000): the 5 Nov 4 h run lost every robot's camera for
~4.4 s every 5 minutes, in step with free roam writing its 5-minute part. camrec's video is now written by its own
thread; free roam's parts by theirs, streamed. The old ways stay behind MIRLT_REC_WRITE=inline / MIRLT_PART_WRITE=inline.
No hardware."""
import os
import sys
import threading
import time

import numpy as np
import pytest

PI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.dirname(PI)
for p in (PI, os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import camrec  # noqa: E402
from test_camrec_dual import FakeCapture  # noqa: E402


class StallingWriter:
    """A VideoWriter whose drive stalls 1.5 s once (the 5-minute part landing on it)."""

    def __init__(self, stall_at=15, stall_s=1.5):
        self.n, self.stall_at, self.stall_s = 0, stall_at, stall_s

    def write(self, frame):
        self.n += 1
        if self.n == self.stall_at:
            time.sleep(self.stall_s)

    def release(self):
        pass


def live_gaps(tmp_path, mode):
    old = camrec.REC_WRITE
    camrec.REC_WRITE = mode
    try:
        cr = camrec.CamRec(FakeCapture(fps=30), "fake", log=lambda *a: None)
        assert cr.first_frame()
        cr.open_outputs(str(tmp_path / mode), {"test": True})
        cr.writer = StallingWriter()
        times = []
        put = cr.live.put

        def spy(frame, n, t):
            times.append(time.monotonic())
            put(frame, n, t)
        cr.live.put = spy
        cr.live.clients = 1                       # a tracker is connected
        stop = threading.Event()
        cr.loop(time.monotonic(), stop, seconds=2.5)
        cr.close(time.monotonic() - 2.5)
        gaps = np.diff(times)
        return float(gaps.max()), cr
    finally:
        camrec.REC_WRITE = old


def test_a_stalled_drive_does_not_pause_the_live_pictures(tmp_path):
    g, cr = live_gaps(tmp_path, "thread")
    assert g < 0.3                                          # the live pictures kept coming
    rows = open(os.path.join(cr.folder, "frames.csv")).read().strip().split("\n")[1:]
    assert len(rows) == cr.arec.written                     # frames.csv still matches the video, frame for frame
    g_old, _ = live_gaps(tmp_path, "inline")
    assert g_old > 1.2                                      # the old way: the live picture waited for the drive


def test_the_async_recorder_drops_recorded_frames_not_live_ones():
    import csv
    import io
    w = StallingWriter(stall_at=1, stall_s=0.5)
    buf = io.StringIO()
    ar = camrec.AsyncRecorder(w, csv.writer(buf), threading.Lock(), maxq=5)
    t0 = time.monotonic()
    for k in range(40):
        ar.put(np.zeros((2, 2), np.uint8), [k])
    assert time.monotonic() - t0 < 0.1                      # never waits
    ar.close()
    assert ar.dropped > 0 and ar.written + ar.dropped == 40


def test_free_roam_parts_are_written_by_their_own_thread_streamed(tmp_path, monkeypatch):
    import doe_loop as DL
    import e02_framesaver as FS
    jpegs = [bytes([0xFF, 0xD8]) + os.urandom(3000 + k) + bytes([0xFF, 0xD9]) for k in range(50)]
    FS.write_mjpeg_avi(str(tmp_path / "a.avi"), jpegs, 12.0, 64, 48)
    FS.write_mjpeg_avi_stream(str(tmp_path / "b.avi"), jpegs, 12.0, 64, 48, pause_every_mb=0.05,
                              sleep=lambda s: None)
    assert open(tmp_path / "a.avi", "rb").read() == open(tmp_path / "b.avi", "rb").read()   # the same file

    class Slow:
        def __init__(self):
            self.FS = FS

    cf = DL.CamFrames.__new__(DL.CamFrames)
    cf.cam, cf.rate, cf.dir, cf.label, cf.mb = "overhead", 12.0, str(tmp_path), "move", 0.0
    os.makedirs(tmp_path / "track", exist_ok=True)
    cf.FS = FS
    slow = {"n": 0}

    def slow_stream(*a, **k):
        slow["n"] += 1
        time.sleep(1.0)                                     # a slow drive
        return FS.write_mjpeg_avi(*a[:5])
    monkeypatch.setattr(FS, "write_mjpeg_avi_stream", slow_stream)
    monkeypatch.setattr(DL, "PART_WRITE", "thread")
    cf.part = [(1000.0 + k / 12.0, j, k) for k, j in enumerate(jpegs)]
    cf.part_t0 = cf.part[0][0]
    t0 = time.monotonic()
    cf._flush()
    assert time.monotonic() - t0 < 0.2                      # the reader is back on the stream at once
    cf.drain_parts(5.0)
    assert slow["n"] == 1 and any(f.endswith(".avi") for f in os.listdir(tmp_path / "track"))
