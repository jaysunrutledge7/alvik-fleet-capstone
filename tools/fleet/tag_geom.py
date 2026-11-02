#!/usr/bin/env python3
"""Where a robot is, from its roof tag (conference-table build, 2026-11-02).

The tag lies flat on the roof, TAG_HEIGHT above the table, centred over the middle of the wheel axle (the robot's
pose point). So the robot's floor position is straight below the tag's middle, and its heading is the way the tag's
printed top edge (FRONT on the sheet) points. Each corner's ray from the camera is met with the plane z = height (not
the table, z = 0): reading the tag on the table plane instead would put a robot H/Z of its distance from below the
camera too far out (6 cm high, camera 190 cm up, 150 cm out: 4.7 cm). That is the "parallax" the site fit uses too.

    pose_from_detection(cam, det, height_cm, tag_cm) -> TagPose (x, y, heading, size check, sigma)
    fuse([TagPose, ...]) -> one pose from several cameras (weighted by each camera's sigma)
"""
import math

import numpy as np

CORNER_SD_PX = 0.35      # chosen: a tag corner's 1-sigma error after sub-pixel refinement, from the sizing tests' read
                         # error on synthetic pictures (blur 0.8 px, noise 3 grey levels: 0.2-0.4 px). DOE note: a still
                         # robot's tag corners over 100 real pictures give the real spread (camera check-out does this)
HEADING_FLOOR_DEG = 0.5  # the heading's sigma is never said to be better than this (the tag's placement on the roof).
                         # No DOE: a floor
SIZE_WARN = 0.06         # a tag read more than 6 % larger or smaller than printed (on its plane) says the camera's
                         # model or the tag height is off there. Chosen: 0.35 px corners on a 20 px tag are 2.5 % per
                         # read. DOE note: the spread over a real run sets it


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class TagPose:
    __slots__ = ("x", "y", "heading", "sigma_cm", "sigma_deg", "size_ratio", "cam", "t", "id", "px_per_cm", "corners")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    def to_dict(self):
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in
                ((k, getattr(self, k)) for k in self.__slots__ if k != "corners")}


def plane_points(cam, px, z):
    """Pixels (n, 2) -> (n, 2) points on the plane z cm above the table."""
    px = np.asarray(px, float).reshape(-1, 2)
    x, y = cam.to_floor(px[:, 0], px[:, 1], z)
    return np.column_stack([np.atleast_1d(x), np.atleast_1d(y)])


def pose_from_detection(cam, det, height_cm, tag_cm, corner_sd_px=CORNER_SD_PX):
    """The robot's pose from one tag read by one camera (cam: a CameraModel; det: tagcore.Detection)."""
    P = plane_points(cam, det.corners, height_cm)
    if not np.all(np.isfinite(P)):
        return None
    # the middle: the diagonals' crossing on the plane (the picture's centre would be off under perspective)
    a1, a2, b1, b2 = P[0], P[2], P[1], P[3]
    d1, d2 = a2 - a1, b2 - b1
    den = d1[0] * d2[1] - d1[1] * d2[0]
    mid = P.mean(0) if abs(den) < 1e-9 else a1 + (((b1[0] - a1[0]) * d2[1] - (b1[1] - a1[1]) * d2[0]) / den) * d1
    front = (P[0] + P[1]) / 2.0 - (P[2] + P[3]) / 2.0
    left = (P[0] + P[3]) / 2.0 - (P[1] + P[2]) / 2.0
    # heading from both axes (the left axis turned back by 90 deg), weighted equally
    h1 = math.atan2(front[1], front[0])
    h2 = math.atan2(left[1], left[0]) - math.pi / 2
    hd = math.degrees(math.atan2(math.sin(h1) + math.sin(h2), math.cos(h1) + math.cos(h2)))
    sides = [np.linalg.norm(P[k] - P[(k + 1) % 4]) for k in range(4)]
    size_ratio = float(np.mean(sides) / tag_cm)
    ppc = det.size_px / max(1e-6, float(np.mean(sides)))
    # sigma: the middle from 4 corners (sd / sqrt(2) each way); the heading from 2 edges of length tag_cm
    s_cm = corner_sd_px / ppc / math.sqrt(2.0)
    s_deg = max(HEADING_FLOOR_DEG, math.degrees(corner_sd_px / ppc / tag_cm))
    return TagPose(x=float(mid[0]), y=float(mid[1]), heading=float(wrap180(hd)), sigma_cm=float(s_cm),
                   sigma_deg=float(s_deg), size_ratio=size_ratio, cam=det.cam, t=det.t, id=det.id, px_per_cm=float(ppc),
                   corners=P)


def fuse(poses):
    """One pose from several (one per camera, same moment): inverse-variance mean; the heading as a weighted circular
    mean. Returns (TagPose, spread_cm): spread = how far the cameras' positions are apart (0 for one)."""
    poses = [p for p in poses if p is not None]
    if not poses:
        return None, None
    if len(poses) == 1:
        return poses[0], 0.0
    w = np.array([1.0 / p.sigma_cm ** 2 for p in poses])
    x = float(np.sum(w * [p.x for p in poses]) / w.sum())
    y = float(np.sum(w * [p.y for p in poses]) / w.sum())
    wh = np.array([1.0 / p.sigma_deg ** 2 for p in poses])
    s = float(np.sum(wh * [math.sin(math.radians(p.heading)) for p in poses]))
    c = float(np.sum(wh * [math.cos(math.radians(p.heading)) for p in poses]))
    spread = max(math.hypot(p.x - q.x, p.y - q.y) for p in poses for q in poses)
    out = TagPose(x=x, y=y, heading=math.degrees(math.atan2(s, c)), sigma_cm=float(1.0 / math.sqrt(w.sum())),
                  sigma_deg=float(1.0 / math.sqrt(wh.sum())), size_ratio=float(np.mean([p.size_ratio for p in poses])),
                  cam="+".join(str(p.cam) for p in poses), t=max(p.t or 0 for p in poses), id=poses[0].id,
                  px_per_cm=float(max(p.px_per_cm for p in poses)))
    return out, float(spread)


def tag_px_at(cam, x, y, height_cm, tag_cm, heading_deg=None):
    """How many pixels across a tag of tag_cm at (x, y, height) is in this camera (the smaller of its two sides, the
    worst heading when heading_deg is None: a tilted camera squeezes one way)."""
    hs = [heading_deg] if heading_deg is not None else [0.0, 45.0, 90.0, 135.0]
    worst = None
    for hd in hs:
        c, s = math.cos(math.radians(hd)), math.sin(math.radians(hd))
        h = tag_cm / 2.0
        pts = [(x + c * a - s * b, y + s * a + c * b, height_cm) for a, b in ((h, h), (h, -h), (-h, -h), (-h, h))]
        u, v = cam.project(np.array(pts))
        uv = np.column_stack([u, v])
        if not np.all(np.isfinite(uv)):
            return 0.0
        sides = [np.linalg.norm(uv[k] - uv[(k + 1) % 4]) for k in range(4)]
        m = min(sides)
        worst = m if worst is None else min(worst, m)
    return float(worst)
