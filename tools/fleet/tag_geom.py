#!/usr/bin/env python3
"""Where a robot is, from its roof tag (conference-table build, 2026-11-02).

The tag lies flat on the roof, TAG_HEIGHT above the table, centred over the middle of the wheel axle (the robot's
pose point). So the robot's floor position is straight below the tag's middle, and its heading is the way the tag's
printed top edge (FRONT on the sheet) points. Each corner's ray from the camera is met with the plane z = height (not
the table, z = 0): reading the tag on the table plane instead would put a robot H/Z of its distance from below the
camera too far out (6 cm high, camera 190 cm up, 150 cm out: 4.7 cm). That is the "parallax" the site fit uses too.

    pose_from_detection(cam, det, height_cm, tag_cm) -> TagPose (x, y, heading, size check, sigma)
    fuse([TagPose, ...]) -> one pose from several cameras (weighted by each camera's sigma)

A measured roof (2026-11-03, Nick: "checking for height and angle should be part of the auto calibration"):
roof_cal.py measures each robot's tag height, tilt and offset from the axle midpoint; pose_from_detection(..., roof=)
then uses them. The tag's middle is still the crossing of its diagonals (a perspective picture keeps that crossing), so
the middle is exact at the measured middle height whatever the tilt; each corner's ray is met at that corner's own
height (the tilt raises one edge and lowers the other) for the heading and the size check; the robot's turning point
is the middle less the offset (turned with the heading). roof=None is the flat model as before (the switch).
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
    __slots__ = ("x", "y", "heading", "sigma_cm", "sigma_deg", "size_ratio", "cam", "t", "id", "px_per_cm", "corners", "mid")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    def to_dict(self):
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in
                ((k, getattr(self, k)) for k in self.__slots__ if k not in ("corners", "mid"))}


def plane_points(cam, px, z):
    """Pixels (n, 2) -> (n, 2) points on the plane z cm above the table."""
    px = np.asarray(px, float).reshape(-1, 2)
    x, y = cam.to_floor(px[:, 0], px[:, 1], z)
    return np.column_stack([np.atleast_1d(x), np.atleast_1d(y)])


def tilt_vec(tilt_deg, lean_deg):
    """The roof's tilt as a rotation vector (radians) in the robot's frame (ahead, left, up). tilt_deg: the angle
    between the tag's face and the table; lean_deg: which way the face points, degrees anticlockwise from the nose seen
    from above (0 = tipped towards the nose, its back edge higher; 90 = towards the robot's left)."""
    t, f = math.radians(float(tilt_deg or 0.0)), math.radians(float(lean_deg or 0.0))
    return np.array([-t * math.sin(f), t * math.cos(f), 0.0])


def tilt_of_vec(tv):
    """(tilt_deg, lean_deg) of a rotation vector from tilt_vec (lean in 0-360)."""
    tv = np.asarray(tv, float)
    t = float(math.hypot(tv[0], tv[1]))
    return math.degrees(t), (math.degrees(math.atan2(-tv[0], tv[1])) % 360.0) if t > 1e-9 else 0.0


def rodrigues(rv):
    rv = np.asarray(rv, float)
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * Kx + (1 - math.cos(th)) * (Kx @ Kx)


def roof_local(tag_cm, tv=None):
    """The 4 printed corners (top-left, top-right, bottom-right, bottom-left; top = the nose) of a tag of tag_cm around
    its middle, in the robot's frame (ahead, left, up), tilted by the rotation vector tv (None: flat)."""
    h = tag_cm / 2.0
    L = np.array([[h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0], [-h, h, 0.0]])
    return L if tv is None else L @ rodrigues(tv).T


def roof_corners_world(x, y, th_rad, height_cm, tag_cm, tv=None, offset_cm=(0.0, 0.0)):
    """World (n=4, 3) corners of a robot's roof tag: the robot's turning point at (x, y), heading th_rad; the tag's
    middle offset_cm (ahead, left) from it and height_cm above the table; tilted by tv (robot frame)."""
    c, s = math.cos(th_rad), math.sin(th_rad)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    mid = np.array([x, y, 0.0]) + Rz @ np.array([float(offset_cm[0]), float(offset_cm[1]), float(height_cm)])
    return mid + roof_local(tag_cm, tv) @ Rz.T


def roof_of(d):
    """A roof dict ({height_cm, tilt_deg, lean_deg, offset_cm}) -> (height, tv or None, offset (2,))."""
    tv = None
    if d.get("tilt_deg"):
        tv = tilt_vec(d["tilt_deg"], d.get("lean_deg", 0.0))
    off = d.get("offset_cm") or (0.0, 0.0)
    return float(d["height_cm"]), tv, np.asarray(off, float)


def pose_from_detection(cam, det, height_cm, tag_cm, corner_sd_px=CORNER_SD_PX, roof=None):
    """The robot's pose from one tag read by one camera (cam: a CameraModel; det: tagcore.Detection). roof (2026-11-03):
    this robot's measured roof ({height_cm, tilt_deg, lean_deg, offset_cm}; roof_cal.py) or None (flat at height_cm,
    centred over the axle: the model before)."""
    if roof is not None:
        return _pose_roof(cam, det, tag_cm, roof, corner_sd_px)
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


def _crossing(P):
    a1, a2, b1, b2 = P[0], P[2], P[1], P[3]
    d1, d2 = a2 - a1, b2 - b1
    den = d1[0] * d2[1] - d1[1] * d2[0]
    return P.mean(0) if abs(den) < 1e-9 else a1 + (((b1[0] - a1[0]) * d2[1] - (b1[1] - a1[1]) * d2[0]) / den) * d1


def _pose_roof(cam, det, tag_cm, roof, corner_sd_px):
    """pose_from_detection with a measured roof: the middle from the diagonals at the middle's height (exact under any
    tilt), each corner met at its own height, the heading from the corners' layout (a 2-D rotation fit), the turning
    point = the middle less the offset turned by the heading."""
    H, tv, off = roof_of(roof)
    P = plane_points(cam, det.corners, H)
    if not np.all(np.isfinite(P)):
        return None
    mid = _crossing(P)
    loc = roof_local(tag_cm, tv)
    Q = np.vstack([plane_points(cam, det.corners[k:k + 1], H + loc[k, 2])[0] for k in range(4)])
    if not np.all(np.isfinite(Q)):
        return None
    A = loc[:, :2] - loc[:, :2].mean(0)
    B = Q - Q.mean(0)
    Hm = A.T @ B
    th = math.atan2(Hm[0, 1] - Hm[1, 0], Hm[0, 0] + Hm[1, 1])
    Q3 = np.column_stack([Q, H + loc[:, 2]])
    sides = [np.linalg.norm(Q3[k] - Q3[(k + 1) % 4]) for k in range(4)]
    size_ratio = float(np.mean(sides) / tag_cm)
    ppc = det.size_px / max(1e-6, float(np.mean([np.linalg.norm(P[k] - P[(k + 1) % 4]) for k in range(4)])))
    c, s = math.cos(th), math.sin(th)
    x = float(mid[0] - (c * off[0] - s * off[1]))
    y = float(mid[1] - (s * off[0] + c * off[1]))
    s_cm = corner_sd_px / ppc / math.sqrt(2.0)
    s_deg = max(HEADING_FLOOR_DEG, math.degrees(corner_sd_px / ppc / tag_cm))
    return TagPose(x=x, y=y, heading=float(wrap180(math.degrees(th))), sigma_cm=float(s_cm), sigma_deg=float(s_deg),
                   size_ratio=size_ratio, cam=det.cam, t=det.t, id=det.id, px_per_cm=float(ppc), corners=Q,
                   mid=(float(mid[0]), float(mid[1])))


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
