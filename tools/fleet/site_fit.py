#!/usr/bin/env python3
"""The site fit (conference-table build, 2026-11-02): each camera's lens and pose, the table's size and every sticker's
spot, from what the cameras see, with the robots' roof tags as the ruler.

What goes in (all optional except the clicks of at least two table corners per camera and some tags):
  corners    the table's corners clicked in each camera's picture, numbered 1-2-3-4 going round the table
             counter-clockwise seen from above (1 = 0,0; 1 -> 2 is x; 1 -> 4 is y). A camera over half the table
             clicks the two it sees.
  tags       the robots' roof tags read in the pictures (tag_obs.csv rows), grouped into moments: the reads of one
             tag by both cameras at one moment share one robot pose. Their printed size and their height above the
             table are known: that is the scale, and (Nick's idea, 2 Nov) the difference between the tag plane and the
             table plane across many spots and two cameras pins each camera's height and tilt.
  stickers   the production points' stickers clicked in each picture they are in (unknown spots, shared by the cameras)
  ref_tags   (2026-11-03) flat reference tags on the table top (ref_tags.py): one in each corner, fixed by a tile that
             hooks the corner, so its middle is a known distance in from both edges; one on each long side where the
             cameras overlap, its middle a known distance in from that edge, its place along the edge fitted. They give
             each camera its pose, the table's corners and size, and the scale (their printed size) with no clicks,
             and tie the two cameras together; corner 1's tag is (0, 0). No height correction: they lie on the table.
             Corner clicks still work beside them (a switch per site: off at school, on at home).
  typed      optional measurements: the table's length and width; a sticker's coordinates; distances between
             stickers. They are CHECKS first: the fit is run without them, and each typed value is compared with what
             the cameras measured; a disagreement is said in words, never silently averaged in.

The fit (scipy least_squares, robust soft-L1 loss): unknowns are each camera's pose (6), focal length (1, a weak
prior from a typical webcam), optionally its lens bend k1; the table's length and width; each sticker's x, y; each tag
moment's x, y, heading. Residuals: pixels (corner clicks CLICK_SD_PX, sticker clicks STICKER_SD_PX, tag corners
TAG_SD_PX) and the typed measurements (when used).

Starting values (needed for a fit of this size): each camera's tag reads give the table plane in that camera's own
frame (solvePnP of each tag, the plane through their middles less the tag height), the corner clicks on that plane fix
the camera's place, and a camera with fewer than two corners is placed from the tags or stickers it shares with one
already placed.

    python3 tools/fleet/site_fit.py --selftest       # two simulated cameras over the 396 x 168 cm table
    python3 tools/fleet/site_fit.py --obs obs.json --out tagcal/     # a real fit: camera models + site_fit.json
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

CLICK_SD_PX = 2.0        # a person's click on a table corner (corner_setup.CLICK_SD_PX). DOE note there
STICKER_SD_PX = 1.5      # a click on a 2 cm round sticker's middle. Chosen; DOE: Nick clicks one sticker 5 times
TAG_SD_PX = 0.35         # tag_geom.CORNER_SD_PX
TYPED_SD_CM = 0.3        # a tape measurement (typed size, coordinates, distances). Chosen: a careful tape. No DOE
HFOV_PRIOR_DEG = 70.0    # a typical webcam (Logitech C920: 70.4 deg). The focal length's starting value and weak prior
F_PRIOR_FRAC = 0.25      # the prior's 1-sigma, as a share of f: weak (the tags and corners decide it when they can)
K1_SD = 0.15             # lens bend k1 prior (0 +- 0.15), when fitted
AGREE_FRAC = 0.015       # a typed size or distance more than 1.5 % from the cameras' is said as a disagreement (2 % in
                         # corner_setup with the lights; the tags measure better). DOE note: the spread over real setups
AGREE_CM = 1.5           # ... or more than this for a typed coordinate. Chosen: the tracking error tolerated near an edge
MAX_GROUPS = 100         # tag moments used at most (spread over time). 160 took 1.5-5 min a refit here (about 5x that on
                         # a Pi 4); 100 still put the refit within 0.6 cm of the truth. No DOE: a time limit
TAG_SCALE_SD = 0.05      # with typed measurements, the tags' read size is fitted (k = read / printed): a blurred black
                         # square's corners come out slightly inside it (the dry runs: 1.5-3.5 % small), which would make
                         # the whole table that much large. Prior 1 +- 0.05. No DOE: the typed tape decides it
TAG_SCALE_WARN = 0.05    # k more than 5 % from 1: said (the printer scaled the page, or the tape is off). Blur alone
                         # reads a tag 1.5-3.5 % small in the simulator (its rounded corners are found inside the true
                         # ones), so 3 % (the first line) warned on every sharp static setup. DOE note: printed tags of
                         # known size on the real cameras, k against focus


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def _R(rv):
    import cv2
    return cv2.Rodrigues(np.asarray(rv, float).reshape(3, 1))[0]


def _rv(R):
    import cv2
    return cv2.Rodrigues(np.asarray(R, float))[0].ravel()


def project(K, k1, rv, tv, P):
    """World points (n, 3) -> pixels (n, 2) with a one-term radial lens (k1)."""
    R = _R(rv)
    Pc = P @ R.T + tv
    z = Pc[:, 2:3]
    xn = Pc[:, :2] / np.where(np.abs(z) < 1e-9, 1e-9, z)
    r2 = np.sum(xn ** 2, axis=1, keepdims=True)
    xd = xn * (1.0 + k1 * r2)
    return xd * [K[0, 0], K[1, 1]] + K[:2, 2]


def tag_corners_world(x, y, th_deg, z, s):
    """The 4 printed corners (top-left, top-right, bottom-right, bottom-left; top = the robot's nose) of a tag whose
    middle is at (x, y, z) facing th_deg."""
    h = s / 2.0
    c, sn = math.cos(math.radians(th_deg)), math.sin(math.radians(th_deg))
    loc = ((h, h), (h, -h), (-h, -h), (-h, h))
    return np.array([[x + c * a - sn * b, y + sn * a + c * b, z] for a, b in loc])


def K_of(f, size):
    W, H = size
    return np.array([[f, 0, W / 2.0 - 0.5], [0, f, H / 2.0 - 0.5], [0, 0, 1.0]])


# ------------------------------------------------------------------------------------------- starting values
def plane_from_tags(K, groups_cam, s, H):
    """The table plane in one camera's frame from its tag reads: (normal n pointing to the camera side, distance d of
    the TABLE plane: n.X + d = 0 ... returned as (n, d_table)) or None. Each tag's solvePnP (IPPE square) gives its
    middle and its normal; the plane through the middles, moved H along the normal, is the table."""
    import cv2
    obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float64)
    mids, norms = [], []
    flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    for corners in groups_cam:
        ok, rv, tv = cv2.solvePnP(obj, np.asarray(corners, np.float64).reshape(4, 1, 2), K, None, flags=flag)
        if not ok:
            continue
        R = _R(rv)
        nrm = R[:, 2]                          # the tag's own z (out of the printed face) in camera coordinates
        if nrm[2] > 0:
            nrm = -nrm                         # facing the camera
        mids.append(tv.ravel())
        norms.append(nrm)
    if not mids:
        return None
    M = np.array(mids)
    n = np.mean(norms, axis=0)
    n /= np.linalg.norm(n)
    if len(M) >= 3:
        c = M.mean(0)
        _, sv, Vt = np.linalg.svd(M - c)
        if sv[1] > 5.0:                        # the middles spread enough to fit their own plane
            n2 = Vt[2]
            if np.dot(n2, n) < 0:
                n2 = -n2
            if np.dot(n2, n) > math.cos(math.radians(25)):
                n = n2
    c = M.mean(0)
    # tag plane: n.X = n.c ; table plane is H further from the camera along -n
    d_tag = float(np.dot(n, c))                # negative: the camera is on the n side, at the origin
    return n, d_tag - H                        # table plane: n.X = d_table


def ray_plane(K, k1, uv, n, d):
    """Pixels -> camera-frame points on the plane n.X = d."""
    uv = np.asarray(uv, float).reshape(-1, 2)
    xn = (uv - K[:2, 2]) / [K[0, 0], K[1, 1]]
    if k1:
        for _ in range(5):
            r2 = np.sum(xn ** 2, axis=1, keepdims=True)
            xn = ((uv - K[:2, 2]) / [K[0, 0], K[1, 1]]) / (1.0 + k1 * r2)
    rays = np.column_stack([xn, np.ones(len(xn))])
    s = d / (rays @ n)
    return rays * s[:, None]


def rigid_from_plane(Pc, Pw2, n, zs=None):
    """Camera-frame points Pc (m, 3) and their world (x, y) Pw2 (m, 2), m >= 2, each zs[i] above the table (default
    0): the camera's pose (R world->camera, t). The world's z is the plane's normal towards the camera (up)."""
    z_c = n / np.linalg.norm(n)                 # world z in camera coordinates (towards the camera = up)
    # an in-plane basis in the camera frame
    a = np.cross(z_c, [1.0, 0, 0])
    if np.linalg.norm(a) < 0.1:
        a = np.cross(z_c, [0, 1.0, 0])
    a /= np.linalg.norm(a)
    b = np.cross(z_c, a)
    q = np.column_stack([(Pc - Pc.mean(0)) @ a, (Pc - Pc.mean(0)) @ b])       # 2D in the plane's own basis
    w = Pw2 - Pw2.mean(0)
    # 2D rotation (no reflection: the basis (a, b, z) and the world's (x, y, z) are both right-handed)
    Hm = q.T @ w
    ang = math.atan2(Hm[0, 1] - Hm[1, 0], Hm[0, 0] + Hm[1, 1])
    ca, sa = math.cos(ang), math.sin(ang)
    # world x = ca * a_coord - sa * b_coord ... -> camera-frame directions of world x and y
    x_c = ca * a - sa * b
    y_c = sa * a + ca * b
    R = np.column_stack([x_c, y_c, z_c])        # columns: world axes in camera coordinates  (= R world->camera)
    if np.linalg.det(R) < 0:
        R[:, 1] = -R[:, 1]
    # t: camera-frame position of the world origin
    zs = np.zeros(len(Pw2)) if zs is None else np.asarray(zs, float)
    t = Pc.mean(0) - R @ np.array([Pw2[:, 0].mean(), Pw2[:, 1].mean(), zs.mean()])
    return R, t


# ------------------------------------------------------------------------------------------------- the fit
class Problem:
    def __init__(self, obs, use_typed=True, fit_k1=False, fit_height=False, fit_tag_scale=False):
        self.obs = obs
        self.fit_tag_scale = fit_tag_scale
        self.cams = list(obs["cameras"])
        self.s = float(obs["tag_cm"])
        self.H = float(obs["tag_height_cm"])
        self.use_typed = use_typed
        self.fit_k1 = fit_k1
        self.fit_height = fit_height
        self.stickers = sorted(obs.get("stickers", {}))
        self.groups = obs.get("tag_groups", [])
        tb = obs.get("table") or {}
        self.typed_L, self.typed_W = tb.get("length_cm"), tb.get("width_cm")
        # the reference tags (2026-11-03): only the ones some camera read
        rt = obs.get("ref_tags") or {}
        self.ref = rt if any((rt.get("reads") or {}).values()) else None
        if self.ref:
            self.ref_lay = {str(k): v for k, v in (rt.get("layout") or {}).items()}
            self.ref_reads = {c: {str(i): v for i, v in (rt["reads"].get(c) or {}).items() if str(i) in self.ref_lay}
                              for c in self.cams}
            seen = sorted({i for c in self.cams for i in self.ref_reads.get(c, {})}, key=int)
            self.ref_sides = [i for i in seen if self.ref_lay[i]["kind"] == "side"]
            self.ref_s = float(rt.get("tag_cm", 12.0))
            self.ref_z = float(rt.get("z_cm", 0.18))
            self.ref_off = float(rt.get("offset_cm", 7.75))
            self.ref_q = {str(k): int(v) for k, v in (rt.get("quarters") or {}).items()}
            self.typed_along = {str(k): float(v) for k, v in (rt.get("typed_along_cm") or {}).items()
                                if str(k) in self.ref_sides}
        else:
            self.ref_sides, self.typed_along = [], {}

    def ref_pose(self, i, L, W, along):
        """The reference tag i's middle and printed top (x, y, heading) for a table L x W."""
        import ref_tags as RT
        it = self.ref_lay[i]
        x, y = RT.centre(it, L, W, self.ref_off, along.get(i))
        return x, y, RT.heading(it, self.ref_q.get(i, 0))

    # parameter layout
    def layout(self):
        k = 0
        lay = {}
        per = 8 if self.fit_k1 else 7
        for c in self.cams:
            lay[c] = (k, k + per)
            k += per
        lay["LW"] = (k, k + 2)
        k += 2
        lay["_along"] = (k, k + len(self.ref_sides))
        k += len(self.ref_sides)
        if self.fit_tag_scale and self.ref:
            lay["_kref"] = (k, k + 1)
            k += 1
        if self.fit_height:
            lay["H"] = (k, k + 1)
            k += 1
        if self.fit_tag_scale:
            lay["k"] = (k, k + 1)
            k += 1
        lay["stk"] = (k, k + 2 * len(self.stickers))
        k += 2 * len(self.stickers)
        lay["grp"] = (k, k + 3 * len(self.groups))
        k += 3 * len(self.groups)
        self.lay, self.n = lay, k
        return lay

    def cam_params(self, x, c):
        a, b = self.lay[c]
        v = x[a:b]
        k1 = v[7] if self.fit_k1 else 0.0
        return v[0:3], v[3:6], v[6], k1

    def resid(self, x):
        out = []
        L, W = x[self.lay["LW"][0]:self.lay["LW"][1]]
        H = x[self.lay["H"][0]] if self.fit_height else self.H
        ks = x[self.lay["k"][0]] if self.fit_tag_scale else 1.0
        corners_w = {1: (0, 0), 2: (L, 0), 3: (L, W), 4: (0, W)}
        sa = self.lay["stk"][0]
        stk = {sid: x[sa + 2 * i: sa + 2 * i + 2] for i, sid in enumerate(self.stickers)}
        ga = self.lay["grp"][0]
        along = {i: x[self.lay["_along"][0] + j] for j, i in enumerate(self.ref_sides)}
        kref = x[self.lay["_kref"][0]] if "_kref" in self.lay else 1.0
        for c in self.cams:
            rv, tv, f, k1 = self.cam_params(x, c)
            size = self.obs["cameras"][c]["size"]
            K = K_of(f, size)
            if self.ref:
                for i, corners in self.ref_reads.get(c, {}).items():
                    Pw = tag_corners_world(*self.ref_pose(i, L, W, along), self.ref_z, self.ref_s * kref)
                    out.append(((project(K, k1, rv, tv, Pw) - np.asarray(corners, float)) / TAG_SD_PX).ravel())
            cc = (self.obs.get("corners") or {}).get(c) or {}
            if cc:
                ids = sorted(int(k) for k in cc)
                P = np.array([[*corners_w[i], 0.0] for i in ids])
                uv = project(K, k1, rv, tv, P)
                out.append(((uv - np.array([cc[str(i)] if str(i) in cc else cc[i] for i in ids])) / CLICK_SD_PX).ravel())
            ps, uvs = [], []
            for sid in self.stickers:
                u = (self.obs["stickers"][sid].get("cams") or {}).get(c)
                if u is not None:
                    ps.append([*stk[sid], 0.0])
                    uvs.append(u)
            if ps:
                out.append(((project(K, k1, rv, tv, np.array(ps)) - np.array(uvs)) / STICKER_SD_PX).ravel())
            # weak priors: focal length, k1
            f0 = self.obs["cameras"][c].get("f_prior") or (size[0] / 2.0) / math.tan(math.radians(HFOV_PRIOR_DEG) / 2)
            out.append(np.array([(f - f0) / (F_PRIOR_FRAC * f0)]))
            if self.fit_k1:
                out.append(np.array([k1 / K1_SD]))
        for gi, g in enumerate(self.groups):
            gx, gy, gth = x[ga + 3 * gi: ga + 3 * gi + 3]
            Pw = tag_corners_world(gx, gy, gth, H, self.s * ks)
            for c, corners in g["reads"].items():
                rv, tv, f, k1 = self.cam_params(x, c)
                uv = project(K_of(f, self.obs["cameras"][c]["size"]), k1, rv, tv, Pw)
                out.append(((uv - np.asarray(corners, float)) / TAG_SD_PX).ravel())
        if self.use_typed:
            if self.typed_L:
                out.append(np.array([(L - self.typed_L) / TYPED_SD_CM, (W - self.typed_W) / TYPED_SD_CM]))
            for sid in self.stickers:
                xy = self.obs["stickers"][sid].get("xy_cm")
                if xy is not None:
                    out.append((stk[sid] - np.asarray(xy, float)) / TYPED_SD_CM)
            for a_, b_, dcm in self.obs.get("distances", []):
                if a_ in stk and b_ in stk:
                    out.append(np.array([(np.linalg.norm(stk[a_] - stk[b_]) - dcm) / TYPED_SD_CM]))
            for i, v in self.typed_along.items():
                out.append(np.array([(along[i] - v) / TYPED_SD_CM]))
        if "_kref" in self.lay:
            out.append(np.array([(kref - 1.0) / TAG_SCALE_SD]))
        if self.fit_height:
            out.append(np.array([(H - self.H) / 1.0]))          # the measured height, +-1 cm
        if self.fit_tag_scale:
            out.append(np.array([(ks - 1.0) / TAG_SCALE_SD]))
        return np.concatenate(out) if out else np.zeros(1)


def _kabsch2(P, Q):
    """2-D rotation angle (rad) and translation taking points P (n, 2) onto Q (n, 2), no reflection."""
    pc, qc = P.mean(0), Q.mean(0)
    Hm = (P - pc).T @ (Q - qc)
    ang = math.atan2(Hm[0, 1] - Hm[1, 0], Hm[0, 0] + Hm[1, 1])
    Rm = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
    return ang, qc - Rm @ pc


def _rot2(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def ref_initial(obs, prob):
    """Starting values from the reference tags alone (2026-11-03): each camera sees its tags on the table plane (each
    tag's solvePnP gives the plane; the corners meet it); cameras that share a tag (the side tags in the overlap) are
    joined; each joined group's turn to the table frame comes from the tags' known turns (each tag's quarter turn on
    its tile is read, not assumed), then its shift, the table's length and width and the side tags' places along their
    edges from one small linear solve (typed sizes join it as equations). Returns dict(placed {cam: (R, t, f0)}, L, W,
    along {id: cm}, quarters {id: 0-3}, words) or None when no camera can be placed this way."""
    import cv2
    import ref_tags as RT
    s, z, off = prob.ref_s, prob.ref_z, prob.ref_off
    obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float64)
    flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    loc = {}
    for c in prob.cams:
        reads = prob.ref_reads.get(c) or {}
        if not reads:
            continue
        size = obs["cameras"][c]["size"]
        f0 = obs["cameras"][c].get("f_prior") or (size[0] / 2.0) / math.tan(math.radians(HFOV_PRIOR_DEG) / 2)
        K = K_of(f0, size)
        mids, norms = [], []
        for i, cs in reads.items():
            ok, rv, tv = cv2.solvePnP(obj, np.asarray(cs, np.float64).reshape(4, 1, 2), K, None, flags=flag)
            if not ok:
                continue
            nrm = _R(rv)[:, 2]
            mids.append(tv.ravel())
            norms.append(-nrm if nrm[2] > 0 else nrm)
        if not mids:
            continue
        n = np.mean(norms, axis=0)
        n /= np.linalg.norm(n)
        M = np.array(mids)
        if len(M) >= 3:
            _, sv, Vt = np.linalg.svd(M - M.mean(0))
            if sv[1] > 5.0:
                n2 = Vt[2] if np.dot(Vt[2], n) > 0 else -Vt[2]
                if np.dot(n2, n) > math.cos(math.radians(25)):
                    n = n2
        d = float(np.dot(n, M.mean(0)))
        a = np.cross(n, [1.0, 0, 0])
        if np.linalg.norm(a) < 0.1:
            a = np.cross(n, [0, 1.0, 0])
        a /= np.linalg.norm(a)
        b = np.cross(n, a)
        tags = {}
        for i, cs in reads.items():
            P = ray_plane(K, 0.0, cs, n, d)
            q = np.column_stack([P @ a, P @ b])
            top = (q[0] + q[1]) / 2 - (q[2] + q[3]) / 2
            tags[i] = dict(P=P, q=q, c=q.mean(0), h=math.atan2(top[1], top[0]))
        loc[c] = dict(K=K, f0=f0, n=n, d=d, tags=tags)
    if not loc:
        return None
    # join the cameras that share a tag: each camera's plane coordinates -> its group's (the first camera's)
    comps = []
    left = sorted(loc, key=lambda c: -len(loc[c]["tags"]))
    while left:
        base = left.pop(0)
        comp = {"cams": {base: (0.0, np.zeros(2))}, "tags": {i: [t["q"]] for i, t in loc[base]["tags"].items()}}
        grew = True
        while grew:
            grew = False
            for c in list(left):
                shared = [i for i in loc[c]["tags"] if i in comp["tags"]]
                if not shared:
                    continue
                P = np.vstack([loc[c]["tags"][i]["q"] for i in shared])
                Q = np.vstack([np.mean(comp["tags"][i], axis=0) for i in shared])
                ang, t2 = _kabsch2(P, Q)
                comp["cams"][c] = (ang, t2)
                for i, t in loc[c]["tags"].items():
                    comp["tags"].setdefault(i, []).append(t["q"] @ _rot2(ang).T + t2)
                left.remove(c)
                grew = True
        comps.append(comp)
    # each group's turn to the table frame, then one linear solve for the shifts, L, W and the side tags' places
    lay = prob.ref_lay
    sides = prob.ref_sides
    words = []
    quarters = {}
    nc = len(comps)
    ncol = 2 * nc + 2 + len(sides)
    rows, rhs = [], []
    for k, comp in enumerate(comps):
        tq = {i: np.mean(v, axis=0) for i, v in comp["tags"].items()}
        cand = []
        for i, q in tq.items():
            top = (q[0] + q[1]) / 2 - (q[2] + q[3]) / 2
            cand.append(math.radians(RT.heading(lay[i], 0)) - math.atan2(top[1], top[0]))
        # the turn most tags agree with (a paper tag turned on its square disagrees by a quarter turn and is outvoted)
        best = max(cand, key=lambda t0: sum(1 for t1 in cand if abs(wrap180(math.degrees(t1 - t0))) < 20))
        near = [t1 for t1 in cand if abs(wrap180(math.degrees(t1 - best))) < 20]
        th = math.atan2(np.mean([math.sin(v) for v in near]), np.mean([math.cos(v) for v in near]))
        comp["theta"] = th
        Rt = _rot2(th)
        for i, q in tq.items():
            top = (q[0] + q[1]) / 2 - (q[2] + q[3]) / 2
            hw = math.degrees(th + math.atan2(top[1], top[0]))
            quarters[i] = int(round(wrap180(hw - RT.heading(lay[i], 0)) / 90.0)) % 4
            cw_ = Rt @ q.mean(0)
            it = lay[i]
            # world centre = const + cL * L + cW * W + ca * along
            if it["kind"] == "corner":
                cx = {1: (off, 0, 0), 2: (-off, 1, 0), 3: (-off, 1, 0), 4: (off, 0, 0)}[it["corner"]]
                cy = {1: (off, 0, 0), 2: (off, 0, 0), 3: (-off, 0, 1), 4: (-off, 0, 1)}[it["corner"]]
                ca = (0.0, 0.0)
            else:
                e = it["edge"]
                cx = {1: (0, 0, 0), 2: (-off, 1, 0), 3: (0, 0, 0), 4: (off, 0, 0)}[e]
                cy = {1: (off, 0, 0), 2: (0, 0, 0), 3: (-off, 0, 1), 4: (0, 0, 0)}[e]
                ca = {1: (1.0, 0.0), 2: (0.0, 1.0), 3: (1.0, 0.0), 4: (0.0, 1.0)}[e]
            for ax, cc_ in ((0, cx), (1, cy)):
                r = np.zeros(ncol)
                r[2 * k + ax] = 1.0                       # the group's shift
                r[2 * nc] -= cc_[1]                       # L
                r[2 * nc + 1] -= cc_[2]                   # W
                if it["kind"] == "side" and i in sides:
                    r[2 * nc + 2 + sides.index(i)] -= ca[ax]
                rows.append(r)
                rhs.append(cc_[0] - cw_[ax])
    if prob.typed_L:
        for col, v in ((2 * nc, prob.typed_L), (2 * nc + 1, prob.typed_W)):
            r = np.zeros(ncol)
            r[col] = 1.0
            rows.append(r)
            rhs.append(float(v))
    for i, v in prob.typed_along.items():
        r = np.zeros(ncol)
        r[2 * nc + 2 + sides.index(i)] = 1.0
        rows.append(r)
        rhs.append(v)
    A, y = np.array(rows), np.array(rhs)
    sol, _, rank, _ = np.linalg.lstsq(A, y, rcond=None)
    if rank < ncol:
        raise ValueError("the reference tags do not fix the table: %d groups of cameras that share no tag, %d tags seen "
                         "(%s). Each camera needs a corner tag, and the two cameras a shared side tag (or type the "
                         "table's size)" % (nc, sum(len(c["tags"]) for c in comps),
                                            ", ".join(sorted({i for c in comps for i in c["tags"]}, key=int))))
    L0, W0 = float(sol[2 * nc]), float(sol[2 * nc + 1])
    along = {i: float(sol[2 * nc + 2 + j]) for j, i in enumerate(sides)}
    res = A @ sol - y
    bad = float(np.sqrt(np.mean(res ** 2)))
    if bad > 6.0:
        words.append("WARNING: the reference tags do not sit where their ids say (%.1f cm apart from their places): "
                      "is each tag in its own corner (ids %s)?" % (bad, ", ".join("%s = corner %d" % (i, it["corner"])
                                                                                 for i, it in sorted(lay.items())
                                                                                 if it["kind"] == "corner")))
    turned = sorted(i for i, q in quarters.items() if q)
    if turned:
        words.append("reference tag%s %s %s turned on %s square (fine: the turn is read)" % (
            "s" if len(turned) > 1 else "", ", ".join(turned), "are" if len(turned) > 1 else "is",
            "their" if len(turned) > 1 else "its"))
    placed = {}
    prob.ref_q.update(quarters)
    for comp in comps:
        for c in comp["cams"]:
            Pc, Pw = [], []
            for i, t in loc[c]["tags"].items():
                x, y_, hd = prob.ref_pose(i, L0, W0, along)
                Pw.append(tag_corners_world(x, y_, hd, z, s)[:, :2])
                Pc.append(t["P"])
            R, tt = rigid_from_plane(np.vstack(Pc), np.vstack(Pw), loc[c]["n"], np.full(4 * len(Pc), z))
            placed[c] = (R, tt, loc[c]["f0"])
            words.append("%s placed from %d reference tag%s (%s)" % (c, len(Pc), "" if len(Pc) == 1 else "s",
                                                                     ", ".join(sorted(loc[c]["tags"], key=int))))
    return dict(placed=placed, L=L0, W=W0, along=along, quarters=quarters, words=words, rms_cm=round(bad, 2))


def initial(obs, prob):
    """Starting values (see the module doc). Returns x0 and words about how each camera was placed."""
    s, H = prob.s, prob.H
    words = []
    lay = prob.layout()
    x0 = np.zeros(prob.n)
    tb = obs.get("table") or {}
    L0, W0 = tb.get("length_cm"), tb.get("width_cm")
    placed = {}
    planes = {}
    for c in prob.cams:
        size = obs["cameras"][c]["size"]
        f0 = obs["cameras"][c].get("f_prior") or (size[0] / 2.0) / math.tan(math.radians(HFOV_PRIOR_DEG) / 2)
        K = K_of(f0, size)
        reads = [g["reads"][c] for g in prob.groups if c in g["reads"]]
        pl = plane_from_tags(K, reads, s, H)
        if pl is None and prob.ref and (prob.ref_reads.get(c) or {}):
            # no robot tag in this camera: the reference tags give its table plane (they lie on it)
            pl = plane_from_tags(K, list((prob.ref_reads.get(c) or {}).values()), prob.ref_s, prob.ref_z)
        planes[c] = (K, f0, pl)
    if prob.ref:
        r = ref_initial(obs, prob)
        if r:
            placed.update(r["placed"])
            L0, W0 = r["L"], r["W"]
            words += r["words"]
            prob.ref_q.update({str(k): int(v) for k, v in r["quarters"].items()})
            prob.ref_along0 = r["along"]
            words.append("reference tags: table %.1f x %.1f cm to start with" % (L0, W0))
    # the table's size: typed, else measured from a camera that sees 1-2 or 1-4 edges
    if not L0:
        L0, W0 = None, None
        for c in prob.cams:
            K, f0, pl = planes[c]
            cc = (obs.get("corners") or {}).get(c) or {}
            if pl is None or len(cc) < 2:
                continue
            P = {int(k): ray_plane(K, 0.0, v, pl[0], pl[1])[0] for k, v in cc.items()}
            if 1 in P and 2 in P:
                L0 = float(np.linalg.norm(P[2] - P[1]))
            if 1 in P and 4 in P:
                W0 = float(np.linalg.norm(P[4] - P[1]))
            if 2 in P and 3 in P and W0 is None:
                W0 = float(np.linalg.norm(P[3] - P[2]))
            if 4 in P and 3 in P and L0 is None:
                L0 = float(np.linalg.norm(P[3] - P[4]))
        W0 = W0 or 150.0
        L0 = L0 or 2.0 * W0
        words.append("starting size from the tags' scale: %.0f x %.0f cm" % (L0, W0))
    cw = {1: (0, 0), 2: (L0, 0), 3: (L0, W0), 4: (0, W0)}
    # cameras with >= 2 corners: placed from their own plane
    for c in prob.cams:
        K, f0, pl = planes[c]
        cc = (obs.get("corners") or {}).get(c) or {}
        if pl is None or len(cc) < 2 or c in placed:
            continue
        ks = sorted(int(k) for k in cc)
        Pc = ray_plane(K, 0.0, [cc[str(k)] if str(k) in cc else cc[k] for k in ks], pl[0], pl[1])
        R, t = rigid_from_plane(Pc, np.array([cw[k] for k in ks], float), pl[0])
        placed[c] = (R, t, f0)
        words.append("%s placed from its %d corners and %d tag reads" % (c, len(ks), sum(1 for g in prob.groups
                                                                                     if c in g["reads"])))
    # group / sticker starting spots from the placed cameras
    def world_of(c, uv, z):
        R, t, f0 = placed[c]
        K = K_of(f0, obs["cameras"][c]["size"])
        n_c = R[:, 2]
        d = float(np.dot(n_c, t)) + z           # plane z (world) in camera frame: n.(R p + t) ... p_z = z
        Pc = ray_plane(K, 0.0, uv, n_c, d)
        return (Pc - t) @ R                     # R.T (Pc - t)
    for _ in range(2):
        for c in prob.cams:
            if c in placed:
                continue
            K, f0, pl = planes[c]
            if pl is None:
                continue
            Pc, Pw, Zs = [], [], []
            for g in prob.groups:
                if c in g["reads"]:
                    other = [o for o in g["reads"] if o in placed]
                    if other:
                        mid_w = world_of(other[0], [np.mean(g["reads"][other[0]], axis=0)], H)[0]
                        mid_c = ray_plane(K, 0.0, [np.mean(g["reads"][c], axis=0)], pl[0], pl[1] + H)[0]
                        # (the tag's middle on the tag plane, in this camera: the table plane moved H up)
                        Pc.append(mid_c)
                        Pw.append(mid_w[:2])
                        Zs.append(H)
            for sid, st in (obs.get("stickers") or {}).items():
                cs = st.get("cams") or {}
                if c in cs:
                    other = [o for o in cs if o in placed]
                    if other:
                        Pw.append(world_of(other[0], [cs[other[0]]], 0.0)[0][:2])
                        Pc.append(ray_plane(K, 0.0, [cs[c]], pl[0], pl[1])[0])
                        Zs.append(0.0)
            cc = (obs.get("corners") or {}).get(c) or {}
            for k_, v in cc.items():
                Pc.append(ray_plane(K, 0.0, [v], pl[0], pl[1])[0])
                Pw.append(np.array(cw[int(k_)], float))
                Zs.append(0.0)
            if len(Pw) >= 2:
                R, t = rigid_from_plane(np.array(Pc), np.array(Pw, float), pl[0], Zs)
                placed[c] = (R, t, f0)
                words.append("%s placed from %d points it shares with the other camera" % (c, len(Pw)))
    for c in prob.cams:
        a, b = lay[c]
        if c not in placed:
            raise ValueError("%s could not be placed: click at least two table corners in its picture, or make sure "
                             "it sees a robot's tag that the other camera sees too" % c)
        R, t, f0 = placed[c]
        x0[a:a + 3] = _rv(R)
        x0[a + 3:a + 6] = t
        x0[a + 6] = f0
    x0[lay["LW"][0]:lay["LW"][1]] = [L0, W0]
    al0 = getattr(prob, "ref_along0", {}) or {}
    for j, i in enumerate(prob.ref_sides):
        it = prob.ref_lay[i]
        x0[lay["_along"][0] + j] = al0.get(i, (L0 if it["edge"] in (1, 3) else W0) / 2.0)
    if "_kref" in lay:
        x0[lay["_kref"][0]] = 1.0
    if prob.fit_height:
        x0[lay["H"][0]] = prob.H
    sa = lay["stk"][0]
    for i, sid in enumerate(prob.stickers):
        cs = obs["stickers"][sid].get("cams") or {}
        c = next(iter(cs))
        x0[sa + 2 * i: sa + 2 * i + 2] = world_of(c, [cs[c]], 0.0)[0][:2]
    ga = lay["grp"][0]
    for i, g in enumerate(prob.groups):
        c = next(iter(g["reads"]))
        P = world_of(c, g["reads"][c], H)
        mid = P.mean(0)
        front = (P[0] + P[1]) / 2 - (P[2] + P[3]) / 2
        x0[ga + 3 * i: ga + 3 * i + 3] = [mid[0], mid[1], math.degrees(math.atan2(front[1], front[0]))]
    return x0, words


def solve(obs, use_typed=True, fit_k1=None, fit_height=False, x_start=None, fit_tag_scale=False):
    """The fit. Returns a dict: ok, cameras {name: CameraModel dict + summary}, size_cm, stickers {id: [x, y]},
    residuals, checks (typed values against the cameras'), words (plain problems and checks)."""
    from scipy.optimize import least_squares
    if fit_k1 is None:
        fit_k1 = True       # a webcam's lens bends (the simulated ones too); with the 0 +- 0.15 prior a straight lens
                            # stays straight. Left out, a bent lens cost up to 4 cm on the table (self-test)
    prob = Problem(obs, use_typed=use_typed, fit_k1=fit_k1, fit_height=fit_height, fit_tag_scale=fit_tag_scale)
    if x_start is not None:
        prob.layout()
        x0, words = np.array(x_start, float), ["started from the fit with the typed measurements"]
        if len(x0) != prob.n:
            x0, words = initial(obs, prob)
    else:
        x0, words = initial(obs, prob)
    r = least_squares(prob.resid, x0, loss="soft_l1", f_scale=3.0, x_scale="jac", max_nfev=400)
    x = r.x
    L, W = x[prob.lay["LW"][0]:prob.lay["LW"][1]]
    out = {"ok": bool(r.success), "size_cm": [round(float(L), 2), round(float(W), 2)], "words": list(words),
           "used_typed": use_typed, "k1_fitted": fit_k1, "cameras": {}, "stickers": {}, "groups": len(prob.groups)}
    # uncertainty of L, W and each camera's height, from the Jacobian (sigma ~ sqrt(diag((J'J)^-1)))
    try:
        J = r.jac
        cov = np.linalg.pinv(J.T @ J)
        sd = np.sqrt(np.clip(np.diag(cov), 0, None))
    except Exception:    # noqa: BLE001
        sd = np.full(len(x), np.nan)
    a = prob.lay["LW"][0]
    out["size_sd_cm"] = [round(float(sd[a]), 2), round(float(sd[a + 1]), 2)]
    from cameras import CameraModel
    res = prob.resid(x)
    for c in prob.cams:
        rv, tv, f, k1 = prob.cam_params(x, c)
        size = obs["cameras"][c]["size"]
        m = CameraModel(K_of(f, size), [k1, 0, 0, 0, 0], rv, tv, size, name=c,
                        fit={"method": "site_fit (tags, corners, stickers)", "f_sd_px": round(float(sd[prob.lay[c][0] + 6]), 2)})
        d = m.to_dict()
        out["cameras"][c] = d
    sa = prob.lay["stk"][0]
    out["sticker_sd_cm"] = {}
    for i, sid in enumerate(prob.stickers):
        out["stickers"][sid] = [round(float(x[sa + 2 * i]), 2), round(float(x[sa + 2 * i + 1]), 2)]
        out["sticker_sd_cm"][sid] = round(float(math.hypot(sd[sa + 2 * i], sd[sa + 2 * i + 1])), 2)
    ga = prob.lay["grp"][0]
    out["group_poses"] = [[round(float(v), 3) for v in x[ga + 3 * i: ga + 3 * i + 3]] for i in range(len(prob.groups))]
    # per-kind pixel residuals (unweighted px)
    out["rms_px"] = rms_by_kind(prob, x)
    out["x"] = [float(v) for v in x]
    if prob.fit_tag_scale:
        out["tag_scale"] = round(float(x[prob.lay["k"][0]]), 4)
    if prob.ref:
        along = {i: float(x[prob.lay["_along"][0] + j]) for j, i in enumerate(prob.ref_sides)}
        out["ref_tags"] = {"poses": {i: [round(v, 3) for v in prob.ref_pose(i, float(L), float(W), along)]
                                     for i in prob.ref_lay},
                           "seen": {c: sorted(prob.ref_reads.get(c, {}), key=int) for c in prob.cams},
                           "along_cm": {i: round(v, 2) for i, v in along.items()},
                           "along_sd_cm": {i: round(float(sd[prob.lay["_along"][0] + j]), 2)
                                           for j, i in enumerate(prob.ref_sides)},
                           "quarters": dict(prob.ref_q), "tag_cm": prob.ref_s, "z_cm": prob.ref_z,
                           "offset_cm": prob.ref_off}
        if "_kref" in prob.lay:
            out["ref_tag_scale"] = round(float(x[prob.lay["_kref"][0]]), 4)
    out["cost"] = float(r.cost)
    out["nfev"] = int(r.nfev)
    return out


def rms_by_kind(prob, x):
    """RMS pixel error of each kind of observation, per camera."""
    obs = prob.obs
    L, W = x[prob.lay["LW"][0]:prob.lay["LW"][1]]
    H = x[prob.lay["H"][0]] if prob.fit_height else prob.H
    cw = {1: (0, 0), 2: (L, 0), 3: (L, W), 4: (0, W)}
    sa, ga = prob.lay["stk"][0], prob.lay["grp"][0]
    out = {}
    for c in prob.cams:
        rv, tv, f, k1 = prob.cam_params(x, c)
        K = K_of(f, obs["cameras"][c]["size"])
        e = {}
        cc = (obs.get("corners") or {}).get(c) or {}
        if cc:
            P = np.array([[*cw[int(k)], 0] for k in cc])
            e["corners"] = float(np.sqrt(np.mean(np.sum((project(K, k1, rv, tv, P) - np.array(list(cc.values()))) ** 2,
                                                         1))))
        d = []
        for i, sid in enumerate(prob.stickers):
            u = (obs["stickers"][sid].get("cams") or {}).get(c)
            if u is not None:
                d.append(np.linalg.norm(project(K, k1, rv, tv, np.array([[x[sa + 2 * i], x[sa + 2 * i + 1], 0]]))[0] - u))
        if d:
            e["stickers"] = float(np.sqrt(np.mean(np.square(d))))
        d = []
        for gi, g in enumerate(prob.groups):
            if c in g["reads"]:
                Pw = tag_corners_world(*x[ga + 3 * gi: ga + 3 * gi + 3], H,
                                       prob.s * (x[prob.lay["k"][0]] if prob.fit_tag_scale else 1.0))
                d.append(np.linalg.norm(project(K, k1, rv, tv, Pw) - np.asarray(g["reads"][c]), axis=1))
        if d:
            e["tags"] = float(np.sqrt(np.mean(np.square(np.concatenate(d)))))
            e["tag_reads"] = len(d)
        if prob.ref and prob.ref_reads.get(c):
            along = {i: x[prob.lay["_along"][0] + j] for j, i in enumerate(prob.ref_sides)}
            kref = x[prob.lay["_kref"][0]] if "_kref" in prob.lay else 1.0
            d = [np.linalg.norm(project(K, k1, rv, tv, tag_corners_world(*prob.ref_pose(i, L, W, along), prob.ref_z,
                                                                          prob.ref_s * kref)) - np.asarray(cs), axis=1)
                 for i, cs in prob.ref_reads[c].items()]
            e["ref"] = float(np.sqrt(np.mean(np.square(np.concatenate(d)))))
            e["ref_tags"] = len(d)
        out[c] = {k: round(v, 3) if isinstance(v, float) else v for k, v in e.items()}
    return out


def checks(obs, untyped, typed=None):
    """The typed measurements against the cameras' own (the fit without them): [(what, cameras, typed, off, ok)] and
    plain words for each disagreement."""
    rows, words = [], []
    tb = obs.get("table") or {}
    if tb.get("length_cm"):
        for k, nm in ((0, "length"), (1, "width")):
            cam_v, typ = untyped["size_cm"][k], float(tb["length_cm" if k == 0 else "width_cm"])
            off = cam_v - typ
            sd = float((untyped.get("size_sd_cm") or [0, 0])[k] or 0.0)
            # the cameras' own measure is only a check as good as its own uncertainty: a difference inside 3 of its
            # sigma is not a disagreement (2 still robots give the scale to a few %; a run's many tag spots to 0.1 %)
            tol = max(AGREE_FRAC * typ, 3.0 * sd)
            ok = abs(off) <= tol
            rows.append(dict(what="table %s" % nm, cameras=cam_v, typed=typ, off=round(off, 2), ok=ok,
                             cameras_sd=round(sd, 2), weak=bool(3.0 * sd > AGREE_FRAC * typ)))
            if ok and 3.0 * sd > AGREE_FRAC * typ:
                words.append("The cameras alone measure the table's %s only to +- %.1f cm here (few tag spots: drive "
                             "the robots about, then Refit); your %.1f cm is within that and is used." % (nm, sd, typ))
            if not ok:
                words.append("The cameras measure the table's %s as %.1f cm; you typed %.1f cm (%.1f %% apart). Check "
                             "the tape, the tag height (%s cm) and the printed tag size (%s cm): one of them is off."
                             % (nm, cam_v, typ, 100 * off / typ, obs["tag_height_cm"], obs["tag_cm"]))
    st = untyped.get("stickers") or {}
    for sid, d in (obs.get("stickers") or {}).items():
        xy = d.get("xy_cm")
        if xy is not None and sid in st:
            off = math.hypot(st[sid][0] - xy[0], st[sid][1] - xy[1])
            ok = off <= max(AGREE_CM, 3.0 * float((untyped.get("sticker_sd_cm") or {}).get(sid) or 0.0))
            rows.append(dict(what="sticker %s spot" % sid, cameras=st[sid], typed=list(xy), off=round(off, 2), ok=ok))
            if not ok:
                words.append("Sticker %s: the cameras put it at (%.1f, %.1f) cm, you typed (%.1f, %.1f): %.1f cm apart. "
                             "Check the click in each picture and the tape." % (sid, st[sid][0], st[sid][1], xy[0],
                                                                              xy[1], off))
    rt = untyped.get("ref_tags") or {}
    for i, v in ((obs.get("ref_tags") or {}).get("typed_along_cm") or {}).items():
        cam_v = (rt.get("along_cm") or {}).get(str(i))
        if cam_v is None:
            continue
        off = cam_v - float(v)
        ok = abs(off) <= max(AGREE_CM, 3.0 * float((rt.get("along_sd_cm") or {}).get(str(i)) or 0.0))
        rows.append(dict(what="side tag %s along its edge" % i, cameras=cam_v, typed=float(v), off=round(off, 2), ok=ok))
        if not ok:
            words.append("Side tag %s: the cameras put it %.1f cm along its edge, you typed %.1f cm (%.1f cm apart)."
                         % (i, cam_v, float(v), off))
    for a_, b_, dcm in obs.get("distances", []):
        if a_ in st and b_ in st:
            cam_d = math.hypot(st[a_][0] - st[b_][0], st[a_][1] - st[b_][1])
            off = cam_d - dcm
            ok = abs(off) <= max(AGREE_FRAC * dcm, 0.5)
            rows.append(dict(what="distance %s-%s" % (a_, b_), cameras=round(cam_d, 2), typed=dcm, off=round(off, 2),
                             ok=ok))
            if not ok:
                words.append("Stickers %s to %s: the cameras measure %.1f cm, you typed %.1f cm (%.1f %% apart)."
                             % (a_, b_, cam_d, dcm, 100 * off / dcm))
    return rows, words


def full(obs, fit_k1=None, untyped_check=True):
    """The result used, with its checks.

    With typed measurements (Nick has the table's size: 66 x 156 in): the fit with them, the tags' read size fitted
    too (k: a blurred tag's corners come out a little inside it, which alone made the table 1.5-3.5 % large in the dry
    runs). Checks, all said in words, none silently averaged: each typed value against the fit (a value the cameras
    and the other values do not support is named); k against 1 (over TAG_SCALE_WARN: the printer scaled the page, or
    the tape is off); and the cameras' own measure (tags alone, k = 1) for comparison.
    Without typed measurements: the tags alone (k = 1), and the words say the scale rests on the tags' printed size."""
    has_typed = bool((obs.get("table") or {}).get("length_cm")) or any(
        d.get("xy_cm") is not None for d in (obs.get("stickers") or {}).values()) or bool(obs.get("distances")) or \
        bool((obs.get("ref_tags") or {}).get("typed_along_cm"))
    if not has_typed:
        a = solve(obs, use_typed=False, fit_k1=fit_k1)
        a["checks"], a["words"] = [], a["words"] + ["No measurements typed: the scale rests on the tags' printed size "
                                                    "alone (a blurred tag reads 1.5-3.5 % small: type the table's size)."]
        return a
    b = solve(obs, use_typed=True, fit_k1=fit_k1, fit_tag_scale=True)
    a = solve(obs, use_typed=False, fit_k1=fit_k1, x_start=None) if untyped_check else \
        {"size_cm": b["size_cm"], "size_sd_cm": b.get("size_sd_cm")}
    rows, words = checks(obs, b)
    k = b.get("tag_scale", 1.0)
    rows.append(dict(what="tag read size / printed", cameras=k, typed=1.0, off=round(k - 1.0, 4),
                     ok=abs(k - 1.0) <= TAG_SCALE_WARN))
    if "ref_tag_scale" in b:
        kr = b["ref_tag_scale"]
        rows.append(dict(what="reference tag read size / printed", cameras=kr, typed=1.0, off=round(kr - 1.0, 4),
                         ok=abs(kr - 1.0) <= TAG_SCALE_WARN))
        if abs(kr - 1.0) > TAG_SCALE_WARN:
            words.append("The reference tags read %.1f %% %s than printed, against your typed measurements. Measure a "
                         "printed reference tag's black square (should be %.1f cm) and the table again."
                         % (100 * abs(kr - 1), "larger" if kr > 1 else "smaller", obs["ref_tags"].get("tag_cm", 12.0)))
    if abs(k - 1.0) > TAG_SCALE_WARN:
        words.append("The tags read %.1f %% %s than printed, against your typed measurements. Measure a printed "
                     "tag's black square (should be %.1f cm) and the table again: one of them is off."
                     % (100 * abs(k - 1), "larger" if k > 1 else "smaller", obs["tag_cm"]))
    b["untyped_size_cm"], b["untyped_size_sd_cm"] = a["size_cm"], a.get("size_sd_cm")
    if untyped_check and b.get("ref_tags") and obs.get("tag_groups"):
        try:
            hc = height_check(obs, b, fit_k1=fit_k1)
            b["height_check"] = hc
            words.append(hc["words"])
        except Exception as e:    # noqa: BLE001  (a check never stops the setup)
            words.append("(height cross-check not made: %s)" % e)
    b["checks"] = rows
    b["words"] = b["words"] + words + [
        "Typed measurements used; the tags read %.1f %% %s than printed (fitted%s)." % (
            100 * abs(k - 1), "larger" if k >= 1 else "smaller",
            ("; tags alone would make the table %.1f x %.1f cm" % tuple(a["size_cm"])) if untyped_check else "")]
    if all(r["ok"] for r in rows):
        b["words"].append("Every typed measurement agrees with the cameras.")
    return b


def height_check(obs, res, fit_k1=None):
    """The roof tags as the cameras' height cross-check (2026-11-03): the fit without them (the reference tags alone,
    flat on the table) against the fit with them. A camera looking straight down at a flat table cannot tell its height
    from its zoom (the focal length's weak prior decides); the roof tags, a known height above the table, can. Each
    camera's height both ways, and the words."""
    o2 = dict(obs, tag_groups=[])
    r2 = solve(o2, use_typed=True, fit_k1=fit_k1, fit_tag_scale=True)
    out = {}
    for c, d in res["cameras"].items():
        h1 = float(d["camera_position_cm"][2])
        h0 = float(r2["cameras"][c]["camera_position_cm"][2]) if c in r2["cameras"] else float("nan")
        out[c] = {"with_roofs_cm": round(h1, 1), "refs_alone_cm": round(h0, 1)}
    words = ("Camera height cross-check: with the robots' roof tags %s; from the flat reference tags alone %s (flat tags "
             "alone cannot tell height from zoom when a camera looks straight down: the roof tags, %.1f cm up, decide "
             "it; a big gap with few roof reads: drive the robots about, then Refit)." % (
                 ", ".join("%s %.0f cm" % (c, v["with_roofs_cm"]) for c, v in out.items()),
                 ", ".join("%s %.0f cm" % (c, v["refs_alone_cm"]) for c, v in out.items()), float(obs["tag_height_cm"])))
    return {"cameras": out, "words": words}


# ------------------------------------------------------------------------------------------------- inputs
def groups_from_obs_csv(path, ids_to_robot=None, pair_s=0.07, max_groups=MAX_GROUPS, still_cm=None):
    """tag_obs.csv (tag_bridge.py) -> tag groups: the reads of one tag within pair_s by different cameras are one
    moment. At most max_groups, spread evenly over the run."""
    import csv
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r["t"]), r["cam"], int(r["id"]),
                         [[float(r["u%d" % k]), float(r["v%d" % k])] for k in range(4)]))
    rows.sort()
    groups = []
    open_ = {}
    for t, cam, i, cs in rows:
        g = open_.get(i)
        if g is not None and t - g["t"] <= pair_s and cam not in g["reads"]:
            g["reads"][cam] = cs
            continue
        g = {"id": i, "t": t, "reads": {cam: cs}}
        groups.append(g)
        open_[i] = g
    both = [g for g in groups if len(g["reads"]) > 1]
    one = [g for g in groups if len(g["reads"]) == 1]
    pick = both[:: max(1, len(both) // max(1, max_groups // 2))][: max_groups // 2]
    rest = max_groups - len(pick)
    pick += one[:: max(1, len(one) // max(1, rest))][:rest]
    return pick


# ------------------------------------------------------------------------------------------------- self-test
def simulate(seed=1, size=(1280, 960), height=200.0, f_err=None, click_sd=1.5, tag_sd=0.3, n_spots=24, typed=True,
             stickers=10, table=(396.24, 167.64), H=6.0, s=8.0, refs=False, corners=True, hide_ref=(), ref_sd=0.2,
             along=None):
    """Synthetic observations from two true cameras (tag_cam.overhead_pair) and the truth to compare with.
    refs: the six reference tags read (ref_tags.py; hide_ref: ids not seen; along: the side tags' true places);
    corners: the corner clicks (off: the school's way with reference tags)."""
    import tag_cam as CAMS
    rng = np.random.default_rng(seed)
    L, W = table
    cams = CAMS.overhead_pair(table_cm=table, height_cm=height, size=size, hfov_deg=74.0)
    # each camera tilted a little and turned, as a real mount would be
    for k, c in enumerate(cams):
        c.Rw = _R(rng.normal(0, 0.06, 3)) @ c.Rw
        c.rvec = _rv(c.Rw)
        c.tvec = -c.Rw @ c.C
        c.dist[0] = rng.normal(0, 0.03)
    truth = {c.name: c.true_model() for c in cams}
    obs = {"cameras": {}, "corners": {}, "tag_cm": s, "tag_height_cm": H, "stickers": {}, "tag_groups": [],
           "distances": []}
    for c in cams:
        obs["cameras"][c.name] = {"size": list(size), "f_prior": None if f_err is None else c.f * (1 + f_err)}
        cc = {}
        for k, (x, y) in {1: (0, 0), 2: (L, 0), 3: (L, W), 4: (0, W)}.items():
            uv = c.project([[x, y, 0]])[0]
            if np.all(np.isfinite(uv)) and 0 <= uv[0] < size[0] and 0 <= uv[1] < size[1]:
                cc[str(k)] = (uv + rng.normal(0, click_sd, 2)).tolist()
        obs["corners"][c.name] = cc if corners else {}
    if refs:
        import ref_tags as RT
        cfg = RT.default_config()
        al = along or {"24": L / 2 + 6.0, "25": L / 2 - 9.0}
        obs["ref_tags"] = {"layout": RT.layout(cfg), "tag_cm": RT.TAG_CM, "z_cm": RT.Z_CM, "offset_cm": RT.OFFSET_CM,
                           "reads": {}, "truth_along": al}
        for c in cams:
            rd = {}
            for i, (x, y, th) in RT.world_poses(cfg, L, W, along=al).items():
                if i in [str(h) for h in hide_ref]:
                    continue
                uv = c.project(tag_corners_world(x, y, th, RT.Z_CM, RT.TAG_CM))
                if np.all(np.isfinite(uv)) and np.all((uv[:, 0] > 2) & (uv[:, 0] < size[0] - 2) & (uv[:, 1] > 2)
                                                      & (uv[:, 1] < size[1] - 2)):
                    rd[i] = (uv + rng.normal(0, ref_sd, uv.shape)).tolist()
            obs["ref_tags"]["reads"][c.name] = rd
    st_truth = {}
    for k in range(stickers):
        sx, sy = rng.uniform(15, L - 15), rng.uniform(15, W - 15)
        st_truth[str(k + 1)] = (sx, sy)
        cs = {}
        for c in cams:
            uv = c.project([[sx, sy, 0]])[0]
            if np.all(np.isfinite(uv)) and 5 <= uv[0] < size[0] - 5 and 5 <= uv[1] < size[1] - 5:
                cs[c.name] = (uv + rng.normal(0, click_sd, 2)).tolist()
        if cs:
            obs["stickers"][str(k + 1)] = {"cams": cs}
    if typed:
        obs["table"] = {"length_cm": L, "width_cm": W}
        keys = list(obs["stickers"])
        for a_, b_ in zip(keys[:-1:2], keys[1::2]):
            obs["distances"].append([a_, b_, float(np.hypot(*np.subtract(st_truth[a_], st_truth[b_])))])
    for k in range(n_spots):
        x, y, th = rng.uniform(10, L - 10), rng.uniform(10, W - 10), rng.uniform(-180, 180)
        if k < 4:
            x = L / 2 + rng.uniform(-20, 20)       # some in the overlap
        P = tag_corners_world(x, y, th, H, s)
        reads = {}
        for c in cams:
            uv = c.project(P)
            if np.all(np.isfinite(uv)) and np.all((uv[:, 0] > 2) & (uv[:, 0] < size[0] - 2) & (uv[:, 1] > 2)
                                                  & (uv[:, 1] < size[1] - 2)):
                reads[c.name] = (uv + rng.normal(0, tag_sd, uv.shape)).tolist()
        if reads:
            obs["tag_groups"].append({"id": k % 2, "reads": reads, "truth": [x, y, th]})
    return obs, truth, st_truth


def compare(res, truth, st_truth, H=6.0, table=(396.24, 167.64)):
    """Floor error of each fitted camera against the truth: a grid over the table at the table and at the tag plane,
    where each camera sees it; the sticker spots; the size."""
    from cameras import CameraModel
    out = {}
    L, W = table
    xs, ys = np.meshgrid(np.linspace(0, L, 40), np.linspace(0, W, 15))
    P = np.column_stack([xs.ravel(), ys.ravel()])
    for c, d in res["cameras"].items():
        m, t = CameraModel.from_dict(d), truth[c]
        e = []
        for z in (0.0, H):
            u, v = t.project(np.column_stack([P, np.full(len(P), z)]))
            ok = t.in_image(u, v)
            x2, y2 = m.to_floor(u[ok], v[ok], z)
            e.append(np.hypot(x2 - P[ok, 0], y2 - P[ok, 1]))
        e = np.concatenate(e)
        out[c] = dict(median_cm=round(float(np.nanmedian(e)), 2), worst_cm=round(float(np.nanmax(e)), 2),
                      height_cm=round(float(m.center[2]), 1), true_height_cm=round(float(t.center[2]), 1),
                      f=round(m.f, 1), true_f=round(t.f, 1))
    st = [math.hypot(res["stickers"][k][0] - v[0], res["stickers"][k][1] - v[1]) for k, v in st_truth.items()
          if k in res["stickers"]]
    out["stickers_worst_cm"] = round(max(st), 2) if st else None
    return out


def _selftest():
    ok = True
    for name, kw in (("typed size and distances", {}), ("tags alone (nothing typed)", {"typed": False}),
                     ("focal prior 20 % off", {"f_err": 0.2}), ("640 x 480", {"size": (640, 480), "tag_sd": 0.5})):
        obs, truth, st = simulate(**kw)
        res = full(obs)
        cmp_ = compare(res, truth, st)
        L, W = res["size_cm"]
        worst = max(v["worst_cm"] for k, v in cmp_.items() if isinstance(v, dict))
        print("%-28s size %.1f x %.1f (true 396.2 x 167.6), floor worst %.2f cm, stickers worst %s cm, heights %s"
              % (name, L, W, worst, cmp_["stickers_worst_cm"],
                 ", ".join("%s %.0f (true %.0f)" % (c, v["height_cm"], v["true_height_cm"]) for c, v in cmp_.items()
                           if isinstance(v, dict))))
        ok = ok and abs(L - 396.24) < 3 and worst < 1.5
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


def refit(obs_path, tag_obs_csv, out=None, max_groups=MAX_GROUPS):
    """Nick's idea (2 Nov), after a run: the setup's observations plus the run's tag reads (tag_bridge's tag_obs.csv:
    the robots at many spots, seen by one or both cameras) fitted again. The tag plane against the table plane across
    the whole table pins each camera's height, tilt and lens far better than two still robots. Returns the result;
    saved (camera models, site_fit.json) to out when given."""
    with open(obs_path) as f:
        obs = json.load(f)
    groups = groups_from_obs_csv(tag_obs_csv, max_groups=max_groups)
    obs = dict(obs, tag_groups=list(obs.get("tag_groups") or []) + groups)
    res = full(obs, untyped_check=False)
    res["words"].insert(0, "refit with %d moments from the run (%d seen by both cameras)"
                        % (len(groups), sum(1 for g in groups if len(g["reads"]) > 1)))
    if out:
        save(res, out)
        with open(os.path.join(out, "obs_refit.json"), "w") as f:
            json.dump(obs, f, indent=1)
    return res


# ------------------------------------------------------------------------------------------------- refit in the background
# (2026-11-03, risk review #10) the refit after a run took 2 min 19 s here (about 10 min on a Pi, est.) and blocked the
# run's end and Mission Control's screen. Now it runs as its own process; status.json in its folder says where it is,
# and the Refit button reuses its result instead of computing it again.
STATUS_FILE = "status.json"


def _write_status(out, **kw):
    os.makedirs(out, exist_ok=True)
    kw["t"] = round(__import__("time").time(), 1)
    tmp = os.path.join(out, STATUS_FILE + ".tmp")
    with open(tmp, "w") as f:
        json.dump(kw, f, indent=1, default=str)
    os.replace(tmp, os.path.join(out, STATUS_FILE))


def _pid_alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, ValueError, TypeError):
        return False


def refit_status(out):
    """{state: none | running | done | failed, words, size_cm, rms_px, started, seconds} for a refit folder."""
    import time as _t
    p = os.path.join(out, STATUS_FILE)
    try:
        with open(p) as f:
            st = json.load(f)
    except (OSError, ValueError):
        if os.path.exists(os.path.join(out, "site_fit.json")):
            # a refit made before status files (or by the old blocking way): its result is there
            with open(os.path.join(out, "site_fit.json")) as f:
                r = json.load(f)
            return {"state": "done", "words": r.get("words") or [], "size_cm": r.get("size_cm"),
                    "rms_px": r.get("rms_px"), "folder": out}
        return {"state": "none", "folder": out}
    st["folder"] = out
    if st.get("state") == "running":
        st["seconds"] = round(_t.time() - float(st.get("started") or _t.time()), 1)
        if st.get("pid") and not _pid_alive(st["pid"]):
            st["state"] = "failed"
            st["why"] = "the refit process ended without a result (stopped or crashed: see refit.txt)"
    return st


def start_refit_bg(obs_path, tag_obs_csv, out, python=None, cwd=None):
    """Starts the refit as its own process (it keeps going if the caller ends); returns its status."""
    import subprocess
    import time as _t
    os.makedirs(out, exist_ok=True)
    t0 = round(_t.time(), 1)
    _write_status(out, state="running", pid=None, started=t0, obs=obs_path, reads=tag_obs_csv)
    log = open(os.path.join(out, "refit.txt"), "w")
    p = subprocess.Popen([python or sys.executable, os.path.abspath(__file__), "--obs", obs_path, "--refit", tag_obs_csv,
                          "--out", out, "--status"], stdout=log, stderr=subprocess.STDOUT, cwd=cwd or HERE,
                         start_new_session=True)
    log.close()
    st = refit_status(out)
    if st.get("state") == "running" and not st.get("pid"):
        _write_status(out, state="running", pid=p.pid, started=t0, obs=obs_path, reads=tag_obs_csv)
    return refit_status(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--refit", metavar="TAG_OBS_CSV", help="with --obs: add a run's tag reads (tag_obs.csv) and fit again")
    ap.add_argument("--obs", help="observations JSON (Mission Control's setup writes it)")
    ap.add_argument("--out", help="folder for the camera models (<camera>.json) and site_fit.json")
    ap.add_argument("--status", action="store_true", help="with --refit: write status.json in --out (running, done, "
                                                          "failed) for Mission Control's Refit card")
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest()
    if not a.obs:
        print(__doc__)
        return 1
    if a.refit:
        import time as _t
        t0 = _t.time()
        if a.status and a.out:
            prev = refit_status(a.out)
            _write_status(a.out, state="running", pid=os.getpid(), started=prev.get("started") or round(t0, 1),
                          obs=a.obs, reads=a.refit)
        try:
            res = refit(a.obs, a.refit, a.out)
        except Exception as e:     # noqa: BLE001
            if a.status and a.out:
                _write_status(a.out, state="failed", why=str(e), seconds=round(_t.time() - t0, 1))
            raise
        if a.status and a.out:
            _write_status(a.out, state="done", words=res["words"], size_cm=res["size_cm"], rms_px=res["rms_px"],
                          seconds=round(_t.time() - t0, 1))
        print("\n".join(res["words"]))
        return 0
    with open(a.obs) as f:
        obs = json.load(f)
    res = full(obs)
    save(res, a.out)
    print("\n".join(res["words"]))
    return 0


def save(res, out):
    os.makedirs(out, exist_ok=True)
    for c, d in res["cameras"].items():
        with open(os.path.join(out, c + ".json"), "w") as f:
            json.dump(d, f, indent=1)
    with open(os.path.join(out, "site_fit.json"), "w") as f:
        json.dump({k: v for k, v in res.items() if k != "cameras"}, f, indent=1, default=str)


if __name__ == "__main__":
    sys.exit(main())
