"""A simulated camera for the AprilTag robots (conference-table build, 2026-11-02): a plain pinhole camera anywhere
over a table, drawing the table, its numbered stickers (production points), and each robot as a box with a flat roof
carrying its printed tag at a known height. Drawn the way a real webcam blurs and spoils a picture: the scene drawn
SUPERSAMPLE times finer and averaged down (so a tag edge lands between pixels), then lens blur, motion blur along the
robot's travel, sensor noise, an exposure (gain, offset, clipping at white) and JPEG.

Used by the sizing tests (tools/fleet/tag_sizing.py), the camera check-out's tests, and the simulator's tag cameras
(tools/sim/fleet_sim.py --tag-cams), so the real detector (tools/fleet/tagcore.py) reads every simulated picture.
The camera's true model (tools/recognition/cameras.py's CameraModel) is what a perfect setup would measure.
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

import tagcore as TC   # noqa: E402

SUPERSAMPLE = 3          # each tag drawn this many times finer, then averaged down (sub-pixel edges). No DOE: drawing
ROBOT_BODY_CM = 9.5      # the Alvik's footprint (e02_camera.ROBOT_CM); the roof has the same footprint
ROBOT_BODY_H_CM = 4.0    # the body's top (the roof sits on it; the tag's height is set per robot, roof included)


def look_rotation(C, target, roll_deg=0.0):
    """World -> camera rotation for a camera at C looking at target (z up in the world; the picture's down towards
    world -y when looking straight down, so x runs right in the picture as on a plan drawn with y up... flipped by
    the caller if wanted)."""
    z = np.asarray(target, float) - np.asarray(C, float)
    z /= np.linalg.norm(z)
    up = np.array([0.0, 1.0, 0.0]) if abs(z[2]) > 0.95 else np.array([0.0, 0.0, 1.0])
    x = np.cross(z, up) if abs(z[2]) <= 0.95 else np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    if roll_deg:
        c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1.0]]) @ R
    return R


class TagCamera:
    """pos: lens (x, y, z) cm in the table frame (z up from the table top); look_at: a table point; size: (W, H) px;
    hfov_deg: the horizontal field of view (sets f). Picture spoilers: blur_px (lens, 1 sigma), noise (grey levels,
    1 sigma), gain / offset (exposure: grey = gain * scene + offset, clipped), jpeg (quality, 0 = none), k1 (barrel)."""

    def __init__(self, pos, look_at, size=(1280, 720), hfov_deg=70.0, roll_deg=0.0, blur_px=0.8, noise=3.0,
                 gain=1.0, offset=0.0, jpeg=85, k1=0.0, seed=1, name="cam", table=None, table_grey=120,
                 floor_grey=60, stickers=None, sticker_cm=2.0, exposure_ms=16.0, fps=15.0):
        self.name = name
        self.W, self.H = int(size[0]), int(size[1])
        self.f = (self.W / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.K = np.array([[self.f, 0, self.W / 2.0 - 0.5], [0, self.f, self.H / 2.0 - 0.5], [0, 0, 1.0]])
        self.dist = np.array([k1, 0, 0, 0, 0], float)
        self.C = np.asarray(pos, float)
        self.Rw = look_rotation(self.C, look_at, roll_deg)
        self.rvec = cv2.Rodrigues(self.Rw)[0].ravel()
        self.tvec = -self.Rw @ self.C
        self.blur_px, self.noise, self.gain, self.offset, self.jpeg = blur_px, noise, gain, offset, jpeg
        self.exposure_ms, self.fps = exposure_ms, fps
        self.rng = np.random.default_rng(seed)
        self.table = table
        self.table_grey, self.floor_grey = table_grey, floor_grey
        self.stickers = stickers or {}
        self.sticker_cm = sticker_cm
        self._tex = {}
        self._bg = None

    # -- geometry
    def project(self, pts):
        p = np.asarray(pts, float).reshape(-1, 3)
        uv, _ = cv2.projectPoints(p.reshape(-1, 1, 3), self.rvec, self.tvec, self.K, self.dist)
        uv = uv.reshape(-1, 2)
        d = (p @ self.Rw.T + self.tvec)[:, 2]
        uv[d <= 1e-6] = np.nan
        return uv

    def true_model(self, name=None):
        """The camera's CameraModel (what a perfect setup measures)."""
        from cameras import CameraModel
        return CameraModel(self.K, self.dist, self.rvec, self.tvec, (self.W, self.H), name=name or self.name,
                           fit={"method": "simulator truth"})

    def px_per_cm(self, x, y, z=0.0):
        a = self.project([[x, y, z], [x + 1, y, z], [x, y + 1, z]])
        return 0.5 * (np.linalg.norm(a[1] - a[0]) + np.linalg.norm(a[2] - a[0]))

    # -- drawing
    def _poly(self, img, pts3, grey, ss=1, off=(0, 0)):
        uv = self.project(pts3)
        if not np.all(np.isfinite(uv)):
            return
        uv = uv - np.asarray(off, float)
        cv2.fillPoly(img, [np.round(uv * ss * 16).astype(np.int32)], float(grey), lineType=cv2.LINE_AA, shift=4)

    def background(self):
        """The empty table: floor, table top, its stickers (a dark dot with a light ring). Kept (drawn once)."""
        if self._bg is not None:
            return self._bg
        img = np.full((self.H, self.W), float(self.floor_grey), np.float32)
        if self.table is not None:
            x0, y0, x1, y1 = self.table
            n = 30
            edge = [(x0 + (x1 - x0) * k / n, y0) for k in range(n)] + [(x1, y0 + (y1 - y0) * k / n) for k in range(n)]
            edge += [(x1 - (x1 - x0) * k / n, y1) for k in range(n)] + [(x0, y1 - (y1 - y0) * k / n) for k in range(n)]
            self._poly(img, [(x, y, 0.0) for x, y in edge], self.table_grey)
            grain = cv2.GaussianBlur(self.rng.normal(0, 1, (self.H, self.W)).astype(np.float32), (0, 0), 1.5) * 3
            img += grain
        for sid, (sx, sy) in self.stickers.items():
            r = self.sticker_cm / 2.0
            a = np.linspace(0, 2 * math.pi, 20, endpoint=False)
            self._poly(img, [(sx + 1.6 * r * math.cos(t), sy + 1.6 * r * math.sin(t), 0.0) for t in a], 235.0)
            self._poly(img, [(sx + r * math.cos(t), sy + r * math.sin(t), 0.0) for t in a], 25.0)
        self._bg = img
        return img

    def texture(self, family, tag_id, tag_cm, roof_cm):
        """The roof's top as a picture: the tag (black square tag_cm wide) in the middle of a white roof_cm square."""
        key = (family, tag_id, tag_cm, roof_cm)
        if key in self._tex:
            return self._tex[key]
        n = TC.FAMILIES[family][1]
        ppc = 24
        t = TC.tag_image(family, tag_id, px_per_cell=ppc, quiet_cells=0)
        side_px = t.shape[0]
        px_per_cm = side_px / tag_cm
        roof_px = int(round(roof_cm * px_per_cm))
        m = max(0, (roof_px - side_px) // 2)
        tex = cv2.copyMakeBorder(t, m, roof_px - side_px - m, m, roof_px - side_px - m, cv2.BORDER_CONSTANT, value=242)
        self._tex[key] = (tex.astype(np.float32), px_per_cm)
        return self._tex[key]

    def _draw_roof(self, img, x, y, th, z, family, tag_id, tag_cm, roof_cm, grey_scale=1.0, off=(0, 0)):
        tex, ppcm = self.texture(family, tag_id, tag_cm, roof_cm)
        th_ = tex.shape[0]
        h = roof_cm / 2.0
        c, s = math.cos(th), math.sin(th)
        # texture corners (top-left, top-right, bottom-right, bottom-left of the printed tag) -> roof corners: the
        # printed "top" edge towards the robot's nose (+heading), the printed left on the robot's left
        local = [(h, h), (h, -h), (-h, -h), (-h, h)]        # (ahead, left) of the robot
        world = [(x + c * a - s * b, y + s * a + c * b, z) for a, b in local]
        uv = self.project(world)
        if not np.all(np.isfinite(uv)):
            return
        uv = uv - np.asarray(off, float)
        Hh, Ww = img.shape[:2]
        u0, v0 = np.floor(uv.min(0)).astype(int) - 3
        u1, v1 = np.ceil(uv.max(0)).astype(int) + 3
        u0, v0, u1, v1 = max(u0, 0), max(v0, 0), min(u1, Ww), min(v1, Hh)
        if u1 - u0 < 2 or v1 - v0 < 2:
            return
        ss = SUPERSAMPLE
        src = np.float32([[0, 0], [th_ - 1, 0], [th_ - 1, th_ - 1], [0, th_ - 1]])
        dst = np.float32((uv - [u0, v0] + 0.5) * ss - 0.5)
        Hm = cv2.getPerspectiveTransform(src, dst)
        ww, hh = (u1 - u0) * ss, (v1 - v0) * ss
        warped = cv2.warpPerspective(tex, Hm, (ww, hh), flags=cv2.INTER_LINEAR, borderValue=-1)
        mask = (warped >= 0).astype(np.float32)
        warped = np.where(warped >= 0, warped, 0) * grey_scale
        ws = cv2.resize(warped, (u1 - u0, v1 - v0), interpolation=cv2.INTER_AREA)
        ms = cv2.resize(mask, (u1 - u0, v1 - v0), interpolation=cv2.INTER_AREA)
        roi = img[v0:v1, u0:u1]
        img[v0:v1, u0:u1] = roi * (1 - ms) + ws * np.where(ms > 1e-6, 1.0, 0.0)

    def _draw_body(self, img, x, y, th, roof_z, off=(0, 0)):
        """The robot's body below the roof: its sides (seen by a tilted camera), dark grey."""
        h = ROBOT_BODY_CM / 2.0
        c, s = math.cos(th), math.sin(th)
        cs = [(x + c * a - s * b, y + s * a + c * b) for a, b in ((h, h), (-h, h), (-h, -h), (h, -h))]
        for k in range(4):
            p, q = cs[k], cs[(k + 1) % 4]
            self._poly(img, [(p[0], p[1], 0.0), (q[0], q[1], 0.0), (q[0], q[1], roof_z), (p[0], p[1], roof_z)], 70.0,
                       off=off)

    def render(self, robots=(), t=0.0, covered=()):
        """Grey uint8 picture. robots: dicts {x, y, th (rad), z (tag height cm), family, id, tag_cm, roof_cm,
        vx, vy (cm/s, for motion blur)}. covered: robot ids whose tag is hidden (a hand, a cable: drawn plain white)."""
        img = self.background().copy()
        order = sorted(robots, key=lambda r: -np.linalg.norm(self.C[:2] - [r["x"], r["y"]]))
        for r in order:
            self._draw_body(img, r["x"], r["y"], r["th"], r["z"])
            if r["id"] in covered:
                h = r["roof_cm"] / 2.0
                c, s = math.cos(r["th"]), math.sin(r["th"])
                self._poly(img, [(r["x"] + c * a - s * b, r["y"] + s * a + c * b, r["z"])
                                 for a, b in ((h, h), (h, -h), (-h, -h), (-h, h))], 235.0)
                continue
            before = img.copy() if (r.get("vx") or r.get("vy")) else None
            self._draw_roof(img, r["x"], r["y"], r["th"], r["z"], r["family"], r["id"], r["tag_cm"], r["roof_cm"])
            if before is not None:
                # motion blur: the robot moves vx, vy during the exposure: smear its region along its picture motion
                dt = self.exposure_ms / 1000.0
                a = self.project([[r["x"], r["y"], r["z"]], [r["x"] + r["vx"] * dt, r["y"] + r["vy"] * dt, r["z"]]])
                d = a[1] - a[0]
                L = float(np.hypot(*d))
                if L > 0.3:
                    k = int(math.ceil(L)) * 2 + 1
                    ker = np.zeros((k, k), np.float32)
                    cx = k // 2
                    for q in np.linspace(-0.5, 0.5, 4 * k):
                        ker[int(round(cx + q * d[1])), int(round(cx + q * d[0]))] += 1
                    ker /= ker.sum()
                    img = cv2.filter2D(img, -1, ker) * (img != before) + img * (img == before)
        if self.blur_px > 0:
            img = cv2.GaussianBlur(img, (0, 0), self.blur_px)
        img = img * self.gain + self.offset
        if self.noise > 0:
            img = img + self.rng.normal(0, self.noise, img.shape).astype(np.float32)
        out = np.clip(img, 0, 255).astype(np.uint8)
        if self.jpeg:
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg)])
            out = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        return out


    # -- fast pictures for the live simulator (two cameras at 10-15 a second on a small machine): the empty table is
    #    blurred, exposed and given noise once (NOISE_VARIANTS ways); each picture redraws only the robots' squares
    def _fast_setup(self):
        if getattr(self, "_fast", None) is not None:
            return
        bg = self.background()
        b = cv2.GaussianBlur(bg, (0, 0), self.blur_px) if self.blur_px > 0 else bg.copy()
        b = b * self.gain + self.offset
        self._bg_exposed = b.astype(np.float32)
        self._noise = [self.rng.normal(0, self.noise, b.shape).astype(np.float32) for _ in range(3)]
        self._fast = [np.clip(b + n, 0, 255).astype(np.uint8) for n in self._noise]

    def render_fast(self, robots=(), covered=()):
        self._fast_setup()
        k = int(self.rng.integers(0, len(self._fast)))
        out = self._fast[k].copy()
        noise = self._noise[k]
        bg = self.background()
        for r in robots:
            h = max(ROBOT_BODY_CM, r["roof_cm"]) / 2.0 + 1.0
            pts = [(r["x"] + a, r["y"] + b, z) for a in (-h, h) for b in (-h, h) for z in (0.0, r["z"])]
            uv = self.project(pts)
            if not np.all(np.isfinite(uv)):
                continue
            pad = int(math.ceil(3 * self.blur_px)) + 3
            u0, v0 = np.floor(uv.min(0)).astype(int) - pad
            u1, v1 = np.ceil(uv.max(0)).astype(int) + pad
            u0, v0, u1, v1 = max(u0, 0), max(v0, 0), min(u1, self.W), min(v1, self.H)
            if u1 - u0 < 4 or v1 - v0 < 4:
                continue
            roi = bg[v0:v1, u0:u1].copy()
            off = (u0, v0)
            self._draw_body(roi, r["x"], r["y"], r["th"], r["z"], off=off)
            if r["id"] in covered:
                hh = r["roof_cm"] / 2.0
                c, s_ = math.cos(r["th"]), math.sin(r["th"])
                self._poly(roi, [(r["x"] + c * a - s_ * b, r["y"] + s_ * a + c * b, r["z"])
                                 for a, b in ((hh, hh), (hh, -hh), (-hh, -hh), (-hh, hh))], 235.0, off=off)
            else:
                self._draw_roof(roi, r["x"], r["y"], r["th"], r["z"], r["family"], r["id"], r["tag_cm"],
                                r["roof_cm"], off=off)
            if self.blur_px > 0:
                roi = cv2.GaussianBlur(roi, (0, 0), self.blur_px)
            roi = roi * self.gain + self.offset + noise[v0:v1, u0:u1]
            # only the inner part (the blur's edge would show the ROI's border)
            m = pad // 2
            out[v0 + m:v1 - m, u0 + m:u1 - m] = np.clip(roi[m:roi.shape[0] - m, m:roi.shape[1] - m], 0, 255).astype(np.uint8)
        if self.jpeg:
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg)])
            return buf.tobytes(), out
        return None, out


def overhead_pair(table_cm=(396.0, 168.0), height_cm=190.0, size=(1280, 720), hfov_deg=70.0, overlap_cm=40.0,
                  **kw):
    """Two cameras looking straight down, each over one half of a long table, overlapping in the middle (the build's
    recommended school setup). Returns [TagCamera, TagCamera] ('cam1' over x < L/2, 'cam2' over x > L/2)."""
    L, W = table_cm
    xs = (L / 4.0 - 0.0, 3 * L / 4.0)
    out = []
    for k, x in enumerate(xs):
        out.append(TagCamera((x, W / 2.0, height_cm), (x + 0.01, W / 2.0, 0.0), size=size, hfov_deg=hfov_deg,
                             name="cam%d" % (k + 1), table=(0, 0, L, W), seed=11 + k, **kw))
    return out


# ------------------------------------------------------------------------------------------- the live simulator
def cams_from_config(cfg):
    """TagCameras from a tag-cams config ({cams: [{name, pos, look_at, size, hfov_deg, blur_px, noise, gain, offset,
    k1, roll_deg}], table: [L, W], stickers: {label: [x, y]}})."""
    L, W = cfg["table"]
    out = []
    for k, c in enumerate(cfg["cams"]):
        kw = {key: c[key] for key in ("size", "hfov_deg", "blur_px", "noise", "gain", "offset", "k1", "roll_deg", "jpeg")
              if key in c}
        out.append(TagCamera(c["pos"], c["look_at"], name=c["name"], table=(0, 0, L, W), seed=31 + k,
                             stickers={str(a): tuple(b) for a, b in (cfg.get("stickers") or {}).items()}, **kw))
    return out


def default_config(table=(396.24, 167.64), size=(1280, 960), height_cm=200.0, hfov_deg=74.0, seed=5, stickers=12,
                   tags=None, overlap_cm=40.0):
    """The school setup as the build recommends it: two cameras straight down over each half (a little tilted and
    turned, as a hand-mounted camera is), numbered stickers scattered on the table."""
    rng = np.random.default_rng(seed)
    L, W = table
    half = L / 2 + overlap_cm / 2
    cams = []
    for k, x in enumerate((half / 2, L - half / 2)):
        cams.append(dict(name=("overhead", "side")[k], pos=[x + rng.normal(0, 4), W / 2 + rng.normal(0, 4), height_cm],
                         look_at=[x + rng.normal(0, 10), W / 2 + rng.normal(0, 6), 0.0], size=list(size),
                         hfov_deg=hfov_deg, blur_px=0.9, noise=3.5, k1=float(rng.normal(0, 0.03)),
                         roll_deg=float(rng.normal(0, 3)), port=8765 + k))
    st = {}
    while len(st) < stickers:
        x, y = rng.uniform(20, L - 20), rng.uniform(20, W - 20)
        if all(math.hypot(x - a, y - b) > 25 for a, b in st.values()) and math.hypot(x - L / 2, y - W / 2) > 30:
            st[str(len(st) + 1)] = [round(float(x), 1), round(float(y), 1)]
    return {"table": [L, W], "cams": cams, "stickers": st, "fps": 12.0,
            "tags": tags or {"family": TC.TAG_FAMILY, "tag_cm": 8.0, "roof_cm": 9.5, "height_cm": 6.0,
                             "ids": {"JayA": 0, "JayB": 1}}}


def serve_live(sim, cfg, out_dir):
    """fleet_sim.py --tag-cams: each camera's pictures at its port's /live (as the Pi's recorder serves them), drawn
    from the TRUE poses; the true camera models written to <out_dir>/true_<name>.json (what a perfect setup gives,
    for the tests; the run itself uses the site fit's). cfg["gaps"]: [{robot, t, dur}]: that robot's tag covered (a
    hand, a cable) from t s after the plan starts for dur s. cfg["low_res"]: {camera: [W, H]}: that camera sends
    smaller pictures (its model is scaled to match)."""
    import json as _json
    from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
    import threading
    import time as _time
    cams = cams_from_config(cfg)
    tags = cfg["tags"]
    ids = {n: int(i) for n, i in tags["ids"].items()}
    os.makedirs(out_dir, exist_ok=True)
    for c in cams:
        c.true_model().save(os.path.join(out_dir, "true_%s.json" % c.name))
    latest = {c.name: [None] for c in cams}
    cond = threading.Condition()

    def handler(name):
        class H(BaseHTTPRequestHandler):
            def log_message(self, *x):
                pass

            def do_GET(self):
                if self.path.split("?")[0] != "/live":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = 0
                try:
                    while not sim.stop.is_set():
                        with cond:
                            while latest[name][0] is None or latest[name][0][1] <= last:
                                cond.wait(0.5)
                                if sim.stop.is_set():
                                    return
                            body, n, tw = latest[name][0]
                        last = n
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(body)).encode()
                                         + b"\r\nX-Frame: " + str(n).encode() + b"\r\nX-Time: " + ("%.4f" % tw).encode()
                                         + b"\r\n\r\n" + body + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
        return H
    servers = []
    for k, c in enumerate(cams):
        port = int(cfg["cams"][k].get("port", 8765 + k))
        srv = ThreadingHTTPServer(("127.0.0.1", port), handler(c.name))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        servers.append(srv)
        sim.log("serving the simulated tag camera %s (%dx%d) at http://127.0.0.1:%d/live" % (c.name, c.W, c.H, port))
    period = 1.0 / float(cfg.get("fps", 12.0))
    n = 0
    prev = {}
    low = cfg.get("low_res") or {}
    while not sim.stop.is_set():
        t_cap = _time.time()
        rel = (t_cap - sim.t_plan0) if sim.t_plan0 is not None else -1e9
        covered = set()
        for g in cfg.get("gaps") or []:
            if g["t"] <= rel < g["t"] + g["dur"] and g["robot"] in ids:
                covered.add(ids[g["robot"]])
        rs = []
        for name, rb in sim.robots.items():
            if name not in ids or rb.lifted:
                continue
            p = prev.get(name)
            vx = vy = 0.0
            if p is not None and t_cap > p[2]:
                vx, vy = (rb.x - p[0]) / (t_cap - p[2]), (rb.y - p[1]) / (t_cap - p[2])
            prev[name] = (rb.x, rb.y, t_cap)
            rs.append(dict(x=rb.x, y=rb.y, th=rb.th, z=float(tags["height_cm"]), family=tags["family"], id=ids[name],
                           tag_cm=float(tags["tag_cm"]), roof_cm=float(tags["roof_cm"]), vx=vx, vy=vy))
        n += 1
        for c in cams:
            body, img = c.render_fast(rs, covered=covered)
            if c.name in low:
                w, h = low[c.name]
                small = cv2.resize(img, (int(w), int(h)), interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 85])
                body = buf.tobytes()
            if n == 1:
                cv2.imwrite(os.path.join(out_dir, "live_first_%s.png" % c.name), img)
            with cond:
                latest[c.name][0] = (body, n, t_cap + 0.04)
                cond.notify_all()
        _time.sleep(max(0.0, period - (_time.time() - t_cap)))
    for srv in servers:
        srv.shutdown()


# ------------------------------------------------------------------------------------------- the practice room
class _Still:
    def __init__(self, name, x, y, th):
        self.name, self.x, self.y, self.th, self.lifted = name, x, y, th, False


class Room:
    """Mission Control's practice room (tag_job.py): the cameras of a tag config, the robots standing still at given
    poses, nothing else (no simulated firmware, so nothing answers on the robots' WiFi ports)."""

    def __init__(self, robots):
        import threading
        self.robots = {n: _Still(n, *p) for n, p in robots.items()}
        self.stop = threading.Event()
        self.t_plan0 = None

    def log(self, msg):
        print(msg, flush=True)


def room_main(argv=None):
    import argparse
    import json as _json
    import signal
    ap = argparse.ArgumentParser(description="the practice room: tag cameras over a table, robots standing still")
    ap.add_argument("--config", required=True, help="a tag-cams config with 'room_robots': {name: [x, y, heading deg]}")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    with open(a.config) as f:
        cfg = _json.load(f)
    room = Room({n: (p[0], p[1], math.radians(p[2])) for n, p in cfg["room_robots"].items()})
    signal.signal(signal.SIGTERM, lambda *_: room.stop.set())
    try:
        serve_live(room, cfg, a.out)
    except KeyboardInterrupt:
        room.stop.set()
    return 0


if __name__ == "__main__":
    sys.exit(room_main())
