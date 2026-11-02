#!/usr/bin/env python3
"""Mission Control's AprilTag screens (conference-table build, 2026-11-02): the server side of "Cameras & table" and
"Production", and the sync card. Used by mission_control.py when the site's tracking is "tags".

  room        the cameras' pictures: the Pi's recorder (robots), or a simulated room (two tag cameras over the site's
              table, the robots standing at their homes: tools/sim/tag_cam.py through fleet_sim.py) for practice
  check-out   camera_checkout.py on both cameras
  clicks      corners (1-4) and stickers (their numbers) clicked in each camera's picture; typed measurements
  measure     tag_setup.py: the robots' tags read, site_fit.full, camera models saved to <conf_dir>/tagcal/, and the
              workspace map remade: the measured table, the production points as spots, the homes
  refit       site_fit.refit with the last run's tag reads (saved beside the run; "Use it" copies it in)
  production  production.program from the orders; planned like any program (missions.build, the 8 cm net)
  sync        the session id, the Pi's clock status, Start / End marks, the pose timeline, the comparison
"""
import copy
import json
import os
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np    # noqa: E402

REAL_STREAMS = {"overhead": "127.0.0.1:8000", "side": "127.0.0.1:8001"}
SIM_PORTS = {"overhead": 8785, "side": 8786}       # the practice room's cameras (a run's own simulator uses
                                                   # 8765/8766, the corner setup's dry runs 8775/8776)


def tags_of(site):
    t = dict((site or {}).get("tags") or {})
    t.setdefault("family", "tag25h9")
    t.setdefault("tag_cm", 8.0)
    t.setdefault("roof_cm", 9.5)
    if t.get("height_cm") is None:
        t["height_cm"] = 6.0
        t["height_placeholder"] = True
    return t


class TagJob:
    def __init__(self, session, site):
        self.s = session
        self.site = site
        self.tags = tags_of(site)
        self.cal_dir = os.path.join(site["conf_dir"], "tagcal")
        self.state_path = os.path.join(session.dir, "tag_setup.json")
        self.d = {"clicks": {"overhead": {"corners": {}, "stickers": {}}, "side": {"corners": {}, "stickers": {}}},
                  "typed": {"table": dict(site.get("table") or {}), "stickers": {}, "distances": []},
                  "result": None, "checkout": None, "pictures": {}, "session_id": None, "refit": None}
        try:
            with open(self.state_path) as f:
                self.d.update(json.load(f))
        except (OSError, ValueError):
            pass
        self.pics = {}            # camera -> (t, grey)
        self.room = None          # the practice simulator (subprocess)
        self.room_cfg = None
        self.busy = None
        self.lock = threading.Lock()

    def save(self):
        with open(self.state_path, "w") as f:
            json.dump({k: v for k, v in self.d.items() if k != "pictures"}, f, indent=1, default=str)

    # -- the cameras ------------------------------------------------------------------------------------------
    def streams(self, conn):
        if conn == "robots":
            return dict(REAL_STREAMS)
        self.start_room()
        return {c: "127.0.0.1:%d" % p for c, p in SIM_PORTS.items()}

    def start_room(self):
        """The practice room: the simulator's two tag cameras over this site's table, robots at their homes."""
        if self.room is not None and self.room.poll() is None:
            return
        import tag_cam as TCAM
        import production as PR
        tb = self.d["typed"].get("table") or {}
        L, W = float(tb.get("length_cm") or 396.24), float(tb.get("width_cm") or 167.64)
        names = [r["name"] for r in self.site["robots"]]
        cfg = TCAM.default_config(table=(L, W), size=(self.site["capture"]["width"], self.site["capture"]["height"]),
                                  tags=self.tags)
        for k, c in enumerate(cfg["cams"]):
            c["port"] = list(SIM_PORTS.values())[k]
        d = os.path.join(self.s.dir, "practice-room")
        os.makedirs(d, exist_ok=True)
        cp = os.path.join(d, "tag_cams.json")
        with open(cp, "w") as f:
            json.dump(cfg, f, indent=1)
        m, _ = PR.site_map((L, W), names)
        cfg["room_robots"] = {r["name"]: r["start"] for r in m["robots"]}
        with open(cp, "w") as f:
            json.dump(cfg, f, indent=1)
        log = open(os.path.join(d, "room.txt"), "w")
        self.room = subprocess.Popen([sys.executable, os.path.join(TOOLS, "sim", "tag_cam.py"), "--config", cp,
                                      "--out", os.path.join(d, "sim")],
                                     cwd=REPO, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.room_cfg = cfg
        self.room_dir = d
        time.sleep(2.5)

    def stop_room(self):
        if self.room is not None and self.room.poll() is None:
            self.room.terminate()
            try:
                self.room.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.room.kill()
        self.room = None

    def take_pictures(self, conn):
        import tag_setup as TSU
        pics = TSU.grab_pictures(self.streams(conn), 1.5, max_n=20)
        self.pics = {c: p[-1] for c, p in pics.items() if p}
        self.pics_all = pics
        self.d["pictures"] = {c: {"size": [int(g.shape[1]), int(g.shape[0])], "t": t} for c, (t, g) in self.pics.items()}
        if not self.pics:
            raise ValueError("no pictures from the cameras (%s)" % ("is the recorder running? ./run-fleet-pi.sh rec"
                                                                     if conn == "robots" else "the practice room"))
        return self.d["pictures"]

    def picture_jpeg(self, cam, marks=True):
        """The newest picture with what is clicked (corners red, stickers blue) and the tags read (green)."""
        import cv2
        if cam not in self.pics:
            return None
        t, g = self.pics[cam]
        img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if marks:
            import tagcore as TC
            for d in TC.TagDetector(self.tags["family"], ids=list(int(i) for i in self.tags["ids"].values())).detect_full(g):
                cv2.polylines(img, [np.round(d.corners).astype(np.int32)], True, (60, 160, 18), 2)
                cv2.putText(img, "tag %d" % d.id, tuple(int(v) for v in d.corners[0] + [0, -6]), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (60, 160, 18), 2)
            cl = self.d["clicks"].get(cam) or {}
            for k, (u, v) in (cl.get("corners") or {}).items():
                cv2.circle(img, (int(u), int(v)), 9, (24, 35, 180), 2)
                cv2.putText(img, str(k), (int(u) + 10, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (24, 35, 180), 2)
            for k, (u, v) in (cl.get("stickers") or {}).items():
                cv2.circle(img, (int(u), int(v)), 8, (204, 82, 41), 2)
                cv2.putText(img, str(k), (int(u) + 9, int(v) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (204, 82, 41), 2)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes() if ok else None

    # -- clicks and typed -------------------------------------------------------------------------------------
    def set_clicks(self, d):
        cam = d["cam"]
        cl = self.d["clicks"].setdefault(cam, {"corners": {}, "stickers": {}})
        kind = d.get("kind", "corner")
        key = str(d.get("label", "")).strip().upper()
        if not key:
            raise ValueError("give the corner's number (1-4) or the sticker's number")
        if kind == "corner" and key not in ("1", "2", "3", "4"):
            raise ValueError("a corner is 1, 2, 3 or 4 (1 = 0,0; 1 -> 2 along x; 1 -> 4 along y, counter-clockwise "
                             "seen from above)")
        bucket = cl["corners" if kind == "corner" else "stickers"]
        if d.get("remove"):
            bucket.pop(key, None)
        else:
            bucket[key] = [float(d["u"]), float(d["v"])]
        self.save()
        return self.d["clicks"]

    def set_typed(self, d):
        t = self.d["typed"]
        if "table" in d:
            tb = d["table"] or {}
            t["table"] = {k: float(v) for k, v in tb.items() if k in ("length_cm", "width_cm") and v not in ("", None)}
        if "stickers" in d:
            t["stickers"] = {str(k).upper(): [float(a), float(b)] for k, (a, b) in (d["stickers"] or {}).items()}
        if "distances" in d:
            t["distances"] = [[str(a).upper(), str(b).upper(), float(c)] for a, b, c in (d["distances"] or [])]
        self.save()
        return t

    # -- check-out --------------------------------------------------------------------------------------------
    def checkout(self, conn):
        import camera_checkout as CO
        import e02_camera as CAM
        models, area = {}, None
        if os.path.exists(os.path.join(self.cal_dir, "site_fit.json")) and conn == "robots":
            for c in ("overhead", "side"):
                p = os.path.join(self.cal_dir, c + ".json")
                if os.path.exists(p):
                    models[c] = CAM.load_camera(p)
            with open(os.path.join(self.cal_dir, "site_fit.json")) as f:
                L, W = json.load(f)["size_cm"]
            area = (0, 0, L, W)
        elif conn != "robots" and self.d.get("result"):
            area = (0, 0) + tuple(self.d["result"]["size_cm"])
            for c in ("overhead", "side"):
                p = os.path.join(self.sim_cal_dir(), c + ".json")
                if os.path.exists(p):
                    models[c] = CAM.load_camera(p)
        res = CO.run(self.streams(conn), tags=self.tags, models=models or None, area=area,
                     want_size=[self.site["capture"]["width"], self.site["capture"]["height"]],
                     labels={k: v.get("label", k) for k, v in self.site["cameras"].items()},
                     clicks={c: (v.get("corners") or {}) for c, v in self.d["clicks"].items()})
        res["words"] = CO.words(res)
        self.d["checkout"] = res
        self.save()
        return res

    def sim_cal_dir(self):
        return os.path.join(self.s.dir, "practice-room", "tagcal")

    # -- measure ----------------------------------------------------------------------------------------------
    def measure(self, conn):
        import tag_setup as TSU
        import production as PR
        streams = self.streams(conn)
        pics = TSU.grab_pictures(streams, TSU.PICTURE_S)
        reads, ms = TSU.read_tags(pics, self.tags)
        if not reads:
            raise ValueError("no robot's tag was read: put the robots on the table where both cameras see them (their "
                             "homes near the middle), check the camera check-out, then Measure again")
        sizes = {c: [int(p[-1][1].shape[1]), int(p[-1][1].shape[0])] for c, p in pics.items() if p}
        corners = {c: dict((self.d["clicks"].get(c) or {}).get("corners") or {}) for c in sizes}
        st = {}
        for c in sizes:
            for k, uv in ((self.d["clicks"].get(c) or {}).get("stickers") or {}).items():
                st.setdefault(k, {"cams": {}})["cams"][c] = uv
        for k, xy in (self.d["typed"].get("stickers") or {}).items():
            if k in st:
                st[k]["xy_cm"] = xy
        tb = self.d["typed"].get("table") or {}
        obs = TSU.make_obs(sizes, corners, st, self.tags, TSU.groups_from_reads(reads, still=True),
                           table=tb if tb.get("length_cm") else None, distances=self.d["typed"].get("distances"))
        out_dir = self.cal_dir if conn == "robots" else self.sim_cal_dir()
        if conn == "robots" and os.path.isdir(out_dir):
            bk = out_dir + "-before-" + time.strftime("%Y%m%d-%H%M%S")
            shutil.copytree(out_dir, bk)
        res = TSU.fit_and_save(obs, out_dir)
        words = TSU.result_words(res)
        if self.tags.get("height_placeholder"):
            words.insert(0, "The roof's height is NOT measured yet: 6.0 cm used. Measure from the table to the tag's "
                            "face and put it in the site profile (tags.height_cm), then Measure again.")
        L, W = res["size_cm"]
        names = [r["name"] for r in self.site["robots"]]
        m, moved = PR.site_map((L, W), names, points=res["stickers"])
        words += moved
        self.s.map = m
        self.s.save_map()
        self.d["result"] = {"size_cm": res["size_cm"], "words": words, "stickers": res["stickers"],
                            "rms_px": res["rms_px"], "checks": res.get("checks"), "tag_scale": res.get("tag_scale"),
                            "t": time.time(), "conn": conn, "saved_to": out_dir, "reads": len(reads)}
        if conn != "robots" and self.room_cfg:
            truth = {}
            from cameras import CameraModel
            for c in ("overhead", "side"):
                p = os.path.join(self.room_dir, "sim", "true_%s.json" % c)
                if os.path.exists(p):
                    truth[c] = CameraModel.load(p)
            import site_fit as SF
            self.d["result"]["sim_truth"] = SF.compare(res, truth, {k: tuple(v) for k, v in self.room_cfg["stickers"].items()},
                                                       H=float(self.tags["height_cm"]), table=(L, W))
        self.save()
        return self.d["result"]

    def refit(self, run_folder, use=False):
        import site_fit as SF
        ob = os.path.join(self.cal_dir if not run_folder or "-sim" not in run_folder else self.sim_cal_dir(), "obs.json")
        to = os.path.join(run_folder, "bridge", "tag_obs.csv")
        for cand in (os.path.join(run_folder, "tagcal", "obs.json"), ob):
            if os.path.exists(cand):
                ob = cand
                break
        if not (os.path.exists(ob) and os.path.exists(to)):
            raise ValueError("no setup or no tag reads to refit with (%s, %s)" % (ob, to))
        out = os.path.join(run_folder, "tagcal_refit")
        res = SF.refit(ob, to, out)
        self.d["refit"] = {"folder": out, "size_cm": res["size_cm"], "words": res["words"], "rms_px": res["rms_px"]}
        if use:
            bk = self.cal_dir + "-before-" + time.strftime("%Y%m%d-%H%M%S")
            if os.path.isdir(self.cal_dir):
                shutil.copytree(self.cal_dir, bk)
            os.makedirs(self.cal_dir, exist_ok=True)
            for f in os.listdir(out):
                shutil.copy(os.path.join(out, f), os.path.join(self.cal_dir, f))
            self.d["refit"]["used"] = True
        self.save()
        return self.d["refit"]

    # -- production -------------------------------------------------------------------------------------------
    def production_program(self, d):
        import production as PR
        orders = {n: [str(x).strip().upper() for x in v if str(x).strip()] for n, v in (d.get("orders") or {}).items()}
        orders = {n: v for n, v in orders.items() if v or d.get("home", True)}
        bad = PR.check_orders(orders, self.s.map.get("spots") or {})
        if bad:
            raise ValueError("; ".join(bad))
        prog = PR.program(orders, dwell_s=float(d.get("dwell_s", PR.DWELL_S)), home=bool(d.get("home", True)))
        if d.get("sync_spin"):
            prog = add_sync_spin(prog)
        return prog

    # -- sync -------------------------------------------------------------------------------------------------
    def sync_state(self):
        import sync_tools as ST
        if not self.d.get("session_id"):
            self.d["session_id"] = ST.session_id(self.site["name"])
            self.save()
        c = getattr(self, "_clock", None)
        if c is None or time.time() - c[0] > 30:
            self._clock = (time.time(), ST.clock_status())
        return {"session_id": self.d["session_id"], "clock": self._clock[1],
                "marks": ST.read_marks(self.mark_folder())[-10:]}

    def new_session_id(self):
        import sync_tools as ST
        self.d["session_id"] = ST.session_id(self.site["name"])
        self.save()
        return self.sync_state()

    def mark_folder(self):
        run = self.s.run
        if run is not None and run.alive():
            return run.folder
        return os.path.join(self.s.dir, "sync")

    def mark(self, what):
        import sync_tools as ST
        m = ST.mark(self.mark_folder(), what, session=self.d.get("session_id"))
        return m


def add_sync_spin(prog):
    """Each robot spins once on the spot at the start (two half turns at its home): a moment both recordings see."""
    out = copy.deepcopy(prog)
    pre = []
    for n in out["robots"]:
        pre.append({"block": "sync-spin", "robot": n})
    out["blocks"] = pre + out["blocks"]
    out["sync_spin"] = True
    return out
