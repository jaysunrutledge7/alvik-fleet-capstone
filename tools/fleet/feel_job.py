#!/usr/bin/env python3
"""Mission Control's "Feel out the table" step (Nick, 2 Nov 2026 18:33; feel_table.py does the work).

On a tag site, pressing Run for a production run first asks: is there a table calibration for this table and this
camera setup? No (the first run on a new table, or the corners were clicked again): the feel-out runs first (its own
run: calibration, the crawls), its result is saved in the site's folder and applied to the workspace map (the outline
the robots felt), the production mission is planned again on it, and the production run starts by itself. Yes: the
saved outline is used and the feels are skipped. Nick can skip it once ("Skip it this time") or run it again ("Feel
out again"). A result that says the outline is too far off holds the production run: Nick reads it and decides.
Keep going (2026-11-03, the site's on_problem / the Run screen's tick-box): nothing here holds the production run: a
feel-out that did not finish, or an outline too far off, is said and the production run starts on the cameras' outline.
Camera trust (the site's feel_out_trust "camera", the school): a felt-vs-camera difference is reported per side and
logged; the outline is never moved.

The robots' calibration lives in the site's folder (<conf_dir>/feel_table/); the simulator's in Mission Control's
practice room (<mc_dir>/practice-room/feel_table/), so practice never overwrites the real table's.
"""
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import feel_table as FT      # noqa: E402
import fleetmap as FM        # noqa: E402


class FeelJob:
    def __init__(self, session):
        self.s = session
        self.phase = "idle"          # idle, feeling, analyzing, done, held, failed, skipped
        self.words = []
        self.sp = None
        self.folder = None
        self.chain = None            # the production run to start afterwards ({"conn": ...})
        self.skip_once = False
        self.force_next = False
        self.result = None
        self.surface = None
        self.conn = None
        self.t0 = None
        self.err = None
        self.lock = threading.Lock()
        self.marker = os.path.join(session.dir, "feel_applied.json")

    # -- where things are -----------------------------------------------------------------------------------------
    def conf_dir(self, conn):
        if conn == "robots":
            return self.s.site["conf_dir"]
        return os.path.join(self.s.dir, "practice-room")

    def setup_id(self, conn):
        T = self.s.tag
        if conn == "robots":
            return FT.setup_id(T.cal_dir)
        return FT.setup_id(T.sim_cal_dir()) or "simulator"

    def camera_map(self, conn):
        """The workspace map on the cameras' own outline (not one the feel-out moved): the Measure's points."""
        import production as PR
        m = self.s.map
        r = FT.rect(m)
        try:
            with open(self.marker) as f:
                mk = json.load(f)
            if [round(v, 1) for v in mk.get("outline_used") or []] == [round(v, 1) for v in r]:
                r = mk.get("outline_camera") or r
        except (OSError, ValueError):
            pass
        names = [x["name"] for x in self.s.site["robots"]]
        res = (self.s.tag.d.get("result") or {})
        pts = res.get("stickers") or {k: v[:2] for k, v in (m.get("spots") or {}).items()}
        m2, _ = PR.site_map(None, names, points=pts, outline=r)
        return m2

    def key(self, conn):
        # the table's name: the site's typed size when it has one (a refit or a moved side never renames it)
        tb = self.s.site.get("table") or {}
        if tb.get("length_cm") and tb.get("width_cm"):
            return FT.table_key(table=(tb["length_cm"], tb["width_cm"]))
        return FT.table_key(self.camera_map(conn))

    def check(self, conn):
        """(needed, stored calibration, why)."""
        if self.force_next:
            return True, FT.load_cal(self.conf_dir(conn), self.key(conn)), "Feel out again was pressed"
        return FT.needed(self.conf_dir(conn), self.key(conn), self.setup_id(conn))

    # -- the run ----------------------------------------------------------------------------------------------
    def start(self, conn, chain=None, force=False):
        """1. the surface check's run (each robot calibrated, then standing still at home); 2. if the tabletop is
        clearly apart from the edge sensors' thresholds, the crawls' run. Both through Mission Control's run slot, so
        Stop stops them and the Run screen shows them. chain: the production run to start after it."""
        with self.lock:
            if self.phase in ("surface", "feeling", "analyzing"):
                raise ValueError("the feel-out is already running")
            self.conn, self.chain, self.err, self.result = conn, chain, None, None
            self.force_next = False
            m = self.camera_map(conn)
            self.m_cam = m
            self.folder = os.path.join(self.s.dir, "runs", "%s-feel-%s" % (time.strftime("%Y%m%d-%H%M%S"), conn))
            self.surface_paths = FT.prepare_surface(m, os.path.join(self.folder, "plan"))
            self.mp, self.pp, self.sp, mk = FT.prepare(m, os.path.join(self.folder, "plan"))
            self.fm = FM.load(self.mp)
            self.planned_s = mk
            self.surface = None
            self.s.feel_paths = self.surface_paths
            self.s.start_cam_run("feel", conn, {"folder": os.path.join(self.folder, "surface"), "calibrate": True})
            self.phase, self.t0 = "surface", time.time()
            self.words = ["Feeling out the table, 1 of 2: each robot is calibrated, then stands still %.0f s while its "
                          "edge sensors read the tabletop (the surface check)." % FT.SURFACE_HOLD_S]
        threading.Thread(target=self._watch, daemon=True).start()
        return self.status()

    def _wait_run(self):
        run = self.s.run
        while run is not None and run.alive():
            time.sleep(0.5)
        return run is not None and run.status().get("exit") in (0,), (run.status().get("exit") if run else "?")

    def keep_going(self):
        try:
            return self.s.on_problem() == "keep_going"
        except Exception:      # noqa: BLE001
            return False

    def trust(self):
        return (self.s.site or {}).get("feel_out_trust") or FT.TRUST

    def _failed(self, what, code):
        if self.keep_going() and not getattr(self.s, "stop_asked", False):
            # keep going (2026-11-03): the production run starts on the cameras' outline (a Stop is still a stop:
            # Mission Control's Stop sets stop_asked)
            with self.lock:
                self.phase = "failed"
                self.err = "the %s ended without finishing (exit %s): KEEP GOING: nothing saved; the production run " \
                           "starts on the cameras' outline" % (what, code)
                self.words.append(self.err)
            self._chain(self.conn)
            return
        with self.lock:
            self.phase = "failed"
            self.err = "the %s ended without finishing (exit %s): nothing saved, the production run not started" % (
                what, code)
            self.words.append(self.err)

    def _chain(self, conn):
        if self.chain:
            self.s.replan_production()
            self.words.append("The production run starts now.")
            self.s.start_cam_run("mission", self.chain.get("conn", conn), self.chain)

    def _watch(self):
        conn = self.conn
        ok, code = self._wait_run()
        if not ok:
            return self._failed("surface check", code)
        try:
            with self.s.lock:
                key, sid, cdir = self.key(conn), self.setup_id(conn), self.conf_dir(conn)
                self.surface = FT.surface_check(os.path.join(self.folder, "surface", "run"))
                if not self.surface["ok"]:
                    cal = FT.surface_cal(cdir, key, sid, FT.rect(self.m_cam), self.surface, site=self.s.site["name"])
                    self.apply(cal)
                    self.phase = "no crawls"
                    self.words = list(self.surface["words"]) + ["Saved: %s" % FT.cal_path(cdir, key)]
                    self._chain(conn)
                    return
                self.words = list(self.surface["words"]) + [
                    "Feeling out the table, 2 of 2: each robot crawls slowly to %d spots on the edges (%s) until its "
                    "edge sensors feel the drop, then backs straight off." % (
                        len(self.sp), ", ".join("%s %d" % (n, len([x for x in self.sp if x["robot"] == n]))
                                                for n in sorted({x["robot"] for x in self.sp})))]
                self.s.feel_paths = (self.mp, self.pp)
                self.s.start_cam_run("feel", conn, {"folder": os.path.join(self.folder, "crawl")})
                self.phase = "feeling"
        except Exception as e:      # noqa: BLE001
            with self.lock:
                self.phase, self.err = "failed", "%s: %s" % (type(e).__name__, e)
                self.words.append("The surface check could not be read: %s" % self.err)
            if self.keep_going() and not getattr(self.s, "stop_asked", False):
                self.words.append("KEEP GOING: no crawls; the production run starts on the cameras' outline.")
                self._chain(conn)
            return
        ok, code = self._wait_run()
        if not ok:
            return self._failed("feel-out run", code)
        with self.lock:
            self.phase = "analyzing"
        try:
            with self.s.lock:            # the page's requests change the map too
                res, cal = FT.finish(self.fm, os.path.join(self.folder, "crawl", "run"), self.sp, cdir, key, sid,
                                     site=self.s.site["name"], surface=self.surface, trust=self.trust())
                self.result = res
                self.apply(cal)
                self.words = list(res["words"]) + ["Saved as this table's calibration: %s" % FT.cal_path(cdir, key)]
                self.phase = "held" if res.get("hold") and not self.keep_going() else "done"
                if res.get("hold") and self.keep_going():
                    self.words.append("KEEP GOING: the outline was not moved; the production run starts on the "
                                      "cameras' outline.")
                if self.phase == "done":
                    self._chain(conn)
        except Exception as e:      # noqa: BLE001
            with self.lock:
                self.phase = "failed"
                self.err = "%s: %s" % (type(e).__name__, e)
                self.words.append("The feel-out's result could not be used: %s" % self.err)
            if self.keep_going() and not getattr(self.s, "stop_asked", False):
                self.words.append("KEEP GOING: the production run starts on the cameras' outline.")
                self._chain(conn)

    def apply(self, cal):
        """The calibration's outline into the workspace map (homes and points follow it)."""
        if not cal or cal.get("hold"):
            return
        names = [x["name"] for x in self.s.site["robots"]]
        m = self.camera_map(self.conn)
        m2, _w = FT.apply_to_map(m, cal, names)
        self.s.map = m2
        self.s.save_map()
        with open(self.marker, "w") as f:
            json.dump({"outline_camera": cal.get("outline_camera"), "outline_used": cal.get("outline_used"),
                       "setup_id": cal.get("setup_id"), "made": cal.get("made")}, f, indent=1)

    def use_saved(self, conn):
        """Before a production run that skips the feels: the saved outline into the map (once)."""
        need, cal, why = self.check(conn)
        if cal and not need:
            m = self.s.map
            if [round(v, 1) for v in FT.rect(m)] != [round(v, 1) for v in cal.get("outline_used") or []]:
                self.conn = conn
                self.apply(cal)
                self.s.replan_production()
        return need, cal, why

    # -- what the card shows ----------------------------------------------------------------------------------
    def status(self, conn=None):
        conn = conn or self.conn or ("robots" if self.s.conn_default().get("mode") == "robots" else "sim-cameras")
        try:
            need, cal, why = self.check(conn)
        except Exception as e:      # noqa: BLE001  (no map or no site yet: the card says so)
            need, cal, why = None, None, "not ready (%s)" % e
        st = {"phase": self.phase, "conn": conn, "needed": need, "why": why, "skip_once": self.skip_once,
              "force_next": self.force_next, "words": list(self.words), "error": self.err,
              "elapsed_s": round(time.time() - self.t0, 1) if self.t0 and self.phase in ("surface", "feeling", "analyzing")
               else None, "surface": self.surface or ((cal or {}).get("surface") if cal else None),
              "planned_s": getattr(self, "planned_s", None)}
        if self.phase in ("feeling", "analyzing") and self.folder and self.sp:
            st["progress"] = FT.progress(os.path.join(self.folder, "crawl", "run"), self.sp)
            st["spots"] = self.sp
        src = self.result and FT.card(FT.load_cal(self.conf_dir(conn), self.key(conn)))
        if not src and cal:
            src = FT.card(cal)
        if src:
            st["saved"] = src
        return st
