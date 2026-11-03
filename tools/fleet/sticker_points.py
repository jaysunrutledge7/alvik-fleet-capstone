#!/usr/bin/env python3
"""Typed sticker points (Nick 2026-11-03 05:58): Jay-Sun's spreadsheet of the 40 table stickers -> the site's
production points, with no clicking each sticker in the cameras' pictures.

  read      Jay's workbook (sheet "Points": Point ID, x (in), y (in), Zone; openpyxl) or a CSV of id,x_in,y_in[,zone]
  file      the derived coordinates kept in the repo beside the site (tools/fleet/sites/<site>-points.csv: id, x_in,
            y_in, zone, x_cm, y_cm): sticker positions only, as Jay typed them
  overrides the site's points.overrides list, applied when the points are loaded (each with its reason and status), so
            the file stays what Jay typed and a correction can be taken back by deleting one entry
  frame     Jay's frame is the reference tags' frame exactly: (0, 0) = the table corner by antenna A3 (corner 1, tag
            20), x along the 156 in edge toward the door end (A4), y toward the whiteboard (A1)
  check     after a Measure that also saw stickers in the pictures (the old click way, or the fit's sticker spots), a
            sticker more than FLAG_CM from its typed spot is flagged (said, never moved)

    python3 tools/fleet/sticker_points.py import table_points.xlsx --site school-conference   # writes the CSV
    python3 tools/fleet/sticker_points.py show --site school-conference                       # the points in use
"""
import argparse
import csv
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

IN_CM = 2.54             # exact
FLAG_CM = 3.0            # chosen: a sticker the cameras put further than this from its typed spot is flagged. 3 cm is
                         # about a tag camera's fit error (0.5-1 cm) plus a sticker's own size (~2 cm). DOE note: none
                         # needed (a warning threshold, not a control); tighten once the first fit shows the spread
ID_RE = re.compile(r"^P\d+$", re.I)


def norm_id(v):
    s = str(v).strip().upper()
    if re.match(r"^\d+$", s):
        s = "P%02d" % int(s)
    return s


def read_xlsx(path, sheet="Points"):
    """[{id, x_in, y_in, zone}] from Jay's workbook: the header row is the one whose first cell is "Point ID"; the
    example row and blank rows are skipped."""
    import openpyxl
    wb = openpyxl.load_workbook(path, data_only=True, read_only=True)
    ws = wb[sheet] if sheet in wb.sheetnames else wb[wb.sheetnames[0]]
    rows, head = [], None
    for r in ws.iter_rows(values_only=True):
        if head is None:
            if r and str(r[0] or "").strip().lower() == "point id":
                head = [str(c or "").strip().lower() for c in r]
            continue
        if not r or r[0] is None or not ID_RE.match(str(r[0]).strip()):
            continue
        cx = next(i for i, h in enumerate(head) if h.startswith("x (in"))
        cy = next(i for i, h in enumerate(head) if h.startswith("y (in"))
        cz = next((i for i, h in enumerate(head) if h.startswith("zone")), None)
        if r[cx] is None or r[cy] is None:
            continue
        rows.append({"id": norm_id(r[0]), "x_in": float(r[cx]), "y_in": float(r[cy]),
                     "zone": int(r[cz]) if cz is not None and r[cz] not in (None, "") else None})
    if not rows:
        raise ValueError("no points found in %s (sheet %s): expected a header row starting 'Point ID'" % (path, sheet))
    return rows


def read_csv(path):
    """[{id, x_in, y_in, zone}] from a CSV with a header (id, x_in, y_in[, zone]; extra columns ignored) or none."""
    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)]
    out = []
    start = 1 if rows and not re.match(r"^-?[\d.]+$", rows[0][1].strip()) else 0
    for r in rows[start:]:
        out.append({"id": norm_id(r[0]), "x_in": float(r[1]), "y_in": float(r[2]),
                    "zone": int(float(r[3])) if len(r) > 3 and r[3].strip() else None})
    return out


def read_any(path):
    return read_xlsx(path) if path.lower().endswith((".xlsx", ".xlsm")) else read_csv(path)


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "x_in", "y_in", "zone", "x_cm", "y_cm"])
        for r in rows:
            w.writerow([r["id"], "%g" % r["x_in"], "%g" % r["y_in"], "" if r.get("zone") is None else r["zone"],
                        "%.2f" % (r["x_in"] * IN_CM), "%.2f" % (r["y_in"] * IN_CM)])
    return path


def apply_overrides(rows, overrides):
    """The site's corrections: [{id, x_in, y_in, ...}] replace the file's values. Returns (rows, words)."""
    rows = [dict(r) for r in rows]
    by = {r["id"]: r for r in rows}
    words = []
    for o in overrides or []:
        i = norm_id(o["id"])
        if i not in by:
            words.append("override %s: no such point in the file (ignored)" % i)
            continue
        r = by[i]
        old = (r["x_in"], r["y_in"])
        r["x_in"], r["y_in"] = float(o.get("x_in", r["x_in"])), float(o.get("y_in", r["y_in"]))
        r["override"] = {"was_in": list(old), "status": o.get("status", ""), "why": o.get("why", "")}
        words.append("%s: (%g, %g) in used instead of the sheet's (%g, %g) in (%s)" % (
            i, r["x_in"], r["y_in"], old[0], old[1], o.get("status") or "override"))
    return rows, words


def points_file(site):
    p = ((site or {}).get("points") or {}).get("file")
    if not p:
        return None
    return p if os.path.isabs(p) else os.path.join(HERE, "sites", p)


def typed_on(site):
    """True when the site's production points come from the typed file (the switch: points.use "typed"; "clicked" =
    the old way, each sticker clicked in the pictures)."""
    pt = (site or {}).get("points") or {}
    return pt.get("use", "clicked") == "typed" and bool(points_file(site))


def load_rows(site):
    """(rows with overrides applied, words) for a site with a points file."""
    pt = (site or {}).get("points") or {}
    rows = read_csv(points_file(site))
    return apply_overrides(rows, pt.get("overrides"))


def points_cm(site):
    """({id: [x_cm, y_cm]}, {id: zone}, words): the site's typed production points in cm (overrides applied)."""
    rows, words = load_rows(site)
    return ({r["id"]: [round(r["x_in"] * IN_CM, 2), round(r["y_in"] * IN_CM, 2)] for r in rows},
            {r["id"]: r.get("zone") for r in rows}, words)


def zone_of(x_cm, lines_in=(39.0, 78.0, 117.0)):
    x_in = x_cm / IN_CM
    return 1 + sum(1 for v in lines_in if x_in >= v)


def flag_seen(typed, seen, flag_cm=FLAG_CM):
    """Stickers the cameras put somewhere else: [words] for each seen sticker more than flag_cm from its typed spot.
    Never moves a point (the typed spot stays the production point)."""
    out = []
    for k, xy in (seen or {}).items():
        k2 = norm_id(k)
        if k2 in typed:
            d = math.hypot(xy[0] - typed[k2][0], xy[1] - typed[k2][1])
            if d > flag_cm:
                out.append("CHECK: sticker %s: the cameras see it at (%.1f, %.1f) cm, %.1f cm from its typed spot "
                           "(%.1f, %.1f): the typed spot is used; check the sticker or the sheet" % (
                               k2, xy[0], xy[1], d, typed[k2][0], typed[k2][1]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a1 = sub.add_parser("import", help="the spreadsheet (or a CSV) -> the site's points file")
    a1.add_argument("path")
    a1.add_argument("--site", default="school-conference")
    a1.add_argument("--out", default="", help="default: tools/fleet/sites/<site>-points.csv")
    a2 = sub.add_parser("show", help="the site's points in use (overrides applied)")
    a2.add_argument("--site", default="school-conference")
    a = ap.parse_args(argv)
    if a.cmd == "import":
        rows = read_any(a.path)
        out = a.out or os.path.join(HERE, "sites", "%s-points.csv" % a.site)
        write_csv(rows, out)
        print("%d points written to %s" % (len(rows), out))
        return 0
    import sites as SI
    site = SI.load(a.site)
    pts, zones, words = points_cm(site)
    for w in words:
        print(w)
    for k in sorted(pts):
        print("%s  zone %s  (%.1f, %.1f) cm" % (k, zones.get(k), pts[k][0], pts[k][1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
