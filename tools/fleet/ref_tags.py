#!/usr/bin/env python3
"""Reference AprilTags on the table (Nick, 2026-11-03, approved): six flat tags lying on the table top, whose places
are fixed by the table itself, so the cameras find the table's corners, its scale and the shared frame without clicks.

  corner tags   one at each table corner, on a 3D-printed tile with an L-shaped lip underneath that hooks the corner
                (or a paper square put into the corner). The tag's middle sits OFFSET_CM in from both edges: the part
                fixes it. Corner 1's tag defines (0, 0) and the axes: x along the edge from corner 1 to corner 2, y
                along the edge from corner 1 to corner 4 (counter-clockwise 1-2-3-4 seen from above, as the clicks).
  side tags     one on each long side where the two cameras overlap (the table's middle), on a tile with a straight lip
                hooking the edge: its middle OFFSET_CM in from that edge; its place ALONG the edge is fitted (the
                cameras measure it), or typed as a check. They tie the two cameras together.

Every tag is tag25h9 (the robots' family: one detector, the OpenCV 4.6 fix, the sizing tests) with ids 20-25, never a
robot's (Alpha 0, Bravo 1, spares 2-5): the robots' detector only reports the robots' ids, and this one only these.

Which way a tag is turned on its tile does not matter for its place (the tag sits in the middle of a square tile:
every quarter turn puts its middle at the same spot); the fit reads each tag's turn from the picture. Which corner
it is in DOES matter: the id says it (printed on each tile, embossed and on the paper sheet).

    python3 tools/fleet/ref_tags.py sizing      # px per cell at each tag's spot for the school cameras; picks the size
    python3 tools/fleet/ref_tags.py pdf         # docs/hardware/reference-tags-letter.pdf (the paper option)
    python3 tools/fleet/ref_tags.py printcheck  # cell size against a 0.4 mm nozzle; raised cells in the simulator
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

FAMILY = "tag25h9"
CELLS = 7                 # tag25h9: 5 x 5 data cells and a 1-cell black border (tagcore.FAMILIES)
TAG_CM = 12.0             # CHOSEN from the sizing (python3 tools/fleet/ref_tags.py sizing; the report's table): at the
                          # school cameras (1280 x 960, about 175 cm up, 74 deg) a 12 cm tag gets 7.7-9.3 px a cell at
                          # the six spots (8 cm: 5.1-6.2; 640 x 480: half that; the 100 % read line is about 3.5 px a cell
                          # soft and tilted, the conference report's sizing). Bigger tags
                          # give the fit 4 corners further apart (the scale and the corner's place to about a tenth of a
                          # pixel each). 12 cm keeps a tile 15.5 cm square, inside a typical 20 cm printer bed. DOE: 8 /
                          # 12 / 15 cm on the real table, against the fit's corner error and the tile's room on the table
QUIET_CM = 1.75           # white round the black square: a full cell (12 / 7 = 1.71 cm) rounded up (the AprilTag
                          # papers' 1 cell; the roof tags have half a cell because the roof is small)
OFFSET_CM = QUIET_CM + TAG_CM / 2.0      # 7.75: the tag's middle from each edge line its tile hooks (fixed by the part)
TILE_CM = 2 * OFFSET_CM                  # 15.5: the tile's white top, edge line to edge line
Z_CM = 0.18               # the tag's face above the table: 1.2 mm white base + 0.6 mm raised black cells (the paper
                          # option: 0.01 cm; the difference moves a corner by under 0.1 mm at 175 cm: nothing)
CORNER_IDS = {1: 20, 2: 21, 3: 22, 4: 23}
SIDE_IDS = {1: 24, 3: 25}                 # side edge 1 (corner 1 -> 2, y = 0) and edge 3 (corner 3 -> 4, y = W)
BUMP_PX = 3.0             # a reference tag this many pixels from where the setup saw it (median of the camera's tags)
                          # means the camera moved. Chosen: 3 px is about 0.6 cm on the table at 5 px/cm; a still
                          # tag's read spreads 0.1-0.3 px (sizing tests). DOE: the spread of a still camera's reference
                          # tags over an hour (vibration, heat) sets it; then 3 x that spread or 3 px, whichever is more
BUMP_CHECK_S = 5.0        # how often the run checks them. No DOE: 6 small windows every 5 s cost nothing
BUMP_CONFIRM = 2          # this many checks in a row before the warning (one hand over a tag is not a bump)


def default_config():
    """What a site profile's "ref_tags" holds (school-conference.json)."""
    return {"use": True, "family": FAMILY, "tag_cm": TAG_CM, "offset_cm": OFFSET_CM, "z_cm": Z_CM,
            "corners": {str(k): v for k, v in CORNER_IDS.items()},
            "sides": {str(k): v for k, v in SIDE_IDS.items()}}


def layout(cfg=None):
    """{id: {"kind": "corner", "corner": k} or {"kind": "side", "edge": e}} from a site's ref_tags config."""
    cfg = cfg or default_config()
    out = {}
    for k, i in (cfg.get("corners") or {}).items():
        out[str(int(i))] = {"kind": "corner", "corner": int(k)}
    for e, i in (cfg.get("sides") or {}).items():
        out[str(int(i))] = {"kind": "side", "edge": int(e)}
    return out


def ids(cfg=None):
    return sorted(int(i) for i in layout(cfg))


def base_rot_deg(item):
    """The tile's x axis in the world (degrees CCW from +x): along the edge from its corner to the next one."""
    k = item["corner"] if item["kind"] == "corner" else item["edge"]
    return 90.0 * (k - 1)


def centre(item, L, W, off, along=None):
    """The tag's middle (x, y) in the table frame. Sides: along = its place along the edge from the edge's start."""
    if item["kind"] == "corner":
        return {1: (off, off), 2: (L - off, off), 3: (L - off, W - off), 4: (off, W - off)}[item["corner"]]
    e = item["edge"]
    a = (L / 2.0 if e in (1, 3) else W / 2.0) if along is None else along
    return {1: (a, off), 2: (L - off, a), 3: (a, W - off), 4: (off, a)}[e]


def heading(item, quarter=0):
    """The printed top's direction in the world: the tile's +y (into the table) plus `quarter` quarter turns."""
    return (base_rot_deg(item) + 90.0 + 90.0 * quarter) % 360.0


def world_poses(cfg, L, W, along=None, quarters=None):
    """{id: (x, y, heading)} for a table L x W; along {id: cm} for the side tags (default: the middle)."""
    off = float((cfg or {}).get("offset_cm", OFFSET_CM))
    out = {}
    for i, it in layout(cfg).items():
        x, y = centre(it, L, W, off, (along or {}).get(i))
        out[i] = (x, y, heading(it, (quarters or {}).get(i, 0)))
    return out


KEEPOUT_MARGIN_CM = 1.0  # the planner keeps the robots this far off a tile too (a robot's line sensors would read
                         # its black cells as an edge; its 1.8 mm step could catch the caster). No DOE: geometry


def tile_polygon(item, x, y, off=OFFSET_CM, margin=KEEPOUT_MARGIN_CM):
    """The tile's footprint on the table (its white square, edge lines to 2 x off in), plus a margin, as 4 corners."""
    rot = math.radians(base_rot_deg(item))
    ux, uy = np.array([math.cos(rot), math.sin(rot)]), np.array([-math.sin(rot), math.cos(rot)])
    c = np.array([x, y])
    h = off + margin
    return [[round(float(v), 2) for v in c + a * ux + b * uy] for a, b in ((-h, -h), (h, -h), (h, h), (-h, h))]


def keepouts(poses, cfg=None):
    """Keep-out areas for the workspace map: one per reference tile, from its fitted pose {id: (x, y, heading)}."""
    lay = layout(cfg)
    off = float((cfg or {}).get("offset_cm", OFFSET_CM))
    out = []
    for i, p in sorted(poses.items(), key=lambda kv: int(kv[0])):
        it = lay.get(str(i))
        if it is None:
            continue
        out.append({"name": "reference tag %s" % i, "polygon": tile_polygon(it, float(p[0]), float(p[1]), off),
                    "source": "reference tile (ref_tags.py): no driving over it"})
    return out


# ------------------------------------------------------------------------------------------------- detection
def detector(cfg=None):
    import tagcore as TC
    cfg = cfg or default_config()
    return TC.TagDetector(cfg.get("family", FAMILY), ids=ids(cfg))


def read_refs(pictures, cfg=None):
    """{camera: {id: corners 4x2 (median over the pictures)}} from {camera: [(t, grey)]} (whole pictures)."""
    det = detector(cfg)
    out = {}
    for c, pics in pictures.items():
        got = {}
        for t, g in pics[:6]:                 # still tags: 6 pictures are plenty
            for d in det.detect_full(g, cam=c, t=t):
                got.setdefault(str(d.id), []).append(d.corners)
        out[c] = {i: np.median(np.array(v), axis=0).tolist() for i, v in got.items()}
    return out


# ------------------------------------------------------------------------------------------------- sizing
def sizing(tag_cm=TAG_CM, sizes=((1280, 960), (640, 480)), heights=(175.0, 200.0), out=print):
    """px per cell at each reference tag's spot in the school's camera layout (tag_cam.default_config: two cameras
    straight down over the halves, 40 cm overlap, 74 deg), and the robots' tag for comparison."""
    import tag_cam as CAMS
    rows = []
    for size in sizes:
        for h in heights:
            cfg = CAMS.default_config(size=size, height_cm=h, seed=5)
            cams = CAMS.cams_from_config(cfg)
            L, W = cfg["table"]
            poses = world_poses(default_config(), L, W)
            for i, (x, y, th) in sorted(poses.items()):
                best = 0.0
                for c in cams:
                    uv = c.project([[x, y, Z_CM]])[0]
                    if np.all(np.isfinite(uv)) and 0 <= uv[0] < c.W and 0 <= uv[1] < c.H:
                        best = max(best, c.px_per_cm(x, y, Z_CM))
                rows.append(dict(size="%dx%d" % size, lens_cm=h, id=int(i), px_per_cm=round(best, 2),
                                 px_per_cell=round(best * tag_cm / CELLS, 2), px_tag=round(best * tag_cm, 1)))
    for r in rows:
        out("%(size)s lens %(lens_cm).0f cm  tag %(id)d: %(px_per_cm).2f px/cm, %(px_per_cell).2f px a cell, "
            "%(px_tag).0f px across" % r)
    return rows


# ------------------------------------------------------------------------------------------------- paper sheet
def bits(family, tag_id):
    import tagcore as TC
    img = TC.tag_image(family, tag_id, px_per_cell=1, quiet_cells=0)
    return img < 128


def where_words(i, cfg=None):
    it = layout(cfg).get(str(i))
    if not it:
        return "?"
    if it["kind"] == "corner":
        k = it["corner"]
        return "CORNER %d%s" % (k, " = (0, 0)" if k == 1 else "")
    return "SIDE: the middle of the long side from corner %d to corner %d" % (it["edge"], it["edge"] % 4 + 1)


def pdf(out_path, cfg=None):
    """The paper option (letter, one tag a page, exact scale): the tag, its 15.5 cm square cut line, a 10 cm ruler, its
    id and where it goes. Like docs/hardware/checker-pieces-letter.pdf."""
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import cm
    from reportlab.pdfgen import canvas
    cfg = cfg or default_config()
    fam, s, off = cfg.get("family", FAMILY), float(cfg.get("tag_cm", TAG_CM)), float(cfg.get("offset_cm", OFFSET_CM))
    PW, PH = letter
    c = canvas.Canvas(out_path, pagesize=letter)
    c.setTitle("MIRLT reference tags (%s, %.0f cm, ids %s)" % (fam, s, ", ".join(str(i) for i in ids(cfg))))
    lay = layout(cfg)
    order = sorted(lay, key=lambda i: (lay[i]["kind"] != "corner", int(i)))
    for pg, i in enumerate(order):
        it = lay[i]
        c.setFont("Helvetica-Bold", 15)
        c.drawString(1.5 * cm, PH - 1.6 * cm, "Reference tag %s: %s" % (i, where_words(i, cfg)))
        c.setFont("Helvetica", 9)
        lines = ["Print at 100 %% (Actual size, never Fit to page) on MATTE paper. Check: the ruler is 10.0 cm and the "
                 "black square %.1f cm." % s,
                 "Cut on the dashed square (%.1f cm). Glue it on thin card so it lies flat." % (2 * off)]
        if it["kind"] == "corner":
            lines += ["Put it on the table top with the two sides marked EDGE exactly along the table's two edges at "
                      "corner %d" % it["corner"],
                      "(paper edge flush with the table edge, not hanging over). Tape it down flat. The tag's middle is "
                      "then %.2f cm in from both edges." % off]
        else:
            lines += ["Put its side marked EDGE exactly along the long side from corner %d to corner %d, near the "
                      "table's middle" % (it["edge"], it["edge"] % 4 + 1),
                      "(where both cameras see it). Its place along the edge is measured by the cameras; the tag's "
                      "middle is %.2f cm in from the edge." % off]
        lines += ["Which way the tag is turned does not matter; which corner or side it is in does (the id above)."]
        for k, ln in enumerate(lines):
            c.drawString(1.5 * cm, PH - 2.4 * cm - k * 0.42 * cm, ln)
        # the square, centred on the page below the text
        x0 = (PW - 2 * off * cm) / 2.0
        y0 = 3.6 * cm
        side = 2 * off * cm
        c.setDash(4, 3)
        c.setLineWidth(0.6)
        c.setStrokeColorRGB(0.45, 0.45, 0.45)
        c.rect(x0, y0, side, side, stroke=1, fill=0)
        c.setDash()
        b = bits(fam, int(i))
        n = b.shape[0]
        cell = s / n * cm
        tx, ty = x0 + (off - s / 2) * cm, y0 + (off - s / 2) * cm
        c.setFillColorRGB(0, 0, 0)
        for r in range(n):
            for q in range(n):
                if b[r, q]:
                    c.rect(tx + q * cell, ty + (n - 1 - r) * cell, cell + 0.2, cell + 0.2, stroke=0, fill=1)
        # EDGE marks on the square's sides that go along the table's edges (the tile frame's x = 0 and y = 0 sides:
        # the bottom (along the edge to the next corner) and, for corners, the left)
        c.setFont("Helvetica-Bold", 8)
        c.setFillColorRGB(0.1, 0.1, 0.1)
        c.drawCentredString(x0 + side / 2, y0 - 0.45 * cm, "EDGE (the table's edge goes here)")
        if it["kind"] == "corner":
            c.saveState()
            c.translate(x0 - 0.25 * cm, y0 + side / 2)
            c.rotate(90)
            c.drawCentredString(0, 0, "EDGE (the table's edge goes here)")
            c.restoreState()
            c.drawString(x0 - 0.9 * cm, y0 - 0.45 * cm, "corner")
        c.setFont("Helvetica", 8)
        c.drawString(x0, y0 + side + 0.2 * cm, "id %s  %s  (outside the cut line: nothing printed in the white)" % (i, fam))
        # the 10 cm ruler
        c.setStrokeColorRGB(0, 0, 0)
        c.setLineWidth(1.0)
        rx, ry = 1.5 * cm, 1.5 * cm
        c.line(rx, ry, rx + 10 * cm, ry)
        for k in range(11):
            h = 0.35 * cm if k % 5 == 0 else 0.2 * cm
            c.line(rx + k * cm, ry, rx + k * cm, ry + h)
        c.setFont("Helvetica", 8)
        c.drawString(rx + 10.3 * cm, ry - 0.05 * cm, "10 cm: measure it before cutting")
        c.drawRightString(PW - 1.5 * cm, 1.0 * cm, "page %d of %d   tools/fleet/ref_tags.py pdf" % (pg + 1, len(order)))
        c.showPage()
    c.save()
    return out_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["sizing", "pdf", "printcheck", "layout"])
    ap.add_argument("--out", default=None)
    ap.add_argument("--tag-cm", type=float, default=TAG_CM)
    a = ap.parse_args(argv)
    if a.what == "sizing":
        rows = []
        for t in (8.0, 10.0, 12.0, 15.0):
            print("--- %.0f cm reference tag" % t)
            rows += [dict(r, tag_cm=t) for r in sizing(t)]
        if a.out:
            with open(a.out, "w") as f:
                json.dump(rows, f, indent=1)
        return 0
    if a.what == "pdf":
        p = pdf(a.out or os.path.join(REPO, "docs", "hardware", "reference-tags-letter.pdf"))
        print("wrote %s" % p)
        return 0
    if a.what == "layout":
        print(json.dumps(world_poses(default_config(), 396.24, 167.64), indent=1))
        return 0
    if a.what == "printcheck":
        import ref_printcheck as RP
        return RP.main([])
    return 1


if __name__ == "__main__":
    sys.exit(main())
