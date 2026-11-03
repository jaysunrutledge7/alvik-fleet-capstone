"""Reference tag tiles and drop-in roof tag tiles (2026-11-03, Nick approved): 3D-printed AprilTags.

    python hardware/dock/build_ref_tiles.py          (cadquery 2.4+, matplotlib, numpy, opencv: as build.py)

writes
    stl/ref_corner_20.stl .. ref_corner_23.stl   one per table corner (id 20 = corner 1 = (0, 0), 21, 22, 23 going
                                                 counter-clockwise seen from above): a 155 mm white square with the
                                                 120 mm tag raised 0.6 mm in black, and an L-shaped lip underneath that
                                                 hooks the table's corner
    stl/ref_side_24.stl, ref_side_25.stl         the two side tiles (24: the long side from corner 1 to corner 2, 25:
                                                 from corner 3 to corner 4), a straight lip hooking the edge
    stl/roof_tag_0.stl, roof_tag_1.stl           drop-in roof tiles for Alpha (0) and Bravo (1): the 80 mm tag on a
                                                 91.5 mm white tile with one corner cut (it fits its roof pocket one
                                                 way only)
    preview/ref_corner_tile.png, ref_side_tile.png, roof_tag_tile.png, ref_tile_lip_section.png
and prints a check report (ok / WARN / FAIL), the offsets the parts fix, and the roof pocket the roof tile needs.

THE PART FIXES THE TAG'S PLACE. The lip's inner face sits against the table's edge, the plate's underside on its top.
Seen from above, the tag's middle is OFFSET_MM (77.5 mm) in from each edge the tile hooks (both edges at a corner;
the one edge on a side tile); its black square runs from QUIET_MM (17.5 mm) to QUIET_MM + TAG_MM (137.5 mm) in from it.
These are tools/fleet/ref_tags.py's OFFSET_CM and TAG_CM: change them together.

PRINTED FACE DOWN (why, and the colour change). A lip underneath means the tile cannot lie tag-up on the bed without
support under all of it. Printed face down instead, the lips stand up from the bed and nothing needs support: the
black squares are the first 0.6 mm (3 layers of 0.2 mm) on the bed, then a filament change (pause / M600) at 0.6 mm,
then white for the rest. Turned over, it is exactly the check tile's build: a 1.2 mm white base with 0.6 mm raised black
squares. The squares' faces are the bed side: flat, and matte on a textured plate (a smooth plate gives a gloss: then
dull them with fine sandpaper or a matte marker). The white between the squares is printed 0.6 mm above the bed with
nothing under it (short bridges, 17 mm at the most): it may sag a little; only its colour matters to the camera. No
colour change: print it all white and colour the raised squares' faces black with a marker.
The roof tiles have no lip: they print tag up, exactly like the check tile (white, M600 at 1.2 mm, black).

Every number is CHOSEN (with a reason) or follows from ref_tags.py / the roof spec; the table thickness is ASSUMED
(19-38 mm, Nick: a typical conference table; the lip reaches 12 mm down, under the thinnest).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
for p in (os.path.join(REPO, "tools", "fleet"), os.path.join(REPO, "tools", "sim"), os.path.join(REPO, "tools", "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)
import build as B                    # noqa: E402

import ref_tags as RT                # noqa: E402

# =============================================================================== the reference tiles (mm)
TAG_MM = RT.TAG_CM * 10.0             # 120: the black square (ref_tags.TAG_CM)
CELLS = RT.CELLS                      # 7 cells across the black square (tag25h9)
CELL_MM = TAG_MM / CELLS              # 17.14
QUIET_MM = RT.QUIET_CM * 10.0         # 17.5: white round the square (a full cell)
OFFSET_MM = RT.OFFSET_CM * 10.0       # 77.5: the tag's middle from each edge line its tile hooks
PLATE_MM = 2 * OFFSET_MM              # 155: the white top, edge line to edge line
BASE_T_MM = 1.2                       # white base (the check tile's)
RAISE_MM = 0.6                        # black squares stand this proud (the check tile's)
OVERLAP_MM = 0.05                     # neighbouring squares overlap this much (one solid; corner-touching squares share
                                      # a 0.1 mm square so the STL stays manifold). Moves an edge 0.025 mm: nothing
LIP_T_MM = 3.0                        # CHOSEN: the lip's wall, 7 lines of a 0.4 mm nozzle; stiff enough to stay square
LIP_DEPTH_MM = 12.0                   # CHOSEN: how far the lip reaches down the table's edge, under the plate. Under
                                      # the thinnest top assumed (19 mm), so it never meets an apron or a leg rail; 12 mm
                                      # of edge keeps it square to the edge. Parametric: TABLE_TOP_MM below is checked
TABLE_TOP_MM = (19.0, 38.0)           # ASSUMED (Nick): a conference table's top, thinnest and thickest
CORNER_RELIEF_MM = 30.0               # CHOSEN: the corner tile's two lips start this far from the corner, so a table
                                      # whose corner is rounded (up to a 30 mm radius) still meets both lips on its
                                      # straight edges; the tag's place is from the two edge LINES (where they would
                                      # meet), which is the corner the cameras' fit uses
NOTCH_MM = 0.8                        # CHOSEN: a small groove where the lip meets the plate, so a print's bulge in that
                                      # inside corner (or a burr on the table's edge) cannot hold the lip off the edge
LABEL_MM = 5.0                        # the id and place, cut 0.5 mm into the lip's outside face (read when placing)
LABEL_DEPTH_MM = 0.5
BED_MM = 180.0                        # the smallest bed it should fit (Bambu A1 mini 180 mm)

# =============================================================================== the roof tiles (mm)
ROOF_TAG_MM = 80.0                    # the robots' tag (site profile tags.tag_cm 8.0)
ROOF_CM = 95.0                        # the roof's footprint (roof-tag-spec.md: 9.5 cm square)
ROOF_RIM_MM = 1.75                    # CHOSEN: the roof's wall round the pocket (4 lines of a 0.4 mm nozzle)
POCKET_CLEAR_MM = 0.25                # CHOSEN: pocket = tile + this each side (a drop-in fit; 0.2-0.3 mm for PLA)
ROOF_TILE_MM = ROOF_CM - 2 * ROOF_RIM_MM - 2 * POCKET_CLEAR_MM     # 91.0: the tile
ROOF_MARGIN_MM = (ROOF_TILE_MM - ROOF_TAG_MM) / 2.0                 # 5.5: white round the square (0.48 cell)
KEY_MM = 8.0                          # CHOSEN: the back-left corner cut at 45 deg this much along each side: the tile
                                      # goes into the pocket only with its FRONT (the tag's top) towards the nose
POCKET_DEPTH_MM = BASE_T_MM           # the pocket is as deep as the white base: the white sits flush, the black squares
                                      # stand 0.6 mm proud (the tag's face 0.6 mm above the roof's top)
NOZZLE_MM = 0.4
LINE_MM = 0.45                        # a 0.4 mm nozzle's line width (slicers' default)


def bits(tag_id):
    return RT.bits(RT.FAMILY, int(tag_id))


# =============================================================================== geometry
def cells_solid(tag_id, tag_mm, cx, cy, z0, h, mirror_x=False):
    """The tag's black squares as one solid: a box per black cell, raised h from z0, the tag's middle at (cx, cy),
    its printed top towards +y. mirror_x: drawn as seen from below (the face-down print)."""
    c = B.cq()
    b = bits(tag_id)
    n = b.shape[0]
    cell = tag_mm / n
    out = None
    for r in range(n):
        for q in range(n):
            if not b[r, q]:
                continue
            x = cx + (q - (n - 1) / 2.0) * cell
            y = cy + ((n - 1) / 2.0 - r) * cell
            if mirror_x:
                x = 2 * cx - x
            bx = c.Workplane("XY").box(cell + 2 * OVERLAP_MM, cell + 2 * OVERLAP_MM, h, centered=(True, True, False)) \
                .translate((x, y, z0))
            out = bx if out is None else out.union(bx)
    return out


def label(text, size=LABEL_MM):
    c = B.cq()
    return c.Workplane("XZ").text(text, size, -LABEL_DEPTH_MM - 0.2, kind="bold", halign="center", valign="center")


def ref_tile(tag_id, kind):
    """In use, tile frame (mm): the table's top at z = 0; the edge(s) the tile hooks are the lines y = 0 (both kinds)
    and x = 0 (corner): the table lies at x >= 0 (corner), y >= 0. The white base z 0..1.2, black squares to 1.8;
    lips below z = 0 outside the edge lines. The tag's middle at (OFFSET, OFFSET)."""
    c = B.cq()
    t = LIP_T_MM
    if kind == "corner":
        x0, x1 = -t, PLATE_MM
    else:
        x0, x1 = 0.0, PLATE_MM
    plate = c.Workplane("XY").box(x1 - x0, PLATE_MM + t, BASE_T_MM, centered=False).translate((x0, -t, 0))
    # the lip along y = 0 (the edge from this corner to the next one; the side tile's edge)
    ly0 = CORNER_RELIEF_MM if kind == "corner" else 0.0
    lip = c.Workplane("XY").box(PLATE_MM - ly0, t, LIP_DEPTH_MM, centered=False).translate((ly0, -t, -LIP_DEPTH_MM))
    body = plate.union(lip)
    # the groove where the lip meets the plate, on the lip's inner face
    body = body.cut(c.Workplane("XY").box(PLATE_MM + 2 * t, NOTCH_MM, NOTCH_MM, centered=False)
                    .translate((-t, -NOTCH_MM, -NOTCH_MM)))
    if kind == "corner":
        lip2 = c.Workplane("XY").box(t, PLATE_MM - CORNER_RELIEF_MM, LIP_DEPTH_MM, centered=False) \
            .translate((-t, CORNER_RELIEF_MM, -LIP_DEPTH_MM))
        body = body.union(lip2)
        body = body.cut(c.Workplane("XY").box(NOTCH_MM, PLATE_MM + 2 * t, NOTCH_MM, centered=False)
                        .translate((-NOTCH_MM, -t, -NOTCH_MM)))
    body = body.union(cells_solid(tag_id, TAG_MM, OFFSET_MM, OFFSET_MM, BASE_T_MM, RAISE_MM))
    # the id and place on the lip's outside face (y = -t), read upright when the tile is in place
    where = ("C%d" % RT.layout()[str(tag_id)]["corner"]) if kind == "corner" else \
        ("SIDE %d-%d" % (RT.layout()[str(tag_id)]["edge"], RT.layout()[str(tag_id)]["edge"] % 4 + 1))
    try:
        txt = label("%d %s" % (tag_id, where)).translate((PLATE_MM / 2 + (ly0 / 2), -t, -LIP_DEPTH_MM / 2))
        body = body.cut(txt)
    except Exception as e:     # noqa: BLE001  (no font: the label is optional; the paper sheet names it too)
        print("WARN label not cut (%s)" % e)
    return body


def roof_tile(tag_id):
    """In use, roof frame (mm): the tile's middle at (0, 0), the robot's nose towards +y (the tag's printed top), the
    roof's top at z = 0... the tile lies in its pocket: its base z 0..1.2, the black squares to 1.8. The back-left
    corner (-x, -y) cut at 45 deg: the key."""
    c = B.cq()
    h = ROOF_TILE_MM / 2.0
    k = KEY_MM
    pts = [(-h + k, -h), (h, -h), (h, h), (-h, h), (-h, -h + k)]
    base = c.Workplane("XY").polyline(pts).close().extrude(BASE_T_MM)
    return base.union(cells_solid(tag_id, ROOF_TAG_MM, 0.0, 0.0, BASE_T_MM, RAISE_MM))


def roof_pocket_outline(clear=POCKET_CLEAR_MM):
    """The pocket the roof needs (its outline, mm, roof frame): the tile + clear each side, the same key corner."""
    h = ROOF_TILE_MM / 2.0 + clear
    k = KEY_MM
    return [(-h + k, -h), (h, -h), (h, h), (-h, h), (-h, -h + k)]


def face_down(shape):
    """The print orientation: turned over (the black squares on the bed), lying at z >= 0."""
    s = shape.rotate((0, 0, 0), (1, 0, 0), 180)
    b = s.val().BoundingBox()
    return s.translate((-b.xmin, -b.ymin, -b.zmin))


def on_bed(shape):
    b = shape.val().BoundingBox()
    return shape.translate((-b.xmin, -b.ymin, -b.zmin))


# =============================================================================== checks
def checks(parts):
    out, ok = [], True

    def rule(good, text, warn=False):
        nonlocal ok
        tag = "ok  " if good else ("WARN" if warn else "FAIL")
        if not good and not warn:
            ok = False
        out.append("%s %s" % (tag, text))
    rule(CELL_MM >= 10 * LINE_MM, "reference tag cell %.2f mm = %.0f lines of a %.1f mm nozzle (a cell's edge is a "
         "straight perimeter; its corners round by about half a line, %.2f mm = %.1f %% of a cell)" % (
             CELL_MM, CELL_MM / LINE_MM, NOZZLE_MM, LINE_MM / 2, 100 * LINE_MM / 2 / CELL_MM))
    rc = ROOF_TAG_MM / CELLS
    rule(rc >= 10 * LINE_MM, "roof tag cell %.2f mm = %.0f lines (corners round %.1f %% of a cell)" % (
        rc, rc / LINE_MM, 100 * LINE_MM / 2 / rc))
    rule(RAISE_MM >= 2 * 0.2 - 1e-9, "raised squares %.1f mm = %d layers of 0.2 mm (the colour change between "
         "layer %d and %d)" % (RAISE_MM, round(RAISE_MM / 0.2), round(RAISE_MM / 0.2), round(RAISE_MM / 0.2) + 1))
    rule(LIP_DEPTH_MM < TABLE_TOP_MM[0], "the lip reaches %.0f mm down: under the thinnest top assumed (%.0f mm), so "
         "it never meets anything under the top; it holds on any top from %.0f to %.0f mm" % (
             LIP_DEPTH_MM, TABLE_TOP_MM[0], TABLE_TOP_MM[0], TABLE_TOP_MM[1]))
    rule(QUIET_MM >= CELL_MM, "white round the reference tag %.1f mm >= a cell (%.1f mm)" % (QUIET_MM, CELL_MM))
    rule(ROOF_MARGIN_MM >= 0.45 * rc, "white round the roof tag %.2f mm = %.2f cell (the roof tags read with 0.5)" % (
        ROOF_MARGIN_MM, ROOF_MARGIN_MM / rc), warn=True)
    for name, sh in parts.items():
        bb = sh.val().BoundingBox()
        fits = max(bb.xlen, bb.ylen) <= BED_MM - 10
        rule(fits, "%s: %.1f x %.1f x %.1f mm as printed; fits a %.0f mm bed with a 5 mm brim" % (
            name, bb.xlen, bb.ylen, bb.zlen, BED_MM))
        rule(sh.val().isValid(), "%s: a valid solid (one piece, closed)" % name)
    return ok, out


# =============================================================================== previews
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def top_png(tag_id, kind, path):
    """The tile from above as it lies on the table: the white plate, the tag, the lips (dashed: underneath) and the
    offsets the part fixes."""
    plt = _plt()
    from matplotlib.patches import Rectangle
    b = bits(tag_id)
    n = b.shape[0]
    fig, ax = plt.subplots(figsize=(6.8, 6.6), dpi=110)
    t = LIP_T_MM
    x0 = -t if kind == "corner" else 0.0
    ax.add_patch(Rectangle((-40, -40), 260, 40, color="#5b6474", alpha=0.25, lw=0))     # beyond the edge (floor)
    if kind == "corner":
        ax.add_patch(Rectangle((-40, 0), 40, 220, color="#5b6474", alpha=0.25, lw=0))
    ax.add_patch(Rectangle((0, 0), 220, 220, color="#d8c9a8", lw=0))                     # the table top
    ax.add_patch(Rectangle((x0, -t), PLATE_MM - x0, PLATE_MM + t, facecolor="white", edgecolor="#333", lw=1.2))
    cell = CELL_MM
    for r in range(n):
        for q in range(n):
            if b[r, q]:
                ax.add_patch(Rectangle((QUIET_MM + q * cell, QUIET_MM + (n - 1 - r) * cell), cell, cell, color="black",
                                       lw=0))
    ly0 = CORNER_RELIEF_MM if kind == "corner" else 0.0
    ax.add_patch(Rectangle((ly0, -t), PLATE_MM - ly0, t, fill=False, ls="--", ec="#c2410c", lw=1.5))
    if kind == "corner":
        ax.add_patch(Rectangle((-t, ly0), t, PLATE_MM - ly0, fill=False, ls="--", ec="#c2410c", lw=1.5))
    ax.plot([OFFSET_MM], [OFFSET_MM], "+", color="#2952cc", ms=16, mew=2)
    ax.annotate("", (OFFSET_MM, -20), (0, -20), arrowprops=dict(arrowstyle="<->", color="#2952cc"))
    ax.text(OFFSET_MM / 2, -27, "%.1f mm" % OFFSET_MM, ha="center", color="#2952cc", fontsize=9)
    if kind == "corner":
        ax.annotate("", (-20, OFFSET_MM), (-20, 0), arrowprops=dict(arrowstyle="<->", color="#2952cc"))
        ax.text(-24, OFFSET_MM / 2, "%.1f mm" % OFFSET_MM, ha="right", va="center", color="#2952cc", fontsize=9,
                rotation=90)
    where = RT.where_words(tag_id)
    ax.set_title("Tag %d, %s: from above, on the table\nthe tag's middle (+) is %.1f mm in from %s" % (
        tag_id, where.lower(), OFFSET_MM, "both edges" if kind == "corner" else "the edge"), fontsize=10)
    ax.set_xlim(-45, 215 if kind == "side" else 200)
    ax.set_ylim(-45, 200)
    ax.set_aspect("equal")
    ax.set_xlabel("mm along the edge%s" % (" from the corner" if kind == "corner" else ""))
    ax.set_ylabel("mm into the table")
    fig.text(0.5, 0.01, "Tan: the table top; grey: past its edge; dashed orange: the lip underneath (hooks the edge).\n"
             "From the CAD numbers (build_ref_tiles.py).", ha="center", fontsize=8)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def section_png(path):
    """A cut through the lip at the table's edge, on the thinnest and the thickest top assumed; and the print
    orientation (face down)."""
    plt = _plt()
    from matplotlib.patches import Rectangle, Polygon
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.8), dpi=110)
    t = LIP_T_MM
    for ax, top in zip(axs[:2], TABLE_TOP_MM):
        ax.add_patch(Rectangle((0, -top), 40, top, color="#d8c9a8", lw=0))
        ax.add_patch(Polygon([(-t, -LIP_DEPTH_MM), (0, -LIP_DEPTH_MM), (0, -NOTCH_MM), (-NOTCH_MM, -NOTCH_MM),
                              (-NOTCH_MM, 0), (40, 0), (40, BASE_T_MM), (-t, BASE_T_MM)], closed=True,
                             facecolor="white", edgecolor="#333", lw=1.2))
        ax.add_patch(Rectangle((QUIET_MM, BASE_T_MM), 25, RAISE_MM, color="black", lw=0))
        ax.set_title("table top %.0f mm thick" % top, fontsize=10)
        ax.set_xlim(-10, 40)
        ax.set_ylim(-top - 5, 6)
        ax.set_aspect("equal")
        ax.set_xlabel("mm from the table's edge")
    axs[0].set_ylabel("mm above the table top")
    ax = axs[2]
    ax.add_patch(Rectangle((0, 0), 40, RAISE_MM, color="black", lw=0))
    ax.add_patch(Rectangle((-t, RAISE_MM), 43, BASE_T_MM, facecolor="white", edgecolor="#333", lw=1.2))
    ax.add_patch(Rectangle((-t, RAISE_MM + BASE_T_MM), t, LIP_DEPTH_MM, facecolor="white", edgecolor="#333", lw=1.2))
    ax.axhline(0, color="#5b6474", lw=3)
    ax.annotate("filament change\n(M600) at %.1f mm" % RAISE_MM, (30, RAISE_MM), (12, 9), fontsize=8,
                arrowprops=dict(arrowstyle="->"))
    ax.set_title("as printed: face down on the bed", fontsize=10)
    ax.set_xlim(-10, 40)
    ax.set_ylim(-2, 16)
    ax.set_aspect("equal")
    ax.set_xlabel("mm")
    fig.text(0.5, 0.01, "Cuts through the lip (left two: in use; right: printing). The lip's inner face is the edge "
             "line; the plate's underside rests on the top. From the CAD numbers (build_ref_tiles.py).", ha="center",
             fontsize=8)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def roof_png(path):
    plt = _plt()
    from matplotlib.patches import Rectangle, Polygon
    b = bits(0)
    n = b.shape[0]
    fig, ax = plt.subplots(figsize=(6, 6), dpi=110)
    h = ROOF_CM / 2
    ax.add_patch(Rectangle((-h, -h), ROOF_CM, ROOF_CM, facecolor="#e5e7eb", edgecolor="#333", lw=1.2))
    ax.add_patch(Polygon(roof_pocket_outline(), closed=True, facecolor="#cbd5e1", edgecolor="#555", ls="--", lw=1))
    ax.add_patch(Polygon(roof_pocket_outline(0.0), closed=True, facecolor="white", edgecolor="#333", lw=1))
    cell = ROOF_TAG_MM / n
    for r in range(n):
        for q in range(n):
            if b[r, q]:
                ax.add_patch(Rectangle((-ROOF_TAG_MM / 2 + q * cell, -ROOF_TAG_MM / 2 + (n - 1 - r) * cell), cell,
                                       cell, color="black", lw=0))
    ax.annotate("FRONT (nose)", (0, h + 2), (0, h + 9), ha="center", fontsize=9, arrowprops=dict(arrowstyle="->"))
    ax.text(-h + 2, -h + 2, "key", fontsize=8)
    ax.set_xlim(-h - 6, h + 6)
    ax.set_ylim(-h - 6, h + 14)
    ax.set_aspect("equal")
    ax.set_title("Roof tile, tag 0 (Alpha): %.1f mm tile in a %.1f mm pocket, %.1f mm deep\n(grey: the %.0f mm roof; "
                 "one corner cut so it goes in one way only)" % (ROOF_TILE_MM, ROOF_TILE_MM + 2 * POCKET_CLEAR_MM,
                                                                POCKET_DEPTH_MM, ROOF_CM), fontsize=10)
    ax.set_xlabel("mm across the robot (its left is -x)")
    ax.set_ylabel("mm along the robot")
    fig.text(0.5, 0.01, "From the CAD numbers (build_ref_tiles.py); the roof itself is being designed with the RFID "
             "team.", ha="center", fontsize=8)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(path)
    plt.close(fig)


def export(shape, name):
    B.cq().exporters.export(shape, os.path.join(B.STL, name), tolerance=0.02, angularTolerance=0.1)


def main(argv=None):
    os.makedirs(B.STL, exist_ok=True)
    os.makedirs(B.PNG, exist_ok=True)
    parts = {}
    for k, i in sorted(RT.CORNER_IDS.items()):
        parts["ref_corner_%d" % i] = face_down(ref_tile(i, "corner"))
    for k, i in sorted(RT.SIDE_IDS.items()):
        parts["ref_side_%d" % i] = face_down(ref_tile(i, "side"))
    for i in (0, 1):
        parts["roof_tag_%d" % i] = on_bed(roof_tile(i))
    ok, lines = checks(parts)
    print("reference tiles (face down, lips up) and roof tiles (tag up):")
    print("\n".join(lines))
    for name, sh in parts.items():
        export(sh, name + ".stl")
    top_png(20, "corner", os.path.join(B.PNG, "ref_corner_tile.png"))
    top_png(24, "side", os.path.join(B.PNG, "ref_side_tile.png"))
    section_png(os.path.join(B.PNG, "ref_tile_lip_section.png"))
    roof_png(os.path.join(B.PNG, "roof_tag_tile.png"))
    B.preview("ref_corner_20 (as printed: face down)", parts["ref_corner_20"],
              os.path.join(B.PNG, "ref_corner_tile_3d.png"), plan_z=1.0)
    print("\nwhat the parts fix (tools/fleet/ref_tags.py must match):")
    print("  reference tag middle: %.2f mm in from each edge line the tile hooks (OFFSET_CM %.3f)" % (
        OFFSET_MM, RT.OFFSET_CM))
    print("  black square %.1f mm (%d cells of %.2f mm), from %.1f to %.1f mm in; tag face %.1f mm above the table "
          "(Z_CM %.2f)" % (TAG_MM, CELLS, CELL_MM, QUIET_MM, QUIET_MM + TAG_MM, BASE_T_MM + RAISE_MM, RT.Z_CM))
    print("  corner lips start %.0f mm from the corner (fits a corner rounded up to that radius); lip %.0f mm deep, "
          "%.0f mm thick" % (CORNER_RELIEF_MM, LIP_DEPTH_MM, LIP_T_MM))
    print("\nthe roof pocket the roof tile needs (give this to the roof design):")
    print("  %.2f x %.2f mm, %.1f mm deep, its back-left corner cut %.1f mm along each side at 45 deg (the tile is "
          "%.1f mm with the same cut; %.2f mm play each side); centred on the wheel axle's middle, the uncut front edge "
          "towards the nose; a %.2f mm wall round it on the %.0f mm roof; the tag's face %.1f mm above the roof top" % (
              ROOF_TILE_MM + 2 * POCKET_CLEAR_MM, ROOF_TILE_MM + 2 * POCKET_CLEAR_MM, POCKET_DEPTH_MM,
              KEY_MM + POCKET_CLEAR_MM * (2 - math.sqrt(2)), ROOF_TILE_MM, POCKET_CLEAR_MM, ROOF_RIM_MM, ROOF_CM,
              RAISE_MM))
    print("\nwrote %s/{ref_corner_20..23, ref_side_24, ref_side_25, roof_tag_0, roof_tag_1}.stl" % B.STL)
    print("      %s/{ref_corner_tile, ref_corner_tile_3d, ref_side_tile, ref_tile_lip_section, roof_tag_tile}.png" % B.PNG)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
