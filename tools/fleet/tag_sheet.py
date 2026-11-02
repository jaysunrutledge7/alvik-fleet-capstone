#!/usr/bin/env python3
"""The printable tag sheet (conference-table build, 2026-11-02): each robot's roof tag at its exact printed size, the
roof square round it as a cutting line, its id and which edge is the robot's FRONT, plus spares and a 10 cm ruler to
check the printer did not scale the page.

    python3 tools/fleet/tag_sheet.py                       # docs/hardware/apriltag-sheet.pdf (US Letter)
    python3 tools/fleet/tag_sheet.py --ids 0,1,2,3,4,5 --tag-cm 8 --roof-cm 9.5 --out sheet.pdf --names 0=Alpha,1=Bravo

Drawn as vector squares (one per black cell), so the printer's resolution is the only limit. Print at 100 % ("Actual
size", never "Fit to page") on matte paper; measure the ruler and one tag's black square before cutting.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import numpy as np    # noqa: E402

import tagcore as TC  # noqa: E402


def bits(family, tag_id):
    """The tag's cells (True = black), the border included: n x n."""
    n = TC.FAMILIES[family][1]
    img = TC.tag_image(family, tag_id, px_per_cell=1, quiet_cells=0)
    assert img.shape == (n, n)
    return img < 128


def make(out, ids, family=TC.TAG_FAMILY, tag_cm=8.0, roof_cm=9.5, names=None, site=""):
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import cm
    from reportlab.pdfgen import canvas
    names = names or {}
    W, H = letter
    c = canvas.Canvas(out, pagesize=letter)
    c.setTitle("MIRLT roof tags (%s, %.1f cm)" % (family, tag_cm))
    n = TC.FAMILIES[family][1]
    cell = tag_cm / n
    per_page = 4
    pages = [ids[k:k + per_page] for k in range(0, len(ids), per_page)]
    for pg, page_ids in enumerate(pages):
        c.setFont("Helvetica-Bold", 12)
        c.drawString(1.2 * cm, H - 1.4 * cm, "Roof tags: %s, black square %.1f cm, roof %.1f cm%s  (page %d of %d)"
                     % (family, tag_cm, roof_cm, (", site " + site) if site else "", pg + 1, len(pages)))
        c.setFont("Helvetica", 8.5)
        lines = ["Print at 100 % (Actual size, never Fit to page) on MATTE paper: a shiny tag does not read.",
                 "Before cutting: the ruler must measure 10.0 cm and a tag's black square %.1f cm. If not, the" % tag_cm,
                 "printer scaled the page: print again. Cut on the grey dashed line (the roof's %.1f cm square)." % roof_cm,
                 "Glue it flat on the roof with the FRONT arrow towards the robot's nose (its distance sensor),",
                 "the tag's middle over the middle of the wheel axle. Keep spares off the table."]
        for k, ln in enumerate(lines):
            c.drawString(1.2 * cm, H - 2.1 * cm - k * 0.4 * cm, ln)
        y0 = H - 4.3 * cm
        c.setLineWidth(1.2)
        c.setStrokeColorRGB(0, 0, 0)
        c.line(1.2 * cm, y0, 11.2 * cm, y0)
        for k in range(11):
            h = 0.35 * cm if k % 5 == 0 else 0.2 * cm
            c.line((1.2 + k) * cm, y0, (1.2 + k) * cm, y0 + h)
        c.drawString(11.7 * cm, y0 - 0.1 * cm, "10 cm: measure this")
        col_x = [1.2, W / cm - 1.2 - roof_cm]
        row_top = [H / cm - 5.6, H / cm - 5.6 - roof_cm - 1.9]
        for k, i in enumerate(page_ids):
            rx = col_x[k % 2]
            ry = row_top[k // 2] - roof_cm
            c.setStrokeColorRGB(0.55, 0.55, 0.55)
            c.setLineWidth(0.5)
            c.setDash(3, 2)
            c.rect(rx * cm, ry * cm, roof_cm * cm, roof_cm * cm, stroke=1, fill=0)
            c.setDash()
            b = bits(family, i)
            tx = rx + (roof_cm - tag_cm) / 2.0
            ty = ry + (roof_cm - tag_cm) / 2.0
            c.setFillColorRGB(0, 0, 0)
            for r_ in range(n):
                for q in range(n):
                    if b[r_, q]:
                        # row 0 is the tag's top edge (the FRONT): drawn at the top of the square
                        c.rect((tx + q * cell) * cm, (ty + (n - 1 - r_) * cell) * cm, cell * cm + 0.2,
                               cell * cm + 0.2, stroke=0, fill=1)
            c.setFont("Helvetica-Bold", 10)
            c.drawString(rx * cm, (ry - 0.5) * cm, "id %d%s" % (i, (" = " + names[i]) if i in names else " (spare)"))
            # FRONT arrow above the square
            ax = (rx + roof_cm / 2.0) * cm
            ay = (ry + roof_cm + 0.2) * cm
            c.setStrokeColorRGB(0, 0, 0)
            c.setLineWidth(1.0)
            c.line(ax, ay, ax, ay + 0.6 * cm)
            pth = c.beginPath()
            pth.moveTo(ax - 0.15 * cm, ay + 0.42 * cm)
            pth.lineTo(ax, ay + 0.65 * cm)
            pth.lineTo(ax + 0.15 * cm, ay + 0.42 * cm)
            c.drawPath(pth, stroke=1, fill=0)
            c.setFont("Helvetica", 8)
            c.drawString(ax + 0.3 * cm, ay + 0.25 * cm, "FRONT (nose)")
        c.setFont("Helvetica", 7)
        c.drawString(1.2 * cm, 0.35 * cm, "tools/fleet/tag_sheet.py (OpenCV aruco %s: the same tags as the AprilTag "
                     "library's %s)" % (family, family))
        c.showPage()
    c.save()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(REPO, "docs", "hardware", "apriltag-sheet.pdf"))
    ap.add_argument("--ids", default="0,1,2,3,4,5")
    ap.add_argument("--family", default=TC.TAG_FAMILY)
    ap.add_argument("--tag-cm", type=float, default=8.0)
    ap.add_argument("--roof-cm", type=float, default=9.5)
    ap.add_argument("--names", default="0=Alpha,1=Bravo")
    ap.add_argument("--site", default="school-conference")
    a = ap.parse_args(argv)
    names = {int(k): v for k, v in (p.split("=") for p in a.names.split(",") if "=" in p)}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    make(a.out, [int(v) for v in a.ids.split(",")], a.family, a.tag_cm, a.roof_cm, names, a.site)
    print("Tag sheet: %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
