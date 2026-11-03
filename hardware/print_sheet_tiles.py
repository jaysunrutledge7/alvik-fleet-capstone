"""The print sheet's page 2 (2026-11-03): the reference tag tiles and the roof tag tiles, for the print tech.

    python3 hardware/print_sheet_tiles.py       (rewrites hardware/PRINT_SHEET.pdf: page 1 kept as it is, page 2 new)

Page 1 (the dock, nose brackets, check tiles and machine models, 19 Oct) is copied unchanged; page 2 is drawn here
from the parts' numbers (hardware/dock/build_ref_tiles.py). Running it again replaces page 2, never adds a third.
"""
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SHEET = os.path.join(HERE, "PRINT_SHEET.pdf")


def page2():
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import cm
    from reportlab.pdfgen import canvas
    buf = io.BytesIO()
    W, H = letter
    c = canvas.Canvas(buf, pagesize=letter)
    y = H - 1.6 * cm

    def line(t, size=9, bold=False, dy=0.45):
        nonlocal y
        c.setFont("Helvetica-Bold" if bold else "Helvetica", size)
        c.drawString(1.6 * cm, y, t)
        y -= dy * cm
    line("3D print job, page 2: AprilTag tiles (Nick, 3 Nov 2026)", 14, True, 0.7)
    line("8 STL files, 8 parts (one of each). PLA, 0.4 mm nozzle, 0.2 mm layers, no supports. MATTE filament if you have it", 9)
    line("(a shiny tag reflects the ceiling lights and does not read). Files: hardware/dock/stl/ in the repository.", 9, dy=0.7)
    line("Settings that matter", 11, True, 0.5)
    for t in ["1. Millimetres, 100 % scale, no rotation: each file already lies the way it prints. XY size compensation 0.",
              "2. TWO COLOURS by a filament change (pause / M600) at a height. The tag must come out black on white:",
              "   - ref_corner_*.stl and ref_side_*.stl print FACE DOWN (the lips stand up): BLACK first, then change to",
              "     WHITE at 0.6 mm (after layer 3). The black squares are the first three layers on the bed.",
              "   - roof_tag_*.stl print face up, like the check tile: WHITE first, then change to BLACK at 1.2 mm.",
              "   No colour change possible? Print all white; Nick colours the raised squares with a black marker.",
              "3. A textured (matte) bed plate is best for the face-down tiles: their black faces come out matte.",
              "4. Keep the first layer true to size (elephant's-foot compensation on if the slicer has it): the squares' size",
              "   is the tag's size, and the cameras measure the table with it."]:
        line(t, 8.5, dy=0.42)
    y -= 0.3 * cm
    line("Parts", 11, True, 0.55)
    rows = [("File", "Qty", "Size as it lies (mm)", "Colours (order)", "About"),
            ("ref_corner_20.stl .. _23.stl", "1 each", "158 x 158 x 13.8", "black, M600 at 0.6 mm, white", "55 g each"),
            ("ref_side_24.stl, _25.stl", "1 each", "155 x 158 x 13.8", "black, M600 at 0.6 mm, white", "50 g each"),
            ("roof_tag_0.stl, roof_tag_1.stl", "1 each", "91 x 91 x 1.8", "white, M600 at 1.2 mm, black", "15 g each")]
    xs = [1.6, 7.6, 9.4, 12.9, 17.6]
    for k, r in enumerate(rows):
        c.setFont("Helvetica-Bold" if k == 0 else "Helvetica", 8.5)
        for x, t in zip(xs, r):
            c.drawString(x * cm, y, t)
        y -= 0.5 * cm
    y -= 0.2 * cm
    line("Each tile's id is cut into its lip (e.g. \"20 C1\", \"24 SIDE 1-2\"). Keep them apart: which tile goes where matters.", 8.5)
    line("Total about 350 g of PLA (the parts are thin: nearly solid). Print time: an estimate only, the slicer says.", 8.5, dy=0.8)
    line("For Nick (not the print tech)", 11, True, 0.55)
    for t in ["Corner tiles hook the table's corners (20 = corner 1 = (0, 0), then 21, 22, 23 counter-clockwise seen from",
              "above); side tiles hook the long sides in the middle where both cameras see (24: the side from corner 1",
              "to 2; 25: from 3 to 4). The lip goes against the table's edge; press the tile flat; tape it if it slides.",
              "Before use: with a ruler, a tile's black square must be 120 mm (roof tiles 80 mm). Over 1 mm off: say so.",
              "The roof tiles drop into the new roof's pocket: 91.5 x 91.5 mm, 1.2 mm deep, back-left corner cut",
              "8 mm at 45 deg (build_ref_tiles.py prints it). The roof is being designed with the RFID team.",
              "Paper instead: docs/hardware/reference-tags-letter.pdf (one tag a page, exact scale, cut lines)."]:
        line(t, 8.5, dy=0.42)
    c.setFont("Helvetica", 7)
    c.drawRightString(W - 1.6 * cm, 1.0 * cm, "page 2 of 2   hardware/print_sheet_tiles.py (numbers from build_ref_tiles.py)")
    c.showPage()
    c.save()
    return buf.getvalue()


def main():
    from pypdf import PdfReader, PdfWriter
    r = PdfReader(SHEET)
    w = PdfWriter()
    w.add_page(r.pages[0])
    w.add_page(PdfReader(io.BytesIO(page2())).pages[0])
    tmp = SHEET + ".tmp"
    with open(tmp, "wb") as f:
        w.write(f)
    os.replace(tmp, SHEET)
    print("wrote %s (page 1 kept, page 2: the tag tiles)" % SHEET)
    return 0


if __name__ == "__main__":
    sys.exit(main())
