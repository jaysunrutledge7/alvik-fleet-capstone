# Roof with an AprilTag: short spec (conference-table build, 2 Nov 2026)

What the software assumes about the roof. Change a number here and in the site profile
(`tools/fleet/sites/school-conference.json`, `tags`) together.

| Item | Value | Why |
|---|---|---|
| Footprint | Same as the Alvik (9.5 x 9.5 cm square), edges flush with the body | Nick's choice; nothing sticks out to catch the other robot or the table's edge |
| Top | Flat, level, matte white or light grey (paper glued on is enough) | A shiny or curved tag reads badly; the white margin is part of the tag's quiet zone |
| Tag | Family **tag25h9**, black square **8.0 cm**, printed from `docs/hardware/apriltag-sheet.pdf` at 100 %, or the 3D-printed drop-in tile (`hardware/dock/stl/roof_tag_0.stl`, `roof_tag_1.stl`, 2026-11-03) | Sizing tests: section 2 of the build report |
| Tag ids | Alpha = 0, Bravo = 1; 2-5 are spares | Site profile `tags.ids` |
| Placement | The tag's middle over the **middle of the wheel axle** (the robot's turning point), within 2 mm; the FRONT arrow towards the nose (the distance sensor) | The software takes the tag's middle as the robot's position and its top edge as the heading |
| Height | Measured by Nick: table top to the tag's printed face, to 1 mm, typed into `tags.height_cm` | Each camera ray is met at this height (a 1 cm error moves a robot up to 1 cm at the table's far end) |
| Fixing | Removable (two magnets or a clip), so the top buttons and the USB-C port can be reached | Flashing and power need them |
| Weight | Under about 30 g, centred | The wheels' grip and the turns |

**Keep clear:** the front distance sensor's view (60 x 60 deg forward from the nose: the roof must not overhang the
nose), the line sensors underneath, and the power switch.

**The external recorder's RFID tag.** Mount it under the roof plate, centred over the same axle midpoint, flat, with
nothing metal between it and the room (paper, PLA and glue are fine; foil tape, metal screws or the battery right
under it are not). Then both systems measure the same point on the robot, and the comparison has no offset to
guess. If it must stay where the other team put it, measure its offset from the axle midpoint (ahead, left, in cm)
and note it with the session id; the comparison tool can subtract it later.

**Check after fixing:** put the robot on the table, open Mission Control, Cameras & table, Check the cameras: the
tag should read in every picture ("tags read now ... 100 %"). Turn the robot by hand a quarter turn at a time: the
heading on the Run screen should step by about 90 degrees.

**The printed roof tile (2026-11-03).** `hardware/dock/build_ref_tiles.py` makes a drop-in tile per robot: the 8.0 cm
tag raised 0.6 mm in black on a 91.0 mm white tile, 1.2 mm thick, its back-left corner cut 8 mm at 45 deg. The roof
(being designed with the RFID team) needs a **pocket 91.5 x 91.5 mm, 1.2 mm deep, with the same corner cut**, centred
over the middle of the wheel axle, the uncut front edge towards the nose: the tile then goes in one way only (the
review's "tag stuck on turned 90 deg" cannot happen). The tag's face is 0.6 mm above the roof's top; measure the
height to it. The numbers are parameters at the top of the script (`ROOF_RIM_MM`, `POCKET_CLEAR_MM`, `KEY_MM`).
