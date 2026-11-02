# Roof with an AprilTag: short spec (conference-table build, 2 Nov 2026)

What the software assumes about the roof. Change a number here and in the site profile
(`tools/fleet/sites/school-conference.json`, `tags`) together.

| Item | Value | Why |
|---|---|---|
| Footprint | Same as the Alvik (9.5 x 9.5 cm square), edges flush with the body | Nick's choice; nothing sticks out to catch the other robot or the table's edge |
| Top | Flat, level, matte white or light grey (paper glued on is enough) | A shiny or curved tag reads badly; the white margin is part of the tag's quiet zone |
| Tag | Family **tag25h9**, black square **8.0 cm**, printed from `docs/hardware/apriltag-sheet.pdf` at 100 % | Sizing tests: section 2 of the build report |
| Tag ids | JayA = 0, JayB = 1; 2-5 are spares | Site profile `tags.ids` |
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
