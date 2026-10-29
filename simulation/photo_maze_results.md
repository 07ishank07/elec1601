# Photo maze logic simulation

This is an ideal topology/classifier test, not execution of the complete Arduino sketch.
Turns, body clearance, sensor rays, gap recognition and distance estimates are idealised.
The maze graph was manually transcribed from the supplied photograph.

## start1, facing east: PASS

start1: case 1 -> east_end: case 4 -> start1: case 1 -> bottom_T: case 2 -> S1: case 2 -> S2: case 3 -> top_right: case 3 -> top_inner: case 3 -> inner_corner: case 2 -> upper_T: case 2 -> upper_short_end: case 4 -> upper_T: case 1 -> lower_corner: case 2 -> left_T: case 2 -> FINISH: case 4

## start1, facing west: PASS

start1: case 1 -> bottom_T: case 2 -> S1: case 2 -> S2: case 3 -> top_right: case 3 -> top_inner: case 3 -> inner_corner: case 2 -> upper_T: case 2 -> upper_short_end: case 4 -> upper_T: case 1 -> lower_corner: case 2 -> left_T: case 2 -> FINISH: case 4

## start2, facing east: PASS

start2: case 1 -> bottom_T: case 1 -> start1: case 1 -> east_end: case 4 -> start1: case 1 -> bottom_T: case 2 -> S1: case 2 -> S2: case 3 -> top_right: case 3 -> top_inner: case 3 -> inner_corner: case 2 -> upper_T: case 2 -> upper_short_end: case 4 -> upper_T: case 1 -> lower_corner: case 2 -> left_T: case 2 -> FINISH: case 4

## start2, facing west: PASS

start2: case 1 -> west_end: case 4 -> start2: case 1 -> bottom_T: case 1 -> start1: case 1 -> east_end: case 4 -> start1: case 1 -> bottom_T: case 2 -> S1: case 2 -> S2: case 3 -> top_right: case 3 -> top_inner: case 3 -> inner_corner: case 2 -> upper_T: case 2 -> upper_short_end: case 4 -> upper_T: case 1 -> lower_corner: case 2 -> left_T: case 2 -> FINISH: case 4

## Failure probes

- L/M/R = 4/2/4 selects case 0 indefinitely if readings persist, despite a front wall.
- L/M/R = 3/2/5 for 150 ms qualifies a right opening without any forward travel. A missed right wall at a dead end can therefore trigger a turn rather than finish detection.
- A 7 cm post is rejected as an opening by the nominal 10 cm moving-gap filter, but this protection is bypassed when the middle reading is <= 2.
- At the stated wheel speeds, the 950 rather than 1000 turn multiplier requests 85.5 degrees for a quarter-turn and 171 degrees for a U-turn (before integer rounding). The ideal route test assumes exact turns and does not validate this calibration.
- Finish length is supplied as ideal 42 cm; the route pass does not validate when real side sensors establish straightKnown, or the resulting measured length.

Conclusion: both starts pass the ideal route test, but the current sketch is not demonstrated to finish reliably on the physical maze.
