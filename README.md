Some notes:

Lowest frequency for the infrared is 38000 which detects at the highest distance.

Adjust speeds of the both wheels to account for shifting

# Final lab plan

3 hours total: ~30 min calibration, ~2 h 15 min testing and assessment, last 15 min pack up.

## Which file is for which assessment

| Assessment | File | Notes |
|---|---|---|
| Extension 1 | `Essentials/cases2.cpp` | Unchanged, already working. Do **not** use `all.cpp` for this. |
| Extension 2 | `Ex1&2/all.cpp` | Right-hand rule, T-junctions, long dead end check. |
| Advanced | `advanced/last.cpp` | Cell-by-cell exploration + stored map of the test maze. |

Paste each into a `.ino` sketch in the Arduino IDE (functions are used before they are defined, which only works in `.ino`). Serial Monitor at **9600 baud**.

## Calibration (first 30 min)

The calibration constants are identical in all three files. **If you change one, change it in all three.**

1. **Stop pulses and straight driving (~10 min)**
   - Upload `cases2.cpp`. If it still drives straight and turns 90 degrees cleanly, the robot is fine.
   - Otherwise adjust `leftStop`, `rightStop`, then `L_FWD`, `R_FWD` (and the turn offsets if turns are off).
2. **Forward speed (~5 min)** - `FORWARD_SPEED_CM_S`
   - Drive straight for a timed run, measure the distance, speed = distance / time.
   - Current value 5.89 cm/s is from before `speedScale = 1.25`, so it is probably wrong.
   - Extension 2 uses it to measure straights. Advanced depends on it completely (one cell must be exactly 27 cm).
3. **Advanced wall readings (~10 min)** - `last.cpp`
   - Start the robot centred in a cell with the Serial Monitor open.
   - Each cell prints `walls L/F/R` (1 = wall). Check it against the real walls for a few cells.
   - If an open side ever shows as a wall, change `SIDE_WALL_ZONE` from 4 to 3.
4. **Extension 2 dead end distances (~5 min)** - `all.cpp`
   - Drive into a short (1 wall) and a long (2 walls) dead end. Serial prints `Dead end: straight cm = ..., joins = ...`.
   - Expect about **15 cm** (short) and **42 cm** (long). Long is accepted between `LONG_MIN_CM` 28 and `LONG_MAX_CM` 55.
   - Post counting is off (`USE_JOIN_COUNT = false`). Only turn it on if the long dead end prints `joins = 1`.

## Testing order

1. **Extension 1** with `cases2.cpp` - get it signed off first.
2. **Extension 2** with `all.cpp`.
3. **Advanced** with `last.cpp`.

## Things to watch

- **Advanced start:** place the robot centred in its start cell (axle in the middle), square to the walls.
- **Advanced progress:** Serial shows `map matches N`. When it reaches `map matches 1` the robot knows where it is and drives the shortest path. `FINISH` = stopped in the long dead end.
- **180 degree spins:** the rear (plug + ball) is 100 mm behind the axle and the corridor half-width is 100 mm, so clearance is very tight. Watch for rear contact in short dead ends.
- **Extension 2 wrong right turns:** a right turn triggers on a single reading. If it turns into a timber post, that's the cause.
- Time limit for Advanced is 150 s; the simulation took 50-100 s at the current speed.

## How the code works (every member must be able to explain this)

**Extension 2 (`all.cpp`)**
- Right-hand rule: right turn first, then straight, then left, then turn around.
- While driving straight it times the straight. A side opening or a turn restarts the timer.
- At a dead end: straight length 28-55 cm (two wall panels) = long dead end, stop. Otherwise spin 180 and carry on.

**Advanced (`last.cpp`)**
- The maze is a grid: 200 mm panel + 70 mm post = 27 cm per cell. The robot drives one cell, stops at the centre, reads left/front/right walls, then decides.
- **Exploring:** depth-first search. Go to an unvisited neighbour (straight, right, left), otherwise go back the way it came. It never re-enters a visited cell, so islands cannot trap it.
- **Locating itself:** it starts with every possible map position (36 cells x 4 headings x normal/mirrored = 288) and removes any that do not match the walls it sees. Usually only one is left after a few cells.
- **Once located:** it follows the shortest path on the stored map to the long dead end.
- **Finish:** stops when every remaining map position agrees it is at the end of the long dead end. If the map keeps failing to match (different maze), it falls back to exploring and the sensor check: dead end + previous cell has both side walls + the one before does not.
- Why not wall following: in the test maze the long dead end is attached to the central island, so following the outer wall never reaches it.

## Before leaving

- Upload code to each submission link: Extension 1, Extension 2, Advanced.
- Commit any calibration changes to git.
