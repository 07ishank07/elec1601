
#include <Servo.h> // Include servo library

Servo servoLeft;
Servo servoRight;

// Define all pins
// Right IR LED/receiver pair
const int irLedPinRight = 2;
const int irReceiverPinRight = 3;
const int redLedPinRight = A0;
// Middle IR LED/receiver pair
const int irLedPinMid = 6;
const int irReceiverPinMid = 7;
const int redLedPinMid = A1;
// Left IR LED/receiver pair
const int irLedPinLeft = 10;
const int irReceiverPinLeft = 11;
const int redLedPinLeft = A2;

// Physical values to adjust after testing, at the current servo pulse settings.
const float WHEEL_TRACK_CM = 12.0f; // Approx. from the sketch; measure between wheel centres.
const float FORWARD_SPEED_CM_S = 5.89f; // Initial estimate; recheck at speedScale 1.25 (65 mm wheel = 20.4 cm per turn).
// Mean wheel travel speed while spinning, NOT the speed of the robot's centre.
const float CLOCKWISE_SPEED_CM_S = 8.41f;
const float ANTICLOCKWISE_SPEED_CM_S = 8.41f;

const int leftStop  = 1490;
const int rightStop = 1492;

// Offsets in us from each stop pulse, calibrated at speedScale = 1.0
const int L_FWD =  37;   // 1525 - 1495
const int R_FWD = -35;   // 1468 - 1497
const int STEERING_US_PER_ZONE = 3; // Gentle correction towards equal side readings.
const int MAX_STEERING_US = 6;      // Limit the correction so both wheels stay forward.

const int L_REV = -37;   // starts as the mirror of forward; tune it
const int R_REV =  35;

const int L_CW  =  30;   // 1525 - 1495  (about 90 deg in 1400 ms)
const int R_CW  =  36;   // 1533 - 1497

const int L_CCW = -30;   // 1475 - 1495  (about 90 deg in 1400 ms)
const int R_CCW = -36;   // 1467 - 1497

// Controls servo pulse offsets only. If you change this, recheck the physical
// speed estimates above; pulse offset is not a direct measurement of speed.
float speedScale = 1.25;

// ---------- Cell-by-cell movement ----------
// The maze is a grid: 200 mm panels + 70 mm posts = 27 cm from cell centre to cell centre.
// The robot stops with its axle at each cell centre, reads the walls, then decides.
// At a cell centre: front wall ~4 cm from the nose (zone 2), side walls ~5.75 cm (zone 3).
const float CELL_CM = 27.0f;
const float SENSE_AHEAD_CM = 6.0f; // Side sensors are ~6 cm ahead of the axle.
const int FRONT_WALL_ZONE = 3;     // Front reading at/below this = wall in this cell.
const int SIDE_WALL_ZONE = 4;      // Side reading at/below this = wall in this cell.
const int FRONT_STOP_ZONE = 1;     // Emergency stop while driving.
const int FRONT_ALIGN_ZONE = 2;    // With a wall ahead, settle at this zone to resync the grid.

// ---------- Stored map of the given Advanced maze ----------
// Cells are (col,row) from the top-left, directions 0 N, 1 E, 2 S, 3 W.
// MAP_H[j] bit c: wall on the horizontal line above row j, column c (row MAP_N = bottom edge).
// MAP_V[i] bit r: wall on the vertical line left of column i, row r (column MAP_N = right edge).
// The start cell and heading are unknown, so the robot works out where it is by
// matching the walls it sees. Every rotation of the maze is covered by the unknown
// heading; the mirrored maze is checked as well. If nothing matches, it keeps exploring.
const int MAP_N = 6;
const byte MAP_H[MAP_N + 1] = {0b111111, 0b000100, 0b011101, 0b001101, 0b101010, 0b000100, 0b111111};
const byte MAP_V[MAP_N + 1] = {0b111111, 0b101010, 0b010010, 0b000000, 0b110110, 0b010101, 0b111111};

const int DX[4] = {0, 1, 0, -1};
const int DY[4] = {-1, 0, 1, 0};

// Candidate poses on the stored map: cell index, heading (bits 0-1) and mirrored (bit 2).
const int MAX_CANDS = MAP_N * MAP_N * 8;
byte candCell[MAX_CANDS];
byte candHead[MAX_CANDS];
int candCount = 0;
int mapFailures = 0;                // Times no map pose matched what the robot saw.
const int MAP_TRUST_FAILURES = 3;  // After this many, assume a different maze.
byte goalDist[MAP_N * MAP_N]; // Cells to the nearest long dead end on the map (255 = none).

// The robot's own map, in its own frame: it starts at the centre facing "north".
// Bits 0-3 walls (by direction), bit 4 visited, bits 5-6 direction back to the parent, bit 7 has parent.
const int GRID = 16;
const byte VISITED = 0x10;
const byte HAS_PARENT = 0x80;
byte cellInfo[GRID][GRID];
int x = GRID / 2, y = GRID / 2, hr = 0; // Robot position and heading in its own frame.
bool moved = false;      // False while still in the start cell (its back wall is unknown).
int sideL = -1, sideR = -1; // Side walls sampled while driving into this cell (-1 = not sampled).
bool finished = false;

void setup() {
  Serial.begin(9600);

  // setup servos
  servoLeft.attach(13);
  servoRight.attach(12);

  // setup all input pins
  pinMode(irReceiverPinRight, INPUT);
  pinMode(irReceiverPinMid, INPUT);
  pinMode(irReceiverPinLeft, INPUT);

  // setup all output pins
  pinMode(irLedPinRight, OUTPUT);
  pinMode(redLedPinRight, OUTPUT);
  pinMode(irLedPinMid, OUTPUT);
  pinMode(redLedPinMid, OUTPUT);
  pinMode(irLedPinLeft, OUTPUT);
  pinMode(redLedPinLeft, OUTPUT);

  resetCandidates();
  stopServos();
  delay(5000);
}

void loop() {
  if (finished) { // Stay stopped at the end of the long dead end.
    stopServos();
    setLeds(HIGH, HIGH, HIGH);
    return;
  }

  // 1. Read the walls of the cell just reached. The sides were sampled on the
  //    way in (sensors level with the cell middle); at the start, read them here.
  bool wf = frontWall();
  bool wl, wr;
  if (sideL < 0) {
    sideWalls(wl, wr);
  } else {
    wl = sideL;
    wr = sideR;
  }
  setLeds(wl ? HIGH : LOW, wf ? HIGH : LOW, wr ? HIGH : LOW);
  recordCell(wl, wf, wr);
  filterCandidates(wl, wf, wr);
  printStatus(wl, wf, wr);

  // 2. Finish: a straight exactly two cells long ending in a dead end. The map check
  //    is harder to fool with one misread wall, so use it unless the map keeps failing.
  bool atFinish = mapFailures < MAP_TRUST_FAILURES ? mapGoalReached(wl, wf, wr)
                                                   : atLongDeadEnd(wl, wf, wr);
  if (atFinish) {
    finished = true;
    stopServos();
    Serial.println("FINISH");
    return;
  }

  // 3. Turn towards the chosen exit. If a wall turns out to be ahead, remember it,
  //    drop the map poses that expected an opening there, and choose again.
  for (int attempt = 0; attempt < 4; ++attempt) {
    int rel = chooseMove();
    if (rel != 0) {
      turnRelative(rel);
      hr = (hr + rel) & 3;
      rotateCandidates(rel);
    }
    if (!frontWall()) break;
    cellInfo[x][y] |= 1 << hr;
    dropCandidatesExpectingOpening();
  }

  // 4. Drive one cell.
  driveCell();
  x += DX[hr];
  y += DY[hr];
  moved = true;
  moveCandidates();
}

// ---------- Exploring (robot's own map) ----------

bool inGrid(int cx, int cy) {
  return cx >= 0 && cx < GRID && cy >= 0 && cy < GRID;
}

// Store the walls of the current cell in the robot's own frame.
void recordCell(bool wl, bool wf, bool wr) {
  byte info = cellInfo[x][y];
  byte known[3] = {(byte)((hr + 3) & 3), (byte)hr, (byte)((hr + 1) & 3)};
  bool seen[3] = {wl, wf, wr};
  for (int k = 0; k < 3; ++k) {
    if (seen[k]) info |= 1 << known[k];
    else info &= ~(1 << known[k]);
  }
  if (moved) {
    byte back = (hr + 2) & 3;
    info &= ~(1 << back); // We just came through it.
    if (!(info & VISITED)) info |= HAS_PARENT | back << 5;
  }
  cellInfo[x][y] = info | VISITED;
}

// Relative turn for the next move: 0 straight, 1 right, 2 around, 3 left.
int chooseMove() {
  // Location known on a trusted map: take the shortest path.
  if (candCount == 1 && mapFailures < MAP_TRUST_FAILURES) {
    int rel = mapMove();
    if (rel >= 0) return rel;
  }

  // Depth-first search: an unvisited neighbour (straight, right, left), else back to the parent.
  // Visited cells are never re-entered, so islands cannot trap the robot in a loop.
  const int prefs[3] = {0, 1, 3};
  byte info = cellInfo[x][y];
  for (int k = 0; k < 3; ++k) {
    int d = (hr + prefs[k]) & 3;
    int nx = x + DX[d], ny = y + DY[d];
    if (!(info >> d & 1) && inGrid(nx, ny) && !(cellInfo[nx][ny] & VISITED)) return prefs[k];
  }
  if (info & HAS_PARENT) return ((info >> 5 & 3) - hr) & 3;

  // Back at the start with everything explored. Try behind the start (never checked),
  // otherwise forget the visited marks, keep the walls, and explore again.
  if (!(info >> ((hr + 2) & 3) & 1)) return 2;
  for (int i = 0; i < GRID; ++i)
    for (int j = 0; j < GRID; ++j) cellInfo[i][j] &= 0x0F;
  cellInfo[x][y] |= VISITED;
  for (int k = 0; k < 3; ++k) {
    int d = (hr + prefs[k]) & 3;
    if (!(info >> d & 1)) return prefs[k];
  }
  return 2;
}

// Long dead end seen by the robot itself: walls left, front and right here; the cell
// behind has walls on both sides; the cell before that does not (exactly two cells).
bool atLongDeadEnd(bool wl, bool wf, bool wr) {
  if (!(wl && wf && wr)) return false;
  byte s1 = (hr + 1) & 3, s2 = (hr + 3) & 3;
  int bx = x - DX[hr], by = y - DY[hr];
  int cx = bx - DX[hr], cy = by - DY[hr];
  if (!inGrid(cx, cy)) return false;
  byte b = cellInfo[bx][by], c = cellInfo[cx][cy];
  if (!(b & VISITED) || !(b >> s1 & 1) || !(b >> s2 & 1)) return false;
  return (c & VISITED) && !((c >> s1 & 1) && (c >> s2 & 1));
}

// ---------- Locating the robot on the stored map ----------

bool mapWall(byte mirror, int c, int r, int d) {
  if (mirror) {
    c = MAP_N - 1 - c;
    if (d == 1) d = 3;
    else if (d == 3) d = 1;
  }
  if (d == 0) return MAP_H[r] >> c & 1;
  if (d == 2) return MAP_H[r + 1] >> c & 1;
  if (d == 3) return MAP_V[c] >> r & 1;
  return MAP_V[c + 1] >> r & 1;
}

void resetCandidates() {
  candCount = 0;
  for (byte m = 0; m < 2; ++m)
    for (byte cell = 0; cell < MAP_N * MAP_N; ++cell)
      for (byte h = 0; h < 4; ++h) {
        candCell[candCount] = cell;
        candHead[candCount] = h | m << 2;
        ++candCount;
      }
}

// Keep only the poses whose map walls match what the robot sees.
// If none are left (misread or a different maze), start matching again from scratch.
void filterCandidates(bool wl, bool wf, bool wr) {
  for (int pass = 0; pass < 2; ++pass) {
    int kept = 0;
    for (int i = 0; i < candCount; ++i) {
      int c = candCell[i] % MAP_N, r = candCell[i] / MAP_N;
      byte h = candHead[i] & 3, m = candHead[i] >> 2;
      if (mapWall(m, c, r, (h + 3) & 3) == wl && mapWall(m, c, r, h) == wf
          && mapWall(m, c, r, (h + 1) & 3) == wr) {
        candCell[kept] = candCell[i];
        candHead[kept] = candHead[i];
        ++kept;
      }
    }
    candCount = kept;
    if (candCount > 0) return;
    Serial.println("Map mismatch: relocating");
    if (pass == 0) ++mapFailures;
    resetCandidates();
  }
}

// A wall was found straight ahead after turning.
void dropCandidatesExpectingOpening() {
  int kept = 0;
  for (int i = 0; i < candCount; ++i) {
    int c = candCell[i] % MAP_N, r = candCell[i] / MAP_N;
    if (!mapWall(candHead[i] >> 2, c, r, candHead[i] & 3)) continue;
    candCell[kept] = candCell[i];
    candHead[kept] = candHead[i];
    ++kept;
  }
  candCount = kept;
  if (candCount == 0) {
    ++mapFailures;
    resetCandidates();
  }
}

void rotateCandidates(int rel) {
  for (int i = 0; i < candCount; ++i)
    candHead[i] = (candHead[i] & 4) | ((candHead[i] + rel) & 3);
}

// Move every candidate one cell forward; drop those the map says hit a wall.
void moveCandidates() {
  int kept = 0;
  for (int i = 0; i < candCount; ++i) {
    int c = candCell[i] % MAP_N, r = candCell[i] / MAP_N;
    byte h = candHead[i] & 3, m = candHead[i] >> 2;
    if (mapWall(m, c, r, h)) continue;
    candCell[kept] = (r + DY[h]) * MAP_N + c + DX[h];
    candHead[kept] = candHead[i];
    ++kept;
  }
  candCount = kept;
  if (candCount == 0) {
    ++mapFailures;
    resetCandidates();
  }
}

// Long dead end on the map: one opening, the next cell is a closed straight,
// the cell after that is not.
bool mapLongDeadEnd(byte m, int c, int r) {
  int open = -1, openings = 0;
  for (int d = 0; d < 4; ++d)
    if (!mapWall(m, c, r, d)) { open = d; ++openings; }
  if (openings != 1) return false;
  int s1 = (open + 1) & 3, s2 = (open + 3) & 3;
  int bc = c + DX[open], br = r + DY[open];
  if (!mapWall(m, bc, br, s1) || !mapWall(m, bc, br, s2) || mapWall(m, bc, br, open)) return false;
  int cc = bc + DX[open], cr = br + DY[open];
  return !(mapWall(m, cc, cr, s1) && mapWall(m, cc, cr, s2));
}

// Breadth-first distances (in cells) from every map cell to the nearest long dead end.
void computeGoalDist(byte m) {
  byte queue[MAP_N * MAP_N];
  int head = 0, tail = 0;
  for (int i = 0; i < MAP_N * MAP_N; ++i) {
    goalDist[i] = 255;
    if (mapLongDeadEnd(m, i % MAP_N, i / MAP_N)) {
      goalDist[i] = 0;
      queue[tail++] = i;
    }
  }
  while (head < tail) {
    int i = queue[head++];
    int c = i % MAP_N, r = i / MAP_N;
    for (int d = 0; d < 4; ++d) {
      if (mapWall(m, c, r, d)) continue;
      int n = (r + DY[d]) * MAP_N + c + DX[d];
      if (goalDist[n] == 255) {
        goalDist[n] = goalDist[i] + 1;
        queue[tail++] = n;
      }
    }
  }
}

// Next move along the shortest path, preferring fewer turns; -1 if no path.
int mapMove() {
  int c = candCell[0] % MAP_N, r = candCell[0] / MAP_N;
  byte h = candHead[0] & 3, m = candHead[0] >> 2;
  computeGoalDist(m);
  int here = goalDist[candCell[0]];
  if (here == 255 || here == 0) return -1;
  const int prefs[4] = {0, 1, 3, 2};
  for (int k = 0; k < 4; ++k) {
    int d = (h + prefs[k]) & 3;
    if (mapWall(m, c, r, d)) continue;
    if (goalDist[(r + DY[d]) * MAP_N + c + DX[d]] == here - 1) return prefs[k];
  }
  return -1;
}

// Facing the end wall of a long dead end, and every pose that still matches agrees.
bool mapGoalReached(bool wl, bool wf, bool wr) {
  if (!(wl && wf && wr) || candCount == 0) return false;
  for (byte m = 0; m < 2; ++m) {
    computeGoalDist(m);
    for (int i = 0; i < candCount; ++i)
      if ((candHead[i] >> 2) == m && goalDist[candCell[i]] != 0) return false;
  }
  return true;
}

// ---------- Movement ----------

unsigned long turnMs(float angleDeg, float wheelSpeed) {
  return (unsigned long)((PI * WHEEL_TRACK_CM * angleDeg / (360.0 * wheelSpeed)) * 950);
}

// Turn in place at the cell centre: 1 right, 2 around, 3 left.
void turnRelative(int rel) {
  if (rel == 3) {
    setWheels(L_CCW, R_CCW);
    delay(turnMs(90.0, ANTICLOCKWISE_SPEED_CM_S));
  } else {
    setWheels(L_CW, R_CW);
    delay(turnMs(rel == 2 ? 180.0 : 90.0, CLOCKWISE_SPEED_CM_S));
  }
  stopServos();
  delay(100);
}

// Drive one cell forward, balancing between side walls when both are present.
// Side walls are sampled as the side sensors pass the middle of the new cell.
void driveCell() {
  unsigned long driveMs = (unsigned long)(CELL_CM / FORWARD_SPEED_CM_S * 1000);
  unsigned long senseMs = (unsigned long)((CELL_CM - SENSE_AHEAD_CM) / FORWARD_SPEED_CM_S * 1000);
  int samples = 0, leftVotes = 0, rightVotes = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < driveMs) {
    int dM = irDistance(irLedPinMid, irReceiverPinMid);
    int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
    int dR = irDistance(irLedPinRight, irReceiverPinRight);
    if (dM <= FRONT_STOP_ZONE) break; // Never run into the wall ahead.
    if (millis() - startTime >= senseMs && samples < 3) {
      leftVotes += dL <= SIDE_WALL_ZONE;
      rightVotes += dR <= SIDE_WALL_ZONE;
      ++samples;
    }
    int correction = 0;
    if (dL < 5 && dR < 5) { // Only balance when both walls are detected.
      correction = constrain((dR - dL) * STEERING_US_PER_ZONE, -MAX_STEERING_US, MAX_STEERING_US);
    }
    setWheels(L_FWD + correction, R_FWD + correction);
  }
  stopServos();

  // With a wall ahead, settle at a fixed distance from it so the cell count stays in step.
  unsigned long alignStart = millis();
  while (millis() - alignStart < 800) {
    int dM = irDistance(irLedPinMid, irReceiverPinMid);
    if (dM > FRONT_WALL_ZONE || dM == FRONT_ALIGN_ZONE) break;
    if (dM > FRONT_ALIGN_ZONE) setWheels(L_FWD, R_FWD);
    else setWheels(L_REV, R_REV);
  }
  stopServos();
  delay(100);

  if (samples == 3) {
    sideL = leftVotes >= 2;
    sideR = rightVotes >= 2;
  } else {
    sideL = sideR = -1; // Stopped early: read the sides in place instead.
  }
}

// ---------- Sensing ----------

bool frontWall() {
  int votes = 0;
  for (int i = 0; i < 3; ++i) votes += irDistance(irLedPinMid, irReceiverPinMid) <= FRONT_WALL_ZONE;
  return votes >= 2;
}

void sideWalls(bool &wl, bool &wr) {
  int leftVotes = 0, rightVotes = 0;
  for (int i = 0; i < 3; ++i) {
    leftVotes += irDistance(irLedPinLeft, irReceiverPinLeft) <= SIDE_WALL_ZONE;
    rightVotes += irDistance(irLedPinRight, irReceiverPinRight) <= SIDE_WALL_ZONE;
  }
  wl = leftVotes >= 2;
  wr = rightVotes >= 2;
}

void printStatus(bool wl, bool wf, bool wr) {
  Serial.print("cell ");
  Serial.print(x - GRID / 2);
  Serial.print(',');
  Serial.print(y - GRID / 2);
  Serial.print(" heading ");
  Serial.print(hr);
  Serial.print(" walls L/F/R ");
  Serial.print(wl);
  Serial.print(wf);
  Serial.print(wr);
  Serial.print(" map matches ");
  Serial.print(candCount);
  if (candCount == 1) {
    Serial.print(" at map cell ");
    Serial.print(candCell[0] % MAP_N);
    Serial.print(',');
    Serial.print(candCell[0] / MAP_N);
    if (candHead[0] & 4) Serial.print(" (mirrored)");
  }
  Serial.println();
}

// ---------- Helper functions ----------

void setLeds(int left, int mid, int right) {
  digitalWrite(redLedPinLeft, left);
  digitalWrite(redLedPinMid, mid);
  digitalWrite(redLedPinRight, right);
}

void stopServos() {
  servoLeft.writeMicroseconds(leftStop);
  servoRight.writeMicroseconds(rightStop);
}

void setWheels(int leftOffset, int rightOffset) {
  servoLeft.writeMicroseconds(leftStop + (int)round(speedScale * leftOffset));
  servoRight.writeMicroseconds(rightStop + (int)round(speedScale * rightOffset));
}

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency); // Turn on the IR LED square wave
  delay(1);                  // Wait 1 ms
  int ir = digitalRead(irReceiverPin);
  noTone(irLedPin);          // Turn off the IR LED
  delay(1);                  // Down time before recheck
  return ir;                 // Return 0 detect, 1 no detect
}

int irDistance(int irLedPin, int irReceiverPin)
{
  int distance = 0;
  for (long f = 38000; f <= 42000; f += 1000)
  {
    distance += irDetect(irLedPin, irReceiverPin, f);
  }
  return distance;
}
