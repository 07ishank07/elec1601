
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
const float FORWARD_SPEED_CM_S = 5.89f; // Initial estimate from the previous speed setting.
const float REVERSE_SPEED_CM_S = 5.89f; // Measure reverse speed separately if needed.
// Mean wheel travel speed while spinning, NOT the speed of the robot's centre.
// Initial estimates from the previous ~1.12 s quarter-turn and 12 cm track.
const float CLOCKWISE_SPEED_CM_S = 8.41f;
const float ANTICLOCKWISE_SPEED_CM_S = 8.41f;
const float TILT_ANGLE_DEG = 13.0f;
const unsigned long LEFT_START_TIMEOUT_MS = 2000; // Maximum centering attempt, either side.

// Straight travel: time(ms) = distance(cm) / speed(cm/s) * 1000.
// In-place turn: each wheel travels PI * track * angle / 360 centimetres.
// Turn time(ms) = wheel travel / mean wheel speed * 1000.
// These assume negligible wheel slip and approximately equal opposing speeds.

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

// =====================================================================
// ADVANCED: find the long dead end in the big maze within 150 seconds.
//
// The idea in 3 steps:
//   1. The robot moves ONE CELL at a time and looks at the walls in each cell.
//   2. It compares what it sees with a map of the maze, crossing out every
//      place it cannot be. While it is still unsure, it follows the right wall.
//   3. Once only one place is left, it knows where it is, and follows the
//      "distance to finish" numbers downhill to the long dead end.
//
// Why not just follow the right wall? In this maze the long dead end is
// attached to the island in the middle, so following the outside wall
// would never reach it.
// =====================================================================

// ---------- Moving one cell at a time ----------
// Each wall panel is 200 mm and each timber post 70 mm, so cell centres are 27 cm apart.
// The robot stops with its wheels (axle) in the middle of each cell.
const float CELL_CM = 27.0f;
const float SENSOR_AHEAD_CM = 6.0f; // The IR sensors are about 6 cm in front of the axle.

// IR zones (0 = very close, 5 = nothing seen), standing in the middle of a cell:
//   a wall in front is about 4 cm away (zone 2), a wall at the side about 6 cm (zone 3).
const int FRONT_WALL_ZONE = 3; // Front reading this or lower = wall in front.
const int SIDE_WALL_ZONE = 4;  // Side reading this or lower = wall at the side. Try 3 if openings show as walls.
const int FRONT_STOP_ZONE = 1; // Emergency stop while driving: never touch the wall.

// ---------- The maze map ----------
// The maze is 6 x 6 cells. Row 0 is the top, column 0 is the left.
// Directions: 0 = north (up), 1 = east (right), 2 = south (down), 3 = west (left).
const int MAZE_SIZE = 6;
const int ROW_STEP[4] = {-1, 0, 1, 0}; // How the row changes when moving N, E, S, W.
const int COL_STEP[4] = {0, 1, 0, -1}; // How the column changes when moving N, E, S, W.

// Walls of every cell, written as a sum: north = 1, east = 2, south = 4, west = 8.
// Example: 9 = 1 + 8 = walls on the north and west sides.
const byte WALLS[MAZE_SIZE][MAZE_SIZE] = {
  { 9,  1,  5,  1,  3, 11},
  {14, 10, 13,  6, 12,  2},
  {13,  0,  5,  7, 11, 10},
  {11, 12,  1,  5,  0,  6},
  { 8,  3, 12,  3, 10, 11},
  {14, 12,  5,  6, 12,  6},
};

// How many cells each cell is from the finish (0 = the end of the long dead end,
// row 2, column 3). Moving to a neighbour with a smaller number always gets closer.
const byte DIST_TO_FINISH[MAZE_SIZE][MAZE_SIZE] = {
  { 5,  4,  5,  6,  7, 10},
  { 6,  3,  8,  7,  8,  9},
  { 3,  2,  1,  0,  7,  8},
  {12,  3,  4,  5,  6,  7},
  {11, 10,  5,  6,  7, 10},
  {12,  9,  8,  7,  8,  9},
};

// possible[row][col][heading] is true if the robot could be in that cell facing that way.
// At the start we don't know, so everything is possible.
bool possible[MAZE_SIZE][MAZE_SIZE][4];

// Walls seen in the current cell.
bool wallLeft = false, wallRight = false;
bool sidesAlreadyRead = false; // True if the side walls were read while driving in.
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

  startGuessingAgain();
  stopServos();
  delay(5000);
}

void loop() {
  // Finished: stay stopped in the long dead end with all LEDs on.
  if (finished) {
    stopServos();
    setLeds(HIGH, HIGH, HIGH);
    return;
  }

  // STEP 1: look at the walls of this cell.
  // (The side walls are normally read while driving in; at the start, read them now.)
  if (!sidesAlreadyRead) readSideWalls();
  bool wallFront = seesWallInFront();
  setLeds(wallLeft ? HIGH : LOW, wallFront ? HIGH : LOW, wallRight ? HIGH : LOW);

  // STEP 2: cross out every map position that doesn't match these walls.
  keepMatchingGuesses(wallLeft, wallFront, wallRight);
  printStatus(wallFront);

  // STEP 3: are we at the finish? (We know where we are, it's the finish cell,
  // and there are walls left, front and right.)
  int row, col, heading;
  if (getOnlyGuess(row, col, heading) && DIST_TO_FINISH[row][col] == 0
      && wallLeft && wallFront && wallRight) {
    finished = true;
    stopServos();
    Serial.println("FINISH!");
    return;
  }

  // STEP 4: choose which way to go and turn that way.
  int turn = chooseTurn(wallFront);
  turnRobot(turn);
  turnGuesses(turn);

  // STEP 5: drive one cell. If there is a wall in front after turning
  // (a misread), don't drive: go back to step 1 and look again from here.
  if (seesWallInFront()) {
    sidesAlreadyRead = false;
    return;
  }
  driveOneCell(); // Also reads the side walls of the new cell.
  moveGuessesForward();
}

// ---------- Choosing where to go ----------

// Returns the turn to make: 0 = straight, 1 = right, 2 = turn around, 3 = left.
int chooseTurn(bool wallFront) {
  int row, col, heading;
  if (getOnlyGuess(row, col, heading)) {
    // We know where we are: go to a neighbour that is closer to the finish.
    // Try straight first (no turning is fastest), then right, left, behind.
    int tryOrder[4] = {0, 1, 3, 2};
    for (int i = 0; i < 4; i++) {
      int turn = tryOrder[i];
      int dir = (heading + turn) % 4;
      if (mapHasWall(row, col, dir)) continue;
      int nextRow = row + ROW_STEP[dir];
      int nextCol = col + COL_STEP[dir];
      if (DIST_TO_FINISH[nextRow][nextCol] < DIST_TO_FINISH[row][col]) return turn;
    }
  }

  // Not sure where we are yet: right-hand rule (right, straight, left, turn around).
  if (!wallRight) return 1;
  if (!wallFront) return 0;
  if (!wallLeft) return 3;
  return 2;
}

// ---------- Working out where we are on the map ----------

bool mapHasWall(int row, int col, int dir) {
  return WALLS[row][col] & (1 << dir);
}

// Forget everything: the robot could be anywhere, facing any way.
void startGuessingAgain() {
  for (int row = 0; row < MAZE_SIZE; row++)
    for (int col = 0; col < MAZE_SIZE; col++)
      for (int h = 0; h < 4; h++)
        possible[row][col][h] = true;
}

// Cross out positions where the map's walls don't match what the robot sees.
// If nothing is left (a misread wall), start guessing again from scratch.
void keepMatchingGuesses(bool seenLeft, bool seenFront, bool seenRight) {
  for (int attempt = 0; attempt < 2; attempt++) {
    int left = 0;
    for (int row = 0; row < MAZE_SIZE; row++)
      for (int col = 0; col < MAZE_SIZE; col++)
        for (int h = 0; h < 4; h++) {
          if (!possible[row][col][h]) continue;
          bool matches = mapHasWall(row, col, (h + 3) % 4) == seenLeft
                      && mapHasWall(row, col, h) == seenFront
                      && mapHasWall(row, col, (h + 1) % 4) == seenRight;
          possible[row][col][h] = matches;
          if (matches) left++;
        }
    if (left > 0) return;
    Serial.println("No match on the map - starting again");
    startGuessingAgain();
  }
}

// The robot turned, so every guess turns the same way.
void turnGuesses(int turn) {
  bool next[MAZE_SIZE][MAZE_SIZE][4];
  for (int row = 0; row < MAZE_SIZE; row++)
    for (int col = 0; col < MAZE_SIZE; col++)
      for (int h = 0; h < 4; h++)
        next[row][col][(h + turn) % 4] = possible[row][col][h];
  memcpy(possible, next, sizeof(possible));
}

// The robot drove one cell forward, so every guess moves one cell forward.
// Guesses that would have driven through a wall are crossed out.
void moveGuessesForward() {
  bool next[MAZE_SIZE][MAZE_SIZE][4] = {};
  int left = 0;
  for (int row = 0; row < MAZE_SIZE; row++)
    for (int col = 0; col < MAZE_SIZE; col++)
      for (int h = 0; h < 4; h++) {
        if (!possible[row][col][h] || mapHasWall(row, col, h)) continue;
        next[row + ROW_STEP[h]][col + COL_STEP[h]][h] = true;
        left++;
      }
  memcpy(possible, next, sizeof(possible));
  if (left == 0) startGuessingAgain();
}

// True if exactly one position is left; it is returned in row, col, heading.
bool getOnlyGuess(int &row, int &col, int &heading) {
  int count = 0;
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++)
      for (int h = 0; h < 4; h++)
        if (possible[r][c][h]) {
          count++;
          row = r;
          col = c;
          heading = h;
        }
  return count == 1;
}

int countGuesses() {
  int count = 0;
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++)
      for (int h = 0; h < 4; h++)
        if (possible[r][c][h]) count++;
  return count;
}

// ---------- Moving ----------

// Turn on the spot: 1 = right 90, 2 = around 180, 3 = left 90 (0 = no turn).
void turnRobot(int turn) {
  if (turn == 0) return;
  if (turn == 3) {
    setWheels(L_CCW, R_CCW);
    delay((unsigned long)((PI * WHEEL_TRACK_CM * 90.0 / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 950));
  } else {
    float angle = (turn == 2) ? 180.0 : 90.0;
    setWheels(L_CW, R_CW);
    delay((unsigned long)((PI * WHEEL_TRACK_CM * angle / (360.0 * CLOCKWISE_SPEED_CM_S)) * 950));
  }
  stopServos();
  delay(100);
}

// Drive forward one cell (27 cm), keeping centred between the side walls.
// The side walls of the new cell are read when the sensors pass its middle.
void driveOneCell() {
  unsigned long driveTime = (unsigned long)(CELL_CM / FORWARD_SPEED_CM_S * 1000);
  unsigned long readTime = (unsigned long)((CELL_CM - SENSOR_AHEAD_CM) / FORWARD_SPEED_CM_S * 1000);
  sidesAlreadyRead = false;
  unsigned long startTime = millis();

  while (millis() - startTime < driveTime) {
    int distMid   = irDistance(irLedPinMid, irReceiverPinMid);
    int distLeft  = irDistance(irLedPinLeft, irReceiverPinLeft);
    int distRight = irDistance(irLedPinRight, irReceiverPinRight);
    if (distMid <= FRONT_STOP_ZONE) break; // Too close to the wall ahead: stop now.

    // Same steering as Extension 1: balance between the walls when both are seen.
    int correction = 0;
    if (distLeft < 5 && distRight < 5) {
      correction = constrain((distRight - distLeft) * STEERING_US_PER_ZONE,
                             -MAX_STEERING_US, MAX_STEERING_US);
    }
    setWheels(L_FWD + correction, R_FWD + correction);

    if (!sidesAlreadyRead && millis() - startTime >= readTime) {
      readSideWalls(); // Wheels keep turning while this reads.
      sidesAlreadyRead = true;
    }
  }
  stopServos();

  // If there is a wall ahead, creep to the same distance from it (zone 2) every
  // time, so small errors in the cell distance don't add up.
  unsigned long alignStart = millis();
  while (millis() - alignStart < 800) {
    int distMid = irDistance(irLedPinMid, irReceiverPinMid);
    if (distMid > FRONT_WALL_ZONE || distMid == 2) break; // No wall ahead, or already at zone 2.
    if (distMid > 2) setWheels(L_FWD, R_FWD);
    else setWheels(L_REV, R_REV);
  }
  stopServos();
  delay(100);
}

// ---------- Sensing ----------
// Each check reads 3 times and goes with the majority, so one bad reading doesn't count.

bool seesWallInFront() {
  int votes = 0;
  for (int i = 0; i < 3; i++) {
    if (irDistance(irLedPinMid, irReceiverPinMid) <= FRONT_WALL_ZONE) votes++;
  }
  return votes >= 2;
}

void readSideWalls() {
  int leftVotes = 0, rightVotes = 0;
  for (int i = 0; i < 3; i++) {
    if (irDistance(irLedPinLeft, irReceiverPinLeft) <= SIDE_WALL_ZONE) leftVotes++;
    if (irDistance(irLedPinRight, irReceiverPinRight) <= SIDE_WALL_ZONE) rightVotes++;
  }
  wallLeft = leftVotes >= 2;
  wallRight = rightVotes >= 2;
}

void printStatus(bool wallFront) {
  Serial.print("Walls L/F/R: ");
  Serial.print(wallLeft);
  Serial.print(wallFront);
  Serial.print(wallRight);
  Serial.print("   possible places: ");
  Serial.print(countGuesses());
  int row, col, heading;
  if (getOnlyGuess(row, col, heading)) {
    Serial.print("   I am at row ");
    Serial.print(row);
    Serial.print(", column ");
    Serial.print(col);
  }
  Serial.println();
}

// ---------- Helper functions (same as Extension 1) ----------

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
