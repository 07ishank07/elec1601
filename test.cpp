// Sensor-board-only tester for counting timber joins along a corridor.
// No servos: slide the robot/board along the corridor by hand at a steady pace.
// Open Serial Monitor at 9600 baud.
//
// Output:
//   t,L,M,R,joins       one line per loop (set PRINT_EVERY_LOOP = false to hide)
//   # ...               event lines (join, opening, dead-end verdict)
// LEDs: left/right red LED = that side currently reads "weak" (wood or gap),
//       middle red LED = join count is exactly 1 (would be a long dead end).
// Send 'r' over Serial to reset the join count manually.

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

// ---------- PLACEHOLDERS: tune from the logged readings ----------
const int  WEAK_ZONE = 5;            // Side reading at/above this = wood or gap. Try 4 if wood reads 4.
const int  JOIN_MAX_SAMPLES = 15;    // Weak run this short or shorter = timber join.
const int  OPEN_MIN_SAMPLES = 25;    // Weak run this long or longer = real opening (resets count).
const unsigned long PAIR_WINDOW_MS = 600; // Left and right joins this close together = same join.
const bool REQUIRE_PAIR = true;      // true: count a join only when BOTH sides see it.
const int  FRONT_WALL_ZONE = 2;      // Middle reading at/below this = wall in front.
const int  DEAD_END_SAMPLES = 3;     // Consecutive dead-end readings before giving a verdict.
const bool PRINT_EVERY_LOOP = true;
// ------------------------------------------------------------------

int joinCount = 0;

// Per side: index 0 = left, 1 = right
int weakRun[2] = {0, 0};             // Consecutive weak samples so far
unsigned long lastJoinMs[2] = {0, 0}; // When that side's last unpaired join ended (0 = none)

int deadEndRun = 0;
bool verdictGiven = false;

int irDetect(int irLedPin, int irReceiverPin, long frequency);
int irDistance(int irLedPin, int irReceiverPin);
void setLeds(int left, int mid, int right);
void updateSide(int side, int dist);
void onJoin(int side);
void resetCount(const char *reason);

void setup() {
  Serial.begin(9600);

  pinMode(irReceiverPinRight, INPUT);
  pinMode(irReceiverPinMid, INPUT);
  pinMode(irReceiverPinLeft, INPUT);

  pinMode(irLedPinRight, OUTPUT);
  pinMode(redLedPinRight, OUTPUT);
  pinMode(irLedPinMid, OUTPUT);
  pinMode(redLedPinMid, OUTPUT);
  pinMode(irLedPinLeft, OUTPUT);
  pinMode(redLedPinLeft, OUTPUT);

  Serial.println("# join counter test ready");
  Serial.println("t,L,M,R,joins");
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') resetCount("manual");

  // dist is 0-5: 5 = farthest from wall, 0 = closest
  int distMid   = irDistance(irLedPinMid, irReceiverPinMid);
  int distLeft  = irDistance(irLedPinLeft, irReceiverPinLeft);
  int distRight = irDistance(irLedPinRight, irReceiverPinRight);

  updateSide(0, distLeft);
  updateSide(1, distRight);

  // Dead end: wall in front and solid walls both sides, held for a few samples.
  bool deadEnd = distMid <= FRONT_WALL_ZONE && distLeft < WEAK_ZONE && distRight < WEAK_ZONE;
  deadEndRun = deadEnd ? deadEndRun + 1 : 0;
  if (deadEndRun >= DEAD_END_SAMPLES && !verdictGiven) {
    Serial.print("# DEAD END: joins=");
    Serial.print(joinCount);
    Serial.println(joinCount == 1 ? " -> LONG (stop)" : " -> SHORT (turn around)");
    verdictGiven = true;
  }
  if (distMid > FRONT_WALL_ZONE) verdictGiven = false; // Re-arm once the front clears.

  setLeds(distLeft >= WEAK_ZONE ? HIGH : LOW,
          joinCount == 1 ? HIGH : LOW,
          distRight >= WEAK_ZONE ? HIGH : LOW);

  if (PRINT_EVERY_LOOP) {
    Serial.print(millis());
    Serial.print(',');
    Serial.print(distLeft);
    Serial.print(',');
    Serial.print(distMid);
    Serial.print(',');
    Serial.print(distRight);
    Serial.print(',');
    Serial.println(joinCount);
  }
}

// ---------- Join detection ----------

// Track how long a side stays weak. When it becomes strong again, the run
// length decides: short = timber join, long = opening, in between = ignored.
void updateSide(int side, int dist) {
  if (dist >= WEAK_ZONE) {
    weakRun[side]++;
    if (weakRun[side] == OPEN_MIN_SAMPLES) {
      resetCount(side == 0 ? "left opening" : "right opening");
    }
    return;
  }

  int run = weakRun[side];
  weakRun[side] = 0;
  if (run > 0 && run <= JOIN_MAX_SAMPLES) onJoin(side);
}

void onJoin(int side) {
  int other = 1 - side;
  unsigned long now = millis();

  Serial.print("# join seen on ");
  Serial.println(side == 0 ? "left" : "right");

  // The other side saw a join recently: treat both as the same timber post.
  if (lastJoinMs[other] != 0 && now - lastJoinMs[other] <= PAIR_WINDOW_MS) {
    lastJoinMs[other] = 0;
    if (REQUIRE_PAIR) {
      joinCount++;
      Serial.print("# paired join -> count=");
      Serial.println(joinCount);
    }
    return;
  }

  lastJoinMs[side] = now;
  if (!REQUIRE_PAIR) {
    joinCount++;
    Serial.print("# join -> count=");
    Serial.println(joinCount);
  }
}

void resetCount(const char *reason) {
  joinCount = 0;
  lastJoinMs[0] = 0;
  lastJoinMs[1] = 0;
  Serial.print("# reset (");
  Serial.print(reason);
  Serial.println(")");
}

// ---------- Helper functions ----------

void setLeds(int left, int mid, int right) {
  digitalWrite(redLedPinLeft, left);
  digitalWrite(redLedPinMid, mid);
  digitalWrite(redLedPinRight, right);
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
