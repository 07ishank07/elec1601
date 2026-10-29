// Combined Extensions 1 and 2: startup alignment, right-hand navigation, long-end stop.

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
const float REVERSE_SPEED_CM_S = 5.89f; // Measure reverse speed separately if needed.
// Mean wheel travel speed while spinning, NOT the speed of the robot's centre.
// Initial estimates from the previous ~1.12 s quarter-turn and 12 cm track.
const float CLOCKWISE_SPEED_CM_S = 8.41f;
const float ANTICLOCKWISE_SPEED_CM_S = 8.41f;
const float TILT_ANGLE_DEG = 13.0f;
const unsigned long LEFT_START_TIMEOUT_MS = 2000; // Maximum centering attempt, either side.

// Long dead end check. Maze: 200 mm wall panels joined by 70 mm timber posts.
// A straight is measured from where both side walls begin to the stop ~5 cm
// before the end wall: 1 wall ~15 cm, 2 walls ~42 cm, 3 walls ~69 cm.
// The window sits halfway between, to allow for speed and sensor fluctuation.
const float LONG_MIN_CM = 28.0f;
const float LONG_MAX_CM = 55.0f;
const bool USE_JOIN_COUNT = false; // true: also require exactly one timber join (2 walls = 1 middle post). Enable only once Serial shows joins = 1 in a long dead end.
const int WEAK_ZONE = 5;          // Side reading at/above this = timber post or gap. Try 4 if wood reads 4.
const float JOIN_MAX_CM = 15.0f;  // Weak stretch up to this = 7 cm post; longer = side opening.
const float MOUTH_CM = 3.0f;      // Ignore posts right at the start of a straight (corridor entrance).

bool wasDrivingStraight = false;
unsigned long lastLoopMs = 0;
unsigned long straightMs = 0; // Forward driving time along the current straight.
unsigned long gapMs = 0;      // Forward driving time in the current weak side stretch.
bool gapBothSides = false;    // Both sides went weak during this stretch (a post pair).
bool gapAtMouth = false;      // Stretch began at the start of the straight.
int joinCount = 0;            // Timber post pairs passed along the current straight.
bool finished = false;        // Reached the long dead end.

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

const unsigned long START_ALIGNMENT_MS = 7000;
bool movementStarted = false;
unsigned long movementStartMs = 0;
byte wigglePhase = 0; // Right, centre, left, centre; repeat while unknown.

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

  stopServos();
  delay(5000);
  lastLoopMs = millis(); // Do not count the start-up wait as driving time.
}

void loop() {
  if (finished) { // Stay stopped at the end of the long dead end.
    stopServos();
    setLeds(HIGH, HIGH, HIGH);
    return;
  }

  // Measure left, middle, and right distances
  // dist is 0-5: 5 = farthest from wall, 0 = closest
  int distMid   = irDistance(irLedPinMid, irReceiverPinMid);
  int distLeft  = irDistance(irLedPinLeft, irReceiverPinLeft);
  int distRight = irDistance(irLedPinRight, irReceiverPinRight);
  unsigned long now = millis();
  unsigned long dt = wasDrivingStraight ? now - lastLoopMs : 0;
  lastLoopMs = now;
  // Account for travel before classifying the newly observed scenario.
  if (wasDrivingStraight) trackStraight(distLeft, distRight, dt);

  Serial.println(distMid);

  // 20 cm clear corridor, approximately 8.5 cm between side sensor positions:
  // centred side clearance is about 5.75 cm (zone 3). Straight travel compares
  // the sides within two zones, including 5/5 when wood joins are not detected.
  // An opening on ONE side with a clearly detected opposite wall takes turn
  // priority, even with a clear front (zone 5). 4/5 remains straight travel.
  // Steer gently in case 1; use a separate sideways shift at zone 0 (<= 1.5 cm).
  // After correction, require both sides to reach at least zone 2 before
  // finishing, so small zone fluctuations do not immediately retrigger it.
  // This is a practical tolerance, not a precise measurement of centering.
  // Zone 4 still detects a wall; only 5 means no detection.
  // Zones 0/1/2/3/4 end at 1.5/3/5/10/15 cm.
  // Right-hand rule: right turn first, then straight, then left, then turn around.
  // Straight also covers passing a left opening (left 5, right wall detected).
  // Right turn also covers a T-junction (front wall, both sides open).
  // Left turn only happens once the front is blocked.
  int scenario = 0; // Unknown: wiggle briefly and recheck.
  if (distMid >= 3 && (abs(distLeft - distRight) <= 2 || distLeft == 5)) scenario = 1;
  if (distMid >= 1 && distRight == 5 && ((distLeft >= 1 && distLeft <= 3) || (distLeft == 5 && distMid <= 2))) scenario = 2;
  if (distMid >= 1 && distMid <= 2 && distRight >= 1 && distRight <= 3 && distLeft == 5) scenario = 3;
  if (distMid <= 2 && distLeft >= 1 && distLeft <= 4 && distRight >= 1 && distRight <= 4 && (distLeft <= 3 || distRight <= 3)) scenario = 4;
  if (distMid >= 4 && distLeft == 0 && distRight >= 3 && distRight <= 4) scenario = 5;
  if (distMid >= 4 && distRight == 0 && distLeft >= 3 && distLeft <= 4) scenario = 6;
  // Startup alignment only: first seven seconds after the first wheel command.
  bool allowStartupAlignment = !movementStarted || millis() - movementStartMs < START_ALIGNMENT_MS;
  if (allowStartupAlignment && distLeft <= 1 && distMid >= distLeft && distMid <= 5 && distRight >= 3 && distRight <= 5) scenario = 7;  
  if (allowStartupAlignment && distRight <= 1 && distMid >= distRight && distMid <= 5 && distLeft >= 3 && distLeft <= 5) scenario = 8;

  // Case 1 leaves the wheels running between loop iterations. Stop as soon
  // as any different case is selected, before its checks or movement begin.
  if (scenario != 1) stopServos();

  // A single noisy side reading must not start a recentering manoeuvre.
  // Require the initial reading plus two fresh matching readings while still.
  if (scenario == 5 || scenario == 6) {
    for (int sample = 0; sample < 2; ++sample) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if ((scenario == 5 && !(dM >= 4 && dL == 0 && dR >= 3 && dR <= 4))
          || (scenario == 6 && !(dM >= 4 && dR == 0 && dL >= 3 && dL <= 4))) {
        scenario = 0; // Stay still and reclassify on the next loop.
        break;
      }
    }
  }

  /*  Motion control
    Recheck after every movement. All cases are enabled; case 0 wiggles.
    Only one scenario procedure is executed per loop. If tests overlap,
    the higher scenario number has priority.
  */
  if (scenario != 0) wigglePhase = 0;

  switch (scenario)
  {
    case 0: // Unknown: small alternating rotations, sampling between each pulse.
      setLeds(LOW, LOW, LOW);
      // Approximate offsets from entry heading: right, centre, left, centre.
      // No forward drive. Equal pulse times assume similar CW/CCW turn speeds.
      if (wigglePhase == 0 || wigglePhase == 3) setWheels(L_CW, R_CW);
      else setWheels(L_CCW, R_CCW);
      delay(60);
      stopServos();
      wigglePhase = (wigglePhase + 1) % 4;
      break; // loop() reads all three sensors before another pulse.

    case 1: // Keep going straight; loop() continuously checks for another case.
    {
      setLeds(LOW, LOW, HIGH);
      int correction = 0;
      if (distLeft < 5 && distRight < 5) { // Only balance when both walls are detected.
        correction = constrain((distRight - distLeft) * STEERING_US_PER_ZONE,
                               -MAX_STEERING_US, MAX_STEERING_US);
      } else if (distLeft == 5 && distRight < 5) { // Passing a left opening: hold zone 3 off the right wall.
        correction = constrain((distRight - 3) * STEERING_US_PER_ZONE,
                               -MAX_STEERING_US, MAX_STEERING_US);
      }
      // Positive correction steers right: left faster, right less negative/slower.
      setWheels(L_FWD + correction, R_FWD + correction);
      break; // No delay or stop: read the sensors again immediately.
    }

    case 2: // Right 90 degrees, then forward approximately 5 cm.
    {
      setLeds(LOW, HIGH, LOW);
      // A right opening takes priority even with clear space ahead.
      // Advance into the opening, monitoring the front instead of driving blind.
      unsigned long approachStart = millis();
      while (distMid >= 3 && millis() - approachStart < 1500) {
        if (irDistance(irLedPinMid, irReceiverPinMid) <= 2) break;
        setWheels(L_FWD, R_FWD);
      }
      stopServos();

      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 90.0 / (360.0 * CLOCKWISE_SPEED_CM_S)) * 900));
      stopServos();
      delay(100);

      setWheels(L_FWD, R_FWD);
      delay((unsigned long)((5.0 / FORWARD_SPEED_CM_S) * 1000));
      stopServos();
      break;
    }

    case 3: // Mirror of case 2: left 90 degrees, then forward 5 cm.
    {
      setLeds(LOW, HIGH, HIGH);
      // Case 1 already approached the corner while checking sensors.
      
      setWheels(L_CCW, R_CCW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 90.0 / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 800));
      stopServos();
      delay(100);

      setWheels(L_FWD, R_FWD);
      delay((unsigned long)((5.0 / FORWARD_SPEED_CM_S) * 1000));
      stopServos();
      break;
    }

    case 4: // Dead end: stop if it is the long one, otherwise turn 180 degrees.
    {
      float straightCm = straightMs / 1000.0 * FORWARD_SPEED_CM_S;
      Serial.print("Dead end: straight cm = ");
      Serial.print(straightCm);
      Serial.print(", joins = ");
      Serial.println(joinCount);

      // Require the calibrated distance window and, if enabled, one timber join.
      // Right-wall following is not guaranteed to revisit every branch in a looped maze.
      bool isLong = straightCm >= LONG_MIN_CM && straightCm <= LONG_MAX_CM
                    && (!USE_JOIN_COUNT || joinCount == 1);
      for (int sample = 0; isLong && sample < 2; ++sample) { // Confirm it is really a dead end.
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        if (!(dM <= 2 && dL >= 1 && dL <= 4 && dR >= 1 && dR <= 4)) isLong = false;
      }
      if (isLong) {
        finished = true; // Facing forward at the end; loop() keeps the robot stopped.
        break;
      }

      setLeds(HIGH, LOW, LOW);
      delay(100);
      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 180.0 / (360.0 * CLOCKWISE_SPEED_CM_S)) * 950));
      stopServos();
      break;
    }

    case 5: // Shift away from the left wall, straighten, then return level with the start.
    {
      setLeds(HIGH, LOW, HIGH);
      delay(100);
      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * TILT_ANGLE_DEG / (360.0 * CLOCKWISE_SPEED_CM_S)) * 1000));
      stopServos();

      unsigned long startTime = millis();
      unsigned long forwardStart = 0;
      bool movedForward = false;
      int centredReadings = 0;
      while (millis() - startTime < LEFT_START_TIMEOUT_MS) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        if (dM == 0) break;
        // Finish farther from the wall than the zone-0 trigger threshold.
        if (dL >= 2 && dL <= 4 && dR >= 2 && dR <= 4 && (dL <= 3 || dR <= 3)) {
          if (++centredReadings >= 3) break;
        } else {
          centredReadings = 0;
        }
        if (!movedForward) {
          forwardStart = millis();
          movedForward = true;
        }
        setWheels(L_FWD, R_FWD);
      }
      stopServos();
      unsigned long forwardTime = movedForward ? millis() - forwardStart : 0;
      delay(100);
      setWheels(L_CCW, R_CCW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * TILT_ANGLE_DEG / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 1000)); // Tilt back.
      stopServos();

      if (centredReadings >= 3 && movedForward) {
        delay(100);
        setWheels(L_REV, R_REV);
        // Return along the corridor by the forward component of the diagonal:
        // reverse time = forward time * (forward speed / reverse speed) * cos(tilt).
        // forwardTime is already in milliseconds. Assumes clear space behind.
        delay((unsigned long)(forwardTime * FORWARD_SPEED_CM_S / REVERSE_SPEED_CM_S
                              * cos(TILT_ANGLE_DEG * PI / 180.0)));
        stopServos();
      }
      break;
    }

    case 6: // Shift away from the right wall, straighten, then return level with the start.
    {
      setLeds(HIGH, HIGH, LOW);
      delay(100);
      setWheels(L_CCW, R_CCW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * TILT_ANGLE_DEG / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 1000));
      stopServos();

      unsigned long startTime = millis();
      unsigned long forwardStart = 0;
      bool movedForward = false;
      int centredReadings = 0;
      while (millis() - startTime < LEFT_START_TIMEOUT_MS) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        if (dM == 0) break;
        // Finish farther from the wall than the zone-0 trigger threshold.
        if (dL >= 2 && dL <= 4 && dR >= 2 && dR <= 4 && (dL <= 3 || dR <= 3)) {
          if (++centredReadings >= 3) break;
        } else {
          centredReadings = 0;
        }
        if (!movedForward) {
          forwardStart = millis();
          movedForward = true;
        }
        setWheels(L_FWD, R_FWD);
      }
      stopServos();
      unsigned long forwardTime = movedForward ? millis() - forwardStart : 0;
      delay(100);
      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * TILT_ANGLE_DEG / (360.0 * CLOCKWISE_SPEED_CM_S)) * 1000)); // Tilt back.
      stopServos();

      if (centredReadings >= 3 && movedForward) {
        delay(100);
        setWheels(L_REV, R_REV);
        // Return along the corridor by the forward component of the diagonal:
        // reverse time = forward time * (forward speed / reverse speed) * cos(tilt).
        // forwardTime is already in milliseconds. Assumes clear space behind.
        delay((unsigned long)(forwardTime * FORWARD_SPEED_CM_S / REVERSE_SPEED_CM_S
                              * cos(TILT_ANGLE_DEG * PI / 180.0)));
        stopServos();
      }
      break;
    }

    case 7: // Approximate wall alignment; zone sensors cannot prove parallelism.
    {
      setLeds(HIGH, HIGH, HIGH);
      unsigned long turnTime = 0;
      unsigned long turnLimit = (unsigned long)((PI * WHEEL_TRACK_CM * 45.0 / (360.0 * CLOCKWISE_SPEED_CM_S)) * 1000);
      int alignedReadings = 0;
      // Short rotation steps, with stationary measurements between steps.
      // Require both walls, a clear front and clearance on both sides.
      // This is a heuristic; a parallel but off-centre robot can also fail it.
      while (turnTime < turnLimit && (!movementStarted || millis() - movementStartMs < START_ALIGNMENT_MS)) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        if (dM >= 4 && dL >= 2 && dL <= 4 && dR >= 2 && dR <= 4
            && (dL <= 3 || dR <= 3)) {
          if (++alignedReadings >= 3) break;
          continue; // Confirm without rotating farther.
        }
        alignedReadings = 0;
        unsigned long pulseMs = turnLimit - turnTime;
        if (pulseMs > 40) pulseMs = 40;
        if (movementStarted) {
          unsigned long elapsed = millis() - movementStartMs;
          if (elapsed >= START_ALIGNMENT_MS) break;
          if (pulseMs > START_ALIGNMENT_MS - elapsed) pulseMs = START_ALIGNMENT_MS - elapsed;
        }
        setWheels(L_CW, R_CW);
        delay(pulseMs);
        stopServos();
        turnTime += pulseMs;
      }
      stopServos(); // Timeout means unconfirmed alignment; next loop reclassifies.
      break;
    }

    case 8: // Approximate wall alignment; zone sensors cannot prove parallelism.
    {
      setLeds(LOW, LOW, HIGH);
      unsigned long turnTime = 0;
      unsigned long turnLimit = (unsigned long)((PI * WHEEL_TRACK_CM * 45.0 / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 1000);
      int alignedReadings = 0;
      // Short rotation steps, with stationary measurements between steps.
      // Require both walls, a clear front and clearance on both sides.
      // This is a heuristic; a parallel but off-centre robot can also fail it.
      while (turnTime < turnLimit && (!movementStarted || millis() - movementStartMs < START_ALIGNMENT_MS)) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        if (dM >= 4 && dL >= 2 && dL <= 4 && dR >= 2 && dR <= 4
            && (dL <= 3 || dR <= 3)) {
          if (++alignedReadings >= 3) break;
          continue; // Confirm without rotating farther.
        }
        alignedReadings = 0;
        unsigned long pulseMs = turnLimit - turnTime;
        if (pulseMs > 40) pulseMs = 40;
        if (movementStarted) {
          unsigned long elapsed = millis() - movementStartMs;
          if (elapsed >= START_ALIGNMENT_MS) break;
          if (pulseMs > START_ALIGNMENT_MS - elapsed) pulseMs = START_ALIGNMENT_MS - elapsed;
        }
        setWheels(L_CCW, R_CCW);
        delay(pulseMs);
        stopServos();
        turnTime += pulseMs;
      }
      stopServos(); // Timeout means unconfirmed alignment; next loop reclassifies.
      break;
    }

    default:
      setLeds(LOW, LOW, LOW);
      stopServos();
      break;
  }

  // Only navigation turns start a new straight; wiggles and alignment do not.
  // Cases 5/6 approximately cancel longitudinal motion by reversing afterwards.
  if (scenario >= 2 && scenario <= 4) {
    straightMs = 0;
    gapMs = 0;
    gapBothSides = false;
    joinCount = 0;
  }
  wasDrivingStraight = scenario == 1;
  // Continuous case 1 timing starts at its first command, then spans whole loops.
  // Other cases have stopped by this point, so exclude their blocking delays.
  if (!wasDrivingStraight || dt == 0) lastLoopMs = millis();
}

// Measure the current straight and count timber post pairs along it.
// A short weak stretch on both sides is a post; a long one is a side opening,
// which restarts the straight. Called only while driving forward (case 1).
void trackStraight(int distLeft, int distRight, unsigned long dt) {
  bool leftWeak = distLeft >= WEAK_ZONE;
  bool rightWeak = distRight >= WEAK_ZONE;
  if (leftWeak || rightWeak) {
    if (gapMs == 0) gapAtMouth = straightMs / 1000.0 * FORWARD_SPEED_CM_S < MOUTH_CM;
    gapMs += dt;
    if (leftWeak && rightWeak) gapBothSides = true;
    if (gapMs / 1000.0 * FORWARD_SPEED_CM_S > JOIN_MAX_CM) { // An opening, not a post.
      straightMs = 0;
      joinCount = 0;
      return;
    }
  } else if (gapMs > 0) { // Weak stretch just ended.
    if (gapBothSides && !gapAtMouth && gapMs / 1000.0 * FORWARD_SPEED_CM_S <= JOIN_MAX_CM) joinCount++;
    gapMs = 0;
    gapBothSides = false;
  }
  straightMs += dt;
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
  if (!movementStarted && (leftOffset != 0 || rightOffset != 0)) {
    movementStartMs = millis();
    movementStarted = true;
  }
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
