// Separate experimental copy of the supplied sketch; originals are unchanged.

#include <Arduino.h>
#include <Servo.h> // Include servo library

void trackStraight();
void stopServos();
void setLeds(int left, int mid, int right);
void setWheels(int leftOffset, int rightOffset);
int irDetect(int irLedPin, int irReceiverPin, long frequency);
int irDistance(int irLedPin, int irReceiverPin);

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

// Experimental right-hand navigation; calibrate these distances on the actual maze.
// These limits assume the corridor reference and end clearance used during testing.
const float LONG_MIN_CM = 28.0f;
const float LONG_MAX_CM = 55.0f;
const float OPENING_CONFIRM_CM = 10.0f; // Must exceed the apparent timber-post width.
const unsigned long OPENING_CONFIRM_MS = 120; // Reject isolated sensor noise.
float straightCm = 0;
float forwardTravelCm = 0;
float trackedSpeed = 0; // Signed longitudinal speed; zero while stopped/spinning.
float headingProjection = 1.0f;
unsigned long motionUpdatedMs = 0;
bool straightKnown = false; // Starting midway along a corridor is not a known length.
bool awaitingCorridor = false;
bool leftGap = false, rightGap = false;
bool leftOpening = false, rightOpening = false;
float leftGapStartCm = 0, rightGapStartCm = 0;
unsigned long leftGapStartMs = 0, rightGapStartMs = 0;
bool finished = false;

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

void setup() {
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
  motionUpdatedMs = millis();
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
  trackStraight(); // Account for the previous motor command, including sensor-read time.

  // A turn establishes a new straight only once both side walls are visible.
  if (awaitingCorridor && distLeft < 5 && distRight < 5) {
    straightCm = 0;
    straightKnown = true;
    awaitingCorridor = false;
  }
  // Qualify gaps before using them as junctions. A moving robot must traverse
  // more than a post width. At a blocked front, stable readings permit a turn
  // without driving farther into the end wall.
  if (distLeft == 5) {
    if (!leftGap) { leftGap = true; leftGapStartCm = forwardTravelCm; leftGapStartMs = millis(); }
    leftOpening = millis() - leftGapStartMs >= OPENING_CONFIRM_MS
                  && (forwardTravelCm - leftGapStartCm >= OPENING_CONFIRM_CM || distMid <= 2);
  } else {
    if (leftOpening) { straightCm = 0; straightKnown = true; awaitingCorridor = false; }
    leftGap = false; leftOpening = false;
  }
  if (distRight == 5) {
    if (!rightGap) { rightGap = true; rightGapStartCm = forwardTravelCm; rightGapStartMs = millis(); }
    rightOpening = millis() - rightGapStartMs >= OPENING_CONFIRM_MS
                   && (forwardTravelCm - rightGapStartCm >= OPENING_CONFIRM_CM || distMid <= 2);
  } else {
    if (rightOpening) { straightCm = 0; straightKnown = true; awaitingCorridor = false; }
    rightGap = false; rightOpening = false;
  }
  if (leftOpening || rightOpening) straightKnown = false;

  // 20 cm clear corridor, approximately 8.5 cm between side sensor positions:
  // centred side clearance is about 5.75 cm (zone 3). Straight travel compares
  // the sides within two zones, including 5/5 when wood joins are not detected.
  // An opening on ONE side with a clearly detected opposite wall takes turn
  // priority after confirmation, even with a clear front (zone 5).
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
  int scenario = 0; // Unknown: stop the wheels, then check again next loop.
  if (distMid >= 3 && (abs(distLeft - distRight) <= 2 || distLeft == 5)) scenario = 1;
  if (distMid >= 3 && (leftGap || rightGap) && distLeft >= 1 && distRight >= 1) scenario = 1;
  if (distMid >= 1 && rightOpening && distLeft >= 1) scenario = 2;
  if (distMid >= 1 && distMid <= 2 && distRight >= 1 && distRight <= 4 && leftOpening && !rightOpening) scenario = 3;
  if (distMid <= 2 && distLeft >= 1 && distLeft <= 4 && distRight >= 1 && distRight <= 4 && (distLeft <= 3 || distRight <= 3)) scenario = 4;
  if (distMid >= 4 && distLeft == 0 && distRight >= 3 && distRight <= 4) scenario = 5;
  if (distMid >= 4 && distRight == 0 && distLeft >= 3 && distLeft <= 4) scenario = 6;
  if (distLeft <= 1 && distMid >= distLeft && distMid <= 3 && distRight >= 3 && distRight <= 4) scenario = 7;  
  if (distRight <= 1 && distMid >= distRight && distMid <= 3 && distLeft >= 3 && distLeft <= 4) scenario = 8;

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
    Recheck after every movement. All cases are enabled; case 0 waits.
    Only one scenario procedure is executed per loop. If tests overlap,
    the higher scenario number has priority.
  */
  switch (scenario)
  {
    case 0: // Unknown: stop, then check again on the next loop.
      setLeds(LOW, LOW, LOW);
      stopServos();
      break;

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
      // Gap confirmation already advances along the opening. Only creep a
      // little farther, checking the front throughout instead of a blind delay.
      unsigned long approachStart = millis();
      while (millis() - approachStart < 200) {
        if (irDistance(irLedPinMid, irReceiverPinMid) <= 2) break;
        setWheels(L_FWD, R_FWD);
      }
      stopServos();

      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 90.0 / (360.0 * CLOCKWISE_SPEED_CM_S)) * 950));
      stopServos();
      delay(100);

      straightCm = 0; straightKnown = false; awaitingCorridor = true;
      leftGap = rightGap = leftOpening = rightOpening = false;
      unsigned long exitStart = millis();
      while (millis() - exitStart < (unsigned long)(5000.0 / FORWARD_SPEED_CM_S)) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        trackStraight();
        if (awaitingCorridor && dL < 5 && dR < 5) {
          straightCm = 0; straightKnown = true; awaitingCorridor = false;
        }
        if (dM <= 2) break;
        setWheels(L_FWD, R_FWD);
      }
      stopServos();
      break;
    }

    case 3: // Mirror of case 2: left 90 degrees, then forward 5 cm.
    {
      setLeds(LOW, HIGH, HIGH);
      // Gap confirmation already advances along the opening. Only creep a
      // little farther, checking the front throughout instead of a blind delay.
      unsigned long approachStart = millis();
      while (millis() - approachStart < 200) {
        if (irDistance(irLedPinMid, irReceiverPinMid) <= 2) break;
        setWheels(L_FWD, R_FWD);
      }
      stopServos();
      
      setWheels(L_CCW, R_CCW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 90.0 / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 950));
      stopServos();
      delay(100);

      straightCm = 0; straightKnown = false; awaitingCorridor = true;
      leftGap = rightGap = leftOpening = rightOpening = false;
      unsigned long exitStart = millis();
      while (millis() - exitStart < (unsigned long)(5000.0 / FORWARD_SPEED_CM_S)) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        trackStraight();
        if (awaitingCorridor && dL < 5 && dR < 5) {
          straightCm = 0; straightKnown = true; awaitingCorridor = false;
        }
        if (dM <= 2) break;
        setWheels(L_FWD, R_FWD);
      }
      stopServos();
      break;
    }

    case 4: // Dead end: stop if it is the long one, otherwise turn 180 degrees.
    {
      bool isLong = straightKnown && straightCm >= LONG_MIN_CM && straightCm <= LONG_MAX_CM;
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
      straightCm = 0; straightKnown = false; awaitingCorridor = true;
      leftGap = rightGap = leftOpening = rightOpening = false;
      break;
    }

    case 5: // Shift away from the left wall, straighten, then return level with the start.
    {
      setLeds(HIGH, LOW, HIGH);
      delay(100);
      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * TILT_ANGLE_DEG / (360.0 * CLOCKWISE_SPEED_CM_S)) * 1000));
      stopServos();

      headingProjection = cos(TILT_ANGLE_DEG * PI / 180.0);
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
      headingProjection = 1.0f;
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

      headingProjection = cos(TILT_ANGLE_DEG * PI / 180.0);
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
      headingProjection = 1.0f;
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

    case 7: // Turn clockwise until facing along the corridor, without centering.
    {
      setLeds(HIGH, HIGH, HIGH);
      delay(100);
      unsigned long startTime = millis();
      // Approximate heading check: clear front and both side walls detected.
      // IR zones cannot prove exact parallel alignment. Limit a failed turn
      // to about 45 degrees so this loop cannot spin indefinitely.
      while (millis() - startTime < (unsigned long)((PI * WHEEL_TRACK_CM * 45.0 / (360.0 * CLOCKWISE_SPEED_CM_S)) * 1000)) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        if (dM >= 4 && dL <= 4 && dR <= 4 && (dL <= 3 || dR <= 3)) break;
        setWheels(L_CW, R_CW);
      }
      stopServos(); // Next loop may select case 5 if still too far left.
      break;
    }

    case 8: // Mirror of case 7: turn anticlockwise without centering.
    {
      for (int flash = 0; flash < 2; ++flash) {
        setLeds(LOW, LOW, HIGH);
        delay(1000);
        setLeds(LOW, LOW, LOW);
        delay(1000);
      }
      unsigned long startTime = millis();
      while (millis() - startTime < (unsigned long)((PI * WHEEL_TRACK_CM * 45.0 / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 1000)) {
        int dM = irDistance(irLedPinMid, irReceiverPinMid);
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        if (dM >= 4 && dL <= 4 && dR <= 4 && (dL <= 3 || dR <= 3)) break;
        setWheels(L_CCW, R_CCW);
      }
      stopServos(); // Next loop may select case 6 if still too far right.
      break;
    }

    default:
      setLeds(LOW, LOW, LOW);
      stopServos();
      break;
  }

}

// Integrate the previous commanded motion before changing it. Calibration is
// still required: servo pulse offsets do not measure actual wheel speed.
void trackStraight() {
  unsigned long now = millis();
  float displacement = trackedSpeed * ((now - motionUpdatedMs) / 1000.0f);
  if (straightKnown) straightCm += displacement;
  if (displacement > 0) forwardTravelCm += displacement;
  motionUpdatedMs = now;
}

// ---------- Helper functions ----------

void setLeds(int left, int mid, int right) {
  digitalWrite(redLedPinLeft, left);
  digitalWrite(redLedPinMid, mid);
  digitalWrite(redLedPinRight, right);
}

void stopServos() {
  trackStraight();
  trackedSpeed = 0;
  servoLeft.writeMicroseconds(leftStop);
  servoRight.writeMicroseconds(rightStop);
}

void setWheels(int leftOffset, int rightOffset) {
  trackStraight();
  trackedSpeed = 0;
  if (leftOffset > 0 && rightOffset < 0) trackedSpeed = FORWARD_SPEED_CM_S * headingProjection;
  if (leftOffset < 0 && rightOffset > 0) trackedSpeed = -REVERSE_SPEED_CM_S * headingProjection;
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
