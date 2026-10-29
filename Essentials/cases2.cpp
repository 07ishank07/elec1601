
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

byte deadEndState = 0; // 0: none yet, 1: first completed, 2: permanently stopped.
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
}

void loop() {
  if (deadEndState == 2) {
    stopServos();
    setLeds(HIGH, LOW, LOW);
    return; // Remain stopped until reset.
  }
  // Measure left, middle, and right distances
  // dist is 0-5: 5 = farthest from wall, 0 = closest
  int distMid   = irDistance(irLedPinMid, irReceiverPinMid);
  int distLeft  = irDistance(irLedPinLeft, irReceiverPinLeft);
  int distRight = irDistance(irLedPinRight, irReceiverPinRight);

  Serial.println(distMid);

  // 20 cm clear corridor, approximately 8.5 cm between side sensor positions:
  // centred side clearance is about 5.75 cm (zone 3). Straight travel compares
  // the sides within two zones, including 5/5 when wood joins are not detected.
  // With one side open, approach while the front is zone 2-5, then turn
  // at front zone 0-1. Zone 4 on the opposite side still counts as a wall.
  // Steer gently in case 1; use a separate sideways shift at zone 0 (<= 1.5 cm).
  // After correction, require both sides to reach at least zone 2 before
  // finishing, so small zone fluctuations do not immediately retrigger it.
  // This is a practical tolerance, not a precise measurement of centering.
  // Zone 4 still detects a wall; only 5 means no detection.
  // Zones 0/1/2/3/4 end at 1.5/3/5/10/15 cm.
  int scenario = 0; // Unknown: make a small wiggle, then check again next loop.
  if (distMid >= 3 && abs(distLeft - distRight) <= 2) scenario = 1;
  // Approach a corner while the front is still farther away (zones 2-5).
  // Zone 5 on one side is an opening; zones 1-4 on the other still detect a wall.
  if (distMid >= 2 && ((distRight == 5 && distLeft >= 1 && distLeft <= 4)
                   || (distLeft == 5 && distRight >= 1 && distRight <= 4))) scenario = 1;
  // Turn at front zone 0 or 1; do not drive forward towards the wall first.
  if (distMid >= 1 && distMid <= 2 && distLeft >= 1 && distLeft <= 4 && distRight == 5) scenario = 2;
  if (distMid >= 1 && distMid <= 2 && distRight >= 1 && distRight <= 4 && distLeft == 5) scenario = 3;
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
      }
      // Positive correction steers right: left faster, right less negative/slower.
      setWheels(L_FWD + correction, R_FWD + correction);
      break; // No delay or stop: read the sensors again immediately.
    }

    case 2: // Right 90 degrees, then forward approximately 5 cm.
    {
      setLeds(LOW, HIGH, LOW);
      // Case 1 already approached the corner while checking sensors.

      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 90.0 / (360.0 * CLOCKWISE_SPEED_CM_S)) * 950));
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
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 90.0 / (360.0 * ANTICLOCKWISE_SPEED_CM_S)) * 850));
      stopServos();
      delay(100);

      setWheels(L_FWD, R_FWD);
      delay((unsigned long)((5.0 / FORWARD_SPEED_CM_S) * 1000));
      stopServos();
      break;
    }

    case 4: // U-turn at the first dead end; stop permanently at the next.
      setLeds(HIGH, LOW, LOW);
      stopServos();
      if (deadEndState == 1) {
        deadEndState = 2;
        return; // Future loop() calls return before reading any sensors.
      }
      delay(100);
      setWheels(L_CW, R_CW);
      delay((unsigned long)((PI * WHEEL_TRACK_CM * 180.0 / (360.0 * CLOCKWISE_SPEED_CM_S)) * 950));
      stopServos();
      deadEndState = 1;
      break;

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
