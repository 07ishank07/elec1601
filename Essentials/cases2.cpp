
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

// Global constants / calibration values
const double length = 11; //placeholder need to change
const unsigned long LEFT_START_TIMEOUT_MS = 2500; //placeholder to test out later

const int leftStop  = 1495;
const int rightStop = 1497;

// Offsets in us from each stop pulse, calibrated at speedScale = 1.0
const int L_FWD =  30;   // 1525 - 1495
const int R_FWD = -29;   // 1468 - 1497

const int L_REV = -30;   // starts as the mirror of forward; tune it
const int R_REV =  29;

const int L_CW  =  30;   // 1525 - 1495  (about 90 deg in 1400 ms)
const int R_CW  =  36;   // 1533 - 1497

const int L_CCW = -20;   // 1475 - 1495  (about 90 deg in 1400 ms)
const int R_CCW = -30;   // 1467 - 1497

float speedScale = 1.0;

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
  // Measure left, middle, and right distances
  // dist is 0-5: 5 = farthest from wall, 0 = closest
  int distMid   = irDistance(irLedPinMid, irReceiverPinMid);
  int distLeft  = irDistance(irLedPinLeft, irReceiverPinLeft);
  int distRight = irDistance(irLedPinRight, irReceiverPinRight);

  Serial.println(distMid);

  // 20 cm clear corridor, approximately 8.5 cm between side sensor positions:
  // centred side clearance is about 5.75 cm (zone 3). Allow zones 2-3 for
  // coarse readings and small offsets. Zone 4 still detects a wall; only 5
  // means no detection. Zones 0/1/2/3/4 end at 1.5/3/5/10/15 cm.
  int scenario = 0; // Unknown: stop the wheels, then check again next loop.
  if (distMid >= 3 && distLeft >= 2 && distLeft <= 3 && distRight >= 2 && distRight <= 3) scenario = 1;
  if (distMid >= 1 && distMid <= 4 && distLeft >= 2 && distLeft <= 3 && distRight == 5) scenario = 2;
  if (distMid >= 1 && distMid <= 4 && distRight >= 2 && distRight <= 3 && distLeft == 5) scenario = 3;
  if (distMid <= 2 && distLeft >= 2 && distLeft <= 3 && distRight >= 2 && distRight <= 3) scenario = 4;
  if (distMid >= 3 && distLeft <= 2 && distRight >= 3 && distRight <= 4 && (distLeft <= 1 || distRight == 4)) scenario = 5;
  if (distMid >= 3 && distRight <= 2 && distLeft >= 3 && distLeft <= 4 && (distRight <= 1 || distLeft == 4)) scenario = 6;
  if (distLeft <= 1 && distMid >= distLeft && distMid <= 3 && distRight >= 3 && distRight <= 4) scenario = 7;  
  if (distRight <= 1 && distMid >= distRight && distMid <= 3 && distLeft >= 3 && distLeft <= 4) scenario = 8;

  /*  Motion control
    Recheck after every movement. All cases are enabled; case 0 waits.
    Only one scenario procedure is executed per loop. If tests overlap,
    the higher scenario number has priority.
  */
  switch (scenario)
  {
    case 0: //Unknown scenario
    setLeds(LOW, LOW, LOW);
    stopServos();

    break;

    case 1: //middle of corridor, including approaching a front wall in zone 3
    {
    setLeds(LOW, LOW, HIGH);
    delay(1000);
    unsigned long startTime = millis();
    // Up to 5 cm per pass. Recheck before driving and throughout the movement.
    while (millis() - startTime < scaled((5 / 4.71) * 1000)) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM <= 2 || dL < 2 || dL > 3 || dR < 2 || dR > 3) break;
      setWheels(L_FWD, R_FWD);
    }
    stopServos();

    break;
    }

    case 2: //ideal position right turn
    {
    setLeds(LOW, HIGH, LOW);
    delay(1000);

    // Recognise the opening early, but approach to zone 1-2 before turning.
    // With the front sensor 6 cm ahead of the axle, about 4 cm clearance
    // puts the axle near the centre of the 20 cm corner.
    unsigned long approachStart = millis();
    int readyReadings = 0;
    while (millis() - approachStart < 5000) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM == 0 || dM == 5 || dL < 2 || dL > 3 || dR != 5) break;
      if (dM >= 1 && dM <= 2) {
        if (++readyReadings >= 3) break;
        continue; // Confirm the turn position with the wheels stopped.
      }
      readyReadings = 0;
      setWheels(L_FWD, R_FWD);
      delay(scaled(50)); // Small approach step, then measure again.
      stopServos();
      delay(30);
    }
    stopServos();
    if (readyReadings < 3) break; // Reclassify; do not turn on failed checks.

    setWheels(L_CW, R_CW);
    delay(scaled(1400 * 75.0 / 90.0)); // Most of the turn, then refine below.
    stopServos();
    delay(100);

    // Check while stationary, then turn in roughly 2-degree pulses if needed.
    // Three matching samples reduce noise; they do not prove exact heading.
    unsigned long correctionStart = millis();
    unsigned long turnedMs = 0;
    const unsigned long correctionLimit = scaled(1400 / 3.0);
    int alignedReadings = 0;
    while (millis() - correctionStart < 5000) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM >= 4 && dL >= 2 && dL <= 3 && dR >= 2 && dR <= 3) {
        if (++alignedReadings >= 3) break;
        continue; // Stay stopped while confirming the readings.
      }
      alignedReadings = 0;
      if (turnedMs >= correctionLimit) break;
      unsigned long pulseMs = min(scaled(30), correctionLimit - turnedMs);
      if (pulseMs == 0) break;
      setWheels(L_CW, R_CW);
      delay(pulseMs);
      stopServos();
      turnedMs += pulseMs;
      delay(30); // Allow the chassis to settle before reading again.
    }
    stopServos();
    // A timeout/turn limit ends this attempt without driving forward.
    if (alignedReadings < 3) {
      break;
    }

    // Enter the corridor; require three centred readings after moving in.
    {
      unsigned long startTime = millis();
      int centredReadings = 0;
      if (irDistance(irLedPinMid, irReceiverPinMid) > 0) {
        startTime = millis();
        setWheels(L_FWD, R_FWD);
        while (millis() - startTime < scaled(4000)) {
          int dM = irDistance(irLedPinMid, irReceiverPinMid);
          int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
          int dR = irDistance(irLedPinRight, irReceiverPinRight);
          if (dM == 0) break; // Stop if the front wall is <= 1.5 cm away.
          if (millis() - startTime >= scaled(300) && dM >= 4
              && dL >= 2 && dL <= 3 && dR >= 2 && dR <= 3) {
            if (++centredReadings >= 3) break;
          } else {
            centredReadings = 0;
          }
        }
      }
    }
    stopServos();

    break;
    }

    case 3: //ideal position left turn
    {
    setLeds(LOW, HIGH, HIGH);
    delay(1000);

    // Recognise the opening early, but approach to zone 1-2 before turning.
    // With the front sensor 6 cm ahead of the axle, about 4 cm clearance
    // puts the axle near the centre of the 20 cm corner.
    unsigned long approachStart = millis();
    int readyReadings = 0;
    while (millis() - approachStart < 5000) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM == 0 || dM == 5 || dR < 2 || dR > 3 || dL != 5) break;
      if (dM >= 1 && dM <= 2) {
        if (++readyReadings >= 3) break;
        continue; // Confirm the turn position with the wheels stopped.
      }
      readyReadings = 0;
      setWheels(L_FWD, R_FWD);
      delay(scaled(50)); // Small approach step, then measure again.
      stopServos();
      delay(30);
    }
    stopServos();
    if (readyReadings < 3) break; // Reclassify; do not turn on failed checks.

    setWheels(L_CCW, R_CCW);
    delay(scaled(1400 * 75.0 / 90.0)); // Most of the turn, then refine below.
    stopServos();
    delay(100);

    // Check while stationary, then turn in roughly 2-degree pulses if needed.
    // Three matching samples reduce noise; they do not prove exact heading.
    unsigned long correctionStart = millis();
    unsigned long turnedMs = 0;
    const unsigned long correctionLimit = scaled(1400 / 3.0);
    int alignedReadings = 0;
    while (millis() - correctionStart < 5000) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM >= 4 && dL >= 2 && dL <= 3 && dR >= 2 && dR <= 3) {
        if (++alignedReadings >= 3) break;
        continue; // Stay stopped while confirming the readings.
      }
      alignedReadings = 0;
      if (turnedMs >= correctionLimit) break;
      unsigned long pulseMs = min(scaled(30), correctionLimit - turnedMs);
      if (pulseMs == 0) break;
      setWheels(L_CCW, R_CCW);
      delay(pulseMs);
      stopServos();
      turnedMs += pulseMs;
      delay(30); // Allow the chassis to settle before reading again.
    }
    stopServos();
    // A timeout/turn limit ends this attempt without driving forward.
    if (alignedReadings < 3) {
      break;
    }

    // Same corridor entry check as case 2, after the opposite turn.
    {
      unsigned long startTime = millis();
      int centredReadings = 0;
      if (irDistance(irLedPinMid, irReceiverPinMid) > 0) {
        startTime = millis();
        setWheels(L_FWD, R_FWD);
        while (millis() - startTime < scaled(4000)) {
          int dM = irDistance(irLedPinMid, irReceiverPinMid);
          int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
          int dR = irDistance(irLedPinRight, irReceiverPinRight);
          if (dM == 0) break;
          if (millis() - startTime >= scaled(300) && dM >= 4
              && dL >= 2 && dL <= 3 && dR >= 2 && dR <= 3) {
            if (++centredReadings >= 3) break;
          } else {
            centredReadings = 0;
          }
        }
      }
    }
    stopServos();

    break;
    }

    case 4: //dead end
    {
    setLeds(HIGH, LOW, LOW);
    delay(1000);
    setWheels(L_CW, R_CW);
    delay(scaled(1400 * 165.0 / 90.0)); // Most of the turn, then refine below.
    stopServos();
    delay(100);

    // Check while stationary, then turn in roughly 2-degree pulses if needed.
    // Three matching samples reduce noise; they do not prove exact heading.
    unsigned long correctionStart = millis();
    unsigned long turnedMs = 0;
    const unsigned long correctionLimit = scaled(1400 / 3.0);
    int alignedReadings = 0;
    while (millis() - correctionStart < 5000) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM >= 4 && dL >= 2 && dL <= 3 && dR >= 2 && dR <= 3) {
        if (++alignedReadings >= 3) break;
        continue; // Stay stopped while confirming the readings.
      }
      alignedReadings = 0;
      if (turnedMs >= correctionLimit) break;
      unsigned long pulseMs = min(scaled(30), correctionLimit - turnedMs);
      if (pulseMs == 0) break;
      setWheels(L_CW, R_CW);
      delay(pulseMs);
      stopServos();
      turnedMs += pulseMs;
      delay(30); // Allow the chassis to settle before reading again.
    }
    stopServos();
    // A timeout/turn limit ends this attempt without driving forward.
    if (alignedReadings < 3) {
      break;
    }

    // Move approximately 5 cm away from the dead end after turning.
    {
      if (irDistance(irLedPinMid, irReceiverPinMid) > 0) {
        unsigned long startTime = millis();
        setWheels(L_FWD, R_FWD);
        while (millis() - startTime < scaled((5 / 4.71) * 1000)) {
          if (irDistance(irLedPinMid, irReceiverPinMid) == 0) break;
        }
      }
    }
    stopServos();

    break;
    }

    case 5: //left side of corridor parallel
    setLeds(HIGH, LOW, HIGH);
    delay(1000);

    // Rotate clockwise slightly
    setWheels(L_CW, R_CW);
    delay(scaled(200)); // Approximately 13 degrees at the calibrated turn speed.
    stopServos();
    delay(100);

    // Move diagonally until centred, then undo the heading and forward shift.
    {
      unsigned long diagonalTime = 0;
      int centredReadings = 0;
      if (irDistance(irLedPinMid, irReceiverPinMid) > 0) {
        unsigned long startTime = millis();
        setWheels(L_FWD, R_FWD);
        while (millis() - startTime < scaled(LEFT_START_TIMEOUT_MS)) {
          int dM = irDistance(irLedPinMid, irReceiverPinMid);
          int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
          int dR = irDistance(irLedPinRight, irReceiverPinRight);
          if (dM == 0) break;
          if (dL >= 2 && dL <= 3 && dR >= 2 && dR <= 3) {
            if (++centredReadings >= 3) break;
          } else {
            centredReadings = 0;
          }
        }
        stopServos();
        diagonalTime = millis() - startTime;
      }

      // Do not continue the manoeuvre after an obstacle or timeout.
      if (centredReadings >= 3) {
        delay(100);
        setWheels(L_CCW, R_CCW);
        delay(scaled(200)); // Restore the original heading.
        stopServos();
        delay(100);

        // cos(13 degrees) ~= 0.975: reverse the along-corridor component.
        // Assumes equal forward/reverse speeds and clear space behind.
        // diagonalTime is measured real time, so do not scale it again.
        setWheels(L_REV, R_REV);
        delay((unsigned long)(diagonalTime * 0.975));
      }
    }
    stopServos();

    break;
    
    case 6: //right side of corridor parallel
    setLeds(HIGH, HIGH, LOW);
    delay(1000);

    // Mirror case 5: angle away from the right wall.
    setWheels(L_CCW, R_CCW);
    delay(scaled(200));
    stopServos();
    delay(100);

    {
      unsigned long diagonalTime = 0;
      int centredReadings = 0;
      if (irDistance(irLedPinMid, irReceiverPinMid) > 0) {
        unsigned long startTime = millis();
        setWheels(L_FWD, R_FWD);
        while (millis() - startTime < scaled(LEFT_START_TIMEOUT_MS)) {
          int dM = irDistance(irLedPinMid, irReceiverPinMid);
          int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
          int dR = irDistance(irLedPinRight, irReceiverPinRight);
          if (dM == 0) break;
          if (dL >= 2 && dL <= 3 && dR >= 2 && dR <= 3) {
            if (++centredReadings >= 3) break;
          } else {
            centredReadings = 0;
          }
        }
        stopServos();
        diagonalTime = millis() - startTime;
      }

      if (centredReadings >= 3) {
        delay(100);
        setWheels(L_CW, R_CW);
        delay(scaled(200));
        stopServos();
        delay(100);

        // Same reverse-distance approximation as case 5; no second scaling.
        setWheels(L_REV, R_REV);
        delay((unsigned long)(diagonalTime * 0.975));
      }
    }
    stopServos();

    break;
    
    case 7: //close to left wall, pointing towards it
    {
    setLeds(HIGH, HIGH, HIGH);
    delay(1000);
    // No fixed 30-degree turn: stop correcting as soon as the front is clear
    // and both side walls fit the corridor. Unequal sides are allowed here:
    // this case fixes heading; lateral centering belongs to cases 5/6.
    // Limit actual turning to about 45 degrees, and total checking to 5 s.
    // Check while stationary, then turn in roughly 2-degree pulses if needed.
    // Three matching samples reduce noise; they do not prove exact heading.
    unsigned long correctionStart = millis();
    unsigned long turnedMs = 0;
    const unsigned long correctionLimit = scaled(700);
    int alignedReadings = 0;
    while (millis() - correctionStart < 5000) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM >= 4 && dL <= 4 && dR <= 4 && (dL <= 3 || dR <= 3)) {
        if (++alignedReadings >= 3) break;
        continue; // Stay stopped while confirming the readings.
      }
      alignedReadings = 0;
      if (turnedMs >= correctionLimit) break;
      unsigned long pulseMs = min(scaled(30), correctionLimit - turnedMs);
      if (pulseMs == 0) break;
      setWheels(L_CW, R_CW);
      delay(pulseMs);
      stopServos();
      turnedMs += pulseMs;
      delay(30); // Allow the chassis to settle before reading again.
    }
    stopServos();

    break;
    }
    
    case 8: //close to right wall, pointing towards it
    {
    // Flash only the right LED, with one second per on/off state.
    for (int flash = 0; flash < 2; ++flash) {
      setLeds(LOW, LOW, HIGH);
      delay(1000);
      setLeds(LOW, LOW, LOW);
      delay(1000);
    }
    // No fixed 30-degree turn: stop correcting as soon as the front is clear
    // and both side walls fit the corridor. Unequal sides are allowed here:
    // this case fixes heading; lateral centering belongs to cases 5/6.
    // Limit actual turning to about 45 degrees, and total checking to 5 s.
    // Check while stationary, then turn in roughly 2-degree pulses if needed.
    // Three matching samples reduce noise; they do not prove exact heading.
    unsigned long correctionStart = millis();
    unsigned long turnedMs = 0;
    const unsigned long correctionLimit = scaled(700);
    int alignedReadings = 0;
    while (millis() - correctionStart < 5000) {
      int dM = irDistance(irLedPinMid, irReceiverPinMid);
      int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
      int dR = irDistance(irLedPinRight, irReceiverPinRight);
      if (dM >= 4 && dL <= 4 && dR <= 4 && (dL <= 3 || dR <= 3)) {
        if (++alignedReadings >= 3) break;
        continue; // Stay stopped while confirming the readings.
      }
      alignedReadings = 0;
      if (turnedMs >= correctionLimit) break;
      unsigned long pulseMs = min(scaled(30), correctionLimit - turnedMs);
      if (pulseMs == 0) break;
      setWheels(L_CCW, R_CCW);
      delay(pulseMs);
      stopServos();
      turnedMs += pulseMs;
      delay(30); // Allow the chassis to settle before reading again.
    }
    stopServos();

    break;
    }

    default:
    setLeds(LOW, LOW, LOW);
    stopServos();

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

unsigned long scaled(unsigned long t0) {
  return (unsigned long)(t0 / speedScale);
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
