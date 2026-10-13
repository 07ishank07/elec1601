
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

// Global variables
int stopped = 0;

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

  // Detect scenarios
  int scenario = 0; //unknown if no other is detected
  if (distMid >= 4) scenario = 1;                                          // middle of corridor
  if (distMid <= 3 && distLeft <= 3 && distRight >= 4) scenario = 2;       // ideal position for right turn
  if (distMid <= 3 && distRight <= 3 && distLeft >= 4) scenario = 3;       // ideal position for left turn
  if (distLeft <= 3 && distMid <= 3 && distRight <= 3) scenario = 4;       // dead end
  if (distLeft <= 1 && distRight <= 3 && distMid >= 4 && distRight - distLeft >= 2) scenario = 5;                                  // left side of corridor, parallel
  // scenario 6, 7, 8 not implemented in original logic

  /*  Motion control
    Only one scenario procedure is executed per loop. If tests overlap,
    the higher scenario number has priority.
  */
  if (!stopped)
  {
    switch (scenario)
    {
      case 0: //Unknown scenario
      setLeds(LOW, LOW, LOW);
      stopServos();

      stopped = 1;
      break;

      case 1: //middle of corridor
      setLeds(LOW, LOW, HIGH);
      delay(1000);
      setWheels(L_FWD, R_FWD);
      delay(scaled((5 / 4.71) * 1000)); // move forward 5cm
      stopServos();

      stopped = 1;
      break;

      case 2: //ideal position right turn
      setLeds(LOW, HIGH, LOW);
      delay(1000);
      setWheels(L_CW, R_CW);
      delay(scaled(1400)); // 90 deg right
      stopServos();

      stopped = 1;
      break;

      case 3: //ideal position left turn
      setLeds(LOW, HIGH, HIGH);
      delay(1000);
      setWheels(L_CCW, R_CCW);
      delay(scaled(1400)); // 90 deg left
      stopServos();

      stopped = 1;
      break;

      case 4: //dead end
      setLeds(HIGH, LOW, LOW);
      delay(1000);
      setWheels(L_CW, R_CW);
      delay(scaled(2800)); // 180 deg
      stopServos();

      stopped = 1;
      break;

      case 5: //left side of corridor parallel
      setLeds(HIGH, LOW, HIGH);
      delay(1000);

      // Rotate clockwise slightly
      setWheels(L_CW, R_CW);
      delay(scaled(200));

      // Move forward until approximately centred
      {
        int dL = irDistance(irLedPinLeft, irReceiverPinLeft);
        int dR = irDistance(irLedPinRight, irReceiverPinRight);
        unsigned long startTime = millis();

        while (!(dR - dL <= 1) && (millis() - startTime < scaled(LEFT_START_TIMEOUT_MS))) {
          setWheels(L_FWD, R_FWD);
          dL = irDistance(irLedPinLeft, irReceiverPinLeft);
          dR = irDistance(irLedPinRight, irReceiverPinRight);
        }
      }

      stopServos();
      delay(100);

      // Rotate counterclockwise to become parallel again
      setWheels(L_CCW, R_CCW);
      delay(scaled(200));

      // Move backwards to approximately starting position (not implemented in original)

      stopServos();

      stopped = 1;
      break;

      default:
      setLeds(LOW, LOW, LOW);
      stopServos();

      stopped = 1;
    }
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