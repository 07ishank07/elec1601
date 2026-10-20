#include <Arduino.h>
#include <Servo.h>

// Change this number, upload, then press reset to run the selected test once.
// 0 = unknown (no movement), 1 = straight, 2 = right turn, 3 = left turn,
// 4 = dead end, 5/6 = recenter from left/right, 7/8 = straighten from left/right.
// Until the sensors detect this case consistently, the robot stays still
// with all LEDs off. After the attempt it stays stopped until reset.
const int TEST_SCENARIO = 1;
static_assert(TEST_SCENARIO >= 0 && TEST_SCENARIO <= 8, "TEST_SCENARIO must be 0-8");

Servo servoLeft;
Servo servoRight;

// Same wiring and startup sequence as the supplied draft.
const int irLedPinRight = 2;
const int irReceiverPinRight = 3;
const int redLedPinRight = A0;
const int irLedPinMid = 6;
const int irReceiverPinMid = 7;
const int redLedPinMid = A1;
const int irLedPinLeft = 10;
const int irReceiverPinLeft = 11;
const int redLedPinLeft = A2;

// CALIBRATION: wheel pulses and 90-degree times from Essentials/cases2.cpp.
// Check these on YOUR robot. The attached older draft reverses forward.
const int LEFT_STOP = 1495;
const int RIGHT_STOP = 1497;
// Increase drive pulse offsets by 25%; turning keeps its existing calibration.
const float DRIVE_SPEED_SCALE = 1.25f;
const int LEFT_DRIVE_OFFSET = (int)(30.0f * DRIVE_SPEED_SCALE + 0.5f);
const int RIGHT_DRIVE_OFFSET = (int)(29.0f * DRIVE_SPEED_SCALE + 0.5f);
const int LEFT_FORWARD = LEFT_STOP + LEFT_DRIVE_OFFSET;
const int RIGHT_FORWARD = RIGHT_STOP - RIGHT_DRIVE_OFFSET;
const int LEFT_BACKWARD = LEFT_STOP - LEFT_DRIVE_OFFSET;
const int RIGHT_BACKWARD = RIGHT_STOP + RIGHT_DRIVE_OFFSET;
const int LEFT_CLOCKWISE = 1525;
const int RIGHT_CLOCKWISE = 1533;
const int LEFT_ANTICLOCKWISE = 1475;
const int RIGHT_ANTICLOCKWISE = 1467;
const unsigned long CLOCKWISE_90_MS = 1400;
const unsigned long ANTICLOCKWISE_90_MS = 1400;
// Estimates only: servo speed is not exactly proportional to pulse offset.
// Remeasure at the new drive pulses and replace these to calibrate distances.
const float FORWARD_CM_PER_SECOND = 4.71f * DRIVE_SPEED_SCALE;
const float BACKWARD_CM_PER_SECOND = 4.71f * DRIVE_SPEED_SCALE;

// Calibrated raw zones returned by irDistance():
// 0: <= 1.5 cm, 1: > 1.5 to 3 cm, 2: > 3 to 5 cm,
// 3: > 5 to 10 cm, 4: > 10 to 15 cm, 5: no detection/out of range.
// Midpoints estimate side distances; they are not precise measurements.
const float ZONE_MIDPOINT_CM[] = {0.75f, 2.25f, 4.0f, 7.5f, 12.5f};
const float MAZE_WIDTH_CM = 10.0f;
const float CENTER_TOLERANCE_CM = 2.0f; // About 1 cm lateral error.
const float WIDTH_TOLERANCE_CM = 2.0f; // Allow for coarse distance bands.
const int FRONT_CLEAR_MIN = 4; // Beyond 10 cm, or no detection.
const int FRONT_ADVANCE_MIN = 3; // Approach a wall while it is still > 5 cm away.
const int NO_WALL = 5;
const int CLOSE_MAX = 0; // <= 1.5 cm.
const int TURN_WALL_MIN = 1; // > 1.5 cm.
const int TURN_WALL_MAX = 2; // <= 5 cm: about half the corridor width.
const int FAR_WALL_MIN = 2; // > 3 cm, relative to a very close opposite wall.
const int EMERGENCY_FRONT_MAX = 0;
const int STABLE_SAMPLES = 3;

const unsigned long ENTER_CORRIDOR_TIMEOUT_MS = (unsigned long)(4000 / DRIVE_SPEED_SCALE);
const unsigned long RECENTER_TIMEOUT_MS = (unsigned long)(3000 / DRIVE_SPEED_SCALE);
const unsigned long ENTER_CORRIDOR_MIN_MS = (unsigned long)(300 / DRIVE_SPEED_SCALE);
const float RECENTER_ANGLE_DEG = 30.0f;
const float RECENTER_ANGLE_COS = 0.8660254f;
const float BAD_START_ANGLE_DEG = 30.0f;

int stopped = 0; // One selected scenario attempt per reset, even if interrupted.

struct Distances {
    int left;
    int mid;
    int right;
};

// Prototypes allow this file to compile as .cpp as well as an Arduino sketch.
int irDetect(int irLedPin, int irReceiverPin, long frequency);
int irDistance(int irLedPin, int irReceiverPin);
Distances readDistances();
bool seesWall(int distance);
float zoneDistanceCm(int zone);
bool seesSideWall(int zone);
bool corridorWalls(const Distances &d);
bool centered(const Distances &d);
int detectScenario(const Distances &d);
void setLeds(int left, int mid, int right);
void showScenario(int scenario);
void stopServos();
void forward();
void rotate(bool clockwise, float degrees);
bool moveForwardCm(float cm);
bool enterCorridor();
bool recenter(bool closeToLeft);
bool straighten(bool closeToLeft);

void setup() {
    servoLeft.attach(13);
    servoRight.attach(12);
    pinMode(irReceiverPinRight, INPUT);
    pinMode(irLedPinRight, OUTPUT);
    pinMode(redLedPinRight, OUTPUT);
    pinMode(irReceiverPinMid, INPUT);
    pinMode(irLedPinMid, OUTPUT);
    pinMode(redLedPinMid, OUTPUT);
    pinMode(irReceiverPinLeft, INPUT);
    pinMode(irLedPinLeft, OUTPUT);
    pinMode(redLedPinLeft, OUTPUT);
    stopServos();
    setLeds(LOW, LOW, LOW);
    delay(5000);
}

void loop() {
    if (stopped) return;

    // Wait for three matching readings. No other scenario may run.
    stopServos();
    for (int i = 0; i < STABLE_SAMPLES; ++i) {
        if (detectScenario(readDistances()) != TEST_SCENARIO) {
            setLeds(LOW, LOW, LOW);
            delay(50);
            return;
        }
    }

    stopped = 1; // Do not retry or switch cases, even if a movement aborts.
    showScenario(TEST_SCENARIO);
    switch (TEST_SCENARIO) {
        case 0: // Unknown: all LEDs off, no movement.
            break;
        case 1: // Move 5 cm, unless the front obstacle check stops it early.
            moveForwardCm(5.0f);
            break;
        case 2: // Right corner: turn, then enter the corridor.
            rotate(true, 90.0f);
            enterCorridor();
            break;
        case 3: // Left corner.
            rotate(false, 90.0f);
            enterCorridor();
            break;
        case 4: // Dead end: all three sensors see walls.
            rotate(true, 180.0f);
            moveForwardCm(5.0f);
            break;
        case 5: // Parallel, too close to left wall.
            recenter(true);
            break;
        case 6: // Parallel, too close to right wall.
            recenter(false);
            break;
        case 7: // Pointing towards left wall, approximately 30 degrees.
            straighten(true);
            break;
        case 8: // Pointing towards right wall, approximately 30 degrees.
            straighten(false);
            break;
        default:
            setLeds(LOW, LOW, LOW);
            break;
    }

    stopServos();
}

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
    tone(irLedPin, frequency);
    delay(1);
    int ir = digitalRead(irReceiverPin);
    noTone(irLedPin);
    delay(1);
    return ir; // 0 = detected, 1 = not detected.
}

int irDistance(int irLedPin, int irReceiverPin) {
    int distance = 0;
    for (long f = 38000; f <= 42000; f += 1000) {
        distance += irDetect(irLedPin, irReceiverPin, f);
    }
    return distance;
}

Distances readDistances() {
    Distances d;
    d.mid = irDistance(irLedPinMid, irReceiverPinMid);
    d.left = irDistance(irLedPinLeft, irReceiverPinLeft);
    d.right = irDistance(irLedPinRight, irReceiverPinRight);
    return d;
}

bool seesWall(int distance) {
    return distance >= 0 && distance < NO_WALL;
}

float zoneDistanceCm(int zone) {
    // Call only for detected walls; no detection has no known distance.
    return ZONE_MIDPOINT_CM[zone];
}

bool seesSideWall(int zone) {
    // A wall 10-15 cm away is beyond this corridor's width: treat it as
    // a side opening, even though the IR receiver can still detect it.
    return seesWall(zone) && zoneDistanceCm(zone) <= MAZE_WIDTH_CM;
}

bool corridorWalls(const Distances &d) {
    // Side sensors are separated on the chassis, so their distances need not
    // add up to 10 cm. Reject sums that are too large, not smaller sums.
    return seesSideWall(d.left) && seesSideWall(d.right)
        && zoneDistanceCm(d.left) + zoneDistanceCm(d.right)
            <= MAZE_WIDTH_CM + WIDTH_TOLERANCE_CM;
}

bool centered(const Distances &d) {
    return corridorWalls(d)
        && fabs(zoneDistanceCm(d.left) - zoneDistanceCm(d.right)) <= CENTER_TOLERANCE_CM;
}

int detectScenario(const Distances &d) {
    const bool leftWall = seesSideWall(d.left);
    const bool rightWall = seesSideWall(d.right);
    const bool frontCanAdvance = d.mid >= FRONT_ADVANCE_MIN;
    const bool turnFront = d.mid >= TURN_WALL_MIN && d.mid <= TURN_WALL_MAX;
    const bool turnLeft = d.left >= TURN_WALL_MIN && d.left <= TURN_WALL_MAX;
    const bool turnRight = d.right >= TURN_WALL_MIN && d.right <= TURN_WALL_MAX;

    // Angled starts require a distant DETECTED side wall, not an opening.
    // Test these before dead ends; equal open readings are never a corridor.
    if (d.mid <= CLOSE_MAX && d.left <= CLOSE_MAX
        && rightWall && d.right >= FAR_WALL_MIN) return 7;
    if (d.mid <= CLOSE_MAX && d.right <= CLOSE_MAX
        && leftWall && d.left >= FAR_WALL_MIN) return 8;

    if (turnFront && turnLeft && !rightWall) return 2;
    if (turnFront && turnRight && !leftWall) return 3;
    if (d.mid <= TURN_WALL_MAX && centered(d)) return 4;

    if (frontCanAdvance && corridorWalls(d)) {
        // The supplied text swaps left/right in the checks for cases 5/6.
        // Follow the titles and actions: lower score means the nearer wall.
        const float differenceCm = zoneDistanceCm(d.right) - zoneDistanceCm(d.left);
        if (differenceCm > CENTER_TOLERANCE_CM) return 5;
        if (differenceCm < -CENTER_TOLERANCE_CM) return 6;
        return 1;
    }
    return 0; // Ambiguous readings: stop rather than guess.
}

void setLeds(int left, int mid, int right) {
    digitalWrite(redLedPinLeft, left);
    digitalWrite(redLedPinMid, mid);
    digitalWrite(redLedPinRight, right);
}

void showScenario(int scenario) {
    stopServos();
    if (scenario == 8) {
        // Two visible flashes; each on/off state lasts one second.
        for (int i = 0; i < 2; ++i) {
            setLeds(LOW, LOW, HIGH);
            delay(1000);
            setLeds(LOW, LOW, LOW);
            delay(1000);
        }
    } else {
        // A2/A1/A0 display the scenario number as three binary bits.
        setLeds((scenario & 4) ? HIGH : LOW,
                (scenario & 2) ? HIGH : LOW,
                (scenario & 1) ? HIGH : LOW);
        if (scenario != 0) delay(1000); // Show the selected case before moving.
    }
}

void stopServos() {
    servoLeft.writeMicroseconds(LEFT_STOP);
    servoRight.writeMicroseconds(RIGHT_STOP);
}

void forward() {
    servoLeft.writeMicroseconds(LEFT_FORWARD);
    servoRight.writeMicroseconds(RIGHT_FORWARD);
}

void rotate(bool clockwise, float degrees) {
    servoLeft.writeMicroseconds(clockwise ? LEFT_CLOCKWISE : LEFT_ANTICLOCKWISE);
    servoRight.writeMicroseconds(clockwise ? RIGHT_CLOCKWISE : RIGHT_ANTICLOCKWISE);
    const unsigned long quarterTurn = clockwise ? CLOCKWISE_90_MS : ANTICLOCKWISE_90_MS;
    delay((unsigned long)(quarterTurn * degrees / 90.0f));
    stopServos();
    delay(100);
}

bool moveForwardCm(float cm) {
    // Straightness depends on calibrated wheel speeds; there are no encoders.
    const unsigned long duration = (unsigned long)(1000.0f * cm / FORWARD_CM_PER_SECOND + 0.5f);
    if (readDistances().mid <= EMERGENCY_FRONT_MAX) return false;
    const unsigned long start = millis();
    forward();
    while (millis() - start < duration) {
        const Distances d = readDistances();
        if (d.mid <= EMERGENCY_FRONT_MAX) {
            stopServos();
            return false;
        }
    }
    stopServos();
    return true;
}

bool enterCorridor() {
    if (readDistances().mid <= EMERGENCY_FRONT_MAX) return false;
    const unsigned long start = millis();
    int stable = 0;
    forward();
    while (millis() - start < ENTER_CORRIDOR_TIMEOUT_MS) {
        const Distances d = readDistances();
        if (d.mid <= EMERGENCY_FRONT_MAX) break;
        // Move into the corridor even if its walls are visible after turning.
        if (millis() - start >= ENTER_CORRIDOR_MIN_MS
            && d.mid >= FRONT_CLEAR_MIN && centered(d)) {
            if (++stable >= STABLE_SAMPLES) {
                stopServos();
                return true;
            }
        } else {
            stable = 0;
        }
    }
    stopServos();
    return false;
}

bool recenter(bool closeToLeft) {
    rotate(closeToLeft, RECENTER_ANGLE_DEG); // Point away from the nearer wall.
    if (readDistances().mid <= EMERGENCY_FRONT_MAX) return false;
    const unsigned long start = millis();
    int stable = 0;
    bool reachedCenter = false;
    forward();
    while (millis() - start < RECENTER_TIMEOUT_MS) {
        const Distances d = readDistances();
        if (d.mid <= EMERGENCY_FRONT_MAX) break;
        if (centered(d)) {
            if (++stable >= STABLE_SAMPLES) {
                reachedCenter = true;
                break;
            }
        } else {
            stable = 0;
        }
    }
    stopServos();
    const unsigned long diagonalTime = millis() - start;
    if (!reachedCenter) return false;

    rotate(!closeToLeft, RECENTER_ANGLE_DEG); // Restore original heading.
    // Reverse only the along-corridor component of the diagonal travel.
    // This assumes the test area behind the robot is clear (no rear sensor).
    const unsigned long reverseTime = (unsigned long)(diagonalTime
        * RECENTER_ANGLE_COS * FORWARD_CM_PER_SECOND / BACKWARD_CM_PER_SECOND);
    servoLeft.writeMicroseconds(LEFT_BACKWARD);
    servoRight.writeMicroseconds(RIGHT_BACKWARD);
    delay(reverseTime);
    stopServos();
    const Distances finalPosition = readDistances();
    return centered(finalPosition) && finalPosition.mid >= FRONT_CLEAR_MIN;
}

bool straighten(bool closeToLeft) {
    // The test specifies an approximately 30-degree starting angle.
    // Three coarse IR scores cannot measure heading uniquely: use a calibrated
    // turn, then check that the front is clear and both side walls are visible.
    // Correct heading only; cases 5/6 must be selected and tested separately.
    rotate(closeToLeft, BAD_START_ANGLE_DEG);
    const Distances d = readDistances();
    return d.mid >= FRONT_CLEAR_MIN && corridorWalls(d);
}
