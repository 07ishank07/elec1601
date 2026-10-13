#include <Servo.h> // Include servo library


Servo servoLeft;
Servo servoRight;
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
bool case1 = true;
bool case2 = true;
bool case3 = true;
bool deadEndCompleted = false;
const double length = 11; //placeholder need to change
const unsigned long LEFT_START_TIMEOUT_MS = 2500; //placeholder to test out later
//test
const int leftStop  = 1495;
const int rightStop = 1497;

// Offsets in µs from each stop pulse, calibrated at speedScale = 1.0
const int L_FWD =  30;   // 1525 - 1495
const int R_FWD = -29;   // 1468 - 1497

const int L_REV = -30;   // starts as the mirror of forward; tune it (see below)
const int R_REV =  29;

const int L_CW  =  30;   // 1525 - 1495  (about 90 deg in 1400 ms)
const int R_CW  =  36;   // 1533 - 1497

const int L_CCW = -20;   // 1475 - 1495  (about 90 deg in 1400 ms)
const int R_CCW = -30;   // 1467 - 1497

float speedScale = 1.0; 



//keep above

void setup() // Built in initialization block
{
    Serial.begin(9600);
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
    servoLeft.writeMicroseconds(leftStop); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(rightStop);
    delay(5000);
}

void loop() // Main loop auto-repeats
{

    int distMid = irDistance(irLedPinMid, irReceiverPinMid);
    int distLeft = irDistance(irLedPinLeft, irReceiverPinLeft);
    int distRight = irDistance(irLedPinRight, irReceiverPinRight);
    
    Serial.println(distMid);
    //Serial.   println(distLeft);
    //Serial.println(distRight);

    //Note dist gives a num between 0 and 5 to note the distance of the wall, 5 being farthest away and 0 being closest
    // Case 1
    if (distLeft < distRight) { // left side is closer to wall
        leftStart();
    }

    else if (distMid >= 4)  {
        goStraight();
    }

    
    else if (distMid <= 3 && distLeft <= 3 && distRight >= 4) { //right turn
        turnRight();
    }

    else if (distMid <= 3 && distRight <= 3 && distLeft >= 4) { //left turn
        turnLeft();
    }

    
    else if (distLeft <= 3 && distMid <= 3 && distRight <= 3) 
    {
        deadEnd();
    }
    
}

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
    tone(irLedPin, frequency); // Turn on the IR LED square wave
    delay(1); // Wait 1 ms
    int ir = digitalRead(irReceiverPin);
    noTone(irLedPin); // Turn off the IR LED
    delay(1); // Down time before recheck
    return ir; // Return 0 detect, 1 no detect
}

int irDistance(int irLedPin, int irReceiverPin)
{
   int distance = 0;
   for(long f = 38000; f <= 42000; f += 1000)
   {
      distance += irDetect(irLedPin, irReceiverPin, f);
   }
   return distance;
}

void setWheels(int leftOffset, int rightOffset) {
    servoLeft.writeMicroseconds(leftStop + (int)round(speedScale * leftOffset));
    servoRight.writeMicroseconds(rightStop + (int)round(speedScale * rightOffset));
}

unsigned long scaled(unsigned long t0) {
    return (unsigned long)(t0 / speedScale);
}

// Scenario 1
void goStraight() {

    digitalWrite(redLedPinMid, LOW);
    digitalWrite(redLedPinLeft, LOW);
    digitalWrite(redLedPinRight, HIGH);

    //while ((disMid >= 2 && distMid <=5)){
    setWheels(L_FWD, R_FWD);
    //}
    delay(scaled((5/4.71) * 1000));
    servoLeft.writeMicroseconds(leftStop); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(rightStop); 
    case1 = false;
}

// Scenario 2
void turnRight() {
    digitalWrite(redLedPinMid, HIGH);
    digitalWrite(redLedPinLeft, LOW);
    digitalWrite(redLedPinRight, LOW);
    setWheels(L_CW, R_CW);
    delay(scaled(1400)); //(3.141 * length / (4 * 4.71)) * 1000
    servoLeft.writeMicroseconds(leftStop); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(rightStop); 
    case2 = false; 
    servoLeft.writeMicroseconds(leftStop); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(rightStop);
}

// Scenario 3
void turnLeft() {
    digitalWrite(redLedPinMid, HIGH);
    digitalWrite(redLedPinLeft, LOW);
    digitalWrite(redLedPinRight, HIGH);
    setWheels(L_CCW, R_CCW);
    delay(scaled(1400));
    servoLeft.writeMicroseconds(leftStop); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(rightStop); 
    case3 = false; 
    servoLeft.writeMicroseconds(leftStop); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(rightStop);
}

// Scenario 4
void deadEnd() {
    digitalWrite(redLedPinMid, LOW);
    digitalWrite(redLedPinLeft, HIGH);
    digitalWrite(redLedPinRight, LOW);

    // Turn 180 degrees
    setWheels(L_CW, R_CW);

    delay(scaled(2800));

    servoLeft.writeMicroseconds(leftStop); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(rightStop); 
    

    deadEndCompleted = true;
    
}

//scen 5
void leftStart() {

    digitalWrite(redLedPinMid, LOW);
    digitalWrite(redLedPinLeft, HIGH);
    digitalWrite(redLedPinRight, HIGH);

    // Rotate clockwise slightly
    setWheels(L_CW, R_CW);
    delay(scaled(200));

    // Move forward until approximately centred
    int distLeft = irDistance(irLedPinLeft, irReceiverPinLeft);
    int distRight = irDistance(irLedPinRight, irReceiverPinRight);

    unsigned long startTime = millis();

    while (!(distRight - distLeft <= 1) && (millis() - startTime < scaled(LEFT_START_TIMEOUT_MS))) {
        setWheels(L_FWD, R_FWD);

        distLeft = irDistance(irLedPinLeft, irReceiverPinLeft);
        distRight = irDistance(irLedPinRight, irReceiverPinRight);
    }

    // Stop
    servoLeft.writeMicroseconds(leftStop);
    servoRight.writeMicroseconds(rightStop);

    delay(100);

    // Rotate anticlockwise to become parallel again
    setWheels(L_CCW, R_CCW);
    delay(scaled(200));

    //reverse to bring it back
    

    // Stop
    servoLeft.writeMicroseconds(leftStop);
    servoRight.writeMicroseconds(rightStop);
}