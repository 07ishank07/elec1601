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
int speed_Left = 1500;
int speed_Right = 1500;
void setup() // Built in initialization block
{
Serial.begin(9600);
servoLeft.attach(13);
servoRight.attach(12);
pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
pinMode(irReceiverPinRight, INPUT);
pinMode(irLedPinRight, OUTPUT);
pinMode(redLedPinRight, OUTPUT);
pinMode(irReceiverPinMid, INPUT);
pinMode(irLedPinMid, OUTPUT);
pinMode(redLedPinMid, OUTPUT);
pinMode(irReceiverPinLeft, INPUT);
pinMode(irLedPinLeft, OUTPUT);
pinMode(redLedPinLeft, OUTPUT);
servoLeft.writeMicroseconds(1500);
servoRight.writeMicroseconds(1500);
delay(5000);
}

void loop() // Main loop auto-repeats
{
servoLeft.writeMicroseconds(speed_Left);
servoRight.writeMicroseconds(speed_Right);
int irValRight = irDetectRight(41000);
int irValMid = irDetectMid(38000);
int irValLeft = irDetectLeft(41000);
// Case 0
if (irValLeft == 1 && irValMid == 1 && irValRight == 1) {
digitalWrite(redLedPinLeft, LOW);
digitalWrite(redLedPinMid, LOW);
digitalWrite(redLedPinRight, LOW);
while (irValLeft == 1 && irValMid == 1 && irValRight == 1) {
digitalWrite(redLedPinLeft, LOW);
digitalWrite(redLedPinMid, LOW);
digitalWrite(redLedPinRight, LOW);
}
}
// Case 1
else if (irValLeft == 0 && irValMid == 1 && irValRight == 0) {
digitalWrite(redLedPinLeft, LOW);
digitalWrite(redLedPinMid, LOW);
digitalWrite(redLedPinRight, HIGH);
while (irValLeft == 0 && irValMid == 1 && irValRight == 0) {
speed_Left = 1475;
speed_Right = 1525;
}
}
// Case 2
else if (irValLeft == 0 && irValMid == 0 && irValRight == 1) {
digitalWrite(redLedPinLeft, LOW);
digitalWrite(redLedPinMid, HIGH);
digitalWrite(redLedPinRight, LOW);
speed_Left = 1485;
speed_Right = 1500;
delay(1000);
while (irValLeft == 0 && irValMid == 0 && irValRight == 1) {
speed_Left = 1475;
speed_Right = 1525;
}
}
// Case 3
else if (irValLeft == 1 && irValMid == 0 && irValRight == 0) {
digitalWrite(redLedPinLeft, LOW);
digitalWrite(redLedPinMid, HIGH);
digitalWrite(redLedPinRight, HIGH);
while (irValLeft == 1 && irValMid == 0 && irValRight == 0) {
speed_Left = 1500;
speed_Right = 1525;
}
}
// Case 4
else if (irValLeft == 0 && irValMid == 0 && irValRight == 0) {
digitalWrite(redLedPinLeft, HIGH);
digitalWrite(redLedPinMid, LOW);
digitalWrite(redLedPinRight, LOW);
while (irValLeft == 0 && irValMid == 0 && irValRight == 0) {
speed_Left = 1525;
speed_Right = 1525;
}
}
/if (irValRight == 0 && irValLeft == 1) {
servoLeft.writeMicroseconds(1455); // Turn Left
servoRight.writeMicroseconds(1440);
digitalWrite(redLedPinRight, HIGH);
}
if (irValRight == 1 && irValLeft == 0) {
servoLeft.writeMicroseconds(1545); // Turn Right
servoRight.writeMicroseconds(1560);
digitalWrite(redLedPinRight, HIGH);
}
delay(1000);
digitalWrite(redLedPinRight, LOW);/
}

int irDetectRight(long frequency)
{
tone(irLedPinRight, frequency); // Turn on the IR LED square wave
delay(1); // Wait 1 ms
int ir = digitalRead(irReceiverPinRight);
noTone(irLedPinRight); // Turn off the IR LED
delay(1); // Down time before recheck
return ir; // Return 0 detect, 1 no detect
}

int irDetectMid(long frequency)
{
tone(irLedPinMid, frequency); // Turn on the IR LED square wave
delay(1); // Wait 1 ms
int ir = digitalRead(irReceiverPinMid);
noTone(irLedPinMid); // Turn off the IR LED
delay(1); // Down time before recheck
return ir; // Return 0 detect, 1 no detect
}

int irDetectLeft(long frequency)
{
tone(irLedPinLeft, frequency); // Turn on the IR LED square wave
delay(1); // Wait 1 ms
int ir = digitalRead(irReceiverPinLeft);
noTone(irLedPinLeft); // Turn off the IR LED
delay(1); // Down time before recheck
return ir; // Return 0 detect, 1 no detect
}