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
const double length = 1;
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
    servoLeft.writeMicroseconds(1500); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(1500);
    delay(5000);
}

void loop() // Main loop auto-repeats
{


    int distMid = irDistance(irLedPinMid, irReceiverPinMid);
    int distLeft = irDistance(irLedPinLeft, irReceiverPinLeft);
    int distRight = irDistance(irLedPinRight, irReceiverPinRight);
    Serial.println(distMid);
    Serial.println(distLeft);
    Serial.println(distRight);

    //Note dist gives a num between 0 and 5 to note the distance of the wall, 5 being farthest away and 0 being closest
    // Case 1
    if (distMid >= 4 && case1 && abs(distLeft - distRight) <= 1)  {
        doCase1();
    }
    else if (distMid >= 2 && distMid <= 3 && case2 && distLeft >= 2 && distLeft <= 3 && distRight >= 5) { //Ishaanis nightmare fuel
        doCase2();
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

void doCase1() {
    digitalWrite(redLedPinMid, LOW);
    digitalWrite(redLedPinLeft, LOW);
    digitalWrite(redLedPinRight, HIGH);
    servoLeft.writeMicroseconds(1475); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(1525);
    delay((5/4.71) * 1000);
    servoLeft.writeMicroseconds(1500); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(1500); 
    case1 = false;
}

void doCase2() {
    digitalWrite(redLedPinMid, HIGH);
    digitalWrite(redLedPinLeft, LOW);
    digitalWrite(redLedPinRight, LOW);
    servoLeft.writeMicroseconds(1475);
    servoRight.writeMicroseconds(1475);
    delay((3.141 * length / (4 * 4.71)) * 1000);
    servoLeft.writeMicroseconds(1500); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(1500); 
    case2 = false; 
    int distMid = irDistance(irLedPinMid, irReceiverPinMid);
    int distLeft = irDistance(irLedPinLeft, irReceiverPinLeft);
    int distRight = irDistance(irLedPinRight, irReceiverPinRight);
    while (!(distMid >= 4 && abs(distLeft - distRight) <= 1)) {
        servoLeft.writeMicroseconds(1475);
        servoRight.writeMicroseconds(1525);
        distMid = irDistance(irLedPinMid, irReceiverPinMid);
        distLeft = irDistance(irLedPinLeft, irReceiverPinLeft);
        distRight = irDistance(irLedPinRight, irReceiverPinRight);
    }
    servoLeft.writeMicroseconds(1500); //stop, adjust wheels since it will still move forward a bit
    servoRight.writeMicroseconds(1500);
}
