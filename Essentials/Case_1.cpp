/*
What you must display:
LED-R Right (A0): ON 
LED-R Mid (A1): OFF
LED-R Left (A2): OFF
(After this, your LEDs can do what you want)
What your robot must check:
Both left and right sensors should identify a similar reading
You can choose how you measure this. Calibrating your sensors to measure distance is advised
The forward sensor should see no wall, or a wall at least 10cm away.
Once again, calibrating your sensors to measure distance is advised
What your robot must do to:
It must move 5cm forward
It must remain approximately parallel to the walls
The center of the robot must remain within 3cm of the center of the corridor
Stop after completion
Why this scenario is important:
This is the main way your robot should move (along a corridor in a straight line)
*/

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

    //Note dist gives a num between 0 and 5 to note the distance of the wall, 0 being closest and 5 being farthest away
    // Case 1
    if (distMid >= 4 && case1 && abs(distLeft - distRight) <= 1)  {

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


/* 
things to test:

the servo value that means forward for each wheel
whether a bigger irDistance really means farther
the forward sensor's "10 cm or more" threshold
the wheel speed v, which sets the drive time t = d/v
the tolerance $\varepsilon$ for "similar"


*/