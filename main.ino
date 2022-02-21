#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ServoTimer2.h> // couldnt use normal servo library as Timer1 was needed initially for interrupt control of the LEDs. saw no need to reset to the other library when we used discrete logic for the beacon instead

#include "vars.h"
#include "line_follow.h" // line_follow.h already imports servo_control.h so we dont need to here

ServoTimer2 myservo; // create servo object

int leftValue = 0; // initialise external variables
int rightValue = 0;
int middleValue = 0;
int turning_var = 0; // 0 straight, 1 left, 2 right
bool block_captured = false;
bool capturedSoftBlock = true;
bool robotMoving = false;
int last = 0;

int stage_counter = 0; // set stage counter to 0

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // initialise the motorshield
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

void setup()
{
  Serial.begin(9600); // initialise serial communication

  pinMode(orangePin, OUTPUT); // set LED pins to outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  pinMode(leftPin, INPUT); // set line sensor pins to inputs
  pinMode(middlePin, INPUT);
  pinMode(rightPin, INPUT);

  pinMode(startPin, INPUT); // set button pin to an input

  AFMS.begin(); // start the control of the motorshield

  myservo.attach(servoPin); // initialise the servo (attach the servoPin)

  myservo.write(750); // move servo to 750ms - minimum position
  delay(500);
  moveServo(myservo, 70); // moveServo to 70 degrees to initialise

  while (!digitalRead(startPin)) // wait for start button to be pressed
    delay(100);

  Serial.print("\n\n\nSTARTING\n"); // serial debugging print
}

void loop() // main loop
{
  if (stage_counter == -1) // stop case - do nothing
  {
    robotMoving = false;
    setLEDs();
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
  }

  else if (stage_counter == 0) // from start box to first block and collect first block
  {
    setLEDs();
    float volts = analogRead(pinIR) * 0.0048828125; // value from IR pin * (5/1024)
    long int distance = 25.2 * pow(volts, -1);      //calculate the distance (calibrated manually)

    robotMoving = true; // robot is about to move
    setLEDs();

    while (distance >= 14 | distance < 0) // while distance is greater than or equal to 14 - negative conditional is due to occaisional overflow values in distance
    {
      leftValue = digitalRead(leftPin) * 1000; // get the values of the line sensors (once each loop)
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;

      volts = analogRead(pinIR) * 0.0048828125; // calculate distance (once each loop)
      distance = 25.2 * pow(volts, -1);

      FollowLine(leftValue, middleValue, rightValue, leftMotor, rightMotor); // set motors to correct values to follow line
    }

    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);

    robotMoving = false;
    setLEDs();

    Serial.println("BLOCK DETECTED");

    delay(100);

    moveServo(myservo, 0); // open front panel

    delay(100);

    robotMoving = true;
    setLEDs();
    leftMotor->setSpeed(200); // move forward for 1.5 seconds to get block
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(200);
    rightMotor->run(FORWARD);
    delay(1500);

    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    robotMoving = false;

    setLEDs();
    moveServo(myservo, 90); //close servo panel
    delay(1000);            //stop

    Serial.println("Doing squish test");
    doSquishTest(myservo, leftMotor, rightMotor); // sets capturedSoftBlock based on the result of the servo squish test
    block_captured = true;
    setLEDs(); // set LEDs - block captured is now true so one of the block LEDs will be on based on which block is in the robot

    stage_counter++; // incremement the stage counter
  }

  else if (stage_counter == 1) // move back to the drop-off area
  {
    robotMoving = true;
    setLEDs(); // flashing beacon etc on

    long unsigned int start = millis(); // follow line in reverse using correction algorithm for 9 seconds
    while (millis() - start <= 9000)
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLineReverse(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    while (!(leftValue > 800 && middleValue > 800 && rightValue > 800)) // reverse using the simple line follow until all three line sensors read high - ie the robot is at the cross by the drop off area
    // two algorithms are needed as the main algorithm uses timed turns so the robot can miss the cross
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLineReverse2(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }
    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);

    robotMoving = false;
    setLEDs(); // beacon off

    delay(500);
    stage_counter++; // increment stage counter
  }

  else if (stage_counter == 2) // drop off first block and go back to line
  {
    if (capturedSoftBlock) // drop off block to correct zone based on which block it is
      turnLeft(leftMotor, rightMotor, myservo);
    else
      turnRight(leftMotor, rightMotor, myservo);

    setLEDs();
    stage_counter++; // increment the stage counter
  }

  else if (stage_counter == 3) // move to the second block and collect second block
  {
    moveServo(myservo, 70); // reset servo position

    float volts = analogRead(pinIR) * 0.0048828125; // value from IR pin * (5/1024)
    long int distance = 25.2 * pow(volts, -1);      //calculate the distance (calibrated manually)

    robotMoving = true; // robot is about to move
    setLEDs();

    while (distance >= 14 | distance < 0) // while distance is greater than or equal to 14 - negative conditional is due to occaisional overflow values in distance
    {
      leftValue = digitalRead(leftPin) * 1000; // get the values of the line sensors (once each loop)
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;

      volts = analogRead(pinIR) * 0.0048828125; // calculate distance (once each loop)
      distance = 25.2 * pow(volts, -1);

      FollowLine(leftValue, middleValue, rightValue, leftMotor, rightMotor); // set motors to correct values to follow line
    }

    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);

    robotMoving = false;
    setLEDs();

    Serial.println("BLOCK DETECTED");

    delay(100);

    moveServo(myservo, 0); // open front panel

    delay(100);

    robotMoving = true;
    setLEDs();
    leftMotor->setSpeed(200); // move forward for 1.5 seconds to get block
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(200);
    rightMotor->run(FORWARD);
    delay(1500);

    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    robotMoving = false;

    setLEDs();
    moveServo(myservo, 90); //close servo panel
    delay(1000);            //stop

    Serial.println("Doing squish test");
    doSquishTest(myservo, leftMotor, rightMotor); // sets capturedSoftBlock based on the result of the servo squish test
    block_captured = true;
    setLEDs(); // set LEDs - block captured is now true so one of the block LEDs will be on based on which block is in the robot

    stage_counter++; // incremement the stage counter
  }

  else if (stage_counter == 4) // move back to the drop-off area
  {
    robotMoving = true;
    setLEDs(); // flashing beacon etc on

    leftValue = digitalRead(leftPin) * 1000;
    middleValue = digitalRead(middlePin) * 1000;
    rightValue = digitalRead(rightPin) * 1000;

    long unsigned int start = millis(); // follow line in reverse using correction algorithm for 9 seconds
    while (millis() - start <= 9000)
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLineReverse(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    while (!(leftValue > 800 && middleValue > 800 && rightValue > 800)) // reverse using the simple line follow until all three line sensors read high - ie the robot is at the cross by the drop off area
    // two algorithms are needed as the main algorithm uses timed turns so the robot can miss the cross
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLineReverse2(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }
    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);

    robotMoving = false;
    setLEDs(); // beacon off

    delay(500);
    stage_counter++; // increment stage counter
  }

  else if (stage_counter == 5) // drop off second block and go back to line
  {
    if (capturedSoftBlock) // drop off block to correct zone based on which block it is
      turnLeft(leftMotor, rightMotor, myservo);
    else
      turnRight(leftMotor, rightMotor, myservo);

    setLEDs();
    stage_counter++; // increment the stage counter
  }

  else if (stage_counter == 6) // move to the edge of the third block area
  {
    moveServo(myservo, 70); // reset servo to the correct position
    robotMoving = true;
    setLEDs();

    unsigned long start = millis();
    while (millis() - start <= 5000) // follow line forwards forwards for 5 seconds to clear the first cross on the line
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLine(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    while (!(leftValue > 800 && middleValue > 800 && rightValue > 800)) // follow line fowards until the first block position (all three sensors high)
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLine(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    robotMoving = false; // stop to be obvious as a check
    setLEDs();
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    delay(500);

    robotMoving = true;
    setLEDs();
    start = millis();
    while (millis() - start <= 2400) // follow line forwards for 2400ms (experimentally derived)
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLine(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    robotMoving = false; //stop
    setLEDs();
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    delay(500);

    stage_counter++; //increment stage counter
  }

  else if (stage_counter == 7) // find and collect third block and moe to the line
  {
    unsigned long int turnedTime1; // time from far right corner to RH side of third block
    unsigned long int turnedTime2; // time from RH side of block to LH side of block
    unsigned long int forwardTime; // time moved forwards until known distance from block
    int distanceThreshold = 24;    // if a lower value than this measured then it must be a block

    moveServo(myservo, 90); //set panel perpendicular to the ground for best distance measuring range

    robotMoving = true;
    setLEDs();
    unsigned long start = millis();
    while (millis() - start <= 1500) // turn for 1500ms to RH Corner of third block rectangle
    {
      leftMotor->setSpeed(200);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(BACKWARD);
    }
    robotMoving = false; //stop
    setLEDs();
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    delay(500);

    float volts = analogRead(pinIR) * 0.0048828125; // measure distance
    long int distance = 25.2 * pow(volts, -1);

    robotMoving = true;
    setLEDs();
    start = millis();
    while (distance >= distanceThreshold) // Turn until the distance is less than the threshold - ie robot is pointing at the RH edge of the block
    {
      leftMotor->setSpeed(200);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(FORWARD);

      float volts = analogRead(pinIR) * 0.0048828125; // value from IR * (5/1024)
      distance = 25.2 * pow(volts, -1);
    }
    turnedTime1 = millis() - start; // measure the time taken to turn from corner to RH edge of block

    robotMoving = false; // stop
    setLEDs();
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    delay(500);

    robotMoving = true;
    setLEDs();
    start = millis();
    while (distance < distanceThreshold) // Turn again until robot pointing at LH edge of block
    {
      leftMotor->setSpeed(200);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(FORWARD);

      float volts = analogRead(pinIR) * 0.0048828125;
      distance = 25.2 * pow(volts, -1);
    }

    turnedTime2 = millis() - start; //time taken to turn from RH to LH edge of the block

    delay(100);

    start = millis();
    while (millis() - start <= turnedTime2 / 2) // turn in the opposite direction for turnedTime/2 ms so robot is pointing at the centre of the block
    {
      leftMotor->setSpeed(200);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(BACKWARD);

      Serial.println(distance);
    }

    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    robotMoving = false;
    setLEDs();

    volts = analogRead(pinIR) * 0.0048828125; // value from IR * (5/1024)
    distance = 25.2 * pow(volts, -1);

    moveServo(myservo, 70); // set servo to correct measuring position

    robotMoving = true;
    setLEDs();
    start = millis();
    while (distance >= 14) // move towards block until it is a known distance away
    {
      float volts = analogRead(pinIR) * 0.0048828125;
      distance = 25.2 * pow(volts, -1);

      leftMotor->setSpeed(200);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(FORWARD);
    }
    forwardTime = millis() - start; // time taken to move from the line to the block

    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    robotMoving = false;
    setLEDs();

    Serial.println("BLOCK DETECTED");

    delay(100);

    moveServo(myservo, 0); // open front panel

    delay(100);

    robotMoving = true;
    setLEDs();
    leftMotor->setSpeed(200); // move forward for 1.5 seconds to encapsulate block
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(200);
    rightMotor->run(FORWARD);
    delay(1500);

    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    robotMoving = false;
    setLEDs();

    moveServo(myservo, 90); //close servo panel
    delay(1000);            //stop

    Serial.println("Doing squish test");
    doSquishTest(myservo, leftMotor, rightMotor); // sets capturedSoftBlock based on the result of the servo squish test
    block_captured = true;
    setLEDs();

    robotMoving = true;
    setLEDs();
    start = millis();
    while (millis() - start <= 1500 + forwardTime) // move backwards to initial position on the line for total time it went forwards
    {
      leftMotor->setSpeed(200);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(BACKWARD);
    }

    leftMotor->setSpeed(0); //stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    robotMoving = false;
    setLEDs();
    delay(100);

    robotMoving = true;
    setLEDs();
    start = millis();
    while (millis() - start <= (turnedTime2 / 2) + turnedTime1) // turn the robot back to the RH corner of the 3rd block area
    {
      leftMotor->setSpeed(200);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(BACKWARD);
    }

    leftMotor->setSpeed(0); //stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    robotMoving = false;
    setLEDs();
    delay(100);

    robotMoving = true;
    setLEDs();
    start = millis();
    while (millis() - start <= 1300) // turn back straight
    {
      leftMotor->setSpeed(200);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(FORWARD);
    }

    stage_counter++; // increment stage counter
  }

  else if (stage_counter == 8) // move back to the drop-off area
  {
    leftValue = digitalRead(leftPin) * 1000;
    middleValue = digitalRead(middlePin) * 1000;
    rightValue = digitalRead(rightPin) * 1000;

    robotMoving = true;
    setLEDs();
    unsigned long int start = millis();
    while (millis() - start <= 1500) // follow line foward for 1500 seconds to ensure that the robot is on the line - if is at an angle then moving forwards and then backwards will guarantee that it knows where the line is
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLine(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    start = millis(); // follow line in reverse using correction algorithm for 11 seconds
    while (millis() - start <= 11000)
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLineReverse(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    while (!(leftValue > 800 && middleValue > 800 && rightValue > 800)) // reverse using the simple line follow until all three line sensors read high - ie the robot is at the cross by the drop off area
    // two algorithms are needed as the main algorithm uses timed turns so the robot can miss the cross
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLineReverse2(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }
    leftMotor->setSpeed(0); // stop
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);

    robotMoving = false;
    setLEDs(); // beacon off

    delay(500);
    stage_counter++; // increment stage counter
  }

  else if (stage_counter == 9) // drop off third block and go back to line
  {
    if (capturedSoftBlock) // drop off block to correct zone based on which block it is
      turnLeft(leftMotor, rightMotor, myservo);
    else
      turnRight(leftMotor, rightMotor, myservo);

    setLEDs();
    stage_counter++; // increment the stage counter
  }

  else if (stage_counter == 10) // go back to the start area from the line
  {
    leftValue = digitalRead(leftPin) * 1000;
    middleValue = digitalRead(middlePin) * 1000;
    rightValue = digitalRead(rightPin) * 1000;

    robotMoving = true;
    setLEDs();
    while (!(leftValue > 800 && middleValue > 800 && rightValue > 800)) // move back to the edge of the start zone (all three line sensors read high)
    {
      leftValue = digitalRead(leftPin) * 1000;
      middleValue = digitalRead(middlePin) * 1000;
      rightValue = digitalRead(rightPin) * 1000;
      FollowLineReverse2(leftValue, middleValue, rightValue, leftMotor, rightMotor);
    }

    robotMoving = false; //stop
    setLEDs();
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    delay(500);

    robotMoving = true;
    setLEDs();
    unsigned long start = millis();
    while (millis() - start <= 3200) // move back 3200ms so the robot is fully inside the start area
    {
      leftMotor->setSpeed(200);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(BACKWARD);
    }

    robotMoving = false; //stop
    setLEDs();
    leftMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    rightMotor->run(FORWARD);

    stage_counter = -1; //set stage counter to -1 - the stop case
  }
}
