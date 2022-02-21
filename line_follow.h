#include <Adafruit_MotorShield.h>
#include <ServoTimer2.h>
#include "vars.h"
#include "servo_control.h"

int turnTime = 150; //how long to turn in each direction for the correction

void setLEDs()
{
  digitalWrite(orangePin, robotMoving); // set the orange led control pin to the value of robotMoving;
  if (capturedSoftBlock)
  {
    digitalWrite(redPin, block_captured); // if the robot thinks it has the soft block - set the red LED to the value of block_captured, i.e. high if the robot has a block and low if it doesnt
  }
  else if (!capturedSoftBlock)
  {
    digitalWrite(greenPin, block_captured); // same for the blue LED
  }
}

int FollowLine(int left, int middle, int right, Adafruit_DCMotor *leftMotor, Adafruit_DCMotor *rightMotor)
// Follow line forwards with correction
{
  if (middle > 800 && left < 200 && right < 200) // middle high so go straight
  {
    leftMotor->setSpeed(255);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(255);
    rightMotor->run(FORWARD);
    turning_var = 0;
  }

  else if (left > 800 && middle < 200 && right < 200) // left high only so turn a little to the right and then turn back to straight
  {
    //    Serial.println("turn left");
    unsigned long start = millis();
    while (millis() - start <= 2 * turnTime)
    {
      leftMotor->setSpeed(150);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(230);
      rightMotor->run(FORWARD);
    }
    start = millis();
    while (millis() - start <= turnTime)
    {
      leftMotor->setSpeed(200);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(150);
      rightMotor->run(FORWARD);
    }
    turning_var = 1;
  }

  else if (right > 800 && middle < 200 && left < 200) // right high only so turn a little to the left and then turn back to straight
  {
    unsigned long start = millis();
    while (millis() - start <= 2 * turnTime)
    {
      leftMotor->setSpeed(230);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(150);
      rightMotor->run(FORWARD);
    }
    start = millis();
    while (millis() - start <= turnTime)
    {
      leftMotor->setSpeed(150);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(FORWARD);
    }
    turning_var = 2;
  }

  else if (left > 800 && middle > 800 && right > 800) // all three high so just go straight
  {
    Serial.println("straight - all high");
    leftMotor->setSpeed(255);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(255);
    rightMotor->run(FORWARD);
    turning_var = 0;
  }

  else if (left < 200 && middle < 200 && right < 200) // all three sensors low
  {
    if (turning_var == 0) // if robot was last going straight then keep going straight
    {
      rightMotor->setSpeed(255);
      rightMotor->run(FORWARD);
      leftMotor->setSpeed(255);
      leftMotor->run(FORWARD);
    }

    if (turning_var == 1) // if robot was last needing to go right then keep doing that
    {
      unsigned long start = millis();
      while (millis() - start <= 2 * turnTime)
      {
        leftMotor->setSpeed(150);
        leftMotor->run(FORWARD);
        rightMotor->setSpeed(230);
        rightMotor->run(FORWARD);
      }
      start = millis();
      while (millis() - start <= turnTime)
      {
        leftMotor->setSpeed(200);
        leftMotor->run(FORWARD);
        rightMotor->setSpeed(150);
        rightMotor->run(FORWARD);
      }
    }

    if (turning_var == 2) // if robot was last needing to go left then keep doing that
    {
      unsigned long start = millis();
      while (millis() - start <= 2 * turnTime)
      {
        leftMotor->setSpeed(230);
        leftMotor->run(FORWARD);
        rightMotor->setSpeed(150);
        rightMotor->run(FORWARD);
      }
      start = millis();
      while (millis() - start <= turnTime)
      {
        leftMotor->setSpeed(150);
        leftMotor->run(FORWARD);
        rightMotor->setSpeed(200);
        rightMotor->run(FORWARD);
      }
    }
  }

  else // catchall statement - just go straight
  {
    leftMotor->setSpeed(230);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(230);
    rightMotor->run(FORWARD);
    turning_var = 0;
  }
}

int FollowLineReverse(int left, int middle, int right, Adafruit_DCMotor *leftMotor, Adafruit_DCMotor *rightMotor) //exact mirror of the forward line following code with the directions of the motors reversed
{
  if (middle > 800 && left < 200 && right < 200)
  {
    //    Serial.println("straight");
    leftMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(255);
    rightMotor->run(BACKWARD);
    turning_var = 0;
  }

  else if (left > 800 && middle < 200 && right < 200)
  {
    unsigned long start = millis();
    while (millis() - start <= 2 * turnTime)
    {
      leftMotor->setSpeed(150);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(230);
      rightMotor->run(BACKWARD);
    }
    start = millis();
    while (millis() - start <= 1 * turnTime)
    {
      leftMotor->setSpeed(200);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(150);
      rightMotor->run(BACKWARD);
    }
    turning_var = 1;
  }

  else if (right > 800 && middle < 200 && left < 200)
  {
    unsigned long start = millis();
    while (millis() - start <= 2 * turnTime)
    {
      leftMotor->setSpeed(230);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(150);
      rightMotor->run(BACKWARD);
    }
    start = millis();
    while (millis() - start <= 1 * turnTime)
    {
      leftMotor->setSpeed(150);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(BACKWARD);
    }
    turning_var = 2;
  }

  else if (left > 800 && middle > 800 && right > 800)
  {
    //    Serial.println("straight");
    leftMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(255);
    rightMotor->run(BACKWARD);
    turning_var = 0;
  }

  else if (left < 200 && middle < 200 && right < 200)
  {
    if (turning_var == 0)
    {
      rightMotor->setSpeed(255);
      rightMotor->run(BACKWARD);
      leftMotor->setSpeed(255);
      leftMotor->run(BACKWARD);
    }

    if (turning_var == 1)
    {
      unsigned long start = millis();
      while (millis() - start <= 2 * turnTime)
      {
        leftMotor->setSpeed(150);
        leftMotor->run(BACKWARD);
        rightMotor->setSpeed(230);
        rightMotor->run(BACKWARD);
      }
      start = millis();
      while (millis() - start <= turnTime)
      {
        leftMotor->setSpeed(200);
        leftMotor->run(BACKWARD);
        rightMotor->setSpeed(150);
        rightMotor->run(BACKWARD);
      }
    }

    if (turning_var == 2)
    {
      unsigned long start = millis();
      while (millis() - start <= 2 * turnTime)
      {
        leftMotor->setSpeed(230);
        leftMotor->run(BACKWARD);
        rightMotor->setSpeed(150);
        rightMotor->run(BACKWARD);
      }
      start = millis();
      while (millis() - start <= turnTime)
      {
        leftMotor->setSpeed(150);
        leftMotor->run(BACKWARD);
        rightMotor->setSpeed(200);
        rightMotor->run(BACKWARD);
      }
    }
  }

  else
  {
    leftMotor->setSpeed(230);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(230);
    rightMotor->run(BACKWARD);
    turning_var = 0;
  }
}

int FollowLineReverse2(int left, int middle, int right, Adafruit_DCMotor *leftMotor, Adafruit_DCMotor *rightMotor) //version of the line following code without correction to improve repeatability of the sensing of all three line sensors high
{
  if (middle > 800 && left < 200 && right < 200)
  {
    rightMotor->setSpeed(255);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    turning_var = 0;
  }

  else if (left > 800 && middle < 200 && right < 200) // now if only the left (looking forward) sensor high turn to the left hand side of the robot (looking forward) slowly
  {
    leftMotor->setSpeed(150);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(200);
    rightMotor->run(BACKWARD);
    turning_var = 1;
  }

  else if (right > 800 && middle < 200 && left < 200) // now if only the right (looking forward) sensor high turn to the right hand side of the robot (looking forward) slowly
  {
    leftMotor->setSpeed(200);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(150);
    rightMotor->run(BACKWARD);
    turning_var = 2;
  }

  else if (left > 800 && middle > 800 && right > 800)
  {
    rightMotor->setSpeed(255);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    turning_var = 0;
  }

  else if (left < 200 && middle < 200 && right < 200) // also changed to reflect the different turning system
  {
    if (turning_var == 0)
    {
      rightMotor->setSpeed(255);
      rightMotor->run(BACKWARD);
      leftMotor->setSpeed(255);
      leftMotor->run(BACKWARD);
    }

    if (turning_var == 1)
    {
      leftMotor->setSpeed(150);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(200);
      rightMotor->run(BACKWARD);
    }

    if (turning_var == 2)
    {
      leftMotor->setSpeed(200);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(150);
      rightMotor->run(BACKWARD);
    }
  }

  else
  {
    leftMotor->setSpeed(200);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(200);
    rightMotor->run(BACKWARD);
    turning_var = 0;
  }
}

void turnLeft(Adafruit_DCMotor *leftMotor, Adafruit_DCMotor *rightMotor, ServoTimer2 servo) // code to drop off a block in the left hand (soft) drop off area
{
  robotMoving = true;
  digitalWrite(orangePin, robotMoving); //orange beacon
  setLEDs();                            // led signals

  unsigned long start = millis();
  while (millis() - start <= 3000) // follow line backwards for 3 seconds
  {
    leftValue = digitalRead(leftPin) * 1000;
    middleValue = digitalRead(middlePin) * 1000;
    rightValue = digitalRead(rightPin) * 1000;
    FollowLineReverse(leftValue, middleValue, rightValue, leftMotor, rightMotor);
  }

  leftMotor->setSpeed(200); // turn to the left
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(200);
  rightMotor->run(FORWARD);
  delay(1700);

  leftMotor->setSpeed(0); //stop
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(FORWARD);
  delay(100);

  leftMotor->setSpeed(200); // go forwards into box
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(200);
  rightMotor->run(FORWARD);
  delay(2500);

  leftMotor->setSpeed(0); //stop
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(FORWARD);

  robotMoving = false;
  digitalWrite(orangePin, robotMoving); // orange beacon and LEDs
  setLEDs();
  delay(500);

  moveServo(servo, 0); // open panel
  delay(1000);

  robotMoving = true;
  digitalWrite(orangePin, robotMoving); //robot is now moving
  setLEDs();

  leftMotor->setSpeed(200);
  leftMotor->run(BACKWARD); // reverse until between drop-off area and white line
  rightMotor->setSpeed(200);
  rightMotor->run(BACKWARD);
  delay(2000);

  leftValue = digitalRead(leftPin) * 1000;
  while (leftValue <= 200) // reverse until the left line sensor is over the line
  {
    leftMotor->setSpeed(200);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(200);
    rightMotor->run(BACKWARD);
    leftValue = digitalRead(leftPin) * 1000;
  }

  block_captured = false; // block is no longer captured so turn off LEDs
  setLEDs();

  leftMotor->setSpeed(200);
  leftMotor->run(FORWARD); // turn back to straight
  rightMotor->setSpeed(200);
  rightMotor->run(BACKWARD);
  delay(1850);

  robotMoving = false;
  digitalWrite(orangePin, robotMoving); // stop robot and set LEDs
  setLEDs();
  leftMotor->setSpeed(0);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(FORWARD);
  delay(1000);

  moveServo(servo, 70); // reset front panel
}

void turnRight(Adafruit_DCMotor *leftMotor, Adafruit_DCMotor *rightMotor, ServoTimer2 servo) // a mirror image of the previous code to drop off a block to the right hand zone
{
  robotMoving = true;
  digitalWrite(orangePin, robotMoving);
  setLEDs();
  unsigned long start = millis();

  while (millis() - start <= 3000)
  {
    leftValue = digitalRead(leftPin) * 1000;
    middleValue = digitalRead(middlePin) * 1000;
    rightValue = digitalRead(rightPin) * 1000;
    FollowLineReverse(leftValue, middleValue, rightValue, leftMotor, rightMotor);
  }
  leftMotor->setSpeed(200);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(200);
  rightMotor->run(BACKWARD);
  delay(1700);

  leftMotor->setSpeed(0);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(FORWARD);
  delay(100);

  leftMotor->setSpeed(200);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(200);
  rightMotor->run(FORWARD);
  delay(2500);

  leftMotor->setSpeed(0);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(FORWARD);
  robotMoving = false;
  digitalWrite(orangePin, robotMoving);
  setLEDs();
  delay(500);

  moveServo(servo, 0);
  delay(1000);

  robotMoving = true;
  digitalWrite(orangePin, robotMoving);
  setLEDs();
  leftMotor->setSpeed(200);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(200);
  rightMotor->run(BACKWARD);
  delay(2000);

  rightValue = digitalRead(rightPin) * 1000;
  while (rightValue <= 200)
  {
    leftMotor->setSpeed(200);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(200);
    rightMotor->run(BACKWARD);
    rightValue = digitalRead(rightPin) * 1000;
  }

  block_captured = false;
  setLEDs();

  leftMotor->setSpeed(200);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(200);
  rightMotor->run(FORWARD);
  delay(1500);

  robotMoving = false;
  digitalWrite(orangePin, robotMoving);
  setLEDs();
  leftMotor->setSpeed(0);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(FORWARD);
  delay(1000);
  moveServo(servo, 70);
}
