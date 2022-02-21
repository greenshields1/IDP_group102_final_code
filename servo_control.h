#include "vars.h"
#include <ServoTimer2.h>
#include <Adafruit_MotorShield.h>

int toMillis(int degrees) // function to convert a desired servo angle to the correct pulse width in milliseconds
{
  return map(degrees, 0, 180, 750, 2250);
}

int moveServo(ServoTimer2 servo, int desired) // function to move the servo in a controlled way to a new position
// global variable 'last' is kept up to date for the last position the servo was moved to
{
  if (last < desired & desired <= 140) // conditional needed to ensure sweep is in correct direction in for loop
  {
    for (int i = last; i <= desired; i += 1) // sweep in one degree increments to the desired position
    {
      servo.write(toMillis(i));
      delay(2);
    }
  }
  else if (last > desired & desired >= 0)
  {
    for (int i = last; i >= desired; i -= 1)
    {
      servo.write(toMillis(i));
      delay(2);
    }
  }
  last = desired; // set global variable last to the position just moved to
}

float senseCurrent() // reads the voltage on the currentSensePin, which is the voltage across the current shunt for the servo
{
  int raw = analogRead(currentSensePin);
  return raw;
}

void doSquishTest(ServoTimer2 servo, Adafruit_DCMotor *leftMotor, Adafruit_DCMotor *rightMotor)
{
  float current = 0;    // measured current (actually the analogue voltage reading)
  long int sum = 0;     // stores the sum of the measured currents
  long int counter = 0; // counts the current readings
  float average = 0;    // stores the average current
  int position = 95;    // starting position of the servo

  int threshold = 185; // threshold voltage across the current shunt that we are looking for

  moveServo(servo, 90); // ensure servo panel is closed

  long unsigned int start = millis(); // turn to the right for 0.5 seconds to ensure that the block is in the back left corner of the block area of the robot
  while (millis() - start <= 500)
  {
    leftMotor->setSpeed(255);
    leftMotor->run(FORWARD);
    rightMotor->setSpeed(255);
    rightMotor->run(BACKWARD);
  }
  leftMotor->setSpeed(0); // stop
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(BACKWARD);

  while (average <= threshold) // while the average measured voltage is less than the threshold set (decided experimentally)
  {
    position++;                   // increment position
    moveServo(servo, position);   // move to position
    for (int i = 0; i < 500; i++) // do 500 times:
    {
      sum += senseCurrent(); // add the measured voltage to the sum
      counter++;             // increment the counter
      delay(1);              // delay for 1ms in order for the current to be measured over a useful amount of time
    }
    average = sum / counter; // calculate average
  }

  // 'position' now stores the servo position reached for the current to reach a set value

  if (position >= 115)
  {
    capturedSoftBlock = true; // more porous/softer block has a lower youngs modulus so requires a larger strain (bigger servo position) for the same force on the servo (related to the current)
  }
  else
  {
    capturedSoftBlock = false;
  }
  moveServo(servo, 90); // reset servo

  start = millis();
  while (millis() - start <= 500) //turn back straight
  {
    leftMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(255);
    rightMotor->run(FORWARD);
  }
  leftMotor->setSpeed(0);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(0);
  rightMotor->run(BACKWARD);
  delay(500);
}
