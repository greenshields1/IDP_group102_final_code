// Servo Pin
#define servoPin 9

// Pin for start/control button
#define startPin 8

// Line Sensor Pins
#define leftPin 7
#define middlePin 6
#define rightPin 5

//IR distance sensor pin
#define pinIR A3

//Analogue pin for current sensing
#define currentSensePin A0

// LED pins
#define orangePin 13
#define redPin 11
#define greenPin 12

// Global Variables
extern int turning_var;        // stores what the robot was last doing when following the line in case it goes off the line
extern bool block_captured;    // boolean for if the robot is current carrying a block
extern int stage_counter;      // stores the current stage that the robot is in
extern bool capturedSoftBlock; // boolean for which block has been captured
extern bool robotMoving;       // boolean for if the robot is moving
extern int last;               // stores the last servo position moved to
extern int leftValue;          // stores the measured value of the left line sensor
extern int middleValue;        // stores the measured value of the middle line sensor
extern int rightValue;         // stores the measured value of the right line sensor
