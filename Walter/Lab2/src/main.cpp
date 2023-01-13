/*&StateMachine.ino
  Author: Carlotta. A. Berry
  Date: December 3, 2016
  This program will provide a template for an example of implementing a behavior-based control architecture
  for a mobile robot to implement obstacle avoidance and random wander. There are many ways to create a state machine
  and this is just one. It is to help get you started and brainstorm ideas, you are not required to use it.
  Feel free to create your own version of state machine.

  The flag byte (8 bits) variable will hold the IR and sonar data [X X snrRight snrLeft irLeft irRight irRear irFront]
  The state byte (8 bits) variable will hold the state information as well as motor motion [X X X wander runAway collide rev fwd]

  Use the following functions to read, clear and set bits in the byte
  bitRead(state, wander)) { // check if the wander state is active
  bitClear(state, wander);//clear the the wander state
  bitSet(state, wander);//set the wander state

  Hardware Connections:
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  Front IR    A0
  Back IR     A1
  Right IR    A2
  Left IR     A3
  Left Sonar  A8
  Right Sonar A9
  Pushbutton  A15
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <stdlib.h>
#include <time.h>


//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time


//state LEDs connections
#define blueLED 5             //blue LED for displaying states
#define grnLED 6              //green LED for displaying states
#define ylwLED 7              //yellow LED for displaying states
//define stepper motor constants
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define test_led 13 //test led to test interrupt heartbeat

#define robot_spd 1500 //set robot speed
#define max_accel 10000//maximum robot acceleration
#define max_spd 2500//maximum robot speed

#define quarter_rotation 200  //stepper quarter rotation
#define half_rotation 400     //stepper half rotation
#define one_rotation  800     //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  1600    //stepper motor 2 rotations
#define three_rotation 2400   //stepper rotation 3 rotations
#define four_rotation 3200    //stepper rotation 3 rotations
#define five_rotation 4000    //stepper rotation 3 rotations

//define IR sensor connections
#define irFront A0 //front IR analog pin
#define irRear  A1//back IR analog pin
#define irRight A2 //right IR analog pin
#define irLeft  A3 //left IR analog pin
#define button A15 //pushbutton

//define sonar connections
#define snrLeft 8  //front left sonar 
#define snrRight 9 //front right sonar 
long SNRDist;
long SNLDist;

NewPing SNL(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing SNR(snrRight, snrRight);  //create an instance of the right sonar

#define irThresh    5 //in inches // The IR threshold for presence of an obstacle - 400 raw value
#define snrThresh   5   // The sonar threshold for presence of an obstacle
#define minThresh   0   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 9600//set serial communication baud rate

int irFrontArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
int irFrontAvg;  //variable to hold average of current front IR reading
int irLeftAvg;   //variable to hold average of current left IR reading
int irRearAvg;   //variable to hold average of current rear IR reading
int irRightAvg;   //variable to hold average of current right IR reading
int irIdx = 0;//index for 5 IR readings to take the average

int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right sonar readings
int srIdx = 0;//index for 5 sonar readings to take the average
int srLeft;   //variable to hold average of left sonar current reading
int srRight;  //variable to hold average or right sonar current reading
int srLeftAvg;  //variable to holde left sonar data
int srRightAvg; //variable to hold right sonar data

volatile boolean test_state; //variable to hold test led state for timer interrupt

int obsIR[4] = {0, 0, 0, 0};
int obsSonar[2] = {0, 0};
//flag byte to hold sensor data
volatile byte flag = 0;    // Flag to hold IR & Sonar data - used to create the state machine

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip
#define obFLeft   4 // Left Sonar trip
#define obFRight  5 // Right Sonar trip

int count; //count number of times collide has tripped
#define max_collide 250 //maximum number of collides before robot reverses

//state byte to hold robot motion and state data
volatile byte state = 0;   //state to hold robot states and motor motion

//bit definitions for robot motion and state byte
#define fwd     0
#define rev     1
#define collide 2
#define runAway 3
#define wander  4

//define layers of subsumption architecture that are active
byte layers = 2; //[wander runAway collide]
//bit definitions for layers
#define cLayer 0
#define rLayer 1
#define wLayer 2

#define timer_int 250000 // 1/2 second (500000 us) period for timer interrupt

//define encoder pins
#define LEFT 0                            //left encoder
#define RIGHT 1                           //right encoder
#define ltEncoder 18                      //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
#define rtEncoder 19                      //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long int encoder[2] = {0, 0};    //interrupt variable to hold number of encoder counts (left, right)
long int lastSpeed[2] = {0, 0};           //variable to hold encoder speed (left, right)
long int accumTicks[2] = {0, 0};          //variable to hold accumulated ticks since last reset

//Robot Constants
const float WIDTH_BOT = 23.3; //cm
const float RADIUS_BOT = 11.7; //cm
const float CM_TO_STEPS = 29.95;//25.6; 
const float TICKS_TO_STEPS = 20.0;
const float STEPS_TO_TICKS = 1.0/20.0;

//Random Wander movement list
/**
 * 0 - Stop
 * 1 - Move forward
 * 2 - Move backward
 * 3 - Turn forward left
 * 4 - Turn forward right
 * 5 - Turn backward left
 * 6 - Turn backward right
 * 7 - Spin right
 * 8 - Change motor speed
 * 9 - 
 * 10 - 
*/
short wanderList[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

boolean aggressive; //boolean to set robot to aggressive kid
int avoidVector[2];

float leftSonarToInches(float pulse) {
  return 0.0068*pulse - 0.6529;
}

float rightSonarToInches(float pulse) {
  return 0.0067*pulse - 0.3554;

}

// Helper Functions
/**
 * Interrupt function to count left encoder ticks.
 */
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

/**
 * Interrupt function to count right encoder ticks
 */
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}


/**
 * Function to set both wheel speeds
 * 
 * @param rSpeed The speed of the right stepper
 * @param lSpeed The speed of the left stepper
*/
void setBothStepperSpeed(float rSpeed, float lSpeed) {
  stepperRight.setMaxSpeed(rSpeed); //set right motor speed
  stepperLeft.setMaxSpeed(lSpeed); //set left motor speed
}

/**
 * Function to set both wheel positions
 * 
 * @param rPos The position of the right stepper
 * @param lPos The position of the left stepper
*/
void setBothStepperCurrentPosition(long rPos, long lPos) {
  stepperRight.setCurrentPosition(rPos); //set right wheel position to zero
  stepperLeft.setCurrentPosition(lPos); //set left wheel position to zero
}

/**
 * Function to run both wheels to a position at speed
 */
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/**
 * Function to run both wheels continuously at a speed
 */
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*
   This is a sample updateIR() function, the description and code should be updated to take an average, consider all sensor and reflect
   the necesary changes for the lab requirements.
*/

void updateIR() {
  int front = 0, back = 0, left = 0, right = 0;
  for (int i = 0;i < 5;i++) {
    front += analogRead(irFront);
    back += analogRead(irRear);
    left += analogRead(irLeft);
    right += analogRead(irRight);
  }
  front /= 5; front = 2080*pow(front,-1.198);
  back /= 5; back = 2940.1*pow(back, -1.25);
  left /= 5; left = 27664*pow(left, -1.525);
  right /= 5; right = 35047*pow(right,-1.569);

  irFrontAvg = front;
  irRearAvg = back;
  irLeftAvg = left;
  irRightAvg = right;
  //  print IR data
  
    Serial.println("frontIR\tbackIR\tleftIR\trightIR");
    Serial.print(front); Serial.print("\t");
    Serial.print(back); Serial.print("\t");
    Serial.print(left); Serial.print("\t");
    Serial.println(right);
  
  if (right < irThresh) {
    bitSet(flag, obRight);//set the right obstacle
    obsIR[0] = 1;
  } else
    bitClear(flag, obRight);//clear the right obstacle
    obsIR[0] = 0;
  if (left < irThresh) {
    bitSet(flag, obLeft);//set the left obstacle
    obsIR[1] = 1;
  } else
    bitClear(flag, obLeft);//clear the left obstacle
    obsIR[1] = 0;
  if (front < irThresh) {
    bitSet(flag, obFront);//set the front obstacle
    obsIR[2] = 0;
  }
  else
    bitClear(flag, obFront);//clear the front obstacle
    obsIR[2] = 1;
  if (back < irThresh) {
    bitSet(flag, obRear);//set the back obstacle
    obsIR[3] = 1;
  } else {
    bitClear(flag, obRear);//clear the back obstacle
    obsIR[3] = 0;
  }
}

/*
   This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
   the necesary changes for the lab requirements.
*/
void updateSonar() {
  long left, right;
  //read right sonar
  right = 0;
  for (int i = 0;i < 5;i++) {
    pinMode(snrRight, OUTPUT);//set the PING pin as an output
    digitalWrite(snrRight, LOW);//set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    digitalWrite(snrRight, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    digitalWrite(snrRight, LOW);//set pin low first again
    pinMode(snrRight, INPUT);//set pin as input with duration as reception
    right += pulseIn(snrRight, HIGH);//measures how long the pin is high

  }
  right /= 5;
  right = rightSonarToInches(right);
  srRightAvg = right;

  //read left sonar
  left = 0;
  for (int i = 0;i < 5;i++) {
    pinMode(snrLeft, OUTPUT);//set the PING pin as an output
    digitalWrite(snrLeft, LOW);//set the PING pin low first
    delayMicroseconds(2);//wait 2 us
    digitalWrite(snrLeft, HIGH);//trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);//wait 5 us
    digitalWrite(snrLeft, LOW);//set pin low first again
    pinMode(snrLeft, INPUT);//set pin as input with duration as reception
    left += pulseIn(snrLeft, HIGH);//measures how long the pin is high
  }
  left /= 5;
  left = leftSonarToInches(left);
  srLeftAvg = left;
  //  print sonar data
      //Serial.println("leftSNR\trightSNR");
      //Serial.print(left); Serial.print("\t");
      //Serial.println(right);
  if (right < snrThresh) {
    bitSet(flag, obFRight);//set the front right obstacle
    obsSonar[0] = 1;
  } else
    bitClear(flag, obFRight);//clear the front right obstacle
    obsSonar[0] = 0;
  if (left < snrThresh) {
    bitSet(flag, obFLeft);//set the front left obstacle
    obsSonar[1] = 1;
  } else {
    bitClear(flag, obFLeft);//clear the front left obstacle
    obsSonar[1] = 0;
  }
}

/*
   This is a sample updateState() function, the description and code should be updated to reflect the actual state machine that you will implement
   based upon the the lab requirements.
*/
void updateState() {
  if (!(flag)) { //no sensors triggered
    bitSet(state, fwd); //set forward motion
    bitClear(state, collide);//clear collide state
    count--;//decrement collide counter
  } else if (flag & 0b0) { //front sensors triggered
    bitClear(state, fwd); //clear reverse motion
    bitSet(state, collide);//set collide state
  } else if (flag & 0b10) { //rear sensors triggered
    bitClear(state, fwd); //clear reverse motion
    bitSet(state, collide);//set collide state
  } else if (flag & 0b11) { //Right sensors triggered
    bitClear(state, fwd); //clear reverse motion
    bitSet(state, collide);//set collide state
  } else if (flag & 0b100) { //Left
    bitClear(state, fwd); //clear reverse motion
    bitSet(state, collide);//set collide state
  }
  //print flag byte
  //    Serial.println("\trtSNR\tltSNR\tltIR\trtIR\trearIR\tftIR");
  //    Serial.print("flag byte: ");
  //    Serial.println(flag, BIN);
  //print state byte
  //    Serial.println("\twander\trunAway\tcollide\treverse\tforward");
  //    Serial.print("state byte: ");
  //    Serial.println(state, BIN);
}

/*
  This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
  to meet the lab requirements.
*/
void updateSensors() {
  //  Serial.print("updateSensors\t");
  //  Serial.println(test_state);
  //test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  //digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  test_state = !test_state;
  digitalWrite(test_led, test_state);
  flag = 0;       //clear all sensor flags
  state = 0;      //clear all state flags
  updateIR();     //update IR readings and update flag variable and state machine
  updateSonar();  //update Sonar readings and update flag variable and state machine
  updateState();  //update State Machine based upon sensor readings
  delay(1000);     //added so that you can read the data on the serial monitor
}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}


/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  while (runNow) {
    if (!stepperRight.run() ) {
      runNow = 0;
      stop();
    }
    if (!stepperLeft.run()) {
      runNow = 0;
      stop();
    }
  }
}

void forward(int rot) {
  long positions[2]; // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() + rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot;   //left motor absolute position
  stepperRight.move(positions[0]);  //move right motor to position
  stepperLeft.move(positions[1]);   //move left motor to position
  runToStop();//run until the robot reaches the target
}

/**
 * Moves robot forward a distance based on the distance input.
 * 
 * @param distance A value in centimeters
*/
void forwardDist(int distance) {
  encoder[LEFT] = 0;
  encoder[RIGHT] = 0;

  float steps = distance * CM_TO_STEPS;
  float ticks = steps * STEPS_TO_TICKS;
  
  stepperRight.moveTo(steps); //move one full rotation forward relative to current position
  stepperLeft.moveTo(steps); //move one full rotation forward relative to current position
  setBothStepperSpeed(500, 500); //set stepper speeds
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motor

  runToStop(); //run until the robot reaches the target
/*
  int errorLeft = ticks - encoder[LEFT];
  int errorRight = ticks - encoder[RIGHT];

  int correction = TICKS_TO_STEPS * max(errorLeft, errorRight);
  setBothStepperCurrentPosition(0, 0); //reset stepper positions
  stepperRight.moveTo(correction); //move one full rotation forward relative to current position
  stepperLeft.moveTo(correction); //move one full rotation forward relative to current position
  setBothStepperSpeed(250, 250); //set stepper speeds
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motor
  runToStop();
  */
}

/**
 * This function turns the robot forward a quarter circle with a given radius in one direction.
 * 
 * @param radius A turning radius in centimeters. A radius of 0 spins the robot in the clockwise direction 90 degrees (similar to calling spin() a quarter circle).
 * @param turnDirection An array of direction values
 *                        position 0: a forward [0] or reverse [1] turn
 *                        position 1: a left [0] or right [1] turn
 */
void turn(float radius, bool turnDirection[2]) {
  /*
    seems to me that once a radius is given, the robot doesn't make the exact radius turn.
    Steps I took:
      1. I input a 15cm CW turn
      2. Measure 15cm out from the center of the wheel
      3. Start the robot, call the turn function, and stop the robot
      5. Attempt to measure 15cm out from the center of the wheel
      6. The robot is off by exactly 1.5cm
      7. The robot is also crooked and isn't completing the turn properly
    
    Possible Solutions:
      1. A proportional gain must be placed to counter the error
      2. Count how many ticks are required for the robot to move forward and then turn
      3. Have outer turn proportionally less than inner
  */

  setBothStepperCurrentPosition(0, 0); //reset stepper positions
  setBothStepperSpeed(400, 400); //set motor speeds

  //float diam = 60.0*radius/18; //60 steps per 18cm
  float inner = 2.0 * 3.14 * (radius*1.2 - WIDTH_BOT/2.0) * (90.0/360.0) *0.95;
  float outer = 2.0 * 3.14 * (radius*1.2 + WIDTH_BOT/2.0) * (90.0/360.0) *0.95;

  int innerSteps = inner * 29.958;
  int outerSteps = outer * 29.958;

  long positions[2]; // Array of desired stepper positions
  
  // Check the turn direction (left or right, and forward or backward)
  if (turnDirection[0] == 0 && turnDirection[1] == 0) {
    positions[0] = outerSteps; //right motor absolute position
    positions[1] = innerSteps; //left motor absolute position
    steppers.moveTo(positions);
  }
  else if (turnDirection[0] == 0 && turnDirection[1] == 1) {
    positions[0] = innerSteps; //right motor absolute position
    positions[1] = outerSteps; //left motor absolute position
    steppers.moveTo(positions);
  }
  else if (turnDirection[0] == 1 && turnDirection[1] == 0) {
    positions[0] = -outerSteps; //right motor absolute position
    positions[1] = -innerSteps; //left motor absolute position
    steppers.moveTo(positions);
  }
  else {
    positions[0] = -innerSteps; //right motor absolute position
    positions[1] = -outerSteps; //left motor absolute position
    steppers.moveTo(positions);
  }

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/**
 Calculates arc length needed to turn to angle and calls multistepper to turn robot
 After turning, encoders are used to error check and move robot again if it moved too much or too little
*/
void goToAngle(float angle) {
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  encoder[LEFT] = 0;
  encoder[RIGHT] = 0;
  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, LOW); //turn off yellow LED
  float arclength = 2.0 * PI * RADIUS_BOT * (angle/360.0) * 0.89;
  int steps = arclength * CM_TO_STEPS;
  int ticks = steps * STEPS_TO_TICKS;
  ticks = abs(ticks);

  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);
  /*
  Serial.print("Steps: ");
  Serial.println(steps);
  Serial.print("Ticks: ");
  Serial.println(ticks);
  */

  setBothStepperSpeed(400, 400); //set motor speeds
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motorw
  runToStop();
  
  /*
  Serial.print("Left Encoder: ");
  Serial.println(encoder[LEFT]);
  Serial.println("Right Endoder: ");
  Serial.println(encoder[RIGHT]);
  int errorLeft = ticks - encoder[LEFT];
  int errorRight = ticks - encoder[RIGHT];
  Serial.print("Error Left: ");
  Serial.println(errorLeft);
  Serial.print("Error Right: ");
  Serial.println(errorRight);

  int correctStepsLeft = errorLeft * TICKS_TO_STEPS;
  int correctStepsRight = errorRight * TICKS_TO_STEPS;

  setBothStepperCurrentPosition(0, 0); //reset stepper positions
  Serial.print("Correction steps: ");
  Serial.println(correctStepsLeft);
  stepperRight.moveTo(-correctStepsRight);
  stepperLeft.moveTo(correctStepsLeft);
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motor

  runToStop(); //run until the robot reaches the target
  */
}

/**
 Give the robot x y coordiantes in cm 
 Robot will then call goToAngle and spin to a direction that points to the coordiante
 Will then go calcualted distance to reach coordinate
*/
void goToGoal(float x, float y) {
  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, HIGH); //turn on yellow LED

  double radians = atan2(y, x); //returns angle to x, y in radians
  
  float angle = (radians * 180.0/PI);
  //Serial.print("Degrees: ");
  //Serial.println(angle);
  goToAngle(angle);
  float distance = sqrt((x*x) + (y*y));

  forwardDist(distance); 

}

/*
  This function takes the sensor averages and adds them as vectors to if they meet the obstacle threshold.
  This sets avoidVector to have the vector to pass to goToGoal
*/
void randomWander() {
  int direction = random(-1, 1);
}

/*
  Function executes the aggressive kid behavior, which is stopping at an obstacle and turning on the blue led;
*/
void aggressiveKid() {
  digitalWrite(blueLED, HIGH); //turn on blue LED
  stop();
}

/*
  This function takes the sensor averages and adds them as vectors to if they meet the obstacle threshold.
  This sets avoidVector to have the vector to pass to goToGoal
*/
void avoid() {
  //updateSensors();
  avoidVector[0] = 0;
  avoidVector[1] = 0;
  if (irFrontAvg < irThresh) {
    avoidVector[0] -= 1;//irFrontAvg;
    avoidVector[1] -= 0;
  }
  if (irRearAvg < irThresh) {
    avoidVector[0] += 1;//irRearAvg;
    avoidVector[1] += 0;
  }
  if (irLeftAvg < irThresh) {
    avoidVector[0] -= 0;
    avoidVector[1] -= 1;//irLeftAvg;
  }
  if (irRightAvg < irThresh) {
    avoidVector[0] += 0;
    avoidVector[1] += 1;//irRightAvg;
  }
  if (srRightAvg < snrThresh) {
    avoidVector[0] += 0;//srRightAvg* cos(45);
    avoidVector[1] += 1;//srRightAvg;//* sin(45);
  } 
  if (srLeftAvg < snrThresh) {
    avoidVector[0] += 0;//srLeftAvg* cos(-45);
    avoidVector[1] -= 1;//srLeftAvg;//* sin(-45);
  }
} 

/*
   This is a sample robotMotion() function, the description and code should be updated to reflect the actual robot motion function that you will implement
   based upon the the lab requirements.  Some things to consider, you cannot use a blocking motor function because you need to use sensor data to update
   movement.  You also need to continue to poll    the sensors during the motion and update flags and state because this will serve as your interrupt to
   stop or change movement.
*/
void robotMotion() {
  if ((flag & 0b01) && aggressive) { //check for a collide state - aggressive kid
    aggressiveKid();
    Serial.println("robot stop"); 
  } else if (bitRead(state, collide)) {
    avoid();
    Serial.println("robot avoid");
    double radians = atan2(avoidVector[0], avoidVector[1]); //returns angle to x, y in radians
    float angle = (radians * 180.0/PI);
    //Serial.println(angle);
    //goToGoal(sqrt(pow(avoidVector[0], 2) + pow(avoidVector[1], 2)), angle);
    Serial.println(angle);
    Serial.println(avoidVector[0]);
    Serial.println(avoidVector[1]);
    goToAngle(angle);
    runToStop();
    //goToGoal(avoidVector[0], avoidVector[1]); //should avoid properly, may need to adjust magnitude but angle should be correct
  } 
  else{
    //Serial.println("robot forward");
    //forward(one_rotation);//move forward as long as all sensors are clear
  }
}

void randWander() {
  bool FL[2] = {0, 0};
  bool FR[2] = {0, 1};
  bool BL[2] = {1, 0};
  bool BR[2] = {1, 1};
  //randomize the wander movement list
  short len = sizeof(wanderList) / sizeof(int);
  for (short i = 0; i < len - i; i++) {
    Serial.println(wanderList[i]);
  }
  srand(time(NULL));
  for (short i = 0; i < len - 1; ++i) {
    short j = rand() % (len-i) + i;
    short temp = wanderList[i];
    wanderList[i] = wanderList[j];
    wanderList[j] = temp;
  }
  for (short i = 0; i < len - i; i++) {
    Serial.println(wanderList[i]);
  }
  for (short i = 0; i < len - 1; i++) {
    if (wanderList[i] == 0) { //stop
      runToStop();
    } else if (wanderList[i] == 1) { //Move forward
      forward(3);
    } else if (wanderList[i] == 2) { //Move backward
      forward(-4);
    } else if (wanderList[i] == 3) { //Turn forward left
      turn(5, FL);
    } else if (wanderList[i] == 4) { //Turn forward right
      turn(5, FR);
    } else if (wanderList[i] == 5) { //Turn backward left
      turn(5, BL);
    } else if (wanderList[i] == 6) { //Turn backward right
      turn(5, BR);
    } else if (wanderList[i] == 7) { //Spin right
      turn(0, FL);
    } else if (wanderList[i] == 8) { //Change motor speed
      setBothStepperSpeed(250, 250); //set stepper speeds
    }
  }
}

void setup()
{
  //stepper Motor set up
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  stepperRight.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2

  stepperRight.setSpeed(robot_spd);//set right motor speed
  stepperLeft.setSpeed(robot_spd);//set left motor speed

  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);         // initialize timer1, and set a period in microseconds
  Timer1.attachInterrupt(updateSensors);  // attaches updateSensors() as a timer overflow interrupt

  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  delay(3000);//wait 3 seconds before robot moves

  aggressive = false;
}

void loop()
{
  randWander();  //execute robot motions based upon sensor data and current state
}