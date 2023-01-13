/**
 * Author Names: Chirag Sirigere
 * Date Created: 12/2/2022
 * Program Name: Walter_main.cpp
 * Program Layout:
 *  ECE425
 *  |- .pio
 *  |- .vscode
 *  |- include
 *  |- lib
 *  |- src
 *  |-- main.cpp
 *  |- test
 *  |- .gitignore
 *  |- platformio.ini
 *  |- README.md
 * 
 * Program Description:
 *  This program will introduce using the stepper motor library to create motion algorithms for the robot.
 *  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
 *  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
 * 
 * Key Functions:
 *  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
 *  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
 *  forward, reverse - both wheels move with same velocity, same direction
 *  pivot - one wheel stationary, one wheel moves forward or back
 *  spin - both wheels move with same velocity opposite direction
 *  turn - both wheels move with same direction different velocity
 *  stop - both wheels stationary
 * 
 *  Library Functions:
 *    move() is a library function for relative movement to set a target position
 *    moveTo() is a library function for absolute movement to set a target position
 *    stop() is a library function that causes the stepper to stop as quickly as possible
 *    run() is a library function that uses accel and decel to achieve target position, no blocking
 *    runSpeed() is a library function that uses constant speed to achieve target position, no blocking
 *    runToPosition() is a library function that uses blocking with accel/decel to achieve target position
 *    runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
 *    runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
 *  
 * Hardware Connections:
 *  Arduino pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
 * 
 *  MPU6050 Pinout:
 *    digital pin 2 - INT
 *    SDA pin 20 - SDA
 *    SCL pin 21 - SCL
 *    GND - GND
 *    VCC - 5V rail
 * 
 *  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182
 *    digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
 *    digital pin 50 - right stepper motor step pin
 *    digital pin 51 - right stepper motor direction pin
 *    digital pin 52 - left stepper motor step pin
 *    digital pin 53 - left stepper motor direction pin
 *    digital pin 13 - enable LED on microcontroller
 *    GND - MS1
 *    5V - MS2 (driving a motor in quarter-step mode will give the 200-step-per-revolution motor
 *              800 microsteps per revolution by using four different current levels.)
 *    GND - MS3
 * 
 *  LED Pinout:
 *    digital pin 6 - red LED in series with 220 ohm resistor
 *    digital pin 7 - green LED in series with 220 ohm resistor
 *    digital pin 8 - yellow LED in series with 220 ohm resistor
 * 
 *  Encoder Pinout:
 *    digital pin 18 - left encoder pin
 *    digital pin 19 - right encoder pin
 * 
 *  IMU Pinout:
 *    digital pin 2 - IMU INT
 *    digital pin 20 - IMU SDA
 *    digital pin 21 - IMU SCL
 * 
 * Resources:
 *  A4988 Stepper Motor Driver:
 *    https://www.makerguides.com/a4988-stepper-motor-driver-arduino-tutorial/
 * 
 *  Interrupts:
 *    https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *    https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
 *    https://playground.arduino.cc/code/timer1
 *    https://playground.arduino.cc/Main/TimerPWMCheatsheet
 *    http://arduinoinfo.mywikis.net/wiki/HOME
*/

//include all necessary libraries
#include <Arduino.h>          //include for PlatformIO IDE
#include <AccelStepper.h>     //include the stepper motor library
#include <MultiStepper.h>     //include multiple stepper motor library
#include <Adafruit_MPU6050.h> //Include library for MPU6050 IMU
#include <SoftwareSerial.h>   //include Bluetooth module

//state LEDs connections
#define blueLED 5             //blue LED for displaying states
#define grnLED 6              //green LED for displaying states
#define ylwLED 7              //yellow LED for displaying states
#define enableLED 13          //stepper enabled LED

//define motor pin numbers
#define stepperEnable 48      //stepper enable pin on stepStick
#define rtStepPin 50          //right stepper motor step pin
#define rtDirPin 51           // right stepper motor direction pin
#define ltStepPin 52          //left stepper motor step pin
#define ltDirPin 53           //left stepper motor direction pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); //create instance of right stepper motor object
// (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin); //create instance of left stepper motor object
// (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers; //create instance to control multiple steppers at the same time

#define stepperEnTrue false   //variable for enabling stepper motor
#define stepperEnFalse true   //variable for disabling stepper motor


const unsigned short PAUSE_TIME = 2500;         //time before robot moves
const unsigned short STEP_TIME = 500;           //delay time between high and low on step pin
const unsigned short WAIT_TIME = 15000;         //delay for printing data
const float FULL_REV = 800.0;                   //A4988 Stepper Motor Driver quarter step ticks for full revolution of wheel

const float WIDTH_BOT = 23.3; //cm
const float RADIUS_BOT = 11.7; //cm
const float CM_TO_STEPS = 25.6; 
const float TICKS_TO_STEPS = 20.0;
const float STEPS_TO_TICKS = 1.0/20.0;

//define encoder pins
#define LEFT 0                            //left encoder
#define RIGHT 1                           //right encoder
#define ltEncoder 18                      //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
#define rtEncoder 19                      //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long int encoder[2] = {0, 0};    //interrupt variable to hold number of encoder counts (left, right)
long int lastSpeed[2] = {0, 0};           //variable to hold encoder speed (left, right)
long int accumTicks[2] = {0, 0};          //variable to hold accumulated ticks since last reset

//IMU object
Adafruit_MPU6050 IMU;

//Bluetooth module connections
#define BTTX 10               // TX on chip to pin 10 on Arduino Mega
#define BTRX 11               // RX on chip to pin 11 on Arduino Mega
SoftwareSerial BTSerial(BTTX, BTRX);

//****************************************************************
// Initialization Functions
/**
 * Function to initialize Bluetooth
 */
void init_BT(){
  Serial.println("Goodnight moon!");
  BTSerial.println("Hello, world?");
}

/**
 * Function to initialize IMU
 */
void init_IMU(){
  Serial.println("Adafruit MPU6050 init!");

  // Try to initialize!
  if (!IMU.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  IMU.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (IMU.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  IMU.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (IMU.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  IMU.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (IMU.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

/**
 * Function to set all stepper motor variables, outputs and LEDs
 */
void init_stepper(){
  pinMode(rtStepPin, OUTPUT); //sets pin as output
  pinMode(rtDirPin, OUTPUT); //sets pin as output
  pinMode(ltStepPin, OUTPUT); //sets pin as output
  pinMode(ltDirPin, OUTPUT); //sets pin as output
  pinMode(stepperEnable, OUTPUT); //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse); //turns off the stepper motor driver
  pinMode(enableLED, OUTPUT); //set enable LED as output
  digitalWrite(enableLED, LOW); //turn off enable LED
  pinMode(blueLED, OUTPUT); //set red LED as output
  pinMode(grnLED, OUTPUT); //set green LED as output
  pinMode(ylwLED, OUTPUT); //set yellow LED as output
  digitalWrite(blueLED, HIGH); //turn on red LED
  digitalWrite(ylwLED, HIGH); //turn on yellow LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  delay(PAUSE_TIME / 5); //wait 0.5 seconds
  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(ylwLED, LOW); //turn off yellow LED
  digitalWrite(grnLED, LOW); //turn off green LED

  stepperRight.setMaxSpeed(1500); //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(10000); //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(1500); //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(10000); //set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight); //add right motor to MultiStepper
  steppers.addStepper(stepperLeft); //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue); //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH); //turn on enable LED
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
 * Function prints encoder data to serial monitor
 */
void print_encoder_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}

/**
 * Function to print IMU data to the serial monitor
 */
void print_IMU_data(){
    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  IMU.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}

/**
 * Function to send and receive data with the Bluetooth
 */
void Bluetooth_comm(){
  String data="";
  if (Serial.available()) {
     while (Serial.available()){
      char nextChar = Serial.read();
      data = data + String(nextChar); 
      if (nextChar == ';') {
        break;
      }
     }
    Serial.println(data);
    BTSerial.println(data);
  }
  
  if (BTSerial.available()) {
    while (BTSerial.available()){
      char nextChar = BTSerial.read();
      data = data + String(nextChar); 
      if (nextChar == ';') {
        break;
      }
    }
    Serial.println(data);
    BTSerial.println(data);
  }
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

/**
 * Function to run the robot until the target is achieved and then stop it
 */
void runToStop ( void ) {
  bool runNow = 1;
  bool rightStopped = 0;
  bool leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop(); //stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop(); //stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

// Move Functions
/**
 * The move1() function will move the robot forward one full rotation and backwared one full rotation with digitalWrites.
 * Recall that that there are 200 steps in one full rotation or 1.8 degrees per step but we are using 1/4 microsteps (so 800).
 * This function uses setting the step pins high and low with delays to move.
 * The speed is set by the length of the delay.
*/
void move1() {
  digitalWrite(blueLED, HIGH); //turn on blue LED
  digitalWrite(grnLED, LOW); //turn off green LED
  digitalWrite(ylwLED, LOW); //turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 phases for making one full cycle rotation
  for (int x = 0; x < FULL_REV; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(STEP_TIME);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(STEP_TIME);
  }
  delay(1000); // One second delay
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
  // Makes 800 phases for making one full cycle rotation
  for (int x = 0; x < FULL_REV; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(STEP_TIME);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(STEP_TIME);
  }
  delay(1000); // One second delay
}

/**
 * The move2() function will use AccelStepper library functions to move the robot
 * Basic AccelStepper flow: Move to absolute or relative -- to current -- position, set a speed, then run
*/
void move2() {
  int rightSpd = 1000;
  int leftSpd = 1000;

  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, LOW); //turn off yellow LED

  stepperRight.moveTo(FULL_REV); //move one full rotation forward relative to current position
  stepperLeft.moveTo(FULL_REV); //move one full rotation forward relative to current position
  setBothStepperSpeed(rightSpd, leftSpd); //set motor speed
  runAtSpeedToPosition(); //move motors
  runToStop(); //run until the robot reaches the target
  delay(1000); // One second delay

  stepperRight.moveTo(0); //move one full rotation backward relative to current position
  stepperLeft.moveTo(0); //move one full rotation backward relative to current position
  setBothStepperSpeed(rightSpd, leftSpd); //set motor speed
  runAtSpeedToPosition(); //move motors
  runToStop(); //run until the robot reaches the target
  delay(1000); // One second delay
}

/**
 * The move3() function will use the MultiStepper() class to move both motors at once
*/
void move3() {
  float rSpeed = 800;
  float lSpeed = 800;

  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, LOW); //turn off green LED
  digitalWrite(ylwLED, HIGH); //turn on yellow LED

  long positions[2]; // Array of desired stepper positions
  setBothStepperSpeed(rSpeed, lSpeed); // Sets the stepper motor speeds

  positions[0] = FULL_REV; //right motor absolute position
  positions[1] = FULL_REV; //left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000); //wait one second

  // Move to a different coordinate
  positions[0] = 0; //right motor absolute position
  positions[1] = 0; //left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000); //wait one second
}

/**
 * This function will move to target at 2 different speeds
*/
void move4() {
  int leftPos = 5000; //right motor absolute position
  int rightPos = 1000; //left motor absolute position
  int leftSpd = 1000; //right motor speed
  int rightSpd = 800; //left motor speed

  digitalWrite(blueLED, HIGH); //turn on red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, LOW); //turn off yellow LED

  setBothStepperCurrentPosition(0, 0); //reset current postition

  //Uncomment the next 2 lines for absolute movement
  stepperLeft.moveTo(leftPos); //move left wheel to absolute position
  stepperRight.moveTo(rightPos); //move right wheel to absolute position

  //Uncomment the next 2 lines for relative movement
  //stepperLeft.move(leftPos); //move left wheel to relative position
  //stepperRight.move(rightPos); //move right wheel to relative position

  //Uncomment the next two lines to set the speed
  setBothStepperSpeed(rightSpd, leftSpd); //set motor speed
  runAtSpeedToPosition(); //run at speed to target position
}

/**
 * This function will move continuously at 2 different speeds
*/
void move5() {
  int leftSpd = 800; //left motor speed
  int rightSpd = 1000; //right motor speed

  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, HIGH); //turn on yellow LED

  setBothStepperSpeed(rightSpd, leftSpd); //set motor speed
  runAtSpeed(); //run both steppers continuously
}

// Lab 1 Demonstration Functions
/**
 * This function pivots the robot in one direction 90 degrees.
 * The robot has one wheel not moving while the other wheel moves to pivot.
 * 
 * @param direction A positive value to pivot clockwise or a negative value to pivot counterclockwise
 */
void pivot(int direction) {
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  if (direction > 0) {
    stepperRight.moveTo(1000);
    stepperRight.setMaxSpeed(500); //set right motor speed
    stepperRight.runSpeedToPosition(); //move right motor
  } else {
    stepperLeft.moveTo(1000);
    stepperLeft.setMaxSpeed(500); //set left motor speed
    stepperLeft.runSpeedToPosition(); //move left motor
  }
  runToStop(); //run until the robot reaches the target
}

/**
 * This function spins the robot in one direction 360 degrees.
 * 
 * @param direction A positive value to spin clockwise or a negative value to spin counterclockwise
 */
void spin(int direction) { //Currently overshoots sometimes
  if (direction > 0) {
    stepperRight.moveTo(-2000);
    stepperLeft.moveTo(2000);
  } else {
    stepperRight.moveTo(2000);
    stepperLeft.moveTo(-2000);
  }
  setBothStepperSpeed(500, 500); //set motor speeds
  runAtSpeedToPosition(); //move motors
  runToStop(); //run until the robot reaches the target
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
 * Moves robot forward a distance based on the distance input.
 * 
 * @param distance A value in centimeters
*/
void forward(int distance) {
  encoder[LEFT] = 0;
  encoder[RIGHT] = 0;

  float steps = distance * 29.9586;
  float ticks = steps * (40/FULL_REV);
  
  stepperRight.moveTo(steps); //move one full rotation forward relative to current position
  stepperLeft.moveTo(steps); //move one full rotation forward relative to current position
  setBothStepperSpeed(500, 500); //set stepper speeds
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motor

  runToStop(); //run until the robot reaches the target

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
}

/**
 * Moves robot backward a distance based on the distance input.
 * 
 * @param distance A value in centimeters
*/
void reverse(int distance) {
  float steps = distance * 29.9586;
  Serial.print(steps);
  stepperRight.moveTo(-steps); //move one full rotation forward relative to current position
  stepperLeft.moveTo(-steps); //move one full rotation forward relative to current position
  stepperRight.setMaxSpeed(500); //set right motor speed
  stepperLeft.setMaxSpeed(500); //set left motor speed
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motor
  runToStop(); //run until the robot reaches the target
}

/**
 * Stops both stepper motors
*/
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/**
 * Moves the robot in a specified direction in a circle of a given diameter
 * 
 * @param diam the diameter of the circle in inches
 * @param dir A positive value to turn right or a negative value to turn left
*/
void moveCircle(int diam, int dir) {
  digitalWrite(blueLED, HIGH); //turn off red LED
  setBothStepperCurrentPosition(0, 0); //reset stepper positions
  setBothStepperSpeed(500, 500); //set stepper speeds

  float inner = 2 * 3.14 * ((diam/2) - WIDTH_BOT/2) * 0.9;
  float outer = 2 * 3.14 * ((diam/2) + WIDTH_BOT/2) * 0.9;

  int innerSteps = inner * 29.958;
  int outerSteps = outer * 29.958;
  
  long positions[2]; // Array of desired stepper positions
  if (dir > 0) {
    positions[0] = innerSteps; //right motor absolute position
    positions[1] = outerSteps; //left motor absolute position
    steppers.moveTo(positions);
  } else {
    positions[0] = outerSteps; //right motor absolute position
    positions[1] = innerSteps; //left motor absolute position
    steppers.moveTo(positions);
  }
  steppers.runSpeedToPosition(); // Blocks until all are in position
  digitalWrite(blueLED, LOW); //turn off red LED
}

/**
 * The moveFigure8() function takes the diameter in inches as the input.
 * It uses the moveCircle() function twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
 * 
 * @param diam the diameter in centimeters
*/
void moveFigure8(int diam) {
  digitalWrite(blueLED, HIGH); //turn off red LED
  digitalWrite(ylwLED, HIGH); //turn on yellow LED

  moveCircle(diam, -1);
  moveCircle(diam, 1);
  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(ylwLED, LOW); //turn on yellow LED
}

/**
 Calculates arc length needed to turn to angle and calls multistepper to turn robot
 After turning, encoders are used to error check and move robot again if it moved too much or too little
*/
void goToAngle(float angle) {
  encoder[LEFT] = 0;
  encoder[RIGHT] = 0;
  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, LOW); //turn off yellow LED
  float arclength = 2.0 * PI * RADIUS_BOT * (angle/360.0) * 1.12;
  int steps = arclength * CM_TO_STEPS;
  int ticks = steps * STEPS_TO_TICKS;
  ticks = abs(ticks);

  stepperRight.moveTo(steps);
  stepperLeft.moveTo(-steps);
  Serial.print("Steps: ");
  Serial.println(steps);
  Serial.print("Ticks: ");
  Serial.println(ticks);

  setBothStepperSpeed(500, 500); //set motor speeds
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motorw
  runToStop();
  
  
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
  Serial.print("Degrees: ");
  Serial.println(angle);
  goToAngle(angle);
  float distance = sqrt((x*x) + (y*y));

  forward(distance); 

}

/*
  Funtion moves the robot in a square by calling forward 4 times with a given side length, with calls to goToAngle at 90 degrees between them
  Should move the robot in a perfect square
*/
void moveSquare(float sideLength) {
  digitalWrite(blueLED, HIGH); //turn on red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, HIGH); //turn on yellow LED

  forward(sideLength);
  goToAngle(90);
  forward(sideLength);
  goToAngle(90);
  forward(sideLength);
  goToAngle(90);
  forward(sideLength);
}

//// MAIN
void setup()
{
  int baudrate = 9600; //serial monitor baud rate'
  int BTbaud = 9600;  // HC-05 default speed in AT command more
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  BTSerial.begin(BTbaud);     //start Bluetooth communication
  Serial.begin(baudrate);     //start serial monitor communication

  while (!Serial) {
    delay(10); // will pause until serial console opens
  }

  init_BT(); //initialize Bluetooth

  //init_IMU(); //initialize IMU
  
  Serial.println("Robot starting...");
  Serial.println("");
  delay(PAUSE_TIME); //always wait 2.5 seconds before the robot moves
}

void loop()
{
  //uncomment each function one at a time to see what the code does
  //move1(); //call move back and forth function
  //move2(); //call move back and forth function with AccelStepper library functions
  //move3(); //call move back and forth function with MultiStepper library functions
  //move4(); //move to target position with 2 different speeds
  //move5(); //move continuously with 2 different speeds

  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  //print_encoder_data();   //prints encoder data

  //Uncomment to read IMU Data (uncomment to read on serial monitor)
  //print_IMU_data();         //print IMU data

  //Uncomment to Send and Receive with Bluetooth
  //Bluetooth_comm();
  /*
  delay(3000);
  forward(30);
  delay(1000);
  reverse(30);
  delay(1000);
  spin(1);
  delay(1000);
  pivot(1);
  delay(1000);
  turn(1);
  */
  
  //pivot(1);
  //spin(1);
  //bool dir[2] = {1,1};
  //turn(15, dir);
  delay(2000);
  goToAngle(-60);
  delay(5000);
  goToAngle(135);
  delay(5000);
  goToGoal(0, 100);
  delay(5000);
  goToGoal(-67, 100);
  delay(5000);
  moveSquare(100);
  //print_encoder_data();

  //delay(wait_time);               //wait to move robot or read data
  delay(WAIT_TIME); 
}