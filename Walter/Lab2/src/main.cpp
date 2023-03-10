/**
 * Author Name: Chirag Sirigere
 * Date Created: 12/2/2022
 * Program Name: main.cpp
 * Program Descriptions:
 *  Lab1:
 *    This program will introduce using the stepper motor library to create motion algorithms for the robot.
 *    The motions will be teleoperation (stop, forward, spin, reverse, turn), Go-to-Angle, Go-to-Goal, and move in a circle, square, and figure eight
 *    It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
 * 
 *  Lab2:
 *    This program will implement a behavior-based control architecture for a mobile robot for obstacle avoidance and random wander using sensor feedback and a bang-bang or proportional controller.
 *    The design of the program uses subsumption architecture where layer 0 of the control architecture will be the collide and run away behaviors to keep the robot from hitting obstacles.
 *    Layer 1 will be the random wander behavior which moves the robot a random distance and/or heading every n seconds.
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
 *    GND - MS1
 *    5V -  MS2 (driving a motor in quarter-step mode will give the 200-step-per-revolution motor
 *               800 microsteps per revolution by using four different current levels.)
 *    GND - MS3
 * 
 *  LED Pinout:
 *    digital pin 5 - blue LED in series with 220 ohm resistor
 *    digital pin 6 - green LED in series with 220 ohm resistor
 *    digital pin 7 - yellow LED in series with 220 ohm resistor
 *    digital pin 13 - enable LED on microcontroller
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
 *  IR Sensor Pinout:
 *    A0 - Front IR
 *    A1 - Back IR
 *    A2 - Right IR
 *    A3 - Left IR
 * 
 *  Sonar Sensor Pinout:
 *    A4 - Left Sonar
 *    A5 - Right Sonar
 * 
 * Resources:
 *  Arduino pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
 * 
 *  A4988 Stepper Motor Driver:
 *    https://www.makerguides.com/a4988-stepper-motor-driver-arduino-tutorial/
 * 
 *  Interrupts:
 *    https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *    https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
 *    https://playground.arduino.cc/code/timer1
 *    https://playground.arduino.cc/Main/TimerPWMCheatsheet
 *    http://arduinoinfo.mywikis.net/wiki/HOME
 * 
 *  State Machine flag and state byte setup:
 *    The flag byte (8 bits) variable will hold the IR and sonar data [X X snrRight snrLeft irLeft irRight irRear irFront]
 *    The state byte (8 bits) variable will hold the state information as well as motor motion [X X X wander runAway collide rev fwd]
 *    Use the following functions to read, clear and set bits in the byte
 *      bitRead(state, wander)) { // check if the wander state is active
 *      bitClear(state, wander);//clear the the wander state
 *      bitSet(state, wander);//set the wander state
*/

//include all necessary libraries
#include <Arduino.h>          //include for PlatformIO IDE
#include <AccelStepper.h>     //include the stepper motor library
#include <MultiStepper.h>     //include multiple stepper motor library
#include <Adafruit_MPU6050.h> //Include library for MPU6050 IMU
#include <SoftwareSerial.h>   //include Bluetooth module
#include <NewPing.h>          //include Sonar library
#include <TimerOne.h>         //include Timer library
#include <Wire.h>
#include <SPI.h>
#include <stdlib.h>
#include <time.h>

#define baudrate 9600 //serial baud rate

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

#define robot_spd 1500  //set robot speed
#define max_accel 10000 //maximum robot acceleration
#define max_spd 2500    //maximum robot speed

const unsigned short PAUSE_TIME = 3000;         //time before robot moves
const unsigned short STEP_TIME = 500;           //delay time between high and low on step pin
const unsigned short WAIT_TIME = 15000;         //delay for printing data
const float FULL_REV = 800.0;                   //A4988 Stepper Motor Driver quarter step ticks for full revolution of wheel

//define encoder pins
#define LEFT 0                            //left encoder
#define RIGHT 1                           //right encoder
#define ltEncoder 18                      //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
#define rtEncoder 19                      //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long int encoder[2] = {0, 0};    //interrupt variable to hold number of encoder counts (left, right)
long int lastSpeed[2] = {0, 0};           //variable to hold encoder speed (left, right)
long int accumTicks[2] = {0, 0};          //variable to hold accumulated ticks since last reset

//Robot Constants
const float WIDTH_BOT = 23.2; //cm
const float RADIUS_BOT = 11.7; //cm
const float WIDTH_WHEEL = 2.5; //cm
const float RADIUS_WHEEL = 4.25; //cm
const float CM_TO_STEPS = 25.6; 
const float TICKS_TO_STEPS = 20.0;
const float STEPS_TO_TICKS = 1.0/20.0;

//IMU object
Adafruit_MPU6050 IMU;

//Bluetooth module connections
#define BTTX 10               // TX on chip to pin 10 on Arduino Mega
#define BTRX 11               // RX on chip to pin 11 on Arduino Mega
SoftwareSerial BTSerial(BTTX, BTRX);

//*****************************
// Lab 2 Definitions
//*****************************
//define IR sensor connections
#define irFront A0 //front IR analog pin
#define irRear  A1 //back IR analog pin
#define irRight A2 //right IR analog pin
#define irLeft  A3 //left IR analog pin

#define irThresh   5 //in inches // The IR threshold for presence of an obstacle - 400 raw value
#define snrThresh  5   // The sonar threshold for presence of an obstacle
#define minThresh  0   // The sonar minimum threshold to filter out noise
#define stopThresh 150 // If the robot has been stopped for this threshold move

int irFrontArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
int irFrontAvg;  //variable to hold average of current front IR reading
int irLeftAvg;   //variable to hold average of current left IR reading
int irRearAvg;   //variable to hold average of current rear IR reading
int irRightAvg;   //variable to hold average of current right IR reading
int irIdx = 0; //index for 5 IR readings to take the average

//define sonar connections
#define snrLeft A4  //front left sonar
#define snrRight A5 //front right sonar
long SNRDist;
long SNLDist;

NewPing SNL(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing SNR(snrRight, snrRight);  //create an instance of the right sonar

int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right sonar readings
int srLeftAvg;  //variable to holde left sonar data
int srRightAvg; //variable to hold right sonar data
int srIdx = 0; //index for 5 sonar readings to take the average

// State Machine LED
volatile boolean test_state; //variable to hold test led state for timer interrupt

// arrays to hold obstacles
bool obsIR[4] = {0, 0, 0, 0};
bool obsSonar[2] = {0, 0};

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

//Random Wander movement list
/**
 * 0 - Stop
 * 1 - Move forward
 * 2 - Move backward
 * 3 - Turn forward left
 * 4 - Turn forward right
 * 5 - Turn backward left
 * 6 - Turn backward right
 * 7 - Spin left
 * 8 - Spin right
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

  stepperRight.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2

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

//*****************************
// Lab 1 Methods
//*****************************
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
 * Stops both stepper motors
*/
void stop() {
  stepperRight.stop();
  stepperLeft.stop();
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

/**
 * This function pivots the robot in one direction with inputted degrees.
 * The robot has one wheel not moving while the other wheel moves to pivot.
 * 1075 makes a perfect circle witin 5 degrees, giving a 1075 ticks per 90 degree pivot
 * While 1050 is under 10 degrees, 1200 is over 40 degrees, 1100 is over 10 degrees, and 1000 is under 30 degrees
 * @param angle A negative angle to pivot clockwise or a positive value to pivot counterclockwise
 */
void pivot(float angle) {
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  if (angle > 0) {
    stepperRight.moveTo(abs(angle*1075/90));
    stepperRight.setMaxSpeed(500); //set right motor speed
    stepperRight.runSpeedToPosition(); //move right motor
  } else {
    stepperLeft.moveTo(abs(angle*1075/90));
    stepperLeft.setMaxSpeed(500); //set left motor speed
    stepperLeft.runSpeedToPosition(); //move left motor
  }
  runToStop(); //run until the robot reaches the target
}

/**
 * This function spins the robot in one direction with inputted degrees.
 * 2125 makes a perfect circle within 5 degrees, giving a 2125 ticks per 360 degree spin
 * While 2000 is under 15 degrees, 2100 is under 10 degrees, 2150 is over 5 degrees
 * 
 * @param angle A negative angle to spin clockwise or a positive value to spin counterclockwise
 */
void spin(float angle) {
  setBothStepperCurrentPosition(0, 0);
  if (angle > 0) {
    stepperRight.moveTo(abs(angle*2125/360));
    stepperLeft.moveTo(-abs(angle*2125/360));
  } else {
    stepperRight.moveTo(-abs(angle*2125/360));
    stepperLeft.moveTo(abs(angle*2125/360));
  }
  setBothStepperSpeed(500, 500); //set motor speeds
  runAtSpeedToPosition(); //move motors
  runToStop(); //run until the robot reaches the target
}

/**
 * This function turns the robot a quarter circle with a given radius in one direction, forward or reverse.
 * A proportional gain of 1.075 is used to adjust the error within 5 degrees
 * While a proportional gain of 1.1 is over 10 degrees, 1.05 is under 10 degrees. Effectively, the error is 27 degrees per 360 degrees
 * 
 * @param radius A turning radius in centimeters. A radius of 0 spins the robot 90 degrees (similar to calling spin() a quarter circle).
 * @param angle A positive angle value
 * @param turnDirection An array of direction values
 *                        MSB: a forward [0] or reverse [1] turn
 *                        LSB: a left [0] or right [1] turn
 */
void turn(float radius, float angle, bool turnDirection[2]) {
  setBothStepperCurrentPosition(0, 0); //reset stepper positions
  setBothStepperSpeed(500, 500); //set motor speeds

  //float diam = 60.0*radius/18; //60 steps per 18cm
  float outerArc = (radius + (WIDTH_BOT-WIDTH_WHEEL)/2.0) * (angle*PI/180);
  float innerArc = (radius - (WIDTH_BOT-WIDTH_WHEEL)/2.0) * (angle*PI/180);

  int outerSteps = outerArc * FULL_REV/(2 * PI * RADIUS_WHEEL) * 1.075;
  int innerSteps = innerArc * FULL_REV/(2 * PI * RADIUS_WHEEL) * 1.075;

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
void forward(float distance) {
  setBothStepperCurrentPosition(0,0);
  float steps = distance * FULL_REV/(2 * PI * RADIUS_WHEEL);

  stepperRight.moveTo(steps); //move one full rotation forward relative to current position
  stepperLeft.moveTo(steps); //move one full rotation forward relative to current position
  setBothStepperSpeed(500, 500); //set stepper speeds
  stepperRight.runSpeedToPosition(); //move right motor
  stepperLeft.runSpeedToPosition(); //move left motor

  runToStop(); //run until the robot reaches the target
}

/**
 * Moves robot backward a distance based on the distance input.
 * 
 * @param distance A value in centimeters
*/
void reverse(float distance) {
  forward(-distance);
}

/**
 * Calculates arc length needed to turn to angle and calls multistepper to turn robot
 * After turning, encoders are used to error check and move robot again if it moved too much or too little
*/
void goToAngle(float angle) {
  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, LOW); //turn off yellow LED
  spin(angle);
}

/**
 * Give the robot x y coordiantes in cm 
 * Robot will then call goToAngle and spin to a direction that points to the coordiante
 * Will then go calcualted distance to reach coordinate
*/
void goToGoal(float x, float y) {
  digitalWrite(blueLED, LOW); //turn off red LED
  digitalWrite(grnLED, HIGH); //turn on green LED
  digitalWrite(ylwLED, HIGH); //turn on yellow LED

  double radians = atan2(y, x); //returns angle to x, y in radians
  
  float degrees = (radians * 180.0/PI);
  Serial.print("Degrees: ");
  Serial.println(degrees);
  goToAngle(degrees);
  float distance = sqrt((x*x) + (y*y));

  forward(distance);
}

//*****************************
// Lab 2 Methods
//*****************************
/**
 * Updates the IR Sensors
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

/**
 * Updates the Sonar Sensors
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

/**
 * Updates the state of robot motion
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

/**
 * Updates the overall sensors
*/
void updateSensors() {
  //  Serial.print("updateSensors\t");
  //  Serial.println(test_state);
  //test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  //digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  test_state = !test_state;
  digitalWrite(enableLED, test_state);
  flag = 0;       //clear all sensor flags
  state = 0;      //clear all state flags
  updateIR();     //update IR readings and update flag variable and state machine
  updateSonar();  //update Sonar readings and update flag variable and state machine
  updateState();  //update State Machine based upon sensor readings
  delay(1000);     //added so that you can read the data on the serial monitor
}

/**
 * This function takes the sensor averages and adds them as vectors to if they meet the obstacle threshold.
 * This sets avoidVector to have the vector to pass to goToGoal
*/
void randomWander() {
  int direction = random(-1, 1);
}

/**
 * Function executes the aggressive kid behavior, which is stopping at an obstacle and turning on the blue led;
*/
void aggressiveKid() {
  digitalWrite(blueLED, HIGH); //turn on blue LED
  stop();
}

/**
 * This function takes the sensor averages and adds them as vectors to if they meet the obstacle threshold.
 * This sets avoidVector to have the vector to pass to goToGoal
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

/**
 * This is a sample robotMotion() function, the description and code should be updated to reflect the actual robot motion function that you will implement
 * based upon the the lab requirements. Some things to consider, you cannot use a blocking motor function because you need to use sensor data to update
 * movement. You also need to continue to poll the sensors during the motion and update flags and state because this will serve as your interrupt to
 * stop or change movement.
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
    //forward(FULL_REV);//move forward as long as all sensors are clear
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
      reverse(4);
    } else if (wanderList[i] == 3) { //Turn forward left
      turn(5, 10, FL);
    } else if (wanderList[i] == 4) { //Turn forward right
      turn(5, 10, FR);
    } else if (wanderList[i] == 5) { //Turn backward left
      turn(5, 10, BL);
    } else if (wanderList[i] == 6) { //Turn backward right
      turn(5, 10, BR);
    } else if (wanderList[i] == 7) { //Spin left
      spin(10);
    } else if (wanderList[i] == 8) { //Spin right
      spin(-10);
    }
  }
}

void setup()
{  
  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);         // initialize timer1, and set a period in microseconds
  Timer1.attachInterrupt(updateSensors);  // attaches updateSensors() as a timer overflow interrupt

  Serial.begin(baudrate);//start serial communication in order to debug the software while coding
  delay(PAUSE_TIME);//wait 3 seconds before robot moves

  aggressive = false;
}

void loop()
{
  randWander();  //execute robot motions based upon sensor data and current state
}