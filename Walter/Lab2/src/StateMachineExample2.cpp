//StateMachineExample2.ino
//Auth: song-jianjian
//Date: Sept. 19, 2017
//Revised: 11/28/17
//Auth: Carlotta Berry
//You can use this template to design a state machine or create your own using IF CONDITIONS or SWITCH-CASE STATEMENTS
//This state machine will flash an LED, spin a motor, play a buzzer and move a servo
#include <Arduino.h>
#include <Servo.h>    // servo library
Servo SongServo;      //define a servo object

//inputs and outputs
#define FlexSensor A0   //flex sensor analog input
#define SoftPot A1      //soft pot analog input
#define ObjectSensor A5 //object sensor digital input
#define Buzzer 9        //buzzer output

////to avoid conflict with servo. Motor will not turn on pin 10
#define DCmotor 2       //DC motor output
#define ServoMotor 11   //servo motor output
#define LED 8           //LED output

//define the states
#define FLASH_LED 0
#define MOTOR_SPIN 1
#define BUZZER_PLAY 2
#define SERVO_MOVE 3

//sensor status: true or false, HIGH or LOW
//set by read_sensors();
int flex_status, pot_status, object_detect_status;

//servo low (0 degree) and high (180 degree) bounds
//the values need to be calibrated to constrain servo motor
#define SERVOLOW 650
#define SERVOHIGH 2700
#define baud_rate 9600
#define wait_time 1000

//setup function runs once to start serial communication
void setup()
{
  Serial.begin(baud_rate);
  Serial.println("Start State machine");
  SongServo.attach(ServoMotor, SERVOLOW, SERVOHIGH);
  delay(wait_time);
}

//function to flash LED
void flash_led() {
  //flash LED 5 times

}

//function to spin the DC motor
void motor_spin() {
  //spin DC motor for 3 seconds
}

//funcition to play a buzzer
void buzzer_play() {
  //Play a song
}

//function to move the servo motor
void servo_move() {
  //swing servo from 0 to 180 degrees
}

//function to read sensors and set their status values: true or false, HIGH or LOW
void read_sensors() {
  //set status based on reading to be true or false, HIGH or LOW
  //int flex_Status, pot_status, object_detect_status;
  //use Serial.print() to help you debug
}


void run_machine1() {
  static unsigned char state = FLASH_LED;
  //state machine block
  switch (state) {
    case FLASH_LED:
      flash_led();  state = MOTOR_SPIN; Serial.println("Flash LED");
      break;
    case MOTOR_SPIN:
      motor_spin();   state = BUZZER_PLAY; Serial.println("Spin Motor");
      break;
    case BUZZER_PLAY:
      buzzer_play(); state = SERVO_MOVE; Serial.println("Play on Buzzer");
      break;
    case SERVO_MOVE:
      servo_move();  state = FLASH_LED; Serial.println("Move Servo"); Serial.println(" ");
      break;
  }
}

void run_machine2() {
  static unsigned char state = FLASH_LED;
  //flex_status, pot_status, object_detect_status
  //state machine block
  switch (state) {
    case FLASH_LED:
      flash_led();  read_sensors();
      if (flex_status == true) state = FLASH_LED;
      else if (pot_status == true) state =  MOTOR_SPIN;
      else if (object_detect_status == HIGH) state =  SERVO_MOVE;
      else state = FLASH_LED;
      Serial.println("Flash LED");
      break;
    case MOTOR_SPIN:
      motor_spin();   read_sensors(); state = BUZZER_PLAY; Serial.println("Spin Motor");
      break;
    case BUZZER_PLAY:
      buzzer_play(); read_sensors(); state = SERVO_MOVE; Serial.println("Play on Buzzer");
      break;
    case SERVO_MOVE:
      servo_move();  read_sensors(); state = FLASH_LED; Serial.println("Move Servo"); Serial.println(" ");
      break;
  }
}



//thresholds to be adjusted
#define FLESHTHRESHOLD 1
#define POTTHRESHOLD 2
#define OBJECTTHRESHOLD 0




//loop function that runs continuously state machine 1 or 2
void loop() {
  //  run_machine1(); delay(1000);    //Moore Machine - input changes output
  run_machine2(); delay(1000);      //Mealy Machine - input does not affect the output
}
