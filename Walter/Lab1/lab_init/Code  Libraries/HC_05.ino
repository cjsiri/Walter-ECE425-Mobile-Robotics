
/*
  The circuit:
   RX is digital pin 10 on MEGA (connect to TX on HC-05 Bluetooth)
   TX is digital pin 11 on MEGA (connect to RX of HC-05 Bluethooth)

  Note:
  Not all pins on the Mega and Mega 2560 support change interrupts,
  so only the following can be used for RX:
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

  Get Windows Serial Monitor: https://apps.microsoft.com/store/detail/serial-port-monitoring/9NKDKFKRGM05?hl=en-us&gl=us
  Connect your bluetooth and use device manager to determine where it is connecte
*/

#include <SoftwareSerial.h> //include Bluetooth module

//Bluetooth module connections
#define BTTX 10 // TX on chip to pin 10 on Arduino Mega
#define BTRX 11 //, RX on chip to pin 11 on Arduino Mega
SoftwareSerial BTSerial(BTTX, BTRX);

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define brdLED 13           //LED on Arduino Mega pin 13

void setup()
{
  int baudrate = 9600; // HC-05 default speed in AT command more
  int BTbaud = 9600; //serial monitor baud rate
  pinMode(redLED, OUTPUT);
  pinMode(grnLED, OUTPUT);
  pinMode(ylwLED, OUTPUT);
  pinMode(brdLED, OUTPUT);

  //start serial communication
  BTSerial.begin(baudrate);
  Serial.begin(BTbaud);
  Serial.println("Serial on..");
  BTSerial.println("BT on ...");
}

void loop()
{
  byte inputByte;
  while (BTSerial.available() > 0) {
    inputByte = BTSerial.read();
    Serial.write(inputByte);

    if (inputByte == '1') {
      digitalWrite(brdLED, HIGH);
      digitalWrite(redLED, HIGH);
      digitalWrite(grnLED, HIGH);
      digitalWrite(ylwLED, HIGH);
      BTSerial.println("on");
    }
    else if (inputByte == '0') {
      digitalWrite(brdLED, LOW);
      digitalWrite(redLED, LOW);
      digitalWrite(grnLED, LOW);
      digitalWrite(ylwLED, LOW);
      BTSerial.println("off");
    }
  }
}
