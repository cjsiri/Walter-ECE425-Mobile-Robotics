/*DigitalSonar.ino
  4 pin HRC - SR04 sesnro
  trigger and echo pin tied together on pin 12
  data pin 7
  VCC 5V
  GND GND
*/

#include <Arduino.h>          //include for PlatformIO IDE
#include <NewPing.h>
const int PING_PIN = 8; // Arduino pin for both trig and echo tied together on 4 pin [data pin]
NewPing sonar(PING_PIN, PING_PIN);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay
  //unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int uS1 = sonar.ping_in(); //Ping in inches
  //unsigned int uS2 = sonar.ping_cm();//Ping in cm
  Serial.print("4-pin:\t");
  //Serial.print(uS2);
  //Serial.print(uS / US_ROUNDTRIP_CM); // convert time into distance
  //Serial.println(" cm");
  //Serial.print(uS / US_ROUNDTRIP_CM * 0.393); // convert time into distance
  Serial.print(uS1);
  Serial.println(" in");
}

