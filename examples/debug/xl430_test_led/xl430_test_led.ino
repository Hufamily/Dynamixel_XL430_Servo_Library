#include <XL430.h>
#include <SoftwareSerial.h>

XL430 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;

void setup() {
  mySerial.begin(115200);
  robot.begin(mySerial);
  robot.TorqueON(servoID);
}

void loop() {
  robot.LEDON(servoID);
  delay(500);
  robot.LEDOFF(servoID);
  delay(500);
}
