#include <XL330.h>
#include <SoftwareSerial.h>

XL330 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  robot.begin(mySerial);
}

void loop() {
  robot.TorqueOFF(servoID);
  Serial.println("Torque OFF for 2s");
  delay(2000);

  robot.TorqueON(servoID);
  Serial.println("Torque ON for 2s");
  delay(2000);
}
