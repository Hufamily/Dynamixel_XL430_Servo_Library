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
  int temp = robot.getJointTemperature(servoID);
  Serial.print("present temperature (C): ");
  Serial.println(temp);
  delay(500);
}
