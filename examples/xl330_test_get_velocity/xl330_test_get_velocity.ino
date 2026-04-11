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
  int32_t vel = robot.getJointSpeed(servoID);
  Serial.print("present velocity: ");
  Serial.println(vel);
  delay(250);
}
