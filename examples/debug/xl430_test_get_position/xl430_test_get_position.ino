#include <XL430.h>
#include <SoftwareSerial.h>

XL430 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  robot.begin(mySerial);
}

void loop() {
  int32_t pos = robot.getJointPosition(servoID);
  if (pos >= 0) {
    Serial.print("present position: ");
    Serial.println(pos);
  } else if (pos <= -10) {
    Serial.print("servo status error: ");
    Serial.println((-10) - pos);
  } else {
    Serial.print("read error code: ");
    Serial.println(pos);
  }
  delay(250);
}
