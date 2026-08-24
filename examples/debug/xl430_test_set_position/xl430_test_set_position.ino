#include <XL430.h>
#include <SoftwareSerial.h>

XL430 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;

// Safe note: this writes RAM only (goal position), but requires position mode already configured.
void setup() {
  mySerial.begin(115200);
  robot.begin(mySerial);
  robot.TorqueON(servoID);
}

void loop() {
  robot.setJointPosition(servoID, 1024);
  delay(1500);
  robot.setJointPosition(servoID, 3072);
  delay(1500);
}
