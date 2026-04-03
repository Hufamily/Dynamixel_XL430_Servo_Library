#include <XL330.h>
#include <SoftwareSerial.h>

XL330 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;

// Safe note: this writes RAM only (goal pwm), but requires PWM mode already configured.
void setup() {
  mySerial.begin(115200);
  robot.begin(mySerial);
  robot.TorqueON(servoID);
}

void loop() {
  robot.setJointSpeed(servoID, 300);
  delay(1000);
  robot.setJointSpeed(servoID, -300);
  delay(1000);
  robot.setJointSpeed(servoID, 0);
  delay(1000);
}
