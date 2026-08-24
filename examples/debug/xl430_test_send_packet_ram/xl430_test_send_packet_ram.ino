#include <XL430.h>
#include <SoftwareSerial.h>

XL430 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;

// This sketch only writes RAM addresses:
// - XL430_LED (1 byte)
// - XL430_GOAL_PWM (2 bytes)
// - XL430_GOAL_POSITION (4 bytes)
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  robot.begin(mySerial);
  robot.TorqueON(servoID);
}

void loop() {
  int b1 = robot.sendPacket_1byte(servoID, XL430_LED, 1);
  delay(300);
  int b2 = robot.sendPacket(servoID, XL430_GOAL_PWM, 100);
  delay(300);
  int b4 = robot.sendPacket_4bytes(servoID, XL430_GOAL_POSITION, 2048);
  delay(600);
  robot.sendPacket_1byte(servoID, XL430_LED, 0);

  Serial.print("bytes sent [1B,2B,4B]: ");
  Serial.print(b1);
  Serial.print(", ");
  Serial.print(b2);
  Serial.print(", ");
  Serial.println(b4);

  delay(1000);
}
