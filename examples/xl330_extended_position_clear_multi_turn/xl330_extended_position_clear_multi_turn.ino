#include <XL330.h>
#include <SoftwareSerial.h>

XL330 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;
const long servoBaud = 115200;

// SDK-style raw units: 1 turn = 4096 counts.
const int32_t GOAL_POS_POSITIVE = 8192;  // +2 turns
const int32_t GOAL_POS_NEGATIVE = -8192; // -2 turns

void setup() {
  Serial.begin(115200);
  mySerial.begin(servoBaud);
  mySerial.listen();

  robot.begin(mySerial);
  robot.setRxTimeout(300);

  Serial.println("Extended Position + Clear Multi Turn example");
  Serial.print("ID: ");
  Serial.println(servoID);
  Serial.print("Baud: ");
  Serial.println(servoBaud);

  pinMode(2, OUTPUT);

  // Extended Position mode is Operating Mode 4.
  //robot.TorqueOFF(servoID);
  //delay(100);
  //robot.setControlMode(servoID, 4);
  //delay(100);
  //robot.TorqueON(servoID);
  //delay(100);

  // Set motion profile first (both are 4-byte values)
  //robot.sendPacket_4bytes(servoID, XL_PROFILE_ACCELERATION, 32000); // tune
  //robot.sendPacket_4bytes(servoID, XL_PROFILE_VELOCITY, 32000);    // tune

  // printPosition("After mode set");
}

void loop() {
  digitalWrite(2, HIGH);
  // Move +2 turns.
  robot.setJointPosition(servoID, GOAL_POS_POSITIVE);
  delay(2500);
  // Move -2 turns.
  robot.setJointPosition(servoID, GOAL_POS_NEGATIVE);
  delay(2500);
  digitalWrite(2, LOW); 
  robot.setJointPosition(servoID, -4096*3);
  delay(2500);
}