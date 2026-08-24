#include <XL430.h>
//#include <SoftwareSerial.h>
#include <SoftwareSerialWithHalfDuplex.h>

XL430 robot;
//SoftwareSerial mySerial(10, 11); // (RX, TX)
SoftwareSerialWithHalfDuplex mySerial(10,10,false,false);

//Assumes servoID and servoBaud are already set
const int servoID = 1;
const long servoBaud = 115200;

// SDK-style raw units: 1 turn = 4096 counts.
const int32_t GOAL_POS_POSITIVE = 8192;  // +2 turns
const int32_t GOAL_POS_NEGATIVE = -8192; // -2 turns

void setup() {
  Serial.begin(115200);
  mySerial.begin(servoBaud);
  //mySerial.listen();

  robot.begin(mySerial);
  robot.setRxTimeout(300);

  Serial.println("Extended Position + Clear Multi Turn example");
  Serial.print("ID: ");
  Serial.println(servoID);
  Serial.print("Baud: ");
  Serial.println(servoBaud);

  robot.LEDON(servoID);
  delay(500);
  robot.LEDOFF(servoID);
  delay(500);

  // Extended Position mode is Operating Mode 4. Uncomment if operating mode needs to be changed
  //robot.TorqueOFF(servoID);
  //delay(100);
  //robot.setControlMode(servoID, 4);
  //delay(100);
  robot.TorqueON(servoID);
  //delay(100);

  // Set motion profile first (both are 4-byte values)
  //robot.sendPacket_4bytes(servoID, XL430_PROFILE_ACCELERATION, 32000); // tune
  //robot.sendPacket_4bytes(servoID, XL430_PROFILE_VELOCITY, 32000);    // tune

  // printPosition("After mode set");
}

void loop() {
  // Move +2 turns.
  robot.setJointPosition(servoID, GOAL_POS_POSITIVE);
  delay(2500);
  Serial.println(robot.getJointPosition(servoID));
  // Move -2 turns.
  robot.setJointPosition(servoID, GOAL_POS_NEGATIVE);
  delay(2500);
  Serial.println(robot.getJointPosition(servoID));
}