#include <XL330.h>
#include <SoftwareSerial.h>

XL330 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  robot.begin(mySerial);

  Serial.println("XL330 ping test (safe, no EEPROM writes)");
}

void loop() {
  int sent = robot.ping(servoID);
  Serial.print("ping packet bytes sent: ");
  Serial.println(sent);
  delay(1000);
}
