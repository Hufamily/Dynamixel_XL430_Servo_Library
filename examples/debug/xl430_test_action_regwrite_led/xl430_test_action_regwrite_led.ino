#include <XL430.h>
#include <SoftwareSerial.h>

XL430 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;
bool ledOn = false;

// Safe note: regWrite_1byte targets LED RAM address (65), not EEPROM.
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  robot.begin(mySerial);
}

void loop() {
  int regBytes = robot.regWrite_1byte(servoID, XL430_LED, ledOn ? 1 : 0);
  int actionBytes = robot.action(servoID);

  Serial.print("regWrite bytes sent: ");
  Serial.print(regBytes);
  Serial.print(", action bytes sent: ");
  Serial.println(actionBytes);

  ledOn = !ledOn;
  delay(1000);
}
