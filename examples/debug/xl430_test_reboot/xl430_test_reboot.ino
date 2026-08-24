#include <XL430.h>
#include <SoftwareSerial.h>

XL430 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;
bool sent = false;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  robot.begin(mySerial);

  Serial.println("Reboot test sends exactly once.");
}

void loop() {
  if (!sent) {
    int bytes = robot.reboot(servoID);
    Serial.print("reboot packet bytes sent: ");
    Serial.println(bytes);
    sent = true;
  }
  delay(1000);
}
