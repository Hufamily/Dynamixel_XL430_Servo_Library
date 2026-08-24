#include <XL430.h>
#include <SoftwareSerial.h>

XL430 robot;
SoftwareSerial mySerial(10, 11); // (RX, TX)

const int servoID = 1;
const long servoBaud = 57600; // Set this to the servo's actual baud rate.

const char* pingCodeToText(int rc) {
  switch (rc) {
    case 1: return "success";
    case -1: return "stream not initialized";
    case -2: return "timeout or short response";
    case -3: return "invalid packet (header/CRC/length)";
    case -4: return "response ID mismatch";
    case -5: return "unexpected instruction (not status 0x55)";
    case -6: return "response parameter count too short";
    case -7: return "servo returned non-zero error byte";
    default: return "unknown ping result";
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(servoBaud);
  mySerial.listen();
  robot.begin(mySerial);
  robot.setRxTimeout(300);

  Serial.println("XL430 ping test (safe, no EEPROM writes)");
  Serial.print("servo baud: ");
  Serial.println(servoBaud);
}

void loop() {
  int rc = robot.ping(servoID);
  if (rc == 1) {
    Serial.println("ping success");
  } else {
    Serial.println("ping failed");
    Serial.print("code: ");
    Serial.print(rc);
    Serial.print(" (");
    Serial.print(pingCodeToText(rc));
    Serial.println(")");
    robot.printLastPingDebug(Serial);
  }
  delay(1000);
}
