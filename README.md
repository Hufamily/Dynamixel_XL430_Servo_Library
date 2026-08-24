# Dynamixel XL-430 Servo Library

An Arduino library to control [Dynamixel XL-430-W250-T](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
servos directly from a microcontroller over half-duplex TTL serial, using Dynamixel
Protocol 2.0. It should also work with other X-series actuators (e.g. XL-330) since
they share the same protocol and a very similar control table, though only the XL-430
has been tested here.

This library is based on code originally written for the XL-330 by
[Rei Lee](https://github.com/rei039474/Dynamixel_XL330_Servo_Library).

**Work in progress.** Core position control, torque/LED control, and status reads have
been tested end-to-end on real XL-430 hardware. Several other commands are implemented
but not yet confirmed working — see [Status](#status) below.

<img src="XL330_Arduino.jpg" width="50%" alt="Dynamixel X-series servo wired to a microcontroller" title="Dynamixel X-series servo wired to a microcontroller">

---------------

## Installation

Clone this repository into your Arduino IDE libraries folder:

```
$ cd ~/Documents/Arduino/libraries/
$ git clone <this repo>
```

Restart the Arduino IDE afterward. You can then open the example sketches from:

```
Arduino IDE > File > Examples > Dynamixel_XL430
```

The flagship example (`examples/xl430_extended_position_clear_multi_turn`) additionally
depends on the [`SoftwareSerialWithHalfDuplex`](https://github.com/nickstedman/SoftwareSerialWithHalfDuplex)
library for half-duplex serial over a single data pin — install that too if you plan to
use a software serial pin rather than a hardware UART.

---------------

## Hardware

Refer to the e-manual for [XL430-W250-T](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
for full specifications and the EEPROM/RAM control table addresses.

## Wiring Diagram

XL-430 servos can be daisy-chained on a single half-duplex serial bus and controlled
individually by ID.

<img src="https://emanual.robotis.com/assets/images/dxl/x/x_series_ttl_pin.png" width="30%" alt="Dynamixel X-series servo pinout" title="Dynamixel X-series servo pinout">

* PIN1: GND
* PIN2: VDD (5 volts)
* PIN3: Data (Serial RX/TX)

Since the servo has a single data pin, connect it to both the RX and TX pins of your
microcontroller's chosen serial interface (hardware UART or a half-duplex software
serial library — plain `SoftwareSerial` does not support half-duplex on one pin).

<img src="XL330_wiring.png" width="100%" alt="Example wiring between a microcontroller and a Dynamixel X-series servo" title="Example wiring between a microcontroller and a Dynamixel X-series servo">

---------------

## Status

**Confirmed working** (tested against real XL-430 hardware):

- `setJointPosition` / `getJointPosition`, including extended (multi-turn) position mode
- `clearMultiTurnInfo`
- `TorqueON` / `TorqueOFF`
- `LEDON` / `LEDOFF`
- `ping`
- Raw register access: `sendPacket`, `sendPacket_1byte`, `sendPacket_4bytes`

**Implemented but not yet verified:**

- `setJointSpeed` (PWM control)
- `setBaudRate` / `setID` / `setControlMode` (EEPROM writes)
- `getJointSpeed`, `getJointTemperature`, `isJointMoving`
- `reboot`, `factoryReset`, `controlTableBackup`
- `regWrite` / `regWrite_1byte` / `regWrite_4bytes`

Sketches exercising the unverified commands live in [`examples/debug`](examples/debug),
each isolated to one command, along with notes on which ones are EEPROM-risky. Treat
that folder as a bring-up/debugging aid, not a set of working demos.

---------------

## Example: Extended Position Control

[`examples/xl430_extended_position_clear_multi_turn`](examples/xl430_extended_position_clear_multi_turn)
is the reference example — it puts the servo in Extended Position Control Mode
(Operating Mode 4), then repeatedly commands it between +2 and -2 turns while printing
the present position back over `Serial`.

```cpp
#include <XL430.h>
#include <SoftwareSerialWithHalfDuplex.h>

XL430 robot;
SoftwareSerialWithHalfDuplex mySerial(10, 10, false, false);

const int servoID = 1;
const long servoBaud = 115200;

void setup() {
  mySerial.begin(servoBaud);
  robot.begin(mySerial);
  robot.TorqueON(servoID);
}

void loop() {
  robot.setJointPosition(servoID, 8192);   // +2 turns (SDK-style raw units, 4096 counts/turn)
  delay(2500);
  robot.setJointPosition(servoID, -8192);  // -2 turns
  delay(2500);
}
```

Operating mode must already be set to Extended Position (4) before running this —
either via the Dynamixel Wizard, or by uncommenting the `setControlMode` call in the
sketch (an EEPROM write; do this once, with a single servo connected).

Operating Mode reference: `0` current, `1` velocity, `3` position, `4` extended
position, `5` current-based position, `16` PWM.

---------------

## API Reference

Position and PWM control go through dedicated helpers:

```cpp
robot.setJointPosition(id, value); // goal position, raw units (0-4095 per turn in Position mode)
robot.setJointSpeed(id, value);    // goal PWM, -885 ~ 885
```

For any other control table address, look up the value from the
[XL-430 control table](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area)
(or use the `XL430_*` constants defined in `XL430.h`) and send it directly:

```cpp
// 2-byte data
robot.sendPacket(int id, int address, int value);

// 1-byte data
robot.sendPacket_1byte(int id, int address, int value);

// 4-byte data
robot.sendPacket_4bytes(int id, int address, int32_t value);
```
