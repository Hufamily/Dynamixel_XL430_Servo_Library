# XL330 Safe Test Examples (No EEPROM Writes)

These examples are designed to avoid EEPROM calibration/config writes.
They only use status reads or RAM writes.

## Quick Safety Notes

- Use one servo at a time while testing.
- Keep servo ID and baud rate already known/working.
- Avoid running EEPROM-changing examples while validating this suite.
- `setJointPosition` and `setJointSpeed` are RAM-safe but require the servo to already be in the matching mode.

## Command-by-Command Examples

- `ping`: `examples/xl330_test_ping/xl330_test_ping.ino`
- `LEDON` / `LEDOFF`: `examples/xl330_test_led/xl330_test_led.ino`
- `TorqueON` / `TorqueOFF`: `examples/xl330_test_torque/xl330_test_torque.ino`
- `getJointPosition`: `examples/xl330_test_get_position/xl330_test_get_position.ino`
- `getJointSpeed`: `examples/xl330_test_get_velocity/xl330_test_get_velocity.ino`
- `getJointTemperature`: `examples/xl330_test_get_temperature/xl330_test_get_temperature.ino`
- `isJointMoving`: `examples/xl330_test_is_moving/xl330_test_is_moving.ino`
- `setJointPosition` (RAM): `examples/xl330_test_set_position/xl330_test_set_position.ino`
- `setJointSpeed` (RAM): `examples/xl330_test_set_pwm/xl330_test_set_pwm.ino`
- `reboot` (single-shot): `examples/xl330_test_reboot/xl330_test_reboot.ino`
- `action` + `regWrite_1byte` on RAM LED address: `examples/xl330_test_action_regwrite_led/xl330_test_action_regwrite_led.ino`
- `sendPacket_1byte` / `sendPacket` / `sendPacket_4bytes` on RAM addresses only: `examples/xl330_test_send_packet_ram/xl330_test_send_packet_ram.ino`

## Not Included On Purpose (EEPROM-Risky)

- `setID`
- `setBaudRate`
- `setControlMode`
- `factoryReset`
- `controlTableBackup`
- `regWrite` to EEPROM addresses
