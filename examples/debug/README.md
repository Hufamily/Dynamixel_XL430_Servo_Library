# Debug / Bring-Up Examples (Unverified)

**These sketches are not confirmed working against real hardware.** They were written
one-command-at-a-time while bringing up serial communication with the servo, and some
predate fixes made in the flagship example. Treat them as starting points for debugging
your own wiring/serial setup, not as proof that a given API call works.

For a known-good, end-to-end example, see
[`examples/xl430_extended_position_clear_multi_turn`](../xl430_extended_position_clear_multi_turn)
at the top of `examples/`.

Most of these sketches are designed to avoid EEPROM calibration/config writes — they only
use status reads or RAM writes — so they're relatively low-risk to try, but still:

## Quick Safety Notes

- Use one servo at a time while testing.
- Keep servo ID and baud rate already known/working.
- Avoid running EEPROM-changing examples while validating this suite.
- `setJointPosition` and `setJointSpeed` are RAM-safe but require the servo to already be in the matching mode.

## Command-by-Command Examples

- `ping`: `examples/debug/xl430_test_ping/xl430_test_ping.ino`
- `LEDON` / `LEDOFF`: `examples/debug/xl430_test_led/xl430_test_led.ino`
- `TorqueON` / `TorqueOFF`: `examples/debug/xl430_test_torque/xl430_test_torque.ino`
- `getJointPosition`: `examples/debug/xl430_test_get_position/xl430_test_get_position.ino`
- `getJointSpeed`: `examples/debug/xl430_test_get_velocity/xl430_test_get_velocity.ino`
- `getJointTemperature`: `examples/debug/xl430_test_get_temperature/xl430_test_get_temperature.ino`
- `isJointMoving`: `examples/debug/xl430_test_is_moving/xl430_test_is_moving.ino`
- `setJointPosition` (RAM): `examples/debug/xl430_test_set_position/xl430_test_set_position.ino`
- `setJointSpeed` (RAM): `examples/debug/xl430_test_set_pwm/xl430_test_set_pwm.ino`
- `reboot` (single-shot): `examples/debug/xl430_test_reboot/xl430_test_reboot.ino`
- `action` + `regWrite_1byte` on RAM LED address: `examples/debug/xl430_test_action_regwrite_led/xl430_test_action_regwrite_led.ino`
- `sendPacket_1byte` / `sendPacket` / `sendPacket_4bytes` on RAM addresses only: `examples/debug/xl430_test_send_packet_ram/xl430_test_send_packet_ram.ino`

## Potentiometer-Driven Demos (Older Style)

- Position control via potentiometer: `examples/debug/xl430_position_control/xl430_position_control.ino`
- PWM control via potentiometer: `examples/debug/xl430_PWM_control/xl430_PWM_control.ino`

## EEPROM-Risky (Included, Use With Care)

- Setting servo baud rate / ID: `examples/debug/xl430_baud_rate_and_id/xl430_baud_rate_and_id.ino`

Not included at all:

- `factoryReset`
- `controlTableBackup`
- `setControlMode`
- `regWrite` to arbitrary EEPROM addresses
