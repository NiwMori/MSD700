# MSD Main Board — Arduino IDE Port

Arduino IDE version of the main board firmware.
Original PlatformIO project: `../msd-controller-esp-now-experiment-main-board-firmware`

**Target board:** ESP32-S3 DevKitC-1
**Serial port:** Use the native USB port (not UART/CP2102). In Arduino IDE, enable **Tools → USB CDC On Boot → Enabled**.

---

## Revision History

### 2026-06-02

#### Bug fixes

**1. Motor stopped while cylinder still moved (ESP-NOW override)**
- Root cause: `receivePropoSwitch()` was called every loop unconditionally. When any of ch1–ch4 returned 0 (RC signal dropout), the function immediately forced `g_power_relay = 0`, overriding the value already set by ESP-NOW `OnDataRecv()`. The sub-board receives cylinder commands directly from the white controller, so cylinders kept moving while the motor stopped.
- Fix: `receivePropoSwitch()` now applies ch5/ch6 (relay and mode) first, then returns early if `g_control_mode == MODE_ESPNOW` before touching joystick values. `checkRcTimeOut()` is guarded to skip in ESP-NOW mode.

**2. Motor stopped after toggling CH5/CH6 simultaneously**
- Root cause: `lastRcReceiveTime` was only updated when all joystick channels (ch1–ch4) were valid. Switching CH6 to ESP-NOW mode causes an early return in `receivePropoSwitch()`, so `lastRcReceiveTime` was never refreshed. After 500 ms, `checkRcTimeOut()` forced `g_power_relay = 0` even though the RC transmitter was alive.
- Fix: Split into two timestamps — `lastRcReceiveTime` (updated when ch5 or ch6 has signal, meaning transmitter is alive) and `lastJoystickTime` (updated when ch1–ch4 are all valid). `checkRcTimeOut()` now has two levels: full RC loss kills relay after 500 ms; joystick-only dropout zeros velocity after 500 ms but keeps relay on.

**3. Motor stopped permanently after toggling CH5 (relay) — required re-upload to recover**
- Root cause: `stopMotor(true)` was called inside `case 0` (relay OFF). A physical relay takes ~10–20 ms to open its contacts after the GPIO goes LOW. During that window the motor driver is still powered, but MbFree was already pulled LOW by the I2C write. The driver interpreted this as an abrupt brake command while the motor was spinning, triggered its alarm latch, and refused to run again even after the relay was re-energised.
- Fix: Removed `stopMotor(true)` from `case 0`. The relay OFF path now only cuts the GPIO and zeros PWM. The motor driver loses power naturally when the contacts open — no MbFree assertion needed.

**4. Motor driver alarm on relay re-energise**
- Root cause: Even with fix 3, relay switching transients can corrupt MCP23017 output state or trigger the motor driver's alarm latch.
- Fix: Added `motorDriverReinit()`, called once whenever `effective_relay` transitions from 0 → 1. It re-asserts all IO expander outputs to a known safe state (direction neutral, M0 HIGH, StopMode LOW, MbFree HIGH) and pulses AlarmReset HIGH for 50 ms to clear any fault latch.

**5. MbFree pulled LOW while motor driver powered — triggered alarm**
- Root cause: `writeMotorPwm(0, 0)` called `stopMotor(true)` → MbFree = LOW while the driver was still energised (relay ON). Asserting the brake while the motor is spinning causes regenerative/short-circuit current that triggers the driver alarm.
- Fix: `writeMotorPwm()` now always keeps MbFree = HIGH and only controls PWM. Motor coasts (free-wheels) when velocity = 0 rather than actively braking. MbFree = LOW is only safe in `case 0` when the driver has no power, but that path no longer writes to MbFree either.

**6. Left/right motor speed imbalance**
- Root cause: `ik()` contained a conditional swap of `g_left_rpm` / `g_right_rpm` under certain reverse-turning conditions. Direction signals were already set correctly via the IO expander based on the sign of the RPM values, but the swap mismatched the PWM magnitudes with the direction signals.
- Fix: Removed the swap. `g_left_rpm` and `g_right_rpm` are assigned directly from the computed values; `writeMotorPwm()` takes `abs()` of each.

**7. Motor felt slow — deadzone not remapped**
- Root cause: Joystick was normalised to [-1.0, 1.0] and a deadzone of ±0.3 was zeroed, but the remaining range [0.3, 1.0] was never remapped to [0.0, 1.0]. Full stick deflection still reached full speed, but the effective control range felt compressed.
- Fix: Added `remapDeadzone()` — maps [deadzone, 1.0] → [0.0, 1.0] so that any position past the deadzone uses the full speed range linearly.

**8. Lambda declared inside switch-case (C++ undefined behaviour)**
- Root cause: `auto remapDeadzone = [...]` was declared inside `case 1:` of the switch statement without enclosing braces. Declaring a variable inside a switch case without a new scope block is undefined behaviour in C++ and can cause stack corruption on some compiler versions.
- Fix: Moved the lambda declaration outside the if/else block entirely.

#### Other changes
- `receivePropoSwitch()`: ch5/ch6 are now always read and applied before any early return, so relay and mode switches respond regardless of joystick channel state.
- `checkRcTimeOut()`: skips in `MODE_ESPNOW`; uses separate timestamps for full-RC-loss vs joystick-only dropout.
- All Indonesian and Japanese comments translated to English and cleaned up.
