// leds.cpp — status-indicator LED driver implementation.

#include "leds.h"
#include "protocol.h"

namespace msd_arduino
{

void StatusLeds::begin(uint8_t red, uint8_t green, uint8_t blue, uint8_t yellow,
                       uint32_t blink_period_ms)
{
  pin_red_          = red;
  pin_green_        = green;
  pin_blue_         = blue;
  pin_yellow_       = yellow;
  blink_period_ms_  = blink_period_ms;

  pinMode(pin_red_,    OUTPUT);
  pinMode(pin_green_,  OUTPUT);
  pinMode(pin_blue_,   OUTPUT);
  pinMode(pin_yellow_, OUTPUT);
  allOff();
}

void StatusLeds::allOff()
{
  digitalWrite(pin_red_,    LOW);
  digitalWrite(pin_green_,  LOW);
  digitalWrite(pin_blue_,   LOW);
  digitalWrite(pin_yellow_, LOW);
}

void StatusLeds::update(const LedState & s, uint32_t now_ms)
{
  // Red: solid ON while E-STOP (power relay OFF).
  // (Error blink removed — red now matches external LIGHT_RED.)
  digitalWrite(pin_red_, s.estop ? HIGH : LOW);

  // Yellow: solid ON while battery SOC < 10%.
  // (Matches external LIGHT_YELLOW.)
  digitalWrite(pin_yellow_, s.low_battery ? HIGH : LOW);

  // Green / Blue: source indicator + motor-active blink.
  //   - Jetson active, motor running  → green fast-blink (20 Hz)
  //   - Jetson active, idle           → green solid
  //   - SBUS active, motor running    → blue fast-blink (20 Hz)
  //   - SBUS active, idle             → blue solid
  //   - E-STOP                        → both off (red takes over)
  bool green_on = false;
  bool blue_on  = false;

  if (!s.estop) {
    const bool is_jetson = (s.active_src == protocol::CTRL_SRC_JETSON);
    const bool is_sbus   = (s.active_src == protocol::CTRL_SRC_SBUS);

    if (s.motor_active) {
      // Fast blink: 50 ms period = 20 Hz
      const uint32_t kFastPeriod = 50;
      const bool blink_on = ((now_ms % kFastPeriod) < (kFastPeriod / 2));
      if (is_jetson) { green_on = blink_on; }
      if (is_sbus)   { blue_on  = blink_on; }
    } else {
      // Idle: solid
      if (is_jetson) { green_on = true; }
      if (is_sbus)   { blue_on  = true; }
    }
  }

  digitalWrite(pin_green_, green_on ? HIGH : LOW);
  digitalWrite(pin_blue_,  blue_on  ? HIGH : LOW);
}

}  // namespace msd_arduino
