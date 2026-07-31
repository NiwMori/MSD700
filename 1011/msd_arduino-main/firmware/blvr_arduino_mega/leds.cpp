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
  const uint32_t kFastPeriod = 50;
  const bool fast_blink_on = ((now_ms % kFastPeriod) < (kFastPeriod / 2));

  // Red: low battery alert blinks. Otherwise solid while stopped.
  const bool low_battery_blink = ((now_ms % 400) < 200);
  digitalWrite(pin_red_, s.low_battery ? (low_battery_blink ? HIGH : LOW)
                                       : (s.estop ? HIGH : LOW));

  // Green / Blue / Yellow: source indicator + motor-active blink.
  //   - Jetson active, motor running  → green fast-blink (20 Hz)
  //   - Jetson active, idle           → green solid
  //   - SBUS active, motor running    → blue fast-blink (20 Hz)
  //   - SBUS active, idle             → blue solid
  //   - YCON active, motor running    → yellow fast-blink (20 Hz)
  //   - YCON active, idle             → yellow solid
  //   - E-STOP                        → all source LEDs off (red takes over)
  bool green_on = false;
  bool blue_on  = false;
  bool yellow_on = false;
  const bool is_ycon = (s.active_src == protocol::CTRL_SRC_YCON);

  if (is_ycon) {
    yellow_on = s.motor_active ? fast_blink_on : true;
  }

  if (!s.estop) {
    const bool is_jetson = (s.active_src == protocol::CTRL_SRC_JETSON);
    const bool is_sbus   = (s.active_src == protocol::CTRL_SRC_SBUS);

    if (s.motor_active) {
      if (is_jetson) { green_on = fast_blink_on; }
      if (is_sbus)   { blue_on  = fast_blink_on; }
    } else {
      // Idle: solid
      if (is_jetson) { green_on = true; }
      if (is_sbus)   { blue_on  = true; }
    }
  }

  digitalWrite(pin_green_, green_on ? HIGH : LOW);
  digitalWrite(pin_blue_,  blue_on  ? HIGH : LOW);
  digitalWrite(pin_yellow_, yellow_on ? HIGH : LOW);
}

}  // namespace msd_arduino
