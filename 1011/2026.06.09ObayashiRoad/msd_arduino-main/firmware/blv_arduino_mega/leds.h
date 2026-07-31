// leds.h — status-indicator LED driver.
//
// Four LEDs on analog pins used as digital outputs:
//
//   RED    : solid while power relay is OFF (E-STOP / stopped).
//   GREEN  : fast-blink (~20 Hz) while motors are active from Jetson.
//            Solid when Jetson source is selected but idle.
//   BLUE   : fast-blink (~20 Hz) while motors are active from SBUS.
//            Solid when SBUS source is selected but idle.
//   YELLOW : solid while battery SOC < 10%.
//
// Call `begin()` once in setup(), then feed it state every loop via
// `update(state, now_ms)`.

#ifndef MSD_BLV_LEDS_H_
#define MSD_BLV_LEDS_H_

#include <Arduino.h>
#include <stdint.h>

namespace msd_arduino
{

struct LedState
{
  uint8_t  active_src   = 0;   // CTRL_SRC_JETSON / CTRL_SRC_SBUS
  bool     estop        = false;
  bool     motor_active = false;  // true when drive() was called non-zero
  bool     low_battery  = false;  // true when SOC < 10%
};

class StatusLeds
{
public:
  void begin(uint8_t red, uint8_t green, uint8_t blue, uint8_t yellow,
             uint32_t blink_period_ms = 400);

  void update(const LedState & s, uint32_t now_ms);

  void allOff();

private:
  uint8_t pin_red_    = 0;
  uint8_t pin_green_  = 0;
  uint8_t pin_blue_   = 0;
  uint8_t pin_yellow_ = 0;
  uint32_t blink_period_ms_ = 400;
};

}  // namespace msd_arduino

#endif  // MSD_BLV_LEDS_H_
