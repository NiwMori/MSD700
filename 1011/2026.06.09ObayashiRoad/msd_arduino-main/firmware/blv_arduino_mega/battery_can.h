// battery_can.h — CAN bus battery BMS reader (MCP2515).
//
// Reads battery data from two BMS modules via CAN bus:
//   Module 1: 0x056 (voltage/current), 0x055 (temperatures)
//   Module 2: 0x076 (voltage/current), 0x075 (temperatures)
//   SOC:      0x053 (remaining capacity in mAh)
//
// Hardware: MCP2515 on SPI (CS = pin 53, INT = pin 2).
//
// Install the library:
//   arduino-cli lib install "mcp_can"

#ifndef MSD_BLV_BATTERY_CAN_H_
#define MSD_BLV_BATTERY_CAN_H_

#include <Arduino.h>
#include <stdint.h>
#include "protocol.h"

namespace msd_arduino
{

class BatteryCan
{
public:
  // Initialize the MCP2515 CAN controller.
  // Returns true if CAN.begin() succeeds.
  bool begin(uint8_t cs_pin, uint8_t int_pin);

  // Poll for incoming CAN messages. Call every loop().
  // Returns true if any message was processed.
  bool update();

  // Fill a BatteryStatus struct with the latest values.
  void fill(protocol::BatteryStatus & out) const;

  // True once at least one valid CAN frame has been received.
  bool is_valid() const { return received_any_; }

  // Print parsed battery values to a serial port for debugging.
  void debugDump(Stream & out) const;

  // Enable/disable raw CAN frame hex dump on every received message.
  void setRawDebug(Stream * out) { raw_debug_ = out; }

private:
  // Conversion helpers (from BMS raw format)
  static float parseVoltage_(uint16_t raw);
  static float parseCurrent_(uint16_t raw);
  static float parseTemp_(uint16_t raw);

  uint8_t int_pin_ = 2;
  bool    received_any_ = false;

  // Module 1
  float mod1_v_      = 0.0f;
  float mod1_i_      = 0.0f;
  float mod1_t_max_  = 0.0f;
  float mod1_t_fet_  = 0.0f;
  float mod1_t_min_  = 0.0f;
  // Module 2
  float mod2_v_      = 0.0f;
  float mod2_i_      = 0.0f;
  float mod2_t_max_  = 0.0f;
  float mod2_t_fet_  = 0.0f;
  float mod2_t_min_  = 0.0f;
  // SOC
  uint16_t soc_mah_     = 0;
  float    soc_percent_ = 0.0f;

  Stream * raw_debug_ = nullptr;

  static constexpr float CURRENT_SCALE = 0.01119f;
  static constexpr float VOLTAGE_SCALE = 4.8832f / 1000.0f;
  static constexpr uint16_t SOC_MAX_MAH = 44000;  // 22Ah × 2 modules in parallel
};

}  // namespace msd_arduino

#endif  // MSD_BLV_BATTERY_CAN_H_
