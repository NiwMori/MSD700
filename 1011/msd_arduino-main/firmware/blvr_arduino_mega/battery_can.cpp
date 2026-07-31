// battery_can.cpp — CAN bus battery BMS reader implementation.

#include "battery_can.h"
#include <SPI.h>
#include <mcp_can.h>
#include <math.h>

// The MCP_CAN object must live at file scope because the library stores
// state internally and we only create one instance.
static MCP_CAN * g_can = nullptr;

namespace msd_arduino
{

bool BatteryCan::begin(uint8_t cs_pin, uint8_t int_pin)
{
  int_pin_ = int_pin;
  pinMode(int_pin_, INPUT);

  if (g_can) { delete g_can; }
  g_can = new MCP_CAN(cs_pin);

  if (g_can->begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    return false;
  }
  g_can->setMode(MCP_NORMAL);
  return true;
}

float BatteryCan::parseVoltage_(uint16_t raw)
{
  if (raw == 0xFFFE || raw == 0xFFFF) { return 0.0f; }
  return raw * VOLTAGE_SCALE;
}

float BatteryCan::parseCurrent_(uint16_t raw)
{
  if (raw == 0xFFFE || raw == 0xFFFF) { return 0.0f; }
  int32_t val = (int32_t)raw - 0x8000;
  return val * CURRENT_SCALE;
}

float BatteryCan::parseTemp_(uint16_t raw)
{
  if (raw == 0xFFFE || raw == 0xFFFF) { return 0.0f; }
  int32_t val = (int32_t)raw - 0x8000;
  return val * 0.1f;
}

bool BatteryCan::update()
{
  if (!g_can) { return false; }

  bool got_any = false;

  // Process all pending CAN messages.
  while (g_can->checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    unsigned char len;
    unsigned char buf[8];

    if (g_can->readMsgBuf(&rxId, &len, buf) != CAN_OK) {
      continue;
    }

    got_any = true;
    received_any_ = true;

    // Raw hex dump for debugging byte order / scaling issues.
    if (raw_debug_) {
      raw_debug_->print(F("CAN 0x"));
      raw_debug_->print((unsigned int)rxId, HEX);
      raw_debug_->print(F(" ["));
      raw_debug_->print(len);
      raw_debug_->print(F("]:"));
      for (uint8_t i = 0; i < len; ++i) {
        raw_debug_->print(' ');
        if (buf[i] < 0x10) { raw_debug_->print('0'); }
        raw_debug_->print(buf[i], HEX);
      }
      raw_debug_->println();
    }

    switch (rxId) {
      case 0x056:  // Module1 voltage / current
        mod1_v_ = parseVoltage_((uint16_t)(buf[3] << 8) | buf[2]);
        mod1_i_ = parseCurrent_((uint16_t)(buf[1] << 8) | buf[0]);
        break;
      case 0x076:  // Module2 voltage / current
        mod2_v_ = parseVoltage_((uint16_t)(buf[3] << 8) | buf[2]);
        mod2_i_ = parseCurrent_((uint16_t)(buf[1] << 8) | buf[0]);
        break;
      case 0x055:  // Module1 temperatures
        mod1_t_max_ = parseTemp_((uint16_t)(buf[1] << 8) | buf[0]);
        mod1_t_fet_ = parseTemp_((uint16_t)(buf[3] << 8) | buf[2]);
        mod1_t_min_ = parseTemp_((uint16_t)(buf[5] << 8) | buf[4]);
        break;
      case 0x075:  // Module2 temperatures
        mod2_t_max_ = parseTemp_((uint16_t)(buf[1] << 8) | buf[0]);
        mod2_t_fet_ = parseTemp_((uint16_t)(buf[3] << 8) | buf[2]);
        mod2_t_min_ = parseTemp_((uint16_t)(buf[5] << 8) | buf[4]);
        break;
      case 0x053:  // SOC
      {
        uint16_t val = (uint16_t)(buf[1] << 8) | buf[0];
        if (val == 0xFFFE || val == 0xFFFF) {
          soc_mah_     = 0;
          soc_percent_ = 0.0f;
        } else {
          soc_mah_     = val;
          soc_percent_ = ((float)val / SOC_MAX_MAH) * 100.0f;
          if (soc_percent_ > 100.0f) { soc_percent_ = 100.0f; }
        }
      }
      break;
      default:
        break;
    }
  }

  return got_any;
}

void BatteryCan::fill(protocol::BatteryStatus & out) const
{
  // Voltage: float V → uint16_t in 10 mV units (48.0 V → 4800).
  out.mod1_voltage_10mv = (uint16_t)(mod1_v_ * 100.0f + 0.5f);
  out.mod2_voltage_10mv = (uint16_t)(mod2_v_ * 100.0f + 0.5f);

  // Current: float A → int16_t in 10 mA units (1.5 A → 150).
  out.mod1_current_10ma = (int16_t)(mod1_i_ * 100.0f);
  out.mod2_current_10ma = (int16_t)(mod2_i_ * 100.0f);

  // Temperature: float °C → int16_t in 0.1 °C (25.3 °C → 253).
  out.mod1_temp_max_01c = (int16_t)(mod1_t_max_ * 10.0f);
  out.mod1_temp_fet_01c = (int16_t)(mod1_t_fet_ * 10.0f);
  out.mod1_temp_min_01c = (int16_t)(mod1_t_min_ * 10.0f);
  out.mod2_temp_max_01c = (int16_t)(mod2_t_max_ * 10.0f);
  out.mod2_temp_fet_01c = (int16_t)(mod2_t_fet_ * 10.0f);
  out.mod2_temp_min_01c = (int16_t)(mod2_t_min_ * 10.0f);

  out.soc_mah     = soc_mah_;
  out.soc_percent = (uint8_t)(soc_percent_ + 0.5f);
}

static void printValOrNA(Stream & out, float v, uint8_t dec)
{
  if (isnan(v)) { out.print(F("N/A")); }
  else          { out.print(v, dec); }
}

void BatteryCan::debugDump(Stream & out) const
{
  out.print(F("[Module1] V: ")); printValOrNA(out, mod1_v_, 2);
  out.print(F(" V  I: "));      printValOrNA(out, mod1_i_, 2);
  out.print(F(" A  Tmax: "));   printValOrNA(out, mod1_t_max_, 1);
  out.print(F(" C  Tfet: "));   printValOrNA(out, mod1_t_fet_, 1);
  out.print(F(" C  Tmin: "));   printValOrNA(out, mod1_t_min_, 1);
  out.print(F(" C  | "));

  out.print(F("[Module2] V: ")); printValOrNA(out, mod2_v_, 2);
  out.print(F(" V  I: "));      printValOrNA(out, mod2_i_, 2);
  out.print(F(" A  Tmax: "));   printValOrNA(out, mod2_t_max_, 1);
  out.print(F(" C  Tfet: "));   printValOrNA(out, mod2_t_fet_, 1);
  out.print(F(" C  Tmin: "));   printValOrNA(out, mod2_t_min_, 1);
  out.print(F(" C  | "));

  out.print(F("[SOC] ")); out.print(soc_mah_);
  out.print(F(" mAh ("));
  printValOrNA(out, soc_percent_, 1);
  out.println(F("%)"));
  out.println(F("------------------------------------------------"));
}

}  // namespace msd_arduino
