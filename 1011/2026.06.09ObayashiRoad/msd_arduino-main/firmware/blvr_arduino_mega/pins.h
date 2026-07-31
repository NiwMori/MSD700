// pins.h — all pin assignments for the Arduino Mega on MSD2026 (BLV-R variant).
//
// BLV-R uses RS485 Modbus RTU instead of GPIO motor control.
// Motor GPIO pins (D28-37, D3, D4) are freed and replaced by a single
// RS485 DE/RE control pin.
//
// Arduino Mega 2560 UART map:
//   Serial   : pins 0 / 1    USB-CDC (debug)
//   Serial1  : pins 18 / 19  (TX1 / RX1) — SBUS
//   Serial2  : pins 16 / 17  (TX2 / RX2) — RS485 Modbus RTU to BLV-R
//   Serial3  : pins 14 / 15  (TX3 / RX3) — Jetson LINK

#ifndef MSD_BLVR_PINS_H_
#define MSD_BLVR_PINS_H_

#include <Arduino.h>

namespace msd_arduino
{
namespace pins
{

// =====================================================================
// Status LEDs (analog pins used as digital outputs)
// =====================================================================
static const uint8_t LED_RED    = A3;
static const uint8_t LED_GREEN  = A4;
static const uint8_t LED_BLUE   = A5;
static const uint8_t LED_YELLOW = A6;

// =====================================================================
// RS485 transceiver — DE/RE pin for Modbus RTU (BLV-R motor drivers)
// =====================================================================
static const uint8_t RS485_DE_RE = 23;  // HIGH = TX, LOW = RX

// =====================================================================
// Arm cylinders — direction outputs (FWD high → extend, REV high → retract)
// Never set FWD and REV both HIGH simultaneously.
// =====================================================================
static const uint8_t CYL1_FWD = 38;
static const uint8_t CYL1_REV = 39;
static const uint8_t CYL2_FWD = 40;
static const uint8_t CYL2_REV = 41;
static const uint8_t CYL3_FWD = 42;  // reserved (unused in this build)
static const uint8_t CYL3_REV = 43;

// =====================================================================
// Cylinder feedback (end-stops + analog potentiometer)
// =====================================================================
static const uint8_t CYL1_END_IN   = 44;
static const uint8_t CYL1_END_OUT  = 45;
static const uint8_t CYL1_FEEDBACK = A0;

static const uint8_t CYL2_END_IN   = 46;
static const uint8_t CYL2_END_OUT  = 47;
static const uint8_t CYL2_FEEDBACK = A1;

static const uint8_t CYL3_END_IN   = 48;   // reserved
static const uint8_t CYL3_END_OUT  = 49;
static const uint8_t CYL3_FEEDBACK = A2;

// =====================================================================
// I2C (BNO055 IMU + BME280) — fixed pins on Mega
// =====================================================================
//   SDA = pin 20
//   SCL = pin 21

// =====================================================================
// Emergency-stop mode switch (hardware interlock)
// D22 HIGH (energized) = e-stop ON  (stop all motion)
// D22 LOW              = e-stop OFF (normal operation)
// =====================================================================
static const uint8_t MODE_SWITCH   = 22;

// =====================================================================
// Power relay & warning lights
// =====================================================================
static const uint8_t RELAY_POWER   = 24;
static const uint8_t LIGHT_RED     = 25;
static const uint8_t LIGHT_YELLOW  = 26;

// =====================================================================
// MCP2515 CAN bus (battery BMS)
// =====================================================================
static const uint8_t CAN_SPI_CS  = 53;
static const uint8_t CAN_INT     = 2;
// SPI bus uses hardware pins: MISO=50, MOSI=51, SCK=52

}  // namespace pins
}  // namespace msd_arduino

#endif  // MSD_BLVR_PINS_H_
