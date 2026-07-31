// pins.h — all pin assignments for the Arduino Mega on MSD2026.
//
// Keeping them here (and *only* here) makes it easy to audit against the
// wiring harness and to move pins without touching the rest of the code.
//
// Arduino Mega 2560 UART map (shared with Serial0/1/2/3):
//   Serial   : pins 0 / 1    USB-CDC
//   Serial1  : pins 18 / 19  (TX1 / RX1)
//   Serial2  : pins 16 / 17  (TX2 / RX2)
//   Serial3  : pins 14 / 15  (TX3 / RX3)

#ifndef MSD_BLV_PINS_H_
#define MSD_BLV_PINS_H_

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
// BLV driver — LEFT motor
// =====================================================================
static const uint8_t M_LEFT_FWD       = 28;
static const uint8_t M_LEFT_REV       = 29;
static const uint8_t M_LEFT_STOP_MODE = 30;
static const uint8_t M_LEFT_M0        = 31;
static const uint8_t M_LEFT_MB_FREE   = 32;
static const uint8_t M_LEFT_VM        = 3;   // PWM, left mmPin

// =====================================================================
// BLV driver — RIGHT motor
// =====================================================================
static const uint8_t M_RIGHT_FWD       = 33;
static const uint8_t M_RIGHT_REV       = 34;
static const uint8_t M_RIGHT_STOP_MODE = 35;
static const uint8_t M_RIGHT_M0        = 36;
static const uint8_t M_RIGHT_MB_FREE   = 37;
static const uint8_t M_RIGHT_VM        = 4;  // PWM, right mmPin

// =====================================================================
// Arm cylinders — direction outputs (FWD high → extend, REV high → retract)
// Never set FWD and REV both HIGH simultaneously.
// =====================================================================
static const uint8_t CYL1_FWD = 38;
static const uint8_t CYL1_REV = 39;
static const uint8_t CYL2_FWD = 40;  // (cyl2 assumed D40/D41; see docs)
static const uint8_t CYL2_REV = 41;
static const uint8_t CYL3_FWD = 42;  // reserved (unused in this build)
static const uint8_t CYL3_REV = 43;

// =====================================================================
// Cylinder feedback (end-stops + analog potentiometer)
// =====================================================================
static const uint8_t CYL1_END_IN   = 44;
static const uint8_t CYL1_END_OUT  = 45;
static const uint8_t CYL1_FEEDBACK = A0;   // analog in

static const uint8_t CYL2_END_IN   = 46;
static const uint8_t CYL2_END_OUT  = 47;
static const uint8_t CYL2_FEEDBACK = A1;

static const uint8_t CYL3_END_IN   = 48;   // reserved
static const uint8_t CYL3_END_OUT  = 49;
static const uint8_t CYL3_FEEDBACK = A2;

// =====================================================================
// I2C (BNO055 IMU) — fixed pins on Mega
// =====================================================================
//   SDA = pin 20
//   SCL = pin 21
// BNO055 I2C address = 0x28

// =====================================================================
// Emergency-stop mode switch (hardware interlock)
// D22 HIGH (energized) = e-stop ON  (stop all motion)
// D22 LOW              = e-stop OFF (normal operation)
// =====================================================================
static const uint8_t MODE_SWITCH   = 22;

// =====================================================================
// Power relay & warning lights
// =====================================================================
static const uint8_t RELAY_POWER   = 24;  // HIGH = power ON, LOW = power OFF
static const uint8_t LIGHT_RED     = 25;  // external warning: power OFF indicator
static const uint8_t LIGHT_YELLOW  = 26;  // external warning: battery low (< 10%)

// =====================================================================
// MCP2515 CAN bus (battery BMS)
// =====================================================================
static const uint8_t CAN_SPI_CS  = 53;   // SPI chip-select for MCP2515
static const uint8_t CAN_INT     = 2;    // interrupt pin (active-LOW)
// SPI bus uses hardware pins: MISO=50, MOSI=51, SCK=52

}  // namespace pins
}  // namespace msd_arduino

#endif  // MSD_BLV_PINS_H_
