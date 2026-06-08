#ifndef MSD_HPP
#define MSD_HPP

#include <Adafruit_BNO055.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include <bitset>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "Adafruit_NeoPixel.h"
#include "driver/gpio.h"
#include "driver/twai.h"

const gpio_num_t kNeoPixel = GPIO_NUM_48;         // NeoPixel　の出力ピン = 番号;
const gpio_num_t kMoterPowerRelay = GPIO_NUM_17;  // pin No.5 relay swit = ch;
const gpio_num_t kRcTransmitterCh1 = GPIO_NUM_40; // 8 right horizontal = X
const gpio_num_t kRcTransmitterCh2 = GPIO_NUM_39; // 3 left vertical = Y
const gpio_num_t kRcTransmitterCh3 = GPIO_NUM_38; // 46 right vertical = Y
const gpio_num_t kRcTransmitterCh4 = GPIO_NUM_37; // 9 left horizontal = X
const gpio_num_t kRcTransmitterCh5 = GPIO_NUM_36; // 10 A
const gpio_num_t kRcTransmitterCh6 = GPIO_NUM_35; // 11 B
const gpio_num_t kSDA = GPIO_NUM_18;
const gpio_num_t kSCL = GPIO_NUM_8;
const gpio_num_t kUart1Tx = GPIO_NUM_4;
const gpio_num_t kUart1Rx = GPIO_NUM_5;
const gpio_num_t kUart2Tx = GPIO_NUM_15;
const gpio_num_t kUart2Rx = GPIO_NUM_16;
const gpio_num_t kVMLeft = GPIO_NUM_42;
const gpio_num_t kVMRight = GPIO_NUM_41;

// IO expander pin assign
const uint8_t kIoExpanderFowardLeft = 0;     // FWD_L
const uint8_t kIoExpanderRverseLeft = 1;     // REV_L
const uint8_t kIoExpanderStopModeLeft = 2;   // STOP_MODE_L
const uint8_t kIoExpanderM0Left = 3;         // M0_L
const uint8_t kIoExpanderAlarmResetLeft = 4; // ALARM_RESET_L
const uint8_t kIoExpanderMbFreeLeft = 5;     // MB_FREE_L
const uint8_t kIoExpanderSpeedOutLeft = 6;   // SPEED_OUT_L
const uint8_t kIoExpanderEmergencyStopSwitch1 = 7;
const uint8_t kIoExpanderFowardRight = 8;      // FWD_R
const uint8_t kIoExpanderRverseRight = 9;      // REV_R
const uint8_t kIoExpanderStopModeRight = 10;   // STOP_MODE_R
const uint8_t kIoExpanderM0Right = 11;         // M0_R
const uint8_t kIoExpanderAlarmResetRight = 12; // ALARM_RESET_R
const uint8_t kIoExpanderMbFreeRight = 13;     // MB_FREE_R
const uint8_t kIoExpanderSpeedOutRight = 14;   // SPEED_OUT_R
const uint8_t kIoExpanderEmergencyStopSwitch2 = 15;

const int LED_COUNT = 1;  // LEDの連 = 結数;
const int BRIGHTNESS = 1; //  = 輝度;
Adafruit_NeoPixel pixels(LED_COUNT, kNeoPixel, NEO_GRB + NEO_KHZ800);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

Adafruit_MCP23X17 mcp;
const int kMCP_Address = 0x20;

// ######### parameters #########
// calculation velocity to rpm
#define M_PI 3.14
double LINEAR_SPEED_MAX = 0.95;
double ANGULAR_SPEED_MAX = 0.85;

void IoExpanderInitialize();
void GpioModeSet();
void ik(float v, float w);
void stopMotor(bool motor_brake_state);
void writeMotorPwm(int left_rpm, int right_rpm);
void receivePropoSwitch();

float velToRpm(float v);
#endif
