#include <Adafruit_MCP23X17.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/gpio.h"

// ─── Pin definitions ──────────────────────────────────────────────────────────
const gpio_num_t PIN_CH1 = GPIO_NUM_40;  // right stick X → cylinder 2
const gpio_num_t PIN_CH2 = GPIO_NUM_39;  // left  stick Y → linear velocity
const gpio_num_t PIN_CH3 = GPIO_NUM_38;  // right stick Y → cylinder 1
const gpio_num_t PIN_CH4 = GPIO_NUM_37;  // left  stick X → angular velocity
const gpio_num_t PIN_CH5 = GPIO_NUM_36;  // switch A      → relay on/off

const gpio_num_t PIN_VM_LEFT  = GPIO_NUM_42;
const gpio_num_t PIN_VM_RIGHT = GPIO_NUM_41;
const gpio_num_t PIN_RELAY    = GPIO_NUM_17;
const gpio_num_t PIN_SDA      = GPIO_NUM_18;
const gpio_num_t PIN_SCL      = GPIO_NUM_8;

// ─── IO expander (MCP23017, addr 0x20) ───────────────────────────────────────
Adafruit_MCP23X17 mcp;
const uint8_t MCP_ADDR      = 0x20;
const uint8_t FWD_L         = 0;
const uint8_t REV_L         = 1;
const uint8_t STOP_MODE_L   = 2;
const uint8_t M0_L          = 3;
const uint8_t ALARM_RESET_L = 4;
const uint8_t MB_FREE_L     = 5;
const uint8_t FWD_R         = 8;
const uint8_t REV_R         = 9;
const uint8_t STOP_MODE_R   = 10;
const uint8_t M0_R          = 11;
const uint8_t ALARM_RESET_R = 12;
const uint8_t MB_FREE_R     = 13;

// ─── ESP-NOW: sub-board address (broadcast) ───────────────────────────────────
uint8_t SUB_BOARD_ADDR[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Command sent to sub-board — must match SubBoardCmd in attachment firmware.
typedef struct {
  float cylinder_1;  // [-1.0, 1.0]  positive = forward
  float cylinder_2;  // [-1.0, 1.0]  positive = forward
  int   power;       // 1 = on, 0 = off
} SubBoardCmd;

// ─── Tunable parameters ───────────────────────────────────────────────────────
const float DEADZONE          = 0.2f;
const float LINEAR_SPEED_MAX  = 0.8;  // [m/s]
const float ANGULAR_SPEED_MAX = 0.764398163f;  // [rad/s]
const float WHEEL_DISTANCE    = 0.600f;         // [m]
const float WHEEL_RADIUS      = 0.1105f;        // [m]
const float REDUCTION_RATIO   = 100.0f;
const int   MAX_RPM           = 4000;

// ─── State ───────────────────────────────────────────────────────────────────
int g_power_relay = 0;
int g_left_rpm    = 0;
int g_right_rpm   = 0;

// ─────────────────────────────────────────────────────────────────────────────

float normPulse(uint16_t pulse) {
  float x = constrain((float)pulse, 1000.0f, 2000.0f);
  return (x - 1500.0f) / 500.0f;
}

float remapDeadzone(float v, float dz) {
  if (v == 0.0f) return 0.0f;
  float sign = (v > 0.0f) ? 1.0f : -1.0f;
  return sign * (abs(v) - dz) / (1.0f - dz);
}

float velToRpm(float v) {
  return (30.0f / M_PI) * (REDUCTION_RATIO / WHEEL_RADIUS) * v;
}

void ik(float v, float w) {
  if (v < 0) w = -w;

  float left_rpm  = velToRpm(v - WHEEL_DISTANCE * w);
  float right_rpm = velToRpm(v + WHEEL_DISTANCE * w);

  if (left_rpm > 0)       { mcp.digitalWrite(FWD_L, HIGH); mcp.digitalWrite(REV_L, LOW);  }
  else if (left_rpm < 0)  { mcp.digitalWrite(FWD_L, LOW);  mcp.digitalWrite(REV_L, HIGH); }
  else                    { mcp.digitalWrite(FWD_L, LOW);  mcp.digitalWrite(REV_L, LOW);  }

  if (right_rpm > 0)      { mcp.digitalWrite(FWD_R, LOW);  mcp.digitalWrite(REV_R, HIGH); }
  else if (right_rpm < 0) { mcp.digitalWrite(FWD_R, HIGH); mcp.digitalWrite(REV_R, LOW);  }
  else                    { mcp.digitalWrite(FWD_R, LOW);  mcp.digitalWrite(REV_R, LOW);  }

  g_left_rpm  = (int)left_rpm;
  g_right_rpm = (int)right_rpm;
}

void writeMotorPwm(int left_rpm, int right_rpm) {
  int l = map(constrain(abs(left_rpm),  0, MAX_RPM), 0, MAX_RPM, 0, 255);
  int r = map(constrain(abs(right_rpm), 0, MAX_RPM), 0, MAX_RPM, 0, 255);
  mcp.digitalWrite(MB_FREE_L, HIGH);
  mcp.digitalWrite(MB_FREE_R, HIGH);
  analogWrite(PIN_VM_LEFT,  l);
  analogWrite(PIN_VM_RIGHT, r);
}

void motorDriverReinit() {
  mcp.digitalWrite(FWD_L,         LOW);
  mcp.digitalWrite(REV_L,         LOW);
  mcp.digitalWrite(FWD_R,         LOW);
  mcp.digitalWrite(REV_R,         LOW);
  mcp.digitalWrite(STOP_MODE_L,   LOW);
  mcp.digitalWrite(STOP_MODE_R,   LOW);
  mcp.digitalWrite(M0_L,          HIGH);
  mcp.digitalWrite(M0_R,          HIGH);
  mcp.digitalWrite(MB_FREE_L,     HIGH);
  mcp.digitalWrite(MB_FREE_R,     HIGH);
  mcp.digitalWrite(ALARM_RESET_L, HIGH);
  mcp.digitalWrite(ALARM_RESET_R, HIGH);
  delay(50);
  mcp.digitalWrite(ALARM_RESET_L, LOW);
  mcp.digitalWrite(ALARM_RESET_R, LOW);
  delay(50);
}

void ioExpanderInit() {
  mcp.begin_I2C(MCP_ADDR);
  mcp.pinMode(FWD_L,         OUTPUT);
  mcp.pinMode(REV_L,         OUTPUT);
  mcp.pinMode(STOP_MODE_L,   OUTPUT);
  mcp.pinMode(M0_L,          OUTPUT);
  mcp.pinMode(ALARM_RESET_L, OUTPUT);
  mcp.pinMode(MB_FREE_L,     OUTPUT);
  mcp.pinMode(FWD_R,         OUTPUT);
  mcp.pinMode(REV_R,         OUTPUT);
  mcp.pinMode(STOP_MODE_R,   OUTPUT);
  mcp.pinMode(M0_R,          OUTPUT);
  mcp.pinMode(ALARM_RESET_R, OUTPUT);
  mcp.pinMode(MB_FREE_R,     OUTPUT);

  mcp.digitalWrite(STOP_MODE_L, LOW);
  mcp.digitalWrite(STOP_MODE_R, LOW);
  mcp.digitalWrite(M0_L,        HIGH);
  mcp.digitalWrite(M0_R,        HIGH);
  mcp.digitalWrite(MB_FREE_L,   HIGH);
  mcp.digitalWrite(MB_FREE_R,   HIGH);
}

void espNowInit() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, SUB_BOARD_ADDR, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  gpio_set_direction(PIN_CH1,      GPIO_MODE_INPUT);
  gpio_set_direction(PIN_CH2,      GPIO_MODE_INPUT);
  gpio_set_direction(PIN_CH3,      GPIO_MODE_INPUT);
  gpio_set_direction(PIN_CH4,      GPIO_MODE_INPUT);
  gpio_set_direction(PIN_CH5,      GPIO_MODE_INPUT);
  gpio_set_direction(PIN_RELAY,    GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_VM_LEFT,  GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_VM_RIGHT, GPIO_MODE_OUTPUT);

  while (gpio_pullup_en(PIN_SDA) != ESP_OK);
  while (gpio_pullup_en(PIN_SCL) != ESP_OK);
  Wire.setPins(PIN_SDA, PIN_SCL);
  Wire.begin();

  ioExpanderInit();
  espNowInit();
  Serial.println("Ready");
}

void loop() {
  // ── Read RC ─────────────────────────────────────────────────────────────────
  uint16_t ch1 = pulseIn(PIN_CH1, HIGH, 25000);  // right stick X
  uint16_t ch2 = pulseIn(PIN_CH2, HIGH, 25000);  // left  stick Y
  uint16_t ch3 = pulseIn(PIN_CH3, HIGH, 25000);  // right stick Y
  uint16_t ch4 = pulseIn(PIN_CH4, HIGH, 25000);  // left  stick X
  uint16_t ch5 = pulseIn(PIN_CH5, HIGH, 25000);  // relay switch

  if (ch5 != 0) g_power_relay = (ch5 > 1500) ? 1 : 0;

  // ── Relay ───────────────────────────────────────────────────────────────────
  static int prev_relay = 0;

  if (g_power_relay == 0) {
    digitalWrite(PIN_RELAY,   LOW);
    analogWrite(PIN_VM_LEFT,  0);
    analogWrite(PIN_VM_RIGHT, 0);
    prev_relay = 0;

    // Send stop command to sub-board
    SubBoardCmd cmd = { 0.0f, 0.0f, 0 };
    esp_now_send(SUB_BOARD_ADDR, (uint8_t *)&cmd, sizeof(cmd));
    return;
  }

  if (prev_relay == 0) {
    digitalWrite(PIN_RELAY, HIGH);
    delay(30);        // wait for relay contacts to close and driver to power up
    motorDriverReinit();  // AlarmReset pulse now reaches a powered driver
  } else {
    digitalWrite(PIN_RELAY, HIGH);
  }
  prev_relay = 1;

  // ── Joystick → velocity ─────────────────────────────────────────────────────
  float joy_y = 0.0f, joy_x = 0.0f;
  float cyl1  = 0.0f, cyl2  = 0.0f;

  if (ch2 != 0 && ch4 != 0) {
    float raw_y = normPulse(ch2);
    float raw_x = normPulse(ch4);
    if (abs(raw_y) < DEADZONE) raw_y = 0.0f;
    if (abs(raw_x) < DEADZONE) raw_x = 0.0f;
    joy_y = remapDeadzone(raw_y, DEADZONE);
    joy_x = remapDeadzone(raw_x, DEADZONE);
  }

  if (ch1 != 0) cyl2 = normPulse(ch1);  // right stick X → cylinder 2
  if (ch3 != 0) cyl1 = normPulse(ch3);  // right stick Y → cylinder 1

  // Reduce linear speed when turning so wheels don't hit the RPM cap
  // asymmetrically — keeps turning authority at high speed.
  float speed_scale = 1.0f - abs(joy_x) * 0.5f;
  float linear_x  = -joy_y * LINEAR_SPEED_MAX * speed_scale;
  float angular_z = -joy_x * ANGULAR_SPEED_MAX;

  // ── Drive motor ─────────────────────────────────────────────────────────────
  ik(linear_x, angular_z);
  writeMotorPwm(g_left_rpm, g_right_rpm);

  // ── Send cylinder command to sub-board via ESP-NOW ───────────────────────────
  SubBoardCmd cmd = { cyl1, cyl2, g_power_relay };
  esp_now_send(SUB_BOARD_ADDR, (uint8_t *)&cmd, sizeof(cmd));
}
