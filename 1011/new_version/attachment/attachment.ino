#include <WiFi.h>
#include <esp_now.h>
#include "driver/gpio.h"

// ─── Pin definitions ──────────────────────────────────────────────────────────
const gpio_num_t PIN_CYL1_FWD = GPIO_NUM_15;  // cylinder 1 forward
const gpio_num_t PIN_CYL1_REV = GPIO_NUM_16;  // cylinder 1 reverse
const gpio_num_t PIN_CYL2_FWD = GPIO_NUM_17;  // cylinder 2 forward
const gpio_num_t PIN_CYL2_REV = GPIO_NUM_18;  // cylinder 2 reverse
const gpio_num_t PIN_RELAY     = GPIO_NUM_1;   // power relay

const gpio_num_t PIN_PATLITE_GREEN  = GPIO_NUM_21;  // green  — system ready
const gpio_num_t PIN_PATLITE_YELLOW = GPIO_NUM_47;  // yellow — relay ON / active
const gpio_num_t PIN_PATLITE_RED    = GPIO_NUM_8;   // red    — no signal (timeout)

// ─── Command struct ───────────────────────────────────────────────────────────
// Must match SubBoardCmd in main board firmware exactly.
typedef struct {
  float cylinder_1;  // [-1.0, 1.0]  positive = forward
  float cylinder_2;  // [-1.0, 1.0]  positive = forward
  int   power;       // 1 = on, 0 = off
} SubBoardCmd;

// ─── State ───────────────────────────────────────────────────────────────────
float g_cyl1  = 0;
float g_cyl2  = 0;
int   g_power = 0;
unsigned long g_last_recv = 0;
bool  g_timeout = true;

// ─── ESP-NOW receive callback ─────────────────────────────────────────────────
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(SubBoardCmd)) return;

  SubBoardCmd cmd;
  memcpy(&cmd, data, sizeof(cmd));

  g_cyl1      = cmd.cylinder_1;
  g_cyl2      = cmd.cylinder_2;
  g_power     = cmd.power;
  g_last_recv = millis();
  g_timeout   = false;
}

// ─── Cylinder control ─────────────────────────────────────────────────────────
void moveCylinder(float value, gpio_num_t pin_fwd, gpio_num_t pin_rev) {
  if (value >= 0.5f) {
    digitalWrite(pin_fwd, HIGH);
    digitalWrite(pin_rev, LOW);
  } else if (value <= -0.5f) {
    digitalWrite(pin_fwd, LOW);
    digitalWrite(pin_rev, HIGH);
  } else {
    digitalWrite(pin_fwd, LOW);
    digitalWrite(pin_rev, LOW);
  }
}

// ─── Patlite ──────────────────────────────────────────────────────────────────
// GREEN  ON  = system running (always on after setup)
// YELLOW ON  = relay active (power on, cylinders can move)
// RED    ON  = no signal from main board (timeout)
void updatePatlite(bool timeout, bool power_on) {
  digitalWrite(PIN_PATLITE_GREEN,  HIGH);          // always on
  digitalWrite(PIN_PATLITE_YELLOW, power_on ? HIGH : LOW);
  digitalWrite(PIN_PATLITE_RED,    timeout  ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  gpio_set_direction(PIN_CYL1_FWD,      GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_CYL1_REV,      GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_CYL2_FWD,      GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_CYL2_REV,      GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_RELAY,         GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_PATLITE_GREEN,  GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_PATLITE_YELLOW, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_PATLITE_RED,    GPIO_MODE_OUTPUT);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  updatePatlite(true, false);  // red on at start (waiting for signal)
  Serial.println("Ready");
}

void loop() {
  // ── Timeout check ────────────────────────────────────────────────────────────
  if (millis() - g_last_recv > 300) {
    g_cyl1    = 0;
    g_cyl2    = 0;
    g_power   = 0;
    g_timeout = true;
  }

  // ── Apply commands ───────────────────────────────────────────────────────────
  if (g_power == 1) {
    digitalWrite(PIN_RELAY, HIGH);
    moveCylinder(g_cyl1, PIN_CYL1_FWD, PIN_CYL1_REV);
    moveCylinder(g_cyl2, PIN_CYL2_FWD, PIN_CYL2_REV);
  } else {
    digitalWrite(PIN_RELAY, LOW);
    digitalWrite(PIN_CYL1_FWD, LOW);
    digitalWrite(PIN_CYL1_REV, LOW);
    digitalWrite(PIN_CYL2_FWD, LOW);
    digitalWrite(PIN_CYL2_REV, LOW);
  }

  // ── Update Patlite ───────────────────────────────────────────────────────────
  updatePatlite(g_timeout, g_power == 1);
}
