#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include <bitset>
#include <sstream>

#include "driver/gpio.h"
#include "driver/twai.h"

#include <WiFi.h>
#include <esp_now.h>

const gpio_num_t kCylinder1Forward = GPIO_NUM_15;
const gpio_num_t kCylinder1Reverse = GPIO_NUM_16;
const gpio_num_t kCylinder2Forward = GPIO_NUM_17;
const gpio_num_t kCylinder2Reverse = GPIO_NUM_18;
const gpio_num_t kCylinder1Feedback = GPIO_NUM_19;
const gpio_num_t kCylinder2Feedback = GPIO_NUM_2;

const gpio_num_t kPowerRelay = GPIO_NUM_1;
const gpio_num_t kEmergencyStopSwitch1 = GPIO_NUM_11;
const gpio_num_t kEmergencyStopSwitch2 = GPIO_NUM_12;

const gpio_num_t kPatliteRedLight = GPIO_NUM_8;
const gpio_num_t kPatliteGreenLight = GPIO_NUM_21;
const gpio_num_t kPatliteYellowLight = GPIO_NUM_47;
const gpio_num_t kPatliteRedFlash = GPIO_NUM_10;
const gpio_num_t kPatliteYellowFlash = GPIO_NUM_9;

const gpio_num_t kUart1Tx = GPIO_NUM_7; // UART1: Main
const gpio_num_t kUart1RX = GPIO_NUM_6;
const gpio_num_t kUart2Tx = GPIO_NUM_5; // UART2: Jetson
const gpio_num_t kUart2RX = GPIO_NUM_4;

const int kDebugRate = 115200;
const int kUart1Rate = 4800;
const int kUart2Rate = 4800;

float g_cylinder_1;
float g_cylinder_2;

void MoveCylinder(float input_value, gpio_num_t gpio_forward, gpio_num_t gpio_reverse);
void StopCylinder(gpio_num_t gpio_cylinder1_forward, gpio_num_t gpio_cylinder1_reverse, gpio_num_t gpio_cylinder2_forward, gpio_num_t gpio_cylinder2_reverse);

bool Patlite(String color, String mode, bool flag);

float cylinder_1 = 0;
float cylinder_2 = 0;
int power_relay = 0;
int msd_state = 4;

// Structured command sent by the main board when in RC mode (len == 12).
// NOTE: keep this struct in sync with SubBoardCmd in main board firmware.
typedef struct
{
  float cylinder_1; // [-1.0, 1.0], positive = forward
  float cylinder_2; // [-1.0, 1.0], positive = forward
  int power;        // 1 = on, 0 = off
} SubBoardCmd;

// Legacy comma-string sent by the white controller when in ESP-NOW mode (len == 32).
// Format: "emergStop,rJoyX,rJoyY,?,?,?,?,"  values are raw ADC 0-1023
typedef struct
{
  char message[32];
} WhiteControllerMsg;

int g_power_relay = 0;
unsigned long lastReceiveTime = 0;

void OnDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len)
{
  lastReceiveTime = millis();

  if (len == sizeof(SubBoardCmd))
  {
    // RC mode: structured command from main board
    SubBoardCmd cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));
    cylinder_1 = cmd.cylinder_1;
    cylinder_2 = cmd.cylinder_2;
    g_power_relay = cmd.power;
  }
  else if (len == sizeof(WhiteControllerMsg))
  {
    // ESP-NOW mode: comma-string from white controller
    WhiteControllerMsg msg;
    memcpy(&msg, incomingData, sizeof(msg));

    int parsedData[7];
    String message = String(msg.message);
    for (int i = 0; i < 7; i++)
    {
      int commaIndex = message.indexOf(',');
      if (i <= 5 && commaIndex == -1) return;
      parsedData[i] = message.substring(0, commaIndex).toInt();
      message = message.substring(commaIndex + 1);
    }

    g_power_relay = 1 - parsedData[0];
    cylinder_1 = map(parsedData[2], 0, 1023, -100, 100) / 100.0;
    cylinder_2 = map(parsedData[1], 0, 1023, -100, 100) / 100.0 * (-1.0);
  }
}

void checkEspNowTimeOut()
{
  // Main board loop takes ~120ms (6ch pulseIn). Use 300ms to safely cover
  // worst-case send interval and prevent false timeout between packets.
  if (millis() - lastReceiveTime > 300)
  {
    cylinder_1 = 0;
    cylinder_2 = 0;
  }
}

// ######## ESP-NOWの試験コードのエリアここまで #########

void setup()
{
  // GPIO mode init
  gpio_set_direction(kCylinder1Forward, GPIO_MODE_OUTPUT);
  gpio_set_direction(kCylinder1Reverse, GPIO_MODE_OUTPUT);
  gpio_set_direction(kCylinder2Forward, GPIO_MODE_OUTPUT);
  gpio_set_direction(kCylinder2Reverse, GPIO_MODE_OUTPUT);
  gpio_set_direction(kPowerRelay, GPIO_MODE_OUTPUT);
  gpio_set_direction(kEmergencyStopSwitch1, GPIO_MODE_INPUT);
  gpio_set_direction(kEmergencyStopSwitch2, GPIO_MODE_INPUT);
  gpio_set_direction(kPatliteRedLight, GPIO_MODE_OUTPUT);
  gpio_set_direction(kPatliteGreenLight, GPIO_MODE_OUTPUT);
  gpio_set_direction(kPatliteYellowLight, GPIO_MODE_OUTPUT);
  gpio_set_direction(kPatliteRedFlash, GPIO_MODE_OUTPUT);
  gpio_set_direction(kPatliteYellowFlash, GPIO_MODE_OUTPUT);

  gpio_set_direction(kCylinder1Feedback, GPIO_MODE_INPUT);
  gpio_set_direction(kCylinder2Feedback, GPIO_MODE_INPUT);

  // GPIO pullup or pulldown
  while (gpio_pulldown_en(kEmergencyStopSwitch1) != ESP_OK)
    ;
  while (gpio_pulldown_en(kEmergencyStopSwitch2) != ESP_OK)
    ;

  digitalWrite(kPatliteRedLight, LOW);
  digitalWrite(kPatliteRedFlash, LOW);
  digitalWrite(kPatliteYellowLight, LOW);
  digitalWrite(kPatliteYellowFlash, LOW);
  digitalWrite(kPatliteGreenLight, LOW);

  Serial.begin(kDebugRate);
  delay(500);  // allow USB CDC host to connect; don't block indefinitely
  Serial1.begin(kUart1Rate, SERIAL_8E1, kUart1Tx, kUart1RX);

  // ######### ESP-NOWのテストコードのエリア #########
  // ESP-NOWの初期化
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // ESP-NOWの受信コールバック関数を登録
  esp_now_register_recv_cb(OnDataRecv);
  // ----------------------------------------------
  Patlite("Green", "Light", false); // Green light on
  Serial.println("===== Start Program =====");
}

void loop()
{
  checkEspNowTimeOut();
  if (g_power_relay == 1)
  {
    digitalWrite(kPowerRelay, HIGH);
    Patlite("Yellow", "Light", true);
    MoveCylinder(cylinder_1, kCylinder1Forward, kCylinder1Reverse);
    MoveCylinder(cylinder_2, kCylinder2Forward, kCylinder2Reverse);
  }
  else
  {
    digitalWrite(kPowerRelay, LOW);
    Patlite("Yellow", "Light", false);
    StopCylinder(kCylinder1Forward, kCylinder1Reverse, kCylinder2Forward, kCylinder2Reverse);
  }
}

void MoveCylinder(float input_value, gpio_num_t gpio_forward, gpio_num_t gpio_reverse)
{
  if (input_value >= 0.5)
  {
    digitalWrite(gpio_forward, HIGH);
    digitalWrite(gpio_reverse, LOW);
    // Serial.println("-----> Forward\n");
  }
  else if (input_value <= -0.5)
  {
    digitalWrite(gpio_forward, LOW);
    digitalWrite(gpio_reverse, HIGH);
    // Serial.println("-----> Reverse\n");
  }
  else
  {
    digitalWrite(gpio_forward, LOW);
    digitalWrite(gpio_reverse, LOW);
  }
}

void StopCylinder(gpio_num_t gpio_cylinder1_forward, gpio_num_t gpio_cylinder1_reverse, gpio_num_t gpio_cylinder2_forward, gpio_num_t gpio_cylinder2_reverse)
{
  digitalWrite(gpio_cylinder1_forward, LOW);
  digitalWrite(gpio_cylinder1_reverse, LOW);
  digitalWrite(gpio_cylinder2_forward, LOW);
  digitalWrite(gpio_cylinder2_reverse, LOW);
}

bool Patlite(String color, String mode, bool flag)
{
  if (color == "Red")
  {
    if (mode == "Flash")
    {
      if (flag)
      {
        digitalWrite(kPatliteRedLight, LOW);
        digitalWrite(kPatliteRedFlash, LOW);
        flag = false;
      }
      else
      {
        digitalWrite(kPatliteRedLight, HIGH);
        digitalWrite(kPatliteRedFlash, HIGH);
        flag = true;
        Serial.println("Red on...");
      }
    }
    else if (mode == "Light")
    {
      if (flag)
      {
        digitalWrite(kPatliteRedLight, LOW);
        flag = false;
      }
      else
      {
        digitalWrite(kPatliteRedLight, HIGH);
        flag = true;
        Serial.println("Red on...");
      }
    }
  }
  else if (color == "Green")
  {
    if (mode == "Flash")
    {
      Serial.println("Function is nothing...");
    }
    else if (mode == "Light")
    {
      if (flag)
      {
        digitalWrite(kPatliteGreenLight, LOW);
        flag = false;
      }
      else
      {
        digitalWrite(kPatliteGreenLight, HIGH);
        flag = true;
        Serial.println("Green on...");
      }
    }
  }
  else if (color == "Yellow")
  {
    if (mode == "Flash")
    {
      if (flag)
      {
        digitalWrite(kPatliteYellowLight, LOW);
        digitalWrite(kPatliteYellowFlash, LOW);
        flag = false;
      }
      else
      {
        digitalWrite(kPatliteYellowLight, HIGH);
        digitalWrite(kPatliteYellowFlash, HIGH);
        flag = true;
        Serial.println("Yellow on...");
      }
    }
    else if (mode == "Light")
    {
      if (flag)
      {
        digitalWrite(kPatliteYellowLight, LOW);
        flag = false;
      }
      else
      {
        digitalWrite(kPatliteYellowLight, HIGH);
        flag = true;
        Serial.println("Yellow on...");
      }
    }
  }

  return flag;
}