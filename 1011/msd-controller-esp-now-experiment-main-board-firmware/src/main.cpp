#include <WiFi.h>
#include <esp_now.h>

#include "msd.hpp"

#define ENABLE_RC_DEBUG 1

// ######### parameters #########
float g_input_propo_joy_leftY;
float g_input_propo_joy_leftX;
float g_input_propo_joy_rightY;
float g_input_propo_joy_rightX;
// control command data
double g_linear_x = 0;
double g_angular_z = 0;
int g_power_relay = 0; // propo switch A : 0:on 1:off
// calculate variable for encoder
int g_left_rpm = 0;
int g_right_rpm = 0;
// propo switch variable
int g_select_controler = 0; // propo switch C : 0:propo 1:jetson
bool g_motor_brake = true;  // false:Neutral true:Brake On
bool g_control_reverse = false;

enum ControlMode { MODE_RC = 0, MODE_ESPNOW = 1 };
ControlMode g_control_mode = MODE_RC;

// Command sent to sub-board when in RC mode.
// NOTE: keep this struct in sync with SubBoardCmd in sub-board firmware.
typedef struct
{
  float cylinder_1; // [-1.0, 1.0], positive = forward
  float cylinder_2; // [-1.0, 1.0], positive = forward
  int power;        // 1 = on, 0 = off
} SubBoardCmd;

uint8_t g_sub_board_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast

// ######## ESP-NOWの試験コードのエリア #########
// 受信データの構造体
unsigned long lastReceiveTime = 0;    // last ESP-NOW receive time
unsigned long lastRcReceiveTime = 0;  // last valid RC signal time
typedef struct struct_message
{
  char message[32];
} struct_message;

struct_message recvData;

void OnDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len)
{
  // 突貫で書いたのでぐちゃぐちゃなのは勘弁してください
  float deadzone = 0.3; // ジョイスティックのデッドゾーン

  memcpy(&recvData, incomingData, sizeof(recvData));
  // Serial.println(recvData.message);

  int parsedData[7];

  int emergencyStopSwitch = 0;
  int leftJoyStickX = 0;
  int leftJoyStickY = 0;

  String message = String(recvData.message);
  // Serial.println(message);

  int lastCommaIndex = 0; // 最後に見つかったカンマの文字列先頭からの位置
  bool invalidData = false;

  for (int i = 0; i < 7; i++)
  {
    lastCommaIndex = message.indexOf(',');
    // Serial.println(lastCommaIndex);
    if (i <= 5 && lastCommaIndex == -1)
    {
      invalidData = true;
      return; // カンマが見つからない場合は無効なデータ
    }
    parsedData[i] = message.substring(0, lastCommaIndex).toInt();
    message = message.substring(lastCommaIndex + 1);
    invalidData = false;
    lastReceiveTime = millis(); // データを受信した時間を更新
  }

  // Serial.print("ESP-NOW Data: ");
  // for (int i = 0; i < 7; i++) {
  //   Serial.print(parsedData[i]);
  //   Serial.print(",");
  // }
  // Serial.println();

  emergencyStopSwitch = parsedData[0];
  leftJoyStickX = parsedData[4];
  leftJoyStickY = parsedData[5];

  // とりあえずプロポ用の処理を流用
  float input_propo_joy_leftY = map(leftJoyStickY, 0, 1023, -100, 100) / 100.0;
  float input_propo_joy_leftX = map(leftJoyStickX, 0, 1023, -100, 100) / 100.0 * (-1.0); // 左右反転

  // dead zone
  if (-deadzone <= input_propo_joy_leftX && input_propo_joy_leftX <= deadzone)
  {
    input_propo_joy_leftX = 0.00;
  }
  if (-deadzone <= input_propo_joy_leftY && input_propo_joy_leftY <= deadzone)
  {
    input_propo_joy_leftY = 0.00;
  }

  // g_power_relay = 1 - emergencyStopSwitch; // プロポのスイッチは押すと電源が入る仕様なので、1から引いて反転
  g_power_relay = 1; // 不具合のため、常に電源を入れた状態にする
  g_select_controler = 0;
  g_control_mode = MODE_ESPNOW;
  g_input_propo_joy_leftY = input_propo_joy_leftY;
  g_input_propo_joy_leftX = input_propo_joy_leftX;
}

void checkEspNowTimeOut()
{
  if (g_control_mode == MODE_ESPNOW && millis() - lastReceiveTime > 100)
  {
    g_input_propo_joy_leftY = 0;
    g_input_propo_joy_leftX = 0;
    g_control_mode = MODE_RC;
  }
}

void checkRcTimeOut()
{
  // 500ms covers worst-case pulseIn delay (6ch × 25ms) plus margin.
  // Handles both zero-output and failsafe-value receivers.
  if (millis() - lastRcReceiveTime > 500)
  {
    g_input_propo_joy_leftX = 0;
    g_input_propo_joy_leftY = 0;
    g_input_propo_joy_rightX = 0;
    g_input_propo_joy_rightY = 0;
    g_power_relay = 0;
  }
}

// ######## ESP-NOWの試験コードのエリアここまで #########

void IoExpanderInitialize()
{
  mcp.begin_I2C(kMCP_Address);
  mcp.pinMode(kIoExpanderFowardLeft, OUTPUT);
  mcp.pinMode(kIoExpanderRverseLeft, OUTPUT);
  mcp.pinMode(kIoExpanderStopModeLeft, OUTPUT);
  mcp.pinMode(kIoExpanderM0Left, OUTPUT);
  mcp.pinMode(kIoExpanderAlarmResetLeft, OUTPUT);
  mcp.pinMode(kIoExpanderMbFreeLeft, OUTPUT);
  mcp.pinMode(kIoExpanderEmergencyStopSwitch1, OUTPUT);

  mcp.pinMode(kIoExpanderFowardRight, OUTPUT);
  mcp.pinMode(kIoExpanderRverseRight, OUTPUT);
  mcp.pinMode(kIoExpanderStopModeRight, OUTPUT);
  mcp.pinMode(kIoExpanderM0Right, OUTPUT);
  mcp.pinMode(kIoExpanderAlarmResetRight, OUTPUT);
  mcp.pinMode(kIoExpanderMbFreeRight, OUTPUT);
  mcp.pinMode(kIoExpanderEmergencyStopSwitch2, OUTPUT);

  mcp.pinMode(kIoExpanderSpeedOutLeft, INPUT);
  mcp.pinMode(kIoExpanderSpeedOutRight, INPUT);

  mcp.digitalWrite(kIoExpanderStopModeRight, LOW);
  mcp.digitalWrite(kIoExpanderStopModeLeft, LOW);
  mcp.digitalWrite(kIoExpanderMbFreeRight, HIGH);
  mcp.digitalWrite(kIoExpanderMbFreeLeft, HIGH);
  mcp.digitalWrite(kIoExpanderM0Right, HIGH);
  mcp.digitalWrite(kIoExpanderM0Left, HIGH);
}

void GpioModeSet()
{
  gpio_set_direction(kRcTransmitterCh1, GPIO_MODE_INPUT);
  gpio_set_direction(kRcTransmitterCh3, GPIO_MODE_INPUT);
  gpio_set_direction(kRcTransmitterCh2, GPIO_MODE_INPUT);
  gpio_set_direction(kRcTransmitterCh4, GPIO_MODE_INPUT);
  gpio_set_direction(kRcTransmitterCh5, GPIO_MODE_INPUT);
  gpio_set_direction(kRcTransmitterCh6, GPIO_MODE_INPUT);
  gpio_set_direction(kMoterPowerRelay, GPIO_MODE_OUTPUT);

  gpio_set_direction(kVMRight, GPIO_MODE_OUTPUT);
  gpio_set_direction(kVMLeft, GPIO_MODE_OUTPUT);
}

void setup()
{
  Serial.begin(115200); // For debug
  while (!Serial)
  {
    printf("---> Initialize Error: Serial\n");
    continue;
  }
  Serial1.begin(115200, SERIAL_8N1, kUart1Rx, kUart1Tx); // For Jetson
  while (!Serial1)
  {
    printf("---> Initialize Error: Serial1\n");
    continue;
  }
  Serial1.setTimeout(100);
  GpioModeSet();

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

  // Register broadcast peer for sending to sub-board
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, g_sub_board_addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  // ----------------------------------------------

  while (gpio_pullup_en(kSDA) != ESP_OK)
    continue;
  while (gpio_pullup_en(kSCL) != ESP_OK)
    continue;
  Wire.setPins(kSDA, kSCL);
  Wire.begin();
  bno.begin();
  bno.setExtCrystalUse(true);

  IoExpanderInitialize();
}

void loop()
{
  int msd_state;
  // # input control command
  receivePropoSwitch(); // ESP-NOWの実験のため無効化

  // Serial.print("Commands: ");
  // Serial.print(g_linear_x);
  // Serial.print(", ");
  // Serial.print(g_angular_z);
  // Serial.print(", ");
  // Serial.print(g_cylinder_1);
  // Serial.print(", ");
  // Serial.print(g_cylinder_2);
  // Serial.print(", ");
  // Serial.print(g_cylinder_3);
  // Serial.print(", ");
  // Serial.print(g_power_relay);
  // Serial.print(", ");
  // Serial.print(g_select_controler);
  // Serial.println();

  checkEspNowTimeOut();
  checkRcTimeOut();

  // Send cylinder commands to sub-board when in RC mode.
  // In ESP-NOW mode the white controller talks to the sub-board directly.
  if (g_control_mode == MODE_RC)
  {
    SubBoardCmd sub_cmd;
    sub_cmd.cylinder_1 = g_input_propo_joy_rightY; // CH3: right stick Y
    sub_cmd.cylinder_2 = g_input_propo_joy_rightX; // CH1: right stick X
    sub_cmd.power = g_power_relay;
    esp_now_send(g_sub_board_addr, (uint8_t *)&sub_cmd, sizeof(sub_cmd));
  }

  // # move
  // ## motor drive
  switch (g_power_relay)
  {
  case 0:
    digitalWrite(kMoterPowerRelay, LOW);
    stopMotor(true); // ブレーキをかける
    msd_state = 2;
    break;
  case 1:
    digitalWrite(kMoterPowerRelay, HIGH);

    if (g_control_mode == MODE_RC)
    {
      float joy_x = g_input_propo_joy_leftX;
      float joy_y = g_input_propo_joy_leftY;
      if (!g_control_reverse)
        joy_y = -joy_y;

      float angular_sign = -1.0f;
      g_linear_x = (double)(joy_y * LINEAR_SPEED_MAX);
      g_angular_z = (double)(joy_x * angular_sign * ANGULAR_SPEED_MAX);
    }
    else
    {
      g_linear_x = g_input_propo_joy_leftY * LINEAR_SPEED_MAX;
      g_angular_z = g_input_propo_joy_leftX * ANGULAR_SPEED_MAX;
    }

    pixels.setPixelColor(0, pixels.Color(BRIGHTNESS, 0, BRIGHTNESS));
    pixels.show();
    msd_state = 1;

    ik(g_linear_x, g_angular_z);
    writeMotorPwm(g_left_rpm, g_right_rpm);
  }
}

float velToRpm(float v)
{
  // convert velocity to rpm
  const float REDUCTION_RATIO = 100.0; // 減速比 reduction ratio
  const float WHEEL_RADIUS = 0.1105;   // 車輪半径 radius of gyration [m]
  float n = 30 / M_PI * REDUCTION_RATIO / WHEEL_RADIUS * v;

  return n;
}

void ik(float v, float w)
{
  // calculate linear velocity and angur velocity to left and right velocity
  float WHEEL_DISTANCE = 0.600; // 車輪間距離 distance between the wheels [m]
  float v_l = v - WHEEL_DISTANCE * w;
  float v_r = v + WHEEL_DISTANCE * w;

  float left_rpm = velToRpm(v_l);
  float right_rpm = velToRpm(v_r);

  // direction of motor
  if (left_rpm > 0)
  {
    mcp.digitalWrite(kIoExpanderFowardLeft, HIGH);
    mcp.digitalWrite(kIoExpanderRverseLeft, LOW);
    // Serial.print("Left dir:FWD, ");
  }
  else if (left_rpm < 0)
  {
    mcp.digitalWrite(kIoExpanderFowardLeft, LOW);
    mcp.digitalWrite(kIoExpanderRverseLeft, HIGH);
    // Serial.print("Left dir:REV, ");
  }
  else
  {
    mcp.digitalWrite(kIoExpanderFowardLeft, LOW);
    mcp.digitalWrite(kIoExpanderRverseLeft, LOW);
    // Serial.print("Left dir:stop, ");
  }

  if (right_rpm > 0)
  {
    mcp.digitalWrite(kIoExpanderFowardRight, LOW);
    mcp.digitalWrite(kIoExpanderRverseRight, HIGH);
    // Serial.print("Right dir:FWD, ");
  }
  else if (right_rpm < 0)
  {
    mcp.digitalWrite(kIoExpanderFowardRight, HIGH);
    mcp.digitalWrite(kIoExpanderRverseRight, LOW);
    // Serial.print("Right dir:REV, ");
  }
  else
  {
    mcp.digitalWrite(kIoExpanderFowardRight, LOW);
    mcp.digitalWrite(kIoExpanderRverseRight, LOW);
    // Serial.print("Right dir:stop, ");
  }
  // formar 1:backet, 2:brade

  if (abs(left_rpm) > abs(right_rpm) ? left_rpm < 0 : right_rpm < 0)
  {
    g_left_rpm = right_rpm;
    g_right_rpm = left_rpm;
  }
  else
  {
    g_left_rpm = left_rpm;
    g_right_rpm = right_rpm;
  }
}

void stopMotor(bool motor_brake_state)
{
  // HIGH:ブレーキ解放 LOW:ブレーキ保持
  if (motor_brake_state)
  {
    mcp.digitalWrite(kIoExpanderMbFreeLeft, LOW);
    mcp.digitalWrite(kIoExpanderMbFreeRight, LOW);
  }
  else
  {
    mcp.digitalWrite(kIoExpanderMbFreeLeft, HIGH);
    mcp.digitalWrite(kIoExpanderMbFreeRight, HIGH);
  }
  analogWrite(kVMLeft, 0);
  analogWrite(kVMRight, 0);
  //=== STOP表示 ===
  // Serial.print("Motor stop, ");
}

void writeMotorPwm(int left_rpm, int right_rpm)
{
  int left_pwm;
  int right_pwm;

  left_rpm = abs(left_rpm);
  right_rpm = abs(right_rpm);

  left_rpm = min(left_rpm, 4000);
  left_rpm = max(left_rpm, 0);

  right_rpm = min(right_rpm, 4000);
  right_rpm = max(right_rpm, 0);

  left_pwm = map(left_rpm, 0, 4000, 0, 255);
  right_pwm = map(right_rpm, 0, 4000, 0, 255);

  if (left_pwm == 0 && right_pwm == 0)
  {
    stopMotor(true);
  }
  else
  {
    stopMotor(false); // ブレーキを解除
    analogWrite(kVMLeft, left_pwm);
    analogWrite(kVMRight, right_pwm);
  }
}

void receivePropoSwitch()
{
  uint16_t ch1 = pulseIn(kRcTransmitterCh1, HIGH, 25000);
  uint16_t ch2 = pulseIn(kRcTransmitterCh2, HIGH, 25000);
  uint16_t ch3 = pulseIn(kRcTransmitterCh3, HIGH, 25000);
  uint16_t ch4 = pulseIn(kRcTransmitterCh4, HIGH, 25000);
  uint16_t ch5 = pulseIn(kRcTransmitterCh5, HIGH, 25000);
  uint16_t ch6 = pulseIn(kRcTransmitterCh6, HIGH, 25000);

  if (ch1 == 0 || ch2 == 0 || ch3 == 0 || ch4 == 0)
  {
    Serial.println("RC timeout → no signal");
    g_input_propo_joy_leftX = 0;
    g_input_propo_joy_leftY = 0;
    g_input_propo_joy_rightX = 0;
    g_input_propo_joy_rightY = 0;
    g_power_relay = 0;
    return;
  }

  auto norm = [](uint16_t pwm)
  {
    float x = pwm;
    x = constrain(x, 1000, 2000);
    return (x - 1500.0) / 500.0;
  };

  const float joystick_deadzone = 0.3;
  const float brake_deadzone = 0.3;

  g_input_propo_joy_leftX = norm(ch4);
  g_input_propo_joy_leftY = norm(ch2);
  g_input_propo_joy_rightX = norm(ch1);
  g_input_propo_joy_rightY = norm(ch3);

  if (abs(g_input_propo_joy_leftX) < joystick_deadzone)
    g_input_propo_joy_leftX = 0;
  if (abs(g_input_propo_joy_leftY) < joystick_deadzone)
    g_input_propo_joy_leftY = 0;

  if (abs(g_input_propo_joy_rightX) < joystick_deadzone)
    g_input_propo_joy_rightX = 0;
  if (abs(g_input_propo_joy_rightY) < joystick_deadzone)
    g_input_propo_joy_rightY = 0;

  //g_power_relay = (ch5 < 1000 || ch5 > 2000) ? 1 : 0;
  g_power_relay = (ch5 > 1500) ? 1 : 0;
  g_control_reverse = (ch5 < 0) ? true : false; // make it false for this time; reverse mode still not confirm yet
  g_control_mode = (ch6 > 1500) ? MODE_ESPNOW : MODE_RC;
  g_motor_brake = (abs(norm(ch6)) < brake_deadzone) ? true : false;

  lastRcReceiveTime = millis(); // mark last valid RC frame

#if ENABLE_RC_DEBUG
  static unsigned long lastDebugTime = 0;
  const unsigned long debugInterval = 200;

  unsigned long now = millis();
  if (now - lastDebugTime > debugInterval)
  {
    lastDebugTime = now;

    Serial.println("===== RC DEBUG =====");
    Serial.print("ch1: ");
    Serial.print(ch1);
    Serial.print("  ch2: ");
    Serial.print(ch2);
    Serial.print("  ch3: ");
    Serial.print(ch3);
    Serial.print("  ch4: ");
    Serial.print(ch4);
    Serial.print("  ch5: ");
    Serial.print(ch5);
    Serial.print("  ch6: ");
    Serial.println(ch6);

    Serial.print("ch1: ");
    Serial.print(norm(ch1));
    Serial.print("  ch2: ");
    Serial.print(norm(ch2));
    Serial.print("  ch3: ");
    Serial.print(norm(ch3));
    Serial.print("  ch4: ");
    Serial.print(norm(ch4));
    Serial.print("  ch5: ");
    Serial.print(norm(ch5));
    Serial.print("  ch6: ");
    Serial.println(norm(ch6));

    Serial.print("  g_control_reverse : ");
    Serial.println(g_control_reverse);

    Serial.println("====================");
  }
#endif
}
