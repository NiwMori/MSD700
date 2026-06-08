#include <WiFi.h>
#include <esp_now.h>

#include "msd.hpp"

#define ENABLE_RC_DEBUG 1

float g_input_propo_joy_leftY;
float g_input_propo_joy_leftX;
float g_input_propo_joy_rightY;
float g_input_propo_joy_rightX;

double g_linear_x = 0;
double g_angular_z = 0;
int g_power_relay = 0;
int g_left_rpm = 0;
int g_right_rpm = 0;
int g_select_controler = 0;
bool g_motor_brake = true;
bool g_control_reverse = false;

enum ControlMode { MODE_RC = 0, MODE_ESPNOW = 1 };
ControlMode g_control_mode = MODE_RC;

// Command sent to sub-board when in RC mode.
// NOTE: keep this struct in sync with SubBoardCmd in sub-board firmware.
typedef struct {
  float cylinder_1;  // [-1.0, 1.0], positive = forward
  float cylinder_2;  // [-1.0, 1.0], positive = forward
  int power;         // 1 = on, 0 = off
} SubBoardCmd;

uint8_t g_sub_board_addr[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  // broadcast

unsigned long lastReceiveTime   = 0;  // last ESP-NOW receive time
unsigned long lastRcReceiveTime = 0;  // ch5/ch6 valid — RC transmitter alive
unsigned long lastJoystickTime  = 0;  // ch1-ch4 all valid — joystick channels alive

typedef struct struct_message {
  char message[32];
} struct_message;

struct_message recvData;

void OnDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
  float deadzone = 0.3;

  memcpy(&recvData, incomingData, sizeof(recvData));

  int parsedData[7];
  int emergencyStopSwitch = 0;
  int leftJoyStickX = 0;
  int leftJoyStickY = 0;

  String message = String(recvData.message);
  int lastCommaIndex = 0;
  bool invalidData = false;

  for (int i = 0; i < 7; i++) {
    lastCommaIndex = message.indexOf(',');
    if (i <= 5 && lastCommaIndex == -1) {
      invalidData = true;
      return;
    }
    parsedData[i] = message.substring(0, lastCommaIndex).toInt();
    message = message.substring(lastCommaIndex + 1);
    invalidData = false;
    lastReceiveTime = millis();
  }

  emergencyStopSwitch = parsedData[0];
  leftJoyStickX = parsedData[4];
  leftJoyStickY = parsedData[5];

  float input_propo_joy_leftY = map(leftJoyStickY, 0, 1023, -100, 100) / 100.0;
  float input_propo_joy_leftX = map(leftJoyStickX, 0, 1023, -100, 100) / 100.0 * (-1.0);

  if (-deadzone <= input_propo_joy_leftX && input_propo_joy_leftX <= deadzone)
    input_propo_joy_leftX = 0.00;
  if (-deadzone <= input_propo_joy_leftY && input_propo_joy_leftY <= deadzone)
    input_propo_joy_leftY = 0.00;

  // Workaround: always keep power on (emergency stop from white controller not yet implemented)
  g_power_relay = 1;
  g_select_controler = 0;
  g_control_mode = MODE_ESPNOW;
  g_input_propo_joy_leftY = input_propo_joy_leftY;
  g_input_propo_joy_leftX = input_propo_joy_leftX;
}

void checkEspNowTimeOut() {
  if (g_control_mode == MODE_ESPNOW && millis() - lastReceiveTime > 100) {
    g_input_propo_joy_leftY = 0;
    g_input_propo_joy_leftX = 0;
    g_control_mode = MODE_RC;
  }
}

void checkRcTimeOut() {
  if (g_control_mode == MODE_ESPNOW) return;
  if (millis() - lastRcReceiveTime > 500) {
    // RC transmitter fully lost — zero everything including relay
    g_input_propo_joy_leftX = 0;
    g_input_propo_joy_leftY = 0;
    g_input_propo_joy_rightX = 0;
    g_input_propo_joy_rightY = 0;
    g_power_relay = 0;
  } else if (millis() - lastJoystickTime > 500) {
    // ch1-ch4 dropout only (ch5/ch6 still alive) — zero velocity, keep relay
    g_input_propo_joy_leftX = 0;
    g_input_propo_joy_leftY = 0;
    g_input_propo_joy_rightX = 0;
    g_input_propo_joy_rightY = 0;
  }
}

void IoExpanderInitialize() {
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

void GpioModeSet() {
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(115200, SERIAL_8N1, kUart1Rx, kUart1Tx);
  Serial1.setTimeout(100);
  GpioModeSet();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, g_sub_board_addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  while (gpio_pullup_en(kSDA) != ESP_OK) continue;
  while (gpio_pullup_en(kSCL) != ESP_OK) continue;
  Wire.setPins(kSDA, kSCL);
  Wire.begin();
  bno.begin();
  bno.setExtCrystalUse(true);

  IoExpanderInitialize();
}

void loop() {
  int msd_state;
  receivePropoSwitch();

  checkEspNowTimeOut();
  checkRcTimeOut();

  if (g_control_mode == MODE_RC) {
    SubBoardCmd sub_cmd;
    sub_cmd.cylinder_1 = g_input_propo_joy_rightY;  // CH3: right stick Y
    sub_cmd.cylinder_2 = g_input_propo_joy_rightX;  // CH1: right stick X
    sub_cmd.power = g_power_relay;
    esp_now_send(g_sub_board_addr, (uint8_t *)&sub_cmd, sizeof(sub_cmd));
  }

  static int effective_relay = 0;
  static int prev_effective  = 0;

  effective_relay = g_power_relay;

  // remapDeadzone declared outside if/else — lambda inside switch-case is C++ UB
  auto remapDeadzone = [](float v, float dz) -> float {
    if (v == 0.0f) return 0.0f;
    float sign = (v > 0.0f) ? 1.0f : -1.0f;
    return sign * (abs(v) - dz) / (1.0f - dz);
  };

  if (effective_relay == 0) {
    digitalWrite(kMoterPowerRelay, LOW);
    analogWrite(kVMLeft, 0);
    analogWrite(kVMRight, 0);
    msd_state = 2;
  } else {
    if (prev_effective == 0) motorDriverReinit();
    digitalWrite(kMoterPowerRelay, HIGH);

    if (g_control_mode == MODE_RC) {
      float joy_x = remapDeadzone(g_input_propo_joy_leftX, 0.3f);
      float joy_y = remapDeadzone(g_input_propo_joy_leftY, 0.3f);
      if (!g_control_reverse) joy_y = -joy_y;
      float angular_sign = -1.0f;
      g_linear_x = (double)(joy_y * LINEAR_SPEED_MAX);
      g_angular_z = (double)(joy_x * angular_sign * ANGULAR_SPEED_MAX);
    } else {
      g_linear_x = remapDeadzone(g_input_propo_joy_leftY, 0.3f) * LINEAR_SPEED_MAX;
      g_angular_z = remapDeadzone(g_input_propo_joy_leftX, 0.3f) * ANGULAR_SPEED_MAX;
    }

    pixels.setPixelColor(0, pixels.Color(BRIGHTNESS, 0, BRIGHTNESS));
    pixels.show();
    msd_state = 1;

    ik(g_linear_x, g_angular_z);
    writeMotorPwm(g_left_rpm, g_right_rpm);
  }

  prev_effective = effective_relay;
}

float velToRpm(float v) {
  const float REDUCTION_RATIO = 100.0;  // reduction ratio
  const float WHEEL_RADIUS    = 0.1105; // wheel radius [m]
  return 30 / M_PI * REDUCTION_RATIO / WHEEL_RADIUS * v;
}

void ik(float v, float w) {
  float WHEEL_DISTANCE = 0.600;  // distance between wheels [m]
  float v_l = v - WHEEL_DISTANCE * w;
  float v_r = v + WHEEL_DISTANCE * w;

  float left_rpm  = velToRpm(v_l);
  float right_rpm = velToRpm(v_r);

  if (left_rpm > 0) {
    mcp.digitalWrite(kIoExpanderFowardLeft, HIGH);
    mcp.digitalWrite(kIoExpanderRverseLeft, LOW);
  } else if (left_rpm < 0) {
    mcp.digitalWrite(kIoExpanderFowardLeft, LOW);
    mcp.digitalWrite(kIoExpanderRverseLeft, HIGH);
  } else {
    mcp.digitalWrite(kIoExpanderFowardLeft, LOW);
    mcp.digitalWrite(kIoExpanderRverseLeft, LOW);
  }

  if (right_rpm > 0) {
    mcp.digitalWrite(kIoExpanderFowardRight, LOW);
    mcp.digitalWrite(kIoExpanderRverseRight, HIGH);
  } else if (right_rpm < 0) {
    mcp.digitalWrite(kIoExpanderFowardRight, HIGH);
    mcp.digitalWrite(kIoExpanderRverseRight, LOW);
  } else {
    mcp.digitalWrite(kIoExpanderFowardRight, LOW);
    mcp.digitalWrite(kIoExpanderRverseRight, LOW);
  }

  g_left_rpm  = left_rpm;
  g_right_rpm = right_rpm;
}

// Re-assert all IO expander outputs to a safe state after relay turns ON.
// Relay switching transients can corrupt MCP23017 state or trigger motor driver alarm.
void motorDriverReinit() {
  mcp.digitalWrite(kIoExpanderFowardLeft,    LOW);
  mcp.digitalWrite(kIoExpanderRverseLeft,    LOW);
  mcp.digitalWrite(kIoExpanderFowardRight,   LOW);
  mcp.digitalWrite(kIoExpanderRverseRight,   LOW);
  mcp.digitalWrite(kIoExpanderStopModeLeft,  LOW);
  mcp.digitalWrite(kIoExpanderStopModeRight, LOW);
  mcp.digitalWrite(kIoExpanderM0Left,        HIGH);
  mcp.digitalWrite(kIoExpanderM0Right,       HIGH);
  mcp.digitalWrite(kIoExpanderMbFreeLeft,    HIGH);
  mcp.digitalWrite(kIoExpanderMbFreeRight,   HIGH);
  // Pulse AlarmReset to clear fault latch on motor driver
  mcp.digitalWrite(kIoExpanderAlarmResetLeft,  HIGH);
  mcp.digitalWrite(kIoExpanderAlarmResetRight, HIGH);
  delay(50);
  mcp.digitalWrite(kIoExpanderAlarmResetLeft,  LOW);
  mcp.digitalWrite(kIoExpanderAlarmResetRight, LOW);
  delay(50);
}

void stopMotor(bool motor_brake_state) {
  // HIGH = brake released, LOW = brake held
  if (motor_brake_state) {
    mcp.digitalWrite(kIoExpanderMbFreeLeft,  LOW);
    mcp.digitalWrite(kIoExpanderMbFreeRight, LOW);
    analogWrite(kVMLeft,  0);
    analogWrite(kVMRight, 0);
  } else {
    mcp.digitalWrite(kIoExpanderMbFreeLeft,  HIGH);
    mcp.digitalWrite(kIoExpanderMbFreeRight, HIGH);
  }
}

void writeMotorPwm(int left_rpm, int right_rpm) {
  left_rpm  = constrain(abs(left_rpm),  0, 4000);
  right_rpm = constrain(abs(right_rpm), 0, 4000);

  int left_pwm  = map(left_rpm,  0, 4000, 0, 255);
  int right_pwm = map(right_rpm, 0, 4000, 0, 255);

  // Never set MbFree=LOW while driver is powered — asserting brake while motor
  // is spinning causes regenerative/short-circuit current that triggers driver alarm.
  // MbFree=LOW is only safe in relay-OFF state when the driver has no power.
  mcp.digitalWrite(kIoExpanderMbFreeLeft,  HIGH);
  mcp.digitalWrite(kIoExpanderMbFreeRight, HIGH);
  analogWrite(kVMLeft,  left_pwm);
  analogWrite(kVMRight, right_pwm);
}

void receivePropoSwitch() {
  uint16_t ch1 = pulseIn(kRcTransmitterCh1, HIGH, 25000);
  uint16_t ch2 = pulseIn(kRcTransmitterCh2, HIGH, 25000);
  uint16_t ch3 = pulseIn(kRcTransmitterCh3, HIGH, 25000);
  uint16_t ch4 = pulseIn(kRcTransmitterCh4, HIGH, 25000);
  uint16_t ch5 = pulseIn(kRcTransmitterCh5, HIGH, 25000);
  uint16_t ch6 = pulseIn(kRcTransmitterCh6, HIGH, 25000);

  // CH5 and CH6 always applied regardless of joystick channel state
  if (ch5 != 0) g_power_relay  = (ch5 > 1500) ? 1 : 0;
  if (ch6 != 0) g_control_mode = (ch6 > 1500) ? MODE_ESPNOW : MODE_RC;
  // CH5/CH6 valid = RC transmitter alive — prevent checkRcTimeOut() from zeroing relay
  if (ch5 != 0 || ch6 != 0) lastRcReceiveTime = millis();

  // In ESP-NOW mode, don't override joystick values from RC — only relay & mode are updated
  if (g_control_mode == MODE_ESPNOW) return;

  if (ch1 == 0 || ch2 == 0 || ch3 == 0 || ch4 == 0) {
    Serial.println("RC timeout: no signal on ch1-ch4");
    // Joystick values not zeroed here — checkRcTimeOut() handles that after 500ms
    // to prevent brief pulseIn timing misses from immediately stopping the motor
    return;
  }

  auto norm = [](uint16_t pwm) -> float {
    float x = constrain((float)pwm, 1000.0f, 2000.0f);
    return (x - 1500.0f) / 500.0f;
  };

  const float joystick_deadzone = 0.3;
  const float brake_deadzone    = 0.3;

  g_input_propo_joy_leftX  = norm(ch4);
  g_input_propo_joy_leftY  = norm(ch2);
  g_input_propo_joy_rightX = norm(ch1);
  g_input_propo_joy_rightY = norm(ch3);

  if (abs(g_input_propo_joy_leftX)  < joystick_deadzone) g_input_propo_joy_leftX  = 0;
  if (abs(g_input_propo_joy_leftY)  < joystick_deadzone) g_input_propo_joy_leftY  = 0;
  if (abs(g_input_propo_joy_rightX) < joystick_deadzone) g_input_propo_joy_rightX = 0;
  if (abs(g_input_propo_joy_rightY) < joystick_deadzone) g_input_propo_joy_rightY = 0;

  g_control_reverse = false;  // ch5 is uint16 — always >= 0, so (ch5 < 0) is never true
  g_motor_brake     = (abs(norm(ch6)) < brake_deadzone);

  lastJoystickTime = millis();

#if ENABLE_RC_DEBUG
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 200) {
    lastDebugTime = millis();
    Serial.println("===== RC DEBUG =====");
    Serial.print("ch1: "); Serial.print(ch1);
    Serial.print("  ch2: "); Serial.print(ch2);
    Serial.print("  ch3: "); Serial.print(ch3);
    Serial.print("  ch4: "); Serial.print(ch4);
    Serial.print("  ch5: "); Serial.print(ch5);
    Serial.print("  ch6: "); Serial.println(ch6);
    Serial.print("ch1: "); Serial.print(norm(ch1));
    Serial.print("  ch2: "); Serial.print(norm(ch2));
    Serial.print("  ch3: "); Serial.print(norm(ch3));
    Serial.print("  ch4: "); Serial.print(norm(ch4));
    Serial.print("  ch5: "); Serial.print(norm(ch5));
    Serial.print("  ch6: "); Serial.println(norm(ch6));
    Serial.println("====================");
  }
#endif
}
