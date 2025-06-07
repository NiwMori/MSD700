#include <Arduino.h>
#include <ArduinoJson.h>

//左モータードライバーピン割振
#define mLeftFWD 30
#define mLeftREV 31
#define stopModeL 32
#define m0L 33
#define mbFreeL 34
#define mmLPin 2  // left motor speed PWM

//右モータードライバーピン割振
#define mRightFWD 35
#define mRightREV 36
#define stopModeR 37
#define m0R 38
#define mbFreeR 39
#define mmRPin 3  // Right motor speed PWM

// 動力リレー用リレー
#define powerRelay 24

//積層信号灯リレー
#define LightRed 25

//分圧抵抗読み取り
#define BatteryPin A6
int batteryV = 0;

// プロポ
const int ch1 = 6;   // right X
const int ch2 = 7;   // left Y
const int ch3 = 8;   // right Y
const int ch4 = 9;   // left X
const int ch5 = 10;  // switchA
const int ch6 = 11;  // switchB
const int ch7 = 12;  // Null
const int ch8 = 13;  // Null

// int readVoltageRaw() {
//   long sum = 0;
//   for (int i = 0; i < 10; i++) {
//     sum += analogRead(BatteryPin);
//     delay(5);  // 少し待つと安定する
//   }
//   return sum / 10;
// }
// bool allowOperation = false;  // 動作許可フラグ（初期は停止）

// // グローバル変数（setupの上に記述）
// unsigned long previousMillis = 0;
// bool lightState = false;
// float volt = 0;
// int value = 0;


// calculation velocity to rpm
#define M_PI 3.14159265358979323846
float LINEAR_SPEED_MAX = 0.462861317;
float ANGULAR_SPEED_MAX = 0.74;  //64398163;

// control variable
float g_linear_x = 0.0;
float g_angular_z = 0.0;
// calculate variable
float g_left_rpm;
float g_right_rpm;

float g_input_propo_joy_leftY;
float g_input_propo_joy_leftX;
// propo switch variable
int g_power_relay = 0;       // A : 0:on 1:off
int g_mode_switch = 0;       //  0: フタが後ろ 1: フタが前
int g_select_controler = 1;  // C : 0:jetson 1:propo
int g_motor_brake = 1;       // none : 0:neutral 1:brake

void receivePropoSwitch() {
  int togle_A = pulseIn(ch5, HIGH);  // down 2056/2057 (1785) middle 1507/1513 (1238.5) up 963/964
  int togle_B = pulseIn(ch6, HIGH);  // down x2056/2057 (1510.5) up 963/x964

  if (togle_B < 1510) {
    //up
    g_select_controler = 0;
  } else if (togle_B > 1510) {
    //down
    g_select_controler = 1;
  }
  if (togle_A < 1510) {
    //up
    g_power_relay = 0;
  } else if (togle_A > 1510) {
    //down
    g_power_relay = 1;
  }
}

void receivePropoJoy(float deadzone) {  // input proportional controler joysticks
  int PROPO_JOY_MAX = 1918;
  int PROPO_JOY_MIN = 1104;
  float input_propo_joy_leftY = map(pulseIn(ch2, HIGH), PROPO_JOY_MAX, PROPO_JOY_MIN, -100, 100) / 100.0;  // left joy Y
  float input_propo_joy_leftX = map(pulseIn(ch4, HIGH), PROPO_JOY_MAX, PROPO_JOY_MIN, -100, 100) / 100.0;  // left joy X

  // dead zone
  if (-deadzone <= input_propo_joy_leftX && input_propo_joy_leftX <= deadzone) {
    input_propo_joy_leftX = 0.00;
  }
  if (-deadzone <= input_propo_joy_leftY && input_propo_joy_leftY <= deadzone) {
    input_propo_joy_leftY = 0.00;
  }

  if (g_mode_switch == 0) {  // フタが後ろ
    g_input_propo_joy_leftY = input_propo_joy_leftY;
  } else if (g_mode_switch == 1) {  // フタが前
    g_input_propo_joy_leftY = -input_propo_joy_leftY;
  }
  g_input_propo_joy_leftX = input_propo_joy_leftX;

  // Serial.print(" g_input_propo_joy_leftY ");
  // Serial.print(g_input_propo_joy_leftY);
  // Serial.print(" g_input_propo_joy_leftX ");
  // Serial.println(g_input_propo_joy_leftX);
}

// void receiveSerialFromJetson() {
//   JsonDocument doc;
//   String recived_serial_data = Serial.readStringUntil('\n');
//   // Serial.print(recived_serial_data);
//   deserializeJson(doc, recived_serial_data);
//   g_linear_x = doc["linear_x"];
//   g_angular_z = doc["angular_z"];
// }

float velToRpm(float v) {
  // convert velocity to rpm
  const float REDUCTION_RATIO = 100.0;  // 減速比 reduction ratio
  const float WHEEL_RADIUS = 0.1105;    // 車輪半径 radius of gyration [m]
  float n = 30 / M_PI * REDUCTION_RATIO / WHEEL_RADIUS * v;

  return n;
}

void ik(float v, float w) {
  // calculate linear velocity and angur velocity to left and right velocity
  float WHEEL_DISTANCE = 0.600;  // 車輪間距離 distance between the wheels [m]
  float v_l = v - WHEEL_DISTANCE * w;
  float v_r = v + WHEEL_DISTANCE * w;

  float left_rpm = velToRpm(v_l);
  float right_rpm = velToRpm(v_r);

  // direction of motor
  if (left_rpm > 0) {
    digitalWrite(mLeftFWD, HIGH);
    digitalWrite(mLeftREV, LOW);
    // Serial.print("Left dir:FWD, ");
  } else if (left_rpm < 0) {
    digitalWrite(mLeftFWD, LOW);
    digitalWrite(mLeftREV, HIGH);
    // Serial.print("Left dir:REV, ");
  } else {
    digitalWrite(mLeftFWD, LOW);
    digitalWrite(mLeftREV, LOW);
    // Serial.print("Left dir:stop, ");
  }

  if (right_rpm > 0) {
    digitalWrite(mRightFWD, LOW);
    digitalWrite(mRightREV, HIGH);
    // Serial.print("Right dir:FWD, ");
  } else if (right_rpm < 0) {
    digitalWrite(mRightFWD, HIGH);
    digitalWrite(mRightREV, LOW);
    // Serial.print("Right dir:REV, ");
  } else {
    digitalWrite(mRightFWD, LOW);
    digitalWrite(mRightREV, LOW);
    // Serial.print("Right dir:stop, ");
  }

  //  0: フタが後ろ 1: フタが前
  if (g_mode_switch == 0) {  // フタが後ろ
    if (left_rpm < 0 || right_rpm < 0) {
      g_left_rpm = right_rpm;
      g_right_rpm = left_rpm;
    } else {
      g_left_rpm = left_rpm;
      g_right_rpm = right_rpm;
    }
  } else if (g_mode_switch == 1) {  // フタが前
    if (left_rpm < 0 || right_rpm < 0) {
      g_left_rpm = left_rpm;
      g_right_rpm = right_rpm;
    } else {
      g_left_rpm = right_rpm;
      g_right_rpm = left_rpm;
    }
  }
}

void stopMotor(bool motor_brake_state) {
  // HIGH:ブレーキ解放 LOW:ブレーキ保持
  if (motor_brake_state == 0) {
    digitalWrite(mbFreeL, HIGH);
    digitalWrite(mbFreeR, HIGH);
  } else if (motor_brake_state == 1) {
    digitalWrite(mbFreeL, LOW);
    digitalWrite(mbFreeR, LOW);
  }
  analogWrite(mmLPin, 0);
  analogWrite(mmRPin, 0);
  //=== STOP表示 ===
  // Serial.print("Motor stop, ");
}

void writeMotorPwm(int left_rpm, int right_rpm) {
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

  if (left_pwm && right_pwm == 0) {
    stopMotor(g_motor_brake);
  } else {
    analogWrite(mmLPin, left_pwm);
    analogWrite(mmRPin, right_pwm);
    // Serial.print("left_pwm: ");
    // Serial.print(left_pwm);
    // Serial.print(" , ");
    // Serial.print("right_pwm: ");
    // Serial.println(right_pwm);
  }
}

void setup() {
  Serial.begin(115200);

  //モータードライバー速度設定
  pinMode(mmLPin, OUTPUT);
  pinMode(mmRPin, OUTPUT);

  //左モータードライバーアウトプット
  pinMode(mLeftFWD, OUTPUT);
  pinMode(mLeftREV, OUTPUT);
  pinMode(stopModeL, OUTPUT);
  pinMode(m0L, OUTPUT);
  // pinMode(alarmResetL, OUTPUT);
  pinMode(mbFreeL, OUTPUT);
  //左モータードライバーインプット
  // pinMode(alarmOutLN, INPUT);
  // pinMode(wngLN, INPUT);
  // pinMode(speedOutL, INPUT);

  //右モータードライバーアウトプット
  pinMode(mRightFWD, OUTPUT);
  pinMode(mRightREV, OUTPUT);
  pinMode(stopModeR, OUTPUT);
  pinMode(m0R, OUTPUT);
  // pinMode(alarmResetR, OUTPUT);
  pinMode(mbFreeR, OUTPUT);
  //右モータードライバーインプット
  // pinMode(alarmOutRN, INPUT);
  // pinMode(wngRN, INPUT);
  // pinMode(speedOutR, INPUT);

  //動力リレー用リレー
  pinMode(powerRelay, OUTPUT);

  // HIGH:時間設定 LOW:瞬時停止
  digitalWrite(stopModeR, LOW);
  digitalWrite(stopModeL, LOW);

  // HIGH:速度指定を外部入力 LOW:内部速度設定器
  digitalWrite(m0R, HIGH);
  digitalWrite(m0L, HIGH);

  //HIGH:ブレーキ解放 LOW:ブレーキ保持
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mbFreeL, HIGH);

  //レシーバー
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
  pinMode(ch7, INPUT);
  pinMode(ch8, INPUT);

  //積層信号灯用リレー
  pinMode(LightRed, OUTPUT);

  //積層信号灯用リレー
  pinMode(BatteryPin, INPUT);
}

void loop() {
  receivePropoSwitch();
  receivePropoJoy(0.15);
  g_linear_x = g_input_propo_joy_leftY * LINEAR_SPEED_MAX;
  g_angular_z = g_input_propo_joy_leftX * ANGULAR_SPEED_MAX;

  // switch (g_select_controler) {
  //   case 0:
  //     receivePropoJoy(0.30);
  //     g_linear_x   = g_input_propo_joy_leftY * LINEAR_SPEED_MAX;
  //     g_angular_z  = g_input_propo_joy_leftX * ANGULAR_SPEED_MAX;
  //     break;
  //   case 1:
  //     receiveSerialFromJetson();
  //     break;
  // }

  switch (g_power_relay) {
    case 0:
      digitalWrite(powerRelay, LOW);
      stopMotor(g_motor_brake);
      break;
    case 1:
      digitalWrite(powerRelay, HIGH);
      ik(g_linear_x, g_angular_z);
      writeMotorPwm(g_left_rpm, g_right_rpm);
      // Serial.print(" g_linear_x ");
      // Serial.print(g_linear_x);
      // Serial.print(" g_angular_z ");
      // Serial.print(g_angular_z);
      // Serial.print(" g_left_rpm ");
      // Serial.print(g_left_rpm);
      // Serial.print(" g_right_rpm ");
      // Serial.println(g_right_rpm);
      break;
  }
}