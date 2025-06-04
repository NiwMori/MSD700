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

//シリンダーモータードライバーピン割振
#define liftFWD 42
#define liftREV 43
#define dumpFWD 44
#define dumpREV 45

//動力リレー用リレーピン割振
#define switcherOut 24  // switch output

//積層信号灯リレー
#define LightRed 25

//分圧抵抗読み取り
#define BatteryPin A6
int batteryV = 0;

//レシーバーピン割振
const int ch1 = 6;   // right X
const int ch2 = 7;   // left Y
const int ch3 = 8;   // right Y
const int ch4 = 9;   // left X
const int ch5 = 10;  // switchA
const int ch6 = 11;  // switchB
const int ch7 = 12;  // switchE
const int ch8 = 13;  // Null

int gMode = 0;

//レシーバー用変数
int VR = 0;
int rightX = 0;
int rightY = 0;
int leftX, leftXa, leftXb;
int leftY = 0;
int switcher = 0;
int gap;

int SwitchA = 0;
int SwitchB = 0;
int SwitchE = 0;

//レコード関係
int recordMax = 2800;      // 最大記録数
int recordIndex = 0;       // 記録インデックス
bool isRecording = false;  // 記録中フラグ
bool isPlaying = false;    // 再生中フラグ

int recordLeftX[700];   // 左スティックX
int recordLeftY[700];   // 左スティックY
int recordRightX[700];  // dump
int recordRightY[700];  // lift

// タイマー用（再生スピード調整用）
unsigned long lastPlayTime = 0;
const unsigned long playInterval = 300;  // 50ms毎に再生（20Hz）

// タイマー追加
unsigned long lastRecordTime = 0;
const unsigned long recordInterval = 300;  // 記録間隔：100ms

#include <ArduinoJson.h>

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

void setup() {
  Serial1.begin(115200);  // For Jetson

  Serial.begin(9600);

  //モータードライバー速度設定
  pinMode(mmLPin, OUTPUT);
  pinMode(mmRPin, OUTPUT);

  //左モータードライバーアウトプット
  pinMode(mLeftFWD, OUTPUT);
  pinMode(mLeftREV, OUTPUT);
  pinMode(stopModeL, OUTPUT);
  pinMode(m0L, OUTPUT);
  pinMode(mbFreeL, OUTPUT);

  //右モータードライバーアウトプット
  pinMode(mRightFWD, OUTPUT);
  pinMode(mRightREV, OUTPUT);
  pinMode(stopModeR, OUTPUT);
  pinMode(m0R, OUTPUT);
  pinMode(mbFreeR, OUTPUT);

  //シリンダーモータードライバー
  pinMode(liftFWD, OUTPUT);
  pinMode(liftREV, OUTPUT);
  pinMode(dumpFWD, OUTPUT);
  pinMode(dumpREV, OUTPUT);

  //レシーバー
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
  pinMode(ch7, INPUT);
  pinMode(ch8, INPUT);

  //動力リレー用リレー
  pinMode(switcherOut, OUTPUT);

  //積層信号灯用リレー
  pinMode(LightRed, OUTPUT);

  //積層信号灯用リレー
  pinMode(BatteryPin, INPUT);
}

void loop() {
  // 初期設定（停止モード、外部速度指定、ブレーキ解除）
  digitalWrite(stopModeR, LOW);
  digitalWrite(stopModeL, LOW);
  digitalWrite(m0R, HIGH);
  digitalWrite(m0L, HIGH);
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mbFreeL, HIGH);
  delay(10);  // チャタリング防止

  // スイッチ状態読み取り
  SwitchA = pulseIn(ch5, HIGH, 25000);  // 動力スイッチ
  SwitchB = pulseIn(ch6, HIGH, 25000);  // 手動/自動
  SwitchE = pulseIn(ch7, HIGH, 25000);  // 記録/フリー/実行

  if (SwitchA > 1550) {  // 動力ON
    digitalWrite(switcherOut, HIGH);

    // ジョイスティック入力読み取り（直前と比較）
    leftXb = leftXa;
    rightX = pulseIn(ch1, HIGH);
    leftX = pulseIn(ch4, HIGH);
    rightY = pulseIn(ch3, HIGH);
    leftY = pulseIn(ch2, HIGH);
    leftXa = leftX;

    // 入力範囲外補正
    gap = abs(leftXb - leftXa);
    if (leftX < 1000 || leftX > 2000) {
      leftX = leftXb;
      leftXa = leftXb;
    }

    if (SwitchE < 1200) {  // 記録モード
      if (!isRecording) {
        Serial.println("Recording Start");
        isRecording = true;
        isPlaying = false;
        recordIndex = 0;
      }

      // 一定間隔で記録
      unsigned long now = millis();
      if (recordIndex < recordMax && now - lastRecordTime >= recordInterval) {
        recordLeftX[recordIndex] = leftX;
        recordLeftY[recordIndex] = leftY;
        recordRightX[recordIndex] = rightX;
        recordRightY[recordIndex] = rightY;
        recordIndex++;
        lastRecordTime = now;
      }

      // 操作反映
      motor(leftX, leftY);
      lift(rightY);
      dump(rightX);

    } else if (SwitchE > 1400 && SwitchE < 1800) {  // フリーモード
      if (isRecording) {
        Serial.println("Recording Stop");
        isRecording = false;
      }
      isPlaying = false;

      lift(rightY);
      dump(rightX);

      // モーター手動制御
      int leftX = pulseIn(ch4, HIGH, 25000);
      int leftY = pulseIn(ch2, HIGH, 25000);
      motor(leftX, leftY);

    } else if (SwitchE > 1900) {  // 実行モード
      if (isRecording) {
        Serial.println("Recording Stop");
        isRecording = false;
      }
      isPlaying = true;
      playRecord();  // 記録再生
    }

  } else {
    // 動力OFF時
    digitalWrite(switcherOut, LOW);
    stopMachine();  // 全停止
  }
}

// 再生関数
void playRecord() {
  unsigned long now = millis();
  if (now - lastPlayTime < playInterval) return;  // 再生間隔調整

  lastPlayTime = now;
  if (recordIndex == 0) {
    stopMachine();
    return;
  }

  static int playIndex = 0;
  motor(recordLeftX[playIndex], recordLeftY[playIndex]);

  // lift/dump再生
  lift(recordRightY[playIndex]);
  dump(recordRightX[playIndex]);

  playIndex++;

  if (playIndex >= recordIndex) {
    playIndex = 0;  // ループ再生
  }
}

// モーター停止関数
void stopMachine() {
  //モーター速度0
  analogWrite(mmLPin, 0);
  analogWrite(mmRPin, 0);

  //前進後進指示なし
  digitalWrite(liftFWD, LOW);
  digitalWrite(liftREV, LOW);
  digitalWrite(dumpFWD, LOW);
  digitalWrite(dumpREV, LOW);

  //HIGH:ブレーキ解放 LOW:ブレーキ保持
  digitalWrite(mbFreeL, LOW);
  digitalWrite(mbFreeR, LOW);

  // STOP表示
  Serial.println(" Stop ");
}

// モーター関数
void motor(int LeftX, int LeftY) {
  // プロポ閾値
  int thresholdHigh = 1570;
  int thresholdMid = 1510;
  int thresholdLow = 1454;

  // 移動方向フラグ
  int Right = 0;
  int Left = 0;
  int Forward = 0;
  int Reverse = 0;

  // モーター回転方向フラグ
  bool dirR = false;
  bool dirL = false;
  bool canMove = false;

  // 移動方向決定
  if (LeftX >= thresholdHigh) {
    Right = 1;
  } else if (LeftX <= thresholdLow) {
    Left = 1;
  }
  if (LeftY <= thresholdLow) {
    Forward = 1;
  } else if (LeftY >= thresholdHigh) {
    Reverse = 1;
  }

  // 前後左右フラグ反転
  if (gMode == 1) {
    int temp = Forward;
    Forward = Reverse;
    Reverse = temp;
    temp = Right;
    Right = Left;
    Left = temp;
  }

  // モーターの回転方向を反転
  if (gMode == 0) {
    // gModeが1の場合、モーターの回転方向を反転
    dirR = !dirR;
    dirL = !dirL;
  }

  // モーター動作決定
  int diffX = abs(LeftX - thresholdMid);
  int diffY = abs(LeftY - thresholdMid);
  Serial.print("diffX");
  Serial.println(diffX);
  Serial.print("diffY");
  Serial.println(diffY);
  int speedX = map(diffX, 70, 415, 0, 255);
  int speedY = map(diffY, 70, 415, 0, 255);

  if (Forward == 1) {
    Serial.println("Forward");
    canMove = true;
    dirR = false;
    dirL = true;
    if (Right == 1) {
      // 右前進
      Serial.println("2222");
      analogWrite(mmLPin, speedY);
      analogWrite(mmRPin, 0);
    } else if (Left == 1) {
      // 左前進
      Serial.println("8888");
      analogWrite(mmLPin, 0);
      analogWrite(mmRPin, speedY);
    } else {
      // 前進
      Serial.println("1111");
      analogWrite(mmLPin, speedY);
      analogWrite(mmRPin, speedY);
    }
  } else if (Reverse == 1) {
    Serial.println("Reverse");
    canMove = true;
    dirR = true;
    dirL = false;
    if (Right == 1) {
      // 右後進
      Serial.println("4444");
      analogWrite(mmLPin, speedY);
      analogWrite(mmRPin, 0);
    } else if (Left == 1) {
      // 左後進
      Serial.println("6666");
      analogWrite(mmLPin, 0);
      analogWrite(mmRPin, speedY);
    } else {
      // 後進
      Serial.println("5555");
      analogWrite(mmLPin, speedY);
      analogWrite(mmRPin, speedY);
    }
  } else {
    if (Right == 1) {
      // 右旋回
      Serial.println("3333");
      canMove = true;
      if (gMode == 1) {
        dirR = false;
        dirL = false;
      } else {
        dirR = true;
        dirL = true;
      }
      analogWrite(mmLPin, speedX/2);
      analogWrite(mmRPin, speedX/2);
    } else if (Left == 1) {
      // 左旋回
      Serial.println("7777");
      canMove = true;
      if (gMode == 1) {
        dirR = true;
        dirL = true;
      } else {
        dirR = false;
        dirL = false;
      }
      analogWrite(mmLPin, speedX/2);
      analogWrite(mmRPin, speedX/2);
    } else {
      // 停止
      canMove = false;
      analogWrite(mmLPin, 0);
      analogWrite(mmRPin, 0);
    }
  }
  digitalWrite(mRightFWD, dirR & canMove);
  digitalWrite(mRightREV, (!dirR) & canMove);
  digitalWrite(mLeftFWD, dirL & canMove);
  digitalWrite(mLeftREV, (!dirL) & canMove);
}

//アームリフト関数
void lift(int pulse) {
  if (pulse < 1250 && pulse > 950) {
    digitalWrite(liftFWD, HIGH);
    digitalWrite(liftREV, LOW);
  } else if (pulse < 2070 && pulse > 1750) {
    digitalWrite(liftFWD, LOW);
    digitalWrite(liftREV, HIGH);
  } else if (pulse < 1750 && pulse > 1250) {
    digitalWrite(liftFWD, LOW);
    digitalWrite(liftREV, LOW);
  }
}

//アームダンプ関数
void dump(int pulse) {
  if (pulse < 1250 && pulse > 950) {
    digitalWrite(dumpFWD, HIGH);
    digitalWrite(dumpREV, LOW);
  } else if (pulse < 2070 && pulse > 1750) {
    digitalWrite(dumpFWD, LOW);
    digitalWrite(dumpREV, HIGH);
  } else if (pulse < 1750 && pulse > 1250) {
    digitalWrite(dumpFWD, LOW);
    digitalWrite(dumpREV, LOW);
  }
}

// void receiveSerialFromJetson() {
//   JsonDocument doc;
//   if (Serial1.available() > 0) {
//     DeserializationError err1;
//     while (!(err1.code() == DeserializationError::Ok)) {
//       String recived_serial_data = Serial1.readStringUntil('\n');
//       err1 = deserializeJson(doc, recived_serial_data);
//     }
//     float linear_x = doc["linear_x"];
//     float angular_z = doc["angular_z"];

//     // linear_x: ±0.46 → 1000-2000
//     leftY = map(linear_x * 100, -46, 46, 1450, 1000);

//     // angular_z: ±0.78 → 1000-2000
//     leftX = map(angular_z * 100, -78, 78, 1450, 1000);
//     // g_cylinder_1 = doc["cylinder_1"];
//     // g_cylinder_2 = doc["cylinder_2"];
//     // g_power_relay = doc["power_relay"];
//   }
// }
