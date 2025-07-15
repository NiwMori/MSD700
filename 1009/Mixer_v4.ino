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

//インバータ制御用ピン
#define inverterOut 47  // ch6 スイッチ制御出力ピン

//積層信号灯Red
#define LightRed 25

//積層信号灯Yellow
#define LightYellow 27

//ロードセル
#include <HX711_asukiaaa.h>
int pinsDout[] = { 22, 41 };
const int numPins = sizeof(pinsDout) / sizeof(pinsDout[0]);
int pinSlk = 23;
HX711_asukiaaa::Reader reader(pinsDout, numPins, pinSlk);
#define LOAD_CELL_RATED_VOLT 0.0019f
#define LOAD_CELL_RATED_GRAM 500000.0f
#define HX711_R1 20000.0
#define HX711_R2 8200.0
HX711_asukiaaa::Parser parser(LOAD_CELL_RATED_VOLT, LOAD_CELL_RATED_GRAM,
                              HX711_R1, HX711_R2);
float offsetGrams[numPins];

//Display
#include <TM1637Display.h>
#define CLK 53  // TM1637のクロック
#define DIO 51  // TM1637のデータ
TM1637Display display(CLK, DIO);

//ResetSwitch
#define ResetSwitch 40

//LED
#define LedRed A2
#define LedGreen A3
#define LedBlue A4
#define LedYellow A5

//レシーバーピン割振
const int ch1 = 6;   // right X
const int ch2 = 7;   // left Y
const int ch3 = 8;   // right Y
const int ch4 = 9;   // left X
const int ch5 = 10;  // switchA
const int ch6 = 11;  // switchB
const int ch7 = 12;  // switchE
const int ch8 = 13;  // Null

//レシーバー用変数
int VR = 0;
int rightX = 0;
int rightY = 0;
int leftX, leftXa, leftXb;
int leftY = 0;
int switcher = 0;
int gap;
int inverterswitch = 0;

int SwitchA = 0;
int SwitchB = 0;
int SwitchE = 0;

//レコード関係
int recordMax = 1000;      // 最大記録数
int recordIndex = 0;       // 記録インデックス
bool isRecording = false;  // 記録中フラグ
bool isPlaying = false;    // 再生中フラグ

int recordLeftX[500];   // 左スティックX
int recordLeftY[500];   // 左スティックY
int recordRightX[500];  // dump
int recordRightY[500];  // lift

// タイマー用（再生スピード調整用）
unsigned long lastPlayTime = 0;
const unsigned long playInterval = 300;  // 50ms毎に再生（20Hz）

// タイマー追加
unsigned long lastRecordTime = 0;
const unsigned long recordInterval = 300;  // 記録間隔：100ms

unsigned long lastDisplayUpdate = 0;
float currentWeightKg = 0;
float displayWeightKg = 0;

unsigned long lastBlinkTime = 0;
bool yellowLedState = false;

float weightKg = 0.0;

void setup() {

  Serial.begin(9600);

  //モータードライバー速度設定
  pinMode(mmLPin, OUTPUT);
  pinMode(mmRPin, OUTPUT);

  //右モータードライバーアウトプット
  pinMode(mRightFWD, OUTPUT);
  pinMode(mRightREV, OUTPUT);
  pinMode(stopModeR, OUTPUT);
  pinMode(m0R, OUTPUT);
  pinMode(mbFreeR, OUTPUT);

  //左モータードライバーアウトプット
  pinMode(mLeftFWD, OUTPUT);
  pinMode(mLeftREV, OUTPUT);
  pinMode(stopModeL, OUTPUT);
  pinMode(m0L, OUTPUT);
  pinMode(mbFreeL, OUTPUT);

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

  //インバータ制御用ピン
  pinMode(inverterOut, OUTPUT);

  //ロードセル
  reader.begin();
  for (int i = 0; i < reader.doutLen; ++i) {
    offsetGrams[i] = parser.parseToGram(reader.values[i]);
  }

  //Dispyal
  display.setBrightness(0x0f);  // 明るさ最大

  //ResetSwitch
  pinMode(ResetSwitch, INPUT_PULLUP);  // 内蔵プルアップ使用

  //LightRed
  pinMode(LightRed, OUTPUT);

  //LightYellow
  pinMode(LightYellow, OUTPUT);

  //LED
  pinMode(LedRed, OUTPUT);
  pinMode(LedGreen, OUTPUT);
  pinMode(LedBlue, OUTPUT);
  pinMode(LedYellow, OUTPUT);
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
  inverter();  // ← D26 を ON/OFF
  updateAndDisplayWeightWithDisplayAndReset();
    // 重量チェックとYellowライト制御（インバータ停止時のみ動作）
  checkWeightAndSignalLight();


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
// void loop() {
//   // HIGH:時間設定 LOW:瞬時停止
//   digitalWrite(stopModeR, LOW);
//   digitalWrite(stopModeL, LOW);
//   // HIGH:速度指定を外部入力 LOW:内部速度設定器
//   digitalWrite(m0R, HIGH);
//   digitalWrite(m0L, HIGH);
//   //HIGH:ブレーキ解放 LOW:ブレーキ保持
//   digitalWrite(mbFreeR, HIGH);
//   digitalWrite(mbFreeL, HIGH);

//   SwitchA = pulseIn(ch5, HIGH, 25000);  // 動力スイッチ
//   SwitchB = pulseIn(ch6, HIGH, 25000);  // インバータ
//   SwitchE = pulseIn(ch7, HIGH, 25000);  // 記録/フリー/実行
//   inverter();  // ← D26 を ON/OFF
//   updateAndDisplayWeightWithDisplayAndReset();

//   // 重量チェックとYellowライト制御（インバータ停止時のみ動作）
//   checkWeightAndSignalLight();

//   if (SwitchA > 1550) {
//     digitalWrite(switcherOut, HIGH);  //動力リレーON

//     leftXb = leftXa;
//     rightX = pulseIn(ch1, HIGH);  // ch1の計測を開始
//     leftX = pulseIn(ch4, HIGH);   // ch4の計測を開始
//     rightY = pulseIn(ch3, HIGH);  // ch3の計測を開始
//     leftY = pulseIn(ch2, HIGH);   // ch2の計測を開始
//     leftXa = leftX;

//     gap = abs(leftXb - leftXa);
//     if (leftX < 1000 || leftX > 2000) {
//       leftX = leftXb;
//       leftXa = leftXb;
//     }
//     if (SwitchE < 1200) {  // 記録モード
//       if (!isRecording) {
//         Serial.println("Recording Start");
//         isRecording = true;
//         isPlaying = false;
//         recordIndex = 0;
//       }

//       // 一定間隔で記録
//       unsigned long now = millis();
//       if (recordIndex < recordMax && now - lastRecordTime >= recordInterval) {
//         recordLeftX[recordIndex] = leftX;
//         recordLeftY[recordIndex] = leftY;
//         recordRightX[recordIndex] = rightX;
//         recordRightY[recordIndex] = rightY;
//         recordIndex++;
//         lastRecordTime = now;
//       }

//       // 操作反映
//       motor(leftX, leftY);
//       lift(rightY);
//       dump(rightX);

//     } else if (SwitchE > 1400 && SwitchE < 1800) {  // フリーモード
//       if (isRecording) {
//         Serial.println("Recording Stop");
//         isRecording = false;
//       }
//       isPlaying = false;

//       lift(rightY);
//       dump(rightX);
//       motor(leftX, leftY);

//     } else if (SwitchE > 1900) {  // 実行モード
//       if (isRecording) {
//         Serial.println("Recording Stop");
//         isRecording = false;
//       }
//       isPlaying = true;
//       playRecord();  // 記録再生
//     } else {
//       leftX = leftX;
//       leftXa = leftXa;
//     }

//     //アーム動作
//     lift(rightY);
//     dump(rightX);

//     // モーター動作
//     motor(leftX, leftY);

//   } else {
//     digitalWrite(switcherOut, LOW);
//     stopMachine();
//   }
// }

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
void motor(int leftX, int leftY) {

  //up - motor control
  if (leftY <= 1450) {

    //前進
    digitalWrite(mRightFWD, LOW);
    digitalWrite(mRightREV, HIGH);
    digitalWrite(mLeftFWD, HIGH);
    digitalWrite(mLeftREV, LOW);

    // 前進速度
    if (leftY >= 1150) {
      int speed_m = map(leftY, 1450, 1150, 0, 255);
      analogWrite(mmLPin, speed_m);
      analogWrite(mmRPin, speed_m);
      Serial.println("111");
    } else if (leftY < 1150) {
      analogWrite(mmLPin, 255);
      analogWrite(mmRPin, 255);
      Serial.println("111AAA");
    }

    // 前進しながら方向転換
    if (leftX >= 1550) {
      int speed_m = map(leftY, 1450, 1000, 0, 255);
      analogWrite(mmLPin, speed_m);
      analogWrite(mmRPin, speed_m * 1 / 2);
      Serial.println("222");
    } else if (leftX <= 1450) {
      int speed_m = map(leftY, 1450, 1000, 0, 255);
      analogWrite(mmLPin, speed_m * 1 / 2);
      analogWrite(mmRPin, speed_m);
      Serial.println("888");
    }

  } else if (leftY > 1450 && leftY < 1550) {
    if (leftX > 1450 && leftX < 1550) {

      //停止
      digitalWrite(mRightFWD, LOW);
      digitalWrite(mRightREV, LOW);
      digitalWrite(mLeftFWD, LOW);
      digitalWrite(mLeftREV, LOW);

      analogWrite(mmLPin, 0);
      analogWrite(mmRPin, 0);

      // STOP表示
      Serial.println(" No Signal ");
    } else if (leftX >= 1550) {

      // 右旋回
      digitalWrite(mRightFWD, HIGH);
      digitalWrite(mRightREV, LOW);
      digitalWrite(mLeftFWD, HIGH);
      digitalWrite(mLeftREV, LOW);

      if (leftX <= 2000) {
        int speed_m = map(leftX, 2000, 1550, 255, 0);
        analogWrite(mmLPin, speed_m);
        analogWrite(mmRPin, speed_m);
        Serial.println("333");
      } else {
        analogWrite(mmLPin, 255);
        analogWrite(mmRPin, 255);
        Serial.println("333AAA");
      }
    } else if (leftX <= 1450) {  // left side - X

      //左旋回
      digitalWrite(mRightFWD, LOW);
      digitalWrite(mRightREV, HIGH);
      digitalWrite(mLeftFWD, LOW);
      digitalWrite(mLeftREV, HIGH);

      if (leftX >= 1000) {
        int speed_m = map(leftX, 1450, 1000, 0, 255);
        analogWrite(mmLPin, speed_m);
        analogWrite(mmRPin, speed_m);
        Serial.println("777");
      } else {
        analogWrite(mmLPin, 255);
        analogWrite(mmRPin, 255);
        Serial.println("777AAA");
      }
    }
  } else if (leftY >= 1550) {

    //後進
    digitalWrite(mRightFWD, HIGH);
    digitalWrite(mRightREV, LOW);
    digitalWrite(mLeftFWD, LOW);
    digitalWrite(mLeftREV, HIGH);

    //後進速度
    if (leftY <= 1900) {
      int speed_m = map(leftY, 1550, 1900, 0, 255);
      analogWrite(mmLPin, speed_m);
      analogWrite(mmRPin, speed_m);
      Serial.println("555");
    } else if (leftY > 1900) {
      analogWrite(mmLPin, 255);
      analogWrite(mmRPin, 255);
      Serial.println("555AAA");
    }

    //後進しながら方向転換
    if (leftX >= 1550) {  // lower right side - X
      int speed_m = map(leftY, 1550, 2000, 0, 255);
      analogWrite(mmLPin, speed_m);
      analogWrite(mmRPin, speed_m * 1 / 2);
      Serial.print("444");

    } else if (leftX <= 1450) {  // lower left side - X
      int speed_m = map(leftY, 1550, 2000, 0, 255);
      analogWrite(mmLPin, speed_m * 1 / 2);
      analogWrite(mmRPin, speed_m);
      Serial.print("666");
    }
  }
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

void inverter() {
  static unsigned long previousMillis = 0;
  static bool toggleState = false;
  const unsigned long interval = 500;

  if (SwitchB > 1550) {
    // インバータ稼働中：RedとYellowを交互に点滅
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      toggleState = !toggleState;
      digitalWrite(LightRed, toggleState);
      digitalWrite(LightYellow, !toggleState);
      digitalWrite(inverterOut, HIGH);
    }
  } else {
    // インバータ停止中：Red, Yellow両方消灯
    digitalWrite(LightRed, LOW);
    digitalWrite(LightYellow, LOW);
    digitalWrite(inverterOut, LOW);
  }
}

void updateAndDisplayWeightWithDisplayAndReset() {
  static unsigned long lastReadTime = 0;
  unsigned long now = millis();

  // 1秒ごとに処理
  if (now - lastReadTime >= 1000) {
    lastReadTime = now;

    auto readState = reader.read(1);
    if (readState == HX711_asukiaaa::ReadState::Success) {
      float totalGram = 0;
      for (int i = 0; i < reader.doutLen; ++i) {
        float gram = parser.parseToGram(reader.values[i]) - offsetGrams[i];
        totalGram += gram;
      }
      currentWeightKg = totalGram / 1000.0f;
      displayWeightKg = currentWeightKg;

      // ディスプレイに表示（小数点なしで4桁まで）
      int displayValue = (int)(displayWeightKg + 0.5);  // 四捨五入
      display.showNumberDec(displayValue, false, 4, 0);

      // デバッグ出力
      Serial.print("Weight: ");
      Serial.print(displayWeightKg);
      Serial.println(" kg");
    }
  }

  // リセットボタンが押されたとき（Low）
  if (digitalRead(ResetSwitch) == LOW) {
    Serial.println("Zeroing tare offset...");
    auto readState = reader.read(1);
    if (readState == HX711_asukiaaa::ReadState::Success) {
      for (int i = 0; i < reader.doutLen; ++i) {
        offsetGrams[i] = parser.parseToGram(reader.values[i]);
      }
    }
  }
  // 信号灯Yellow制御
  updateYellowLight(currentWeightKg);
}

void updateYellowLight(float weightKg) {
  unsigned long now = millis();

  if (weightKg >= 40.0) {
    // 常時点灯
    digitalWrite(LightRed, HIGH);
    digitalWrite(LightYellow, HIGH);
  } else if (weightKg >= 20.0) {
    // 0.5秒間隔で点滅
    if (now - lastBlinkTime >= 300) {
      yellowLedState = !yellowLedState;
      digitalWrite(LightYellow, yellowLedState ? HIGH : LOW);
      lastBlinkTime = now;
    }
  } else if (weightKg >= 10.0) {
    // 1秒間隔で点滅
    if (now - lastBlinkTime >= 1000) {
      yellowLedState = !yellowLedState;
      digitalWrite(LightYellow, yellowLedState ? HIGH : LOW);
      lastBlinkTime = now;
    }
  } else {
    // それ未満なら消灯
    digitalWrite(LightYellow, LOW);
  }
}

void checkWeightAndSignalLight() {
  if (inverterswitch <= 1550) {  // インバータ停止中のみYellow制御を許可
    if (weightKg >= 100.0) {
      digitalWrite(LightYellow, HIGH);  // 点灯
    } else if (weightKg >= 80.0) {
      // 0.5秒間隔で点滅
      if (millis() / 500 % 2 == 0) {
        digitalWrite(LightYellow, HIGH);
      } else {
        digitalWrite(LightYellow, LOW);
      }
    } else if (weightKg >= 50.0) {
      // 1秒間隔で点滅
      if (millis() / 1000 % 2 == 0) {
        digitalWrite(LightYellow, HIGH);
      } else {
        digitalWrite(LightYellow, LOW);
      }
    } else {
      digitalWrite(LightYellow, LOW);
    }
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
