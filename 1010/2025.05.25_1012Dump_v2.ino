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

#define switcherOut 24

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
const int ch5 = 10;  // SwitchA
const int ch6 = 11;  // Null
const int ch7 = 12;  // SwitchE
const int ch8 = 13;  // Null

//int leftX = 0, leftY = 0;
int rightX = 0, rightY = 0;

int recordMax = 1000;      // 最大記録数
int recordIndex = 0;       // 記録インデックス
bool isRecording = false;  // 記録中フラグ
bool isPlaying = false;    // 再生中フラグ

// 記録配列（leftX, leftYのペア）
// int recordLeftX[10];
// int recordLeftY[10];

//int recordMax = 1000;
int recordRightX[1000];  // dump
int recordRightY[1000];  // lift

// タイマー用（再生スピード調整用）
unsigned long lastPlayTime = 0;
const unsigned long playInterval = 50;  // 50ms毎に再生（20Hz）

// タイマー追加
unsigned long lastRecordTime = 0;
const unsigned long recordInterval = 10;  // 記録間隔：100ms

// //レシーバー用変数
// int VR = 0;
// int rightX = 0;
// int rightY = 0;
// int leftX, leftXa, leftXb;
// int leftY = 0;
// int switcher = 0;
// int gap;

int readVoltageRaw() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BatteryPin);
    delay(5);  // 少し待つと安定する
  }
  return sum / 10;
}
bool allowOperation = false;  // 動作許可フラグ（初期は停止）

// グローバル変数（setupの上に記述）
unsigned long previousMillis = 0;
bool lightState = false;
float volt = 0;
int value = 0;

void setup() {
  Serial.begin(9600);

  pinMode(mmLPin, OUTPUT);
  pinMode(mmRPin, OUTPUT);

  pinMode(mRightFWD, OUTPUT);
  pinMode(mRightREV, OUTPUT);
  pinMode(stopModeR, OUTPUT);
  pinMode(m0R, OUTPUT);
  pinMode(mbFreeR, OUTPUT);

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

  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  //pinMode(ch6, INPUT);
  pinMode(ch7, INPUT);
  //pinMode(ch8, INPUT);

  pinMode(switcherOut, OUTPUT);

  //積層信号灯赤色
  pinMode(LightRed, OUTPUT);
  //積層信号灯用リレー
  pinMode(BatteryPin, INPUT);

  digitalWrite(stopModeR, LOW);
  digitalWrite(stopModeL, LOW);

  digitalWrite(m0R, HIGH);
  digitalWrite(m0L, HIGH);

  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mbFreeL, HIGH);
}

void loop() {
  int SwitchB = pulseIn(ch5, HIGH, 25000);
  bool powerOn = SwitchB > 1500;

  if (!powerOn) {
    digitalWrite(switcherOut, LOW);
    stopMachine();
    isRecording = false;
    isPlaying = false;
    // recordIndex = 0;
    return;
  }
  digitalWrite(switcherOut, HIGH);
  int SwitchE = pulseIn(ch7, HIGH, 25000);
  // leftX = pulseIn(ch4, HIGH);
  // leftY = pulseIn(ch2, HIGH);

  //追加
  rightX = pulseIn(ch1, HIGH);
  rightY = pulseIn(ch3, HIGH);

  if (SwitchE < 1200) {  // 記録モード
    if (!isRecording) {
      Serial.println("Recording Start");
      isRecording = true;
      isPlaying = false;
      recordIndex = 0;
    }
    // 記録インターバルを制限
    unsigned long now = millis();
    if (recordIndex < recordMax && now - lastRecordTime >= recordInterval) {
      // recordLeftX[recordIndex] = leftX;
      // recordLeftY[recordIndex] = leftY;

      //追加
      recordRightX[recordIndex] = rightX;
      recordRightY[recordIndex] = rightY;

      recordIndex++;
      lastRecordTime = now;
    }
    // motor(leftX, leftY);

    //追加
    lift(rightY);
    dump(rightX);

    } else if (SwitchE > 1400 && SwitchE < 1800) {  // フリーモード
    if (isRecording) {
      Serial.println("Recording Stop");
      isRecording = false;
    }
    isPlaying = false;

    //追加
    lift(rightY);
    dump(rightX);

    // ★ ここを追加 ★
    int leftX = pulseIn(ch4, HIGH, 25000);
    int leftY = pulseIn(ch2, HIGH, 25000);
    motor(leftX, leftY);

  } else if (SwitchE > 1900) {  // 実行モード
    if (isRecording) {
      Serial.println("Recording Stop");
      isRecording = false;
    }
    isPlaying = true;
    playRecord();
  } else {
    stopMachine();
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

  // lift/dump再生
  lift(recordRightY[playIndex]);
  dump(recordRightX[playIndex]);

  playIndex++;

  if (playIndex >= recordIndex) {
    playIndex = 0;  // ループ再生
  }
}

void stopMachine() {
  digitalWrite(liftFWD, LOW);
  digitalWrite(liftREV, LOW);
  digitalWrite(dumpFWD, LOW);
  digitalWrite(dumpREV, LOW);
  Serial.println("Stop");
}

// モーター関数
void motor(int leftX, int leftY) {

  //up - motor control
  if (leftY <= 1450) {

    //前進
    digitalWrite(mRightFWD, HIGH);
    digitalWrite(mRightREV, LOW);
    digitalWrite(mLeftFWD, LOW);
    digitalWrite(mLeftREV, HIGH);

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
      analogWrite(mmLPin, speed_m * 1 / 2);
      analogWrite(mmRPin, speed_m);
      Serial.println("222");
    } else if (leftX <= 1450) {
      int speed_m = map(leftY, 1450, 1000, 0, 255);
      analogWrite(mmLPin, speed_m);
      analogWrite(mmRPin, speed_m * 1 / 2);
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
    digitalWrite(mRightFWD, LOW);
    digitalWrite(mRightREV, HIGH);
    digitalWrite(mLeftFWD, HIGH);
    digitalWrite(mLeftREV, LOW);

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
      analogWrite(mmLPin, speed_m * 1 / 2);
      analogWrite(mmRPin, speed_m);
      Serial.print("444");

    } else if (leftX <= 1450) {  // lower left side - X
      int speed_m = map(leftY, 1550, 2000, 0, 255);
      analogWrite(mmLPin, speed_m);
      analogWrite(mmRPin, speed_m * 1 / 2);
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
