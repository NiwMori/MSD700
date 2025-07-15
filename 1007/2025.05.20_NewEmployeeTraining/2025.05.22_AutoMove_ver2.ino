// ピン定義はあなたのコードのまま使います
#define mLeftFWD 43
#define mLeftREV 45
#define stopModeL 35
#define m0L 37
#define mbFreeL 41
#define mmLPin 3

#define mRightFWD 33
#define mRightREV 31
#define stopModeR 47
#define m0R 49
#define mbFreeR 53
#define mmRPin 2

#define switcherOut 39

const int ch1 = 13;
const int ch2 = 12;
const int ch3 = 11;
const int ch4 = 10;
const int ch5 = 9;
const int ch6 = 8;
const int ch7 = 7;
// const int ch8 = 6;

int leftX = 0, leftY = 0;
int rightX = 0, rightY = 0;

int recordMax = 2000;      // 最大記録数
int recordIndex = 0;       // 記録インデックス
bool isRecording = false;  // 記録中フラグ
bool isPlaying = false;    // 再生中フラグ

// 記録配列（leftX, leftYのペア）
int recordLeftX[1000];
int recordLeftY[1000];

// タイマー用（再生スピード調整用）
unsigned long lastPlayTime = 0;
const unsigned long playInterval = 50;  // 50ms毎に再生（20Hz）

// タイマー追加
unsigned long lastRecordTime = 0;
const unsigned long recordInterval = 10;  // 記録間隔：100ms

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

  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
  pinMode(ch7, INPUT);
  // pinMode(ch8, INPUT);

  pinMode(switcherOut, OUTPUT);

  digitalWrite(stopModeR, LOW);
  digitalWrite(stopModeL, LOW);

  digitalWrite(m0R, HIGH);
  digitalWrite(m0L, HIGH);

  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mbFreeL, HIGH);
}

void loop() {
  int swB = pulseIn(ch5, HIGH, 25000);
  bool powerOn = swB > 1500;

  if (!powerOn) {
    digitalWrite(switcherOut, LOW);
    stopMachine();
    isRecording = false;
    isPlaying = false;
    recordIndex = 0;
    return;
  }
  digitalWrite(switcherOut, HIGH);

  int ch7Val = pulseIn(ch7, HIGH, 25000);

  leftX = pulseIn(ch4, HIGH);
  leftY = pulseIn(ch2, HIGH);

  if (ch7Val < 1200) {  // 記録モード
    if (!isRecording) {
      Serial.println("Recording Start");
      isRecording = true;
      isPlaying = false;
      recordIndex = 0;
    }

    // 記録インターバルを制限
    unsigned long now = millis();
    if (recordIndex < recordMax && now - lastRecordTime >= recordInterval) {
      recordLeftX[recordIndex] = leftX;
      recordLeftY[recordIndex] = leftY;
      recordIndex++;
      lastRecordTime = now;
    }

    motor(leftX, leftY);
  } else if (ch7Val > 1400 && ch7Val < 1800) {  // フリーモード
    if (isRecording) {
      Serial.println("Recording Stop");
      isRecording = false;
    }
    isPlaying = false;
    motor(leftX, leftY);
  } else if (ch7Val > 1900) {  // 実行モード
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

  // 記録データを順に再生
  motor(recordLeftX[playIndex], recordLeftY[playIndex]);
  playIndex++;

  if (playIndex >= recordIndex) {
    playIndex = 0;  // ループ再生
  }
}

void stopMachine() {
  analogWrite(mmLPin, 0);
  analogWrite(mmRPin, 0);

  digitalWrite(mbFreeL, LOW);
  digitalWrite(mbFreeR, LOW);

  Serial.println("Stop");
}

// モーター関数
void motor(int leftX, int leftY) {

  //up - motor control
  if (leftY <= 1450) {

    //前進
    digitalWrite(mRightFWD, HIGH);
    digitalWrite(mRightREV, LOW);
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
      digitalWrite(mRightFWD, LOW);
      digitalWrite(mRightREV, HIGH);
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
      digitalWrite(mRightFWD, HIGH);
      digitalWrite(mRightREV, LOW);
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
// motor関数は質問のコードをそのまま使用してください
// もし修正したいならここにコピペしてください

// --- motor() 関数は省略 ---
// 質問にある motor(leftX, leftY) をここに入れてください
