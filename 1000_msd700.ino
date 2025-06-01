/*
2022-10-26

*/




#define RightmbFree 53
#define RightAlarmReset 51
#define Rightm0 49
#define RightStopMode 47
#define RightAlarmOut 40
#define RightwngN 38
#define RightSpeedOut 36
#define RightFWD 33
#define RightREV 31
#define RightmmPin 3

#define LeftREV 45
#define LeftFWD 43
#define LeftmbFree 41
#define LeftAlarmReset 39
#define Leftm0 37
#define LeftStopMode 35 
#define LeftSpeedOut 34
#define LeftwngN 32
#define LeftAlarmOutN 30
#define LeftmmPin 2

#define DumpFWD 50
#define DumpREV 52
#define LiftFWD 46
#define LiftREV 48

#define LiftFB A0
#define DumpFB A1

#define BatteryPin A2
const int ch1 = 13;
const int ch3 = 11;
const int ch2 = 12;
const int ch4 = 10;
const int ch5 = 9;
const int ch7 = 7;
const int SwitcherIn = 8;
const int SwitcherOut = 20;

int BatteryV = 0, Type = 0, VR = 0, switcher = 0, gap;
int RightX = 0, RightY = 0, LeftX, LeftXa, LeftXb, LeftY = 0;
double DumpData = 0,LiftData = 0;
int thresholdHigh = 1715, thresholdMid = 1510, thresholdLow = 1308;//プロポ閾値


/*for Serialprint*/
long CurrentType;
int speedX,speedY, Extend = 0, Retract = 0;
double batteryVo, batteryVoltage;

void AttachmentModeChange(){
    CurrentType = digitalRead(ch5);
    switch (CurrentType){
    case 0:
        Type = 0;
        break;
    case 1:
        Type = 1;
        break;
    }
}

void batteryData(){
    BatteryV = analogRead(BatteryPin);
      batteryVo = ((BatteryV+0.5)*5.00)/1024;
      batteryVoltage = batteryVo*30.00/5.00;
    //Serial.print(" Battery Voltage : ");  
    Serial.print(BatteryV); 
    Serial.print(","); Serial.print(batteryVo);
    Serial.print(","); Serial.println(batteryVoltage);
}

void printall(){
  /*battery output values for IoT sysytem
  IoTシステム用バッテリー出力値*/
    //Serial.print(" Battery Voltage : ");  
    Serial.print(BatteryV); 
    Serial.print(","); Serial.print(batteryVo);
    Serial.print(","); Serial.println(batteryVoltage);
     
  /*Value of the controller's switch for type switching
  タイプ切り替えのためのコントローラのスイッチの値*/
    Serial.print(CurrentType);

  /*Motor speed output from MotorDrive function
  モータの速度の表示*/
    Serial.print("speed X => ");
    Serial.println(speedX);
    Serial.print("speed Y => ");
    Serial.println(speedY);

  /*Value of the attachment arms from Lift function
  バケット/ブレードのシリンダの値*/
    Serial.print("Extend => ");
    Serial.println(Extend);
    Serial.print("Retract => ");
    Serial.println(Retract);
}

void Lift(int pulse){   
    bool dirLift = false;
    bool canLift = false;
    
    if (pulse >= thresholdHigh) {
      Retract = 1;
    } else if (pulse <= thresholdLow) { 
      Extend = 1;
    }
  
    if (Extend == 1) {
      dirLift = true;
      canLift = true;
    } else if (Retract == 1) {
      dirLift = false;
      canLift = true;
    } else {
      canLift = false;
    }
    digitalWrite(LiftFWD, dirLift & canLift);
    digitalWrite(LiftREV, (!dirLift) & canLift);
}

void Dump(int pulse){
  bool dirDump = false;
  bool canDump = false;
  
  if (pulse >= thresholdHigh) {
    Extend = 1;
  } else if (pulse <= thresholdLow) { 
    Retract = 1;
  }

  if (Extend == 1) {
    dirDump = true;
    canDump = true;
  } else if (Retract == 1) {
    dirDump = false;
    canDump = true;
  } else {
    canDump = false;
  }

  if (Type == 0) {
    digitalWrite(DumpFWD, dirDump & canDump);
    digitalWrite(DumpREV, (!dirDump) & canDump);
  } else {
    digitalWrite(DumpFWD, (!dirDump) & canDump);
    digitalWrite(DumpREV, dirDump & canDump);
  }
}

void MotorDrive(int LeftX, int LeftY){
    //移動方向フラグ
    int Right = 0;
    int Left = 0;
    int Forward = 0;
    int Reverse = 0;
  
    //モーター回転方向フラグ
    bool dirR = false;
    bool dirL = false;
    bool canMove = false;
  
    // 移動方向決定
    if (LeftX >= thresholdHigh) {
      Right = 1;
    } else if (LeftX <= thresholdLow) {
      Left = 1;
    }
    if (LeftY >= thresholdHigh) {
      Forward = 1;
    } else if (LeftY <= thresholdLow) {
      Reverse = 1;
    }
  
    //前後左右フラグ反転
    if(Type == 0){
      int temp = Forward;
      Forward = Reverse;
      Reverse = temp;
      temp = Right;
      Right = Left;
      Left = temp;
    }
  
    //モーター動作決定
    int diffX = abs(LeftX-thresholdMid);
    int diffY = abs(LeftY-thresholdMid);
    int speedX = map(diffX, 0, 410, 0, 255);
    int speedY = map(diffY, 0, 410, 0, 255);

    if (Forward == 1) {
      canMove = true;
      dirR = dirL = false;
      if (Right == 1) {
        //右前進
        analogWrite(LeftmmPin, speedY);
        analogWrite(RightmmPin, 0);
      } else if (Left == 1) {
        //左前進
        analogWrite(LeftmmPin, 0);
        analogWrite(RightmmPin, speedY);
      } else {
        //前進
        analogWrite(LeftmmPin, speedY);
        analogWrite(RightmmPin, speedY);
      }
    } else if (Reverse == 1) {
      canMove = true;
      dirR = dirL = true;
      if (Right == 1) {
        //右後進
        analogWrite(LeftmmPin, speedY);
        analogWrite(RightmmPin, 0);
      } else if (Left == 1) {
        //左後進
        analogWrite(LeftmmPin, 0);
        analogWrite(RightmmPin, speedY);
      } else {
        //後進
        analogWrite(LeftmmPin, speedY);
        analogWrite(RightmmPin, speedY);
      }
    } else {
      if (Right == 1) {
        //右旋回
        canMove = true;
        if (Type == 0) {
          dirR = true;
          dirL = false;
        } else {
          dirR = false;
          dirL = true;  
        }
        analogWrite(LeftmmPin, speedX);
        analogWrite(RightmmPin, speedX);
      } else if (Left == 1) {
        //左旋回
        canMove = true;
        if (Type == 0) {
          dirR = false;
          dirL = true;
        } else {
          dirR = true;
          dirL = false;  
        }
        analogWrite(LeftmmPin, speedX);
        analogWrite(RightmmPin, speedX);
      } else {
        //停止
        canMove = false;
        analogWrite(LeftmmPin, 0);
        analogWrite(RightmmPin, 0);
      }
    }
    digitalWrite(RightFWD, dirR & canMove);
    digitalWrite(RightREV, (!dirR) & canMove);
    digitalWrite(LeftFWD, dirL & canMove);
    digitalWrite(LeftREV, (!dirL) & canMove);
}

void StopMachine(){
  analogWrite(LeftmmPin, 0);//モーター速度0
  analogWrite(RightmmPin, 0);
  //前進後進指示なし
  digitalWrite(LiftFWD, LOW);
  digitalWrite(LiftREV, LOW);
  digitalWrite(DumpFWD, LOW);
  digitalWrite(DumpREV, LOW);

  //HIGH:ブレーキ解放 LOW:ブレーキ保持
  digitalWrite(LeftmbFree, LOW);
  digitalWrite(RightmbFree, LOW);

  // STOP表示
  Serial.println(" Stop ");
}

void setup() {
    Serial.begin(9600);

    //モータードライバー速度設定
    pinMode(LeftmmPin, OUTPUT);
    pinMode(RightmmPin, OUTPUT);
  
    //右モータードライバーアウトプット
    pinMode(RightFWD, OUTPUT);
    pinMode(RightREV, OUTPUT);
    pinMode(RightStopMode, OUTPUT);
    pinMode(Rightm0, OUTPUT);
    pinMode(RightAlarmReset, OUTPUT);
    pinMode(RightmbFree, OUTPUT);
  
    //右モータードライバーインプット
    pinMode(RightAlarmOut, INPUT);
    pinMode(RightwngN, INPUT);
    pinMode(RightSpeedOut, INPUT);
  
    //左モータードライバーアウトプット
    pinMode(LeftFWD, OUTPUT);
    pinMode(LeftREV, OUTPUT);
    pinMode(LeftStopMode; OUTPUT);
    pinMode(Leftm0, OUTPUT);
    pinMode(LeftAlarmReset, OUTPUT);
    pinMode(LeftmbFree, OUTPUT);
  
    //左モータードライバーインプット
    pinMode(LeftAlarmOutN, INPUT);
    pinMode(LeftwngN, INPUT);
    pinMode(LeftSpeedOut, INPUT);
  
    //シリンダーモータードライバー
    pinMode(LiftFWD, OUTPUT);
    pinMode(LiftREV, OUTPUT);
    pinMode(DumpFWD, OUTPUT);
    pinMode(DumpREV, OUTPUT);
  
    //レシーバー
    pinMode(ch1, INPUT);
    pinMode(ch3, INPUT);
    pinMode(ch2, INPUT);
    pinMode(ch4, INPUT);
    pinMode(ch5, INPUT);
    pinMode(ch7, INPUT);
    pinMode(SwitcherIn, INPUT);
    pinMode(SwitcherOut, OUTPUT);    //動力リレー用リレー
  
    //アームフィードバック
    pinMode(LiftFB, INPUT);
    pinMode(DumpFB, INPUT);
  
    // HIGH:時間設定 LOW:瞬時停止
    digitalWrite(RightStopMode, LOW);
    digitalWrite(LeftStopMode, LOW);
    
    // HIGH:速度指定を外部入力 LOW:内部速度設定器
    digitalWrite(Rightm0, HIGH);
    digitalWrite(Leftm0, HIGH);
    
    //HIGH:ブレーキ解放 LOW:ブレーキ保持
    digitalWrite(RightmbFree, HIGH);
    digitalWrite(LeftmbFree, HIGH);
  
    //Battery Pin
    pinMode(BatteryPin, INPUT);
}

void loop() {
    AttachmentModeChange();
    // batteryData();
    switcher = pulseIn(SwitcherIn, HIGH);

    if (switcher > 1550) {
        digitalWrite(SwitcherOut, LOW);
        VR = pulseIn(ch7, HIGH);

    if (VR > 1700) {
      LeftXb = LeftXa;
      RightX = pulseIn(ch1, HIGH); // ch1の計測を開始
      RightY = pulseIn(ch3, HIGH); // ch3の計測を開始
      LeftX = pulseIn(ch2, HIGH);  // ch2の計測を開始
      LeftY = pulseIn(ch4, HIGH);  // ch4の計測を開始
      LeftXa = LeftX;

      gap = abs(LeftXb - LeftXa);
      if (LeftX < 1000 || LeftX > 2000) {
        LeftX = LeftXb;
        LeftXa = LeftXb;
      } else {
        LeftX = LeftX;
        LeftXa = LeftXa;
      }

      //アーム動作
      Lift(RightY);
      Dump(RightX);

      // モーター動作
      MotorDrive(LeftX, LeftY);
    } else {
      StopMachine();
    }
    } else {
      digitalWrite(SwitcherOut, HIGH);
      StopMachine();
    }

    printall();
}