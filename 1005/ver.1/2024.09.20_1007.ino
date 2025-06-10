
// 左モータードライバーピン割振
#define mRightFWD 33 // CR7
#define mRightREV 31 // CR8
#define stopModeR 47 // CR9
#define m0R 49       // CR10
// #define alarmResetR 51 //CR11
#define mbFreeR 53 // CR12
// #define alarmOutLN 30
// #define wngLN 32
// #define speedOutL 34
#define mmLPin 2 // left motor speed PWM --  pin 12

// 右モータードライバーピン割振
#define mLeftFWD 43  // CR1
#define mLeftREV 45  // CR2
#define stopModeL 35 // CR3
#define m0L 37       // CR4
// #define alarmResetL 39 // CR5
#define mbFreeL 41 // CR6
// #define alarmOutRN 40
// #define wngRN 38
// #define speedOutR 36
#define mmRPin 3 // right motor speed PWM --  pin 12

// シリンダーモータードライバーピン割振
#define dumpFWD 50
#define dumpREV 52
#define liftFWD 46
#define liftREV 48

// battery feedback
int batteryPin = A2; // battery voltage reader
int batteryV = 0;
int value;
float volt;

// 動力リレー用リレーピン割振
#define switcherOut 39 // 5 // switch output

// 信号灯制御
#define led 51

// レシーバーピン割振
const int ch1 = 13; // right horizontal
const int ch4 = 12; // 12; // left vertical
const int ch3 = 11; // right vertical
const int ch2 = 10; // 10; // left horizontal
const int ch5 = 9;  // sw(B) for convert FWD/REV direction
// const int ch7 = 7; /// VR channel
const int switcherIn = 8; // ch6 switch input

// モード切り替え
#define modeselect 23
// モード設定
int gMode = 0; // バケットモード:1, ブレードモード:0

// レシーバー用変数
int VR = 0;
int rightX = 0;
int rightY = 0;
int leftX, leftXa, leftXb;
int leftY = 0;
int switcher = 0;
int gap;
int directionMode = 0;

void handleModeSwitch()
{ // スイッチの状態を読み取る
    int state = digitalRead(modeselect);
    if (state == HIGH)
    {
        gMode = 1; // モードを切り替える
        Serial.println("GMODE 1");
        // Serial.println("23番ピン: HIGH (未接続またはプルアップ)");
    }
    else
    {
        // Serial.println("23番ピン: LOW (GNDに接続)");
        gMode = 0;

        Serial.println("GMODE 0");
    }
}

void setup()
{

    Serial.begin(9600);

    // モータードライバー速度設定
    pinMode(mmLPin, OUTPUT);
    pinMode(mmRPin, OUTPUT);

    // 右モータードライバーアウトプット
    pinMode(mRightFWD, OUTPUT);
    pinMode(mRightREV, OUTPUT);
    pinMode(stopModeR, OUTPUT);
    pinMode(m0R, OUTPUT);
    pinMode(mbFreeR, OUTPUT);

    // 左モータードライバーアウトプット
    pinMode(mLeftFWD, OUTPUT);
    pinMode(mLeftREV, OUTPUT);
    pinMode(stopModeL, OUTPUT);
    pinMode(m0L, OUTPUT);
    pinMode(mbFreeL, OUTPUT);

    // シリンダーモータードライバー
    pinMode(liftFWD, OUTPUT);
    pinMode(liftREV, OUTPUT);
    pinMode(dumpFWD, OUTPUT);
    pinMode(dumpREV, OUTPUT);

    // レシーバー
    pinMode(ch1, INPUT);
    pinMode(ch2, INPUT);
    pinMode(ch3, INPUT);
    pinMode(ch4, INPUT);
    pinMode(ch5, INPUT);
    // pinMode(ch7, INPUT);
    pinMode(switcherIn, INPUT);

    // 動力リレー用リレー
    pinMode(switcherOut, OUTPUT);

    // HIGH:時間設定 LOW:瞬時停止
    digitalWrite(stopModeR, LOW);
    digitalWrite(stopModeL, LOW);

    // HIGH:速度指定を外部入力 LOW:内部速度設定器
    digitalWrite(m0R, HIGH);
    digitalWrite(m0L, HIGH);

    // HIGH:ブレーキ解放 LOW:ブレーキ保持
    digitalWrite(mbFreeR, HIGH);
    digitalWrite(mbFreeL, HIGH);

    // Battery Pin
    pinMode(batteryPin, INPUT);

    // SignalLight
    pinMode(led, OUTPUT);

    // ModeSelect
    pinMode(modeselect, INPUT_PULLUP);
}

void loop()
{
    handleModeSwitch(); // バケットモードとブレードモードの切り替え
    batteryData();
    switcher = pulseIn(switcherIn, HIGH);
    // Serial.println(switcher);

    if (switcher > 1550)
    {
        digitalWrite(switcherOut, HIGH); // 動力リレーON
        rightX = pulseIn(ch1, HIGH);     // ch1の計測を開始
        rightY = pulseIn(ch3, HIGH);     // ch3の計測を開始
        leftX = pulseIn(ch2, HIGH);      // ch2の計測を開始
        leftY = pulseIn(ch4, HIGH);      // ch4の計測を開始

        // アーム動作
        lift(rightY);
        dump(rightX);

        // モーター動作
        motor(leftX, leftY);
    }
    else
    {
        digitalWrite(switcherOut, LOW);
        stopMachine();
    }
}

// モーター停止関数
void stopMachine()
{
    // モーター速度0
    analogWrite(mmLPin, 0);
    analogWrite(mmRPin, 0);

    // 前進後進指示なし
    digitalWrite(liftFWD, LOW);
    digitalWrite(liftREV, LOW);
    digitalWrite(dumpFWD, LOW);
    digitalWrite(dumpREV, LOW);

    // HIGH:ブレーキ解放 LOW:ブレーキ保持
    digitalWrite(mbFreeL, HIGH);
    digitalWrite(mbFreeR, HIGH); // LOW
}

// モーター関数
void motor(int leftX, int leftY)
{
    // プロポ閾値
    int thresholdHigh = 1750;
    int thresholdMid = 1510;
    int thresholdLow = 1304;

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
    if (leftX >= thresholdHigh)
    {
        Right = 1;
    }
    else if (leftX <= thresholdLow)
    {
        Left = 1;
    }
    if (leftY >= thresholdHigh)
    {
        Forward = 1;
    }
    else if (leftY <= thresholdLow)
    {
        Reverse = 1;
    }

    // 前後左右フラグ反転
    if (gMode == 1)
    {
        int temp = Forward;
        Forward = Reverse;
        Reverse = temp;
        temp = Right;
        Right = Left;
        Left = temp;
    }

    // モーターの回転方向を反転
    if (gMode == 0)
    {
        // gModeが1の場合、モーターの回転方向を反転
        dirR = !dirR;
        dirL = !dirL;
    }

    // モーター動作決定
    int diffX = abs(leftX - thresholdMid);
    int diffY = abs(leftY - thresholdMid);
    int speedX = map(diffX, 0, 550, 0, 255);
    int speedY = map(diffY, 0, 415, 0, 255);

    if (Forward == 1)
    {
        Serial.println("11111");
        canMove = true;
        dirR = dirL = false;
        if (Right == 1)
        {
            Serial.println("22222");
            // 右前進
            analogWrite(mmLPin, speedY);
            analogWrite(mmRPin, 0);
        }
        else if (Left == 1)
        {
            Serial.println("333333");
            // 左前進
            analogWrite(mmLPin, 0);
            analogWrite(mmRPin, speedY);
        }
        else
        {
            Serial.println("444444");
            // 前進
            analogWrite(mmLPin, speedY);
            analogWrite(mmRPin, speedY);
        }
    }
    else if (Reverse == 1)
    {
        Serial.println("666666");
        canMove = true;
        dirR = dirL = true;
        if (Right == 1)
        {
            Serial.println("77777");
            // 右後進
            analogWrite(mmLPin, speedY);
            analogWrite(mmRPin, 0);
        }
        else if (Left == 1)
        {
            
            // 左後進
            analogWrite(mmLPin, 0);
            analogWrite(mmRPin, speedY);
        }
        else
        {
            // 後進
            analogWrite(mmLPin, speedY);
            analogWrite(mmRPin, speedY);
        }
    }
    else
    {
        if (Right == 1)
        {
            // 右旋回
            canMove = true;
            if (gMode == 1)
            {
                dirR = true;
                dirL = false;
            }
            else
            {
                dirR = false;
                dirL = true;
            }
            analogWrite(mmLPin, speedX);
            analogWrite(mmRPin, speedX);
        }
        else if (Left == 1)
        {
            // 左旋回
            canMove = true;
            if (gMode == 1)
            {
                dirR = false;
                dirL = true;
            }
            else
            {
                dirR = true;
                dirL = false;
            }
            analogWrite(mmLPin, speedX);
            analogWrite(mmRPin, speedX);
        }
        else
        {
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

// void motor(int leftX, int leftY) {
//   //プロポ閾値
//   int thresholdHigh = 1750; //1715->1615->1565->1900->1850-> --1750
//   int thresholdMid = 1510; //1510->1410->1350->1500->1490->1450->--1510
//   int thresholdLow = 1304; //1308->1208->1158->1100->--1304

//   //移動方向フラグ
//   int Right = 0;
//   int Left = 0;
//   int Forward = 0;
//   int Reverse = 0;

//   //モーター回転方向フラグ
//   bool dirR = false;
//   bool dirL = false;
//   bool canMove = false;

//   // 移動方向決定
//   if (leftX >= thresholdHigh) {
//     Right = 1;
//   } else if (leftX <= thresholdLow) {
//     Left = 1;
//   }
//   if (leftY >= thresholdHigh) {
//     Forward = 1;
//   } else if (leftY <= thresholdLow) {
//     Reverse = 1;
//   }

//   //前後左右フラグ反転
//   if(gMode == 0){
//     int temp = Forward;
//     Forward = Reverse;
//     Reverse = temp;
//     temp = Right;
//     Right = Left;
//     Left = temp;
//   }

//   //モーター動作決定
//   int diffX = abs(leftX-thresholdMid);
//   int diffY = abs(leftY-thresholdMid);
//   int speedX = map(diffX, 0, 550, 0, 255); //410->610->--510--
//     //Serial.print("speedX => ");
//     //Serial.println(speedX);
//   int speedY = map(diffY, 0, 415, 0, 255); //410->610->510 --415--
//     //Serial.print("speedY => ");
//     //Serial.println(speedY);

//   if (Forward == 1) {
//     canMove = true;
//     dirR = dirL = false;
//     if (Right == 1) {
//       //右前進
//       analogWrite(mmLPin, speedY);
//       analogWrite(mmRPin, 0);
//     } else if (Left == 1) {
//       //左前進
//       analogWrite(mmLPin, 0);
//       analogWrite(mmRPin, speedY);
//     } else {
//       //前進
//       analogWrite(mmLPin, speedY);
//       analogWrite(mmRPin, speedY);
//     }
//   } else if (Reverse == 1) {
//     canMove = true;
//     dirR = dirL = true;
//     if (Right == 1) {
//       //右後進
//       analogWrite(mmLPin, speedY);
//       analogWrite(mmRPin, 0);
//     } else if (Left == 1) {
//       //左後進
//       analogWrite(mmLPin, 0);
//       analogWrite(mmRPin, speedY);
//     } else {
//       //後進
//       analogWrite(mmLPin, speedY);
//       analogWrite(mmRPin, speedY);
//     }
//   } else {
//     if (Right == 1) {
//       //右旋回
//       canMove = true;
//       if (gMode == 0) {
//         dirR = true;
//         dirL = false;
//       } else {
//         dirR = false;
//         dirL = true;
//       }
//       analogWrite(mmLPin, speedX);
//       analogWrite(mmRPin, speedX);
//     } else if (Left == 1) {
//       //左旋回
//       canMove = true;
//       if (gMode == 0) {
//         dirR = false;
//         dirL = true;
//       } else {
//         dirR = true;
//         dirL = false;
//       }
//       analogWrite(mmLPin, speedX);
//       analogWrite(mmRPin, speedX);
//     } else {
//       //停止
//       canMove = false;
//       analogWrite(mmLPin, 0);
//       analogWrite(mmRPin, 0);
//     }
//   }
//   digitalWrite(mRightFWD, dirR & canMove);
//   digitalWrite(mRightREV, (!dirR) & canMove);
//   digitalWrite(mLeftFWD, dirL & canMove);
//   digitalWrite(mLeftREV, (!dirL) & canMove);
// }

// アームリフト関数
void lift(int pulse)
{
    // プロポ閾値
    int thresholdHigh = 1715;
    int inresholdMid = 1510;
    int thresholdLow = 1308;

    int Extend = 0;
    int Retract = 0;

    bool dirLift = false;
    bool canLift = false;

    if (pulse >= thresholdHigh)
    {
        Retract = 1;
    }
    else if (pulse <= thresholdLow)
    {
        Extend = 1;
    }
    // Serial.print("Extend => ");
    // Serial.println(Extend);
    // Serial.print("Retract => ");
    // Serial.println(Retract);

    if (Extend == 1)
    {
        dirLift = true;
        canLift = true;
    }
    else if (Retract == 1)
    {
        dirLift = false;
        canLift = true;
    }
    else
    {
        canLift = false;
    }
    digitalWrite(liftFWD, (dirLift)&canLift); // dirLift->(!dirLift)//入れ替え20220909
    digitalWrite(liftREV, (!dirLift) & canLift);
}

// アームダンプ関数
void dump(int pulse)
{
    // プロポ閾値
    int thresholdHigh = 1715;
    int thresholdMid = 1510;
    int thresholdLow = 1308;

    int Extend = 0;
    int Retract = 0;

    bool dirDump = false;
    bool canDump = false;

    if (pulse >= thresholdHigh)
    {
        Extend = 1;
    }
    else if (pulse <= thresholdLow)
    {
        Retract = 1;
    }

    // Serial.print("Extend => ");
    // Serial.println(Extend);
    // Serial.print("Retract => ");
    // Serial.println(Retract);

    if (Extend == 1)
    {
        dirDump = true;
        canDump = true;
    }
    else if (Retract == 1)
    {
        dirDump = false;
        canDump = true;
    }
    else
    {
        canDump = false;
    }

    if (gMode == 0)
    {
        digitalWrite(dumpFWD, dirDump & canDump);
        digitalWrite(dumpREV, (!dirDump) & canDump);
    }
    else
    {
        digitalWrite(dumpFWD, (!dirDump) & canDump);
        digitalWrite(dumpREV, dirDump & canDump);
    }
}

void batteryData()
{
    value = analogRead(batteryPin);
    volt = value * 5.0 / 1023.0;
    volt = volt * 30.0 / 5.0; // 30Vバッテリーの場合のスケーリング

    // Serial.print("Value: ");
    // Serial.print(value);
    // Serial.print("  Volt: ");
    // Serial.println(volt);

    // 電圧が20V未満の場合、走行モーターのみ稼働
    if (volt < 20.00 && volt >= 18.0)
    {
        Serial.println("Voltage below 20V. Only driving motors are active.");
        // アーム動作を停止
        digitalWrite(led, HIGH);
        digitalWrite(liftFWD, LOW);
        digitalWrite(liftREV, LOW);
        digitalWrite(dumpFWD, LOW);
        digitalWrite(dumpREV, LOW);
        // 走行モーターは稼働（必要に応じてmotor関数内の条件で処理）
    }
    // 電圧が18V未満の場合、すべての動作を停止
    else if (volt < 18.0)
    {
        Serial.println("Voltage below 18V. Shutting down all operations.");
        digitalWrite(led, HIGH);
        stopMachine(); // すべての動作を停止
    }
}