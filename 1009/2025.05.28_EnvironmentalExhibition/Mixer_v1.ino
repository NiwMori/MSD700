//2025.05.08

//左モータードライバーピン割振
#define mLeftFWD 30
#define mLeftREV 31
#define stopModeL 32
#define m0L 33
#define mbFreeL 34
#define mmLPin 2 // left motor speed PWM

//右モータードライバーピン割振
#define mRightFWD 35
#define mRightREV 36
#define stopModeR 37
#define m0R 38
#define mbFreeR 39
#define mmRPin 3 // Right motor speed PWM

//シリンダーモータードライバーピン割振
#define liftFWD 42
#define liftREV 43
#define dumpFWD 44
#define dumpREV 45

//動力リレー用リレーピン割振
#define switcherOut 24 // switch output

//インバータ制御用ピン
#define inverterOut 47  // ch6 スイッチ制御出力ピン

//レシーバーピン割振
const int ch1 = 6; // right X
const int ch2 = 7; // left Y
const int ch3 = 8; // right Y
const int ch4 = 9; // left X
const int ch5 = 10; /// switchA
const int ch6 = 11; // switchB
                           
//レシーバー用変数
int VR = 0;
int rightX = 0;
int rightY = 0;
int leftX, leftXa , leftXb;
int leftY = 0;
int switcher = 0;
int gap;
int inverterswitch = 0;

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

    //動力リレー用リレー
    pinMode(switcherOut, OUTPUT);

    //インバータ制御用ピン
    pinMode(inverterOut, OUTPUT);


}

void loop() {
    
  // HIGH:時間設定 LOW:瞬時停止
  digitalWrite(stopModeR, LOW);
  digitalWrite(stopModeL, LOW);
  // HIGH:速度指定を外部入力 LOW:内部速度設定器
  digitalWrite(m0R, HIGH);
  digitalWrite(m0L, HIGH);
  //HIGH:ブレーキ解放 LOW:ブレーキ保持
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mbFreeL, HIGH);
  
  switcher = pulseIn(ch5, HIGH);
  inverterswitch = pulseIn(ch6,HIGH);
   inverter(); // ← D26 を ON/OFF
  Serial.print("ch6 pulse: ");
  Serial.println(inverterswitch);
  
  if (switcher > 1550){
    digitalWrite(switcherOut, HIGH); //動力リレーON

      leftXb = leftXa;
      rightX = pulseIn(ch1, HIGH); // ch1の計測を開始
      leftX = pulseIn(ch4, HIGH);  // ch4の計測を開始
      rightY = pulseIn(ch3, HIGH); // ch3の計測を開始
      leftY = pulseIn(ch2, HIGH);  // ch2の計測を開始
      leftXa = leftX;
   
      gap = abs(leftXb - leftXa);
      if(leftX < 1000 || leftX > 2000){
        leftX = leftXb;
        leftXa = leftXb;
      } else {
        leftX = leftX;
        leftXa = leftXa;
      } 


      //アーム動作
      lift(rightY);
      dump(rightX);
      
      // モーター動作
      motor(leftX, leftY);

  } else {
    digitalWrite(switcherOut, LOW);
    stopMachine();
  }
}  

// モーター停止関数
void stopMachine(){

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
void motor(int leftX, int leftY){
  
      //up - motor control
      if (leftY <= 1450) {
        
        //前進
        digitalWrite(mRightFWD, LOW);
        digitalWrite(mRightREV, HIGH);
        digitalWrite(mLeftFWD, HIGH);
        digitalWrite(mLeftREV, LOW);

        // 前進速度
        if (leftY >= 1150){
            int speed_m = map(leftY, 1450, 1150, 0, 255);
            analogWrite(mmLPin, speed_m);
            analogWrite(mmRPin, speed_m);
            Serial.println("111");
        } else if (leftY < 1150){
            analogWrite(mmLPin, 255);
            analogWrite(mmRPin, 255);
            Serial.println("111AAA");
        }
        
        // 前進しながら方向転換
        if (leftX>=1550){
          int speed_m = map(leftY, 1450, 1000, 0, 255);
          analogWrite(mmLPin, speed_m);
          analogWrite(mmRPin, speed_m*1/2);
          Serial.println("222");
        } else if (leftX<=1450){
          int speed_m = map(leftY, 1450, 1000, 0, 255);
          analogWrite(mmLPin, speed_m*1/2);
          analogWrite(mmRPin, speed_m);
          Serial.println("888");
        }      

      } else if (leftY > 1450 && leftY<1550){
        if (leftX > 1450 && leftX<1550){
          
          //停止
          digitalWrite(mRightFWD, LOW);
          digitalWrite(mRightREV, LOW);
          digitalWrite(mLeftFWD, LOW);
          digitalWrite(mLeftREV, LOW);
          
          analogWrite(mmLPin, 0);
          analogWrite(mmRPin, 0);

          // STOP表示
          Serial.println(" No Signal ");
        } else if (leftX>=1550){
          
          // 右旋回
          digitalWrite(mRightFWD, HIGH);
          digitalWrite(mRightREV, LOW);
          digitalWrite(mLeftFWD, HIGH);
          digitalWrite(mLeftREV, LOW);
   
          if (leftX <= 2000){ 
            int speed_m = map(leftX, 2000, 1550, 255, 0);
            analogWrite(mmLPin, speed_m);
            analogWrite(mmRPin, speed_m);
            Serial.println("333");
          } else {
            analogWrite(mmLPin, 255);
            analogWrite(mmRPin, 255);
            Serial.println("333AAA");
          }
        }
        else if(leftX<=1450){ // left side - X

          //左旋回
          digitalWrite(mRightFWD, LOW);
          digitalWrite(mRightREV, HIGH);
          digitalWrite(mLeftFWD, LOW);
          digitalWrite(mLeftREV, HIGH);

          if (leftX >= 1000){
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
      } else if (leftY >= 1550){
        
        //後進
        digitalWrite(mRightFWD, HIGH);
        digitalWrite(mRightREV, LOW);
        digitalWrite(mLeftFWD, LOW);
        digitalWrite(mLeftREV, HIGH);
        
        //後進速度
        if (leftY <= 1900){
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
        if (leftX>=1550){ // lower right side - X
          int speed_m = map(leftY, 1550, 2000, 0, 255);
          analogWrite(mmLPin, speed_m);
          analogWrite(mmRPin, speed_m*1/2);
          Serial.print("444");

        } else if (leftX<=1450){  // lower left side - X
          int speed_m = map(leftY, 1550, 2000, 0, 255);
          analogWrite(mmLPin, speed_m*1/2);
          analogWrite(mmRPin, speed_m);
          Serial.print("666");
        }
      }  
  
} 

//アームリフト関数
void lift(int pulse){
      if (pulse<1250 && pulse>950){
        digitalWrite(liftFWD,HIGH);
        digitalWrite(liftREV,LOW);
      }else if(pulse<2070 && pulse>1750){
        digitalWrite(liftFWD,LOW);
        digitalWrite(liftREV,HIGH);
      }else if (pulse<1750 && pulse>1250){
        digitalWrite(liftFWD,LOW);
        digitalWrite(liftREV,LOW);
      }
}

//アームダンプ関数
void dump(int pulse){
      if (pulse<1250 && pulse>950){
        digitalWrite(dumpFWD,HIGH);
        digitalWrite(dumpREV,LOW);
      }else if(pulse<2070 && pulse>1750){
        digitalWrite(dumpFWD,LOW);
        digitalWrite(dumpREV,HIGH);
      }else if (pulse<1750 && pulse>1250){
        digitalWrite(dumpFWD,LOW);
        digitalWrite(dumpREV,LOW);
      }
}

//インバータ
void inverter() {
  // スイッチが OFF→ON に変化した瞬間だけ切り替え
  if (inverterswitch > 1550) {
    digitalWrite(inverterOut,HIGH);
  }else{
    digitalWrite(inverterOut,LOW);
  }

}

