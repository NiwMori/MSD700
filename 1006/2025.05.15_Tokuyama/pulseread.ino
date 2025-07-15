// 受信機チャンネル割り当て（接続したピン番号に合わせてください）
const int ch1 = 6;
const int ch2 = 7;
const int ch3 = 8;
const int ch4 = 9;
const int ch5 = 10;
const int ch6 = 11;
const int ch7 = 12;
const int ch8 = 13;

void setup() {
  Serial.begin(9600);

  // 入力ピン設定
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
  pinMode(ch7, INPUT);
  pinMode(ch8, INPUT);
}

void loop() {
  // 各チャンネルのパルス幅を取得（タイムアウト25ms）
  int val1 = pulseIn(ch1, HIGH, 25000);
  int val2 = pulseIn(ch2, HIGH, 25000);
  int val3 = pulseIn(ch3, HIGH, 25000);
  int val4 = pulseIn(ch4, HIGH, 25000);
  int val5 = pulseIn(ch5, HIGH, 25000);
  int val6 = pulseIn(ch6, HIGH, 25000);
  int val7 = pulseIn(ch7, HIGH, 25000);
  int val8 = pulseIn(ch8, HIGH, 25000);

  // シリアルモニタに出力
  Serial.print("CH1: "); Serial.print(val1);
  Serial.print(" | CH2: "); Serial.print(val2);
  Serial.print(" | CH3: "); Serial.print(val3);
  Serial.print(" | CH4: "); Serial.print(val4);
  Serial.print(" | CH5: "); Serial.print(val5);
  Serial.print(" | CH6: "); Serial.print(val6);
  Serial.print(" | CH7: "); Serial.print(val7);
  Serial.print(" | CH8: "); Serial.println(val8);

  delay(100); // 少し待機
}
