// Arduino Megaでピン24〜25, 30〜39を順番に制御（0.5秒オン・0.5秒オフ、1周後5秒停止）

int relayPins[] = {
  24, 25, 26, 27,// 動力リレー・積層信号灯
  30, 31, 32, 33, 34, // 左モーター関連
  35, 36, 37, 38, 39  // 右モーター関連
};

void setup() {
  for (int i = 0; i < sizeof(relayPins) / sizeof(relayPins[0]); i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW); // 初期状態：すべてOFF
  }
}

void loop() {
  int numPins = sizeof(relayPins) / sizeof(relayPins[0]);

  for (int i = 0; i < numPins; i++) {
    digitalWrite(relayPins[i], HIGH); // リレーON
    delay(500);                       // 0.5秒待機
    digitalWrite(relayPins[i], LOW);  // リレーOFF
    delay(500);                       // 0.5秒待機
  }

  delay(5000); // 一周後に5秒停止
}
