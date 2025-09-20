// リレーピン
const int relayPins[4] = {22, 23, 24, 25};

unsigned long prevMillis = 0;
const unsigned long interval = 500; // 0.5秒
int currentRelay = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Relay Test Start");

  // ピン初期化
  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }
}

void loop() {
  unsigned long now = millis();

  if (now - prevMillis >= interval) {
    prevMillis = now;

    // すべてOFF
    for (int i = 0; i < 4; i++) {
      digitalWrite(relayPins[i], LOW);
    }

    // 現在のリレーだけON
    digitalWrite(relayPins[currentRelay], HIGH);

    // シリアルに表示
    Serial.print("Relay "); Serial.print(currentRelay); Serial.println(" ON");

    // 次のリレーに切り替え
    currentRelay++;
    if (currentRelay >= 4) currentRelay = 0;
  }
}
