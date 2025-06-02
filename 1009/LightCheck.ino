#define LightRed 25
#define LightYellow 27

void setup() {
  pinMode(LightRed, OUTPUT);
  pinMode(LightYellow, OUTPUT);
}

void loop() {
  digitalWrite(LightRed, HIGH);     // 赤 ON
  digitalWrite(LightYellow, LOW);   // 黄 OFF
  delay(1000);                      // 1秒待つ

  digitalWrite(LightRed, LOW);      // 赤 OFF
  digitalWrite(LightYellow, HIGH);  // 黄 ON
  delay(1000);                      // 1秒待つ
}
