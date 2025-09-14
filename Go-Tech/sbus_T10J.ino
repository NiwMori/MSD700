#include <sbus.h>

bfs::SbusRx sbus_rx(&Serial1);  // RX1を使用
bfs::SbusData data;             // 受信データ格納用

void setup() {
  Serial.begin(115200);
  sbus_rx.Begin();  // SBUS通信開始
}

void loop() {
  if (sbus_rx.Read()) {  // 新しいフレームを受信したら
    data = sbus_rx.data();  // データ取得

    // CH1〜CH10を表示
    for (int i = 0; i < 10; i++) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    // フェイルセーフやフレームロストも表示
    Serial.print("Failsafe: "); Serial.print(data.failsafe);
    Serial.print("\tLostFrame: "); Serial.println(data.lost_frame);
  }
}
