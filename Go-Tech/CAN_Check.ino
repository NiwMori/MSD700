#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 53;    // MCP2515 CSピン
const int CAN_INT_PIN = 2;    // MCP2515 INTピン

MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // MCP2515初期化（CAN速度250kbps、クロック8MHz）
  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN init OK!");
  } else {
    Serial.println("CAN init FAIL!");
    while (1);
  }

  CAN.setMode(MCP_NORMAL);       // ノーマルモード
  pinMode(CAN_INT_PIN, INPUT);   // 割り込みピン入力
}

void loop() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];

    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      
      // モジュール1
      if (rxId == 0x056) {
        uint16_t voltage_raw = (rxBuf[3] << 8) | rxBuf[2];
        uint16_t current_raw = (rxBuf[1] << 8) | rxBuf[0];
        
        int32_t current_signed = (int32_t)current_raw - 0x8000;  // 符号変換
        float current_A = current_signed * 0.01119;
        float voltage_V = (voltage_raw * 4.8832) / 1000.0;

        Serial.print("[Module1] Voltage: ");
        Serial.print(voltage_V); Serial.print(" V  ");
        Serial.print("Current: ");
        Serial.print(current_A); Serial.println(" A");
      }

      // モジュール2
      else if (rxId == 0x076) {
        uint16_t voltage_raw = (rxBuf[3] << 8) | rxBuf[2];
        uint16_t current_raw = (rxBuf[1] << 8) | rxBuf[0];
        
        int32_t current_signed = (int32_t)current_raw - 0x8000;  // 符号変換
        float current_A = current_signed * 0.01119;
        float voltage_V = (voltage_raw * 4.8832) / 1000.0;

        Serial.print("[Module2] Voltage: ");
        Serial.print(voltage_V); Serial.print(" V  ");
        Serial.print("Current: ");
        Serial.print(current_A); Serial.println(" A");
      }
    }
  }
}
