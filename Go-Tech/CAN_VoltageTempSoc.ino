#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN  = 53;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(SPI_CS_PIN);

// 受信値保持
float module1_V = NAN, module1_I = NAN, module1_Tmax = NAN, module1_Tfet = NAN, module1_Tmin = NAN;
float module2_V = NAN, module2_I = NAN, module2_Tmax = NAN, module2_Tfet = NAN, module2_Tmin = NAN;
uint16_t soc_mAh = 0;
float soc_percent = NAN;

// 受信フラグ
bool receivedModule1 = false, receivedModule2 = false, receivedSOC = false;

// 定数
const float CURRENT_SCALE = 0.01119;
const float VOLTAGE_SCALE = 4.8832 / 1000.0;
const uint16_t SOC_MAX_mAh = 22000;

// ----------------- 値の変換関数 -----------------
float parseVoltage(uint16_t raw){
  if(raw == 0xFFFE || raw == 0xFFFF) return NAN;
  return raw * VOLTAGE_SCALE;
}

float parseCurrent(uint16_t raw){
  if(raw == 0xFFFE || raw == 0xFFFF) return NAN;
  int32_t val = (int32_t)raw - 0x8000;
  return val * CURRENT_SCALE;
}

float parseTemp(uint16_t raw){
  if(raw == 0xFFFE || raw == 0xFFFF) return NAN;
  int32_t val = (int32_t)raw - 0x8000;
  return val * 0.1;
}

// ----------------- CANメッセージ処理 -----------------
void processCANMessage(long unsigned int rxId, unsigned char* rxBuf){
  switch(rxId){
    case 0x056: // Module1電圧・電流
      module1_V = parseVoltage((rxBuf[3]<<8)|rxBuf[2]);
      module1_I = parseCurrent((rxBuf[1]<<8)|rxBuf[0]);
      receivedModule1 = true;
      break;

    case 0x076: // Module2電圧・電流
      module2_V = parseVoltage((rxBuf[3]<<8)|rxBuf[2]);
      module2_I = parseCurrent((rxBuf[1]<<8)|rxBuf[0]);
      receivedModule2 = true;
      break;

    case 0x055: // Module1温度
      module1_Tmax = parseTemp((rxBuf[1]<<8)|rxBuf[0]);
      module1_Tfet = parseTemp((rxBuf[3]<<8)|rxBuf[2]);
      module1_Tmin = parseTemp((rxBuf[5]<<8)|rxBuf[4]);
      receivedModule1 = true;
      break;

    case 0x075: // Module2温度
      module2_Tmax = parseTemp((rxBuf[1]<<8)|rxBuf[0]);
      module2_Tfet = parseTemp((rxBuf[3]<<8)|rxBuf[2]);
      module2_Tmin = parseTemp((rxBuf[5]<<8)|rxBuf[4]);
      receivedModule2 = true;
      break;

    case 0x053: // SOC
    {
      uint16_t val = (rxBuf[1]<<8)|rxBuf[0];
      soc_mAh = val;
      if(val == 0xFFFE || val == 0xFFFF){
        soc_percent = NAN;
        soc_mAh = 0;
      } else {
        soc_percent = ((float)val / SOC_MAX_mAh) * 100.0;
        if(soc_percent > 100.0) soc_percent = 100.0;
      }
      receivedSOC = true;
    }
    break;
  }
}

// ----------------- 全体表示関数 -----------------
void printAllData(){
  Serial.print("[Module1] V: "); Serial.print(isnan(module1_V)?"N/A":String(module1_V,2));
  Serial.print(" V  I: "); Serial.print(isnan(module1_I)?"N/A":String(module1_I,2));
  Serial.print(" A  Tmax: "); Serial.print(isnan(module1_Tmax)?"N/A":String(module1_Tmax,1));
  Serial.print(" C  Tfet: "); Serial.print(isnan(module1_Tfet)?"N/A":String(module1_Tfet,1));
  Serial.print(" C  Tmin: "); Serial.print(isnan(module1_Tmin)?"N/A":String(module1_Tmin,1));
  Serial.print(" C  | ");

  Serial.print("[Module2] V: "); Serial.print(isnan(module2_V)?"N/A":String(module2_V,2));
  Serial.print(" V  I: "); Serial.print(isnan(module2_I)?"N/A":String(module2_I,2));
  Serial.print(" A  Tmax: "); Serial.print(isnan(module2_Tmax)?"N/A":String(module2_Tmax,1));
  Serial.print(" C  Tfet: "); Serial.print(isnan(module2_Tfet)?"N/A":String(module2_Tfet,1));
  Serial.print(" C  Tmin: "); Serial.print(isnan(module2_Tmin)?"N/A":String(module2_Tmin,1));
  Serial.print(" C  | ");

  Serial.print("[SOC] "); Serial.print(soc_mAh);
  Serial.print(" mAh ("); Serial.print(isnan(soc_percent)?"-":String(soc_percent,1)); Serial.println("%)");
  Serial.println("------------------------------------------------");
}

// ----------------- Arduino setup/loop -----------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  if(CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK){
    Serial.println("CAN init OK!");
  } else {
    Serial.println("CAN init FAIL!");
    while(1);
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT);
}

void loop() {
  if(CAN.checkReceive() == CAN_MSGAVAIL){
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];

    if(CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK){
      processCANMessage(rxId, rxBuf);
    }
  }

  static unsigned long lastPrint = 0;
  if(millis() - lastPrint >= 100){
    lastPrint = millis();

    printAllData();

    // 未受信データはリセット
    if(!receivedModule1){ module1_V = NAN; module1_I = NAN; module1_Tmax = NAN; module1_Tfet = NAN; module1_Tmin = NAN; }
    if(!receivedModule2){ module2_V = NAN; module2_I = NAN; module2_Tmax = NAN; module2_Tfet = NAN; module2_Tmin = NAN; }
    if(!receivedSOC){ soc_mAh = 0; soc_percent = NAN; }

    receivedModule1 = false;
    receivedModule2 = false;
    receivedSOC = false;
  }
}
