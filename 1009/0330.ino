#include <HX711_asukiaaa.h>
#include <avr/wdt.h>
int pinsDout[] = { 22,41};
const int numPins = sizeof(pinsDout) / sizeof(pinsDout[0]);
int pinSlk = 23;
HX711_asukiaaa::Reader reader(pinsDout, numPins, pinSlk);
//Load cell SC134 50kg
#define LOAD_CELL_RATED_VOLT_50KG 0.0019f
#define LOAD_CELL_RATED_GRAM_50KG 500000.0f
//Load cell SC134 100kg
#define LOAD_CELL_RATED_VOLT_100KG 0.001f
#define LOAD_CELL_RATED_GRAM_100KG 100000.0f
//---------------------------------------------------//
// Resistors for HX711 module AE-HX711-SIP
// https://akizukidenshi.com/catalog/g/gK-12370/
//---------------------------------------------------//
#define HX711_R1 20000.0
#define HX711_R2 8200.0
HX711_asukiaaa::Parser parser500kg(LOAD_CELL_RATED_VOLT_50KG,
                                  LOAD_CELL_RATED_GRAM_50KG, HX711_R1,
                                  HX711_R2);
// HX711_asukiaaa::Parser parser100kg(LOAD_CELL_RATED_VOLT_100KG,
//                                    LOAD_CELL_RATED_GRAM_100KG, HX711_R1, HX711_R2);
HX711_asukiaaa::Parser* parsers[numPins] = {
  &parser50kg,
  &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser100kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
  // &parser50kg,
};
// float offsetGrams[numPins];
// int numSensors = 41;
// int cols = 9;
// int rows[] = { 5, 4, 5, 4, 5, 4, 5, 4, 5 };
// int sensorValues[9][5];
// int cellSizeH = 270;
// int cellSizeW = 150;
// int counter = 0;
// int ecounter = 0;
// float preWeight = 0;

// float filteredCenterX = 0.0;
// float filteredCenterY = 0.0;
// const float alpha = 0.5;  // 平滑化係数

// int sensorIndex = 0;
// float sumX = 0, sumY = 0;
// float totalWeight = 0;

void setup() {
  Serial.begin(9600);
  reader.begin();
  for (int i = 0; i < reader.doutLen; ++i) {
    offsetGrams[i] = parsers[i]->parseToGram(reader.values[i]);
  }
}
void loop() {
  // if (Serial.available()) {
  //   String command = Serial.readStringUntil(':');
  //   command.trim();
  //   if (command == "r") {
  //     wdt_enable(WDTO_15MS);
  //     while (1)
  //       ;
  //   }
  // }
  auto readState = reader.read(1);
  if (readState == HX711_asukiaaa::ReadState::Success) {
    // sensorIndex = 0;
    // sumX = 0, sumY = 0;
    // totalWeight = 0;
    // ecounter = 0;
    // for (int x = 0; x < cols; x++) {
    //   for (int y = 0; y < rows[x]; y++) {
    //     if (sensorIndex < numSensors) {

    //       auto parser = *parsers[sensorIndex];
          float gram = parser.parseToGram(reader.values[1]) - offsetGrams[sensorIndex];
          
          // float value = (gram > 800) ? float(gram) : 0;

          // if (value > 10) {
          //   ecounter += 1;
          // }

          sensorValues[x][y] = value;
          int posX = x * cellSizeW;
          int posY = y * cellSizeH + (x % 2 == 1 ? cellSizeH / 2 : 0);
          sumX += posX * value / 100.0;
          sumY += posY * value / 100.0;
          totalWeight += value / 100.0;
          sensorIndex++;
        }
      }
    }






    if (abs(preWeight - totalWeight) < 10 || ecounter == 1) {
      counter += 1;
    } else {
      counter = 0;
    }
    preWeight = totalWeight;
    if (counter > 200) {
      wdt_enable(WDTO_15MS);
      while (1)
        ;
    }
    Serial.print("aaa");
    if (totalWeight > 0 && ecounter != 1) {
      float rawCenterX = sumX / totalWeight / 1350;
      float rawCenterY = sumY / totalWeight / 1350;

      filteredCenterX = alpha * rawCenterX + (1 - alpha) * filteredCenterX;
      filteredCenterY = alpha * rawCenterY + (1 - alpha) * filteredCenterY;

      int checkSUM = int(filteredCenterX * 100) + int(filteredCenterY * 100);
      Serial.print(filteredCenterX, 4);
      Serial.print(',');
      Serial.print(filteredCenterY, 4);
      Serial.print(',');
      Serial.print(totalWeight / 1000);
      Serial.print(',');
      Serial.print(checkSUM);
    } else {
      Serial.print("0.0000");
      Serial.print(',');
      Serial.print("0.0000");
      Serial.print(',');
      Serial.print(totalWeight / 1000);
      Serial.print(',');
      Serial.print('0');
    }
    Serial.print(',');
    Serial.print(counter);
    Serial.print(':');
  }
}