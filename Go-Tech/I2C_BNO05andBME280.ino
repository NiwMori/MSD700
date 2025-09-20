#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>

// ------------------- I2Cアドレス -------------------
#define BNO_ADDRESS 0x28
#define BME_ADDRESS 0x76

// ------------------- センサオブジェクト -------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS);
Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  // ------------------- BNO055初期化 -------------------
  if (!bno.begin()) {
    Serial.println("BNO055 not detected at 0x28!");
    while (1);
  } else {
    Serial.println("BNO055 detected at 0x28");
  }

  // ------------------- BME280初期化 -------------------
  if (!bme.begin(BME_ADDRESS)) {
    Serial.println("BME280 not detected at 0x76!");
    while (1);
  } else {
    Serial.println("BME280 detected at 0x76");
  }
}

void loop() {
  // ------------------- BNO055読み取り -------------------
  sensors_event_t bno_event;
  bno.getEvent(&bno_event);
  Serial.print("BNO X: "); Serial.print(bno_event.orientation.x);
  Serial.print(" Y: "); Serial.print(bno_event.orientation.y);
  Serial.print(" Z: "); Serial.println(bno_event.orientation.z);

  // ------------------- BME280読み取り -------------------
  bme.takeForcedMeasurement();  // 強制測定モード
  Serial.print("BME Temp: "); Serial.print(bme.readTemperature()); Serial.println(" °C");
  Serial.print("BME Humidity: "); Serial.print(bme.readHumidity()); Serial.println(" %");
  Serial.print("BME Pressure: "); Serial.print(bme.readPressure() / 100.0F); Serial.println(" hPa");

  Serial.println("-----------------------------");
  delay(100);
}
