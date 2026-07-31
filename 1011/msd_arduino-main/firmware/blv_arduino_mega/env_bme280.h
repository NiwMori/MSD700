// env_bme280.h — BME280 environment sensor (temperature, humidity, pressure).
//
// Uses the Adafruit BME280 library over I2C.
// Address: 0x76 (SDO → GND) or 0x77 (SDO → VCC).
// Shares the I2C bus with BNO055 (0x28) on SDA=D20, SCL=D21.
//
// Install:
//   arduino-cli lib install "Adafruit BME280 Library"

#ifndef MSD_BLV_ENV_BME280_H_
#define MSD_BLV_ENV_BME280_H_

#include <Arduino.h>
#include <Wire.h>
#include "protocol.h"

namespace msd_arduino
{

class EnvBme280
{
public:
  // Initialize the sensor. Returns false if the chip is not responding.
  bool begin(uint8_t i2c_addr = 0x76, TwoWire * wire = &Wire);

  bool isReady() const { return ready_; }

  // Fill the protocol EnvSensor struct with latest readings.
  // Returns false if sensor is not ready.
  bool read(protocol::EnvSensor & out);

private:
  bool ready_ = false;
};

}  // namespace msd_arduino

#endif  // MSD_BLV_ENV_BME280_H_
