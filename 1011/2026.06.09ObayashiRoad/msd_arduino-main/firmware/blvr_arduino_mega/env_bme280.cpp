// env_bme280.cpp — BME280 environment sensor implementation.

#include "env_bme280.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// File-scope BME280 object (library stores internal state).
static Adafruit_BME280 g_bme;

namespace msd_arduino
{

bool EnvBme280::begin(uint8_t i2c_addr, TwoWire * wire)
{
  ready_ = g_bme.begin(i2c_addr, wire);
  return ready_;
}

bool EnvBme280::read(protocol::EnvSensor & out)
{
  if (!ready_) { return false; }

  float temp     = g_bme.readTemperature();    // degrees C
  float humidity = g_bme.readHumidity();        // %
  float pressure = g_bme.readPressure();        // Pa

  // temperature in 0.01 deg C (25.12 C -> 2512)
  out.temperature_cdeg  = (int16_t)(temp * 100.0f + 0.5f);
  // humidity in 0.01 % (65.3% -> 6530)
  out.humidity_cpercent = (uint16_t)(humidity * 100.0f + 0.5f);
  // pressure in Pa (already in Pa from the library)
  out.pressure_pa       = (uint32_t)(pressure + 0.5f);

  return true;
}

}  // namespace msd_arduino
