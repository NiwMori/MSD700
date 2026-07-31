// imu_bno055.h — BNO055 9-DoF orientation sensor wrapper.
//
// Uses the Adafruit_BNO055 library (Adafruit_Sensor + Adafruit_BNO055).
// In the Arduino IDE install via Library Manager:
//   - "Adafruit BNO055"  (this pulls in Adafruit_Sensor)
//
// Wiring: I2C on Arduino Mega pins SDA (D20) / SCL (D21).
// Default address 0x29 (ADR pin high).
//
// The sensor outputs fused Euler angles (3 float) and a quaternion (4
// float) directly. We pack them into the wire `protocol::Imu` struct using
// the fixed-point scales defined in the protocol (0.001 deg for euler,
// 1e-4 for quaternion).

#ifndef MSD_BLV_IMU_BNO055_H_
#define MSD_BLV_IMU_BNO055_H_

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "protocol.h"

namespace msd_arduino
{

class ImuBno055
{
public:
  ImuBno055();

  // Initialize the sensor. Returns false if the chip is not responding.
  // Safe to call again on retry.
  bool begin(uint8_t i2c_addr = 0x29, TwoWire * wire = &Wire);

  bool isReady() const {return ready_;}

  // Fill the packed `protocol::Imu` struct (euler + quaternion fields).
  // Returns false if the sensor is not ready.
  bool read(protocol::Imu & out);

private:
  Adafruit_BNO055 bno_;
  bool            ready_ = false;
};

}  // namespace msd_arduino

#endif  // MSD_BLV_IMU_BNO055_H_
