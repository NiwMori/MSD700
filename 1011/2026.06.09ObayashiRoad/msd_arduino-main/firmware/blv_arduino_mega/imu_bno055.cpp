// imu_bno055.cpp — BNO055 wrapper implementation.

#include "imu_bno055.h"

namespace msd_arduino
{

ImuBno055::ImuBno055()
: bno_(-1, 0x28, &Wire)
{
}

bool ImuBno055::begin(uint8_t i2c_addr, TwoWire * wire)
{
  bno_ = Adafruit_BNO055(-1, i2c_addr, wire);
  ready_ = bno_.begin();
  if (ready_) {
    bno_.setExtCrystalUse(true);
  }
  return ready_;
}

static int16_t clamp_i16_(float v)
{
  if (v >  32767.0f) {return  32767;}
  if (v < -32768.0f) {return -32768;}
  return (int16_t)v;
}

bool ImuBno055::read(protocol::Imu & out)
{
  if (!ready_) {return false;}

  imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion q    = bno_.getQuat();

  // Euler → 0.001 deg
  out.euler_x_mdeg = clamp_i16_((float)euler.x() * 1000.0f);
  out.euler_y_mdeg = clamp_i16_((float)euler.y() * 1000.0f);
  out.euler_z_mdeg = clamp_i16_((float)euler.z() * 1000.0f);

  // Quaternion → ±10000
  out.quat_w_1e4 = clamp_i16_((float)q.w() * 10000.0f);
  out.quat_x_1e4 = clamp_i16_((float)q.x() * 10000.0f);
  out.quat_y_1e4 = clamp_i16_((float)q.y() * 10000.0f);
  out.quat_z_1e4 = clamp_i16_((float)q.z() * 10000.0f);
  return true;
}

}  // namespace msd_arduino
