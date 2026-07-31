// motors.h — differential-drive BLV motor controller.
//
// Wraps the two BLV drivers (left / right) and does the forward kinematics
// (linear velocity + angular velocity → two wheel RPMs), direction pin
// selection, and PWM output. Pins are passed in at begin() time so this
// module is reusable across robots.
//
// Input units:
//   linear_mps   : m/s
//   angular_rps  : rad/s  (not rpm — radians-per-second)
// Internal:
//   RPM is computed from wheel radius + reduction ratio, then mapped
//   linearly to 0..255 PWM through `max_rpm`.

#ifndef MSD_BLV_MOTORS_H_
#define MSD_BLV_MOTORS_H_

#include <Arduino.h>
#include <stdint.h>

namespace msd_arduino
{

struct MotorPins
{
  uint8_t fwd;
  uint8_t rev;
  uint8_t stop_mode;
  uint8_t m0;
  uint8_t mb_free;
  uint8_t vm;          // PWM pin
};

struct DiffDriveGeometry
{
  float wheel_radius_m    = 0.1105f;  // radius [m]
  float wheel_distance_m  = 0.600f;   // track width [m]
  float reduction_ratio   = 100.0f;   // gearbox
  float max_rpm           = 4000.0f;  // motor-shaft RPM limit (PWM full-scale)
};

class DiffDriveMotors
{
public:
  void begin(const MotorPins & left, const MotorPins & right,
             const DiffDriveGeometry & geom);

  // Apply a Twist-like command. Safe to call every loop().
  void drive(float linear_mps, float angular_rps);

  // Stop both wheels. `brake` = true → hold brake, false → free-spin.
  void stop(bool brake);

  // Getter for telemetry.
  float lastLeftRpm()  const {return last_left_rpm_;}
  float lastRightRpm() const {return last_right_rpm_;}

private:
  void apply_side_(const MotorPins & p, float rpm);
  float vel_to_rpm_(float v) const;

  MotorPins         left_{};
  MotorPins         right_{};
  DiffDriveGeometry geom_{};
  float last_left_rpm_  = 0.0f;
  float last_right_rpm_ = 0.0f;
};

}  // namespace msd_arduino

#endif  // MSD_BLV_MOTORS_H_
