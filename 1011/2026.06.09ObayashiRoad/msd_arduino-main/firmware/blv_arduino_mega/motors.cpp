// motors.cpp — differential-drive BLV motor controller implementation.

#include "motors.h"
#include <math.h>

namespace msd_arduino
{

void DiffDriveMotors::begin(const MotorPins & left, const MotorPins & right,
                            const DiffDriveGeometry & geom)
{
  left_  = left;
  right_ = right;
  geom_  = geom;

  const uint8_t outs[] = {
    left.fwd,  left.rev,  left.stop_mode,  left.m0,  left.mb_free,  left.vm,
    right.fwd, right.rev, right.stop_mode, right.m0, right.mb_free, right.vm,
  };
  for (uint8_t i = 0; i < sizeof(outs); ++i) {
    pinMode(outs[i], OUTPUT);
    digitalWrite(outs[i], LOW);
  }

  // BLV driver pin defaults (see BLV manual):
  //   STOP_MODE LOW  = instantaneous stop (瞬時停止)
  //   M0        HIGH = speed controlled by external PWM (VM) input
  //   MB_FREE   HIGH = electromagnetic brake released (normal running)
  // Brake is applied by the stop() function (MB_FREE → LOW).
  digitalWrite(left_.stop_mode,  LOW);
  digitalWrite(right_.stop_mode, LOW);
  digitalWrite(left_.m0,         HIGH);
  digitalWrite(right_.m0,        HIGH);
  digitalWrite(left_.mb_free,    HIGH);
  digitalWrite(right_.mb_free,   HIGH);

  analogWrite(left_.vm,  0);
  analogWrite(right_.vm, 0);
}

float DiffDriveMotors::vel_to_rpm_(float v) const
{
  // n [rpm] = 60 / (2π r) × ratio × v
  //         = 30 / π  × ratio / r × v
  return (30.0f / (float)M_PI) * (geom_.reduction_ratio / geom_.wheel_radius_m) * v;
}

void DiffDriveMotors::apply_side_(const MotorPins & p, float rpm)
{
  if (rpm > 0.1f) {
    digitalWrite(p.fwd, HIGH);
    digitalWrite(p.rev, LOW);
  } else if (rpm < -0.1f) {
    digitalWrite(p.fwd, LOW);
    digitalWrite(p.rev, HIGH);
  } else {
    digitalWrite(p.fwd, LOW);
    digitalWrite(p.rev, LOW);
    analogWrite(p.vm, 0);
    return;
  }

  float mag = fabsf(rpm);
  if (mag > geom_.max_rpm) {mag = geom_.max_rpm;}
  const float duty = (geom_.max_rpm > 0.1f) ? (mag / geom_.max_rpm) : 0.0f;
  const int pwm = (int)(duty * 255.0f + 0.5f);
  analogWrite(p.vm, pwm);
}

static float clamp_f(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

void DiffDriveMotors::drive(float linear_mps, float angular_rps)
{
  // Always release the electromagnetic brakes before applying drive.
  // stop(brake=true) latches MB_FREE LOW; drive() must undo that so
  // motors are not mechanically locked after an E-STOP clear.
  digitalWrite(left_.mb_free,  HIGH);
  digitalWrite(right_.mb_free, HIGH);

  // When moving backward, flip angular so turning feels intuitive
  // (car-like: "turn right" command → robot goes right even in reverse).
  const float eff_ang = (linear_mps < 0.0f) ? -angular_rps : angular_rps;
  const float v_l = linear_mps - geom_.wheel_distance_m * eff_ang;
  const float v_r = linear_mps + geom_.wheel_distance_m * eff_ang;
  float rpm_l = vel_to_rpm_(v_l);
  float rpm_r = vel_to_rpm_(v_r);
  // Right motor is physically mounted reversed — negate its RPM.
  rpm_r = -rpm_r;
  // Clamp to ±max_rpm so both PWM output and telemetry stay bounded.
  rpm_l = clamp_f(rpm_l, -geom_.max_rpm, geom_.max_rpm);
  rpm_r = clamp_f(rpm_r, -geom_.max_rpm, geom_.max_rpm);
  last_left_rpm_  = rpm_l;
  last_right_rpm_ = rpm_r;
  apply_side_(left_,  rpm_l);
  apply_side_(right_, rpm_r);
}

void DiffDriveMotors::stop(bool brake)
{
  analogWrite(left_.vm,  0);
  analogWrite(right_.vm, 0);
  digitalWrite(left_.fwd,  LOW);
  digitalWrite(left_.rev,  LOW);
  digitalWrite(right_.fwd, LOW);
  digitalWrite(right_.rev, LOW);
  // HIGH = brake released, LOW = hold brake
  digitalWrite(left_.mb_free,  brake ? LOW : HIGH);
  digitalWrite(right_.mb_free, brake ? LOW : HIGH);
  last_left_rpm_  = 0.0f;
  last_right_rpm_ = 0.0f;
}

}  // namespace msd_arduino
