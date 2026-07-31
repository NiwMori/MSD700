// cylinders.cpp — linear-actuator direction + feedback implementation.

#include "cylinders.h"

namespace msd_arduino
{

static bool pinValid_(uint8_t p)
{
  return p != 0xFF && p != 0;
}

static float clamp01_(float v)
{
  if (v < 0.0f) {return 0.0f;}
  if (v > 1.0f) {return 1.0f;}
  return v;
}

void Cylinders::begin(const CylinderPins & c1,
                      const CylinderPins & c2,
                      const CylinderPins & c3,
                      const CylinderCalibration & cal)
{
  c_[0] = c1;
  c_[1] = c2;
  c_[2] = c3;
  cal_  = cal;

  for (uint8_t i = 0; i < 3; ++i) {
    used_[i] = pinValid_(c_[i].fwd) && pinValid_(c_[i].rev);
    ema_[i]  = 0.0f;
    if (!used_[i]) {continue;}
    pinMode(c_[i].fwd, OUTPUT);
    pinMode(c_[i].rev, OUTPUT);
    digitalWrite(c_[i].fwd, LOW);
    digitalWrite(c_[i].rev, LOW);
    if (pinValid_(c_[i].end_in))  {pinMode(c_[i].end_in,  INPUT_PULLUP);}
    if (pinValid_(c_[i].end_out)) {pinMode(c_[i].end_out, INPUT_PULLUP);}
    // Seed EMA with one reading so the filter starts near the actual value.
    if (pinValid_(c_[i].feedback)) {
      ema_[i] = (float)readAvg_(c_[i].feedback);
    }
  }
}

void Cylinders::writePins_(const CylinderPins & p, int8_t state)
{
  if (state > 0) {
    digitalWrite(p.fwd, HIGH);
    digitalWrite(p.rev, LOW);
  } else if (state < 0) {
    digitalWrite(p.fwd, LOW);
    digitalWrite(p.rev, HIGH);
  } else {
    digitalWrite(p.fwd, LOW);
    digitalWrite(p.rev, LOW);
  }
}

void Cylinders::command(uint8_t index, float input)
{
  if (index >= 3 || !used_[index]) {return;}

  // Hysteresis: high threshold to engage, low threshold to release.
  // Prevents direction-line chatter when `input` jitters around 0.5
  // (SBUS stick at half-position with electrical noise, or arm_cmd values
  // near the boundary toggling each ROS publish cycle).
  static const float kOnThr  = 0.5f;
  static const float kOffThr = 0.3f;

  int8_t & st = state_[index];
  if (st == 0) {
    if (input >=  kOnThr)       { st =  1; }
    else if (input <= -kOnThr)  { st = -1; }
  } else if (st > 0) {
    if (input <  kOffThr)       { st = 0; }
  } else {  // st < 0
    if (input > -kOffThr)       { st = 0; }
  }

  writePins_(c_[index], st);
}

void Cylinders::stopAll()
{
  for (uint8_t i = 0; i < 3; ++i) {
    state_[i] = 0;
    if (!used_[i]) {continue;}
    digitalWrite(c_[i].fwd, LOW);
    digitalWrite(c_[i].rev, LOW);
  }
}

uint16_t Cylinders::readAvg_(uint8_t analog_pin) const
{
  uint32_t sum = 0;
  const uint8_t n = (cal_.n_samples > 0) ? cal_.n_samples : 1;
  for (uint8_t i = 0; i < n; ++i) {
    sum += (uint32_t)analogRead(analog_pin);
  }
  return (uint16_t)(sum / n);
}

CylinderFeedback Cylinders::read(uint8_t index)
{
  CylinderFeedback fb;
  if (index >= 3 || !used_[index]) {return fb;}
  const CylinderPins & p = c_[index];

  if (pinValid_(p.end_in))  {fb.end_in  = (digitalRead(p.end_in)  == LOW);}
  if (pinValid_(p.end_out)) {fb.end_out = (digitalRead(p.end_out) == LOW);}

  if (pinValid_(p.feedback)) {
    fb.raw_adc = readAvg_(p.feedback);
    // Exponential moving average.
    ema_[index] = cal_.ema_alpha * (float)fb.raw_adc
                + (1.0f - cal_.ema_alpha) * ema_[index];
    fb.raw_filtered = (uint16_t)(ema_[index] + 0.5f);
    // Map to mm.
    const int32_t range = (int32_t)cal_.raw_max - (int32_t)cal_.raw_min;
    if (range > 0) {
      float pos01 = (float)((int32_t)fb.raw_filtered - (int32_t)cal_.raw_min)
                   / (float)range;
      pos01 = clamp01_(pos01);
      fb.pos_mm = pos01 * cal_.stroke_mm;
    }
  }
  return fb;
}

}  // namespace msd_arduino
