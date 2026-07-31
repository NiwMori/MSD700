// cylinders.h — linear-actuator (LINAK) direction + feedback.
//
// Each cylinder has two direction lines (FWD / REV). The public API takes
// a normalized command `-1 .. +1`.  Per-cylinder state machine with
// hysteresis prevents twitching when the input jitters around the on
// threshold:
//   STOP  → EXTEND  when input ≥ +ON_THR  (default 0.5)
//   STOP  → RETRACT when input ≤ -ON_THR
//   EXTEND → STOP   when input <  +OFF_THR (default 0.3)
//   RETRACT → STOP  when input > -OFF_THR
//
// Feedback: two end-stop digital inputs (IN/OUT) and one analog pot line
// (stroke position). The class performs a multi-sample read + EMA filter
// and maps the result to mm using a calibration range.

#ifndef MSD_BLV_CYLINDERS_H_
#define MSD_BLV_CYLINDERS_H_

#include <Arduino.h>
#include <stdint.h>

namespace msd_arduino
{

struct CylinderPins
{
  uint8_t fwd;
  uint8_t rev;
  uint8_t end_in;
  uint8_t end_out;
  uint8_t feedback;   // analog
};

// Calibration constants for the analog potentiometer → mm mapping.
// Adjust after measuring the real actuator.
struct CylinderCalibration
{
  uint16_t raw_min    = 0;
  uint16_t raw_max    = 1017;
  float    stroke_mm  = 155.0f;  // 15.5 cm
  float    ema_alpha  = 0.25f;   // EMA smoothing (0..1)
  uint8_t  n_samples  = 16;      // moving-average window for analogRead
};

struct CylinderFeedback
{
  bool     end_in      = false;
  bool     end_out     = false;
  uint16_t raw_adc     = 0;      // 0..1023 raw
  uint16_t raw_filtered = 0;     // after EMA
  float    pos_mm      = 0.0f;   // mapped position [mm]
};

class Cylinders
{
public:
  // Up to 3 cylinders. Unused ones can be zero-initialized (fwd==0).
  void begin(const CylinderPins & c1,
             const CylinderPins & c2,
             const CylinderPins & c3,
             const CylinderCalibration & cal = CylinderCalibration());

  // Command a single cylinder by normalized input in [-1, +1].
  void command(uint8_t index, float input);

  // Stop all cylinders (FWD/REV low).
  void stopAll();

  // Read one cylinder's feedback (end-stops + filtered analog + mm).
  // Should be called periodically (e.g. 50 Hz) so the EMA converges.
  CylinderFeedback read(uint8_t index);

private:
  // Apply hardware pins for the given direction state.
  //   state >  0 → extend, state < 0 → retract, state == 0 → stop
  static void writePins_(const CylinderPins & p, int8_t state);
  uint16_t readAvg_(uint8_t analog_pin) const;

  CylinderPins        c_[3]{};
  bool                used_[3]  = {false, false, false};
  int8_t              state_[3] = {0, 0, 0};  // -1=retract, 0=stop, +1=extend
  uint32_t            state_since_ms_[3] = {0, 0, 0};
  CylinderCalibration cal_{};
  float               ema_[3]  = {0.0f, 0.0f, 0.0f};  // per-cylinder EMA state
};

}  // namespace msd_arduino

#endif  // MSD_BLV_CYLINDERS_H_
