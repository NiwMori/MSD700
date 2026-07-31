// sbus_reader.h — S.BUS receiver using the Bolder Flight Systems SBUS library.
//
// Wraps `bfs::SbusRx` and adds:
//   - per-channel calibration (ch1-4 sticks have a different range from
//     ch5-16 switches / knobs on a typical Futaba / FrSky radio)
//   - center deadzone
//   - timeout detection
//   - normalized output in [-1, +1]
//
// Install the library:
//   arduino-cli lib install "Bolder Flight Systems SBUS"
//
// Wiring: SBUS receiver → 2SC1815 inverter → Serial1 RX (D19).
// Set `inverted = false` because the external transistor already inverts
// the signal. The bfs library on AVR ignores the inverted parameter
// anyway (only ESP32 supports software inversion).

#ifndef MSD_BLV_SBUS_READER_H_
#define MSD_BLV_SBUS_READER_H_

#include <Arduino.h>
#include <sbus.h>          // bfs::SbusRx, bfs::SbusData (library)

namespace msd_arduino
{

// Calibration ranges for the raw 11-bit SBUS values.
// CH1-4 (sticks) typically sweep a narrower range than CH5-16 (switches).
struct SbusCalibration
{
  int ch14_min  = 368;
  int ch14_mid  = 1024;
  int ch14_max  = 1680;
  int ch516_min = 144;
  int ch516_mid = 1024;
  int ch516_max = 1904;
  float deadzone_pct = 3.0f;   // percent of full-scale treated as zero
};

class SbusReader
{
public:
  static const uint8_t NUM_CHANNELS = 16;

  // Initialize. Call once in setup() BEFORE the main loop.
  //   serial   — HardwareSerial bound to the SBUS receiver (e.g. Serial1)
  //   inverted — true for raw Futaba-inverted signal, false otherwise
  //   cal      — calibration ranges
  //   timeout_ms — mark invalid after this many ms without a good frame
  void begin(HardwareSerial & serial, bool inverted = false,
             const SbusCalibration & cal = SbusCalibration(),
             uint32_t timeout_ms = 200);

  // Read any pending SBUS frames. Call every loop().
  void update();

  // --- State ---
  bool     is_valid()   const { return valid_; }
  bool     failsafe()   const { return failsafe_; }
  bool     frame_lost() const { return frame_lost_; }
  uint32_t rx_count()   const { return rx_count_; }

  // Normalized channel value in [-1, +1] with deadzone applied.
  float    channel(uint8_t i) const
  {
    return (i < NUM_CHANNELS) ? ch_norm_[i] : 0.0f;
  }

  // Raw 11-bit channel value as received (for debugging / tuning).
  int16_t  channelRaw(uint8_t i) const
  {
    return (i < NUM_CHANNELS) ? ch_raw_[i] : 0;
  }

  uint32_t last_valid_ms() const { return last_valid_ms_; }

private:
  float normalize_(int raw, int minV, int midV, int maxV, float dz) const;

  SbusCalibration  cal_{};
  uint32_t         timeout_ms_ = 200;

  int16_t  ch_raw_[NUM_CHANNELS]   = {0};
  float    ch_norm_[NUM_CHANNELS]  = {0};
  bool     valid_      = false;
  bool     failsafe_   = true;
  bool     frame_lost_ = true;
  uint32_t last_valid_ms_ = 0;
  uint32_t rx_count_   = 0;         // total frames received (debug)
};

}  // namespace msd_arduino

#endif  // MSD_BLV_SBUS_READER_H_
