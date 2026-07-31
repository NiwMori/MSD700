// sbus_reader.cpp — S.BUS receiver implementation using bfs::SbusRx.
//
// Key change from earlier version: bfs::SbusRx is allocated as a
// file-scope static (matching the user's working standalone sketch)
// instead of with `new` on the heap. AVR's 8 KB RAM can be finicky
// with heap allocation.

#include "sbus_reader.h"
#include <math.h>

// File-scope static bfs::SbusRx — avoids heap allocation on AVR.
// Only one SbusReader instance is expected in this firmware.
static bfs::SbusRx g_bfs_rx(&Serial1);
static bool g_bfs_active = false;

namespace msd_arduino
{

void SbusReader::begin(HardwareSerial & serial, bool inverted,
                       const SbusCalibration & cal,
                       uint32_t timeout_ms)
{
  cal_        = cal;
  timeout_ms_ = timeout_ms;
  valid_      = false;
  failsafe_   = true;
  frame_lost_ = true;
  rx_count_   = 0;

  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    ch_raw_[i]  = 0;
    ch_norm_[i] = 0.0f;
  }

  // Reinitialise the file-scope bfs::SbusRx with the correct serial port.
  // bfs::SbusRx stores HardwareSerial* internally; we reconstruct in-place
  // to bind it to the caller's chosen serial and inversion flag.
  g_bfs_rx = bfs::SbusRx(&serial, inverted);
  g_bfs_rx.Begin();
  g_bfs_active = true;
}

// Convert a raw SBUS value to [-1, +1] with asymmetric min/mid/max and deadzone.
float SbusReader::normalize_(int raw, int minV, int midV, int maxV, float dz) const
{
  // Clamp to calibrated range.
  if (raw < minV) { raw = minV; }
  if (raw > maxV) { raw = maxV; }

  // Map to [-100, +100] percent.
  float pct = 0.0f;
  if (raw < midV) {
    pct = -100.0f * (float)(midV - raw) / (float)(midV - minV);
  } else if (raw > midV) {
    pct = 100.0f * (float)(raw - midV) / (float)(maxV - midV);
  }

  // Apply deadzone.
  if (fabsf(pct) < dz) { pct = 0.0f; }

  // Clamp and convert to [-1, +1].
  if (pct >  100.0f) { pct =  100.0f; }
  if (pct < -100.0f) { pct = -100.0f; }
  return pct * 0.01f;
}

void SbusReader::update()
{
  if (!g_bfs_active) { return; }

  // bfs::SbusRx::Read() returns true when a new complete frame is available.
  if (g_bfs_rx.Read()) {
    bfs::SbusData d = g_bfs_rx.data();

    failsafe_   = d.failsafe;
    frame_lost_ = d.lost_frame;
    rx_count_++;

    for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
      ch_raw_[i] = d.ch[i];

      // CH1-4 (sticks) use one calibration range, CH5-16 use another.
      if (i < 4) {
        ch_norm_[i] = -normalize_(d.ch[i],
          cal_.ch14_min, cal_.ch14_mid, cal_.ch14_max, cal_.deadzone_pct);
      } else {
        ch_norm_[i] = normalize_(d.ch[i],
          cal_.ch516_min, cal_.ch516_mid, cal_.ch516_max, cal_.deadzone_pct);
      }
    }

    if (!d.lost_frame) {
      valid_ = true;
      last_valid_ms_ = millis();
    }
  }

  // Age out stale signal.
  if (valid_ && (millis() - last_valid_ms_) > timeout_ms_) {
    valid_    = false;
    failsafe_ = true;
  }
}

}  // namespace msd_arduino
