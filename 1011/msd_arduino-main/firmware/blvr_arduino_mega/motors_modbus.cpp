// motors_modbus.cpp — BLV-R Modbus RTU motor controller implementation.
//
// Based on working reference code and BLV-R Modbus register map:
//   Register 0x005A: Direct data operation (14 registers, 28 bytes)
//     [0-3]   Operation type  = 0x00000030 (continuous velocity)
//     [4-7]   Position        = 0x00000000 (unused in velocity mode)
//     [8-11]  Velocity        = signed int32 RPM
//     [12-15] Acceleration    = uint32 ms
//     [16-19] Deceleration    = uint32 ms
//     [20-23] Torque limit    = uint32 (x10 percent)
//     [24-27] Trigger         = 0x00000001
//   Register 0x007C: CMD2 lower word
//     bit 0: SON (servo on)
//     bit 5: STOP

#include "motors_modbus.h"
#include <math.h>

namespace msd_arduino
{

static const uint32_t kDiagNormalPeriodMs = 5000;
static const uint32_t kDiagFastPeriodMs = 500;
static const uint32_t kDiagFastHoldMs = 30000;
static const uint32_t kOdoReadPeriodMs = 3600000UL;
static const int32_t kDiagTorqueWarn01Pct = 1800;  // 180.0 %
static const int32_t kDiagTempWarn01C = 800;       // 80.0 C
static const int32_t kDiagCurrentWarn001A = 15000; // 15.000 A
static const int32_t kEncoderDeltaRejectTicks = 20000000;

static int32_t baud_to_blvr_code_(uint32_t baud)
{
  switch (baud) {
    case 9600:   return 0;
    case 19200:  return 1;
    case 38400:  return 2;
    case 57600:  return 3;
    case 115200: return 4;
    case 230400: return 5;
    default:     return 4;
  }
}

// Standard BLV-R baud rates probed during boot / relay-on auto-detect.
// Order matters: the most likely current state goes first to minimize
// the number of garbage frames sent at wrong bauds (which accumulate
// against the BLV-R communication-error counter, default threshold 3).
//   115200 — our migration target (re-boot with already-migrated driver)
//   230400 — BLV-R factory default
//   57600/38400 — manually set values via MEXE02
//   19200/9600  — rarely used but supported by BLV-R
static const uint32_t kProbeBauds[] = {
  115200, 230400, 57600, 38400, 19200, 9600,
};
static const uint8_t kProbeBaudsCount =
  (uint8_t)(sizeof(kProbeBauds) / sizeof(kProbeBauds[0]));

// =====================================================================
// CRC-16 Modbus (poly 0xA001, init 0xFFFF)
// =====================================================================
uint16_t ModbusDiffDrive::crc16_modbus_(const uint8_t * data, uint8_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 1) { crc = (crc >> 1) ^ 0xA001; }
      else         { crc >>= 1; }
    }
  }
  return crc;
}

void ModbusDiffDrive::put_i32_be_(uint8_t * p, int32_t v)
{
  p[0] = (uint8_t)((v >> 24) & 0xFF);
  p[1] = (uint8_t)((v >> 16) & 0xFF);
  p[2] = (uint8_t)((v >>  8) & 0xFF);
  p[3] = (uint8_t)( v        & 0xFF);
}

// =====================================================================
// RS485 direction control
// =====================================================================
void ModbusDiffDrive::rs485_tx_()
{
  digitalWrite(de_re_pin_, HIGH);
  delayMicroseconds(150);
}

void ModbusDiffDrive::rs485_rx_()
{
  // flush() returns after TXC (last stop bit clocked out).  Switch the
  // transceiver to RX immediately — its direction-change time is in the
  // ns range, while slave Tb2 (response start) can be as low as ~100 µs,
  // so any extra delay here clobbers the first bytes of the response.
  serial_->flush();
  digitalWrite(de_re_pin_, LOW);
}

void ModbusDiffDrive::drainRx_()
{
  while (serial_->available()) { serial_->read(); }
}

// =====================================================================
// Send raw Modbus frame (PDU + CRC)
// =====================================================================
void ModbusDiffDrive::sendRaw_(const uint8_t * pdu, uint8_t len_no_crc)
{
  uint8_t buf[64];
  if (len_no_crc + 2 > (uint8_t)sizeof(buf)) { return; }

  memcpy(buf, pdu, len_no_crc);
  uint16_t crc = crc16_modbus_(buf, len_no_crc);
  buf[len_no_crc]     = crc & 0xFF;          // CRC low byte first
  buf[len_no_crc + 1] = (crc >> 8) & 0xFF;

  // Silent interval ≥ C3.5 (≥2.5 ms at 230.4 kbps) before transmitting.
  delayMicroseconds(3000);
  drainRx_();

  rs485_tx_();
  serial_->write(buf, len_no_crc + 2);
  serial_->flush();
  rs485_rx_();

  // Wait long enough for the full FC 0x10 response to arrive *and* the
  // C3.5 silent interval that follows. At 230.4 kbps:
  //   Tb2 (slave processing)           ≈ 3.0 ms (default)
  //   response TX time (8 bytes x11b)  ≈ 0.4 ms
  //   C3.5 silent interval             ≥ 2.5 ms
  // Total ≈ 6 ms. Using delay(3) earlier left the tail of the response in
  // the RX buffer, which got re-parsed as the first bytes of the NEXT
  // read, corrupting every FC 0x03 response.
  delay(7);
  drainRx_();
}

// =====================================================================
// Modbus commands
// =====================================================================
void ModbusDiffDrive::sendServoOn_(uint8_t slave_id)
{
  // FC 0x10, Register 0x007C, 2 registers (4 bytes): upper=0x0000, lower=SON
  uint8_t f[] = {
    slave_id, 0x10,
    0x00, 0x7C,         // start register
    0x00, 0x02,         // register count
    0x04,               // byte count
    0x00, 0x00,         // upper word (unused)
    0x00, 0x01,         // lower word: SON = 1
  };
  sendRaw_(f, sizeof(f));
}

void ModbusDiffDrive::sendStopPulse_(uint8_t slave_id)
{
  // Set STOP bit (bit 5) + keep SON
  uint8_t son = 0x01;
  uint16_t lower = son | (1u << 5);  // SON + STOP
  uint8_t f1[] = {
    slave_id, 0x10,
    0x00, 0x7C, 0x00, 0x02, 0x04,
    0x00, 0x00,
    (uint8_t)((lower >> 8) & 0xFF), (uint8_t)(lower & 0xFF),
  };
  sendRaw_(f1, sizeof(f1));
  delay(5);

  // Clear STOP bit, keep SON
  uint8_t f2[] = {
    slave_id, 0x10,
    0x00, 0x7C, 0x00, 0x02, 0x04,
    0x00, 0x00,
    0x00, son,
  };
  sendRaw_(f2, sizeof(f2));
}

void ModbusDiffDrive::writeParam32_(uint8_t slave_id, uint16_t addr,
                                     int32_t value)
{
  // FC 0x10, 2 registers (4 bytes), high-word-first big-endian.
  uint8_t f[13];
  f[0] = slave_id;
  f[1] = 0x10;
  f[2] = (uint8_t)((addr >> 8) & 0xFF);
  f[3] = (uint8_t)( addr       & 0xFF);
  f[4] = 0x00;
  f[5] = 0x02;
  f[6] = 0x04;
  put_i32_be_(&f[7], value);
  sendRaw_(f, sizeof(f));
}

void ModbusDiffDrive::sendVelocity_(uint8_t slave_id, int32_t rpm,
                                     int32_t acc, int32_t dec, int32_t torque)
{
  uint8_t f[35];
  uint8_t i = 0;

  f[i++] = slave_id;
  f[i++] = 0x10;         // FC: Write Multiple Registers
  f[i++] = 0x00;
  f[i++] = 0x5A;         // Start register: 0x005A
  f[i++] = 0x00;
  f[i++] = 0x0E;         // 14 registers
  f[i++] = 0x1C;         // 28 bytes

  uint8_t * d = &f[i];
  put_i32_be_(d +  0, 0x00000030);   // operation type: continuous velocity
  put_i32_be_(d +  4, 0);            // position (unused)
  put_i32_be_(d +  8, rpm);          // velocity [RPM]
  put_i32_be_(d + 12, acc);          // acceleration [ms]
  put_i32_be_(d + 16, dec);          // deceleration [ms]
  put_i32_be_(d + 20, torque);       // torque limit [x10 %]
  put_i32_be_(d + 24, 1);            // trigger

  sendRaw_(f, sizeof(f));
}

// =====================================================================
// FC 0x03 — Read Holding Registers
// =====================================================================
bool ModbusDiffDrive::readHoldingRegisters_(uint8_t slave_id,
                                             uint16_t start_addr,
                                             uint16_t count,
                                             uint8_t * out_data,
                                             uint8_t out_size)
{
  if (!serial_ || count == 0 || out_size < count * 2) { return false; }

  // Build request: [ID][0x03][AddrHi][AddrLo][CntHi][CntLo][CrcLo][CrcHi]
  uint8_t tx[8];
  tx[0] = slave_id;
  tx[1] = 0x03;
  tx[2] = (uint8_t)((start_addr >> 8) & 0xFF);
  tx[3] = (uint8_t)( start_addr        & 0xFF);
  tx[4] = (uint8_t)((count >> 8) & 0xFF);
  tx[5] = (uint8_t)( count        & 0xFF);
  uint16_t crc = crc16_modbus_(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)((crc >> 8) & 0xFF);

  // Silent interval before sending (BLV-R requires ≥2.5 ms at 230.4 kbps).
  delayMicroseconds(3000);
  drainRx_();

  rs485_tx_();
  serial_->write(tx, sizeof(tx));
  serial_->flush();
  rs485_rx_();

  // No post-switch drain: rs485_rx_() now switches direction immediately
  // after flush().  Slave Tb2 (~100-200 µs) gives us enough time to start
  // polling before the response arrives.  Any spurious 0x00/0xFF latched
  // by a transceiver glitch will appear as a CRC mismatch, not a header
  // corruption — preferable to eating real data.

  // Expected normal response: ID + FC + byteCount + 2*count data + 2 CRC.
  // Exception response is shorter: ID + (FC|0x80) + code + 2 CRC.
  const uint8_t expected = 5 + count * 2;
  const uint8_t exception_len = 5;
  uint8_t rx[64];
  if (expected > (uint8_t)sizeof(rx)) { return false; }

  // 15 ms timeout: at 115200 bps a 9-byte response is ~0.9 ms TX + ~3 ms
  // slave processing → typical response < 5 ms.  Shorter timeout prevents
  // the main loop from stalling for ~90 ms per cycle when BLV-R is dead
  // (3 reads × 30 ms = 270 ms vs 3 × 15 ms = 45 ms).
  const uint32_t t_start = millis();
  uint32_t last_rx_ms = t_start;
  uint8_t rx_len = 0;
  while ((millis() - t_start) < 15 && rx_len < (uint8_t)sizeof(rx)) {
    bool got = false;
    while (serial_->available() && rx_len < (uint8_t)sizeof(rx)) {
      rx[rx_len++] = (uint8_t)serial_->read();
      last_rx_ms = millis();
      got = true;
    }

    // Once a complete response plus the following C3.5-ish gap has passed,
    // stop collecting.  This still leaves room to recover if one stale/noisy
    // byte arrived before the real Modbus frame.
    if (rx_len >= expected && !got && (millis() - last_rx_ms) >= 3) {
      break;
    }
  }

  uint8_t frame_start = 0xFF;
  uint8_t frame_len = expected;
  if (rx_len >= exception_len) {
    for (uint8_t i = 0; i <= rx_len - exception_len; ++i) {
      if (rx[i] == slave_id && (rx[i + 1] == 0x03 || rx[i + 1] == 0x83)) {
        frame_start = i;
        frame_len = (rx[i + 1] == 0x83) ? exception_len : expected;
        break;
      }
    }
  }
  const uint8_t * frame =
    (frame_start == 0xFF) ? rx : (const uint8_t *)&rx[frame_start];

  const char * reason = nullptr;
  if (rx_len < exception_len) {
    reason = "TO";   // timeout
  } else if (frame_start == 0xFF || frame[0] != slave_id) {
    reason = "ID";
  } else if (rx_len - frame_start < frame_len) {
    reason = "TO";
  } else if (frame[1] == 0x83) {
    uint16_t rx_crc = (uint16_t)frame[3] | ((uint16_t)frame[4] << 8);
    uint16_t calc   = crc16_modbus_(frame, 3);
    reason = (rx_crc == calc) ? "EX" : "CRC";
  } else if (frame[1] != 0x03) {
    reason = "FC";
  } else if (frame[2] != count * 2) {
    reason = "BC";   // byte count
  } else {
    uint16_t rx_crc = (uint16_t)frame[expected - 2] |
                      ((uint16_t)frame[expected - 1] << 8);
    uint16_t calc   = crc16_modbus_(frame, expected - 2);
    if (rx_crc != calc) { reason = "CRC"; }
  }

  if (reason) {
    ++err_count_;
    // Rate-limited dump of failed response (≤1/sec) to identify root cause.
    if (dbg_) {
      static uint32_t s_next_err_dump_ms = 0;
      uint32_t now = millis();
      if ((int32_t)(now - s_next_err_dump_ms) >= 0) {
        s_next_err_dump_ms = now + 1000;
        dbg_->print(F("MB ERR "));
        dbg_->print(reason);
        dbg_->print(F(" id=0x"));
        dbg_->print(slave_id, HEX);
        dbg_->print(F(" addr=0x"));
        dbg_->print(start_addr, HEX);
        dbg_->print(F(" tx:"));
        for (uint8_t i = 0; i < (uint8_t)sizeof(tx); ++i) {
          dbg_->print(' ');
          if (tx[i] < 0x10) { dbg_->print('0'); }
          dbg_->print(tx[i], HEX);
        }
        if (frame_start != 0xFF && frame[1] == 0x83 &&
            rx_len - frame_start >= exception_len) {
          dbg_->print(F(" ex=0x"));
          dbg_->print(frame[2], HEX);
        }
        dbg_->print(F(" got["));
        dbg_->print(rx_len);
        dbg_->print(F("]:"));
        for (uint8_t i = 0; i < rx_len; ++i) {
          dbg_->print(' ');
          if (rx[i] < 0x10) { dbg_->print('0'); }
          dbg_->print(rx[i], HEX);
        }
        dbg_->println();
      }
    }
    return false;
  }

  memcpy(out_data, &frame[3], count * 2);
  return true;
}

bool ModbusDiffDrive::readI32_(uint8_t slave_id, uint16_t start_addr,
                                int32_t & out)
{
  uint8_t d[4];
  if (!readHoldingRegisters_(slave_id, start_addr, 2, d, sizeof(d))) {
    return false;
  }
  // BLV-R byte order: high word first, big-endian within word.
  out = ((int32_t)d[0] << 24) | ((int32_t)d[1] << 16) |
        ((int32_t)d[2] <<  8) |  (int32_t)d[3];
  return true;
}

uint32_t ModbusDiffDrive::probeAllBauds_()
{
  for (uint8_t i = 0; i < kProbeBaudsCount; ++i) {
    if (probeBaud_(kProbeBauds[i], 1)) {
      return kProbeBauds[i];
    }
  }
  return 0;
}

#if MSD_USE_ID_SHARE_MODE
// =====================================================================
// ID Share Mode — single-frame write/read to both drivers at once
// =====================================================================
void ModbusDiffDrive::sendShareWrite_(int32_t vel_l, int32_t vel_r,
                                       int32_t accel, int32_t decel,
                                       int32_t trigger)
{
  // Frame layout:
  //   [GID] [FC=10] [start=0x0000 (2B)] [reg_count (2B)] [byte_count (1B)]
  //   [slave1 16B: vel/accel/decel/trigger] [slave2 16B] [CRC 2B]
  uint8_t f[7 + blvr_share::WRITE_BYTE_COUNT];
  uint8_t i = 0;
  f[i++] = blvr_share::GLOBAL_ID;
  f[i++] = 0x10;
  f[i++] = 0x00;
  f[i++] = 0x00;                                  // start IDshare addr = 0
  f[i++] = 0x00;
  f[i++] = blvr_share::WRITE_REG_COUNT;           // 16 regs
  f[i++] = blvr_share::WRITE_BYTE_COUNT;          // 32 bytes

  // Slave 1 (Local ID 1, left) data
  put_i32_be_(&f[i], vel_l);   i += 4;
  put_i32_be_(&f[i], accel);   i += 4;
  put_i32_be_(&f[i], decel);   i += 4;
  put_i32_be_(&f[i], trigger); i += 4;
  // Slave 2 (Local ID 2, right) data
  put_i32_be_(&f[i], vel_r);   i += 4;
  put_i32_be_(&f[i], accel);   i += 4;
  put_i32_be_(&f[i], decel);   i += 4;
  put_i32_be_(&f[i], trigger); i += 4;

  sendRaw_(f, i);
}

bool ModbusDiffDrive::readShareData_()
{
  if (!serial_) { return false; }

  // Build query: [GID] [FC=03] [start=0x0000] [count (2B)] [CRC]
  uint8_t tx[8];
  tx[0] = blvr_share::GLOBAL_ID;
  tx[1] = 0x03;
  tx[2] = 0x00;
  tx[3] = 0x00;
  tx[4] = 0x00;
  tx[5] = blvr_share::READ_REG_COUNT;             // 26 regs
  uint16_t crc = crc16_modbus_(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF);
  tx[7] = (uint8_t)((crc >> 8) & 0xFF);

  delayMicroseconds(3000);
  drainRx_();
  rs485_tx_();
  serial_->write(tx, sizeof(tx));
  serial_->flush();
  rs485_rx_();

  // Fast odometry reads only slot0(rpm) + slot1(count).  Some BLV-R ID Share
  // responses omit the final inter-slave check while byteCount still reports
  // the full two-check length.  Therefore read until the bus goes idle, then
  // scan for a CRC-valid Modbus frame instead of trusting one fixed length.
  const uint8_t expected_full = 3 + blvr_share::READ_BYTE_COUNT + 2;
  const uint8_t expected_compact =
    3 + (blvr_share::READ_BYTE_COUNT - 2) + 2;
  uint8_t rx[96];
  if (expected_full > (uint8_t)sizeof(rx)) { ++err_count_; return false; }

  // 15 ms timeout: at 115200 bps the response is ~2.4 ms TX + ~3 ms
  // slave processing.  Keep the timeout short so failed reads do not stall
  // SBUS/control handling.
  const uint32_t t_start = millis();
  uint8_t rx_len = 0;
  uint32_t last_rx_ms = t_start;
  while ((millis() - t_start) < 15 && rx_len < (uint8_t)sizeof(rx)) {
    bool got = false;
    while (serial_->available() && rx_len < (uint8_t)sizeof(rx)) {
      rx[rx_len++] = (uint8_t)serial_->read();
      last_rx_ms = millis();
      got = true;
    }
    if (rx_len >= expected_compact && !got && (millis() - last_rx_ms) >= 3) {
      break;
    }
  }

  // Validate by finding a CRC-valid frame.  This also tolerates a stray 0x00
  // sampled before the real 0x0F header.
  const char * reason = nullptr;
  uint8_t frame_start = 0;
  uint8_t frame_len = 0;
  if (rx_len < expected_compact) {
    reason = "TO";
  } else {
    for (uint8_t s = 0; s + expected_compact <= rx_len; ++s) {
      if (rx[s] != blvr_share::GLOBAL_ID) { continue; }
      if ((rx[s + 1] & 0x7F) != 0x03) { continue; }
      if (rx[s + 1] & 0x80) { reason = "EX"; continue; }
      if (rx[s + 2] != blvr_share::READ_BYTE_COUNT) { reason = "BC"; continue; }

      const uint8_t candidates[2] = { expected_compact, expected_full };
      for (uint8_t ci = 0; ci < 2; ++ci) {
        const uint8_t len = candidates[ci];
        if (s + len > rx_len) { continue; }
        uint16_t rx_crc = (uint16_t)rx[s + len - 2] |
                          ((uint16_t)rx[s + len - 1] << 8);
        uint16_t calc = crc16_modbus_(&rx[s], len - 2);
        if (rx_crc == calc) {
          frame_start = s;
          frame_len = len;
          reason = nullptr;
          break;
        }
      }
      if (frame_len) { break; }
    }
    if (!frame_len && !reason) { reason = "CRC"; }
  }

  if (reason) {
    ++err_count_;
    if (dbg_) {
      static uint32_t s_next_err_dump_ms = 0;
      uint32_t now = millis();
      if ((int32_t)(now - s_next_err_dump_ms) >= 0) {
        s_next_err_dump_ms = now + 1000;
        dbg_->print(F("IDS ERR "));
        dbg_->print(reason);
        dbg_->print(F(" got["));
        dbg_->print(rx_len);
        dbg_->print(F("]:"));
        const uint8_t dump_n = (rx_len < 16) ? rx_len : 16;
        for (uint8_t k = 0; k < dump_n; ++k) {
          dbg_->print(' ');
          if (rx[k] < 0x10) { dbg_->print('0'); }
          dbg_->print(rx[k], HEX);
        }
        dbg_->println();
      }
    }
    return false;
  }

  const uint8_t * frame = &rx[frame_start];

  // Parse the payload:
  //   slave1 = 8B data + 2B check, slave2 = 8B data (+ optional check)
  // Slot order matches the first two MEXE02 Share Read data entries:
  //   [0..3] Slot 0 検出速度          (r/min)
  //   [4..7] Slot 1 検出32bitカウンタ (step)
  auto rd_i32 = [](const uint8_t * p) -> int32_t {
    return ((int32_t)p[0] << 24) | ((int32_t)p[1] << 16) |
           ((int32_t)p[2] <<  8) |  (int32_t)p[3];
  };

  const uint8_t * s1 = &frame[3];
  const uint8_t * s2_one_check = &frame[3 + blvr_share::READ_SLOTS * 4 + 1];
  const uint8_t * s2_two_check = &frame[3 + blvr_share::READ_BYTES_PER_SLAVE];

  fb_rpm_l_    =  rd_i32(s1 +  0);
  fb_count_l_  =  rd_i32(s1 +  4);

  // The BLV-R ID Share response on this machine sometimes places one
  // inter-slave check byte after slave1, and sometimes two.  Try both right
  // block offsets and keep the count closest to the previous accepted sample.
  const int32_t r_count_a = rd_i32(s2_one_check + 4);
  const int32_t r_count_b = rd_i32(s2_two_check + 4);
  const int32_t prev_raw_r = -fb_count_r_;
  const int32_t da = (r_count_a >= prev_raw_r) ? (r_count_a - prev_raw_r)
                                               : (prev_raw_r - r_count_a);
  const int32_t db = (r_count_b >= prev_raw_r) ? (r_count_b - prev_raw_r)
                                               : (prev_raw_r - r_count_b);
  const bool use_one_check =
    (fb_count_pair_seq_ == 0) ? (labs(r_count_a) <= labs(r_count_b))
                              : (da <= db);
  const uint8_t * s2 = use_one_check ? s2_one_check : s2_two_check;

  // Right motor: invert rotation-direction quantities (rpm/count/pos/odo)
  // to robot frame (right physically reversed).  Torque/temp/alarm/current
  // are scalar magnitudes — keep as-is.
  fb_rpm_r_    = -rd_i32(s2 +  0);
  fb_count_r_  = -(use_one_check ? r_count_a : r_count_b);
  ++fb_count_pair_seq_;

  return true;
}
#endif  // MSD_USE_ID_SHARE_MODE

bool ModbusDiffDrive::readEncoderPairDirect_()
{
  int32_t left = 0;
  int32_t right_raw = 0;
  if (!readI32_(cfg_.left_slave_id, 0x0120, left)) {
    return false;
  }
  if (!readI32_(cfg_.right_slave_id, 0x0120, right_raw)) {
    return false;
  }

  const int32_t right = -right_raw;
  if (fb_count_pair_seq_ != 0) {
    const int32_t dl = left - fb_count_l_;
    const int32_t dr = right - fb_count_r_;
    if (labs(dl) > kEncoderDeltaRejectTicks ||
        labs(dr) > kEncoderDeltaRejectTicks) {
      ++err_count_;
      if (dbg_) {
        static uint32_t s_next_enc_drop_ms = 0;
        const uint32_t now = millis();
        if ((int32_t)(now - s_next_enc_drop_ms) >= 0) {
          s_next_enc_drop_ms = now + 1000;
          dbg_->print(F("ENC PAIR DROP prev=["));
          dbg_->print(fb_count_l_);
          dbg_->print(',');
          dbg_->print(fb_count_r_);
          dbg_->print(F("] new=["));
          dbg_->print(left);
          dbg_->print(',');
          dbg_->print(right);
          dbg_->println(']');
        }
      }
      return false;
    }
  }

  fb_count_l_ = left;
  fb_count_r_ = right;
  ++fb_count_pair_seq_;
  return true;
}

bool ModbusDiffDrive::diagnosticsFastNeeded_() const
{
  if (fb_alarm_l_ != 0 || fb_alarm_r_ != 0) { return true; }
  if (fb_torque_l_ > kDiagTorqueWarn01Pct || fb_torque_r_ > kDiagTorqueWarn01Pct) { return true; }
  if (fb_torque_l_ < -kDiagTorqueWarn01Pct || fb_torque_r_ < -kDiagTorqueWarn01Pct) { return true; }
  if (fb_tdrv_l_ > kDiagTempWarn01C || fb_tdrv_r_ > kDiagTempWarn01C) { return true; }
  if (fb_tmot_l_ > kDiagTempWarn01C || fb_tmot_r_ > kDiagTempWarn01C) { return true; }
  if (fb_imain_l_ > kDiagCurrentWarn001A || fb_imain_r_ > kDiagCurrentWarn001A) { return true; }
  if (fb_imain_l_ < -kDiagCurrentWarn001A || fb_imain_r_ < -kDiagCurrentWarn001A) { return true; }
  return false;
}

void ModbusDiffDrive::updateSlowDiagnostics_(uint32_t now)
{
  if (!serial_ || !init_done_) { return; }

  if (err_count_ != diag_last_err_count_) {
    diag_last_err_count_ = err_count_;
    diag_fast_until_ms_ = now + kDiagFastHoldMs;
  }
  if (diagnosticsFastNeeded_()) {
    diag_fast_until_ms_ = now + kDiagFastHoldMs;
  }

  const bool fast = ((int32_t)(diag_fast_until_ms_ - now) > 0);
  const uint32_t period = fast ? kDiagFastPeriodMs : kDiagNormalPeriodMs;
  if ((int32_t)(now - diag_next_read_ms_) < 0) { return; }
  diag_next_read_ms_ = now + period;

  int32_t v = 0;
  switch (diag_phase_) {
    case 0: if (readI32_(cfg_.left_slave_id,  0x0080, v)) { fb_alarm_l_ = v; } break;
    case 1: if (readI32_(cfg_.right_slave_id, 0x0080, v)) { fb_alarm_r_ = v; } break;
    case 2: if (readI32_(cfg_.left_slave_id,  0x00D6, v)) { fb_torque_l_ = v; } break;
    case 3: if (readI32_(cfg_.right_slave_id, 0x00D6, v)) { fb_torque_r_ = v; } break;
    case 4: if (readI32_(cfg_.left_slave_id,  0x00F8, v)) { fb_tdrv_l_ = v; } break;
    case 5: if (readI32_(cfg_.right_slave_id, 0x00F8, v)) { fb_tdrv_r_ = v; } break;
    case 6: if (readI32_(cfg_.left_slave_id,  0x00FA, v)) { fb_tmot_l_ = v; } break;
    case 7: if (readI32_(cfg_.right_slave_id, 0x00FA, v)) { fb_tmot_r_ = v; } break;
    case 8: if (readI32_(cfg_.left_slave_id,  0x0136, v)) { fb_imain_l_ = v; } break;
    case 9: if (readI32_(cfg_.right_slave_id, 0x0136, v)) { fb_imain_r_ = v; } break;
  }
  ++diag_phase_;
  if (diag_phase_ >= 10) { diag_phase_ = 0; }

  if ((int32_t)(now - diag_next_odo_ms_) >= 0) {
    diag_next_odo_ms_ = now + kOdoReadPeriodMs;
    if (readI32_(cfg_.left_slave_id, 0x00FC, v)) { fb_odo_l_ = v; }
    if (readI32_(cfg_.right_slave_id, 0x00FC, v)) { fb_odo_r_ = v; }
  }
}

bool ModbusDiffDrive::probeBaud_(uint32_t baud, uint8_t attempts)
{
  if (!serial_) { return false; }
  serial_->end();
  delay(50);
  serial_->begin(baud, SERIAL_8E1);
  delay(100);
  drainRx_();
  // Suppress error logging during the probe — we expect failures here.
  Stream * saved_dbg = dbg_;
  dbg_ = nullptr;
  bool ok = false;
  int32_t v;
  for (uint8_t i = 0; i < attempts && !ok; ++i) {
    // Read alarm code (0x0080) — small, always available, no side effect.
    if (readI32_(cfg_.left_slave_id, 0x0080, v)) { ok = true; break; }
    delay(20);
  }
  dbg_ = saved_dbg;
  if (dbg_) {
    dbg_->print(F("MB: probe @"));
    dbg_->print(baud);
    dbg_->println(ok ? F(" OK") : F(" FAIL"));
  }
  // Reset the error counter — probe failures are not real comm errors.
  err_count_ = 0;
  return ok;
}

// =====================================================================
// Kinematics (same as GPIO variant)
// =====================================================================
float ModbusDiffDrive::vel_to_rpm_(float v) const
{
  return (30.0f / (float)M_PI) *
         (cfg_.reduction_ratio / cfg_.wheel_radius_m) * v;
}

static float clamp_f(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

// =====================================================================
// Public API
// =====================================================================
void ModbusDiffDrive::begin(HardwareSerial & serial, uint8_t de_re_pin,
                            const ModbusDriveConfig & cfg)
{
  serial_    = &serial;
  de_re_pin_ = de_re_pin;
  cfg_       = cfg;

  pinMode(de_re_pin_, OUTPUT);
  digitalWrite(de_re_pin_, LOW);

  // Open at the target baud first.  BLV-R defaults: 8 data, even parity,
  // 1 stop bit (SERIAL_8E1).  Using SERIAL_8N1 triggers comm-error
  // alarm 0x84 on the driver side.
  serial_->begin(cfg_.baud, SERIAL_8E1);

  // USB-reset mitigation: when USB is plugged the Mega auto-resets on DTR.
  // Hold the bus idle ≥500 ms so any TX glitch is past before the first
  // Modbus frame, otherwise BLV-R latches alarm 0x84.
  delay(500);
  drainRx_();

  if (dbg_) {
    dbg_->print(F("MB: begin baud="));
    dbg_->print(cfg_.baud);
    dbg_->print(F(" DE_RE=D"));
    dbg_->println(de_re_pin_);
  }

  // Probe-then-migrate: never write to the driver unless we have positively
  // identified its current baud.  Walk every standard BLV-R baud rate
  // (115200/230400/57600/38400/19200/9600) so a driver in any state is
  // detected, including ones manually set via MEXE02 to non-default values.
  const uint32_t found = probeAllBauds_();
  if (found == 0) {
    if (dbg_) {
      dbg_->println(F("MB: WARN no BLV-R response at ANY standard baud."));
      dbg_->println(F("MB:   Check 1) wiring (A/B/GND/120Ω terminator),"));
      dbg_->println(F("MB:         2) driver power (24 V at V+/0V),"));
      dbg_->println(F("MB:         3) DIP slave-ID switch (1=L, 2=R),"));
      dbg_->println(F("MB:         4) front-panel LED (red 8-blink = alarm 0x84,"));
      dbg_->println(F("MB:            requires physical main-power cycle)."));
      dbg_->println(F("MB:   If LED is green and bus still silent, run MEXE02"));
      dbg_->println(F("MB:   factory reset via the BLV-R PC port."));
    }
    serial_->end();
    delay(50);
    serial_->begin(cfg_.baud, SERIAL_8E1);
    delay(100);
    drainRx_();
  } else if (found == cfg_.baud) {
    if (dbg_) {
      dbg_->print(F("MB: BLV-R at target baud "));
      dbg_->println(cfg_.baud);
    }
  } else {
    if (dbg_) {
      dbg_->print(F("MB: BLV-R found at "));
      dbg_->print(found);
      dbg_->print(F(", migrating to "));
      dbg_->print(cfg_.baud);
      dbg_->println(F(" (NV-save, power-cycle drivers to apply)"));
    }
    // Serial2 is now at `found` baud — write the new comm parameters,
    // NV-save, then reopen at cfg_.baud.  Reflection timing per BLV-R
    // manual HP-5141J §13-3: baud = D (effective on driver power cycle).
    const int32_t baud_code = baud_to_blvr_code_(cfg_.baud);
    writeParam32_(cfg_.left_slave_id,  0x1382, baud_code);  // baud
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x1382, baud_code);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x1386, 1);  // even parity (default)
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x1386, 1);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x1388, 0);  // 1 stop bit (default)
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x1388, 0);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x138E, 30); // 3.0 ms response wait
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x138E, 30);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x018C, 1);  // RAM → NV save
    delay(50);
    writeParam32_(cfg_.right_slave_id, 0x018C, 1);
    delay(50);

    serial_->end();
    delay(100);
    serial_->begin(cfg_.baud, SERIAL_8E1);
    delay(100);
    drainRx_();
  }

  if (dbg_) {
    dbg_->print(F("MB: begin baud="));
    dbg_->print(cfg_.baud);
    dbg_->print(F(" DE_RE=D"));
    dbg_->println(de_re_pin_);
  }

  // Reference-code init: write 1 to register 0x0296 on both drivers.
  // Purpose (empirical): required to clear/unlatch communication-error
  // state after a power cycle or USB-reset; without this the driver
  // stays in alarm 0x84 even with correct 8-E-1 framing.
  if (cfg_.init_param_0296) {
    if (dbg_) { dbg_->println(F("MB: init param 0x0296=1")); }
    writeParam32_(cfg_.left_slave_id,  0x0296, 1);
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x0296, 1);
    delay(20);
  }

  // Make the driver tolerant of brief master interruptions (e.g. an Arduino
  // auto-reset triggered by opening /dev/ttyACM0 with screen).  Spec P.342:
  //   0x138C "通信異常アラーム(Modbus)" — consecutive comm-error count that
  //          triggers alarm 0x84.  Default 3 → raise to 10 (max).
  //   0x138A "通信タイムアウト(Modbus)" — silence-timeout that triggers
  //          alarm 0x85.  Default 0 (disabled) — write 0 explicitly.
  writeParam32_(cfg_.left_slave_id,  0x138C, 10);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x138C, 10);
  delay(20);
  writeParam32_(cfg_.left_slave_id,  0x138A, 0);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x138A, 0);
  delay(20);

  // Servo ON both drivers
  if (dbg_) { dbg_->print(F("MB: SON slave 0x0")); dbg_->println(cfg_.left_slave_id, HEX); }
  sendServoOn_(cfg_.left_slave_id);
  delay(50);
  if (dbg_) { dbg_->print(F("MB: SON slave 0x0")); dbg_->println(cfg_.right_slave_id, HEX); }
  sendServoOn_(cfg_.right_slave_id);
  delay(50);

#if MSD_USE_ID_SHARE_MODE
  // ID Share Mode: the Share Write data list (configured via MEXE02) carries
  // velocity / accel / decel / trigger only.  Static fields — operation type
  // (continuous velocity = 0x30) and torque limit — must be written once
  // here via individual unicast addressing.  These live in driver RAM, so
  // they need to be re-applied after every BLV-R power cycle.
  if (dbg_) { dbg_->println(F("MB: IDSHARE static init (op_type, torque)")); }
  writeParam32_(cfg_.left_slave_id,  0x005A, 0x30);  // op_type = continuous velocity
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x005A, 0x30);
  delay(20);
  writeParam32_(cfg_.left_slave_id,  0x0064, cfg_.torque_limit);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x0064, cfg_.torque_limit);
  delay(20);
#endif

  // Prime throttling state so first update() always sends.
  last_sent_rpm_l_ = 0x7FFFFFFF;
  last_sent_rpm_r_ = 0x7FFFFFFF;
  last_sent_ms_    = 0;

  init_done_ = true;

  // Clear any latched alarms inherited from the previous run.
  resetAlarms();

  if (dbg_) { dbg_->println(F("MB: init done")); }
}

void ModbusDiffDrive::reinitAfterPowerOn()
{
  // Called after the power relay closes and the BLV-R drivers have had
  // enough time (~2 s) to finish their internal power-on sequence.
  // Re-sends the same init sequence as begin() but without the 500 ms
  // USB-reset guard (that guard is only relevant on cold boot).
  if (!serial_) { return; }

  if (dbg_) { dbg_->println(F("MB: reinit after power-on")); }

  // Same all-baud probe-then-migrate as begin().  After a relay cycle the
  // driver wakes at its NV-saved baud, which may differ from cfg_.baud
  // if a previous migration's NV save did not take effect.
  const uint32_t found = probeAllBauds_();
  if (found == 0) {
    if (dbg_) {
      dbg_->println(F("MB: WARN no BLV-R response at ANY standard baud."));
      dbg_->println(F("MB:   See begin() warning notes (wiring/power/alarm/MEXE02)."));
    }
    serial_->end();
    delay(50);
    serial_->begin(cfg_.baud, SERIAL_8E1);
    delay(100);
  } else if (found == cfg_.baud) {
    if (dbg_) {
      dbg_->print(F("MB: BLV-R at target baud "));
      dbg_->println(cfg_.baud);
    }
  } else {
    if (dbg_) {
      dbg_->print(F("MB: relay-on saw "));
      dbg_->print(found);
      dbg_->print(F(", migrating to "));
      dbg_->println(cfg_.baud);
    }
    const int32_t baud_code = baud_to_blvr_code_(cfg_.baud);
    writeParam32_(cfg_.left_slave_id,  0x1382, baud_code);
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x1382, baud_code);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x1386, 1);
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x1386, 1);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x1388, 0);
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x1388, 0);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x138E, 30);
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x138E, 30);
    delay(20);
    writeParam32_(cfg_.left_slave_id,  0x018C, 1);
    delay(50);
    writeParam32_(cfg_.right_slave_id, 0x018C, 1);
    delay(50);

    serial_->end();
    delay(100);
    serial_->begin(cfg_.baud, SERIAL_8E1);
    delay(100);
    if (dbg_) {
      dbg_->println(F("MB: migrate done — power-cycle BLV-R to apply"));
    }
  }

  drainRx_();

  if (cfg_.init_param_0296) {
    writeParam32_(cfg_.left_slave_id,  0x0296, 1);
    delay(20);
    writeParam32_(cfg_.right_slave_id, 0x0296, 1);
    delay(20);
  }

  // Comm-error tolerance (see begin()).
  writeParam32_(cfg_.left_slave_id,  0x138C, 10);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x138C, 10);
  delay(20);
  writeParam32_(cfg_.left_slave_id,  0x138A, 0);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x138A, 0);
  delay(20);

  sendServoOn_(cfg_.left_slave_id);
  delay(50);
  sendServoOn_(cfg_.right_slave_id);
  delay(50);

#if MSD_USE_ID_SHARE_MODE
  // Re-apply ID-Share static fields after BLV-R power cycle (RAM-only).
  writeParam32_(cfg_.left_slave_id,  0x005A, 0x30);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x005A, 0x30);
  delay(20);
  writeParam32_(cfg_.left_slave_id,  0x0064, cfg_.torque_limit);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x0064, cfg_.torque_limit);
  delay(20);
#endif

  // Force the throttle to send a velocity frame on the very next update().
  last_sent_rpm_l_ = 0x7FFFFFFF;
  last_sent_rpm_r_ = 0x7FFFFFFF;
  last_sent_ms_    = 0;
  init_done_       = true;

  // Clear any latched alarms (0x20 overvoltage from previous regen, 0x84
  // RS-485 comm error, etc.) so the drivers come up ready to run.  Issued
  // after init_done_ so the maintenance command goes via the now-reopened
  // serial.  Cause-still-present alarms (e.g. 0x21 supply undervoltage)
  // will re-fire immediately — that's diagnostic information, not noise.
  resetAlarms();

  if (dbg_) { dbg_->println(F("MB: reinit done")); }
}

void ModbusDiffDrive::resetAlarms()
{
  // Maintenance command: write 1 to register 0x0180 (edge-triggered).
  // Spec P.307 ("メンテナンスコマンド" / "9-1 実行方法").
  // Each writeParam32_ is FC 0x10 across two registers (0x0180/0x0181).
  if (!serial_) { return; }
  if (dbg_) { dbg_->println(F("MB: alarm reset both drivers")); }
  drainRx_();
  writeParam32_(cfg_.left_slave_id,  0x0180, 1);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x0180, 1);
  delay(20);
  // Edge-trigger: also clear back to 0 so a subsequent reset still fires.
  writeParam32_(cfg_.left_slave_id,  0x0180, 0);
  delay(20);
  writeParam32_(cfg_.right_slave_id, 0x0180, 0);
  delay(20);
  err_count_ = 0;
}

void ModbusDiffDrive::drive(float linear_mps, float angular_rps)
{
  braking_ = false;

  // When moving backward, flip angular for intuitive steering.
  const float eff_ang = (linear_mps < 0.0f) ? -angular_rps : angular_rps;
  const float v_l = linear_mps - cfg_.wheel_distance_m * eff_ang;
  const float v_r = linear_mps + cfg_.wheel_distance_m * eff_ang;

  float rpm_l = vel_to_rpm_(v_l);
  float rpm_r = vel_to_rpm_(v_r);
  // Right motor physically reversed.
  rpm_r = -rpm_r;

  rpm_l = clamp_f(rpm_l, -cfg_.max_rpm, cfg_.max_rpm);
  rpm_r = clamp_f(rpm_r, -cfg_.max_rpm, cfg_.max_rpm);

  cmd_rpm_l_ = rpm_l;
  cmd_rpm_r_ = rpm_r;
}

void ModbusDiffDrive::stop(bool brake)
{
  cmd_rpm_l_ = 0.0f;
  cmd_rpm_r_ = 0.0f;

  // Only send soft-stop commands once on transition to stop.  Avoid the BLV-R
  // STOP bit here: with a heavy/inertial load it can regenerate enough energy
  // to trip the over-voltage alarm.  A zero-velocity direct-data command with
  // a long decel time gives the DC bus time to bleed down.
  if (brake && !braking_) {
    braking_ = true;
#if MSD_USE_ID_SHARE_MODE
    sendShareWrite_(0, 0, cfg_.accel_brake, cfg_.decel_brake, /*trigger=*/1);
    delay(5);
#else
    sendVelocity_(cfg_.left_slave_id,  0, cfg_.accel_brake, cfg_.decel_brake, cfg_.torque_brake);
    delay(5);
    sendVelocity_(cfg_.right_slave_id, 0, cfg_.accel_brake, cfg_.decel_brake, cfg_.torque_brake);
    delay(5);
#endif
  } else if (!brake) {
    braking_ = false;
  }
}

void ModbusDiffDrive::update()
{
  if (!serial_ || !init_done_) { return; }

  const uint32_t now = millis();
  if ((int32_t)(now - last_cmd_ms_) < (int32_t)cfg_.cmd_period_ms) {
    return;
  }
  last_cmd_ms_ = now;

  // Send velocity to both drivers every cmd_period_ms.
  const int32_t rpm_l = (int32_t)cmd_rpm_l_;
  const int32_t rpm_r = (int32_t)cmd_rpm_r_;

  int32_t acc, dec, torque;
  if (braking_) {
    acc     = cfg_.accel_brake;
    dec     = cfg_.decel_brake;
    torque  = cfg_.torque_brake;
  } else {
    acc     = cfg_.accel_ms;
    dec     = cfg_.decel_ms;
    torque  = cfg_.torque_limit;
  }

  // Command throttling (reference-code behavior): only send a new velocity
  // frame if the RPM delta exceeds a threshold OR refresh_ms has elapsed
  // since the last send. Cuts Modbus traffic ~10× at steady-state and
  // eliminates bus saturation that can trigger alarm 0x81.
  const int32_t dl = rpm_l - last_sent_rpm_l_;
  const int32_t dr = rpm_r - last_sent_rpm_r_;
  const int32_t absdl = dl < 0 ? -dl : dl;
  const int32_t absdr = dr < 0 ? -dr : dr;
  const bool    elapsed = (now - last_sent_ms_) >= cfg_.refresh_ms;
  const bool    changed = (absdl >= cfg_.rpm_change_threshold) ||
                          (absdr >= cfg_.rpm_change_threshold);
  const bool    must_send = changed || elapsed;

#if MSD_USE_ID_SHARE_MODE
  // ID Share Mode is kept for synchronous command writes only. Encoder ticks
  // are read by short individual FC03 reads from both drivers in one update;
  // this avoids the machine-specific ID Share read framing ambiguity while
  // still publishing left/right as one validated pair.
  if (must_send) {
    // Send velocity (and accel/decel/trigger) to both motors in one frame.
    sendShareWrite_(rpm_l, rpm_r, acc, dec, /*trigger=*/1);
    last_sent_rpm_l_ = rpm_l;
    last_sent_rpm_r_ = rpm_r;
    last_sent_ms_    = now;
    delay(2);  // brief gap before issuing the read query
  }

#if MSD_MODBUS_READ_ENABLE
  readEncoderPairDirect_();
  updateSlowDiagnostics_(now);
#else
  fb_rpm_l_   = (int32_t)cmd_rpm_l_;
  fb_rpm_r_   = -(int32_t)cmd_rpm_r_;
  fb_count_l_ = 0;
  fb_count_r_ = 0;
#endif

#else  // !MSD_USE_ID_SHARE_MODE — original individual-addressing path
  if (must_send) {
    sendVelocity_(cfg_.left_slave_id,  rpm_l, acc, dec, torque);
    delay(6);
    sendVelocity_(cfg_.right_slave_id, rpm_r, acc, dec, torque);
    delay(6);
    last_sent_rpm_l_ = rpm_l;
    last_sent_rpm_r_ = rpm_r;
    last_sent_ms_    = now;
  }

#if MSD_MODBUS_READ_ENABLE
  // Read the encoder source directly with ordinary Modbus FC03.  ID Share
  // read frames have been unreliable on the current wiring/configuration
  // (intermittent ID/timeout errors and shifted 76-byte payloads), so keep
  // odometry independent from the share-read list.
  int32_t v;
  switch (read_phase_) {
    case 0:  // 0x0120 検出32bitカウンタ [step]
      if (readI32_(cfg_.left_slave_id,  0x0120, v)) { fb_count_l_ =  v; } break;
    case 1:  // right motor is physically reversed; flip to robot frame.
      if (readI32_(cfg_.right_slave_id, 0x0120, v)) {
        fb_count_r_ = -v;
        ++fb_count_pair_seq_;
      }
      break;
    case 2:  // 0x00CE 検出速度 [r/min], diagnostics only
      if (readI32_(cfg_.left_slave_id,  0x00CE, v)) { fb_rpm_l_ =  v; } break;
    case 3:
      if (readI32_(cfg_.right_slave_id, 0x00CE, v)) { fb_rpm_r_ = -v; } break;
  }
  read_phase_ = (uint8_t)(read_phase_ + 1);
  if (read_phase_ >= 4) { read_phase_ = 0; }
#else
  fb_rpm_l_   = (int32_t)cmd_rpm_l_;
  fb_rpm_r_   = -(int32_t)cmd_rpm_r_;
  fb_count_l_ = 0;
  fb_count_r_ = 0;
#endif

#endif  // MSD_USE_ID_SHARE_MODE

  // Periodic debug (every ~2 sec)
  if (dbg_) {
    static uint32_t s_next_dbg = 0;
    if ((int32_t)(now - s_next_dbg) >= 0) {
      s_next_dbg = now + 2000;
#if MSD_USE_ID_SHARE_MODE
      dbg_->print(F("IDS rpm cmd L=")); dbg_->print(rpm_l);
#else
      dbg_->print(F("MB rpm cmd L=")); dbg_->print(rpm_l);
#endif
      dbg_->print(F(" R="));           dbg_->print(rpm_r);
      dbg_->print(F(" fb L="));        dbg_->print(fb_rpm_l_);
      dbg_->print(F(" R="));           dbg_->print(fb_rpm_r_);
      dbg_->print(F(" cnt L="));       dbg_->print(fb_count_l_);
      dbg_->print(F(" R="));           dbg_->print(fb_count_r_);
      dbg_->print(F(" trq L="));       dbg_->print(fb_torque_l_ * 0.1f, 1);
      dbg_->print(F("% R="));          dbg_->print(fb_torque_r_ * 0.1f, 1);
      dbg_->print(F("% Tdrv L="));     dbg_->print(fb_tdrv_l_ * 0.1f, 1);
      dbg_->print(F("C R="));          dbg_->print(fb_tdrv_r_ * 0.1f, 1);
      dbg_->print(F("C Tmot L="));     dbg_->print(fb_tmot_l_ * 0.1f, 1);
      dbg_->print(F("C R="));          dbg_->print(fb_tmot_r_ * 0.1f, 1);
      dbg_->print(F("C Vm L="));       dbg_->print(fb_vmain_l_ * 0.1f, 1);
      dbg_->print(F("V R="));          dbg_->print(fb_vmain_r_ * 0.1f, 1);
      dbg_->print(F("V Im L="));       dbg_->print(fb_imain_l_ * 0.001f, 2);
      dbg_->print(F("A R="));          dbg_->print(fb_imain_r_ * 0.001f, 2);
      dbg_->print(F("A pos L="));      dbg_->print(fb_pos_l_);
      dbg_->print(F(" R="));           dbg_->print(fb_pos_r_);
      dbg_->print(F(" odo L="));       dbg_->print(fb_odo_l_ * 0.0001f, 3);
      dbg_->print(F("krev R="));       dbg_->print(fb_odo_r_ * 0.0001f, 3);
      dbg_->print(F("krev err="));     dbg_->print(err_count_);
      dbg_->print(F(" almL=0x"));      dbg_->print((uint32_t)fb_alarm_l_, HEX);
      dbg_->print(F(" almR=0x"));      dbg_->print((uint32_t)fb_alarm_r_, HEX);
      dbg_->print(F(" brk="));         dbg_->println(braking_ ? '1' : '0');
    }
  }
}

}  // namespace msd_arduino
