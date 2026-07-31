// ROS2Serial protocol — v0.1 (Arduino Mega, BLV)
//
// Self-contained protocol definitions for the Arduino side. The Jetson ROS2
// node has its own copy; the two sides only need to agree on the wire format
// (frame layout, message IDs, payload binary layout) — the source files are
// intentionally independent so this firmware can be developed and tested on
// its own without touching the ROS2 tree.
//
// Frame layout (little-endian, LEN ≤ MAX_PAYLOAD):
//   [0] SOF1  = 0xAA
//   [1] SOF2  = 0x55
//   [2] VER   = 0x01
//   [3] TYPE
//   [4] LEN
//   [5] SEQ_L
//   [6] SEQ_H
//   [7..7+LEN-1]     PAYLOAD
//   [7+LEN..8+LEN]   CRC16-CCITT (little-endian) over VER..PAYLOAD_END

#ifndef MSD_BLV_PROTOCOL_H_
#define MSD_BLV_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace msd_arduino
{
namespace protocol
{

// ---- Frame constants ----
static const uint8_t  SOF1            = 0xAA;
static const uint8_t  SOF2            = 0x55;
static const uint8_t  VERSION         = 0x01;
static const size_t   HEADER_SIZE     = 7;
static const size_t   CRC_SIZE        = 2;
static const size_t   FRAME_OVERHEAD  = HEADER_SIZE + CRC_SIZE;  // 9
static const size_t   MAX_PAYLOAD     = 64;
static const size_t   MAX_FRAME_SIZE  = MAX_PAYLOAD + FRAME_OVERHEAD;

static const size_t OFFSET_SOF1    = 0;
static const size_t OFFSET_SOF2    = 1;
static const size_t OFFSET_VER     = 2;
static const size_t OFFSET_TYPE    = 3;
static const size_t OFFSET_LEN     = 4;
static const size_t OFFSET_SEQ_L   = 5;
static const size_t OFFSET_SEQ_H   = 6;
static const size_t OFFSET_PAYLOAD = 7;

// ---- Message IDs ----
// Jetson → Arduino
static const uint8_t MSG_J2A_HEARTBEAT    = 0x10;
static const uint8_t MSG_J2A_CMD_VEL      = 0x11;
static const uint8_t MSG_J2A_ARM_CMD      = 0x12;
static const uint8_t MSG_J2A_CONTROL_MODE = 0x13;
static const uint8_t MSG_J2A_SYSTEM_CMD   = 0x14;
static const uint8_t MSG_J2A_RELAY_CMD    = 0x15;  // payload: 1 byte, 0=OFF 1=ON

// Arduino → Jetson
static const uint8_t MSG_A2J_HEARTBEAT     = 0x20;
static const uint8_t MSG_A2J_ENCODER       = 0x21;
static const uint8_t MSG_A2J_IMU           = 0x22;
static const uint8_t MSG_A2J_ENV           = 0x23;
static const uint8_t MSG_A2J_CONTROL_STATE = 0x24;
static const uint8_t MSG_A2J_ERROR         = 0x25;
static const uint8_t MSG_A2J_SYSTEM_STATUS = 0x26;  // combined heartbeat + ctrl + error
static const uint8_t MSG_A2J_SBUS_JOY      = 0x28;
static const uint8_t MSG_A2J_BATTERY       = 0x29;
static const uint8_t MSG_A2J_MOTOR_STATUS  = 0x2A;  // BLV-R diagnostics (torque/temp/alarm)

// ---- Error flag bits ----
static const uint32_t ERR_CRC        = 1UL << 0;
static const uint32_t ERR_CMD_VEL_TO = 1UL << 1;
static const uint32_t ERR_ARM_TO     = 1UL << 2;
static const uint32_t ERR_HB_LOST    = 1UL << 3;
static const uint32_t ERR_SENSOR     = 1UL << 4;
static const uint32_t ERR_ACTUATOR   = 1UL << 5;
static const uint32_t ERR_FRAME      = 1UL << 6;
static const uint32_t ERR_ESTOP      = 1UL << 7;
static const uint32_t ERR_SBUS      = 1UL << 8;   // SBUS receiver not responding
static const uint32_t ERR_HW_ESTOP   = 1UL << 9;   // D22 hardware e-stop active
static const uint32_t ERR_YCON       = 1UL << 10;  // ycon I2C controller not responding

// ---- System command codes ----
static const uint8_t SYS_CMD_CLEAR_ERRORS    = 0;
static const uint8_t SYS_CMD_RESET_TARGETS   = 1;
static const uint8_t SYS_CMD_ESTOP           = 2;
static const uint8_t SYS_CMD_ESTOP_CLEAR     = 3;
static const uint8_t SYS_CMD_MOTOR_ALM_RESET = 4;  // BLV-R reg 0x0180=1

// ---- Watchdog / timeout parameters (ms) ----
static const uint32_t HEARTBEAT_PERIOD_MS  = 50;
static const uint32_t HEARTBEAT_TIMEOUT_MS = 200;
static const uint32_t CMD_VEL_TIMEOUT_MS   = 500;
static const uint32_t ARM_CMD_TIMEOUT_MS   = 1000;

// ---- Communication states ----
static const uint8_t COMM_INIT = 0;
static const uint8_t COMM_OK   = 1;
static const uint8_t COMM_LOST = 2;

// ---- Control source (for control_mode field in 0x24) ----
static const uint8_t CTRL_SRC_JETSON = 0;
static const uint8_t CTRL_SRC_SBUS   = 1;
static const uint8_t CTRL_SRC_YCON   = 2;

// ---- Driver variant (for McuHeartbeat.driver_variant) ----
// Allows the Jetson node to auto-detect which motor backend the firmware
// is built for, and enable/disable BLV-R-only topics accordingly.
static const uint8_t DRIVER_VARIANT_BLV  = 0;
static const uint8_t DRIVER_VARIANT_BLVR = 1;

// ---------- Payload structs (packed, little-endian) ----------
#pragma pack(push, 1)

struct Heartbeat        // 0x10
{
  uint32_t host_time_ms;
  uint8_t  system_state;
};

struct CmdVel           // 0x11 (payload is also reused for 0x27 echo)
{
  int16_t linear_x_mmps;
  int16_t angular_z_mradps;
};

struct ArmCmd           // 0x12
{
  int16_t cyl1_target;
  int16_t cyl2_target;
  uint8_t command_mode;
};

struct McuHeartbeat     // 0x20
{
  uint32_t mcu_time_ms;
  uint8_t  system_state;
  // Extended field (added later). Jetson side may receive a 5-byte payload
  // from older firmware — dispatch must still accept len >= 5 and default
  // this field to DRIVER_VARIANT_BLV.
  uint8_t  driver_variant;   // DRIVER_VARIANT_BLV or DRIVER_VARIANT_BLVR
};

struct RelayCmd         // 0x15
{
  uint8_t state;  // power_relay: 0 = OFF, non-zero = ON
};

struct Encoder          // 0x21
{
  int32_t  left_motor_count;
  int32_t  right_motor_count;
  int32_t  cyl1_count;
  int32_t  cyl2_count;
  uint32_t sample_time_ms;
};

struct Imu              // 0x22 — BNO055 euler + quaternion
{
  // Euler angles in 0.001 deg (BNO055 VECTOR_EULER, signed).
  int16_t euler_x_mdeg;
  int16_t euler_y_mdeg;
  int16_t euler_z_mdeg;
  // Quaternion scaled by 1e4 (range ±1.0 → ±10000).
  int16_t quat_w_1e4;
  int16_t quat_x_1e4;
  int16_t quat_y_1e4;
  int16_t quat_z_1e4;
};

struct EnvSensor        // 0x23
{
  int16_t  temperature_cdeg;
  uint16_t humidity_cpercent;
  uint32_t pressure_pa;
};

struct ControlState     // 0x24
{
  uint8_t control_mode;   // active source: CTRL_SRC_JETSON / _SBUS / _YCON
  uint8_t comm_state;
  uint8_t motor_state;
  uint8_t arm_state;
};

struct ErrorState       // 0x25
{
  uint32_t error_flags;
};

// 0x26 — Combined heartbeat / control / error in one packet (BLV-R latency
// reduction).  Replaces separate 0x20 + 0x24 + 0x25 frames.
struct SystemStatus
{
  uint32_t mcu_time_ms;     // [0..3]
  uint32_t error_flags;     // [4..7]
  uint8_t  system_state;    // [8]
  uint8_t  driver_variant;  // [9]  DRIVER_VARIANT_BLV / _BLVR
  uint8_t  control_mode;    // [10] active source: CTRL_SRC_JETSON / _SBUS / _YCON
  uint8_t  comm_state;      // [11] COMM_INIT / _OK / _LOST
  uint8_t  motor_state;     // [12] 0 = idle, 1 = active
  uint8_t  arm_state;       // [13]
  uint8_t  relay_state;     // [14] power_relay_state: 0 = OFF, 1 = ON
};

struct SbusChannels     // 0x28 — SBUS debug output (ch1-8 normalized)
{
  int16_t ch[8];        // normalized × 10000 (±1.0 → ±10000)
};

struct MotorStatus      // 0x2A — BLV-R diagnostics from Modbus monitors
{
  // Torque (符号付き 0.1 %).  BLV-R reg 0x00D6/D7.  ±32767 → ±3276.7 %
  int16_t torque_l_01pct;
  int16_t torque_r_01pct;
  // Driver temperature (signed 0.1 °C). BLV-R reg 0x00F8/F9.
  int16_t temp_drv_l_01c;
  int16_t temp_drv_r_01c;
  // Motor temperature (signed 0.1 °C). BLV-R reg 0x00FA/FB.
  int16_t temp_mot_l_01c;
  int16_t temp_mot_r_01c;
  // Current alarm code (0 = no alarm). BLV-R reg 0x0080/81.
  uint16_t alarm_l;
  uint16_t alarm_r;
  // 検出位置 [step, signed].  BLV-R reg 0x00CC/CD (NET-ID 102).  Right-side
  // sign-flipped to robot frame.
  int32_t pos_l_step;
  int32_t pos_r_step;
  // ODOメータ [0.1 krev = 100 rev units, signed].  BLV-R reg 0x00FC/FD.
  int32_t odo_l_01krev;
  int32_t odo_r_01krev;
  // 主電源電流 [0.001 A = 1 mA units, signed].  BLV-R reg 0x0136/37.
  int32_t imain_l_001a;
  int32_t imain_r_001a;
};

struct BatteryStatus    // 0x29 — CAN battery BMS
{
  uint16_t mod1_voltage_10mv;   // module1 voltage in 10 mV units
  int16_t  mod1_current_10ma;   // module1 current in 10 mA units (signed)
  int16_t  mod1_temp_max_01c;   // module1 max cell temp in 0.1 °C
  int16_t  mod1_temp_fet_01c;   // module1 FET temp in 0.1 °C
  int16_t  mod1_temp_min_01c;   // module1 min cell temp in 0.1 °C
  uint16_t mod2_voltage_10mv;   // module2 voltage
  int16_t  mod2_current_10ma;   // module2 current
  int16_t  mod2_temp_max_01c;   // module2 max cell temp
  int16_t  mod2_temp_fet_01c;   // module2 FET temp
  int16_t  mod2_temp_min_01c;   // module2 min cell temp
  uint16_t soc_mah;             // remaining capacity in mAh
  uint8_t  soc_percent;         // 0-100 %
};

#pragma pack(pop)

// ---- CRC-16-CCITT (poly 0x1021, init 0xFFFF) ----
inline uint16_t crc16_ccitt(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

// ---- Frame encoder ----
inline size_t encode_frame(
  uint8_t type, uint16_t seq,
  const uint8_t * payload, uint8_t payload_len,
  uint8_t * out_buf, size_t out_buf_size)
{
  const size_t frame_len = FRAME_OVERHEAD + payload_len;
  if (out_buf_size < frame_len || payload_len > MAX_PAYLOAD) {
    return 0;
  }

  out_buf[OFFSET_SOF1]  = SOF1;
  out_buf[OFFSET_SOF2]  = SOF2;
  out_buf[OFFSET_VER]   = VERSION;
  out_buf[OFFSET_TYPE]  = type;
  out_buf[OFFSET_LEN]   = payload_len;
  out_buf[OFFSET_SEQ_L] = (uint8_t)(seq & 0xFF);
  out_buf[OFFSET_SEQ_H] = (uint8_t)((seq >> 8) & 0xFF);

  for (uint8_t i = 0; i < payload_len; ++i) {
    out_buf[OFFSET_PAYLOAD + i] = payload[i];
  }

  const size_t crc_len = 5 + payload_len;
  const uint16_t crc = crc16_ccitt(&out_buf[OFFSET_VER], crc_len);
  out_buf[OFFSET_PAYLOAD + payload_len + 0] = (uint8_t)(crc & 0xFF);
  out_buf[OFFSET_PAYLOAD + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

  return frame_len;
}

// ---- Byte-at-a-time frame parser ----
class FrameParser
{
public:
  enum Result
  {
    NEED_MORE   = 0,
    FRAME_READY = 1,
    CRC_ERROR   = 2,
    FRAME_ERROR = 3,
  };

  FrameParser() : state_(WAIT_SOF1), type_(0), payload_len_(0),
                  seq_(0), idx_(0), crc_l_(0), crc_buf_idx_(0) {}

  void reset()
  {
    state_ = WAIT_SOF1;
    payload_len_ = 0;
    idx_ = 0;
    crc_buf_idx_ = 0;
  }

  Result feed(uint8_t b)
  {
    switch (state_) {
      case WAIT_SOF1:
        if (b == SOF1) {state_ = WAIT_SOF2;}
        return NEED_MORE;

      case WAIT_SOF2:
        if (b == SOF2) {
          state_ = WAIT_VER;
          crc_buf_idx_ = 0;
        } else if (b != SOF1) {
          state_ = WAIT_SOF1;
        }
        return NEED_MORE;

      case WAIT_VER:
        if (b != VERSION) {
          reset();
          return FRAME_ERROR;
        }
        crc_buf_[crc_buf_idx_++] = b;
        state_ = WAIT_TYPE;
        return NEED_MORE;

      case WAIT_TYPE:
        type_ = b;
        crc_buf_[crc_buf_idx_++] = b;
        state_ = WAIT_LEN;
        return NEED_MORE;

      case WAIT_LEN:
        payload_len_ = b;
        if (payload_len_ > MAX_PAYLOAD) {
          reset();
          return FRAME_ERROR;
        }
        crc_buf_[crc_buf_idx_++] = b;
        state_ = WAIT_SEQ_L;
        return NEED_MORE;

      case WAIT_SEQ_L:
        seq_ = b;
        crc_buf_[crc_buf_idx_++] = b;
        state_ = WAIT_SEQ_H;
        return NEED_MORE;

      case WAIT_SEQ_H:
        seq_ |= (uint16_t)b << 8;
        crc_buf_[crc_buf_idx_++] = b;
        idx_ = 0;
        state_ = (payload_len_ == 0) ? WAIT_CRC_L : WAIT_PAYLOAD;
        return NEED_MORE;

      case WAIT_PAYLOAD:
        payload_[idx_] = b;
        crc_buf_[crc_buf_idx_++] = b;
        ++idx_;
        if (idx_ >= payload_len_) {state_ = WAIT_CRC_L;}
        return NEED_MORE;

      case WAIT_CRC_L:
        crc_l_ = b;
        state_ = WAIT_CRC_H;
        return NEED_MORE;

      case WAIT_CRC_H: {
          const uint16_t rx_crc = (uint16_t)crc_l_ | ((uint16_t)b << 8);
          const uint16_t calc = crc16_ccitt(crc_buf_, crc_buf_idx_);
          state_ = WAIT_SOF1;
          return (rx_crc == calc) ? FRAME_READY : CRC_ERROR;
        }
    }
    return NEED_MORE;
  }

  uint8_t         get_type()        const {return type_;}
  uint16_t        get_seq()         const {return seq_;}
  uint8_t         get_payload_len() const {return payload_len_;}
  const uint8_t * get_payload()     const {return payload_;}

private:
  enum State
  {
    WAIT_SOF1 = 0, WAIT_SOF2, WAIT_VER, WAIT_TYPE, WAIT_LEN,
    WAIT_SEQ_L, WAIT_SEQ_H, WAIT_PAYLOAD, WAIT_CRC_L, WAIT_CRC_H
  };

  State    state_;
  uint8_t  type_;
  uint8_t  payload_len_;
  uint16_t seq_;
  uint8_t  idx_;
  uint8_t  crc_l_;
  uint16_t crc_buf_idx_;
  uint8_t  payload_[MAX_PAYLOAD];
  uint8_t  crc_buf_[5 + MAX_PAYLOAD];
};

}  // namespace protocol
}  // namespace msd_arduino

#endif  // MSD_BLV_PROTOCOL_H_
