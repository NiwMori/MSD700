// ROS2Serial protocol definitions — v0.1
//
// Used by the ROS2 node. The Arduino side (firmware/blv_arduino_mega/
// protocol.h) has its own independent copy; the two files only need to
// agree on the wire format (frame layout, message IDs, and payload
// binary layout) so the firmware can be developed and built on its own.
//
// Frame layout (little-endian payload, LEN ≤ MAX_PAYLOAD):
//   [0] SOF1  = 0xAA
//   [1] SOF2  = 0x55
//   [2] VER   = 0x01
//   [3] TYPE
//   [4] LEN
//   [5] SEQ_L
//   [6] SEQ_H
//   [7..7+LEN-1]            PAYLOAD
//   [7+LEN..8+LEN]          CRC16-CCITT (little-endian) over VER..PAYLOAD_END

#ifndef MSD_ARDUINO__PROTOCOL_HPP_
#define MSD_ARDUINO__PROTOCOL_HPP_

#include <stddef.h>
#include <stdint.h>

namespace msd_arduino
{
namespace protocol
{

// ---- Frame constants ----
constexpr uint8_t  SOF1            = 0xAA;
constexpr uint8_t  SOF2            = 0x55;
constexpr uint8_t  VERSION         = 0x01;
constexpr size_t   HEADER_SIZE     = 7;       // SOF1..SEQ_H
constexpr size_t   CRC_SIZE        = 2;
constexpr size_t   FRAME_OVERHEAD  = HEADER_SIZE + CRC_SIZE;  // 9 bytes
constexpr size_t   MAX_PAYLOAD     = 64;      // headroom over current max (20)
constexpr size_t   MAX_FRAME_SIZE  = MAX_PAYLOAD + FRAME_OVERHEAD;

// Field offsets
constexpr size_t OFFSET_SOF1    = 0;
constexpr size_t OFFSET_SOF2    = 1;
constexpr size_t OFFSET_VER     = 2;
constexpr size_t OFFSET_TYPE    = 3;
constexpr size_t OFFSET_LEN     = 4;
constexpr size_t OFFSET_SEQ_L   = 5;
constexpr size_t OFFSET_SEQ_H   = 6;
constexpr size_t OFFSET_PAYLOAD = 7;

// ---- Message IDs ----
// Jetson → Arduino
constexpr uint8_t MSG_J2A_HEARTBEAT    = 0x10;
constexpr uint8_t MSG_J2A_CMD_VEL      = 0x11;
constexpr uint8_t MSG_J2A_ARM_CMD      = 0x12;
constexpr uint8_t MSG_J2A_CONTROL_MODE = 0x13;
constexpr uint8_t MSG_J2A_SYSTEM_CMD   = 0x14;
constexpr uint8_t MSG_J2A_RELAY_CMD    = 0x15;  // payload: 1 byte, 0=OFF 1=ON

// Arduino → Jetson
constexpr uint8_t MSG_A2J_HEARTBEAT     = 0x20;
constexpr uint8_t MSG_A2J_ENCODER       = 0x21;
constexpr uint8_t MSG_A2J_IMU           = 0x22;
constexpr uint8_t MSG_A2J_ENV           = 0x23;
constexpr uint8_t MSG_A2J_CONTROL_STATE = 0x24;
constexpr uint8_t MSG_A2J_ERROR         = 0x25;
constexpr uint8_t MSG_A2J_SYSTEM_STATUS = 0x26;  // combined heartbeat + ctrl + error
constexpr uint8_t MSG_A2J_SBUS_JOY      = 0x28;
constexpr uint8_t MSG_A2J_BATTERY       = 0x29;
constexpr uint8_t MSG_A2J_MOTOR_STATUS  = 0x2A;  // BLV-R diagnostics

// ---- Error flag bits (error_flags in 0x25) ----
constexpr uint32_t ERR_CRC         = 1u << 0;
constexpr uint32_t ERR_CMD_VEL_TO  = 1u << 1;
constexpr uint32_t ERR_ARM_TO      = 1u << 2;
constexpr uint32_t ERR_HB_LOST     = 1u << 3;
constexpr uint32_t ERR_SENSOR      = 1u << 4;
constexpr uint32_t ERR_ACTUATOR    = 1u << 5;
constexpr uint32_t ERR_FRAME       = 1u << 6;
constexpr uint32_t ERR_ESTOP       = 1u << 7;   // emergency stop active
constexpr uint32_t ERR_SBUS        = 1u << 8;   // SBUS receiver not responding
constexpr uint32_t ERR_HW_ESTOP    = 1u << 9;   // D22 hardware e-stop active
constexpr uint32_t ERR_YCON        = 1u << 10;  // ycon I2C controller not responding

// ---- System command codes (payload byte of 0x14) ----
constexpr uint8_t SYS_CMD_CLEAR_ERRORS    = 0;
constexpr uint8_t SYS_CMD_RESET_TARGETS   = 1;
constexpr uint8_t SYS_CMD_ESTOP           = 2;  // enter emergency stop
constexpr uint8_t SYS_CMD_ESTOP_CLEAR     = 3;  // leave emergency stop
constexpr uint8_t SYS_CMD_MOTOR_ALM_RESET = 4;  // BLV-R alarm reset (reg 0x0180=1)

// ---- Watchdog / timeout parameters (ms) ----
constexpr uint32_t HEARTBEAT_PERIOD_MS  = 50;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 200;
constexpr uint32_t CMD_VEL_TIMEOUT_MS   = 500;
constexpr uint32_t ARM_CMD_TIMEOUT_MS   = 1000;

// ---- Communication states (comm_state in 0x24) ----
constexpr uint8_t COMM_INIT = 0;
constexpr uint8_t COMM_OK   = 1;
constexpr uint8_t COMM_LOST = 2;

// ---- Active control source (control_mode in 0x24) ----
constexpr uint8_t CTRL_SRC_JETSON = 0;
constexpr uint8_t CTRL_SRC_SBUS   = 1;
constexpr uint8_t CTRL_SRC_YCON   = 2;

// ---- Driver variant (driver_variant in McuHeartbeat) ----
// Runtime detection of firmware motor backend. When reading McuHeartbeat,
// payloads from older firmware will be 5 bytes (no driver_variant field);
// treat that case as DRIVER_VARIANT_BLV for backward compatibility.
constexpr uint8_t DRIVER_VARIANT_BLV  = 0;
constexpr uint8_t DRIVER_VARIANT_BLVR = 1;

// ---------- Payload structs (packed, little-endian) ----------
// AVR (Arduino Mega) and ARM64 (Jetson) are both little-endian, so memcpy of
// these structs into the PAYLOAD region is wire-compatible.
#pragma pack(push, 1)

struct Heartbeat        // 0x10
{
  uint32_t host_time_ms;
  uint8_t  system_state;
};

struct CmdVel           // 0x11
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
  uint8_t  driver_variant;  // DRIVER_VARIANT_BLV / _BLVR (appended field)
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
  // Euler angles in 0.001 deg steps (range ±180 deg → ±180000).
  int16_t euler_x_mdeg;   // BNO055 getVector(VECTOR_EULER).x()
  int16_t euler_y_mdeg;
  int16_t euler_z_mdeg;
  // Quaternion components scaled by 1e4 (range ±1.0 → ±10000).
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
  uint8_t control_mode;
  uint8_t comm_state;
  uint8_t motor_state;
  uint8_t arm_state;
};

struct ErrorState       // 0x25
{
  uint32_t error_flags;
};

struct SystemStatus     // 0x26 — combined heartbeat / control / error
{
  uint32_t mcu_time_ms;
  uint32_t error_flags;
  uint8_t  system_state;
  uint8_t  driver_variant;
  uint8_t  control_mode;
  uint8_t  comm_state;
  uint8_t  motor_state;
  uint8_t  arm_state;
  uint8_t  relay_state;  // power_relay_state: 0 = OFF, 1 = ON
};

struct SbusChannels     // 0x28 — SBUS debug output (ch1-8 normalized)
{
  int16_t ch[8];        // normalized x 10000 (range +-1.0 -> +-10000)
};

struct MotorStatus      // 0x2A — BLV-R diagnostics from Modbus monitors
{
  int16_t  torque_l_01pct;   // 0.1 % (BLV-R reg 0x00D6/D7)
  int16_t  torque_r_01pct;
  int16_t  temp_drv_l_01c;   // 0.1 °C (BLV-R reg 0x00F8/F9)
  int16_t  temp_drv_r_01c;
  int16_t  temp_mot_l_01c;   // 0.1 °C (BLV-R reg 0x00FA/FB)
  int16_t  temp_mot_r_01c;
  uint16_t alarm_l;          // alarm code (BLV-R reg 0x0080/81)
  uint16_t alarm_r;
  int32_t  pos_l_step;       // 検出位置 step (BLV-R reg 0x00CC/CD)
  int32_t  pos_r_step;
  int32_t  odo_l_01krev;     // ODO 0.1 krev (BLV-R reg 0x00FC/FD)
  int32_t  odo_r_01krev;
  int32_t  imain_l_001a;     // 主電源電流 mA (BLV-R reg 0x0136/37)
  int32_t  imain_r_001a;
};

struct BatteryStatus    // 0x29 — CAN battery BMS
{
  uint16_t mod1_voltage_10mv;
  int16_t  mod1_current_10ma;
  int16_t  mod1_temp_max_01c;
  int16_t  mod1_temp_fet_01c;
  int16_t  mod1_temp_min_01c;
  uint16_t mod2_voltage_10mv;
  int16_t  mod2_current_10ma;
  int16_t  mod2_temp_max_01c;
  int16_t  mod2_temp_fet_01c;
  int16_t  mod2_temp_min_01c;
  uint16_t soc_mah;
  uint8_t  soc_percent;
};

#pragma pack(pop)

// ---- CRC-16-CCITT (poly 0x1021, init 0xFFFF, no refl, xorout 0) ----
inline uint16_t crc16_ccitt(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

// ---- Frame encoder ----
// Writes a full frame (header + payload + CRC) into `out_buf`.
// Returns the total frame length on success, or 0 on buffer overflow.
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
  out_buf[OFFSET_SEQ_L] = static_cast<uint8_t>(seq & 0xFF);
  out_buf[OFFSET_SEQ_H] = static_cast<uint8_t>((seq >> 8) & 0xFF);

  for (uint8_t i = 0; i < payload_len; ++i) {
    out_buf[OFFSET_PAYLOAD + i] = payload[i];
  }

  // CRC target: VER..PAYLOAD end = 5 + payload_len bytes
  const size_t crc_len = 5 + payload_len;
  const uint16_t crc = crc16_ccitt(&out_buf[OFFSET_VER], crc_len);
  out_buf[OFFSET_PAYLOAD + payload_len + 0] = static_cast<uint8_t>(crc & 0xFF);
  out_buf[OFFSET_PAYLOAD + payload_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  return frame_len;
}

// ---- Byte-at-a-time frame parser (state machine, zero-alloc) ----
class FrameParser
{
public:
  enum Result
  {
    NEED_MORE   = 0,  // incomplete, keep feeding
    FRAME_READY = 1,  // valid frame parsed; fields accessible
    CRC_ERROR   = 2,  // header valid but CRC mismatch
    FRAME_ERROR = 3,  // version/length invalid — parser was reset
  };

  void reset()
  {
    state_ = WAIT_SOF1;
    payload_len_ = 0;
    idx_ = 0;
    crc_buf_idx_ = 0;
  }

  // Feeds one received byte. When the return value is FRAME_READY, the
  // accessors below expose the parsed frame until the next feed() call.
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
        } else if (b == SOF1) {
          // stay — repeated SOF1
        } else {
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
        seq_ |= static_cast<uint16_t>(b) << 8;
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
          const uint16_t rx_crc =
            static_cast<uint16_t>(crc_l_) |
            (static_cast<uint16_t>(b) << 8);
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

  State    state_        = WAIT_SOF1;
  uint8_t  type_         = 0;
  uint8_t  payload_len_  = 0;
  uint16_t seq_          = 0;
  uint8_t  idx_          = 0;
  uint8_t  crc_l_        = 0;
  uint16_t crc_buf_idx_  = 0;
  uint8_t  payload_[MAX_PAYLOAD];
  uint8_t  crc_buf_[5 + MAX_PAYLOAD];   // VER..PAYLOAD end
};

}  // namespace protocol
}  // namespace msd_arduino

#endif  // MSD_ARDUINO__PROTOCOL_HPP_
