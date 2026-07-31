// msd_link.cpp — high-level protocol manager implementation.

#include "msd_arduino/msd_link.hpp"

#include <cstring>
#include <utility>

namespace msd_arduino
{

MsdSerialLink::MsdSerialLink()
: tx_seq_(0)
{
}

bool MsdSerialLink::open(const std::string & port, int baud, int post_open_delay_ms)
{
  if (!serial_.open_port(port, baud, post_open_delay_ms)) {
    last_error_ = serial_.last_error();
    return false;
  }
  parser_.reset();
  return true;
}

void MsdSerialLink::close()
{
  serial_.close_port();
}

bool MsdSerialLink::is_open() const
{
  return serial_.is_open();
}

void MsdSerialLink::log_(const std::string & msg)
{
  if (log_sink_) {
    log_sink_(msg);
  }
}

int MsdSerialLink::update()
{
  if (!serial_.is_open()) {
    return 0;
  }

  int frames = 0;
  uint8_t buf[256];

  for (int guard = 0; guard < 32; ++guard) {
    int n = serial_.read_some(buf, sizeof(buf));
    if (n < 0) {
      log_("serial read error: " + serial_.last_error());
      serial_.close_port();
      return frames;
    }
    if (n == 0) {
      break;
    }
    for (int i = 0; i < n; ++i) {
      auto r = parser_.feed(buf[i]);
      if (r == protocol::FrameParser::FRAME_READY) {
        stats_.rx_frames++;
        dispatch_(frames);
      } else if (r == protocol::FrameParser::CRC_ERROR) {
        stats_.rx_crc_errs++;
      } else if (r == protocol::FrameParser::FRAME_ERROR) {
        stats_.rx_frame_errs++;
      }
    }
  }
  return frames;
}

void MsdSerialLink::dispatch_(int & frame_count)
{
  using namespace protocol;
  const uint8_t  type = parser_.get_type();
  const uint8_t  len  = parser_.get_payload_len();
  const uint8_t * p   = parser_.get_payload();
  ++frame_count;

  switch (type) {
    case MSG_A2J_HEARTBEAT:
      // driver_variant was appended later. Older firmware sends a 5-byte
      // payload (mcu_time_ms + system_state). Accept both sizes and default
      // variant to BLV when not present.
      if (len >= 5) {
        rx_.mcu_hb = {};
        rx_.mcu_hb.driver_variant = DRIVER_VARIANT_BLV;
        const size_t copy_n =
          (len < sizeof(McuHeartbeat)) ? len : sizeof(McuHeartbeat);
        std::memcpy(&rx_.mcu_hb, p, copy_n);
        rx_.mcu_hb_fresh = true;
        if (cb_hb_) {cb_hb_(rx_.mcu_hb);}
      }
      break;
    case MSG_A2J_ENCODER:
      if (len >= sizeof(Encoder)) {
        std::memcpy(&rx_.encoder, p, sizeof(Encoder));
        rx_.encoder_fresh = true;
        if (cb_enc_) {cb_enc_(rx_.encoder);}
      }
      break;
    case MSG_A2J_IMU:
      if (len >= sizeof(Imu)) {
        std::memcpy(&rx_.imu, p, sizeof(Imu));
        rx_.imu_fresh = true;
        if (cb_imu_) {cb_imu_(rx_.imu);}
      }
      break;
    case MSG_A2J_ENV:
      if (len >= sizeof(EnvSensor)) {
        std::memcpy(&rx_.env, p, sizeof(EnvSensor));
        rx_.env_fresh = true;
        if (cb_env_) {cb_env_(rx_.env);}
      }
      break;
    case MSG_A2J_CONTROL_STATE:
      if (len >= sizeof(ControlState)) {
        std::memcpy(&rx_.ctrl, p, sizeof(ControlState));
        rx_.ctrl_fresh = true;
        if (cb_ctrl_) {cb_ctrl_(rx_.ctrl);}
      }
      break;
    case MSG_A2J_ERROR:
      if (len >= sizeof(ErrorState)) {
        ErrorState es;
        std::memcpy(&es, p, sizeof(es));
        rx_.error_flags = es.error_flags;
        rx_.error_fresh = true;
        if (cb_err_) {cb_err_(es.error_flags);}
      }
      break;
    case MSG_A2J_SBUS_JOY:
      if (len >= sizeof(SbusChannels)) {
        std::memcpy(&rx_.sbus, p, sizeof(SbusChannels));
        rx_.sbus_fresh = true;
        if (cb_sbus_) {cb_sbus_(rx_.sbus);}
      }
      break;
    case MSG_A2J_BATTERY:
      if (len >= sizeof(BatteryStatus)) {
        std::memcpy(&rx_.battery, p, sizeof(BatteryStatus));
        rx_.battery_fresh = true;
        if (cb_bat_) {cb_bat_(rx_.battery);}
      }
      break;
    case MSG_A2J_MOTOR_STATUS:
      if (len >= sizeof(MotorStatus)) {
        std::memcpy(&rx_.motor_status, p, sizeof(MotorStatus));
        rx_.motor_status_fresh = true;
        if (cb_mstat_) {cb_mstat_(rx_.motor_status);}
      }
      break;
    case MSG_A2J_SYSTEM_STATUS:
      if (len >= sizeof(SystemStatus)) {
        std::memcpy(&rx_.system_status, p, sizeof(SystemStatus));
        rx_.system_status_fresh = true;
        if (cb_sysstat_) {cb_sysstat_(rx_.system_status);}
      }
      break;
    default:
      stats_.rx_unknown++;
      break;
  }
}

bool MsdSerialLink::sendRaw(uint8_t type, const void * payload, uint8_t len)
{
  if (!serial_.is_open()) {
    return false;
  }
  uint8_t buf[protocol::MAX_FRAME_SIZE];
  size_t n = protocol::encode_frame(
    type, tx_seq_++,
    static_cast<const uint8_t *>(payload), len,
    buf, sizeof(buf));
  if (n == 0) {
    return false;
  }
  if (!serial_.write_all(buf, n)) {
    last_error_ = serial_.last_error();
    log_("serial write error: " + last_error_);
    return false;
  }
  stats_.tx_frames++;
  return true;
}

bool MsdSerialLink::sendHeartbeat(uint32_t host_ms, uint8_t state)
{
  protocol::Heartbeat hb;
  hb.host_time_ms = host_ms;
  hb.system_state = state;
  return sendRaw(protocol::MSG_J2A_HEARTBEAT, &hb, sizeof(hb));
}

bool MsdSerialLink::sendCmdVel(int16_t linear_mmps, int16_t angular_mradps)
{
  protocol::CmdVel c;
  c.linear_x_mmps    = linear_mmps;
  c.angular_z_mradps = angular_mradps;
  return sendRaw(protocol::MSG_J2A_CMD_VEL, &c, sizeof(c));
}

bool MsdSerialLink::sendArmCmd(int16_t cyl1, int16_t cyl2, uint8_t mode)
{
  protocol::ArmCmd a;
  a.cyl1_target  = cyl1;
  a.cyl2_target  = cyl2;
  a.command_mode = mode;
  return sendRaw(protocol::MSG_J2A_ARM_CMD, &a, sizeof(a));
}

bool MsdSerialLink::sendControlMode(uint8_t mode)
{
  return sendRaw(protocol::MSG_J2A_CONTROL_MODE, &mode, sizeof(mode));
}

bool MsdSerialLink::sendSystemCmd(uint8_t cmd)
{
  return sendRaw(protocol::MSG_J2A_SYSTEM_CMD, &cmd, sizeof(cmd));
}

bool MsdSerialLink::sendRelayCmd(uint8_t state)
{
  protocol::RelayCmd r;
  r.state = state;
  return sendRaw(protocol::MSG_J2A_RELAY_CMD, &r, sizeof(r));
}

}  // namespace msd_arduino
