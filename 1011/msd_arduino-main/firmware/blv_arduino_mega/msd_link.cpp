// msd_link.cpp — Arduino-side protocol manager implementation.

#include "msd_link.h"

#include <string.h>

namespace msd_arduino
{

MsdProtocolLink::MsdProtocolLink()
{
}

void MsdProtocolLink::begin(Stream & link_stream)
{
  stream_ = &link_stream;
  parser_.reset();
  tx_seq_ = 0;
}

int MsdProtocolLink::update()
{
  if (!stream_) {
    return 0;
  }
  int frames = 0;
  // Bound the inner loop so a flood of bytes cannot starve the rest of
  // the main loop.
  for (int guard = 0; guard < 256 && stream_->available() > 0; ++guard) {
    const int b_in = stream_->read();
    if (b_in < 0) {break;}
    const protocol::FrameParser::Result r = parser_.feed((uint8_t)b_in);
    if (r == protocol::FrameParser::FRAME_READY) {
      stats_.rx_frames++;
      dispatch_();
      ++frames;
    } else if (r == protocol::FrameParser::CRC_ERROR) {
      stats_.rx_crc_errs++;
      if (dbg_) {dbg_->println(F("LINK: CRC error"));}
    } else if (r == protocol::FrameParser::FRAME_ERROR) {
      stats_.rx_frame_errs++;
      if (dbg_) {dbg_->println(F("LINK: frame error"));}
    }
  }
  return frames;
}

void MsdProtocolLink::dispatch_()
{
  using namespace protocol;
  const uint8_t  type = parser_.get_type();
  const uint8_t  len  = parser_.get_payload_len();
  const uint8_t *p    = parser_.get_payload();
  const uint32_t now  = millis();

  switch (type) {
    case MSG_J2A_HEARTBEAT:
      if (len >= sizeof(Heartbeat)) {
        memcpy(&rx_.host_hb, p, sizeof(Heartbeat));
        rx_.host_hb_fresh    = true;
        rx_.host_hb_stamp_ms = now;
        rx_.any_host_hb_seen = true;
      }
      break;
    case MSG_J2A_CMD_VEL:
      if (len >= sizeof(CmdVel)) {
        memcpy(&rx_.cmd_vel, p, sizeof(CmdVel));
        rx_.cmd_vel_fresh    = true;
        rx_.cmd_vel_stamp_ms = now;
        rx_.any_cmd_vel_seen = true;
      }
      break;
    case MSG_J2A_ARM_CMD:
      if (len >= sizeof(ArmCmd)) {
        memcpy(&rx_.arm_cmd, p, sizeof(ArmCmd));
        rx_.arm_cmd_fresh    = true;
        rx_.arm_cmd_stamp_ms = now;
        rx_.any_arm_cmd_seen = true;
      }
      break;
    case MSG_J2A_CONTROL_MODE:
      if (len >= 1) {
        rx_.control_mode          = p[0];
        rx_.control_mode_fresh    = true;
        rx_.control_mode_stamp_ms = now;
      }
      break;
    case MSG_J2A_SYSTEM_CMD:
      if (len >= 1) {
        rx_.system_cmd          = p[0];
        rx_.system_cmd_fresh    = true;
        rx_.system_cmd_stamp_ms = now;
      }
      break;
    case MSG_J2A_RELAY_CMD:
      if (len >= 1) {
        rx_.relay_cmd          = p[0];
        rx_.relay_cmd_fresh    = true;
        rx_.relay_cmd_stamp_ms = now;
      }
      break;
    default:
      stats_.rx_unknown++;
      if (dbg_) {
        dbg_->print(F("LINK: unknown type 0x"));
        dbg_->println(type, HEX);
      }
      break;
  }
}

bool MsdProtocolLink::sendRaw(uint8_t type, const void * payload, uint8_t len)
{
  if (!stream_) {return false;}
  uint8_t buf[protocol::MAX_FRAME_SIZE];
  const uint16_t seq = tx_seq_++;
  const size_t n = protocol::encode_frame(
    type, seq,
    (const uint8_t *)payload, len,
    buf, sizeof(buf));
  if (n == 0) {return false;}
  stream_->write(buf, n);
  stats_.tx_frames++;
  return true;
}

bool MsdProtocolLink::sendMcuHeartbeat(uint32_t mcu_time_ms, uint8_t system_state,
                                       uint8_t driver_variant)
{
  protocol::McuHeartbeat hb;
  hb.mcu_time_ms    = mcu_time_ms;
  hb.system_state   = system_state;
  hb.driver_variant = driver_variant;
  return sendRaw(protocol::MSG_A2J_HEARTBEAT, &hb, (uint8_t)sizeof(hb));
}

bool MsdProtocolLink::sendEncoder(const protocol::Encoder & enc)
{
  return sendRaw(protocol::MSG_A2J_ENCODER, &enc, (uint8_t)sizeof(enc));
}

bool MsdProtocolLink::sendImu(const protocol::Imu & imu)
{
  return sendRaw(protocol::MSG_A2J_IMU, &imu, (uint8_t)sizeof(imu));
}

bool MsdProtocolLink::sendEnv(const protocol::EnvSensor & env)
{
  return sendRaw(protocol::MSG_A2J_ENV, &env, (uint8_t)sizeof(env));
}

bool MsdProtocolLink::sendControlState(uint8_t control_mode, uint8_t comm_state,
                                       uint8_t motor_state, uint8_t arm_state)
{
  protocol::ControlState cs;
  cs.control_mode = control_mode;
  cs.comm_state   = comm_state;
  cs.motor_state  = motor_state;
  cs.arm_state    = arm_state;
  return sendRaw(protocol::MSG_A2J_CONTROL_STATE, &cs, (uint8_t)sizeof(cs));
}

bool MsdProtocolLink::sendError(uint32_t error_flags)
{
  protocol::ErrorState es;
  es.error_flags = error_flags;
  return sendRaw(protocol::MSG_A2J_ERROR, &es, (uint8_t)sizeof(es));
}

bool MsdProtocolLink::sendSbusJoy(const protocol::SbusChannels & sbus)
{
  return sendRaw(protocol::MSG_A2J_SBUS_JOY, &sbus, (uint8_t)sizeof(sbus));
}

bool MsdProtocolLink::sendBattery(const protocol::BatteryStatus & bat)
{
  return sendRaw(protocol::MSG_A2J_BATTERY, &bat, (uint8_t)sizeof(bat));
}

}  // namespace msd_arduino
