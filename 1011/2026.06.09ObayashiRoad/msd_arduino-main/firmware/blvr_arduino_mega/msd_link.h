// msd_link.h — Arduino-side protocol manager (library-style).
//
// Wraps the wire-level FrameParser/encoder from protocol.h and exposes:
//   - a `LinkRxState` struct holding the most recent values received from
//     the host (cmd_vel, arm_cmd, host_hb, control_mode, system_cmd) plus
//     "fresh" flags and last-received timestamps;
//   - typed senders for every Arduino → Jetson message type;
//   - simple statistics counters.
//
// Usage in the .ino:
//   MsdProtocolLink g_link;
//   void setup() { g_link.begin(Serial); }
//   void loop()  { g_link.update();
//                  if (g_link.rx().cmd_vel_fresh) { ... } }

#ifndef MSD_BLV_LINK_H_
#define MSD_BLV_LINK_H_

#include <Arduino.h>
#include "protocol.h"

namespace msd_arduino
{

// Most-recent values received from the Jetson plus per-message metadata.
struct LinkRxState
{
  // Last decoded payloads.
  protocol::Heartbeat host_hb{};
  protocol::CmdVel    cmd_vel{};
  protocol::ArmCmd    arm_cmd{};
  uint8_t             control_mode = 0;
  uint8_t             system_cmd   = 0;
  uint8_t             relay_cmd    = 0;  // last received relay state

  // Set true when a frame of that type is dispatched. Caller clears.
  bool host_hb_fresh      = false;
  bool cmd_vel_fresh      = false;
  bool arm_cmd_fresh      = false;
  bool control_mode_fresh = false;
  bool system_cmd_fresh   = false;
  bool relay_cmd_fresh    = false;

  // millis() at the time the most recent frame of each type was received.
  uint32_t host_hb_stamp_ms      = 0;
  uint32_t cmd_vel_stamp_ms      = 0;
  uint32_t arm_cmd_stamp_ms      = 0;
  uint32_t control_mode_stamp_ms = 0;
  uint32_t system_cmd_stamp_ms   = 0;
  uint32_t relay_cmd_stamp_ms    = 0;

  // "Has a frame of this type ever been seen?" — used by watchdogs that
  // should not trip until the host has spoken at least once.
  bool any_host_hb_seen = false;
  bool any_cmd_vel_seen = false;
  bool any_arm_cmd_seen = false;
};

struct LinkStats
{
  uint32_t rx_frames     = 0;
  uint32_t tx_frames     = 0;
  uint32_t rx_crc_errs   = 0;
  uint32_t rx_frame_errs = 0;
  uint32_t rx_unknown    = 0;
};

class MsdProtocolLink
{
public:
  MsdProtocolLink();

  // Bind to a Stream for the host link. Caller is responsible for calling
  // `link_stream.begin(baud)` first.
  void begin(Stream & link_stream);

  // Drain RX bytes and dispatch any complete frames. Returns the number of
  // frames dispatched in this call.
  int update();

  // ---- Outbound senders (Arduino → Jetson) ----
  bool sendMcuHeartbeat(uint32_t mcu_time_ms, uint8_t system_state,
                        uint8_t driver_variant = protocol::DRIVER_VARIANT_BLV);
  bool sendEncoder(const protocol::Encoder & enc);
  bool sendImu(const protocol::Imu & imu);
  bool sendEnv(const protocol::EnvSensor & env);
  bool sendControlState(uint8_t control_mode, uint8_t comm_state,
                        uint8_t motor_state, uint8_t arm_state);
  bool sendError(uint32_t error_flags);
  bool sendSystemStatus(const protocol::SystemStatus & st);
  bool sendSbusJoy(const protocol::SbusChannels & sbus);
  bool sendBattery(const protocol::BatteryStatus & bat);
  bool sendMotorStatus(const protocol::MotorStatus & ms);
  bool sendRaw(uint8_t type, const void * payload, uint8_t len);

  // ---- State / stats access ----
  const LinkRxState & rx() const {return rx_;}
  LinkRxState       & rx()       {return rx_;}
  const LinkStats   & stats() const {return stats_;}

  // Optional: route warnings/errors to Serial1 (or any Stream).
  void setDebugSink(Stream * dbg) {dbg_ = dbg;}

private:
  void dispatch_();

  Stream *              stream_   = nullptr;
  Stream *              dbg_      = nullptr;
  protocol::FrameParser parser_;
  uint16_t              tx_seq_   = 0;
  LinkRxState           rx_;
  LinkStats             stats_;
};

}  // namespace msd_arduino

#endif  // MSD_BLV_LINK_H_
