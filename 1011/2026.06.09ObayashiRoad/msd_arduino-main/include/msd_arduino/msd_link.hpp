// msd_link.hpp — high-level protocol manager (ROS2 side).
//
// Wraps a SerialPort and the binary frame parser/encoder. Holds the most
// recent values received from the Arduino in a `LinkRxState` struct so the
// node code can read them as plain variables, and exposes typed senders
// for outbound messages. Optional per-message callbacks let the node
// publish ROS topics immediately when a frame arrives.

#ifndef MSD_ARDUINO__MSD_LINK_HPP_
#define MSD_ARDUINO__MSD_LINK_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "msd_arduino/protocol.hpp"
#include "msd_arduino/serial_port.hpp"

namespace msd_arduino
{

// Latest values received from the Arduino. *_fresh flags become true
// when a new frame of that type is dispatched and stay true until the
// application clears them.
struct LinkRxState
{
  protocol::McuHeartbeat mcu_hb{};
  protocol::Encoder      encoder{};
  protocol::Imu          imu{};
  protocol::EnvSensor    env{};
  protocol::ControlState   ctrl{};
  protocol::SbusChannels   sbus{};
  protocol::BatteryStatus  battery{};
  protocol::MotorStatus    motor_status{};
  protocol::SystemStatus   system_status{};
  uint32_t error_flags = 0;

  bool mcu_hb_fresh   = false;
  bool encoder_fresh  = false;
  bool imu_fresh      = false;
  bool env_fresh      = false;
  bool ctrl_fresh     = false;
  bool sbus_fresh     = false;
  bool battery_fresh  = false;
  bool motor_status_fresh = false;
  bool system_status_fresh = false;
  bool error_fresh    = false;
};

struct LinkStats
{
  uint32_t rx_frames     = 0;
  uint32_t tx_frames     = 0;
  uint32_t rx_crc_errs   = 0;
  uint32_t rx_frame_errs = 0;
  uint32_t rx_unknown    = 0;
};

class MsdSerialLink
{
public:
  using LogSink = std::function<void(const std::string &)>;

  using CbMcuHeartbeat = std::function<void(const protocol::McuHeartbeat &)>;
  using CbEncoder      = std::function<void(const protocol::Encoder &)>;
  using CbImu          = std::function<void(const protocol::Imu &)>;
  using CbEnv          = std::function<void(const protocol::EnvSensor &)>;
  using CbControl      = std::function<void(const protocol::ControlState &)>;
  using CbError        = std::function<void(uint32_t)>;
  using CbSbusJoy      = std::function<void(const protocol::SbusChannels &)>;
  using CbBattery      = std::function<void(const protocol::BatteryStatus &)>;
  using CbMotorStatus  = std::function<void(const protocol::MotorStatus &)>;
  using CbSystemStatus = std::function<void(const protocol::SystemStatus &)>;

  MsdSerialLink();

  // Open the underlying serial port. Safe to call again to reopen.
  bool open(const std::string & port, int baud, int post_open_delay_ms = 2000);
  void close();
  bool is_open() const;
  const std::string & last_error() const {return last_error_;}

  // Drain RX buffer and dispatch any complete frames. Returns the number
  // of frames dispatched in this call.
  int update();

  // ---- Outbound senders (Jetson → Arduino) ----
  bool sendHeartbeat(uint32_t host_ms, uint8_t state);
  bool sendCmdVel(int16_t linear_mmps, int16_t angular_mradps);
  bool sendArmCmd(int16_t cyl1, int16_t cyl2, uint8_t mode);
  bool sendControlMode(uint8_t mode);
  bool sendSystemCmd(uint8_t cmd);
  bool sendRelayCmd(uint8_t state);
  bool sendRaw(uint8_t type, const void * payload, uint8_t len);

  // ---- State / stats ----
  const LinkRxState & rx() const {return rx_;}
  LinkRxState       & rx()       {return rx_;}
  const LinkStats   & stats() const {return stats_;}

  // ---- Optional callbacks ----
  void onMcuHeartbeat(CbMcuHeartbeat cb) {cb_hb_   = std::move(cb);}
  void onEncoder(CbEncoder cb)           {cb_enc_  = std::move(cb);}
  void onImu(CbImu cb)                   {cb_imu_  = std::move(cb);}
  void onEnv(CbEnv cb)                   {cb_env_  = std::move(cb);}
  void onControlState(CbControl cb)      {cb_ctrl_ = std::move(cb);}
  void onError(CbError cb)               {cb_err_  = std::move(cb);}
  void onSbusJoy(CbSbusJoy cb)           {cb_sbus_ = std::move(cb);}
  void onBattery(CbBattery cb)           {cb_bat_  = std::move(cb);}
  void onMotorStatus(CbMotorStatus cb)   {cb_mstat_ = std::move(cb);}
  void onSystemStatus(CbSystemStatus cb) {cb_sysstat_ = std::move(cb);}

  void setLogSink(LogSink sink) {log_sink_ = std::move(sink);}

private:
  void dispatch_(int & frame_count);
  void log_(const std::string & msg);

  SerialPort serial_;
  protocol::FrameParser parser_;
  uint16_t   tx_seq_;
  LinkRxState rx_;
  LinkStats   stats_;
  std::string last_error_;

  CbMcuHeartbeat cb_hb_;
  CbEncoder      cb_enc_;
  CbImu          cb_imu_;
  CbEnv          cb_env_;
  CbControl      cb_ctrl_;
  CbError        cb_err_;
  CbSbusJoy      cb_sbus_;
  CbBattery      cb_bat_;
  CbMotorStatus  cb_mstat_;
  CbSystemStatus cb_sysstat_;
  LogSink        log_sink_;
};

}  // namespace msd_arduino

#endif  // MSD_ARDUINO__MSD_LINK_HPP_
