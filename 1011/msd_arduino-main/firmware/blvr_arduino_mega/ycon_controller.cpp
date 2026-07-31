#include "ycon_controller.h"

namespace msd_arduino
{

namespace
{
static const uint32_t YCON_POLL_PERIOD_MS = 20;
static const uint32_t YCON_FEEDBACK_PERIOD_MS = 200;

int16_t sat_i16(int32_t v)
{
  if (v > 32767) { return 32767; }
  if (v < -32768) { return -32768; }
  return (int16_t)v;
}
}  // namespace

bool YconController::begin(uint8_t i2c_addr, TwoWire * wire)
{
  addr_ = i2c_addr;
  wire_ = wire ? wire : &Wire;
  present_ = false;
  control_ = {};
  last_read_ms_ = 0;
  last_poll_ms_ = 0;
  last_feedback_ms_ = 0;
  feedback_seq_ = 0;
  error_count_ = 0;
  return true;
}

bool YconController::update(uint32_t now_ms, const YconFeedbackSnapshot & snapshot)
{
  bool ok = true;
  if ((int32_t)(now_ms - last_poll_ms_) >= 0) {
    last_poll_ms_ = now_ms + YCON_POLL_PERIOD_MS;
    ok = readControl_(now_ms) && ok;
  }
  if ((int32_t)(now_ms - last_feedback_ms_) >= 0) {
    last_feedback_ms_ = now_ms + YCON_FEEDBACK_PERIOD_MS;
    ok = sendFeedback_(snapshot) && ok;
  }
  return ok;
}

bool YconController::isControlFresh(uint32_t now_ms, uint32_t timeout_ms) const
{
  return present_ && control_.valid && ((uint32_t)(now_ms - last_read_ms_) <= timeout_ms);
}

bool YconController::readControl_(uint32_t now_ms)
{
  const int read_size = (int)sizeof(YconControlData);
  int requested = wire_->requestFrom((int)addr_, read_size);
  int available = wire_->available();
  if (requested == read_size && available == read_size) {
    uint8_t * raw = (uint8_t *)&control_;
    for (int i = 0; i < read_size; ++i) {
      raw[i] = (uint8_t)wire_->read();
    }
    last_read_ms_ = now_ms;
    present_ = true;
    return true;
  }

  while (wire_->available()) {
    wire_->read();
  }
  ++error_count_;
  return false;
}

bool YconController::sendFeedback_(const YconFeedbackSnapshot & snapshot)
{
  YconSensorData data{};
  buildSensorData_(snapshot, data);
  wire_->beginTransmission(addr_);
  wire_->write((const uint8_t *)&data, sizeof(data));
  const uint8_t err = wire_->endTransmission();
  if (err == 0) {
    present_ = true;
    ++feedback_seq_;
    return true;
  }
  ++error_count_;
  return false;
}

void YconController::buildSensorData_(const YconFeedbackSnapshot & snapshot, YconSensorData & data)
{
  data.battery_V = sat_i16(snapshot.battery_voltage_10mv);
  data.battery_I = snapshot.battery_current_10ma;
  data.battery_SOC = (int16_t)snapshot.soc_percent * 10;
  data.battery_temp_max = snapshot.battery_temp_max_01c;
  data.battery_temp_min = snapshot.battery_temp_min_01c;
  data.soc_mAh = snapshot.soc_mah;
  data.bnoX = snapshot.imu_euler_x_mdeg / 100;
  data.bnoY = snapshot.imu_euler_y_mdeg / 100;
  data.bnoZ = snapshot.imu_euler_z_mdeg / 100;
  data.temp = snapshot.env_temp_cdeg / 10;
  data.humid = sat_i16((int32_t)snapshot.env_humidity_cpercent / 10);
  data.press = sat_i16((int32_t)snapshot.env_pressure_hpa * 10);
}

}  // namespace msd_arduino
