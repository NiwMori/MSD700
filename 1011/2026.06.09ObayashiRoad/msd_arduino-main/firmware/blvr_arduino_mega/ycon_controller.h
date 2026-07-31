// ycon_controller.h — I2C bridge to the M5Stack Tough ycon controller.
//
// The M5 Tough is an I2C slave at 0x55. Arduino Mega is the I2C master:
//   - requestFrom(0x55) reads the latest ycon joystick/control packet
//   - beginTransmission(0x55) writes one compact feedback page
//
// Keep the packet layouts compatible with the existing `msd_controller`
// Tough/M5Stack firmware:
//   ToughControlData: 16 bytes, read by Arduino via requestFrom()
//   SensorData:       24 bytes, written by Arduino and forwarded to M5Stack

#ifndef MSD_BLVR_YCON_CONTROLLER_H_
#define MSD_BLVR_YCON_CONTROLLER_H_

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

namespace msd_arduino
{

static const uint8_t YCON_I2C_ADDRESS = 0x55;

#pragma pack(push, 1)

struct YconControlData
{
  int16_t linear_x;      // -1000..+1000
  int16_t angular_z;     // -1000..+1000
  int16_t arm;           // -1000..+1000
  int16_t bucket;        // -1000..+1000
  uint8_t valid;         // 1 while the Tough has fresh joystick data
  uint8_t estop;         // 1 requests stop
  uint16_t seq;
  uint32_t updated_ms;   // Tough-side millis()
};

struct YconSensorData
{
  int16_t battery_V;          // 0.01 V
  int16_t battery_I;          // 0.01 A
  int16_t battery_SOC;        // 0.1 %
  int16_t battery_temp_max;   // 0.1 C
  int16_t battery_temp_min;   // 0.1 C
  uint16_t soc_mAh;
  int16_t bnoX;               // 0.1 deg, euler x
  int16_t bnoY;               // 0.1 deg, euler y
  int16_t bnoZ;               // 0.1 deg, euler z
  int16_t temp;               // 0.1 C
  int16_t humid;              // 0.1 %
  int16_t press;              // 0.1 hPa
};

#pragma pack(pop)

struct YconFeedbackSnapshot
{
  uint32_t mcu_time_ms = 0;
  uint32_t error_flags = 0;
  uint8_t active_source = 0;
  bool relay_on = false;
  bool estop = false;
  bool ycon_valid = false;
  bool imu_valid = false;
  bool env_valid = false;
  bool battery_valid = false;
  uint8_t soc_percent = 0;

  int16_t applied_linear_mmps = 0;
  int16_t applied_angular_mradps = 0;
  int16_t cyl1_01mm = 0;
  int16_t cyl2_01mm = 0;

  int16_t imu_euler_x_mdeg = 0;
  int16_t imu_euler_y_mdeg = 0;
  int16_t imu_euler_z_mdeg = 0;
  int16_t imu_quat_w_1e4 = 0;
  int16_t imu_quat_x_1e4 = 0;
  int16_t imu_quat_y_1e4 = 0;
  int16_t imu_quat_z_1e4 = 0;

  uint16_t battery_voltage_10mv = 0;
  int16_t battery_current_10ma = 0;
  uint16_t soc_mah = 0;
  int16_t battery_temp_max_01c = 0;
  int16_t battery_temp_min_01c = 0;
  int16_t env_temp_cdeg = 0;
  uint16_t env_humidity_cpercent = 0;
  uint16_t env_pressure_hpa = 0;

  int16_t motor_l_rpm = 0;
  int16_t motor_r_rpm = 0;
  int16_t torque_l_01pct = 0;
  int16_t torque_r_01pct = 0;
  int16_t temp_drv_l_01c = 0;
  int16_t temp_drv_r_01c = 0;
  uint16_t alarm_l = 0;
  uint16_t alarm_r = 0;
};

class YconController
{
public:
  bool begin(uint8_t i2c_addr = YCON_I2C_ADDRESS, TwoWire * wire = &Wire);
  bool update(uint32_t now_ms, const YconFeedbackSnapshot & snapshot);

  bool isPresent() const { return present_; }
  bool isControlFresh(uint32_t now_ms, uint32_t timeout_ms) const;
  const YconControlData & control() const { return control_; }
  uint32_t lastReadMs() const { return last_read_ms_; }
  uint16_t errorCount() const { return error_count_; }
  uint16_t feedbackSeq() const { return feedback_seq_; }

private:
  bool readControl_(uint32_t now_ms);
  bool sendFeedback_(const YconFeedbackSnapshot & snapshot);
  void buildSensorData_(const YconFeedbackSnapshot & snapshot, YconSensorData & data);

  TwoWire * wire_ = &Wire;
  uint8_t addr_ = YCON_I2C_ADDRESS;
  bool present_ = false;
  YconControlData control_{};
  uint32_t last_read_ms_ = 0;
  uint32_t last_poll_ms_ = 0;
  uint32_t last_feedback_ms_ = 0;
  uint16_t feedback_seq_ = 0;
  uint16_t error_count_ = 0;
};

}  // namespace msd_arduino

#endif  // MSD_BLVR_YCON_CONTROLLER_H_
