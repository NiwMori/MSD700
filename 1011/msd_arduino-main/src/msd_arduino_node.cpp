// msd_arduino_node.cpp — ROS2 entry point.
//
// All serial / protocol handling is delegated to msd_arduino::MsdSerialLink
// (see include/msd_arduino/msd_link.hpp). This file is just the ROS glue:
// parameters, publishers, subscribers, and the periodic timers.
//
// Parameters:
//   port                 (string, default "/dev/ttyACM0")
//   baud                 (int,    default 115200)
//   open_delay_ms        (int,    default 2000)   waits out the Arduino
//                                                 bootloader after open
//   heartbeat_period_ms  (int,    default 50)
//   frame_id_imu         (string, default "imu_link")
//
// Topics:
//   Subscribe:
//     /msd/cmd_vel                  geometry_msgs/Twist
//     /msd/arm_cmd                  geometry_msgs/Vector3  (x=cyl1, y=cyl2, z=mode)
//     /msd/cmd/power_relay          std_msgs/UInt8  (0=OFF, 1=ON)  — power relay
//     /msd/cmd/relay_cmd            std_msgs/UInt8  legacy alias for /msd/cmd/power_relay
//     /msd/cmd/motor_alarm_reset    std_msgs/Empty                — BLV-R alarm reset
//     /msd/cmd/control              std_msgs/UInt8  (0=AUTO, 1=SBUS, 2=YCON) — source override
//   Publish:
//     /msd/motor/encoder_tick       msd_msgs/Encoder [left_tick, right_tick]
//     /msd/cylinder/tick            msd_msgs/Cylinder [cyl1, cyl2, cyl3]
//     /msd/imu/data        sensor_msgs/Imu
//     /msd/imu/euler       geometry_msgs/Vector3   (deg)
//     /msd/env             std_msgs/Float32MultiArray [temp_C, humidity_pct, pressure_Pa]
//     /msd/system_status   std_msgs/UInt8MultiArray  (15 byte packed:
//                              [0..3]  mcu_time_ms (LE u32)
//                              [4..7]  error_flags (LE u32)
//                              [8]     system_state
//                              [9]     driver_variant (0=BLV, 1=BLV-R)
//                              [10]    control_mode  (0=Jetson, 1=SBUS, 2=YCON)
//                              [11]    comm_state    (0=init, 1=ok, 2=lost)
//                              [12]    motor_state   (0=idle, 1=active)
//                              [13]    arm_state
//                              [14]    relay_state  (0=OFF, 1=ON))
//     /msd/battery         sensor_msgs/BatteryState
//     /msd/power/relay_state        std_msgs/UInt8
//     /msd/safety/hardware_estop    std_msgs/Bool
//     /msd/raw/*                    Arduino-origin raw aliases for AI logging
//     /msd/hw_estop        std_msgs/Bool
//     /sbus/channels       std_msgs/Float32MultiArray  CH1-8 normalized [-1,+1]
//     /sbus/cmd_vel        geometry_msgs/Twist
//     /sbus/arm_cmd        geometry_msgs/Vector3
//     /msd/motor/state         sensor_msgs/JointState (BLV-R, 50 Hz):
//                                  name     = ["left_wheel_motor", "right_wheel_motor"]
//                                  position = motor-shaft angle [rad]
//                                  velocity = motor-shaft angular velocity [rad/s]
//                                  effort   = torque [% of rated]
//     /msd/motor/diagnostics   std_msgs/Float32MultiArray (BLV-R, 5 Hz, 10 el:
//                                  [0..1]  L_Tdrv,  R_Tdrv   (°C)
//                                  [2..3]  L_Tmot,  R_Tmot   (°C)
//                                  [4..5]  L_alarm, R_alarm  (raw code)
//                                  [6..7]  L_odo,   R_odo    (krev)
//                                  [8..9]  L_Imain, R_Imain  (A))

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <msd_msgs/msg/cylinder.hpp>
#include <msd_msgs/msg/encoder.hpp>

#include "msd_arduino/msd_link.hpp"

namespace msd_arduino
{

namespace topic
{

constexpr const char * kCmdVel = "msd/cmd_vel";
constexpr const char * kArmCmd = "msd/arm_cmd";
constexpr const char * kPowerRelayCmd = "msd/cmd/power_relay";
constexpr const char * kPowerRelayCmdLegacy = "msd/cmd/relay_cmd";
constexpr const char * kMotorAlarmReset = "msd/cmd/motor_alarm_reset";
constexpr const char * kControl = "msd/cmd/control";

constexpr const char * kMotorEncoderTick = "msd/motor/encoder_tick";
constexpr const char * kCylinderTick = "msd/cylinder/tick";
constexpr const char * kImuData = "msd/imu/data";
constexpr const char * kImuEuler = "msd/imu/euler";
constexpr const char * kEnv = "msd/env";
constexpr const char * kBattery = "msd/battery";
constexpr const char * kSystemStatus = "msd/system_status";
constexpr const char * kPowerRelayState = "msd/power/relay_state";
constexpr const char * kPowerRelayStateLegacy = "msd/relay_state";
constexpr const char * kHardwareEstop = "msd/safety/hardware_estop";
constexpr const char * kHardwareEstopLegacy = "msd/hw_estop";
constexpr const char * kSbusChannels = "msd/sbus/channels";
constexpr const char * kSbusCmdVel = "msd/sbus/cmd_vel";
constexpr const char * kSbusArmCmd = "msd/sbus/arm_cmd";
constexpr const char * kSbusChannelsLegacy = "/sbus/channels";
constexpr const char * kSbusCmdVelLegacy = "/sbus/cmd_vel";
constexpr const char * kSbusArmCmdLegacy = "/sbus/arm_cmd";
constexpr const char * kMotorState = "msd/motor/state";
constexpr const char * kMotorDiagnostics = "msd/motor/diagnostics";

constexpr const char * kRawMotorEncoderTick = "msd/raw/motor/encoder_tick";
constexpr const char * kRawCylinderTick = "msd/raw/cylinder/tick";
constexpr const char * kRawImuData = "msd/raw/imu/data";
constexpr const char * kRawImuEuler = "msd/raw/imu/euler";
constexpr const char * kRawEnv = "msd/raw/env";
constexpr const char * kRawBattery = "msd/raw/battery";
constexpr const char * kRawSbusChannels = "msd/raw/sbus/channels";
constexpr const char * kRawSbusCmdVel = "msd/raw/sbus/cmd_vel";
constexpr const char * kRawSbusArmCmd = "msd/raw/sbus/arm_cmd";
constexpr const char * kRawMotorState = "msd/raw/motor/state";
constexpr const char * kRawMotorDiagnostics = "msd/raw/motor/diagnostics";

}  // namespace topic

class MsdArduinoNode : public rclcpp::Node
{
public:
  MsdArduinoNode()
  : Node("msd_arduino_node")
  {
    port_           = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_           = declare_parameter<int>("baud", 115200);
    open_delay_ms_  = declare_parameter<int>("open_delay_ms", 2000);
    int hb_period_ms = declare_parameter<int>("heartbeat_period_ms", 50);
    imu_frame_id_   = declare_parameter<std::string>("frame_id_imu", "imu_link");
    setup_publishers_();
    setup_subscribers_();
    setup_link_();

    rx_timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&MsdArduinoNode::on_rx_tick, this));
    hb_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, hb_period_ms)),
      std::bind(&MsdArduinoNode::on_hb_tick, this));

    start_time_ = now();
  }

private:
  // ===================================================================
  // Setup
  // ===================================================================
  void setup_publishers_()
  {
    pub_encoder_       = create_publisher<msd_msgs::msg::Encoder>(topic::kMotorEncoderTick, 10);
    pub_cylinder_tick_ = create_publisher<msd_msgs::msg::Cylinder>(topic::kCylinderTick, 10);
    pub_imu_           = create_publisher<sensor_msgs::msg::Imu>(topic::kImuData, 10);
    pub_imu_euler_     = create_publisher<geometry_msgs::msg::Vector3>(topic::kImuEuler, 10);
    pub_env_           = create_publisher<std_msgs::msg::Float32MultiArray>(topic::kEnv, 10);
    pub_battery_       = create_publisher<sensor_msgs::msg::BatteryState>(topic::kBattery, 10);
    pub_system_status_ = create_publisher<std_msgs::msg::UInt8MultiArray>(topic::kSystemStatus, 10);
    pub_relay_state_   =
      create_publisher<std_msgs::msg::UInt8>(topic::kPowerRelayStateLegacy, 10);
    pub_power_relay_state_ =
      create_publisher<std_msgs::msg::UInt8>(topic::kPowerRelayState, 10);
    pub_hw_estop_      = create_publisher<std_msgs::msg::Bool>(topic::kHardwareEstopLegacy, 10);
    pub_hardware_estop_ =
      create_publisher<std_msgs::msg::Bool>(topic::kHardwareEstop, 10);
    pub_sbus_channels_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(topic::kSbusChannelsLegacy, 10);
    pub_sbus_cmd_vel_  =
      create_publisher<geometry_msgs::msg::Twist>(topic::kSbusCmdVelLegacy, 10);
    pub_sbus_arm_cmd_  =
      create_publisher<geometry_msgs::msg::Vector3>(topic::kSbusArmCmdLegacy, 10);
    pub_msd_sbus_channels_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(topic::kSbusChannels, 10);
    pub_msd_sbus_cmd_vel_ =
      create_publisher<geometry_msgs::msg::Twist>(topic::kSbusCmdVel, 10);
    pub_msd_sbus_arm_cmd_ =
      create_publisher<geometry_msgs::msg::Vector3>(topic::kSbusArmCmd, 10);
    pub_motor_state_   = create_publisher<sensor_msgs::msg::JointState>(topic::kMotorState, 10);
    pub_motor_diag_    =
      create_publisher<std_msgs::msg::Float32MultiArray>(topic::kMotorDiagnostics, 10);

    pub_raw_encoder_       =
      create_publisher<msd_msgs::msg::Encoder>(topic::kRawMotorEncoderTick, 10);
    pub_raw_cylinder_tick_ =
      create_publisher<msd_msgs::msg::Cylinder>(topic::kRawCylinderTick, 10);
    pub_raw_imu_           = create_publisher<sensor_msgs::msg::Imu>(topic::kRawImuData, 10);
    pub_raw_imu_euler_     =
      create_publisher<geometry_msgs::msg::Vector3>(topic::kRawImuEuler, 10);
    pub_raw_env_           =
      create_publisher<std_msgs::msg::Float32MultiArray>(topic::kRawEnv, 10);
    pub_raw_battery_       =
      create_publisher<sensor_msgs::msg::BatteryState>(topic::kRawBattery, 10);
    pub_raw_sbus_channels_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(topic::kRawSbusChannels, 10);
    pub_raw_sbus_cmd_vel_  =
      create_publisher<geometry_msgs::msg::Twist>(topic::kRawSbusCmdVel, 10);
    pub_raw_sbus_arm_cmd_  =
      create_publisher<geometry_msgs::msg::Vector3>(topic::kRawSbusArmCmd, 10);
    pub_raw_motor_state_   =
      create_publisher<sensor_msgs::msg::JointState>(topic::kRawMotorState, 10);
    pub_raw_motor_diag_    =
      create_publisher<std_msgs::msg::Float32MultiArray>(topic::kRawMotorDiagnostics, 10);
  }

  void setup_subscribers_()
  {
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      topic::kCmdVel, 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!link_.is_open()) {return;}
        const int16_t lin = saturate_i16(msg->linear.x  * 1000.0);
        const int16_t ang = saturate_i16(msg->angular.z * 1000.0);
        if (!link_.sendCmdVel(lin, ang)) {
          RCLCPP_WARN(get_logger(), "sendCmdVel failed: %s",
            link_.last_error().c_str());
        }
      });

    sub_arm_cmd_ = create_subscription<geometry_msgs::msg::Vector3>(
      topic::kArmCmd, 10,
      [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (!link_.is_open()) {return;}
        // x,y are normalized -1..+1 from the topic; scale ×1000 for the
        // protocol (Arduino divides by 1000 to recover ±1.0).
        link_.sendArmCmd(
          saturate_i16(msg->x * 1000.0), saturate_i16(msg->y * 1000.0),
          static_cast<uint8_t>(std::clamp<int>(static_cast<int>(msg->z), 0, 255)));
      });

    // Power relay command: 0=OFF, non-zero=ON.  The old relay_cmd topic is
    // kept as a compatibility alias while new code uses power_relay.
    sub_power_relay_cmd_ = create_subscription<std_msgs::msg::UInt8>(
      topic::kPowerRelayCmd, 10,
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        handle_power_relay_cmd_(msg->data, topic::kPowerRelayCmd);
      });
    sub_relay_cmd_ = create_subscription<std_msgs::msg::UInt8>(
      topic::kPowerRelayCmdLegacy, 10,
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        handle_power_relay_cmd_(msg->data, topic::kPowerRelayCmdLegacy);
      });

    // Any message on this topic clears active BLV-R driver alarms
    // (Modbus maintenance command: write 1 → register 0x0180).
    sub_motor_alm_reset_ = create_subscription<std_msgs::msg::Empty>(
      topic::kMotorAlarmReset, 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        if (!link_.is_open()) {
          RCLCPP_WARN(get_logger(), "motor_alarm_reset: serial not open");
          return;
        }
        RCLCPP_INFO(get_logger(), "motor_alarm_reset -> SYS_CMD");
        if (!link_.sendSystemCmd(protocol::SYS_CMD_MOTOR_ALM_RESET)) {
          RCLCPP_WARN(get_logger(), "sendSystemCmd failed: %s",
            link_.last_error().c_str());
        }
      });

    // Control source override.  Without this topic the firmware uses automatic
    // Jetson/SBUS/YCON arbitration based on manual joystick intent.
    //   0 = AUTO arbitration (Jetson default, manual intent can take over)
    //   1 = force CTRL_SRC_SBUS (SBUS sticks drive the robot)
    //   2 = force CTRL_SRC_YCON (M5Stack Tough ycon controller)
    sub_control_ = create_subscription<std_msgs::msg::UInt8>(
      topic::kControl, 10,
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        if (!link_.is_open()) {
          RCLCPP_WARN(get_logger(), "cmd/control received but serial not open");
          return;
        }
        const uint8_t mode =
          (msg->data == protocol::CTRL_SRC_SBUS) ? protocol::CTRL_SRC_SBUS :
          (msg->data == protocol::CTRL_SRC_YCON) ? protocol::CTRL_SRC_YCON :
                                                   protocol::CTRL_SRC_JETSON;
        RCLCPP_INFO(get_logger(), "cmd/control -> %s",
                    mode == protocol::CTRL_SRC_SBUS ? "SBUS" :
                    mode == protocol::CTRL_SRC_YCON ? "YCON" : "AUTO");
        if (!link_.sendControlMode(mode)) {
          RCLCPP_WARN(get_logger(), "sendControlMode failed: %s",
            link_.last_error().c_str());
        }
      });
  }

  void setup_link_()
  {
    link_.setLogSink([this](const std::string & m) {
        RCLCPP_WARN(get_logger(), "%s", m.c_str());
      });

    // Cache + republish strategy:
    //   /msd/system_status is published whenever ANY of its component fields
    //   updates — heartbeat (0x20) / control (0x24) / error (0x25) on legacy
    //   firmware (BLV), or the combined SystemStatus (0x26) on BLV-R.
    link_.onMcuHeartbeat([this](const protocol::McuHeartbeat & hb) {
        cached_mcu_time_ms_   = hb.mcu_time_ms;
        cached_system_state_  = hb.system_state;
        cached_driver_variant_ = hb.driver_variant;
        publish_system_status_();
      });
    link_.onControlState([this](const protocol::ControlState & cs) {
        cached_control_mode_ = cs.control_mode;
        cached_comm_state_   = cs.comm_state;
        cached_motor_state_  = cs.motor_state;
        cached_arm_state_    = cs.arm_state;
        publish_system_status_();
      });
    link_.onError([this](uint32_t flags) {
        cached_error_flags_ = flags;
        publish_system_status_();
        log_error_flags_(flags);
      });
    link_.onSystemStatus([this](const protocol::SystemStatus & st) {
        cached_mcu_time_ms_    = st.mcu_time_ms;
        cached_error_flags_    = st.error_flags;
        cached_system_state_   = st.system_state;
        cached_driver_variant_ = st.driver_variant;
        cached_control_mode_   = st.control_mode;
        cached_comm_state_     = st.comm_state;
        cached_motor_state_    = st.motor_state;
        cached_arm_state_      = st.arm_state;
        cached_relay_state_    = st.relay_state;
        publish_system_status_();
        log_error_flags_(st.error_flags);
      });

    link_.onEncoder([this](const protocol::Encoder & e) {
        // Pass through the Arduino encoder payload without delta/spike
        // processing. Odometry owns delta calculation and filtering.
        msd_msgs::msg::Encoder m;
        m.header.stamp = now();
        m.header.frame_id = "base_footprint";
        m.mcu_time_ms = e.sample_time_ms;
        m.left_encoder = static_cast<int64_t>(e.left_motor_count);
        m.right_encoder = static_cast<int64_t>(e.right_motor_count);
        pub_encoder_->publish(m);
        pub_raw_encoder_->publish(m);

        msd_msgs::msg::Cylinder cyl;
        cyl.cylinder1 = static_cast<double>(e.cyl1_count);
        cyl.cylinder2 = static_cast<double>(e.cyl2_count);
        cyl.cylinder3 = 0.0;
        pub_cylinder_tick_->publish(cyl);
        pub_raw_cylinder_tick_->publish(cyl);
      });
    link_.onImu([this](const protocol::Imu & imu) {publish_imu_(imu);});
    link_.onEnv([this](const protocol::EnvSensor & e) {publish_env_(e);});
    link_.onBattery([this](const protocol::BatteryStatus & bat) {
        publish_battery_(bat);
      });
    link_.onMotorStatus([this](const protocol::MotorStatus & ms) {
        publish_motor_(ms);
      });
    link_.onSbusJoy([this](const protocol::SbusChannels & sbus) {
        publish_sbus_(sbus);
      });

    RCLCPP_INFO(get_logger(),
      "Opening %s @ %d (waiting %d ms for Arduino bootloader)...",
      port_.c_str(), baud_, open_delay_ms_);
    if (!link_.open(port_, baud_, open_delay_ms_)) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s",
        port_.c_str(), link_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial port ready");
    }
  }

  // ===================================================================
  // Publishers (helpers for the more complex conversions)
  // ===================================================================
  void publish_imu_(const protocol::Imu & imu)
  {
    // --- sensor_msgs/Imu with quaternion directly from BNO055 ---
    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = now();
    msg.header.frame_id = imu_frame_id_;

    constexpr double kQScale = 1.0e-4;  // int16 → unit quaternion
    msg.orientation.w = imu.quat_w_1e4 * kQScale;
    msg.orientation.x = imu.quat_x_1e4 * kQScale;
    msg.orientation.y = imu.quat_y_1e4 * kQScale;
    msg.orientation.z = imu.quat_z_1e4 * kQScale;

    // BNO055 fused output is orientation only.  Give EKF users a conservative
    // non-zero orientation covariance, and mark accel/gyro as unavailable.
    msg.orientation_covariance[0] = 0.05 * 0.05;  // roll variance [rad^2]
    msg.orientation_covariance[4] = 0.05 * 0.05;  // pitch variance [rad^2]
    msg.orientation_covariance[8] = 0.10 * 0.10;  // yaw variance [rad^2]
    msg.linear_acceleration_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0]    = -1.0;
    pub_imu_->publish(msg);
    pub_raw_imu_->publish(msg);

    // --- Euler in plain degrees on a separate topic ---
    geometry_msgs::msg::Vector3 e;
    e.x = imu.euler_x_mdeg * 1.0e-3;
    e.y = imu.euler_y_mdeg * 1.0e-3;
    e.z = imu.euler_z_mdeg * 1.0e-3;
    pub_imu_euler_->publish(e);
    pub_raw_imu_euler_->publish(e);
  }

  // Combined motor topic — diagnostics from the MotorStatus frame.
  void publish_motor_(const protocol::MotorStatus & ms)
  {
    // /msd/motor/state — sensor_msgs/JointState for control feedback.
    // Motor-shaft angles, derived from the BLV-R 32-bit step counter using
    // the driver's default electronic-gear ratio (36000 step/rev).  If the
    // user changes ユーザー位置単位設定 (manual §1-2) on the driver, update
    // STEPS_PER_MOTOR_REV below to keep position[] in radians.
    constexpr double STEPS_PER_MOTOR_REV = 36000.0;
    constexpr double TWO_PI              = 6.283185307179586;
    constexpr double STEP_TO_RAD         = TWO_PI / STEPS_PER_MOTOR_REV;

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name     = {"left_wheel_motor", "right_wheel_motor"};
    js.position = {
      static_cast<double>(ms.pos_l_step) * STEP_TO_RAD,
      static_cast<double>(ms.pos_r_step) * STEP_TO_RAD};
    js.velocity = {};
    js.effort = {
      static_cast<double>(ms.torque_l_01pct) * 0.1,    // % of rated torque
      static_cast<double>(ms.torque_r_01pct) * 0.1};
    pub_motor_state_->publish(js);
    pub_raw_motor_state_->publish(js);

    // /msd/motor/diagnostics — slower-rate housekeeping metrics.
    //   [0..1]  L_Tdrv, R_Tdrv  (°C)
    //   [2..3]  L_Tmot, R_Tmot  (°C)
    //   [4..5]  L_alarm, R_alarm (raw code; 0 = OK)
    //   [6..7]  L_odo, R_odo    (krev — total motor-shaft revolutions / 1000)
    //   [8..9]  L_Imain, R_Imain (A — main supply current per driver)
    std_msgs::msg::Float32MultiArray diag;
    diag.data = {
      static_cast<float>(ms.temp_drv_l_01c) * 0.1f,
      static_cast<float>(ms.temp_drv_r_01c) * 0.1f,
      static_cast<float>(ms.temp_mot_l_01c) * 0.1f,
      static_cast<float>(ms.temp_mot_r_01c) * 0.1f,
      static_cast<float>(ms.alarm_l),
      static_cast<float>(ms.alarm_r),
      static_cast<float>(ms.odo_l_01krev) * 0.0001f,    // 0.1 krev → krev
      static_cast<float>(ms.odo_r_01krev) * 0.0001f,
      static_cast<float>(ms.imain_l_001a) * 0.001f,     // mA → A
      static_cast<float>(ms.imain_r_001a) * 0.001f};
    pub_motor_diag_->publish(diag);
    pub_raw_motor_diag_->publish(diag);
  }

  // Combined system status — heartbeat + control + error in one byte array.
  // Called whenever any of the source fields updates (BLV: 0x20/24/25;
  // BLV-R: 0x26 SystemStatus single frame).
  void publish_system_status_()
  {
    std_msgs::msg::UInt8MultiArray m;
    m.data.resize(15);
    // [0..3]  mcu_time_ms (LE u32)
    m.data[0] = static_cast<uint8_t>( cached_mcu_time_ms_        & 0xFF);
    m.data[1] = static_cast<uint8_t>((cached_mcu_time_ms_ >> 8 ) & 0xFF);
    m.data[2] = static_cast<uint8_t>((cached_mcu_time_ms_ >> 16) & 0xFF);
    m.data[3] = static_cast<uint8_t>((cached_mcu_time_ms_ >> 24) & 0xFF);
    // [4..7]  error_flags (LE u32)
    m.data[4] = static_cast<uint8_t>( cached_error_flags_        & 0xFF);
    m.data[5] = static_cast<uint8_t>((cached_error_flags_ >> 8 ) & 0xFF);
    m.data[6] = static_cast<uint8_t>((cached_error_flags_ >> 16) & 0xFF);
    m.data[7] = static_cast<uint8_t>((cached_error_flags_ >> 24) & 0xFF);
    // [8..13] system / driver / control / comm / motor / arm
    m.data[8]  = cached_system_state_;
    m.data[9]  = cached_driver_variant_;
    m.data[10] = cached_control_mode_;
    m.data[11] = cached_comm_state_;
    m.data[12] = cached_motor_state_;
    m.data[13] = cached_arm_state_;
    m.data[14] = cached_relay_state_;
    pub_system_status_->publish(m);

    std_msgs::msg::UInt8 relay;
    relay.data = cached_relay_state_;
    pub_relay_state_->publish(relay);
    pub_power_relay_state_->publish(relay);

    std_msgs::msg::Bool hw_estop;
    hw_estop.data = (cached_error_flags_ & protocol::ERR_HW_ESTOP) != 0;
    pub_hw_estop_->publish(hw_estop);
    pub_hardware_estop_->publish(hw_estop);

    if (cached_control_mode_ != last_logged_control_mode_) {
      last_logged_control_mode_ = cached_control_mode_;
      RCLCPP_INFO(get_logger(), "MCU control mode -> %s (%u)",
        control_mode_name_(cached_control_mode_), cached_control_mode_);
    }
  }

  void publish_battery_(const protocol::BatteryStatus & bat)
  {
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "battery";
    // Total voltage (series: mod1 + mod2). Units: V.
    msg.voltage = (bat.mod1_voltage_10mv + bat.mod2_voltage_10mv) * 0.01f;
    // Current (parallel assumption → sum). Units: A.
    msg.current = (bat.mod1_current_10ma + bat.mod2_current_10ma) * 0.01f;
    // Temperature: average of max temps from both modules.
    msg.temperature =
      (bat.mod1_temp_max_01c + bat.mod2_temp_max_01c) * 0.5f * 0.1f;
    // Charge in Ah.
    msg.charge = bat.soc_mah * 0.001f;
    msg.percentage = bat.soc_percent * 0.01f;
    msg.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    // Per-cell voltages: report each module as a "cell" for detail.
    msg.cell_voltage.resize(2);
    msg.cell_voltage[0] = bat.mod1_voltage_10mv * 0.01f;
    msg.cell_voltage[1] = bat.mod2_voltage_10mv * 0.01f;
    // Temperature detail in cell_temperature (non-standard but useful).
    msg.cell_temperature.resize(6);
    msg.cell_temperature[0] = bat.mod1_temp_max_01c * 0.1f;
    msg.cell_temperature[1] = bat.mod1_temp_fet_01c * 0.1f;
    msg.cell_temperature[2] = bat.mod1_temp_min_01c * 0.1f;
    msg.cell_temperature[3] = bat.mod2_temp_max_01c * 0.1f;
    msg.cell_temperature[4] = bat.mod2_temp_fet_01c * 0.1f;
    msg.cell_temperature[5] = bat.mod2_temp_min_01c * 0.1f;
    pub_battery_->publish(msg);
    pub_raw_battery_->publish(msg);
  }

  void publish_sbus_(const protocol::SbusChannels & sbus)
  {
    constexpr double kScale = 1.0e-4;
    constexpr double kSbusMaxLinearMps = 0.46;
    constexpr double kSbusMaxAngularRps = 0.75;
    constexpr double kSbusMotorDeadband = 0.30;
    constexpr double kSbusCylinderThreshold = 0.75;
    constexpr int kChLinear = 1;   // CH2
    constexpr int kChAngular = 3;  // CH4
    constexpr int kChCyl1 = 2;     // CH3
    constexpr int kChCyl2 = 0;     // CH1

    auto ch = [&sbus](int i) -> double {
      return static_cast<double>(sbus.ch[i]) * kScale;
    };
    auto motor_axis = [](double v) -> double {
      return (std::abs(v) < kSbusMotorDeadband) ? 0.0 : v;
    };
    auto cyl_axis = [](double v) -> double {
      if (v >= kSbusCylinderThreshold) {return 1.0;}
      if (v <= -kSbusCylinderThreshold) {return -1.0;}
      return 0.0;
    };

    std_msgs::msg::Float32MultiArray channels;
    channels.data.resize(8);
    for (size_t i = 0; i < channels.data.size(); ++i) {
      channels.data[i] = static_cast<float>(ch(static_cast<int>(i)));
    }
    pub_sbus_channels_->publish(channels);
    pub_msd_sbus_channels_->publish(channels);
    pub_raw_sbus_channels_->publish(channels);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = motor_axis(ch(kChLinear)) * kSbusMaxLinearMps;
    cmd_vel.angular.z = motor_axis(ch(kChAngular)) * kSbusMaxAngularRps;
    pub_sbus_cmd_vel_->publish(cmd_vel);
    pub_msd_sbus_cmd_vel_->publish(cmd_vel);
    pub_raw_sbus_cmd_vel_->publish(cmd_vel);

    geometry_msgs::msg::Vector3 arm_cmd;
    arm_cmd.x = cyl_axis(ch(kChCyl1));
    arm_cmd.y = cyl_axis(ch(kChCyl2));
    arm_cmd.z = 1.0;
    pub_sbus_arm_cmd_->publish(arm_cmd);
    pub_msd_sbus_arm_cmd_->publish(arm_cmd);
    pub_raw_sbus_arm_cmd_->publish(arm_cmd);

  }

  void log_error_flags_(uint32_t flags)
  {
    if (flags == last_error_flags_) { return; }  // only log on change
    last_error_flags_ = flags;
    if (flags == 0) {
      RCLCPP_INFO(get_logger(), "MCU errors cleared");
      return;
    }
    std::string msg = "MCU errors: ";
    if (flags & protocol::ERR_CRC)        { msg += "[CRC] "; }
    if (flags & protocol::ERR_CMD_VEL_TO) { msg += "[CMD_VEL_TIMEOUT] "; }
    if (flags & protocol::ERR_ARM_TO)     { msg += "[ARM_TIMEOUT] "; }
    if (flags & protocol::ERR_HB_LOST)    { msg += "[HEARTBEAT_LOST] "; }
    if (flags & protocol::ERR_SENSOR)     { msg += "[SENSOR] "; }
    if (flags & protocol::ERR_ACTUATOR)   { msg += "[ACTUATOR] "; }
    if (flags & protocol::ERR_FRAME)      { msg += "[FRAME] "; }
    if (flags & protocol::ERR_ESTOP)      { msg += "[E-STOP] "; }
    if (flags & protocol::ERR_SBUS)       { msg += "[SBUS_NO_DATA] "; }
    if (flags & protocol::ERR_HW_ESTOP)   { msg += "[HW_ESTOP_D22] "; }
    if (flags & protocol::ERR_YCON)       { msg += "[YCON_NO_DATA] "; }
    RCLCPP_WARN(get_logger(), "%s (0x%04X)", msg.c_str(), flags);
  }

  static const char * control_mode_name_(uint8_t mode)
  {
    switch (mode) {
      case protocol::CTRL_SRC_SBUS:
        return "SBUS";
      case protocol::CTRL_SRC_YCON:
        return "YCON";
      case protocol::CTRL_SRC_JETSON:
      default:
        return "JETSON";
    }
  }

  void publish_env_(const protocol::EnvSensor & e)
  {
    // Single combined topic for all on-board environmental sensor values.
    //   [0] temperature [°C]
    //   [1] relative humidity [%]      (0–100)
    //   [2] absolute pressure [Pa]
    std_msgs::msg::Float32MultiArray env;
    env.data = {
      static_cast<float>(e.temperature_cdeg) * 0.01f,
      static_cast<float>(e.humidity_cpercent) * 0.01f,
      static_cast<float>(e.pressure_pa)};
    pub_env_->publish(env);
    pub_raw_env_->publish(env);
  }

  void handle_power_relay_cmd_(uint8_t data, const char * topic_name)
  {
    if (!link_.is_open()) {
      RCLCPP_WARN(get_logger(), "%s received but serial not open", topic_name);
      return;
    }
    const uint8_t state = (data != 0) ? 1u : 0u;
    RCLCPP_INFO(get_logger(), "%s -> %s", topic_name, state ? "ON" : "OFF");
    if (!link_.sendRelayCmd(state)) {
      RCLCPP_WARN(get_logger(), "sendRelayCmd failed: %s",
        link_.last_error().c_str());
    }
  }

  // ===================================================================
  // Periodic ticks
  // ===================================================================
  void on_rx_tick()
  {
    if (!link_.is_open()) {
      // Try to reopen periodically (5 ms × 200 = once per second).
      if ((reopen_counter_++ % 200) == 0) {
        RCLCPP_INFO(get_logger(), "Trying to reopen %s...", port_.c_str());
        if (link_.open(port_, baud_, open_delay_ms_)) {
          RCLCPP_INFO(get_logger(), "Reopened %s", port_.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Reopen failed: %s",
            link_.last_error().c_str());
        }
      }
      return;
    }

    int n = link_.update();
    if (n > 0 && !first_rx_logged_) {
      first_rx_logged_ = true;
      RCLCPP_INFO(get_logger(),
        "First frame received from Arduino — link is alive");
    }
    if (!first_rx_logged_) {
      // Once per ~second, remind the user.
      if ((rx_tick_count_++ % 200) == 0 && rx_tick_count_ > 1) {
        RCLCPP_INFO(get_logger(),
          "Waiting for first frame from Arduino on %s ...", port_.c_str());
      }
    }
  }

  void on_hb_tick()
  {
    if (!link_.is_open()) {return;}
    auto elapsed = now() - start_time_;
    uint32_t host_ms = static_cast<uint32_t>(
      (elapsed.nanoseconds() / 1'000'000LL) & 0xFFFFFFFFLL);
    link_.sendHeartbeat(host_ms, 1);
  }

  static int16_t saturate_i16(double v)
  {
    if (v >  32767.0) {return 32767;}
    if (v < -32768.0) {return -32768;}
    return static_cast<int16_t>(std::lround(v));
  }

  // ===================================================================
  // Members
  // ===================================================================
  std::string port_;
  int         baud_ = 115200;
  int         open_delay_ms_ = 2000;
  std::string imu_frame_id_;
  rclcpp::Time start_time_;
  int          reopen_counter_  = 0;
  int          rx_tick_count_   = 0;
  bool         first_rx_logged_ = false;
  uint32_t     last_error_flags_ = 0xFFFFFFFF;  // force first log

  MsdSerialLink link_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_arm_cmd_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr        sub_power_relay_cmd_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr        sub_relay_cmd_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr        sub_motor_alm_reset_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr        sub_control_;

  rclcpp::Publisher<msd_msgs::msg::Encoder>::SharedPtr             pub_encoder_;
  rclcpp::Publisher<msd_msgs::msg::Cylinder>::SharedPtr            pub_cylinder_tick_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr        pub_imu_euler_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_env_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     pub_battery_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr     pub_system_status_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr               pub_relay_state_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr               pub_power_relay_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_hw_estop_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_hardware_estop_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_sbus_channels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          pub_sbus_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr        pub_sbus_arm_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_msd_sbus_channels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          pub_msd_sbus_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr        pub_msd_sbus_arm_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       pub_motor_state_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_motor_diag_;
  rclcpp::Publisher<msd_msgs::msg::Encoder>::SharedPtr             pub_raw_encoder_;
  rclcpp::Publisher<msd_msgs::msg::Cylinder>::SharedPtr            pub_raw_cylinder_tick_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_raw_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr        pub_raw_imu_euler_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_raw_env_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     pub_raw_battery_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_raw_sbus_channels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          pub_raw_sbus_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr        pub_raw_sbus_arm_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       pub_raw_motor_state_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   pub_raw_motor_diag_;

  // Cached state for combined publishers.
  uint32_t cached_mcu_time_ms_    = 0;
  uint32_t cached_error_flags_    = 0;
  uint8_t  cached_system_state_   = 0;
  uint8_t  cached_driver_variant_ = protocol::DRIVER_VARIANT_BLV;
  uint8_t  cached_control_mode_   = protocol::CTRL_SRC_JETSON;
  uint8_t  last_logged_control_mode_ = 0xff;
  uint8_t  cached_comm_state_     = protocol::COMM_INIT;
  uint8_t  cached_motor_state_    = 0;
  uint8_t  cached_arm_state_      = 0;
  uint8_t  cached_relay_state_    = 0;

  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
};

}  // namespace msd_arduino

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msd_arduino::MsdArduinoNode>());
  rclcpp::shutdown();
  return 0;
}
