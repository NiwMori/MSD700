#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <msd_msgs/msg/cylinder.hpp>
#include <msd_msgs/msg/encoder.hpp>

namespace
{

constexpr uint32_t ERR_HW_ESTOP = 1u << 9;
constexpr const char * kPowerRelayCmd = "msd/cmd/power_relay";
constexpr const char * kPowerRelayCmdLegacy = "msd/cmd/relay_cmd";
constexpr const char * kPowerRelayState = "msd/power/relay_state";
constexpr const char * kPowerRelayStateLegacy = "msd/relay_state";
constexpr const char * kHardwareEstop = "msd/safety/hardware_estop";
constexpr const char * kHardwareEstopLegacy = "msd/hw_estop";

void put_u32_le(std::array<uint8_t, 15> & data, const std::size_t offset, const uint32_t value)
{
  data[offset + 0] = static_cast<uint8_t>(value & 0xffu);
  data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
  data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xffu);
  data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xffu);
}

}  // namespace

class MsdArduinoDummyNode : public rclcpp::Node
{
public:
  MsdArduinoDummyNode()
  : Node("msd_arduino_dummy_node")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    control_mode_ = declare_parameter<int>("control_mode", 0);
    relay_on_ = declare_parameter<bool>("relay_on", true);
    hw_estop_ = declare_parameter<bool>("hw_estop", false);

    pub_encoder_ = create_publisher<msd_msgs::msg::Encoder>("msd/motor/encoder_tick", 10);
    pub_cylinder_tick_ = create_publisher<msd_msgs::msg::Cylinder>("msd/cylinder/tick", 10);
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("msd/imu/data", 10);
    pub_imu_euler_ = create_publisher<geometry_msgs::msg::Vector3>("msd/imu/euler", 10);
    pub_env_ = create_publisher<std_msgs::msg::Float32MultiArray>("msd/env", 10);
    pub_battery_ = create_publisher<sensor_msgs::msg::BatteryState>("msd/battery", 10);
    pub_system_status_ = create_publisher<std_msgs::msg::UInt8MultiArray>("msd/system_status", 10);
    pub_power_relay_state_ = create_publisher<std_msgs::msg::UInt8>(kPowerRelayState, 10);
    pub_power_relay_state_legacy_ =
      create_publisher<std_msgs::msg::UInt8>(kPowerRelayStateLegacy, 10);
    pub_hw_estop_ = create_publisher<std_msgs::msg::Bool>(kHardwareEstopLegacy, 10);
    pub_hardware_estop_ = create_publisher<std_msgs::msg::Bool>(kHardwareEstop, 10);
    pub_motor_state_ = create_publisher<sensor_msgs::msg::JointState>("msd/motor/state", 10);
    pub_motor_diag_ = create_publisher<std_msgs::msg::Float32MultiArray>("msd/motor/diagnostics", 10);
    pub_sbus_channels_ = create_publisher<std_msgs::msg::Float32MultiArray>("msd/sbus/channels", 10);
    pub_sbus_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("msd/sbus/cmd_vel", 10);
    pub_sbus_arm_cmd_ = create_publisher<geometry_msgs::msg::Vector3>("msd/sbus/arm_cmd", 10);
    pub_raw_encoder_ = create_publisher<msd_msgs::msg::Encoder>("msd/raw/motor/encoder_tick", 10);
    pub_raw_cylinder_tick_ =
      create_publisher<msd_msgs::msg::Cylinder>("msd/raw/cylinder/tick", 10);
    pub_raw_imu_ = create_publisher<sensor_msgs::msg::Imu>("msd/raw/imu/data", 10);
    pub_raw_imu_euler_ = create_publisher<geometry_msgs::msg::Vector3>("msd/raw/imu/euler", 10);
    pub_raw_env_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("msd/raw/env", 10);
    pub_raw_battery_ = create_publisher<sensor_msgs::msg::BatteryState>("msd/raw/battery", 10);
    pub_raw_motor_state_ = create_publisher<sensor_msgs::msg::JointState>("msd/raw/motor/state", 10);
    pub_raw_motor_diag_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("msd/raw/motor/diagnostics", 10);
    pub_raw_sbus_channels_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("msd/raw/sbus/channels", 10);
    pub_raw_sbus_cmd_vel_ =
      create_publisher<geometry_msgs::msg::Twist>("msd/raw/sbus/cmd_vel", 10);
    pub_raw_sbus_arm_cmd_ =
      create_publisher<geometry_msgs::msg::Vector3>("msd/raw/sbus/arm_cmd", 10);

    sub_control_ = create_subscription<std_msgs::msg::UInt8>(
      "msd/cmd/control", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        control_mode_ = std::min<int>(msg->data, 2);
      });
    sub_power_relay_ = create_subscription<std_msgs::msg::UInt8>(
      kPowerRelayCmd, 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        relay_on_ = msg->data != 0;
      });
    sub_relay_legacy_ = create_subscription<std_msgs::msg::UInt8>(
      kPowerRelayCmdLegacy, 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        relay_on_ = msg->data != 0;
      });

    start_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MsdArduinoDummyNode::on_timer, this));
  }

private:
  void on_timer()
  {
    const double t = (now() - start_time_).seconds();
    const uint32_t t_ms = static_cast<uint32_t>(t * 1000.0);

    publish_encoder(t_ms);
    publish_imu(t);
    publish_env();
    publish_battery();
    publish_system_status(t_ms);
    publish_motor_state(t);
    publish_motor_diag(t);
    publish_sbus_echo();
  }

  void publish_encoder(const uint32_t t_ms)
  {
    msd_msgs::msg::Encoder encoder;
    encoder.header.stamp = now();
    encoder.header.frame_id = "base_footprint";
    encoder.mcu_time_ms = t_ms;
    encoder.left_encoder = 0;
    encoder.right_encoder = 0;
    pub_encoder_->publish(encoder);
    pub_raw_encoder_->publish(encoder);

    msd_msgs::msg::Cylinder cylinder;
    cylinder.cylinder1 = 47.0;
    cylinder.cylinder2 = 1547.0;
    cylinder.cylinder3 = 0.0;
    pub_cylinder_tick_->publish(cylinder);
    pub_raw_cylinder_tick_->publish(cylinder);
  }

  void publish_imu(const double t)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = "imu_link";
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.z = 0.01 * std::sin(t);
    imu.linear_acceleration.z = 9.80665;
    pub_imu_->publish(imu);
    pub_raw_imu_->publish(imu);

    geometry_msgs::msg::Vector3 euler;
    euler.z = 5.0 * std::sin(t * 0.2);
    pub_imu_euler_->publish(euler);
    pub_raw_imu_euler_->publish(euler);
  }

  void publish_env()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {25.0f, 45.0f, 101325.0f};
    pub_env_->publish(msg);
    pub_raw_env_->publish(msg);
  }

  void publish_battery()
  {
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.voltage = 24.0f;
    msg.current = relay_on_ ? 0.8f : 0.1f;
    msg.percentage = 0.75f;
    msg.present = true;
    pub_battery_->publish(msg);
    pub_raw_battery_->publish(msg);
  }

  void publish_system_status(const uint32_t t_ms)
  {
    std_msgs::msg::UInt8MultiArray msg;
    std::array<uint8_t, 15> data{};
    uint32_t errors = 0;
    if (hw_estop_ || !relay_on_) {
      errors |= ERR_HW_ESTOP;
    }
    put_u32_le(data, 0, t_ms);
    put_u32_le(data, 4, errors);
    data[8] = relay_on_ ? 1 : 0;
    data[9] = 1;
    data[10] = static_cast<uint8_t>(std::clamp(control_mode_, 0, 2));
    data[11] = 1;
    data[12] = relay_on_ ? 1 : 0;
    data[13] = 0;
    data[14] = relay_on_ ? 1 : 0;
    msg.data.assign(data.begin(), data.end());
    pub_system_status_->publish(msg);

    std_msgs::msg::UInt8 relay;
    relay.data = relay_on_ ? 1 : 0;
    pub_power_relay_state_->publish(relay);
    pub_power_relay_state_legacy_->publish(relay);

    std_msgs::msg::Bool estop;
    estop.data = hw_estop_ || !relay_on_;
    pub_hw_estop_->publish(estop);
    pub_hardware_estop_->publish(estop);
  }

  void publish_motor_state(const double t)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"left_wheel_motor", "right_wheel_motor"};
    msg.position = {0.1 * t, 0.1 * t};
    msg.velocity = {0.1, 0.1};
    msg.effort = {0.0, 0.0};
    pub_motor_state_->publish(msg);
    pub_raw_motor_state_->publish(msg);
  }

  void publish_motor_diag(const double)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {35.0f, 35.0f, 32.0f, 32.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f};
    pub_motor_diag_->publish(msg);
    pub_raw_motor_diag_->publish(msg);
  }

  void publish_sbus_echo()
  {
    std_msgs::msg::Float32MultiArray channels;
    channels.data = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, -1.0f, 0.0f};
    pub_sbus_channels_->publish(channels);
    pub_raw_sbus_channels_->publish(channels);

    geometry_msgs::msg::Twist cmd_vel;
    pub_sbus_cmd_vel_->publish(cmd_vel);
    pub_raw_sbus_cmd_vel_->publish(cmd_vel);

    geometry_msgs::msg::Vector3 arm;
    pub_sbus_arm_cmd_->publish(arm);
    pub_raw_sbus_arm_cmd_->publish(arm);
  }

  double publish_rate_hz_{20.0};
  int control_mode_{0};
  bool relay_on_{true};
  bool hw_estop_{false};
  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<msd_msgs::msg::Encoder>::SharedPtr pub_encoder_;
  rclcpp::Publisher<msd_msgs::msg::Cylinder>::SharedPtr pub_cylinder_tick_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_imu_euler_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_env_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_system_status_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_power_relay_state_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_power_relay_state_legacy_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hw_estop_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hardware_estop_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_motor_state_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_diag_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_sbus_channels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_sbus_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_sbus_arm_cmd_;
  rclcpp::Publisher<msd_msgs::msg::Encoder>::SharedPtr pub_raw_encoder_;
  rclcpp::Publisher<msd_msgs::msg::Cylinder>::SharedPtr pub_raw_cylinder_tick_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_raw_imu_euler_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_raw_env_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_raw_battery_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_raw_motor_state_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_raw_motor_diag_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_raw_sbus_channels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_raw_sbus_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_raw_sbus_arm_cmd_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_control_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_power_relay_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_relay_legacy_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsdArduinoDummyNode>());
  rclcpp::shutdown();
  return 0;
}
