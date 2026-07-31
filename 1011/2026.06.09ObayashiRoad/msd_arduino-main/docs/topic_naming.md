# msd_arduino Topic Naming

This package bridges Arduino firmware frames and ROS 2 topics.  The naming
rules below keep the Arduino wire protocol, ROS topics, and AI-training bags
consistent.

## Goal

Use the robot data for ACT-model policy learning of scoop and dump behavior,
including mobile base, arm, and bucket control.  Topics must preserve raw
Arduino-origin measurements so bags can be converted into training datasets
without guessing units or source devices.

## Naming Rules

- Arduino frame names stay hardware/protocol oriented: `CmdVel`, `ArmCmd`,
  `RelayCmd`, `Encoder`, `Imu`, `EnvSensor`, `SystemStatus`, `SbusChannels`,
  `BatteryStatus`, and `MotorStatus`.
- ROS command topics use `/msd/cmd/...` for commands sent toward hardware.
- ROS safety topics use `/msd/safety/...`.
- ROS power topics use `/msd/power/...`.
- Arduino-origin raw data aliases use `/msd/raw/...`.  These aliases mirror the
  existing public topics and are intended as the preferred rosbag inputs for
  ACT data collection.
- Legacy topics stay published/subscribed until all launch files, UI plugins,
  and scripts are migrated.

## Command Topics

| Canonical topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/msd/cmd_vel` | `geometry_msgs/Twist` | Jetson -> Arduino | Base velocity command. |
| `/msd/arm_cmd` | `geometry_msgs/Vector3` | Jetson -> Arduino | `x=cyl1`, `y=cyl2`, `z=mode`. |
| `/msd/cmd/power_relay` | `std_msgs/UInt8` | Jetson -> Arduino | Canonical power relay command, `0=OFF`, `1=ON`. |
| `/msd/cmd/relay_cmd` | `std_msgs/UInt8` | Jetson -> Arduino | Legacy alias for `/msd/cmd/power_relay`. |
| `/msd/cmd/control` | `std_msgs/UInt8` | Jetson -> Arduino | `0=AUTO`, `1=SBUS`, `2=YCON`. |
| `/msd/cmd/motor_alarm_reset` | `std_msgs/Empty` | Jetson -> Arduino | BLV-R alarm reset. |

## Safety And Power Topics

| Canonical topic | Type | Source | Notes |
| --- | --- | --- | --- |
| `/msd/power/relay_state` | `std_msgs/UInt8` | Arduino | Canonical power relay state, `0=OFF`, `1=ON`. |
| `/msd/relay_state` | `std_msgs/UInt8` | Arduino | Legacy alias. |
| `/msd/safety/hardware_estop` | `std_msgs/Bool` | Arduino | Physical E-stop input state derived from `ERR_HW_ESTOP`. |
| `/msd/hw_estop` | `std_msgs/Bool` | Arduino | Legacy alias. |
| `/msd/system_status` | `std_msgs/UInt8MultiArray` | Arduino | Packed status frame. Byte 14 is power relay state. |

## Raw Data Topics For AI Bags

| Canonical raw topic | Existing mirrored topic | Type | Notes |
| --- | --- | --- | --- |
| `/msd/raw/motor/encoder_tick` | `/msd/motor/encoder_tick` | `msd_msgs/Encoder` | Raw motor encoder counts. |
| `/msd/raw/cylinder/tick` | `/msd/cylinder/tick` | `msd_msgs/Cylinder` | Raw cylinder encoder counts. |
| `/msd/raw/imu/data` | `/msd/imu/data` | `sensor_msgs/Imu` | BNO055 fused orientation. |
| `/msd/raw/imu/euler` | `/msd/imu/euler` | `geometry_msgs/Vector3` | Euler angles in degrees. |
| `/msd/raw/env` | `/msd/env` | `std_msgs/Float32MultiArray` | Temperature, humidity, pressure. |
| `/msd/raw/battery` | `/msd/battery` | `sensor_msgs/BatteryState` | CAN BMS data. |
| `/msd/raw/sbus/channels` | `/msd/sbus/channels` | `std_msgs/Float32MultiArray` | CH1-8 normalized input. |
| `/msd/raw/sbus/cmd_vel` | `/msd/sbus/cmd_vel` | `geometry_msgs/Twist` | SBUS-derived drive command. |
| `/msd/raw/sbus/arm_cmd` | `/msd/sbus/arm_cmd` | `geometry_msgs/Vector3` | SBUS-derived arm command. |
| `/msd/raw/motor/state` | `/msd/motor/state` | `sensor_msgs/JointState` | BLV-R motor shaft state. |
| `/msd/raw/motor/diagnostics` | `/msd/motor/diagnostics` | `std_msgs/Float32MultiArray` | BLV-R temperature, alarm, odometer, current. |

For ACT data collection, record the canonical raw topics together with command,
status, TF, and camera/LiDAR topics needed by the training pipeline.
