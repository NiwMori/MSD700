# 通信プロトコル（ROS2Serial v0.1）

## フレーム構造

```
[0]  SOF1 = 0xAA
[1]  SOF2 = 0x55
[2]  VER  = 0x01
[3]  TYPE
[4]  LEN  (payload 長)
[5]  SEQ_L
[6]  SEQ_H
[7..] PAYLOAD (little-endian packed struct)
[..]  CRC16-CCITT little-endian  (VER..PAYLOAD 末尾まで)
```

CRC 多項式: `0x1021`，初期値: `0xFFFF`．対象は VER (offset 2) から PAYLOAD 末尾まで．

---

## メッセージ ID

| 方向 | ID | 意味 | payload struct |
|---|---|---|---|
| J → A | 0x10 | heartbeat | `Heartbeat` (host_time_ms:u32, system_state:u8) |
| J → A | 0x11 | cmd_vel | `CmdVel` (linear_x_mmps:i16, angular_z_mradps:i16) |
| J → A | 0x12 | arm_cmd | `ArmCmd` (cyl1_target:i16, cyl2_target:i16, command_mode:u8) |
| J → A | 0x13 | control_mode | `uint8` |
| J → A | 0x14 | system_cmd | `uint8` (下記参照) |
| A → J | 0x20 | MCU heartbeat | `McuHeartbeat` (mcu_time_ms:u32, system_state:u8) |
| A → J | 0x21 | encoder | `Encoder` (L:i32, R:i32, cyl1:i32, cyl2:i32, t_ms:u32) |
| A → J | 0x22 | IMU | `Imu` (euler_x/y/z_mdeg:i16, quat_w/x/y/z_1e4:i16) |
| A → J | 0x23 | env | `EnvSensor` (temp_cdeg:i16, hum_cpercent:u16, pressure_pa:u32) |
| A → J | 0x24 | control_state | `ControlState` (control_mode:u8, comm_state:u8, motor_state:u8, arm_state:u8) |
| A → J | 0x25 | error_flags | `ErrorState` (error_flags:u32) |

---

## システムコマンド (0x14 payload)

| 値 | 定数 | 効果 |
|---|---|---|
| 0 | `SYS_CMD_CLEAR_ERRORS` | 全エラーフラグをクリア |
| 1 | `SYS_CMD_RESET_TARGETS` | cmd_vel / arm 目標値を 0 にリセット |
| 2 | `SYS_CMD_ESTOP` | 非常停止モード ON（全出力停止） |
| 3 | `SYS_CMD_ESTOP_CLEAR` | 非常停止モード OFF |

---

## エラーフラグ (error_flags bitmask)

| ビット | 名前 | トリガ | 自動クリア | 効果 |
|---|---|---|---|---|
| 0 | `ERR_CRC` | CRC 不一致 | なし | 物理層ノイズ指標 |
| 1 | `ERR_CMD_VEL_TO` | 200ms 無通信 | 復活時 | `linear/angular` 強制 0 |
| 2 | `ERR_ARM_TO` | 300ms 無通信 | 復活時 | フラグのみ |
| 3 | `ERR_HB_LOST` | 200ms 無 heartbeat | 復活時 | `comm_state=LOST` → SBUS なければ停止 |
| 4 | `ERR_SENSOR` | BNO055 read 失敗 | 成功時 | IMU データ未送信 |
| 5 | `ERR_ACTUATOR` | 未使用 | - | BLV ALM_OUT 監視で使う予定 |
| 6 | `ERR_FRAME` | payload 長不足/未知タイプ | なし | プロトコル不一致の検知 |
| 7 | `ERR_ESTOP` | `SYS_CMD_ESTOP` or SBUS ch5 | `SYS_CMD_ESTOP_CLEAR` | 全出力停止 |

---

## 単位の早見表

| フィールド名 | 単位 | 分解能 | 換算 |
|---|---|---|---|
| `linear_x_mmps` | mm/s | 1 mm/s | `× 1e-3` → m/s |
| `angular_z_mradps` | mrad/s | 1 mrad/s | `× 1e-3` → rad/s |
| `euler_*_mdeg` | 0.001 deg | 0.001 deg | `× 1e-3` → deg |
| `quat_*_1e4` | ×1e-4 | 0.0001 | `× 1e-4` → 単位クォータニオン |
| `temperature_cdeg` | 0.01 ℃ | 0.01 | `× 0.01` → ℃ |
| `humidity_cpercent` | 0.01 % | 0.01 | `× 0.01` → %RH |
| `pressure_pa` | Pa | 1 Pa | そのまま |
| `cyl*_count` (encoder) | 0.1 mm | 0.1 mm | `× 0.1` → mm |
| `left/right_motor_count` (encoder) | RPM (理論値) | 1 RPM | そのまま |

---

## ROS2 トピック

### サブスクライブ

| トピック | 型 | 備考 |
|---|---|---|
| `/msd/cmd_vel` | `geometry_msgs/Twist` | `linear.x` [m/s], `angular.z` [rad/s] |
| `/msd/arm_cmd` | `geometry_msgs/Vector3` | `x`=cyl1, `y`=cyl2 (±1000→±1.0), `z`=mode |

### パブリッシュ

| トピック | 型 | 内容 |
|---|---|---|
| `/msd/mcu_heartbeat` | `std_msgs/UInt32` | Arduino の `millis()` |
| `/msd/encoder` | `std_msgs/Int32MultiArray` | `[L_rpm, R_rpm, cyl1_0.1mm, cyl2_0.1mm, t_ms]` |
| `/msd/imu/data` | `sensor_msgs/Imu` | Quat（covariance=-1 で accel/gyro 欠損通知） |
| `/msd/imu/euler` | `geometry_msgs/Vector3` | Euler 角 [deg] |
| `/msd/env/temperature` | `sensor_msgs/Temperature` | |
| `/msd/env/humidity` | `sensor_msgs/RelativeHumidity` | |
| `/msd/env/pressure` | `sensor_msgs/FluidPressure` | |
| `/msd/control_state` | `std_msgs/UInt8MultiArray` | `[src, comm, motor, arm]` |
| `/msd/error_flags` | `std_msgs/UInt32` | ERR_* bitmask |
