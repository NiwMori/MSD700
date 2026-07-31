# エラー対処法

---

## エラーフラグ一覧 (`/msd/error_flags`)

Arduino MCU が検知するエラーはビットフラグとして `g_error_flags` に蓄積され、
ROS2 トピック `/msd/error_flags` およびデバッグシリアルに出力されます。

| ビット | 16進値   | 10進値 | 定数名           | 意味                                     | 発生条件                                              | 解除条件                                            |
|--------|----------|--------|------------------|------------------------------------------|-------------------------------------------------------|-----------------------------------------------------|
| 0      | `0x0001` | 1      | `ERR_CRC`        | CRC エラー                               | 受信フレームの CRC-16 が不一致                        | 次の正常フレーム受信で自動解除なし（手動クリア）    |
| 1      | `0x0002` | 2      | `ERR_CMD_VEL_TO` | cmd_vel タイムアウト                     | Jetson からの cmd_vel が 200ms 以上途切れた           | cmd_vel 受信再開で自動解除                          |
| 2      | `0x0004` | 4      | `ERR_ARM_TO`     | アームコマンド タイムアウト             | Jetson からの arm_cmd が 300ms 以上途切れた           | arm_cmd 受信再開で自動解除                          |
| 3      | `0x0008` | 8      | `ERR_HB_LOST`    | ハートビート喪失                         | Jetson heartbeat が 200ms 以上途切れた                | heartbeat 受信再開で自動解除                        |
| 4      | `0x0010` | 16     | `ERR_SENSOR`     | センサー異常                             | BNO055 IMU の読み取り失敗                             | IMU 読み取り成功で自動解除                          |
| 5      | `0x0020` | 32     | `ERR_ACTUATOR`   | アクチュエータ異常                       | （予約 — 現在未使用）                                 | —                                                   |
| 6      | `0x0040` | 64     | `ERR_FRAME`      | フレームエラー                           | プロトコルフレームの構造不正                          | 手動クリア（SYS_CMD_CLEAR_ERRORS）                  |
| 7      | `0x0080` | 128    | `ERR_ESTOP`      | 緊急停止（E-STOP）                       | SBUS CH5 (Switch A) が OFF (0)、または起動直後       | Switch A を ON (1) にする                           |
| 8      | `0x0100` | 256    | `ERR_SBUS`       | SBUS 受信機無応答                        | SBUS フレームが 200ms 以上受信できない / failsafe    | SBUS 受信再開で自動解除                             |

### よく見るパターン

| 表示例                          | 状態                                             |
|---------------------------------|--------------------------------------------------|
| `0x0080` — `[E-STOP]`          | E-STOP 発動中。Switch A を ON にする             |
| `0x0180` — `[E-STOP][SBUS]`    | SBUS 未接続 + E-STOP（起動直後の正常状態）       |
| `0x0002` — `[CMD_VEL_TO]`      | Jetson から cmd_vel が来ていない                 |
| `0x0008` — `[HB_LOST]`         | Jetson ノードが落ちている                        |
| `0x0010` — `[SENSOR]`          | BNO055 IMU 通信失敗                              |
| `0x0100` — `[SBUS]`            | SBUS 受信機が応答しない                          |

### 手動エラークリア

ROS2 側から全エラーフラグをリセットする:

```bash
# system_cmd = 0 (SYS_CMD_CLEAR_ERRORS) を送信
ros2 topic pub --once /msd/system_cmd std_msgs/msg/UInt8 "{data: 0}"
```

### 起動時の動作

安全のため、Arduino は **E-STOP 状態で起動** します（リレー OFF）。
SBUS 受信機が接続され、Switch A が ON (1 = 動作) になるまでリレーは ON になりません。
SBUS が一度も接続されない場合は、Jetson からの `SYS_CMD_ESTOP_CLEAR` で解除できます。

---

## 1. `Failed to open /dev/ttyACM0: Permission denied`

```bash
sudo usermod -aG dialout $USER
# 一度ログアウト/ログインし直す
```

udev ルールで `MODE="0666"` を設定済みなら `/dev/arduino-mega` を使えば
`dialout` グループ不要．

---

## 2. `First frame received` が出ない（通信不通）

- `dmesg | tail` で `/dev/ttyACM0` （or `/dev/arduino-mega`）に列挙されてるか確認
- `ls -l /dev/ttyACM* /dev/arduino-mega` でデバイスノードが実在するか確認
- Arduino を抜き差ししても同じなら:
  - `ros2 run msd_arduino msd_arduino_node --ros-args -p open_delay_ms:=3000`（bootloader 待ち延長）
  - Arduino IDE のシリアルモニタを**開きっぱなしにしない**．`/dev/ttyACM0` を占有するとノードが開けない
- `Serial2` に FTDI を繋いで Arduino 側のデバッグ print (`=== blv_arduino_mega boot ===`) が出ているか確認

---

## 3. `IMU: BNO055 not found` が出続ける

- I2C 配線（SDA=D20, SCL=D21）・プルアップ抵抗（内蔵でも可だが外付け 4.7k 推奨）
- BNO055 の ADR ピンが GND（= `0x28`）か．HIGH なら `0x29` → `imu_bno055.cpp` のアドレス修正
- `i2cdetect` 互換のスケッチで 0x28 が見えるかチェック

---

## 4. `fatal error: Adafruit_Sensor.h: No such file or directory`

BNO055 ライブラリ（と依存の Adafruit Unified Sensor）が未インストール:

```bash
arduino-cli lib update-index
arduino-cli lib install "Adafruit BNO055" "Adafruit Unified Sensor"
arduino-cli lib list | grep -i -E "bno055|unified"   # 両方出ればOK
```

インストール先が分からなくなったら:

```bash
arduino-cli config dump | grep -i "directories"
```

`user` ディレクトリ配下 `libraries/` に `Adafruit_BNO055` と `Adafruit_Sensor` があること．

---

## 5. `/msd/error_flags` に `0x02` (ERR_CMD_VEL_TO) が立つ

- cmd_vel の pub レートが 5Hz 未満だと 200ms タイムアウトが発火する
  - 連続指令は `ros2 topic pub -r 10 ...` のように最低 5Hz 推奨
- タイムアウトを変えたい場合は `protocol.hpp` と `protocol.h` の `CMD_VEL_TIMEOUT_MS` を**両方**揃えて変更

---

## 6. `/msd/error_flags` に `0x08` (ERR_HB_LOST) が立つ

- ROS ノードが落ちたか，heartbeat タイマが動いていない
- `ros2 node list` でノードが生きてるか確認
- `ros2 topic hz /msd/mcu_heartbeat` で 20Hz 近く出ているか確認

---

## 7. `'T' does not name a type` 等の Arduino IDE コンパイルエラー

.ino の自動プロトタイプ生成がテンプレ関数を壊す既知の問題．現在の構成ではテンプレ関数は使っていないが，将来追加する際は `.h`/`.cpp` に出すか，明示的に宣言を .ino に書くこと．

---

## 8. モータが逆回転する

ハードの結線で決まる．

- `pins.h` で `M_LEFT_FWD`/`M_LEFT_REV` を交換するか
- `motors.cpp::apply_side_` の `fwd`/`rev` を入れ替える

運動学の計算側（`ik`）は触らない．

---

## 9. ビルドエラー `'sensor_msgs/msg/imu.hpp' file not found` 等

ROS 2 Humble の依存パッケージが入っていない:

```bash
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs
```

---

## 10. systemd サービスが起動しない

```bash
# ログ確認
journalctl -u msd_arduino.service -e

# よくある原因:
# - /dev/arduino-mega が存在しない → udev ルール未適用 or Arduino 未接続
# - ROS 2 の source パスが間違っている → msd_arduino.service 内のパスを確認
# - ビルドしていない → colcon build --packages-select msd_arduino

# 手動テスト
sudo systemctl restart msd_arduino.service
sudo systemctl status msd_arduino.service
```

---

## 11. udev ルールが効かない

```bash
# ルールの再読み込み
sudo udevadm control --reload-rules
sudo udevadm trigger

# Arduino のベンダ/プロダクト ID を確認
lsusb | grep Arduino
# Bus 001 Device 005: ID 2341:0042 Arduino SA Mega 2560 R3 ...

# もし ID が違えば 99-arduino-mega.rules の idVendor/idProduct を修正
```
