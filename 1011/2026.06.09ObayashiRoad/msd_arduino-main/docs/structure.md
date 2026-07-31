# ディレクトリ構成と各ファイルの役割

## ディレクトリツリー

```
msd_arduino/
├── CMakeLists.txt
├── package.xml
├── 99-arduino-mega.rules            ← udev ルールファイル
├── msd_arduino.service              ← systemd 自動起動サービス
├── docs/
│   ├── README.md                    ← ドキュメント索引
│   ├── structure.md                 ← このファイル
│   ├── protocol.md                  ← 通信プロトコル仕様
│   ├── execute.md                   ← 実行・セットアップ手順
│   └── error-handler.md             ← エラー対処法
├── include/msd_arduino/              ← ROS2側ヘッダ
│   ├── protocol.hpp
│   ├── serial_port.hpp
│   └── msd_link.hpp
├── src/                              ← ROS2側実装
│   ├── msd_arduino_node.cpp
│   ├── serial_port.cpp
│   └── msd_link.cpp
├── launch/
│   └── msd_arduino.launch.py
└── firmware/blv_arduino_mega/        ← Arduino Mega 2560 スケッチ
    ├── blv_arduino_mega.ino
    ├── protocol.h
    ├── pins.h
    ├── msd_link.h / .cpp
    ├── sbus.h / .cpp
    ├── leds.h / .cpp
    ├── motors.h / .cpp
    ├── cylinders.h / .cpp
    └── imu_bno055.h / .cpp
```

---

## ROS2 側

| ファイル | 役割 |
|---|---|
| `protocol.hpp` | `0xAA 0x55` 始まりのフレーム，`CRC-16-CCITT`，`FrameParser`，`encode_frame`，全 payload struct．Arduino 側 `protocol.h` と**独立**だがワイヤ上では完全互換 |
| `serial_port.hpp` / `serial_port.cpp` | `open_port(path, baud, post_open_delay_ms)`／`read_some`／`write_all`．`cfmakeraw` + `HUPCL` クリアで **Arduino の DTR auto-reset をやり過ごす**のが肝 |
| `msd_link.hpp` / `msd_link.cpp` | `MsdSerialLink` クラス．`SerialPort` と `FrameParser` を抱え，最新受信値を `LinkRxState` に保持．`sendCmdVel` 等の型付きセンダ，`onImu` 等のコールバック，統計 `LinkStats` |
| `msd_arduino_node.cpp` | ROS ノード本体．パラメータ宣言，Pub/Sub 生成，タイマ起動，`MsdSerialLink` のコールバックから ROS トピックへの橋渡し |

---

## Arduino 側

| ファイル | 役割 |
|---|---|
| `protocol.h` | ROS 側 `protocol.hpp` と同じワイヤフォーマット定義．AVR 用に独立 |
| `pins.h` | 全ピンアサインをここに集約．配線変更時はここだけ見れば足りるよう意図 |
| `msd_link.h` / `.cpp` | `MsdProtocolLink` クラス．`Stream` をラップして `LinkRxState`（`cmd_vel`/`arm_cmd`/`host_hb` と `*_fresh`/`*_stamp_ms`/`any_*_seen`）を更新．`sendMcuHeartbeat` など型付きセンダ．統計 `LinkStats` |
| `sbus.h` / `.cpp` | `SbusReader`．`HardwareSerial` を開いて 25 バイトフレームを同期・デコード．`channel(i)` で -1..+1，`is_valid()`/`failsafe()` を提供 |
| `leds.h` / `.cpp` | `StatusLeds`．Red=エラー点滅，Green=Jetson，Blue=SBUS，Yellow=E-STOP（ソリッド）/ モータ駆動中（20Hz点滅） |
| `motors.h` / `.cpp` | `DiffDriveMotors`．`DiffDriveGeometry`（車輪半径0.1105m・減速比100・トレッド0.600m・最大RPM150）を受け取り `drive(linear_mps, angular_rps)` で両輪 RPM → 方向ピン + PWM．`begin()` で **M0=HIGH**（外部PWM入力），**STOP_MODE=LOW**（瞬時停止），**MB_FREE=HIGH**（ブレーキ解放） |
| `cylinders.h` / `.cpp` | `Cylinders`．最大 3 本．`command(i, input)` で ±0.5 を閾値に伸縮．`read(i)` で端点スイッチ + EMAフィルタ付きADC + mm変換（`RAW_MIN=0`, `RAW_MAX=1017`, `STROKE_MM=155.0`） |
| `imu_bno055.h` / `.cpp` | `ImuBno055`．Adafruit_BNO055 を抱え，`read(Imu&)` で Euler 3 + Quat 4 を固定小数（0.001 deg / 1e-4）にパック |
| `blv_arduino_mega.ino` | 上の全部をつなぐ．`setup` で各 `begin()`，`loop` で `update → consume → watchdog → arbitrate → drive → periodic senders` |

---

## ハードウェア対応表

### UART 割り当て（現状）

| UART | ピン | 役割 | 備考 |
|---|---|---|---|
| `Serial` (USB-CDC) | 0 / 1 | Jetson ⇄ Arduino 通信（当面） | Arduino IDE 書き込みも兼用 |
| `Serial1` | 18 / **D19** | **SBUS 受信専用**（RX のみ） | 100000 8E2 インバート |
| `Serial2` | **D16** / 17 | デバッグ print | FTDI USB-TTL の RX を D16 に．115200 8N1 |
| `Serial3` | 14 / 15 | **将来** Jetson UART リンクに | 現状未接続 |

将来の移行: `.ino` の `#define LINK_SERIAL Serial` → `Serial3`，USB は書き込み + デバッグ専用になる．

### ピン割り当て一覧（`pins.h` より）

```
ステータス LED:   RED=A3  GREEN=A4  BLUE=A5  YELLOW=A6
BLV 左モータ:     FWD=D28 REV=D29 STOP_MODE=D30 M0=D31 MB_FREE=D32 VM=D3(PWM)
BLV 右モータ:     FWD=D33 REV=D34 STOP_MODE=D35 M0=D36 MB_FREE=D37 VM=D4(PWM)
シリンダ 1:      FWD=D38 REV=D39 END_IN=D44 END_OUT=D45 FEEDBACK=A0
シリンダ 2:      FWD=D40 REV=D41 END_IN=D46 END_OUT=D47 FEEDBACK=A1
シリンダ 3 予約: FWD=D42 REV=D43 END_IN=D48 END_OUT=D49 FEEDBACK=A2
BNO055 IMU:     I2C  SDA=D20 SCL=D21 (addr=0x28)
```

### LED ステータス表

| LED | 色 | 点灯条件 |
|---|---|---|
| A3  | 赤 | `error_flags != 0` の間，400ms 周期で点滅 |
| A4  | 緑 | Jetson 制御が有効かつ非 E-STOP |
| A5  | 青 | SBUS 制御が有効 |
| A6  | 黄 | E-STOP 中 → ソリッド点灯 / モータ駆動中 → 20Hz 高速点滅 |
