# MSD Controller (`tokuyama_v2`)

MSD700の制御ファームウェア。このbranchで使用する書き込み対象は以下の3種類。

## 1. Arduino Mega

- 実機: Arduino Mega 2560
- プロジェクト: `mega_msd_controller/`
- メイン: `mega_msd_controller/src/blv_arduino_mega.ino`
- 役割: S.BUS / 白コンI2C入力、クローラ、Lift、Dump、バッテリCAN、センサ制御
- 書き込み:

```sh
cd mega_msd_controller
pio run -t upload
```

### S.BUS割り当て

| CH | 用途 |
|---:|---|
| 1 | Dump（右スティック左右） |
| 2 | 前後走行（左スティック上下） |
| 3 | Lift（右スティック上下） |
| 4 | 旋回（左スティック左右） |
| 5 | 電源リレー: `+1 = ON`, `-1 = OFF` |
| 6 | 制御元: `-1 = 白コン`, `+1 = プロポ` |

### 実機出力

| 機能 | Megaピン |
|---|---|
| Lift FWD / REV | D42 / D43 |
| Dump FWD / REV | D44 / D45 |
| 電源リレー | D24 |
| I2C | SDA D20 / SCL D21 |

## 2. Tough代替 M5Stack CoreS3

- 実機: M5Stack CoreS3 (`MAC 1C:DB:D4:BA:80:F4`)
- プロジェクト: `tough_bridge/`
- メイン: `tough_bridge/src/main.cpp`
- 役割: 白コンからESP-NOW受信、MegaとI2C `0x55`で通信
- I2C Port A: SDA G2 / SCL G1 / GND
- 書き込み:

```sh
cd tough_bridge
pio run -t upload
```

### 白コン操作

| スティック | 用途 |
|---|---|
| 左上下 | クローラ前後 |
| 左左右 | クローラ旋回 |
| 右上下 | Lift |
| 右左右 | Dump |

## 3. 白コン M5Stack CoreS3

- 実機: 白コン側 M5Stack CoreS3
- PlatformIO入口: `platformio.ini`, `src/main.cpp`
- 実装本体: `ram_stack_send_db_new_1.ino`
- 役割: ジョイスティック取得、ESP-NOW送信、センサ表示・ログ
- ESP-NOW送信先: `1C:DB:D4:BA:80:F4`
- 書き込み:

```sh
pio run -t upload
```

`src/main.cpp`は`../ram_stack_send_db_new_1.ino`を読み込むPlatformIO用ラッパー。

## 通信経路

```text
白コン CoreS3
    | ESP-NOW
    v
Tough代替 CoreS3
    | I2C slave 0x55
    v
Arduino Mega 2560
```

## 確認済み

- Arduino Megaファームウェア: PlatformIO build / upload成功
- Tough代替CoreS3: PlatformIO build / upload成功
- 白コン→Tough代替: ESP-NOW通信成功
- Tough代替↔Mega: I2C通信成功
