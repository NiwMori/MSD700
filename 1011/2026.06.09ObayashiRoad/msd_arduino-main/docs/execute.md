# 実行・セットアップ手順

## 1. Arduino への書き込み

### 方法 A: Jetson CLI（arduino-cli）

#### 1-1. arduino-cli インストール（初回のみ）

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
  | BINDIR=$HOME/bin sh
echo 'export PATH=$HOME/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
```

#### 1-2. 必要ライブラリ

```bash
arduino-cli lib update-index
arduino-cli lib install "Adafruit BNO055" "Adafruit Unified Sensor"
```

確認:

```bash
arduino-cli lib list | grep -i -E "bno055|unified"
# Adafruit BNO055           1.6.x  ...
# Adafruit Unified Sensor   1.1.x  ...
```

もし見つからない場合:

```bash
arduino-cli config init
arduino-cli core update-index
arduino-cli lib update-index
arduino-cli lib search bno055
```

#### 1-3. コンパイル + 書き込み

```bash
cd /home/msd700/ros2_ws/src/msd_arduino
arduino-cli compile --fqbn arduino:avr:mega firmware/blv_arduino_mega
arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega firmware/blv_arduino_mega
```

`.ino` と同じディレクトリに置いた `.h`/`.cpp` は自動で拾ってコンパイルされる．

### 方法 B: Arduino IDE

1. Arduino IDE 2.x を開く
2. ライブラリマネージャで `Adafruit BNO055` をインストール（依存で `Adafruit Unified Sensor` が入る）
3. `File → Open` → `firmware/blv_arduino_mega/blv_arduino_mega.ino`
4. `Tools → Board: Arduino Mega or Mega 2560`，`Processor: ATmega2560`，`Port: /dev/ttyACM0`
5. `Upload`（→右矢印）

---

## 2. udev ルール（デバイス名固定）

Arduino Mega が USB に接続されたとき `/dev/arduino-mega` という固定シンボリックリンクを自動作成する．

ルールファイル: `99-arduino-mega.rules`

```
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", MODE="0666", SYMLINK+="arduino-mega"
```

### 有効化コマンド

```bash
# ルールをシステムに配置
sudo cp /home/msd700/ros2_ws/src/msd_arduino/99-arduino-mega.rules /etc/udev/rules.d/

# udev に再読み込みさせる
sudo udevadm control --reload-rules
sudo udevadm trigger

# Arduino を USB に接続した状態で確認
ls -l /dev/arduino-mega
# lrwxrwxrwx 1 root root 7 ... /dev/arduino-mega -> ttyACM0
```

`MODE="0666"` により `dialout` グループ未所属でも読み書き可能．
セキュリティを厳しくする場合は `MODE="0660", GROUP="dialout"` に変更して `sudo usermod -aG dialout $USER` で対応．

---

## 3. ROS2 ノードの実行

### 3-1. ビルド

```bash
cd /home/msd700/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select msd_arduino
```

### 3-2. 手動起動

```bash
source /home/msd700/ros2_ws/install/setup.bash

# 直接実行
ros2 run msd_arduino msd_arduino_node \
  --ros-args -p port:=/dev/arduino-mega -p baud:=115200

# launch ファイル経由
ros2 launch msd_arduino msd_arduino.launch.py
```

### 3-3. 動作確認

```bash
# MCU ハートビート（Arduino が生きてれば 20Hz で増えていく）
ros2 topic echo /msd/mcu_heartbeat

# IMU（BNO055 接続時）
ros2 topic echo /msd/imu/euler

# 速度指令（0.3 m/s 前進）
ros2 topic pub -r 10 /msd/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'

# 非常停止
ros2 topic pub --once /msd/system_cmd std_msgs/UInt8 "{data: 2}"
```

起動直後の正常ログ:

```
[INFO] Opening /dev/arduino-mega @ 115200 (waiting 2000 ms for Arduino bootloader)...
[INFO] Serial port ready
[INFO] First frame received from Arduino — link is alive
```

---

## 4. systemd による自動起動

Jetson 起動時に自動でノードを立ち上げる．

### 4-1. サービスインストール

```bash
# サービスファイルを配置
sudo cp /home/msd700/ros2_ws/src/msd_arduino/msd_arduino.service \
  /etc/systemd/system/

# systemd に認識させる
sudo systemctl daemon-reload

# 自動起動を有効化
sudo systemctl enable msd_arduino.service

# すぐに起動してみる（テスト）
sudo systemctl start msd_arduino.service
```

### 4-2. 管理コマンド

```bash
# ステータス確認
sudo systemctl status msd_arduino.service

# ログ確認
journalctl -u msd_arduino.service -f

# 停止
sudo systemctl stop msd_arduino.service

# 自動起動を無効化
sudo systemctl disable msd_arduino.service
```

### 4-3. 動作の流れ

1. Jetson 起動 → systemd が `msd_arduino.service` を開始待ち
2. Arduino USB が認識され `/dev/arduino-mega` が出現（udev ルール）
3. サービスが `ros2 launch msd_arduino msd_arduino.launch.py` を実行
4. ノードが Arduino との通信を確立（bootloader 待ち 2 秒 → link alive）
5. 失敗時は 5 秒後に自動リトライ（`Restart=on-failure`）

---

## 5. ビルドと再生成の早見

```bash
# ROS2 再ビルド
cd /home/msd700/ros2_ws
colcon build --packages-select msd_arduino
source install/setup.bash

# Arduino 再書き込み
cd /home/msd700/ros2_ws/src/msd_arduino
arduino-cli compile --fqbn arduino:avr:mega firmware/blv_arduino_mega
arduino-cli upload  -p /dev/arduino-mega --fqbn arduino:avr:mega firmware/blv_arduino_mega

# デバッグシリアル monitor（Serial2 → D16 TX）
screen /dev/ttyUSB0 115200
# 終了: Ctrl+A → K → y
```
