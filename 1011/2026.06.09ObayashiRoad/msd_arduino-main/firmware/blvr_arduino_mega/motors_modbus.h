// motors_modbus.h — BLV-R differential-drive motor controller via RS485 Modbus RTU.
//
// Replaces the GPIO-based DiffDriveMotors with Modbus RTU communication
// to two Oriental Motor BLV-R drivers (left = slave 0x01, right = slave 0x02).
//
// Hardware:
//   Serial2 (pins 16/17) at 230400 baud → RS485 transceiver → BLV-R drivers
//   DE/RE pin controls transceiver direction (HIGH = TX, LOW = RX)
//
// Protocol:
//   Function Code 0x10 (Write Multiple Registers)
//   - Register 0x007C: CMD2 lower word (SON, STOP bits)
//   - Register 0x005A: Direct velocity data (14 registers = 28 bytes)
//   Function Code 0x03 (Read Holding Registers) for feedback
//
// Usage:
//   Call begin() once in setup(), update() every loop(), drive()/stop() as needed.

#ifndef MSD_BLVR_MOTORS_MODBUS_H_
#define MSD_BLVR_MOTORS_MODBUS_H_

#include <Arduino.h>
#include <stdint.h>

// Set to 0 to disable FC 0x03 feedback reads (useful to isolate alarm 0x84
// during RS485 bring-up).  Root cause of 0x84 was SERIAL_8N1 instead of
// SERIAL_8E1 — now fixed, so reads are enabled by default.
#ifndef MSD_MODBUS_READ_ENABLE
#define MSD_MODBUS_READ_ENABLE 1
#endif

// Modbus RTU IDシェアモード (manual HP-5141J §12).
// When 1, both motors are commanded and read in single shared frames addressed
// to the global ID — halves the number of Modbus transactions per cycle and
// guarantees synchronous left/right command timing.  Setup of the share group
// (Global ID, Number, Local ID, Share Read/Write data NET-IDs) must already
// be NV-saved on each driver via MEXE02 — see docs.
#ifndef MSD_USE_ID_SHARE_MODE
#define MSD_USE_ID_SHARE_MODE 1
#endif

namespace msd_arduino
{

#if MSD_USE_ID_SHARE_MODE
// Share-group constants.  Values must match the MEXE02 NV configuration on
// every driver in the group.
namespace blvr_share
{
  static const uint8_t GLOBAL_ID            = 0x0F;  // Share Control Global ID
  static const uint8_t LOCAL_ID_LEFT        = 0x01;  // Share Control Local ID (L)
  static const uint8_t LOCAL_ID_RIGHT       = 0x02;  // Share Control Local ID (R)
  static const uint8_t SHARE_NUMBER         = 2;     // Share Control Number

  // Share Write data slot configuration (NET-ID list NV-saved per slave):
  //   Slot 0 = NET-ID 47  (ダイレクトデータ運転 速度)         r/min
  //   Slot 1 = NET-ID 48  (ダイレクトデータ運転 加速レート)   ms
  //   Slot 2 = NET-ID 49  (ダイレクトデータ運転 減速レート)   ms
  //   Slot 3 = NET-ID 51  (ダイレクトデータ運転 反映トリガ)
  static const uint8_t WRITE_SLOTS          = 4;
  // Each slot is 32-bit = 2 IDshare regs; per-slave reg count = WRITE_SLOTS*2
  // Total reg count across all slaves (no inter-slave check on writes):
  static const uint8_t WRITE_REG_COUNT      = WRITE_SLOTS * 2 * SHARE_NUMBER;  // 16
  static const uint8_t WRITE_BYTE_COUNT     = WRITE_REG_COUNT * 2;             // 32
  static const uint8_t WRITE_BYTES_PER_SLAVE = WRITE_SLOTS * 4;                 // 16

  // Share Read data slot configuration (must match the beginning of the
  // MEXE02 NV setup).  The drivers may have more slots configured, but the
  // fast odometry query intentionally reads only the first two slots to keep
  // the response short and robust:
  //   Slot 0 = NET-ID 103 (検出速度)              r/min
  //   Slot 1 = NET-ID 144 (検出32bitカウンタ)     step
  static const uint8_t READ_SLOTS           = 2;
  // Reads include 1 inter-slave check register per slave; manual formula
  //   reg_count = (read_addr_count + 1) × Share Control Number
  //   read_addr_count = READ_SLOTS × 2  (each slot is 2 IDshare regs)
  static const uint8_t READ_REG_COUNT       =
    (READ_SLOTS * 2 + 1) * SHARE_NUMBER;  // (4+1)*2 = 10
  static const uint8_t READ_BYTE_COUNT      = READ_REG_COUNT * 2;  // 20
  static const uint8_t READ_BYTES_PER_SLAVE = READ_SLOTS * 4 + 2;  // 8 + 2 check = 10
}  // namespace blvr_share
#endif  // MSD_USE_ID_SHARE_MODE

struct ModbusDriveConfig
{
  float    wheel_radius_m   = 0.1105f;
  float    wheel_distance_m = 0.600f;
  float    reduction_ratio  = 100.0f;
  float    max_rpm          = 4000.0f;
  uint8_t  left_slave_id    = 0x01;
  uint8_t  right_slave_id   = 0x02;
  // Arduino Mega's 16 MHz UART has a large baud-rate error at 230400 bps
  // Target Modbus baud.  The Mega 2560 / 16 MHz UART has only 3.55 % error
  // at 230400 (above the ~2 % reliability threshold) → reads are corrupted
  // (e.g. FC 0x03 echoed as 0x30).  At 115200 the error is 2.12 %, which
  // is reliable enough.  begin() probes the actual driver baud; if the
  // driver is still at the factory default (230400), it migrates to this
  // value and NV-saves — exactly once.  Subsequent boots see the driver
  // already at 115200 and skip the migration.
  uint32_t baud             = 115200;
  uint32_t cmd_period_ms    = 20;     // velocity command send interval
  int32_t  accel_ms         = 1000;   // acceleration time [ms]
  int32_t  decel_ms         = 100;   // deceleration time [ms]
  // BLV-R direct-data torque limit uses 0.1% units.  The driver applies the
  // smallest of operation torque, TRQ-LMT input, ATL, stop, alarm, and supply
  // limits.  This robot uses 200 W BLV-R motors; manual maximum is 210%.
  int32_t  torque_limit     = 2100;   // normal torque (2100 = 210.0%)
  int32_t  torque_brake     = 100;    // soft-stop torque (100 = 10.0%)
  int32_t  accel_brake      = 1000;   // soft-stop acceleration
  int32_t  decel_brake      = 100;   // soft-stop deceleration

  // Command throttling (reference-code behavior). Skip sending velocity if
  // the RPM changed less than rpm_change_threshold AND the last send was
  // less than refresh_ms ago. Prevents over-saturating the Modbus link.
  int32_t  rpm_change_threshold = 30;
  uint32_t refresh_ms           = 200;

  // Write parameter 0x0296 = 1 on init (reference-code init sequence — likely
  // enables communication error auto-recovery / clears latched alarms on the
  // driver side). Set false to skip.
  bool     init_param_0296      = true;
};

class ModbusDiffDrive
{
public:
  void begin(HardwareSerial & serial, uint8_t de_re_pin,
             const ModbusDriveConfig & cfg = ModbusDriveConfig());

  // Must be called every loop() — drives the RS485 state machine.
  void update();

  // Set velocity command (same API as DiffDriveMotors).
  void drive(float linear_mps, float angular_rps);

  // Emergency stop / hold brake.
  void stop(bool brake);

  // Commanded RPM (for telemetry, matches old API).
  float lastLeftRpm()  const { return cmd_rpm_l_; }
  float lastRightRpm() const { return cmd_rpm_r_; }

  // Actual feedback from drivers (read via Modbus FC 0x03).
  // Verified register map per BLV-R manual HP-5141J (アドレスコード一覧/モニタコマンド):
  //   0x00CE/0x00CF  検出速度 [r/min, signed int32]
  //   0x0120/0x0121  検出32bitカウンタ [step, signed int32]
  //   0x0080/0x0081  現在アラーム [uint32]
  //   0x00D6/0x00D7  トルクモニタ [0.1%, signed int32]
  //   0x00D8/0x00D9  負荷率モニタ [0.1%, signed int32]
  //   0x00F8/0x00F9  ドライバ温度 [0.1°C, signed int32]
  //   0x00FA/0x00FB  モータ温度  [0.1°C, signed int32]
  //   0x0148/0x0149  主電源電圧  [0.1 V, signed int32]
  // Right-side speed/count are sign-flipped to match the robot frame
  // (right motor is physically reversed).
  int32_t actualLeftRpm()    const { return fb_rpm_l_; }
  int32_t actualRightRpm()   const { return fb_rpm_r_; }
  int32_t actualLeftCount()  const { return fb_count_l_; }
  int32_t actualRightCount() const { return fb_count_r_; }
  uint32_t encoderSampleSeq() const { return fb_count_pair_seq_; }
  int32_t alarmLeft()        const { return fb_alarm_l_; }
  int32_t alarmRight()       const { return fb_alarm_r_; }
  int32_t torqueLeft()       const { return fb_torque_l_; }   // 0.1 %
  int32_t torqueRight()      const { return fb_torque_r_; }
  int32_t tempDriverLeft()   const { return fb_tdrv_l_; }     // 0.1 °C
  int32_t tempDriverRight()  const { return fb_tdrv_r_; }
  int32_t tempMotorLeft()    const { return fb_tmot_l_; }     // 0.1 °C
  int32_t tempMotorRight()   const { return fb_tmot_r_; }
  int32_t vmainLeft()        const { return fb_vmain_l_; }    // 0.1 V
  int32_t vmainRight()       const { return fb_vmain_r_; }
  int32_t posLeft()          const { return fb_pos_l_; }      // step
  int32_t posRight()         const { return fb_pos_r_; }
  int32_t odoLeft()          const { return fb_odo_l_; }      // 0.1 krev
  int32_t odoRight()         const { return fb_odo_r_; }
  int32_t imainLeft()        const { return fb_imain_l_; }    // 0.001 A
  int32_t imainRight()       const { return fb_imain_r_; }

  // True after both drivers have been initialized (SON sent).
  bool isReady() const { return init_done_; }

  // Re-send init + SON after a power cycle (relay turn-on).
  // Call this ~2 s after the relay closes so the drivers have finished
  // their own power-on sequence.  Blocks ~120 ms for Modbus transactions.
  void reinitAfterPowerOn();

  // Clear active alarms on both drivers via maintenance command
  // (write 1 → register 0x0180, edge-triggered).  Blocks ~50 ms.
  void resetAlarms();

  // Modbus communication error counter (for diagnostics).
  uint16_t errorCount() const { return err_count_; }

  // Set a debug stream for Modbus diagnostics.
  void setDebug(Stream * s) { dbg_ = s; }

private:
  // Kinematics
  float vel_to_rpm_(float v) const;

  // RS485 low-level
  void rs485_tx_();
  void rs485_rx_();
  void sendRaw_(const uint8_t * pdu, uint8_t len_no_crc);
  void drainRx_();

  // Modbus commands
  void sendServoOn_(uint8_t slave_id);
  void sendStopPulse_(uint8_t slave_id);
  void sendVelocity_(uint8_t slave_id, int32_t rpm,
                     int32_t acc, int32_t dec, int32_t torque);

  // Write a 32-bit parameter (FC 0x10, 2 registers) — used for driver
  // configuration parameters like 0x0296 (communication error recovery).
  void writeParam32_(uint8_t slave_id, uint16_t addr, int32_t value);

  // FC 0x03 read of `count` holding registers starting at `start_addr`.
  // Response payload (big-endian bytes) is copied into out_data.
  // Returns true on success; increments err_count_ on failure.
  bool readHoldingRegisters_(uint8_t slave_id, uint16_t start_addr,
                             uint16_t count,
                             uint8_t * out_data, uint8_t out_size);

  // Read a 32-bit signed value spanning two registers (high-word first).
  bool readI32_(uint8_t slave_id, uint16_t start_addr, int32_t & out);

  // Probe whether BLV-R responds at the given baud (left slave, alarm reg).
  // Reconfigures Serial2 to `baud` and tries up to `attempts` reads.  Returns
  // true on first successful read.  Default attempts=2 stays under the
  // BLV-R communication-error threshold (0x138C, factory default 3).
  bool probeBaud_(uint32_t baud, uint8_t attempts = 2);

  // Walk the kProbeBauds list and return the first baud at which BLV-R
  // responds (or 0 if none).  Uses 1 attempt per baud to minimize garbage
  // frames at wrong bauds (each generates a comm error at the driver).
  // On success Serial2 is left configured at the returned baud.
  uint32_t probeAllBauds_();

#if MSD_USE_ID_SHARE_MODE
  // Send a single FC 0x10 frame to the share group's GLOBAL_ID.
  //   Slot 0: velocity (slave1=vel_l, slave2=vel_r)
  //   Slot 1: acceleration time
  //   Slot 2: deceleration time
  //   Slot 3: trigger (1 = normal launch — commits the velocity command)
  // Both motors receive the new command synchronously.
  void sendShareWrite_(int32_t vel_l, int32_t vel_r,
                       int32_t accel, int32_t decel, int32_t trigger);

  // Send a single FC 0x03 query to GLOBAL_ID and parse the combined response.
  // Populates fb_rpm_l/r_, fb_count_l/r_, fb_alarm_l/r_, fb_torque_l/r_,
  // fb_tdrv_l/r_, fb_tmot_l/r_.  Right-side rpm/count are sign-flipped to
  // robot frame.  Returns true on validated frame, false on TO/CRC/etc.
  bool readShareData_();
#endif

  bool readEncoderPairDirect_();
  bool diagnosticsFastNeeded_() const;
  void updateSlowDiagnostics_(uint32_t now);

  // CRC
  static uint16_t crc16_modbus_(const uint8_t * data, uint8_t len);
  static void put_i32_be_(uint8_t * p, int32_t v);

  HardwareSerial * serial_    = nullptr;
  uint8_t          de_re_pin_ = 0;
  ModbusDriveConfig cfg_{};

  // State machine
  enum Phase : uint8_t {
    PHASE_IDLE = 0,
    PHASE_SEND_LEFT,
    PHASE_SEND_RIGHT,
    PHASE_DONE,
  };
  Phase    phase_         = PHASE_IDLE;
  uint32_t last_cmd_ms_   = 0;
  bool     init_done_     = false;
  bool     braking_       = false;

  // Commanded RPM (from drive())
  float cmd_rpm_l_ = 0.0f;
  float cmd_rpm_r_ = 0.0f;

  // Last-sent RPM (for change detection / throttling)
  int32_t last_sent_rpm_l_ = 0;
  int32_t last_sent_rpm_r_ = 0;
  uint32_t last_sent_ms_   = 0;

  // Feedback from drivers (populated by periodic FC 0x03 reads)
  int32_t fb_rpm_l_    = 0;
  int32_t fb_rpm_r_    = 0;
  int32_t fb_count_l_  = 0;
  int32_t fb_count_r_  = 0;
  uint32_t fb_count_pair_seq_ = 0;
  int32_t fb_alarm_l_  = 0;
  int32_t fb_alarm_r_  = 0;
  int32_t fb_torque_l_ = 0;
  int32_t fb_torque_r_ = 0;
  int32_t fb_tdrv_l_   = 0;
  int32_t fb_tdrv_r_   = 0;
  int32_t fb_tmot_l_   = 0;
  int32_t fb_tmot_r_   = 0;
  int32_t fb_vmain_l_  = 0;
  int32_t fb_vmain_r_  = 0;
  int32_t fb_pos_l_    = 0;   // 検出位置 (step)
  int32_t fb_pos_r_    = 0;
  int32_t fb_odo_l_    = 0;   // ODO (0.1 krev)
  int32_t fb_odo_r_    = 0;
  int32_t fb_imain_l_  = 0;   // 主電源電流 (0.001 A)
  int32_t fb_imain_r_  = 0;

  // Rotating read phase for low-priority diagnostics. The fast encoder path
  // is ID Share and does not depend on this phase.
  uint8_t read_phase_ = 0;
  uint8_t diag_phase_ = 0;
  uint32_t diag_next_read_ms_ = 0;
  uint32_t diag_fast_until_ms_ = 0;
  uint32_t diag_next_odo_ms_ = 0;
  uint16_t diag_last_err_count_ = 0;

  Stream * dbg_       = nullptr;
  uint16_t err_count_ = 0;
};

}  // namespace msd_arduino

#endif  // MSD_BLVR_MOTORS_MODBUS_H_
