// blvr_arduino_mega.ino — main sketch (BLV-R Modbus RTU variant).
//
// Same architecture as blv_arduino_mega, but motors are controlled via
// RS485 Modbus RTU instead of GPIO pins.
//
// UART map:
//   Serial   (USB-CDC)     — debug print (Arduino IDE serial monitor)
//   Serial1  (pins 18/19)  — SBUS input (RX on D19)
//   Serial2  (pins 16/17)  — RS485 Modbus RTU to BLV-R motor drivers
//   Serial3  (pins 14/15)  — Jetson LINK (UART protocol)

#include "pins.h"
#include "protocol.h"
#include "msd_link.h"
#include "sbus_reader.h"
#include "leds.h"
#include "motors_modbus.h"
#include "cylinders.h"
#include "imu_bno055.h"
#include "env_bme280.h"
#include "battery_can.h"
#include "ycon_controller.h"

using namespace msd_arduino;
using namespace msd_arduino::protocol;

// =====================================================================
// Build-time configuration
// =====================================================================
#define LINK_SERIAL     Serial3
static const uint32_t LINK_BAUD = 115200;

#define DEBUG_ENABLE    1
#define DEBUG_SERIAL    Serial        // USB-CDC (Serial2 is used by Modbus)
static const uint32_t DEBUG_BAUD = 115200;

#define SBUS_ENABLE     1
#define SBUS_SERIAL     Serial1
static const uint32_t SBUS_BAUD = 100000;

// When 0, SBUS link loss does NOT drive E-STOP and the SBUS CH5 switch
// is ignored — Jetson-only operation is allowed to continue. Set to 1
// only if the SBUS radio is a required safety layer.
#define SBUS_FOR_ESTOP  0

// Manual source auto switching. SBUS/YCON availability alone does not steal
// control from Jetson; a real joystick intent must persist before latching.
static const float MANUAL_INTENT_THRESHOLD = 0.12f;
static const uint32_t MANUAL_INTENT_DWELL_MS = 100;
static const uint32_t MANUAL_RETURN_NEUTRAL_MS = 2000;

// Full-scale command mapping used when SBUS drives the robot.
static const float SBUS_MAX_LINEAR_MPS   = 0.5f;
static const float SBUS_MAX_ANGULAR_RPS  = 0.80f;
static const float SBUS_MOTOR_DEADBAND   = 0.30f;
static const float SBUS_CYL_ON_THRESHOLD = 0.75f;
static const uint32_t SBUS_CYL_DEBOUNCE_MS = 120;
static const float YCON_MAX_LINEAR_MPS   = 0.5f;
static const float YCON_MAX_ANGULAR_RPS  = 0.80f;
static const uint32_t YCON_CONTROL_TIMEOUT_MS = 300;

// SBUS channel assignments (0-indexed).
static const uint8_t CH_LINEAR    = 1;  // CH2: left_joy_y → linear.x
static const uint8_t CH_ANGULAR   = 3;  // CH4: left_joy_x → angular.z
static const uint8_t CH_CYL1      = 2;  // CH3: right_joy_y → cylinder 1
static const uint8_t CH_CYL2      = 0;  // CH1: right_joy_x → cylinder 2
static const uint8_t CH_ESTOP_SW  = 4;  // CH5: switch_A → E-STOP
static const uint8_t CH_ALM_RST   = 5;  // CH6: switch_B → motor alarm reset

// CAN battery enable
#define CAN_BATTERY_ENABLE 1

// I2C sensors. Bring them up one at a time while debugging the new IMU.
// A stuck I2C device can stop startup before the Jetson UART frames begin.
#define IMU_BNO055_ENABLE 1   // BNO055 at 0x29
#define ENV_BME280_ENABLE 1   // BME280 at 0x76
#define YCON_ENABLE       1   // M5Stack Tough ycon bridge at 0x55
#define I2C_SCAN_ON_BOOT  0
#define I2C_SENSORS_ENABLE (IMU_BNO055_ENABLE || ENV_BME280_ENABLE)
#define I2C_REQUIRED (I2C_SENSORS_ENABLE || YCON_ENABLE)
static const uint8_t I2C_SDA_PIN = 20;
static const uint8_t I2C_SCL_PIN = 21;

// Outbound send periods (ms).
//   Latency-optimised set for BLV-R: heartbeat / control_state / error_flags
//   are coalesced into a single 0x26 SystemStatus frame at 10 Hz.
static const uint32_t SYSTEM_STATUS_PERIOD_MS = 100;  // 10 Hz combined
static const uint32_t ENCODER_PERIOD_MS    = 20;
static const uint32_t SBUS_JOY_PERIOD_MS   = 50;
static const uint32_t IMU_PERIOD_MS        = 20;
static const uint32_t ENV_PERIOD_MS        = 200;
static const uint32_t BATTERY_PERIOD_MS    = 500;
static const uint32_t MOTOR_STATUS_PERIOD_MS = 200;  // 5 Hz BLV-R diagnostics
static const uint32_t LED_UPDATE_PERIOD_MS = 10;

// =====================================================================
// Debug macros
// =====================================================================
#if DEBUG_ENABLE
  #define DBG_BEGIN()      DEBUG_SERIAL.begin(DEBUG_BAUD)
  #define DBG_PRINT(x)     DEBUG_SERIAL.print(x)
  #define DBG_PRINT2(x,y)  DEBUG_SERIAL.print((x), (y))
  #define DBG_PRINTLN(x)   DEBUG_SERIAL.println(x)
#else
  #define DBG_BEGIN()
  #define DBG_PRINT(x)
  #define DBG_PRINT2(x,y)
  #define DBG_PRINTLN(x)
#endif

// =====================================================================
// Library instances
// =====================================================================
static MsdProtocolLink  g_link;
#if SBUS_ENABLE
static SbusReader       g_sbus;
#endif
static StatusLeds       g_leds;
static ModbusDiffDrive  g_motors;       // BLV-R via RS485 Modbus RTU
static Cylinders        g_cyls;
static ImuBno055        g_imu;
static EnvBme280        g_env;
static YconController   g_ycon;
#if CAN_BATTERY_ENABLE
static BatteryCan       g_battery;
#endif

// =====================================================================
// Arbitrated targets (fed to the hardware each loop)
// =====================================================================
static float   g_applied_linear_mps  = 0.0f;
static float   g_applied_angular_rps = 0.0f;
static float   g_applied_cyl1_input  = 0.0f;
static float   g_applied_cyl2_input  = 0.0f;
static uint8_t g_applied_arm_mode    = 0;

// Jetson's last known targets.
static float   g_jetson_linear_mps   = 0.0f;
static float   g_jetson_angular_rps  = 0.0f;
static float   g_jetson_cyl1_input   = 0.0f;
static float   g_jetson_cyl2_input   = 0.0f;
static uint8_t g_jetson_arm_mode     = 0;
static uint8_t g_system_state        = 0;

static uint8_t  g_active_source = CTRL_SRC_JETSON;
static uint8_t  g_comm_state    = COMM_INIT;
static uint32_t g_error_flags   = 0;
static bool     g_sw_estop      = false;  // set/cleared by SYS_CMD_ESTOP / SYS_CMD_ESTOP_CLEAR
static bool     g_hw_estop      = false;  // set when D22 modeSwitch is LOW
static bool     g_ycon_estop    = false;  // set while fresh YCON control requests e-stop
static bool     g_estop         = false;  // combined: g_sw_estop || g_hw_estop || g_ycon_estop
static bool     g_motor_active  = false;
static bool     g_power_relay   = false;
static bool     g_relay_enable  = false;   // commanded by ROS relay topic
static bool     g_sbus_ever_valid = false;
static uint8_t  g_soc_percent   = 100;
static bool     g_imu_ok        = false;
static bool     g_env_ok        = false;
static bool     g_battery_ok    = false;
static Imu      g_last_imu      = {};
static EnvSensor g_last_env     = {};
static BatteryStatus g_last_battery = {};

// Auto-switching state for SBUS/YCON manual intent.
static bool     g_control_override_seen = false;
static uint8_t  g_manual_intent_source = CTRL_SRC_JETSON;
static bool     g_manual_intent_tracking = false;
static uint32_t g_manual_intent_since_ms = 0;
static bool     g_manual_neutral_tracking = false;
static uint32_t g_manual_neutral_since_ms = 0;

// BLV-R re-init after relay power-on.
// When the relay closes the drivers power up and need ~2 s before they
// accept Modbus.  We schedule a non-blocking reinit here.
static bool     g_motors_reinit_pending = false;
static uint32_t g_motors_reinit_at_ms   = 0;

// Schedule timestamps.
static uint32_t g_next_sysstat_tx_ms = 0;
static uint32_t g_next_encoder_tx_ms = 0;
static uint32_t g_last_encoder_tx_seq = 0;
static uint32_t g_next_sbus_joy_tx_ms = 0;
static uint32_t g_next_imu_tx_ms     = 0;
static uint32_t g_next_env_tx_ms     = 0;
static uint32_t g_next_battery_ms    = 0;
static uint32_t g_next_motor_status_ms = 0;
static uint32_t g_next_led_ms        = 0;
static uint32_t g_next_ycon_ms       = 0;

static bool i2c_present(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool imu_detect_addr(uint8_t & addr)
{
  if (i2c_present(0x29)) {
    addr = 0x29;
    return true;
  }
  if (i2c_present(0x28)) {
    addr = 0x28;
    return true;
  }
  return false;
}

static bool i2c_bus_idle()
{
  // Do not enable the Mega's 5 V internal pullups here. The new IMU is
  // powered at 3.3 V, so the I2C bus should be pulled up externally to
  // 3.3 V or passed through a level shifter.
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  delay(2);
  return digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH;
}

static float arm_deadband(float v)
{
  return (fabsf(v) < 0.2f) ? 0.0f : v;
}

static float sbus_motor_axis(uint8_t index, float raw)
{
  if (index >= 2) {return 0.0f;}

  if (fabsf(raw) < SBUS_MOTOR_DEADBAND) {
    return 0.0f;
  }
  return raw;
}

static float sbus_cylinder_cmd(uint8_t index, float raw, uint32_t now_ms)
{
  static int8_t   committed[2] = {0, 0};
  static int8_t   pending[2]   = {0, 0};
  static uint32_t pending_since_ms[2] = {0, 0};

  if (index >= 2) {return 0.0f;}

  int8_t want = 0;
  if (raw >= SBUS_CYL_ON_THRESHOLD) {
    want = 1;
  } else if (raw <= -SBUS_CYL_ON_THRESHOLD) {
    want = -1;
  }

  if (want != pending[index]) {
    pending[index] = want;
    pending_since_ms[index] = now_ms;
  }

  if (pending[index] != committed[index] &&
      (int32_t)(now_ms - pending_since_ms[index]) >=
        (int32_t)SBUS_CYL_DEBOUNCE_MS) {
    committed[index] = pending[index];
  }

  return (float)committed[index];
}

static float max_abs4(float a, float b, float c, float d)
{
  float m = fabsf(a);
  const float ab = fabsf(b);
  const float ac = fabsf(c);
  const float ad = fabsf(d);
  if (ab > m) { m = ab; }
  if (ac > m) { m = ac; }
  if (ad > m) { m = ad; }
  return m;
}

#if SBUS_ENABLE
static float sbus_manual_magnitude()
{
  return max_abs4(
    g_sbus.channel(CH_LINEAR),
    g_sbus.channel(CH_ANGULAR),
    g_sbus.channel(CH_CYL1),
    g_sbus.channel(CH_CYL2));
}
#endif

#if YCON_ENABLE
static float ycon_manual_magnitude()
{
  const YconControlData & yc = g_ycon.control();
  return max_abs4(
    (float)yc.linear_x / 1000.0f,
    (float)yc.angular_z / 1000.0f,
    (float)yc.arm / 1000.0f,
    (float)yc.bucket / 1000.0f);
}
#endif

static bool i2c_recover_bus()
{
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  delay(2);
  if (digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH) {
    return true;
  }

  // If a sensor reset while holding SDA low mid-transfer, 9 SCL pulses can
  // release it. This will not fix a hard short or wrong wiring.
  pinMode(I2C_SCL_PIN, OUTPUT);
  for (uint8_t i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; ++i) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  pinMode(I2C_SCL_PIN, INPUT);
  delay(2);
  return digitalRead(I2C_SDA_PIN) == HIGH && digitalRead(I2C_SCL_PIN) == HIGH;
}

static void i2c_scan_debug()
{
#if DEBUG_ENABLE
  DBG_PRINTLN(F("I2C: scan start"));
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      DBG_PRINT(F("I2C: ACK 0x"));
      if (addr < 0x10) { DBG_PRINT(F("0")); }
      DBG_PRINT2(addr, HEX);
      DBG_PRINTLN(F(""));
      ++found;
    }
  }
  DBG_PRINT(F("I2C: scan found "));
  DBG_PRINTLN(found);
#endif
}

// =====================================================================
// Pin bundles (cylinders only — motors are via Modbus now)
// =====================================================================
static CylinderPins cyl_pins(uint8_t i) {
  CylinderPins p;
  switch (i) {
    case 0:
      // Logical cylinder 1 is wired to the board's CYL2 connector.
      p.fwd=pins::CYL2_FWD; p.rev=pins::CYL2_REV;
      p.end_in=pins::CYL2_END_IN; p.end_out=pins::CYL2_END_OUT;
      p.feedback=pins::CYL2_FEEDBACK; break;
    case 1:
      // Logical cylinder 2 is wired to the board's CYL1 connector.
      p.fwd=pins::CYL1_FWD; p.rev=pins::CYL1_REV;
      p.end_in=pins::CYL1_END_IN; p.end_out=pins::CYL1_END_OUT;
      p.feedback=pins::CYL1_FEEDBACK; break;
    case 2:
      p.fwd=pins::CYL3_FWD; p.rev=pins::CYL3_REV;
      p.end_in=pins::CYL3_END_IN; p.end_out=pins::CYL3_END_OUT;
      p.feedback=pins::CYL3_FEEDBACK; break;
  }
  return p;
}

// =====================================================================
// Consume any fresh inbound frames in g_link.rx()
// =====================================================================
static void consume_link_rx()
{
  LinkRxState & rx = g_link.rx();

  if (rx.host_hb_fresh) {
    rx.host_hb_fresh = false;
    g_system_state   = rx.host_hb.system_state;
  }

  if (rx.cmd_vel_fresh) {
    rx.cmd_vel_fresh     = false;
    g_jetson_linear_mps  = rx.cmd_vel.linear_x_mmps    * 1e-3f;
    g_jetson_angular_rps = rx.cmd_vel.angular_z_mradps * 1e-3f;
  }

  if (rx.arm_cmd_fresh) {
    rx.arm_cmd_fresh = false;
    g_jetson_cyl1_input = arm_deadband((float)rx.arm_cmd.cyl1_target / 1000.0f);
    g_jetson_cyl2_input = arm_deadband((float)rx.arm_cmd.cyl2_target / 1000.0f);
    g_jetson_arm_mode = rx.arm_cmd.command_mode;
  }

  // /msd/cmd/control: 0 returns to automatic arbitration, 1/2 force SBUS/YCON.
  if (rx.control_mode_fresh) {
    if (rx.control_mode == CTRL_SRC_JETSON) {
      g_control_override_seen = false;
      g_active_source = CTRL_SRC_JETSON;
      g_manual_intent_source = CTRL_SRC_JETSON;
      g_manual_intent_tracking = false;
      g_manual_neutral_tracking = false;
    } else {
      g_control_override_seen = true;
    }
    rx.control_mode_fresh = false;
  }

  if (rx.relay_cmd_fresh) {
    rx.relay_cmd_fresh = false;
    const bool req = (rx.relay_cmd != 0);
    if (req != g_relay_enable) {
      g_relay_enable = req;
      DBG_PRINT(F("RELAY cmd -> "));
      DBG_PRINTLN(g_relay_enable ? F("ON") : F("OFF"));
    }
  }

  if (rx.system_cmd_fresh) {
    rx.system_cmd_fresh = false;
    switch (rx.system_cmd) {
      case SYS_CMD_CLEAR_ERRORS:
        g_error_flags = 0;
        DBG_PRINTLN(F("SYS: errors cleared"));
        break;
      case SYS_CMD_RESET_TARGETS:
        g_jetson_linear_mps  = 0.0f;
        g_jetson_angular_rps = 0.0f;
        g_jetson_cyl1_input  = 0.0f;
        g_jetson_cyl2_input  = 0.0f;
        DBG_PRINTLN(F("SYS: targets reset"));
        break;
      case SYS_CMD_ESTOP:
        g_sw_estop = true;
        DBG_PRINTLN(F("SYS: SW E-STOP engaged"));
        break;
      case SYS_CMD_ESTOP_CLEAR:
        g_sw_estop = false;
        DBG_PRINTLN(F("SYS: SW E-STOP cleared"));
        break;
      case SYS_CMD_MOTOR_ALM_RESET:
        DBG_PRINTLN(F("SYS: motor alarm reset"));
        g_motors.resetAlarms();
        break;
    }
  }
}

// =====================================================================
// Watchdogs
// =====================================================================
static void update_watchdogs()
{
  const uint32_t now = millis();
  const LinkRxState & rx = g_link.rx();

  if (rx.any_host_hb_seen &&
      (now - rx.host_hb_stamp_ms) > HEARTBEAT_TIMEOUT_MS) {
    if ((g_error_flags & ERR_HB_LOST) == 0) {
      DBG_PRINTLN(F("WDG: heartbeat lost"));
    }
    g_error_flags |= ERR_HB_LOST;
    g_comm_state = COMM_LOST;
  } else if (rx.any_host_hb_seen) {
    g_comm_state = COMM_OK;
    g_error_flags &= ~ERR_HB_LOST;
  }

  if (rx.any_cmd_vel_seen &&
      (now - rx.cmd_vel_stamp_ms) > CMD_VEL_TIMEOUT_MS) {
    g_jetson_linear_mps  = 0.0f;
    g_jetson_angular_rps = 0.0f;
    g_error_flags |= ERR_CMD_VEL_TO;
  } else if (rx.any_cmd_vel_seen) {
    g_error_flags &= ~ERR_CMD_VEL_TO;
  }

  if (rx.any_arm_cmd_seen &&
      (now - rx.arm_cmd_stamp_ms) > ARM_CMD_TIMEOUT_MS) {
    // Zero cylinder targets on timeout so a stale "extend" command does not
    // keep driving the actuators after ROS stops publishing.
    g_jetson_cyl1_input = 0.0f;
    g_jetson_cyl2_input = 0.0f;
    g_jetson_arm_mode   = 0;
    g_error_flags |= ERR_ARM_TO;
  } else if (rx.any_arm_cmd_seen) {
    g_error_flags &= ~ERR_ARM_TO;
  }

  const bool sbus_ok =
#if SBUS_ENABLE
    g_sbus.is_valid() && !g_sbus.failsafe();
#else
    false;
#endif

#if SBUS_ENABLE
  if (!sbus_ok) {
    if ((g_error_flags & ERR_SBUS) == 0) {
      DBG_PRINTLN(F("WDG: SBUS no valid data"));
    }
    g_error_flags |= ERR_SBUS;
  } else {
    g_error_flags &= ~ERR_SBUS;
  }
#endif

  // Do not zero Jetson drive targets on heartbeat loss alone. The serial link
  // can briefly miss heartbeat frames while cmd_vel is still arriving; the
  // cmd_vel watchdog above is the authority for stale drive commands.

  // E-STOP from SBUS CH5 (switch_A) — HOLD style.
  // Disabled by default (SBUS_FOR_ESTOP=0): the RC radio is treated as an
  // optional manual override, not a safety interlock. E-STOP is driven by
  // the ROS SYS_CMD path only. This prevents the relay from cutting power
  // when the SBUS transmitter is turned off.
#if SBUS_ENABLE && SBUS_FOR_ESTOP
  if (sbus_ok) {
    g_sbus_ever_valid = true;
    const bool sw_stop = (g_sbus.channel(CH_ESTOP_SW) < 0.5f);
    if (sw_stop && !g_estop) {
      DBG_PRINTLN(F("SBUS: E-STOP engaged (CH5)"));
    } else if (!sw_stop && g_estop) {
      DBG_PRINTLN(F("SBUS: E-STOP released (CH5)"));
    }
    g_estop = sw_stop;
    if (sw_stop) {
      g_error_flags |= ERR_ESTOP;
    } else {
      g_error_flags &= ~ERR_ESTOP;
    }
  }
#else
#if SBUS_ENABLE
  if (sbus_ok) {
    g_sbus_ever_valid = true;
    // CH5 (switch A) edge → relay toggle.  Hysteresis + 100 ms debounce
    // prevent spurious flips from SBUS noise / momentary invalid frames
    // (sbus_ok=false → channel() returns stale 0 → false edge).
    //   Hysteresis: ON requires raw > 0.7, OFF requires raw < 0.3.
    //   Debounce:   the new state must persist 100 ms before it commits.
    static bool     s_ch5_committed = false;
    static bool     s_ch5_pending   = false;
    static uint32_t s_ch5_pend_ms   = 0;
    const float    ch5_raw = g_sbus.channel(CH_ESTOP_SW);
    const uint32_t now_ms  = millis();
    const bool     ch5_pending_new = s_ch5_committed
        ? (ch5_raw > 0.3f)   // currently ON, drop only when raw < 0.3
        : (ch5_raw > 0.7f);  // currently OFF, raise only when raw > 0.7
    if (ch5_pending_new != s_ch5_pending) {
      s_ch5_pending = ch5_pending_new;
      s_ch5_pend_ms = now_ms;
    }
    if (s_ch5_pending != s_ch5_committed &&
        (int32_t)(now_ms - s_ch5_pend_ms) >= 100) {
      s_ch5_committed = s_ch5_pending;
      g_relay_enable  = s_ch5_committed;
      DBG_PRINT(F("RELAY (SBUS CH5 edge) -> "));
      DBG_PRINTLN(g_relay_enable ? F("ON") : F("OFF"));
    }

    // CH6 OFF→ON edge: motor alarm reset.  Same hysteresis + debounce.
    static bool     s_ch6_committed = false;
    static bool     s_ch6_pending   = false;
    static uint32_t s_ch6_pend_ms   = 0;
    const float    ch6_raw = g_sbus.channel(CH_ALM_RST);
    const bool     ch6_pending_new = s_ch6_committed
        ? (ch6_raw > 0.3f)
        : (ch6_raw > 0.7f);
    if (ch6_pending_new != s_ch6_pending) {
      s_ch6_pending = ch6_pending_new;
      s_ch6_pend_ms = now_ms;
    }
    if (s_ch6_pending != s_ch6_committed &&
        (int32_t)(now_ms - s_ch6_pend_ms) >= 100) {
      const bool ch6_was_on_to_off = (s_ch6_committed && !s_ch6_pending);
      s_ch6_committed = s_ch6_pending;
      if (s_ch6_committed && !ch6_was_on_to_off) {
        DBG_PRINTLN(F("SBUS CH6 edge -> motor alarm reset"));
        g_motors.resetAlarms();
      }
    }
  }
#endif
#endif

  // Hardware e-stop: D22 modeSwitch. HIGH (energized) = e-stop ON.
  g_hw_estop = (digitalRead(pins::MODE_SWITCH) == HIGH);
#if YCON_ENABLE
  {
    const bool ycon_fresh = g_ycon.isControlFresh(millis(), YCON_CONTROL_TIMEOUT_MS);
    g_ycon_estop = ycon_fresh && (g_ycon.control().estop != 0);
  }
#else
  g_ycon_estop = false;
#endif
  g_estop    = g_sw_estop || g_hw_estop || g_ycon_estop;
  if (g_estop) {
    g_error_flags |= ERR_ESTOP;
  } else {
    g_error_flags &= ~ERR_ESTOP;
  }
  if (g_hw_estop) {
    g_error_flags |= ERR_HW_ESTOP;
  } else {
    g_error_flags &= ~ERR_HW_ESTOP;
  }

  // Power relay: (SBUS CH5 in SBUS-mode, or ROS relay_cmd) AND not estopped.
  // g_relay_enable is updated by SBUS when SBUS is valid, or by ROS
  // relay_cmd when SBUS is absent — whichever wrote last persists.
  const bool new_relay = g_relay_enable && !g_estop;
  if (new_relay != g_power_relay) {
    g_power_relay = new_relay;
    DBG_PRINT(F("RELAY -> "));
    DBG_PRINTLN(g_power_relay ? F("ON") : F("OFF"));

    if (g_power_relay) {
      // Relay just turned ON — BLV-R drivers are now powering up.
      // Schedule Modbus re-init after 2 s (driver power-on time).
      g_motors_reinit_pending = true;
      g_motors_reinit_at_ms   = millis() + 2000;
      DBG_PRINTLN(F("RELAY ON: motor reinit in 2 s"));
    }
  }
  digitalWrite(pins::RELAY_POWER, g_power_relay ? HIGH : LOW);

  // Warning lights
  const bool low_battery = g_battery_ok && (g_soc_percent < 10);
  const bool red_blink = ((millis() % 400) < 200);
  const bool ycon_selected = (g_active_source == CTRL_SRC_YCON);
  const bool ycon_blink = ((millis() % 200) < 100);
  digitalWrite(pins::LIGHT_RED, low_battery ? (red_blink ? HIGH : LOW)
                                            : (g_power_relay ? LOW : HIGH));
  digitalWrite(pins::LIGHT_YELLOW,
               ycon_selected ? (g_motor_active ? (ycon_blink ? HIGH : LOW) : HIGH)
                             : LOW);
}

static void zero_applied_commands()
{
  g_applied_linear_mps  = 0.0f;
  g_applied_angular_rps = 0.0f;
  g_applied_cyl1_input  = 0.0f;
  g_applied_cyl2_input  = 0.0f;
  g_applied_arm_mode    = 0;
}

// =====================================================================
// Arbitrate Jetson / SBUS / YCON.
// Jetson is the default. Manual sources take control only after stick intent
// persists, then return to Jetson after neutral input is held.
// =====================================================================
static void arbitrate_and_apply()
{
  const uint32_t now_arb = millis();
  const uint8_t old_source = g_active_source;

  bool sbus_ok = false;
#if SBUS_ENABLE
  sbus_ok = g_sbus.is_valid() && !g_sbus.failsafe();
#endif

  bool ycon_fresh = false;
#if YCON_ENABLE
  ycon_fresh = g_ycon.isControlFresh(now_arb, YCON_CONTROL_TIMEOUT_MS);
#endif

  if (g_control_override_seen) {
    const uint8_t override_source = g_link.rx().control_mode;
    if (override_source == CTRL_SRC_SBUS) {
      g_active_source = CTRL_SRC_SBUS;
    } else if (override_source == CTRL_SRC_YCON) {
      g_active_source = CTRL_SRC_YCON;
    } else {
      g_active_source = CTRL_SRC_JETSON;
    }

    g_manual_intent_source = CTRL_SRC_JETSON;
    g_manual_intent_tracking = false;
    g_manual_neutral_tracking = false;
  } else {
    uint8_t intent_source = CTRL_SRC_JETSON;
    float active_manual_magnitude = 0.0f;

#if SBUS_ENABLE
    const float sbus_mag = sbus_ok ? sbus_manual_magnitude() : 0.0f;
    if (sbus_mag >= MANUAL_INTENT_THRESHOLD) {
      intent_source = CTRL_SRC_SBUS;
    }
#endif

#if YCON_ENABLE
    const float ycon_mag = ycon_fresh ? ycon_manual_magnitude() : 0.0f;
    if (intent_source == CTRL_SRC_JETSON &&
        ycon_mag >= MANUAL_INTENT_THRESHOLD) {
      intent_source = CTRL_SRC_YCON;
    }
#endif

    if (intent_source != CTRL_SRC_JETSON) {
      if (!g_manual_intent_tracking ||
          g_manual_intent_source != intent_source) {
        g_manual_intent_source = intent_source;
        g_manual_intent_tracking = true;
        g_manual_intent_since_ms = now_arb;
      } else if ((int32_t)(now_arb - g_manual_intent_since_ms) >=
                 (int32_t)MANUAL_INTENT_DWELL_MS) {
        g_active_source = intent_source;
        g_manual_neutral_tracking = false;
      }
    } else {
      g_manual_intent_tracking = false;
      g_manual_intent_source = CTRL_SRC_JETSON;
    }

    if (g_active_source == CTRL_SRC_SBUS) {
      if (!sbus_ok) {
        g_active_source = CTRL_SRC_JETSON;
        g_manual_intent_tracking = false;
        g_manual_neutral_tracking = false;
        zero_applied_commands();
        if (old_source != g_active_source) {
          DBG_PRINT(F("CTRL -> "));
          DBG_PRINTLN(F("JETSON"));
        }
        return;
      }
#if SBUS_ENABLE
      active_manual_magnitude = sbus_manual_magnitude();
#endif
    } else if (g_active_source == CTRL_SRC_YCON) {
      if (!ycon_fresh) {
        g_active_source = CTRL_SRC_JETSON;
        g_manual_intent_tracking = false;
        g_manual_neutral_tracking = false;
        zero_applied_commands();
        if (old_source != g_active_source) {
          DBG_PRINT(F("CTRL -> "));
          DBG_PRINTLN(F("JETSON"));
        }
        return;
      }
#if YCON_ENABLE
      active_manual_magnitude = ycon_manual_magnitude();
#endif
    }

    if (g_active_source == CTRL_SRC_SBUS ||
        g_active_source == CTRL_SRC_YCON) {
      if (active_manual_magnitude < MANUAL_INTENT_THRESHOLD) {
        if (!g_manual_neutral_tracking) {
          g_manual_neutral_tracking = true;
          g_manual_neutral_since_ms = now_arb;
        } else if ((int32_t)(now_arb - g_manual_neutral_since_ms) >=
                   (int32_t)MANUAL_RETURN_NEUTRAL_MS) {
          g_active_source = CTRL_SRC_JETSON;
          g_manual_intent_tracking = false;
          g_manual_neutral_tracking = false;
        }
      } else {
        g_manual_neutral_tracking = false;
      }
    }
  }

  if (old_source != g_active_source) {
    DBG_PRINT(F("CTRL -> "));
    DBG_PRINTLN(
      g_active_source == CTRL_SRC_SBUS ? F("SBUS") :
      g_active_source == CTRL_SRC_YCON ? F("YCON") : F("JETSON"));
  }

  if (g_estop) {
    zero_applied_commands();
    return;
  }

#if SBUS_ENABLE
  if (g_active_source == CTRL_SRC_SBUS) {
    if (!sbus_ok) {
      zero_applied_commands();
      return;
    }
    const float lin = sbus_motor_axis(0, g_sbus.channel(CH_LINEAR));
    const float ang = sbus_motor_axis(1, g_sbus.channel(CH_ANGULAR));
    g_applied_linear_mps  = lin * SBUS_MAX_LINEAR_MPS;
    g_applied_angular_rps = ang * SBUS_MAX_ANGULAR_RPS;
    // Cylinders are on/off actuators. Use a separate, large threshold plus
    // debounce so SBUS jitter near center cannot twitch them.
    float cyl1 = g_sbus.channel(CH_CYL1);
    float cyl2 = g_sbus.channel(CH_CYL2);
    const uint32_t now_ms = millis();
    g_applied_cyl1_input  = sbus_cylinder_cmd(0, cyl1, now_ms);
    g_applied_cyl2_input  = -sbus_cylinder_cmd(1, cyl2, now_ms);
    g_applied_arm_mode    = 1;
    return;
  }
#endif

#if YCON_ENABLE
  if (g_active_source == CTRL_SRC_YCON) {
    if (ycon_fresh) {
      const YconControlData & yc = g_ycon.control();
      g_applied_linear_mps  = ((float)yc.linear_x / 1000.0f) * YCON_MAX_LINEAR_MPS;
      g_applied_angular_rps = ((float)yc.angular_z / 1000.0f) * YCON_MAX_ANGULAR_RPS;
      g_applied_cyl1_input  = arm_deadband((float)yc.arm / 1000.0f);
      g_applied_cyl2_input  = arm_deadband((float)yc.bucket / 1000.0f);
      g_applied_arm_mode    = 1;
    } else {
      zero_applied_commands();
    }
    return;
  }
#endif

  g_applied_linear_mps  = g_jetson_linear_mps;
  g_applied_angular_rps = g_jetson_angular_rps;
  g_applied_cyl1_input  = g_jetson_cyl1_input;
  g_applied_cyl2_input  = -g_jetson_cyl2_input;
  g_applied_arm_mode    = g_jetson_arm_mode;
}

// =====================================================================
// Drive the hardware
// =====================================================================
static void drive_hardware()
{
  // BLV-R: execute pending post-relay-on reinit when the timer expires.
  // Skip if relay turned OFF again before the timer fired.
  if (g_motors_reinit_pending &&
      (int32_t)(millis() - g_motors_reinit_at_ms) >= 0) {
    g_motors_reinit_pending = false;
    if (g_power_relay) {
      g_motors.reinitAfterPowerOn();   // ~120 ms blocking — one-time only
    }
  }

  // Stop everything when the power relay is open OR during E-STOP.
  // Cylinders share the relay-controlled supply, so they must also halt
  // when the relay is open — otherwise the last commanded direction keeps
  // driving the actuators even with the safety relay disengaged.
  if (!g_power_relay || g_estop) {
    g_motors.stop(/*brake=*/g_estop);
    g_cyls.stopAll();
    g_motor_active = false;
    return;
  }

  g_motors.drive(g_applied_linear_mps, g_applied_angular_rps);
  g_cyls.command(0, g_applied_cyl1_input);
  g_cyls.command(1, g_applied_cyl2_input);

  const float kEps = 1e-4f;
  g_motor_active =
    (fabsf(g_applied_linear_mps)  > kEps) ||
    (fabsf(g_applied_angular_rps) > kEps);
}

static int16_t sat_i16_local(int32_t v)
{
  if (v > 32767) { return 32767; }
  if (v < -32768) { return -32768; }
  return (int16_t)v;
}

static uint16_t ycon_cell_voltage_10mv()
{
  const uint16_t v1 = g_last_battery.mod1_voltage_10mv;
  const uint16_t v2 = g_last_battery.mod2_voltage_10mv;
  if (v1 > 0 && v2 > 0) {
    return (uint16_t)(((uint32_t)v1 + (uint32_t)v2) / 2);
  }
  return v1 > 0 ? v1 : v2;
}

static uint16_t ycon_capacity_01ah()
{
  // M5Stack UI displays SensorData.soc_mAh / 10.0 as Ah.
  return (uint16_t)((uint32_t)g_last_battery.soc_mah / 100);
}

static YconFeedbackSnapshot build_ycon_snapshot(uint32_t now)
{
  YconFeedbackSnapshot s;
  s.mcu_time_ms = now;
  s.error_flags = g_error_flags;
  s.active_source = g_active_source;
  s.relay_on = g_power_relay;
  s.estop = g_estop;
  s.ycon_valid = g_ycon.isControlFresh(now, 300);
  s.imu_valid = g_imu_ok;
  s.env_valid = g_env_ok;
  s.battery_valid = g_battery_ok;
  s.soc_percent = g_soc_percent;
  s.applied_linear_mmps = sat_i16_local((int32_t)(g_applied_linear_mps * 1000.0f));
  s.applied_angular_mradps = sat_i16_local((int32_t)(g_applied_angular_rps * 1000.0f));

  const CylinderFeedback fb1 = g_cyls.read(0);
  const CylinderFeedback fb2 = g_cyls.read(1);
  s.cyl1_01mm = sat_i16_local((int32_t)(fb1.pos_mm * 10.0f + 0.5f));
  s.cyl2_01mm = sat_i16_local((int32_t)(fb2.pos_mm * 10.0f + 0.5f));

  s.imu_euler_x_mdeg = g_last_imu.euler_x_mdeg;
  s.imu_euler_y_mdeg = g_last_imu.euler_y_mdeg;
  s.imu_euler_z_mdeg = g_last_imu.euler_z_mdeg;
  s.imu_quat_w_1e4 = g_last_imu.quat_w_1e4;
  s.imu_quat_x_1e4 = g_last_imu.quat_x_1e4;
  s.imu_quat_y_1e4 = g_last_imu.quat_y_1e4;
  s.imu_quat_z_1e4 = g_last_imu.quat_z_1e4;

  // The ycon/M5Stack UI expects a cell/module voltage, not the sum of two
  // BMS module voltages. Use the average when both modules are present.
  s.battery_voltage_10mv = ycon_cell_voltage_10mv();
  s.battery_current_10ma =
    sat_i16_local((int32_t)g_last_battery.mod1_current_10ma +
                  (int32_t)g_last_battery.mod2_current_10ma);
  s.soc_mah = ycon_capacity_01ah();
  s.battery_temp_max_01c =
    (g_last_battery.mod1_temp_max_01c > g_last_battery.mod2_temp_max_01c)
      ? g_last_battery.mod1_temp_max_01c : g_last_battery.mod2_temp_max_01c;
  s.battery_temp_min_01c =
    (g_last_battery.mod1_temp_min_01c < g_last_battery.mod2_temp_min_01c)
      ? g_last_battery.mod1_temp_min_01c : g_last_battery.mod2_temp_min_01c;
  s.env_temp_cdeg = g_last_env.temperature_cdeg;
  s.env_humidity_cpercent = g_last_env.humidity_cpercent;
  s.env_pressure_hpa = (uint16_t)(g_last_env.pressure_pa / 100);

  s.motor_l_rpm = sat_i16_local(g_motors.actualLeftRpm());
  s.motor_r_rpm = sat_i16_local(g_motors.actualRightRpm());
  s.torque_l_01pct = sat_i16_local(g_motors.torqueLeft());
  s.torque_r_01pct = sat_i16_local(g_motors.torqueRight());
  s.temp_drv_l_01c = sat_i16_local(g_motors.tempDriverLeft());
  s.temp_drv_r_01c = sat_i16_local(g_motors.tempDriverRight());
  s.alarm_l = (uint16_t)(g_motors.alarmLeft() & 0xffff);
  s.alarm_r = (uint16_t)(g_motors.alarmRight() & 0xffff);
  return s;
}

// =====================================================================
// Periodic senders
// =====================================================================
static void periodic_senders(uint32_t now)
{
  // Combined heartbeat + control_state + error_flags at 10 Hz.
  if ((int32_t)(now - g_next_sysstat_tx_ms) >= 0) {
    g_next_sysstat_tx_ms = now + SYSTEM_STATUS_PERIOD_MS;
    SystemStatus st;
    st.mcu_time_ms    = now;
    st.error_flags    = g_error_flags;
    st.system_state   = g_system_state;
    st.driver_variant = DRIVER_VARIANT_BLVR;
    st.control_mode   = g_active_source;
    st.comm_state     = g_comm_state;
    st.motor_state    =
      (g_applied_linear_mps == 0.0f && g_applied_angular_rps == 0.0f) ? 0 : 1;
    st.arm_state      = g_applied_arm_mode;
    st.relay_state    = g_power_relay ? 1 : 0;
    g_link.sendSystemStatus(st);
  }

  const uint32_t encoder_seq = g_motors.encoderSampleSeq();
  if (encoder_seq != g_last_encoder_tx_seq &&
      (int32_t)(now - g_next_encoder_tx_ms) >= 0) {
    g_next_encoder_tx_ms = now + ENCODER_PERIOD_MS;
    g_last_encoder_tx_seq = encoder_seq;
    Encoder enc;
    const CylinderFeedback fb1 = g_cyls.read(0);
    const CylinderFeedback fb2 = g_cyls.read(1);
    // Signed detected 32-bit counters from BLV-R drivers. These are the
    // "cnt L/R" values in debug output and are more stable for odometry than
    // the separate detected-position monitor, which can jump at wrap points.
    enc.left_motor_count  = g_motors.actualLeftCount();
    enc.right_motor_count = g_motors.actualRightCount();
    enc.cyl1_count        = (int32_t)(fb1.pos_mm * 10.0f + 0.5f);
    enc.cyl2_count        = (int32_t)(fb2.pos_mm * 10.0f + 0.5f);
    enc.sample_time_ms    = now;
    g_link.sendEncoder(enc);
  }

#if SBUS_ENABLE
  if ((int32_t)(now - g_next_sbus_joy_tx_ms) >= 0) {
    g_next_sbus_joy_tx_ms = now + SBUS_JOY_PERIOD_MS;
    SbusChannels sbus;
    for (uint8_t i = 0; i < 8; ++i) {
      float v = g_sbus.channel(i) * 10000.0f;
      if (v > 32767.0f) { v = 32767.0f; }
      if (v < -32768.0f) { v = -32768.0f; }
      sbus.ch[i] = (int16_t)v;
    }
    g_link.sendSbusJoy(sbus);
  }
#endif

  if ((int32_t)(now - g_next_imu_tx_ms) >= 0) {
    g_next_imu_tx_ms = now + IMU_PERIOD_MS;
#if IMU_BNO055_ENABLE
    Imu imu;
    if (g_imu.read(imu)) {
      g_last_imu = imu;
      g_imu_ok = true;
      g_link.sendImu(imu);
      g_error_flags &= ~ERR_SENSOR;
    } else {
      g_imu_ok = false;
      g_error_flags |= ERR_SENSOR;
    }
#else
    g_error_flags |= ERR_SENSOR;
#endif
  }

  if ((int32_t)(now - g_next_env_tx_ms) >= 0) {
    g_next_env_tx_ms = now + ENV_PERIOD_MS;
#if ENV_BME280_ENABLE
    EnvSensor env;
    if (g_env.read(env)) {
      g_last_env = env;
      g_env_ok = true;
      g_link.sendEnv(env);
    } else {
      g_env_ok = false;
    }
#endif
  }

  // Note: control_state (0x24), error_flags (0x25), and heartbeat (0x20)
  // are no longer sent individually — heartbeat/ctrl/error are bundled
  // into 0x26 SystemStatus above. SBUS stick values are still echoed as
  // 0x28 so Jetson-side data collection can observe manual operation.

#if CAN_BATTERY_ENABLE
  if ((int32_t)(now - g_next_battery_ms) >= 0) {
    g_next_battery_ms = now + BATTERY_PERIOD_MS;
    BatteryStatus bat;
    g_battery.fill(bat);
    g_last_battery = bat;
    g_battery_ok = g_battery.is_valid();
    if (g_battery_ok) {
      g_soc_percent = bat.soc_percent;
    }
    g_link.sendBattery(bat);
  }
#endif

#if YCON_ENABLE
  if ((int32_t)(now - g_next_ycon_ms) >= 0) {
    g_next_ycon_ms = now + 20;
    const bool ycon_ok = g_ycon.update(now, build_ycon_snapshot(now));
    if (ycon_ok || g_ycon.isControlFresh(now, 300)) {
      g_error_flags &= ~ERR_YCON;
    } else {
      g_error_flags |= ERR_YCON;
    }
  }
#endif

  // BLV-R motor diagnostics (torque / driver temp / motor temp / alarm)
  if ((int32_t)(now - g_next_motor_status_ms) >= 0) {
    g_next_motor_status_ms = now + MOTOR_STATUS_PERIOD_MS;
    MotorStatus ms;
    // Saturate int32 register values into int16 payload field.
    auto sat16 = [](int32_t v) -> int16_t {
      if (v >  32767) { return  32767; }
      if (v < -32768) { return -32768; }
      return (int16_t)v;
    };
    ms.torque_l_01pct = sat16(g_motors.torqueLeft());
    ms.torque_r_01pct = sat16(g_motors.torqueRight());
    ms.temp_drv_l_01c = sat16(g_motors.tempDriverLeft());
    ms.temp_drv_r_01c = sat16(g_motors.tempDriverRight());
    ms.temp_mot_l_01c = sat16(g_motors.tempMotorLeft());
    ms.temp_mot_r_01c = sat16(g_motors.tempMotorRight());
    ms.alarm_l = (uint16_t)(g_motors.alarmLeft()  & 0xFFFF);
    ms.alarm_r = (uint16_t)(g_motors.alarmRight() & 0xFFFF);
    ms.pos_l_step    = g_motors.posLeft();
    ms.pos_r_step    = g_motors.posRight();
    ms.odo_l_01krev  = g_motors.odoLeft();
    ms.odo_r_01krev  = g_motors.odoRight();
    ms.imain_l_001a  = g_motors.imainLeft();
    ms.imain_r_001a  = g_motors.imainRight();
    g_link.sendMotorStatus(ms);
  }

  if ((int32_t)(now - g_next_led_ms) >= 0) {
    g_next_led_ms = now + LED_UPDATE_PERIOD_MS;
    LedState s;
    s.active_src   = g_active_source;
    s.estop        = !g_power_relay;   // RED LED ON when power relay is OFF
    s.motor_active = g_motor_active;
    s.low_battery  = g_battery_ok && (g_soc_percent < 10);
    g_leds.update(s, now);
  }
}

// =====================================================================
// Arduino entry points
// =====================================================================
void setup()
{
  LINK_SERIAL.begin(LINK_BAUD);
  DBG_BEGIN();

  while (!LINK_SERIAL && millis() < 2000) {
    // wait briefly for USB-CDC enumeration
  }

  // LEDs first.
  g_leds.begin(pins::LED_RED, pins::LED_GREEN,
               pins::LED_BLUE, pins::LED_YELLOW);

  // Hardware e-stop switch (D22): HIGH = pressed/active, LOW = normal.
  pinMode(pins::MODE_SWITCH, INPUT);

  // Power relay & warning lights.
  pinMode(pins::RELAY_POWER,  OUTPUT);
  pinMode(pins::LIGHT_RED,    OUTPUT);
  pinMode(pins::LIGHT_YELLOW, OUTPUT);
  digitalWrite(pins::RELAY_POWER,  LOW);
  digitalWrite(pins::LIGHT_RED,    HIGH);
  digitalWrite(pins::LIGHT_YELLOW, LOW);

  g_link.begin(LINK_SERIAL);
#if DEBUG_ENABLE
  g_link.setDebugSink(&DEBUG_SERIAL);
#endif
#if SBUS_ENABLE
  g_sbus.begin(SBUS_SERIAL, /*inverted=*/false);
#endif

  // BLV-R motors via Modbus RTU on Serial2.
  // NOTE: setDebug() must be called BEFORE begin() so the auto-probe and
  // baud-migration logs from begin() are visible.  Otherwise the entire
  // boot-time probe sequence runs silently and we cannot tell whether
  // the BLV-R is at 115200, 230400, or unresponsive.
  ModbusDriveConfig mcfg;
  // mcfg defaults are suitable; override here if needed:
  // mcfg.max_rpm = 4000.0f;
  // mcfg.accel_ms = 500;
#if DEBUG_ENABLE
  g_motors.setDebug(&DEBUG_SERIAL);
#endif
  g_motors.begin(Serial2, pins::RS485_DE_RE, mcfg);
  DBG_PRINTLN(F("MOTOR: BLV-R Modbus RTU ready"));

  g_cyls.begin(cyl_pins(0), cyl_pins(1), cyl_pins(2));

#if I2C_REQUIRED
  if (!i2c_bus_idle() && !i2c_recover_bus()) {
    g_error_flags |= ERR_SENSOR;
    DBG_PRINT(F("I2C: bus stuck SDA="));
    DBG_PRINT(digitalRead(I2C_SDA_PIN));
    DBG_PRINT(F(" SCL="));
    DBG_PRINTLN(digitalRead(I2C_SCL_PIN));
  } else {
    Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
    // Avoid a dead I2C device or wiring fault blocking the main loop and
    // stopping the Jetson UART frames entirely.
    Wire.setWireTimeout(25000, true);
#endif

    DBG_PRINTLN(F("I2C: begin"));

#if I2C_SCAN_ON_BOOT
    i2c_scan_debug();
#endif

#if IMU_BNO055_ENABLE
    uint8_t imu_addr = 0;
    if (!imu_detect_addr(imu_addr)) {
      g_error_flags |= ERR_SENSOR;
      DBG_PRINTLN(F("IMU: no ACK at 0x29 or 0x28 (sensor error)"));
    } else if (!g_imu.begin(imu_addr, &Wire)) {
      g_error_flags |= ERR_SENSOR;
      DBG_PRINTLN(F("IMU: BNO055 not found (sensor error)"));
    } else {
      DBG_PRINT(F("IMU: BNO055 ready at 0x"));
      DBG_PRINT2(imu_addr, HEX);
      DBG_PRINTLN(F(""));
    }
#else
    g_error_flags |= ERR_SENSOR;
    DBG_PRINTLN(F("IMU: disabled"));
#endif

#if ENV_BME280_ENABLE
    if (!i2c_present(0x76)) {
      DBG_PRINTLN(F("ENV: no ACK at 0x76"));
    } else if (!g_env.begin(0x76, &Wire)) {
      DBG_PRINTLN(F("ENV: BME280 not found at 0x76"));
    } else {
      DBG_PRINTLN(F("ENV: BME280 ready"));
    }
#else
    DBG_PRINTLN(F("ENV: disabled"));
#endif

#if YCON_ENABLE
    g_ycon.begin(YCON_I2C_ADDRESS, &Wire);
    DBG_PRINT(F("YCON: I2C bridge enabled at 0x"));
    DBG_PRINT2(YCON_I2C_ADDRESS, HEX);
    DBG_PRINTLN(F(""));
#endif
  }
#else
  g_error_flags |= ERR_SENSOR;
  DBG_PRINTLN(F("I2C: sensors disabled"));
#endif

#if CAN_BATTERY_ENABLE
  if (!g_battery.begin(pins::CAN_SPI_CS, pins::CAN_INT)) {
    DBG_PRINTLN(F("CAN: MCP2515 init FAIL"));
  } else {
    DBG_PRINTLN(F("CAN: battery BMS ready"));
  }
#endif

  const uint32_t now = millis();
  g_next_sysstat_tx_ms = now;
  g_next_encoder_tx_ms = now;
  g_next_sbus_joy_tx_ms = now;
  g_next_imu_tx_ms     = now;
  g_next_env_tx_ms     = now;
  g_next_battery_ms    = now;
  g_next_motor_status_ms = now;
  g_next_led_ms        = now;
  g_next_ycon_ms       = now;

  DBG_PRINTLN(F("=== blvr_arduino_mega boot ==="));
  DBG_PRINT(F("LINK baud=")); DBG_PRINTLN(LINK_BAUD);
  DBG_PRINT(F("MODBUS baud=")); DBG_PRINTLN(mcfg.baud);
#if SBUS_ENABLE
  DBG_PRINT(F("SBUS on Serial1 RX=D19, inverted=false, baud="));
  DBG_PRINTLN(SBUS_BAUD);
#endif
}

// =====================================================================
// Debug: periodic SBUS status print
// =====================================================================
#if SBUS_ENABLE && DEBUG_ENABLE
static uint32_t g_next_sbus_dbg_ms = 0;
static void debug_sbus_status(uint32_t now)
{
  if ((int32_t)(now - g_next_sbus_dbg_ms) < 0) { return; }
  g_next_sbus_dbg_ms = now + 2000;

  DEBUG_SERIAL.print(F("SBUS: rx="));
  DEBUG_SERIAL.print(g_sbus.rx_count());
  DEBUG_SERIAL.print(F(" valid="));
  DEBUG_SERIAL.print(g_sbus.is_valid() ? '1' : '0');
  DEBUG_SERIAL.print(F(" fs="));
  DEBUG_SERIAL.print(g_sbus.failsafe() ? '1' : '0');
  DEBUG_SERIAL.print(F(" raw=["));
  for (uint8_t i = 0; i < 8; ++i) {
    DEBUG_SERIAL.print(g_sbus.channelRaw(i));
    if (i < 7) { DEBUG_SERIAL.print(','); }
  }
  DEBUG_SERIAL.print(F("] norm=["));
  for (uint8_t i = 0; i < 8; ++i) {
    DEBUG_SERIAL.print(g_sbus.channel(i), 2);
    if (i < 7) { DEBUG_SERIAL.print(','); }
  }
  DEBUG_SERIAL.println(']');
}
#endif

// =====================================================================
// Debug: periodic battery CAN dump
// =====================================================================
#if CAN_BATTERY_ENABLE && DEBUG_ENABLE
static uint32_t g_next_bat_dbg_ms = 0;
static void debug_battery_status(uint32_t now)
{
  if ((int32_t)(now - g_next_bat_dbg_ms) < 0) { return; }
  g_next_bat_dbg_ms = now + 3000;

  if (!g_battery.is_valid()) {
    DEBUG_SERIAL.println(F("BAT: no CAN data yet"));
    return;
  }
  g_battery.debugDump(DEBUG_SERIAL);
}
#endif

// =====================================================================
// Debug: compact robot status print
// =====================================================================
#if DEBUG_ENABLE
static uint32_t g_next_robot_dbg_ms = 0;
static const __FlashStringHelper * control_source_name(uint8_t src)
{
  switch (src) {
    case CTRL_SRC_SBUS:
      return F("SBUS");
    case CTRL_SRC_YCON:
      return F("YCON");
    case CTRL_SRC_JETSON:
    default:
      return F("JETSON");
  }
}

static void debug_robot_status(uint32_t now)
{
  if ((int32_t)(now - g_next_robot_dbg_ms) < 0) { return; }
  g_next_robot_dbg_ms = now + 1000;

  DEBUG_SERIAL.print(F("STAT mode="));
  DEBUG_SERIAL.print(control_source_name(g_active_source));
  DEBUG_SERIAL.print(F(" relay="));
  DEBUG_SERIAL.print(g_power_relay ? F("ON") : F("OFF"));
  DEBUG_SERIAL.print(F(" relay_cmd="));
  DEBUG_SERIAL.print(g_relay_enable ? F("ON") : F("OFF"));
  DEBUG_SERIAL.print(F(" estop="));
  DEBUG_SERIAL.print(g_estop ? '1' : '0');
  DEBUG_SERIAL.print(F(" ycon_estop="));
  DEBUG_SERIAL.print(g_ycon_estop ? '1' : '0');
  DEBUG_SERIAL.print(F(" in v="));
  DEBUG_SERIAL.print(g_applied_linear_mps, 3);
  DEBUG_SERIAL.print(F(" w="));
  DEBUG_SERIAL.print(g_applied_angular_rps, 3);
  DEBUG_SERIAL.print(F(" cyl=["));
  DEBUG_SERIAL.print(g_applied_cyl1_input, 2);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(g_applied_cyl2_input, 2);
  DEBUG_SERIAL.print(F("] cmd_rpm=["));
  DEBUG_SERIAL.print(g_motors.lastLeftRpm(), 0);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(g_motors.lastRightRpm(), 0);
  DEBUG_SERIAL.print(F("] fb_rpm=["));
  DEBUG_SERIAL.print(g_motors.actualLeftRpm());
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(g_motors.actualRightRpm());
  DEBUG_SERIAL.print(F("] err=0x"));
  DEBUG_SERIAL.println(g_error_flags, HEX);
}
#endif

// =====================================================================
// Debug: periodic ycon status print
// =====================================================================
#if YCON_ENABLE && DEBUG_ENABLE
static uint32_t g_next_ycon_dbg_ms = 0;
static void debug_ycon_status(uint32_t now)
{
  if ((int32_t)(now - g_next_ycon_dbg_ms) < 0) { return; }
  g_next_ycon_dbg_ms = now + 2000;

  const YconControlData & yc = g_ycon.control();
  const bool fresh = g_ycon.isControlFresh(now, YCON_CONTROL_TIMEOUT_MS);
  const uint32_t age = g_ycon.lastReadMs() == 0 ? 0 : (uint32_t)(now - g_ycon.lastReadMs());

  DEBUG_SERIAL.print(F("YCON: present="));
  DEBUG_SERIAL.print(g_ycon.isPresent() ? '1' : '0');
  DEBUG_SERIAL.print(F(" fresh="));
  DEBUG_SERIAL.print(fresh ? '1' : '0');
  DEBUG_SERIAL.print(F(" age="));
  DEBUG_SERIAL.print(age);
  DEBUG_SERIAL.print(F("ms err="));
  DEBUG_SERIAL.print(g_ycon.errorCount());
  DEBUG_SERIAL.print(F(" valid="));
  DEBUG_SERIAL.print(yc.valid);
  DEBUG_SERIAL.print(F(" estop="));
  DEBUG_SERIAL.print(yc.estop);
  DEBUG_SERIAL.print(F(" seq="));
  DEBUG_SERIAL.print(yc.seq);
  DEBUG_SERIAL.print(F(" fbseq="));
  DEBUG_SERIAL.print(g_ycon.feedbackSeq());
  DEBUG_SERIAL.print(F(" updated="));
  DEBUG_SERIAL.print(yc.updated_ms);
  DEBUG_SERIAL.print(F(" cmd=["));
  DEBUG_SERIAL.print(yc.linear_x);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(yc.angular_z);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(yc.arm);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(yc.bucket);
  DEBUG_SERIAL.print(F("] map=[v:"));
  DEBUG_SERIAL.print(((float)yc.linear_x / 1000.0f) * YCON_MAX_LINEAR_MPS, 3);
  DEBUG_SERIAL.print(F(",w:"));
  DEBUG_SERIAL.print(((float)yc.angular_z / 1000.0f) * YCON_MAX_ANGULAR_RPS, 3);
  DEBUG_SERIAL.print(F(",c1:"));
  DEBUG_SERIAL.print(arm_deadband((float)yc.arm / 1000.0f), 2);
  DEBUG_SERIAL.print(F(",c2:"));
  DEBUG_SERIAL.print(arm_deadband((float)yc.bucket / 1000.0f), 2);
  DEBUG_SERIAL.print(F("] active="));
  DEBUG_SERIAL.print(
    g_active_source == CTRL_SRC_SBUS ? F("SBUS") :
    g_active_source == CTRL_SRC_YCON ? F("YCON") : F("JETSON"));
  DEBUG_SERIAL.print(F(" fb_ok=[imu:"));
  DEBUG_SERIAL.print(g_imu_ok ? '1' : '0');
  DEBUG_SERIAL.print(F(",env:"));
  DEBUG_SERIAL.print(g_env_ok ? '1' : '0');
  DEBUG_SERIAL.print(F(",bat:"));
  DEBUG_SERIAL.print(g_battery_ok ? '1' : '0');
  DEBUG_SERIAL.print(F("] bat="));
  DEBUG_SERIAL.print(ycon_cell_voltage_10mv());
  DEBUG_SERIAL.print(F("x10mV curr="));
  DEBUG_SERIAL.print((int16_t)((int32_t)g_last_battery.mod1_current_10ma +
                               (int32_t)g_last_battery.mod2_current_10ma));
  DEBUG_SERIAL.print(F("x10mA cap="));
  DEBUG_SERIAL.print(ycon_capacity_01ah());
  DEBUG_SERIAL.print(F("x0.1Ah soc="));
  DEBUG_SERIAL.print(g_last_battery.soc_percent);
  DEBUG_SERIAL.print(F("% T=["));
  DEBUG_SERIAL.print((g_last_battery.mod1_temp_max_01c > g_last_battery.mod2_temp_max_01c)
      ? g_last_battery.mod1_temp_max_01c : g_last_battery.mod2_temp_max_01c);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print((g_last_battery.mod1_temp_min_01c < g_last_battery.mod2_temp_min_01c)
      ? g_last_battery.mod1_temp_min_01c : g_last_battery.mod2_temp_min_01c);
  DEBUG_SERIAL.print(']');
  DEBUG_SERIAL.print(F("% euler_mdeg=["));
  DEBUG_SERIAL.print(g_last_imu.euler_x_mdeg);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(g_last_imu.euler_y_mdeg);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(g_last_imu.euler_z_mdeg);
  DEBUG_SERIAL.print(F("] env=["));
  DEBUG_SERIAL.print(g_last_env.temperature_cdeg);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(g_last_env.humidity_cpercent);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(g_last_env.pressure_pa);
  DEBUG_SERIAL.println(']');
}
#endif

void loop()
{
  g_link.update();
#if SBUS_ENABLE
  g_sbus.update();
#endif
#if CAN_BATTERY_ENABLE
  g_battery.update();
#endif
  g_motors.update();    // Modbus state machine — must run every loop
#if SBUS_ENABLE
  // Modbus traffic can take long enough for new SBUS frames to arrive.
  // Drain again immediately before arbitration so stale stick positions do
  // not survive one extra control cycle.
  g_sbus.update();
#endif

  consume_link_rx();
  update_watchdogs();
  arbitrate_and_apply();
  drive_hardware();

  const uint32_t now_ms = millis();
  periodic_senders(now_ms);

  #if DEBUG_ENABLE
    debug_robot_status(now_ms);
  #endif
  // Keep SBUS debug visible while tuning controller deadbands.
  #if SBUS_ENABLE && DEBUG_ENABLE
    debug_sbus_status(now_ms);
  #endif
  #if YCON_ENABLE && DEBUG_ENABLE
    debug_ycon_status(now_ms);
  #endif
  // #if CAN_BATTERY_ENABLE && DEBUG_ENABLE
  //   debug_battery_status(now_ms);
  // #endif
}
