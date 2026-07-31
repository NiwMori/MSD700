// blv_arduino_mega.ino — main sketch.
//
// Everything that touches hardware lives in its own translation unit:
//   - msd_link.*       wire protocol (frame parser + senders)
//   - sbus.*           S.BUS receiver
//   - leds.*           status LEDs (R/G/B/Y)
//   - motors.*         BLV differential drive
//   - cylinders.*      LINAK linear actuators + feedback
//   - imu_bno055.*     BNO055 orientation sensor
//   - pins.h           all pin assignments
//
// This file just instantiates the modules and orchestrates them.
//
// UART map (current):
//   Serial   (USB-CDC)     — LINK: ROS2 protocol to Jetson, also Arduino
//                            IDE upload. Planned: move LINK to Serial3
//                            when Jetson is wired to UART3; Serial USB
//                            then becomes programming + debug only.
//   Serial1  (pins 18/19)  — SBUS input (RX on D19). 100000 8E2 inverted;
//                            use an SBUS receiver that emits non-inverted
//                            signal OR put a hardware inverter on D19.
//   Serial2  (pins 16/17)  — debug print (TX on D16). Hook a USB-TTL
//                            cable to D16+GND to read logs.
//   Serial3  (pins 14/15)  — reserved for future Jetson UART link.

#include "pins.h"
#include "protocol.h"
#include "msd_link.h"
#include "sbus_reader.h"
#include "leds.h"
#include "motors.h"
#include "cylinders.h"
#include "imu_bno055.h"
#include "env_bme280.h"
#include "battery_can.h"

using namespace msd_arduino;
using namespace msd_arduino::protocol;

// =====================================================================
// Build-time configuration
// =====================================================================
#define LINK_SERIAL     Serial3        // hardware UART → Jetson (TX3=pin14, RX3=pin15)
static const uint32_t LINK_BAUD = 115200;

#define DEBUG_ENABLE    1
#define DEBUG_SERIAL    Serial       // USB-CDC (for debug monitoring via Arduino IDE)
static const uint32_t DEBUG_BAUD = 115200;

#define SBUS_ENABLE     1
#define SBUS_SERIAL     Serial1       // RX on D19
static const uint32_t SBUS_BAUD = 100000;

// Full-scale command mapping used when SBUS drives the robot.
static const float SBUS_MAX_LINEAR_MPS   = 0.5f;   // 500 mm/s
static const float SBUS_MAX_ANGULAR_RPS  = 0.79f;   // 2.0 rad/s
static const float SBUS_MOTOR_THRESHOLD  = 0.3f;   // stick must exceed ±0.3 to move

// When 0, SBUS link loss does NOT drive E-STOP and the SBUS CH5 switch
// is ignored — Jetson-only operation is allowed to continue. Set to 1
// only if the SBUS radio is a required safety layer.
#define SBUS_FOR_ESTOP  0

// When 0, SBUS CH8 does not switch control source. Jetson-side
// /msd/cmd/control can still request SBUS mode.
#define SBUS_CH8_MODE_SWITCH_ENABLE 0

// Default controller source. Set to CTRL_SRC_JETSON or CTRL_SRC_SBUS.
#define DEFAULT_CONTROL_SOURCE CTRL_SRC_SBUS

// When 0, the controller source is fixed to DEFAULT_CONTROL_SOURCE.
// When 1, CH8 (if enabled) and Jetson-side /msd/cmd/control can switch it.
#define CONTROL_SOURCE_SWITCH_ENABLE 0

// Hysteresis (ms) for Jetson↔SBUS source arbitration. Prevents flicker
// when SBUS validity toggles rapidly.
static const uint32_t SBUS_SWITCH_HYST_MS = 500;

// SBUS channel assignments (0-indexed, i.e. CH1=0, CH8=7).
//   ch1 (idx 0) = right_joy_x → cylinder 2
//   ch2 (idx 1) = left_joy_y  → drive linear.x
//   ch3 (idx 2) = right_joy_y → cylinder 1
//   ch4 (idx 3) = left_joy_x  → drive angular.z
//   ch5 (idx 4) = switch_A    → E-STOP
//   ch6 (idx 5) = switch_B    → (reserved)
//   ch7 (idx 6) = switch_D    → (reserved)
//   ch8 (idx 7) = switch_G    → Jetson/SBUS mode switch, if enabled
static const uint8_t CH_LINEAR    = 1;  // CH2: left_joy_y → linear.x
static const uint8_t CH_ANGULAR   = 3;  // CH4: left_joy_x → angular.z
static const uint8_t CH_CYL1      = 2;  // CH3: right_joy_y → cylinder 1
static const uint8_t CH_CYL2      = 0;  // CH1: right_joy_x → cylinder 2
static const uint8_t CH_ESTOP_SW  = 4;  // CH5: switch_A → E-STOP (> 0.5)
static const uint8_t CH_MANUAL_SW = 7;  // CH8: switch_G → SBUS priority (> 0.5), if enabled

// CAN battery enable
#define CAN_BATTERY_ENABLE 1

// Outbound send periods (ms).
static const uint32_t MCU_HB_PERIOD_MS  = 50;
static const uint32_t ENCODER_PERIOD_MS = 20;
static const uint32_t IMU_PERIOD_MS     = 20;
static const uint32_t ENV_PERIOD_MS     = 200;
static const uint32_t CTRL_PERIOD_MS    = 100;
static const uint32_t ERROR_PERIOD_MS   = 200;
static const uint32_t SBUS_JOY_PERIOD_MS = 100;   // 10 Hz, debug
static const uint32_t BATTERY_PERIOD_MS  = 500;   // 2 Hz
static const uint32_t LED_UPDATE_PERIOD_MS = 10;   // 100 Hz so fast blinks look clean

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
static MsdProtocolLink g_link;
#if SBUS_ENABLE
static SbusReader      g_sbus;
#endif
static StatusLeds      g_leds;
static DiffDriveMotors g_motors;
static Cylinders       g_cyls;
static ImuBno055       g_imu;
static EnvBme280       g_env;
#if CAN_BATTERY_ENABLE
static BatteryCan      g_battery;
#endif

// =====================================================================
// Arbitrated targets (fed to the hardware each loop)
// =====================================================================
static float   g_applied_linear_mps  = 0.0f;
static float   g_applied_angular_rps = 0.0f;
static float   g_applied_cyl1_input  = 0.0f;   // -1..+1
static float   g_applied_cyl2_input  = 0.0f;
static uint8_t g_applied_arm_mode    = 0;

// Jetson's last known targets (zeroed on watchdog / estop).
static float   g_jetson_linear_mps   = 0.0f;
static float   g_jetson_angular_rps  = 0.0f;
static float   g_jetson_cyl1_input   = 0.0f;
static float   g_jetson_cyl2_input   = 0.0f;
static uint8_t g_jetson_arm_mode     = 0;
static uint8_t g_system_state        = 0;

static uint8_t  g_active_source = DEFAULT_CONTROL_SOURCE;
static uint8_t  g_comm_state    = COMM_INIT;
static uint32_t g_error_flags   = 0;
static bool     g_sw_estop      = false;  // set/cleared by SYS_CMD_ESTOP / SYS_CMD_ESTOP_CLEAR
static bool     g_hw_estop      = false;  // set when D22 modeSwitch is LOW
static bool     g_estop         = false;  // combined: g_sw_estop || g_hw_estop
static bool     g_motor_active  = false;  // LED debug flag
static bool     g_power_relay   = false;  // true = power ON
static bool     g_relay_enable  = false;  // commanded by ROS relay topic
static bool     g_sbus_ever_valid = false; // true after first valid SBUS frame
static uint8_t  g_soc_percent   = 100;    // latest SOC for low-battery warning

// Hysteresis state for Jetson↔SBUS arbitration.
static bool     g_sbus_manual_want     = false;
static uint32_t g_sbus_manual_since_ms = 0;

// Schedule timestamps.
static uint32_t g_next_hb_tx_ms      = 0;
static uint32_t g_next_encoder_tx_ms = 0;
static uint32_t g_next_imu_tx_ms     = 0;
static uint32_t g_next_env_tx_ms     = 0;
static uint32_t g_next_ctrl_tx_ms    = 0;
static uint32_t g_next_err_tx_ms     = 0;
static uint32_t g_next_sbus_joy_ms   = 0;
static uint32_t g_next_battery_ms    = 0;
static uint32_t g_next_led_ms        = 0;

// =====================================================================
// Pin bundles (from pins.h)
// =====================================================================
static MotorPins left_pins() {
  MotorPins p;
  p.fwd       = pins::M_LEFT_FWD;
  p.rev       = pins::M_LEFT_REV;
  p.stop_mode = pins::M_LEFT_STOP_MODE;
  p.m0        = pins::M_LEFT_M0;
  p.mb_free   = pins::M_LEFT_MB_FREE;
  p.vm        = pins::M_LEFT_VM;
  return p;
}
static MotorPins right_pins() {
  MotorPins p;
  p.fwd       = pins::M_RIGHT_FWD;
  p.rev       = pins::M_RIGHT_REV;
  p.stop_mode = pins::M_RIGHT_STOP_MODE;
  p.m0        = pins::M_RIGHT_M0;
  p.mb_free   = pins::M_RIGHT_MB_FREE;
  p.vm        = pins::M_RIGHT_VM;
  return p;
}
static CylinderPins cyl_pins(uint8_t i) {
  CylinderPins p;
  switch (i) {
    case 0:
      p.fwd=pins::CYL1_FWD; p.rev=pins::CYL1_REV;
      p.end_in=pins::CYL1_END_IN; p.end_out=pins::CYL1_END_OUT;
      p.feedback=pins::CYL1_FEEDBACK; break;
    case 1:
      p.fwd=pins::CYL2_FWD; p.rev=pins::CYL2_REV;
      p.end_in=pins::CYL2_END_IN; p.end_out=pins::CYL2_END_OUT;
      p.feedback=pins::CYL2_FEEDBACK; break;
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
    // Interpret cyl*_target as normalized input ×1000 (so ±1000 = ±1.0).
    g_jetson_cyl1_input =
      (float)rx.arm_cmd.cyl1_target / 1000.0f;
    g_jetson_cyl2_input =
      (float)rx.arm_cmd.cyl2_target / 1000.0f;
    g_jetson_arm_mode = rx.arm_cmd.command_mode;
  }

  if (rx.control_mode_fresh) {
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

  // SBUS error flag: set when receiver is not providing valid data.
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

  if (g_comm_state == COMM_LOST && !sbus_ok) {
    g_jetson_linear_mps  = 0.0f;
    g_jetson_angular_rps = 0.0f;
  }

  // E-STOP from SBUS CH5 (switch_A) — disabled by default (SBUS_FOR_ESTOP=0).
  // Treating SBUS as optional lets Jetson-only operation continue when the
  // RC transmitter is off; the receiver's failsafe channel values would
  // otherwise latch E-STOP and cut the relay.
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
    // Edge-detect CH5 (switch A) so either SBUS or ROS relay_cmd can
    // toggle the relay without one source continuously overriding the
    // other.  Only a LOW→HIGH or HIGH→LOW transition writes g_relay_enable;
    // a steady CH5 state leaves the last write (from SBUS or ROS) intact.
    static bool s_ch5_prev = false;
    const bool ch5_now = (g_sbus.channel(CH_ESTOP_SW) > 0.5f);
    if (ch5_now != s_ch5_prev) {
      s_ch5_prev     = ch5_now;
      g_relay_enable = ch5_now;
      DBG_PRINT(F("RELAY (SBUS CH5 edge) -> "));
      DBG_PRINTLN(g_relay_enable ? F("ON") : F("OFF"));
    }
  }
#endif
#endif

  // Hardware e-stop: D22 modeSwitch. HIGH (energized) = e-stop ON.
  g_hw_estop = (digitalRead(pins::MODE_SWITCH) == HIGH);
  g_estop    = g_sw_estop || g_hw_estop;
  if (g_estop) {
    g_error_flags |= ERR_ESTOP;
  } else {
    g_error_flags &= ~ERR_ESTOP;
  }

  // Power relay: (SBUS CH5 or ROS relay_cmd) AND not estopped.
  // g_relay_enable is updated by SBUS when SBUS is valid, or by ROS
  // relay_cmd when SBUS is absent — whichever wrote last persists.
  const bool new_relay = g_relay_enable && !g_estop;
  if (new_relay != g_power_relay) {
    g_power_relay = new_relay;
    DBG_PRINT(F("RELAY -> "));
    DBG_PRINT(g_power_relay ? F("ON") : F("OFF"));
    DBG_PRINT(F("  enable="));
    DBG_PRINT(g_relay_enable ? '1' : '0');
    DBG_PRINT(F("  estop="));
    DBG_PRINTLN(g_estop ? '1' : '0');
  }
  digitalWrite(pins::RELAY_POWER, g_power_relay ? HIGH : LOW);

  // Warning lights
  digitalWrite(pins::LIGHT_RED, g_power_relay ? LOW : HIGH);   // red ON when power OFF
  digitalWrite(pins::LIGHT_YELLOW, (g_soc_percent < 10) ? HIGH : LOW);
}

// =====================================================================
// Arbitrate SBUS vs Jetson
// =====================================================================
static void arbitrate_and_apply()
{
#if CONTROL_SOURCE_SWITCH_ENABLE
  // SBUS CH8 toggle physically requests SBUS mode when enabled.
  const bool sbus_ch8_req =
#if SBUS_ENABLE && SBUS_CH8_MODE_SWITCH_ENABLE
    g_sbus.is_valid() && !g_sbus.failsafe() &&
    (g_sbus.channel(CH_MANUAL_SW) > 0.5f);
#else
    false;
#endif

  // ROS /msd/cmd/control can also request SBUS mode (override from Jetson).
  const bool ros_sbus_req =
    (g_link.rx().control_mode == CTRL_SRC_SBUS);

  const bool sbus_manual_req = sbus_ch8_req || ros_sbus_req;

  // Hysteresis: request must be stable for SBUS_SWITCH_HYST_MS before the
  // active source is flipped. Without this, SBUS validity flicker (e.g.
  // transmitter losing link) causes rapid SBUS↔Jetson switching.
  const uint32_t now_arb = millis();
  if (sbus_manual_req != g_sbus_manual_want) {
    g_sbus_manual_want     = sbus_manual_req;
    g_sbus_manual_since_ms = now_arb;
  }
  const bool stable =
    (now_arb - g_sbus_manual_since_ms) >= SBUS_SWITCH_HYST_MS;
  const bool sbus_manual = stable ? g_sbus_manual_want
                                  : (g_active_source == CTRL_SRC_SBUS);

  const uint8_t new_source = sbus_manual ? CTRL_SRC_SBUS : CTRL_SRC_JETSON;
#else
  const uint8_t new_source = DEFAULT_CONTROL_SOURCE;
#endif
  if (new_source != g_active_source) {
    g_active_source = new_source;
    DBG_PRINT(F("CTRL -> "));
    DBG_PRINTLN(new_source == CTRL_SRC_SBUS ? F("SBUS") : F("JETSON"));
  }

  if (g_estop) {
    g_applied_linear_mps  = 0.0f;
    g_applied_angular_rps = 0.0f;
    g_applied_cyl1_input  = 0.0f;
    g_applied_cyl2_input  = 0.0f;
    g_applied_arm_mode    = 0;
    return;
  }

#if SBUS_ENABLE
  if (new_source == CTRL_SRC_SBUS) {
    // Apply threshold: stick must exceed ±SBUS_MOTOR_THRESHOLD to move.
    float lin = g_sbus.channel(CH_LINEAR);
    float ang = g_sbus.channel(CH_ANGULAR);
    if (fabsf(lin) < SBUS_MOTOR_THRESHOLD) { lin = 0.0f; }
    if (fabsf(ang) < SBUS_MOTOR_THRESHOLD) { ang = 0.0f; }
    g_applied_linear_mps  = lin * SBUS_MAX_LINEAR_MPS;
    g_applied_angular_rps = ang * SBUS_MAX_ANGULAR_RPS;
    // Apply the same dead-zone threshold to cylinder sticks to prevent
    // micro-movement from stick trim or slight off-centre rest position.
    float cyl1 = g_sbus.channel(CH_CYL1);
    float cyl2 = g_sbus.channel(CH_CYL2);
    if (fabsf(cyl1) < SBUS_MOTOR_THRESHOLD) { cyl1 = 0.0f; }
    if (fabsf(cyl2) < SBUS_MOTOR_THRESHOLD) { cyl2 = 0.0f; }
    g_applied_cyl1_input  = -cyl1;  // CH3 inverted (hardware direction)
    g_applied_cyl2_input  = cyl2;
    g_applied_arm_mode    = 1;
    return;
  }
#endif

  g_applied_linear_mps  = g_jetson_linear_mps;
  g_applied_angular_rps = g_jetson_angular_rps;
  g_applied_cyl1_input  = -g_jetson_cyl1_input;
  g_applied_cyl2_input  = g_jetson_cyl2_input;
  g_applied_arm_mode    = g_jetson_arm_mode;
}

// =====================================================================
// Drive the hardware
// =====================================================================
static void drive_hardware()
{
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

  // Engage electromagnetic brake when velocity command is zero so the robot
  // holds position instead of coasting.
  const float kEps = 1e-4f;
  const bool motors_zero =
    (fabsf(g_applied_linear_mps) <= kEps) &&
    (fabsf(g_applied_angular_rps) <= kEps);
  if (motors_zero) {
    g_motors.stop(/*brake=*/true);
  } else {
    g_motors.drive(g_applied_linear_mps, g_applied_angular_rps);
  }
  g_motor_active = !motors_zero;

  g_cyls.command(0, g_applied_cyl1_input);
  g_cyls.command(1, g_applied_cyl2_input);
}

// =====================================================================
// Periodic senders
// =====================================================================
static void periodic_senders(uint32_t now)
{
  if ((int32_t)(now - g_next_hb_tx_ms) >= 0) {
    g_next_hb_tx_ms = now + MCU_HB_PERIOD_MS;
    g_link.sendMcuHeartbeat(now, g_system_state, DRIVER_VARIANT_BLV);
  }

  if ((int32_t)(now - g_next_encoder_tx_ms) >= 0) {
    g_next_encoder_tx_ms = now + ENCODER_PERIOD_MS;
    Encoder enc;
    const CylinderFeedback fb1 = g_cyls.read(0);
    const CylinderFeedback fb2 = g_cyls.read(1);
    // No real motor encoder — report theoretical RPM from kinematics.
    // Units: plain integer RPM (fractional part truncated).
    enc.left_motor_count  = (int32_t)g_motors.lastLeftRpm();
    enc.right_motor_count = (int32_t)g_motors.lastRightRpm();
    // Cylinder feedback: position in 0.1 mm (e.g. 1550 = 155.0 mm).
    enc.cyl1_count        = (int32_t)(fb1.pos_mm * 10.0f + 0.5f);
    enc.cyl2_count        = (int32_t)(fb2.pos_mm * 10.0f + 0.5f);
    enc.sample_time_ms    = now;
    g_link.sendEncoder(enc);
  }

  if ((int32_t)(now - g_next_imu_tx_ms) >= 0) {
    g_next_imu_tx_ms = now + IMU_PERIOD_MS;
    Imu imu;
    if (g_imu.read(imu)) {
      g_link.sendImu(imu);
      g_error_flags &= ~ERR_SENSOR;
    } else {
      g_error_flags |= ERR_SENSOR;
    }
  }

  if ((int32_t)(now - g_next_env_tx_ms) >= 0) {
    g_next_env_tx_ms = now + ENV_PERIOD_MS;
    EnvSensor env;
    if (g_env.read(env)) {
      g_link.sendEnv(env);
    }
  }

  if ((int32_t)(now - g_next_ctrl_tx_ms) >= 0) {
    g_next_ctrl_tx_ms = now + CTRL_PERIOD_MS;
    const uint8_t motor_state =
      (g_applied_linear_mps == 0.0f && g_applied_angular_rps == 0.0f) ? 0 : 1;
    g_link.sendControlState(g_active_source, g_comm_state,
                            motor_state, g_applied_arm_mode);
  }

  if ((int32_t)(now - g_next_err_tx_ms) >= 0) {
    g_next_err_tx_ms = now + ERROR_PERIOD_MS;
    g_link.sendError(g_error_flags);
  }

#if SBUS_ENABLE
  if ((int32_t)(now - g_next_sbus_joy_ms) >= 0) {
    g_next_sbus_joy_ms = now + SBUS_JOY_PERIOD_MS;
    SbusChannels sj;
    for (uint8_t i = 0; i < 8; ++i) {
      sj.ch[i] = (int16_t)(g_sbus.channel(i) * 10000.0f);
    }
    g_link.sendSbusJoy(sj);
  }
#endif

#if CAN_BATTERY_ENABLE
  if ((int32_t)(now - g_next_battery_ms) >= 0) {
    g_next_battery_ms = now + BATTERY_PERIOD_MS;
    BatteryStatus bat;
    g_battery.fill(bat);
    g_soc_percent = bat.soc_percent;
    g_link.sendBattery(bat);
  }
#endif

  if ((int32_t)(now - g_next_led_ms) >= 0) {
    g_next_led_ms = now + LED_UPDATE_PERIOD_MS;
    LedState s;
    s.active_src   = g_active_source;
    s.estop        = !g_power_relay;   // RED LED ON when power relay is OFF
    s.motor_active = g_motor_active;
    s.low_battery  = (g_soc_percent < 10);
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
  // NOTE: SBUS_SERIAL.begin() is NOT called here — bfs::SbusRx::Begin()
  // opens the serial port at 100000 8E2 internally.

  while (!LINK_SERIAL && millis() < 2000) {
    // wait briefly for USB-CDC enumeration
  }

  // LEDs first — so we can flash something even if other modules hang.
  g_leds.begin(pins::LED_RED, pins::LED_GREEN,
               pins::LED_BLUE, pins::LED_YELLOW);

  // Hardware e-stop switch (D22). INPUT_PULLUP: LOW = e-stop ON.
  pinMode(pins::MODE_SWITCH, INPUT_PULLUP);

  // Power relay & warning lights.
  pinMode(pins::RELAY_POWER,  OUTPUT);
  pinMode(pins::LIGHT_RED,    OUTPUT);
  pinMode(pins::LIGHT_YELLOW, OUTPUT);
  digitalWrite(pins::RELAY_POWER,  LOW);   // start with power OFF (safe)
  digitalWrite(pins::LIGHT_RED,    HIGH);  // red ON = power OFF
  digitalWrite(pins::LIGHT_YELLOW, LOW);

  g_link.begin(LINK_SERIAL);
#if DEBUG_ENABLE
  g_link.setDebugSink(&DEBUG_SERIAL);
#endif
#if SBUS_ENABLE
  // bfs::SbusRx handles Serial1 setup (100000 8E2).
  // `inverted = false` for receivers that output non-inverted SBUS.
  g_sbus.begin(SBUS_SERIAL, /*inverted=*/false);
#endif

  // Hardware
  DiffDriveGeometry geom;
  g_motors.begin(left_pins(), right_pins(), geom);
  g_cyls.begin(cyl_pins(0), cyl_pins(1), cyl_pins(2));

  if (!g_imu.begin(0x28, &Wire)) {
    g_error_flags |= ERR_SENSOR;
    DBG_PRINTLN(F("IMU: BNO055 not found (sensor error)"));
  } else {
    DBG_PRINTLN(F("IMU: BNO055 ready"));
  }

  if (!g_env.begin(0x76, &Wire)) {
    DBG_PRINTLN(F("ENV: BME280 not found at 0x76"));
  } else {
    DBG_PRINTLN(F("ENV: BME280 ready"));
  }

#if CAN_BATTERY_ENABLE
  if (!g_battery.begin(pins::CAN_SPI_CS, pins::CAN_INT)) {
    DBG_PRINTLN(F("CAN: MCP2515 init FAIL"));
  } else {
    DBG_PRINTLN(F("CAN: battery BMS ready"));
  }
#endif

  const uint32_t now = millis();
  g_next_hb_tx_ms      = now;
  g_next_encoder_tx_ms = now;
  g_next_imu_tx_ms     = now;
  g_next_env_tx_ms     = now;
  g_next_ctrl_tx_ms    = now;
  g_next_err_tx_ms     = now;
  g_next_sbus_joy_ms   = now;
  g_next_battery_ms    = now;
  g_next_led_ms        = now;

  DBG_PRINTLN(F("=== blv_arduino_mega boot ==="));
  DBG_PRINT(F("LINK baud=")); DBG_PRINTLN(LINK_BAUD);
#if SBUS_ENABLE
  DBG_PRINT(F("SBUS on Serial1 RX=D19, inverted=false, baud="));
  DBG_PRINTLN(SBUS_BAUD);
#endif
}

// =====================================================================
// Debug: periodic SBUS status print (every 2 seconds on Serial2)
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
  DEBUG_SERIAL.println(']');
}
#endif

// =====================================================================
// Debug: periodic battery CAN dump (every 3 seconds on DEBUG_SERIAL)
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

void loop()
{
  g_link.update();
#if SBUS_ENABLE
  g_sbus.update();
#endif
#if CAN_BATTERY_ENABLE
  g_battery.update();
#endif

  consume_link_rx();
  update_watchdogs();
  arbitrate_and_apply();
  drive_hardware();

  const uint32_t now_ms = millis();
  periodic_senders(now_ms);

#if SBUS_ENABLE && DEBUG_ENABLE
  debug_sbus_status(now_ms);
#endif
#if CAN_BATTERY_ENABLE && DEBUG_ENABLE
  debug_battery_status(now_ms);
#endif
}
