// LAST MODIFIED: 2026-11-05 AEST — FRAME_REJECT switch (default 0 = count only). Set to 1 to drop
//   frames from AR3 with a bad/missing checksum and unframed text. Tested ON at TestSys 5 Nov.
// LAST MODIFIED: 2026-11-04 AEST — HB from AR3 read from the value after "HB:" (works framed);
//   a queued new command cancels the previous command's retries. For AR3's framed ACK/HB:0.
// LAST MODIFIED: 2026-11-04 AEST — Backup link receive. The R4 SoftwareSerial receiver sometimes
//   holds back the last 1-2 bytes of a burst until the next one starts; AR1 now accepts a backup
//   frame whose checksum verifies after 0.5 s quiet, else waits up to 2.5 s for the held bytes.
//   Frames wait up to 0.5 s for '>' (was a 25 ms gap). TX window timed from the active path only;
//   on backup (full duplex) AR1 sends anyway if it has not sent for 4 s.
// LAST MODIFIED: 2026-11-02 AEST — Restart safety. After any restart AR1 restores the saved mode
//   (it forced Manual, which took AR3 out of Auto for ~5 min). EEPROM mode 2 = Auto because MQTT
//   was lost (MQTT back -> Manual, as before). Also fixed: MQTT loss while already in Auto no
//   longer switches to Manual when MQTT returns. Connection check: no MQTT for 15 min -> restart
//   (not in first 30 min, max 4 in a row). HA switch home/selfrestart/set (state .../state) goes to
//   AR3 in STAT as SR:0/1. Reset causes published retained on home/ar1|ar2|ar3/last_reset.
// LAST MODIFIED: 2026-11-02 AEST — Shed cooling (P2) fixes. AR1 no longer runs its own P2
//   cycle in Auto (AR3 owns shed cooling; AR1's cycle made HA show "cooling ON" for ~2 s every
//   2 min). The "Disable Shed Cooling" switch never energises the coil any more (boot and the
//   set handler had it inverted). home/relay/peripheral2 always reports the coil; telemetry no
//   longer overwrites home/peripheral2/state (the Disable switch) with the relay state.
// LAST MODIFIED: 2026-11-01 AEST — Gel needs reduced flow (the constriction makes the pressure
//   difference that drives the Dosatron DN009; P3 OFF = reduced, ON = high flow). Turning gel
//   (P4) ON also switches P3 to reduced; while gel is on, P3 ON (high flow) is refused and held
//   OFF with home/peripheral3/info "GEL ON - reduced flow required". Turning gel off leaves the
//   flow reduced. At boot, if both were stored ON, P3 is restored OFF.
// LAST MODIFIED: 2026-10-31 AEST — Watchdog now actually runs (5000 ms; 8192 exceeded the RA4M1
//   maximum and was rejected). Long waits bounded so it can't fire by mistake: every WiFi-module
//   command capped at 1 s, WiFi join 3 s, MQTT connect 2 s + 2 s, watchdog refreshed inside the
//   WiFi join/DHCP waits and before the direct-pump-start wait.
// LAST MODIFIED: 2026-11-01 AEST — Publishes the tank level source from AR3's WLMETHOD on
//   home/sensor/wl_source ("Gauge" or the flow-estimate warning), retained, on change.
// LAST MODIFIED: 2026-11-01 AEST — Recirculation valve polarity corrected (confirmed by Jim and
//   the Aug 2025 AR3): coil OFF = PASS-THROUGH to sprinklers, coil ON = recirculate to tank. The
//   manual valve cycle had it inverted (recirculating in TimeON, spraying in TimeOFF; switching
//   the Intermittent Sprinkler switch on with 0/0 times started recirculation). home/relay/peripheral1
//   now reports the valve coil (published on every change), not the Intermittent switch state.
// LAST MODIFIED: 2026-11-01 AEST — AUTO → MANUAL now starts with pump, P1, P3, P4 OFF (P2 kept)
//   instead of copying AR3's last relay states, which left a hidden "pump ON" that showed in HA
//   on unlock while AR3 stayed off.
// LAST MODIFIED: 2026-11-01 AEST — Auto status text covers AR3's new modes (cool-down >80 C
//   continuous; temperature-sensor fallback with its stage), and AR3's TFAIL bits are published
//   retained on home/sensor/temp_fail as text ("OK" or e.g. "EXT, EXT2 failed"), in any mode.
// LAST MODIFIED: 2026-10-31 AEST — Publishes a one-line Auto status (home/auto/status): ON/OFF,
//   button test or not, and the current autonomous phase, from AR3's FSMODE/PHASE/MODE/TST fields.
// LAST MODIFIED: 2026-10-30 AEST — Publishes AR3's autonomous sprinkler cycle and shed-cooling
//   state (telemetry SPR:/SHD:) as readable text, retained, on home/auto/sprinkler_cycle and
//   home/auto/shed_cooling (read-only HA display; the manual sliders are untouched).
//   Parsed in processRS485() (sees partial chunks too), not onRS485FrameFromAR3() (whole frames only).
// LAST MODIFIED: 2026-08-22 14:57 AEST — Added diesel fuel level sender support:
//   parse FUEL: from AR3's telemetry frame into RX_FUEL_LEVEL, publish to
//   home/sensor/fuel_level (matches existing home/sensor/* MQTT convention).
// LAST MODIFIED: 2026-07-15 22:18 AEST — Stripped out DIAG_DISABLE_WIFI and all
//   three of its guarded call sites (setup_wifi(), reconnect(), the periodic
//   SSID-check block) — that diagnostic (WiFi disabled entirely, to test
//   whether WiFi/SPI activity disrupted primary RS485) did its job and ruled
//   out AR1's WiFi as a contributor. AR1 is back to normal WiFi/MQTT behaviour.
//   Also fixed a stale comment near the boot-time TestSys detection code that
//   still said "D13" (leftover from before the pin moved to A3 — see the
//   correct comment above JF_TESTSYS_PIN's #define). RS485 flapping on
//   TestSys is RESOLVED — root cause was on AR3 (tickLCD() hanging on a
//   misdetected LCD), not AR1; see AR3.ino header and
//   project_dn50_flow_sensor.md memory for the full writeup.
//AR1
// Improved Arduino Sketch
// Date: 14 Oct 2025
// T43 (2026-07-12): Disable Shed Cooling (P2) now respects LOCK, inverted vs P1/P3/P4.
//   - home/peripheral2/set: while sysLocked(), OFF commands (re-enabling cooling) are
//     blocked and the switch is held ON (cooling forced off) until Enable Controls
//     (switch.pump_enable → lockState=false) is restored. ON commands still pass at
//     any time. Previously P2 ignored LOCK entirely ("lock override for shed cooling").
//   - Unlocked behaviour unchanged: switch toggles freely both ways.
// T38 (2026-05-12): Reduce AR3 HB_STABLE_MS/HB_FRESH_FOR_EXIT_MS for faster AUTO exit.
//   - HB_FRESH_FOR_EXIT_MS: 35 s → 10 s (spans 2× STAT_HB_PERIOD_MS with margin)
//   - HB_STABLE_MS: 60 s → 15 s (EXIT AUTO within ~20 s of enabling REMOTE)
//   - ar3InAutoMode flag: parse FSMODE from AR3 telemetry; suppress pumpState update
//     from CMD AUTO guard ACKs while AR3 is autonomous (prevents phantom pump state in HA).
//   - Lock UX: force HA lock switch ON while AR3 is in AUTO mode (on FSMODE transition and
//     every 5 s via periodic heartbeat block) so operator sees system is locked until AUTO exits.
// T37 (2026-05-12): Add periodic STAT heartbeat in loop() so AR3 receives HB:1 every 5 s.
//   - Root cause of overnight AUTO entry: no STAT frame sent for >5 min during idle → AR3 enters AUTO.
//   - Without periodic HB:1, AR3's HB_STABLE_MS (60 s) exit window kept resetting.
// T42 (2026-05-31): Always poll backup serial so path watchdog can detect backup is alive on boot.
//   - Previously swPathLastTelMs was only updated when ar3ActivePath==PATH_BACKUP, so on boot
//     (PATH_PRIMARY) the watchdog condition (primaryPathLastTelMs==0 && swPathLastTelMs>0) never
//     fired and AR1 stayed on primary even when RS485 was dead. Now backup is always polled;
//     duplicate frame processing guarded by ar3ActivePath check inside processRS485().
// T41 (2026-05-31): Fix watchdog init (8000→8192ms, valid RA4M1 value); add MQTT reboot command.
//   - home/ar1/reboot: payload "1" triggers NVIC_SystemReset() for remote Arduino restart
//   - wdtInterval corrected to 8192ms (nearest valid RA4M1 WDT timeout above 8s)
// T25 (2026-05-08): Add SoftwareSerial backup path for AR1↔AR3 comms.
//   - Backup path: PIN_SW_RX=A5, PIN_SW_TX=A4
//   - Cable: AR1.TX(A4) → AR3.RX(A2), AR3.TX(A1) → AR1.RX(A5)
//   - RS485Send() routes to swSerial when ar3ActivePath == PATH_BACKUP
//   - SwSerialReceive() mirrors RS485Receive() on swSerial
//   - tickPathWatchdog() detects primary silence >10s, switches to backup, publishes MQTT alert
//   - Primary recovery: auto-switch back when Serial1 telemetry resumes
//   - Fixed checksum mismatch: verifyChecksum/addChecksum now use XOR/hex (matches AR3)
//   - Fixed misleading "Pump ON/OFF command sent to AR3" log when ar3Ready gate blocks it
//   - Added early ar3Ready detection in processRS485()
//   - HA topics: home/ar3/comms_path ("primary"/"backup"), home/ar3/comms_alert ("0"/"1")

// NOTE: This file includes the surgical changes to adopt AR3 state on Heartbeat→ON,
// and to accept HB:0/1 from AR3 over RS-485. MQTT device states stay numeric.

#include <WiFiS3.h>
#include <PubSubClient.h>
#include <WDT.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// --- JF: Heartbeat retained-command boot guard ---
static unsigned long mqttBootMs = 0;                       // set in setup() after MQTT connect/subscribe
static const unsigned long HB_LEGACY_IGNORE_MS = 12000UL;  // ignore legacy ON/OFF for first 12 s


bool manualPumpOverride = false;  // When true, manual pump control overrides peripheral cycling

// Select transceiver: 1 = Gravity DFR0845 (no DE/RE pin), 0 = MAX485-style (needs DE/RE on pin 12)
#ifndef USE_GRAVITY_RS485
#define USE_GRAVITY_RS485 1
#endif

#ifndef RS485_ENABLE
#define RS485_ENABLE 1
#endif
#ifndef RS485_TX_ONLY
#define RS485_TX_ONLY 0  // AR1: TX + RX
#endif
#ifndef RS485_RX_ONLY
#define RS485_RX_ONLY 0  // was 1 → blocked RS485Send(); must be 0
#endif

// TestSys detection — A3 strapped to GND on TestSys hardware, floating on Production
// D13 is unsuitable (onboard LED circuit loads it, giving ~1.9V not 3.3V on INPUT_PULLUP)
#define JF_TESTSYS_PIN A3
static bool JF_isTestSys = false;

// Pins
const int heartbeatPin = 8;
// oilPressureSwitchPin removed — oil pressure switch is on AR2 (D10), not AR1.
// AR1 receives oil pressure state via RX_OIL_OK from AR2 telemetry.
const int waterPressureSwitchPin = A2;
#define TANK_SENSOR_PIN A0
#define MAX_VOLTAGE 5.0
#define MAX_LEVEL 3.2
const int relayPumpOn = 2;
const int relayPumpOff = 3;
const int relayPeripheral1 = 4;
const int relayPeripheral2 = 5;
const int relayPeripheral3 = 6;
const int relayPeripheral4 = 7;
const int directPumpPin = 11;
const int tempSensor1Pin = 9;
const int tempSensor2Pin = 10;
// emberDetectorPin removed — ember detector will be read on AR2, not AR1.
// AR1 receives ember state via RX_EMBER from AR2 telemetry.

// Backup SoftwareSerial path
// Cable: AR1.TX(A4) → AR3.RX(A2), AR3.TX(A1) → AR1.RX(A5)
#define PIN_SW_RX  A5
#define PIN_SW_TX  A4
SoftwareSerial swSerial(PIN_SW_RX, PIN_SW_TX);

// EEPROM addrs
const int pumpStateAddr = 0;
const int peripheral1StateAddr = 1;
const int peripheral2StateAddr = 2;
const int peripheral3StateAddr = 3;
const int peripheral4StateAddr = 4;
const int peripheral1TimeOnAddr = 5;
const int peripheral1TimeOffAddr = 6;
const int peripheral2TimeOnAddr = 7;
const int peripheral2TimeOffAddr = 8;
const int autonomousStateAddr = 9;  // 0=Autonomous,1=Manual,2=Autonomous because MQTT was lost

// Dev / Prod boot policy (stored in EEPROM)
// 0 = PRODUCTION (restore EEPROM states)
// 1 = DEVELOPMENT (clear EEPROM states on every reboot)
const int devModeAddr = 10;
bool devMode = false;

// ---------------- Restart safety (2026-11-02) ----------------
// AR1 now comes back after a restart in the mode it was in (it used to force Manual, which told
// AR3 to leave Auto). Connection check: no MQTT for 15 min -> restart (also resets the WiFi
// chip). Never in the first 30 min after boot, at most 4 in a row (then it waits for MQTT to come
// back). HA switch "Connection self-restarts" (home/selfrestart/set) turns the check off on all
// three boards (passed to AR3 in STAT as SR:0/1, and by AR3 to AR2). Why each board last
// restarted is published (retained) on home/ar1/last_reset, home/ar3/last_reset, home/ar2/last_reset.
const int restartPendingAddr = 20;    // reason for a restart AR1 asked for itself (4 or 5)
const int selfRestartCountAddr = 21;  // connection restarts in a row
const int selfRestartEnAddr = 22;     // 0 = connection restarts off (0xFF/other = on)
unsigned long SELF_RESTART_NO_MQTT_MS = 15UL * 60000UL;     // TestSys: 3 min (set in setup)
unsigned long SELF_RESTART_MIN_UPTIME_MS = 30UL * 60000UL;  // TestSys: 2 min
const uint8_t SELF_RESTART_MAX_IN_ROW = 4;
uint8_t ar1ResetCause = 3;
bool selfRestartEnabled = true;
uint8_t selfRestartCount = 0;
int ar3ResetCause = -1, ar2ResetCause = -1;  // from AR3 telemetry (RST, RST2)
bool resetInfoPublished = false;
bool testNoMqtt = false;  // USB test command NOMQTT: stay off MQTT until the next restart

// 0 power-on, 1 watchdog, 2 software, 3 reset pin / upload, 4 connection check, 5 HA command
static const char* resetCauseText(int c) {
  switch (c) {
    case 0: return "Power on";
    case 1: return "Watchdog";
    case 2: return "Software / upload";
    case 3: return "Reset button / upload";
    case 4: return "Connection check";
    case 5: return "HA command";
    default: return "unknown";
  }
}

// Read (and clear) the RA4M1 reset flags once at boot.
static uint8_t readResetCause() {
  const uint8_t r0 = R_SYSTEM->RSTSR0, r1 = R_SYSTEM->RSTSR1;
  uint8_t c;
  if (r0 & 0x0F) c = 0;        // power-on or low-voltage reset
  else if (r1 & 0x03) c = 1;   // independent / normal watchdog
  else if (r1 & 0x04) c = 2;   // software reset
  else c = 3;                  // reset pin (upload, button)
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
  if (c == 2) {
    const uint8_t p = EEPROM.read(restartPendingAddr);
    if (p == 4 || p == 5) c = p;
  }
  EEPROM.update(restartPendingAddr, 0xFF);
  return c;
}

static void selfRestart(uint8_t reason) {
  EEPROM.update(restartPendingAddr, reason);
  if (reason == 4) EEPROM.update(selfRestartCountAddr, (uint8_t)min(255, selfRestartCount + 1));
  Serial.print(F("[RESTART] reason "));
  Serial.println(resetCauseText(reason));
  delay(200);
  NVIC_SystemReset();
}

// Globals
bool pumpState = false;
bool peripheral1State = false;
bool peripheral2State = false;
bool peripheral3State = false;
bool peripheral4State = false;
bool lockState = true;
unsigned long listenStart = 0;
static bool lastManualPumpOverride = false;

unsigned long mqttFailStartTime = 0;
bool mqttFailed = false;
const unsigned long MQTT_FAIL_TIMEOUT = 300000;

bool flameDetected = false;
unsigned long flameDetectionTime = 0;
bool flameAcknowledged = false;
unsigned long lastFlameAlertTime = 0;
const unsigned long flameAlertInterval = 300000;

bool RS485Debug = false;
unsigned long lastDebugTime = 0;
const unsigned long debugInterval = 1000;
bool RS485_Status = false;

// AR3 readiness: becomes true once we see at least one telemetry frame.
static bool ar3Ready = false;

// AR3 mode: true while AR3 reports FSMODE:1 (AUTONOMOUS). Used to suppress
// pumpState updates from CMD AUTO guard ACKs (which carry AR3's AUTO pump state,
// not the operator's intended state).
static bool ar3InAutoMode = false;

// Debug: rate-limit "[AR3 WAIT]" logging while we are gating commands.
static unsigned long ar3WaitLastPrintMs = 0;

// Dual-path comms state
enum AR3CommPath : uint8_t { PATH_PRIMARY = 0, PATH_BACKUP = 1 };
static AR3CommPath ar3ActivePath = PATH_PRIMARY;
static unsigned long primaryPathLastTelMs = 0;        // millis() when Serial1 last delivered telemetry
static unsigned long swPathLastTelMs = 0;             // millis() when swSerial last delivered telemetry
static unsigned long ar3FrameEndMs = 0;               // millis() of last AR3 telemetry; drives TX window
static const unsigned long TX_GUARD_MS = 5UL;         // guard delay after AR3 frame before AR1 may TX
static const unsigned long TX_WINDOW_MAX_MS = 500UL;  // ...and AR1 must TX within 0.5 s of it (4 Nov 2026)
// TX window: AR1 may transmit only between 5 ms and 0.5 s after the end of AR3's latest
// telemetry frame. AR3 sends every 2 s and a frame takes ~0.35 s, so this keeps AR1 clear of
// AR3's next frame. Before 4 Nov the window had no upper limit: a STAT due on AR1's own 5 s
// timer went out immediately even if AR3 had started its next frame, and the two collided
// every 10 s whenever the timers drifted into step. If AR1 misses the window it waits for the next.
static unsigned long ar1SwLastTxMs = 0;  // last AR1 transmit on the backup path
static inline bool txWindowOpen() {
  // Backup is two separate wires (full duplex), so collisions can't happen there. If no
  // window has opened for 4 s (e.g. AR3's frames all arriving late), send anyway rather
  // than starve AR3 of STAT/heartbeat.
  if (ar3ActivePath == PATH_BACKUP && (millis() - ar1SwLastTxMs) > 4000UL) return true;
  if (ar3FrameEndMs == 0) return false;
  const unsigned long age = millis() - ar3FrameEndMs;
  return age >= TX_GUARD_MS && age <= TX_WINDOW_MAX_MS;
}
static const unsigned long PRIMARY_FAIL_MS = 15000UL; // switch to backup after 15s silence on primary
static const unsigned long PRIMARY_STEADY_MS = 60000UL; // on backup: RS485 must be steady this long before switching back (stops flapping)
static unsigned long primarySteadySinceMs = 0;          // when RS485 telemetry resumed without a gap (0 = not steady)
static unsigned long lastPathPublishMs = 0;
static const unsigned long PATH_PUBLISH_INTERVAL_MS = 30000UL;
// Test hook: when true, tickPathWatchdog() holds the current path and won't auto-switch.
// Release with MQTT topic home/ar3/comms_path/force payload "auto".
static bool ar3PathForced = false;

volatile unsigned long lastRS485Seen = 0;
volatile float RX_EXT_TEMP = NAN;
volatile float RX_INT_TEMP = NAN;
volatile float RX_EXT2_TEMP = NAN;
volatile float RX_FLOW_LPM = NAN;
volatile float RX_WATER_LVL = NAN;
volatile int RX_EMBER = 0;
volatile int RX_PHASE = -1;  // 0=WARMUP, 1=COOLDOWN
volatile int RX_MODE = -1;   // numeric mode code from AR3

// AR3 live-state cache for takeover
volatile bool ar3HasLiveState = false;
volatile unsigned long ar3StateMs = 0;
volatile float ar3WL_m = 0.0f;
volatile bool ar3Pump = false;
volatile bool ar3P1 = false, ar3P2 = false, ar3P3 = false, ar3P4 = false;

// Pump pressure as reported by AR3 (originating from AR2)
volatile bool RX_OIL_OK = false;        // 1 when oil pressure detected
volatile float RX_WATER_PRESS_V = NAN;  // raw voltage at water-pressure node
volatile float RX_FUEL_LEVEL = NAN;     // 0-100%, from AR3 telemetry

// --- JF: Edge+ACK command state for AR1→AR3 relay control ---
// We now send pump + P1..P4 in one CMD frame so that every relay command
// is positively ACKed by AR3 (single in-flight command at a time, by SEQ).
static int8_t JF_desiredPump = -1;             // -1 unknown, 0 OFF, 1 ON
static int8_t JF_lastSentPump = -1;            // last CMD sent
static int8_t JF_lastAckPump = -1;             // last ACK'd pump state from AR3
static uint8_t JF_cmdSeq = 0;                  // 0..15 sequence number
static unsigned long JF_cmdFreezeUntilMs = 0;  // block opposite CMD for a few seconds
static const unsigned long JF_CMD_FREEZE_MS = 400UL;
static const unsigned long STAT_HB_PERIOD_MS = 5000UL; // periodic STAT heartbeat when REMOTE active

// --- Pump stop retry ---
// If 45 s after a stop command both oil pressure and water pressure still indicate the
// pump is running, a single further stop command is sent and the operator is alerted.
static const unsigned long PUMP_STOP_RETRY_MS = 45000UL;
static bool     stopRetryPending  = false;
static unsigned long stopRetryCheckMs = 0;

// Last-sent relay bits (for P1..P4)
static int8_t JF_lastSentP1 = -1;
static int8_t JF_lastSentP2 = -1;
static int8_t JF_lastSentP3 = -1;
static int8_t JF_lastSentP4 = -1;

// --- FS fast retry state (ACK-gated) ---
static bool FS_waitAck = false;
static uint8_t FS_waitAckSeq = 0;
static uint8_t FS_waitAckPump = 0;  // 0/1
static uint8_t FS_waitAckP1 = 0;
static uint8_t FS_waitAckP2 = 0;
static uint8_t FS_waitAckP3 = 0;
static uint8_t FS_waitAckP4 = 0;
static unsigned long FS_ackT0 = 0;
static uint8_t FS_ackRetries = 0;
static const unsigned long FS_ACK_TIMEOUT_MS = 300;  // retry cadence
static const uint8_t FS_ACK_MAX_RETRY = 3;           // total sends = 1 + 3 = 4 tries

// --- TX window pending flags (Phase 4-5) ---
// Actual RS485 sends are deferred into the TX window that opens 5ms after each AR3 telemetry frame.
static bool    pendingTxStat = false;
static bool    pendingTxCmd  = false;
static bool    pendingTxCfg  = false;
static int     pendingCmdPump = 0;
static int     pendingCmdP1 = 0, pendingCmdP2 = 0, pendingCmdP3 = 0, pendingCmdP4 = 0;

bool MQTT_SUMMARY_DEBUG = false;
bool RS485_AnnouncedAtStartup = false;
bool RS485_LastState = false;
unsigned long RS485_BootMs = 0;
const unsigned long RS485_STARTUP_ANNOUNCE_DELAY = 10000;

unsigned long timeOn = 0, timeOff = 1800000, lastToggleTime = 0;
bool relay1On = false;  // P1 coil energised = RECIRCULATE to tank; off = PASS-THROUGH to sprinklers

// Manual sprinkler driver mode: RECIRC (default) vs PUMP-cycled
static bool sprinklerUsesPump = false;       // HA request: true = PUMP, false = RECIRC
static bool sprinklerPumpEffective = false;  // last effective mode after safety checks
static bool sprinklerPumpOnPhase = false;    // PUMP-mode phase: true = pump ON
static unsigned long sprinklerPumpPhaseStartMs = 0;
static bool sprinklerOwnsPump = false;  // true if sprinkler logic was the one that turned the pump ON

unsigned long timeOn2 = 0, timeOff2 = 1800000, lastToggleTime2 = 0;
bool relay2On = false;
unsigned long lastRelay3Update = 0, lastRelay4Update = 0;
const unsigned long relay3Interval = 1000, relay4Interval = 1000;


const float batteryVoltageThreshold = 12.5;
float batteryVoltage = 0.0;

// Safety thresholds for manual PUMP-cycled sprinkler driver
const float MAN_BAT_MIN_V = 12.25f;            // V
const int16_t MAN_PUMP_MODE_MIN_OFF_MIN = 10;  // Tmin minutes: PUMP-cycling allowed only if TimeON>Tmin AND TimeOFF>Tmin (set -1 to disable)

const float AutoTriggerTemp1 = 60.0, AutoTriggerTemp2 = 60.0;
bool TempAutoTrigger = false;

// Helper: is PUMP-mode sprinkler allowed given current config + battery?
// True only when HA requests PUMP-cycling AND safety gates allow it.
static bool isSprinklerPumpModeAllowed() {
  // True only when HA requests PUMP-cycling AND safety gates allow it.
  if (!sprinklerUsesPump) return false;

  // Battery gate (NaN fails this >= check too)
  if (!(batteryVoltage >= MAN_BAT_MIN_V)) {
    Serial.print("Sprinkler pump-mode DENY: BAT=");
    Serial.print(batteryVoltage, 2);
    Serial.print("V < MIN=");
    Serial.print(MAN_BAT_MIN_V, 2);
    Serial.println("V");
    return false;
  }

  // 0/0 is safe idle — no cycling, sprinkler must not drive the pump.
  // Return false (VALVE mode) so the 0/0 branch in loop() can release any pump claim without
  // a spurious VALVE→PUMP mode flip that clears sprinklerOwnsPump before the branch runs.
  if (timeOn == 0 && timeOff == 0) {
    return false;
  }

  // Otherwise PUMP-cycling requires BOTH phases present (>0) and BOTH strictly > Tmin.
  if (timeOn == 0 || timeOff == 0) {
    Serial.print("Sprinkler pump-mode DENY: needs TimeON>0 and TimeOFF>0. TimeON=");
    Serial.print(timeOn / 60000UL);
    Serial.print("min TimeOFF=");
    Serial.print(timeOff / 60000UL);
    Serial.println("min");
    return false;
  }

  unsigned long onMin = timeOn / 60000UL;
  unsigned long offMin = timeOff / 60000UL;

  if (MAN_PUMP_MODE_MIN_OFF_MIN >= 0) {
    unsigned long tmin = (unsigned long)MAN_PUMP_MODE_MIN_OFF_MIN;
    if (onMin <= tmin || offMin <= tmin) {
      Serial.print("Sprinkler pump-mode DENY: TimeON=");
      Serial.print(onMin);
      Serial.print("min TimeOFF=");
      Serial.print(offMin);
      Serial.print("min <= Tmin=");
      Serial.println(MAN_PUMP_MODE_MIN_OFF_MIN);
      return false;
    }
  }

  //     Serial.print(batteryVoltage, 2);
  //   Serial.print("V TimeON=");
  //   Serial.print(onMin);
  //   Serial.print("min TimeOFF=");
  //   Serial.print(offMin);
  //   Serial.print("min Tmin=");
  //   Serial.println(MAN_PUMP_MODE_MIN_OFF_MIN);
  return true;
}

long heartbeatValue = 0;
bool publishHeartbeat = true;
unsigned long lastHeartbeatUpdate = 0;
const unsigned long heartbeatInterval = 5000;
const unsigned long HEARTBEAT_INTERVAL_NORMAL = 1000;
const unsigned long HEARTBEAT_INTERVAL_FAST = 500;
bool confirmationReceived = false;
bool heartbeatEnabled = true;
bool heartbeatActive = true;
bool hbStoppedByMqttLoss = false;  // Auto only because MQTT was lost (EEPROM mode 2): MQTT back -> Manual

const unsigned long sensorUpdateInterval = 15000;
unsigned long lastSensorUpdate = 0;
const long wdtInterval = 5000;  // runs as 5592 ms (chip rounds up; its max). 8192 was rejected, so AR1 had no watchdog (fixed 2026-10-31)

// WiFi + MQTT site profiles
struct NetProfile {
  const char* ssid;
  const char* pass;
  const char* mqttHost;
  const char* haUrl;
};
#include "config.h"
static NetProfile NETS[] = {
  { NET1_SSID, NET1_PASS, NET1_MQTT, NET1_LABEL },
  { NET2_SSID, NET2_PASS, NET2_MQTT, NET2_LABEL },
};
static const size_t NETS_LEN = sizeof(NETS) / sizeof(NETS[0]);
static const NetProfile* ACTIVE = nullptr;
static char ACTIVE_SSID[33] = { 0 };
static char ACTIVE_MQTT[32] = { 0 };
const char* mqtt_user = MQTT_USER;

// TestSys mode always talks to the Rowe St broker directly (site-to-site
// Fritzbox/Tailscale routing restored 2026-07-12 makes the Site 2 broker (NET2_MQTT)
// reachable from Fairhaven regardless of which local SSID is active).
static const char* resolveMqttHost(const char* mqttHost) {
  return JF_isTestSys ? NET2_MQTT : mqttHost;
}
const char* mqtt_password = MQTT_PASSWORD;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
static unsigned long lastMqttRetryMs = 0;
static char MQTT_CLIENT_ID[48] = { 0 };
const unsigned long MQTT_RETRY_INTERVAL_MS = 15000;

// WiFi retry state
static unsigned long lastWifiRetryMs = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 30000;  // 30 s between WiFi re-attempts

// RS-485 helpers/checksums
// CRC-8 (polynomial 0x07) over a message, matching AR3. Replaced the old XOR byte
// checksum on 4 Nov 2026: CRC-8 also catches double-bit errors, short noise bursts
// and swapped bytes, which XOR misses.
static uint8_t crc8(const String &s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c ^= (uint8_t)s[i];
    for (uint8_t b = 0; b < 8; ++b) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// CRC-8/hex checksum — matches AR3 fsAddChecksum() exactly
static bool verifyChecksum(String message) {
  if (message == "PING_AR3" || message == "PONG_AR3" || message == "PING_AR1" || message == "PONG_AR1") {
    return true;
  }

  int sepPipe = message.indexOf('|');
  String bare = (sepPipe < 0) ? message : message.substring(0, sepPipe);
  if (bare == "PING_AR1" || bare == "PING_AR3" || bare == "PING" || bare == "PONG_AR1" || bare == "PONG_AR3" || bare == "PONG") return true;

  int sep = message.lastIndexOf('|');
  if (sep < 0) return false;
  String payload = message.substring(0, sep);
  String chkStr = message.substring(sep + 1);
  chkStr.trim();
  if (chkStr.length() != 2 || !isHexadecimalDigit(chkStr[0]) || !isHexadecimalDigit(chkStr[1])) return false;
  uint8_t calc = crc8(payload, payload.length());
  uint8_t rx = (uint8_t)strtoul(chkStr.c_str(), nullptr, 16);
  return (calc == rx);
}
static String addChecksum(String message) {
  char buf[3];
  snprintf(buf, sizeof(buf), "%02X", crc8(message, message.length()));
  return message + "|" + buf;
}

// Frame check, COUNT ONLY (4 Nov 2026): every frame from AR3 is checked and counted,
// but frames are still accepted exactly as before. Counts go to HA (home/ar3/frame_check)
// every 60 s and to the serial log every 10 min. Rejection comes later, once the
// counts show every message type passing.
// FRAME_REJECT 1 = drop frames from AR3 with a bad or missing checksum (and unframed text).
// 0 = count only. Committed as 0 until tested on TestSys and loaded at Fairhaven on site.
#ifndef FRAME_REJECT
#define FRAME_REJECT 0
#endif
#define FRAME_ACTION_TXT (FRAME_REJECT ? "), rejected: " : "), still accepted: ")
static uint32_t fcOk = 0, fcBad = 0, fcRaw = 0;       // frames AR1 received from AR3
static long ar3FcOk = -1, ar3FcBad = -1, ar3FcNoChk = -1;  // AR3's counts of frames from AR1
static unsigned long fcLastBadLogMs = 0, fcLastRawLogMs = 0;
static void frameCount(bool ok, const String &msg, const __FlashStringHelper *path) {
  if (ok) { fcOk++; return; }
  fcBad++;
  if (fcLastBadLogMs == 0 || millis() - fcLastBadLogMs >= 10000UL) {
    fcLastBadLogMs = millis();
    Serial.print(F("[FRAME] bad checksum ("));
    Serial.print(path);
    Serial.print(FRAME_ACTION_TXT);
    Serial.println(msg.substring(0, 80));
  }
}
static void frameRawCount(const String &chunk, const __FlashStringHelper *path) {
  fcRaw++;
  if (fcLastRawLogMs == 0 || millis() - fcLastRawLogMs >= 60000UL) {
    fcLastRawLogMs = millis();
    Serial.print(F("[FRAME] unframed text ("));
    Serial.print(path);
    Serial.print(FRAME_ACTION_TXT);
    Serial.println(chunk.substring(0, 60));
  }
}
static unsigned long swRxOnTime = 0, swRxTimeoutOk = 0, swRxLate = 0, swRxAbandoned = 0;
static void tickFrameStats() {
  static unsigned long lastPubMs = 0, lastLogMs = 0;
  const unsigned long now = millis();
  if (now - lastPubMs >= 60000UL) {
    lastPubMs = now;
    if (mqttClient.connected()) {
      char buf[160];
      snprintf(buf, sizeof(buf),
               "{\"ok\":%lu,\"bad\":%lu,\"raw\":%lu,\"ar3_ok\":%ld,\"ar3_bad\":%ld,\"ar3_nochk\":%ld}",
               (unsigned long)fcOk, (unsigned long)fcBad, (unsigned long)fcRaw, ar3FcOk, ar3FcBad, ar3FcNoChk);
      mqttClient.publish("home/ar3/frame_check", buf);
    }
  }
  if (now - lastLogMs >= 600000UL) {
    lastLogMs = now;
    Serial.print(F("[FRAME] since boot: AR1 got ok="));
    Serial.print(fcOk);
    Serial.print(F(" bad="));
    Serial.print(fcBad);
    Serial.print(F(" unframed="));
    Serial.print(fcRaw);
    Serial.print(F(" | AR3 got ok="));
    Serial.print(ar3FcOk);
    Serial.print(F(" bad="));
    Serial.print(ar3FcBad);
    Serial.print(F(" nochk="));
    Serial.println(ar3FcNoChk);
    if (swRxOnTime + swRxTimeoutOk + swRxLate + swRxAbandoned > 0) {
      Serial.print(F("[FRAME] backup frames since boot: on-time=")); Serial.print(swRxOnTime);
      Serial.print(F(" checksum-on-timeout=")); Serial.print(swRxTimeoutOk);
      Serial.print(F(" late=")); Serial.print(swRxLate);
      Serial.print(F(" abandoned=")); Serial.println(swRxAbandoned);
    }
  }
}

// Send on the backup path and note when (used by txWindowOpen's backup fallback).
static inline void swTx(const String &f) {
  swSerial.print(f);
  ar1SwLastTxMs = millis();
}

// --- AR1 → AR3: Heartbeat control — sent on active path only ---
static inline void rs485SendHB(bool on) {
  String payload = String("HB:") + (on ? "1" : "0");
  String framedData = "<" + addChecksum(payload) + ">";
  if (ar3ActivePath == PATH_BACKUP) {
    swTx(framedData);
  } else {
    Serial1.print(framedData);
    Serial1.flush();
  }
}

// --- RS485 frame dispatcher from AR3 ---
static inline void onRS485FrameFromAR3(const String& msg) {
  if (msg == "PING" || msg == "PING_AR3" || msg.startsWith("PING_")) {
    RS485Send("PONG_AR1");
    return;
  }

  // 0) ACK from AR3: "ACK:SEQ:<n>|PUMP:<0/1>"
  if (msg.startsWith("ACK:")) {
    int iSeq = msg.indexOf("SEQ:");
    int iPump = msg.indexOf("PUMP:");
    int ackPump = -1;
    if (iPump >= 0) {
      int bar = msg.indexOf('|', iPump);
      String v = (bar < 0) ? msg.substring(iPump + 5) : msg.substring(iPump + 5, bar);
      v.trim();
      ackPump = v.toInt();
    }
    // --- NEW: extract ACK SEQ ---
    int ackSeq = -1;
    if (iSeq >= 0) {
      int j = iSeq + 4, k = j;
      while (k < (int)msg.length() && isDigit(msg[k])) k++;
      ackSeq = (msg.substring(j, k).toInt() & 0x0F);
    }

    if (ackPump == 0 || ackPump == 1) {
      JF_lastAckPump = ackPump;
      // In AUTO mode AR3's CMD guard ACKs with its autonomous pump state, not the
      // operator's intent. Updating pumpState from those ACKs would corrupt the HA UI
      // and break the freeze logic. Only adopt ACK pump state in REMOTE mode.
      if (!ar3InAutoMode) {
        pumpState = (ackPump != 0);
        publishPumpState();
      }
      if (RS485Debug) {
        Serial.print("[AR3 ACK] pump=");
        Serial.print(ackPump ? "1" : "0");
        if (ar3InAutoMode) Serial.print(" (AUTO: ignored)");
        Serial.println();
      }
      if (JF_lastSentPump == ackPump) JF_cmdFreezeUntilMs = 0;
    }
    // --- NEW: stop retries when SEQ matches ---
    if (FS_waitAck && ackSeq >= 0 && ((uint8_t)ackSeq) == FS_waitAckSeq) {
      FS_waitAck = false;
    }
    return;
  }

  // 1) Heartbeat control (HB:0/1) — keep your existing behavior
  if (msg.startsWith("HB:")) {
    // Read the value after "HB:"; framed messages end in a checksum ("HB:1|3F").
    bool on = (msg.length() > 3 && msg.charAt(3) == '1');
    onHBControlFromAR3(on);  // already present
    return;
  }

  // Mark AR3 as ready once we see a telemetry frame from it.
  // We look for core sensor fields rather than ACK/HB.
  if (!ar3Ready && msg.indexOf("INT_TEMP:") >= 0 && msg.indexOf("WATER_LEVEL:") >= 0) {
    ar3Ready = true;
    Serial.println(F("[AR3 READY] Telemetry detected; enabling RS485 relay commands."));
    maybeSendPumpCmd();  // sync any relay state queued before AR3 was ready
  }

  // 2) Telemetry frames: INT_TEMP|...|WATER_LEVEL:...|...|PUMP:n|P1:n|...|P4:n
  //    Only *publish* relay states to HA while HB is OFF (Autonomous active).
  if (!heartbeatActive) {
    int i;
    auto getInt = [&](const char* key, int def) -> int {
      String k = String(key) + ":";
      i = msg.indexOf(k);
      if (i < 0) return def;
      int j = msg.indexOf('|', i + k.length());
      String v = (j < 0 ? msg.substring(i + k.length()) : msg.substring(i + k.length(), j));
      v.trim();
      return v.toInt();
    };

    const int pumpN = getInt("PUMP", -1);
    const int p1N = getInt("P1", -1);
    const int p2N = getInt("P2", -1);
    const int p3N = getInt("P3", -1);
    const int p4N = getInt("P4", -1);

    if (pumpN >= 0) mqttClient.publish("home/relay/pump", pumpN ? "1" : "0", true);
    if (p1N >= 0) {
      mqttClient.publish("home/peripheral1/state", p1N ? "1" : "0", true);
      mqttClient.publish("home/relay/peripheral1", p1N ? "1" : "0", true);
    }
    if (p2N >= 0) {
      // Relay only: home/peripheral2/state is the "Disable Shed Cooling" switch, not the relay.
      mqttClient.publish("home/relay/peripheral2", p2N ? "1" : "0", true);
    }
    if (p3N >= 0) {
      mqttClient.publish("home/peripheral3/state", p3N ? "1" : "0", true);
      mqttClient.publish("home/relay/peripheral3", p3N ? "1" : "0", true);
    }
    if (p4N >= 0) {
      mqttClient.publish("home/peripheral4/state", p4N ? "1" : "0", true);
      mqttClient.publish("home/relay/peripheral4", p4N ? "1" : "0", true);
    }
  }
}


const unsigned long RS485_RESPONSE_TIMEOUT = 1000;
const unsigned long RS485_RETRY_INTERVAL = 2000;
unsigned long lastRS485Activity = 0;
const unsigned long RS485_ACTIVITY_TIMEOUT = 7000;

// When HB control arrives from AR3, publish current states immediately (numeric, retained)
static inline void onHBControlFromAR3(bool heartbeatOn) {
  // Publish pump
  {
    char p[2];
    snprintf(p, sizeof(p), "%d", pumpState ? 1 : 0);
    mqttClient.publish("home/relay/pump", p, true);
    pendingTxStat = true;
  }
  // Publish peripherals (state + relay mirrors)
  {
    char v[2];
    snprintf(v, sizeof(v), "%d", peripheral1State ? 1 : 0);
    mqttClient.publish("home/peripheral1/state", v, true);
    mqttClient.publish("home/relay/peripheral1", relay1On ? "1" : "0", true);  // valve coil

    snprintf(v, sizeof(v), "%d", peripheral2State ? 1 : 0);
    mqttClient.publish("home/peripheral2/state", v, true);                     // Disable switch
    mqttClient.publish("home/relay/peripheral2", relay2On ? "1" : "0", true);  // cooling relay

    snprintf(v, sizeof(v), "%d", peripheral3State ? 1 : 0);
    mqttClient.publish("home/peripheral3/state", v, true);
    mqttClient.publish("home/relay/peripheral3", v, true);

    snprintf(v, sizeof(v), "%d", peripheral4State ? 1 : 0);
    mqttClient.publish("home/peripheral4/state", v, true);
    mqttClient.publish("home/relay/peripheral4", v, true);
  }
  // Optional: also push the current water level snapshot if present
  {
    char wlbuf[24];
    dtostrf(RX_WATER_LVL, 1, 3, wlbuf);
    mqttClient.publish("home/sensor/water_level_m", wlbuf, true);
  }
}

// RS485Send: routes to backup SoftwareSerial when ar3ActivePath == PATH_BACKUP
bool RS485Send(String data) {
#if RS485_ENABLE && !RS485_RX_ONLY
  String framedData = "<" + addChecksum(data) + ">";
  if (ar3ActivePath == PATH_BACKUP) {
    swTx(framedData);
  } else {
    Serial1.print(framedData);
    Serial1.flush();
  }
  return true;
#else
  return false;
#endif
}

// Inside a <...> frame, wait up to FRAME_TIMEOUT_MS for the closing '>' instead of
// cutting at the 25 ms gap. The gap is measured from when AR1 reads a char, so a busy
// loop (WiFi/MQTT) was splitting frames in two. 25 ms still applies to unframed text.
static const unsigned long FRAME_TIMEOUT_MS = 500UL;

String RS485Receive() {
  static String buffer = "";
  static unsigned long lastRx = 0;
  static bool inFrame = false;
  const unsigned long FRAME_GAP_MS = 25;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    unsigned long now = millis();
    lastRS485Activity = now;
    if ((uint8_t)c >= 0x80) {
      if (RS485Debug) {
        Serial.print("␡");
        Serial.print((uint8_t)c, HEX);
        Serial.print(' ');
      }
      continue;
    }
    if (RS485Debug) {
      uint8_t b = (uint8_t)c;
      if (b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
      Serial.print(' ');
    }
    if (c == '<') {
      buffer = "";
      lastRx = now;
      inFrame = true;
      continue;
    }
    if (c == '>') {
      String msg = buffer;
      buffer = "";
      lastRx = now;
      inFrame = false;
      if (msg.length() == 0) continue;
      bool ok = verifyChecksum(msg);
      frameCount(ok, msg, F("RS485"));
      if (!ok && !FRAME_REJECT) {
        if (msg == "PING" || msg.startsWith("PING_") || msg.startsWith("PONG_")) ok = true;
        if (msg.indexOf(':') >= 0) ok = true;
      }
      if (RS485Debug) {
        Serial.print("RS485 frame: [");
        Serial.print(msg);
        Serial.print("]  ");
        Serial.println(ok ? "ACCEPT" : "DROP");
      }
      if (ok) {
        onRS485FrameFromAR3(msg);  // <── NEW: publish relays when HB=OFF, handle HB:0/1
        return msg;
      }
      continue;
    }
    buffer += c;
    lastRx = now;
  }
  if (buffer.length() > 0 && (millis() - lastRx) > (inFrame ? FRAME_TIMEOUT_MS : FRAME_GAP_MS)) {
    String chunk = buffer;
    chunk.trim();
    buffer = "";
    inFrame = false;
    if (chunk.length() > 0) {
      frameRawCount(chunk, F("RS485"));
      if (!FRAME_REJECT) return chunk;
    }
  }
  return "";
}

// Backup path receiver — mirrors RS485Receive() on swSerial
// The R4 SoftwareSerial receiver holds back the last 1-2 bytes of a burst until the
// next burst starts (~2 s later), found 4 Nov 2026. So on backup: when a frame goes quiet
// for FRAME_TIMEOUT_MS, accept it if its checksum already verifies (only '>' held back);
// otherwise keep waiting up to SW_FRAME_ABANDON_MS for the held-back byte(s).
static const unsigned long SW_FRAME_ABANDON_MS = 2500UL;
static bool swFrameLate = false;   // last returned frame completed only when the next burst began

static String SwSerialReceive() {
  static String swBuf = "";
  static unsigned long swLastRx = 0;
  static bool swInFrame = false;
  static bool swTimeoutChecked = false;
  const unsigned long FRAME_GAP_MS = 25;
  swFrameLate = false;
  while (swSerial.available()) {
    char c = (char)swSerial.read();
    unsigned long now = millis();
    if ((uint8_t)c >= 0x80) continue;
    if (c == '<') { swBuf = ""; swLastRx = now; swInFrame = true; swTimeoutChecked = false; continue; }
    if (c == '>') {
      String msg = swBuf;
      const bool wasLate = swInFrame && swTimeoutChecked;
      swBuf = "";
      swLastRx = now;
      swInFrame = false;
      if (msg.length() == 0) continue;   // e.g. the held-back '>' of a frame accepted on timeout
      bool ok = verifyChecksum(msg);
      frameCount(ok, msg, F("backup"));
      if (ok) { if (wasLate) { swRxLate++; swFrameLate = true; } else swRxOnTime++; }
      if (!ok && !FRAME_REJECT) {
        if (msg == "PING" || msg.startsWith("PING_") || msg.startsWith("PONG_")) ok = true;
        if (msg.indexOf(':') >= 0) ok = true;
      }
      if (ok) {
        onRS485FrameFromAR3(msg);
        return msg;
      }
      continue;
    }
    swBuf += c;
    swLastRx = now;
  }
  if (swBuf.length() > 0) {
    const unsigned long idle = millis() - swLastRx;
    if (swInFrame) {
      if (idle > FRAME_TIMEOUT_MS && !swTimeoutChecked) {
        swTimeoutChecked = true;
        if (verifyChecksum(swBuf)) {
          String msg = swBuf;
          swBuf = "";
          swInFrame = false;
          swRxTimeoutOk++;
          frameCount(true, msg, F("backup"));
          onRS485FrameFromAR3(msg);
          return msg;
        }
      }
      if (idle <= SW_FRAME_ABANDON_MS) return "";
      swRxAbandoned++;
    } else if (idle <= FRAME_GAP_MS) {
      return "";
    }
    String chunk = swBuf;
    chunk.trim();
    swBuf = "";
    swInFrame = false;
    if (chunk.length() > 0) {
      frameRawCount(chunk, F("backup"));
      if (!FRAME_REJECT) return chunk;
    }
  }
  return "";
}

// Path watchdog: detects primary silence, switches to backup, detects recovery.
// Suppressed while ar3PathForced==true (test hook via home/ar3/comms_path/force).
static void tickPathWatchdog() {
  unsigned long now = millis();

  if (!ar3PathForced) {
    if (ar3ActivePath == PATH_PRIMARY) {
      // RS485 silent if it has never delivered (but backup proves AR3 is alive)
      // or if it has not delivered for PRIMARY_FAIL_MS.
      bool rs485Silent = (primaryPathLastTelMs == 0)
                           ? (swPathLastTelMs > 0)
                           : ((now - primaryPathLastTelMs) >= PRIMARY_FAIL_MS);
      if (rs485Silent) {
        ar3ActivePath = PATH_BACKUP;
        ar3Ready = false;  // re-arm gate until backup path confirms AR3 alive
        primarySteadySinceMs = 0;
        Serial.println(F("[PATH] Primary RS485 silent — switched to BACKUP SoftwareSerial"));
        if (mqttClient.connected()) {
          mqttClient.publish("home/ar3/comms_path", "backup", true);
          mqttClient.publish("home/ar3/comms_alert", "1", true);
        }
        lastPathPublishMs = now;
      }
    } else {
      // Recovery: switch back only after Serial1 telemetry has arrived steadily (no gap of
      // 5 s or more) for PRIMARY_STEADY_MS. A flaky RS485 wire then stays on backup instead
      // of flipping between paths every ~15 s.
      const bool primaryFresh = (primaryPathLastTelMs > 0 && (now - primaryPathLastTelMs) < 5000UL);
      if (!primaryFresh) {
        primarySteadySinceMs = 0;
      } else if (primarySteadySinceMs == 0) {
        primarySteadySinceMs = now;
        Serial.println(F("[PATH] Primary RS485 heard again — returning to PRIMARY after 60 s steady"));
      }
      if (primaryFresh && primarySteadySinceMs != 0 && (now - primarySteadySinceMs) >= PRIMARY_STEADY_MS) {
        primarySteadySinceMs = 0;
        ar3ActivePath = PATH_PRIMARY;
        Serial.println(F("[PATH] Primary RS485 restored — switched back to PRIMARY"));
        if (mqttClient.connected()) {
          mqttClient.publish("home/ar3/comms_path", "primary", true);
          mqttClient.publish("home/ar3/comms_alert", "0", true);
        }
        lastPathPublishMs = now;
      }
    }
  }

  // Periodic heartbeat publish of current path status
  if (now - lastPathPublishMs >= PATH_PUBLISH_INTERVAL_MS) {
    lastPathPublishMs = now;
    if (mqttClient.connected()) {
      mqttClient.publish("home/ar3/comms_path",
        (ar3ActivePath == PATH_PRIMARY) ? "primary" : "backup", true);
      mqttClient.publish("home/ar3/comms_alert",
        (ar3ActivePath == PATH_BACKUP) ? "1" : "0", true);
    }
  }
}

static inline void tickFastAckRetry() {
  if (!FS_waitAck) return;
  // A newer command is queued: it carries the current state, so drop the old
  // command's retries (stops a stale pump ON going out after an OFF).
  if (pendingTxCmd) { FS_waitAck = false; return; }

  unsigned long now = millis();
  if ((now - FS_ackT0) < FS_ACK_TIMEOUT_MS) return;
  if (FS_ackRetries < FS_ACK_MAX_RETRY && !txWindowOpen()) return;  // retries also wait for the TX window

  if (FS_ackRetries < FS_ACK_MAX_RETRY) {
    FS_ackRetries++;
    FS_ackT0 = now;

    // Re-send same SEQ, but rebuild CURRENT relay bits so we don't undo timing/lock logic.
    int p1_raw = relay1On ? 1 : 0;
    int p2_raw = relay2On ? 1 : 0;
    int p3_raw = peripheral3State ? 1 : 0;
    int p4_raw = peripheral4State ? 1 : 0;

    // IMPORTANT:
    // If we are intentionally commanding OFF (JF_desiredPump==0), retries must NOT
    // fall back to pumpState (which may still be 1 until ACK arrives), otherwise
    // the retry will re-assert ON and the pump will never drop during TimeOFF.
    int pump_raw;
    if (JF_desiredPump >= 0) {
      pump_raw = (JF_desiredPump > 0) ? 1 : 0;
    } else {
      pump_raw = pumpState ? 1 : 0;
    }

    // Apply lock policy (P2 allowed under lock; pump/P1/P3/P4 forced off)
    const bool lockEffective = (lockState && heartbeatActive);

    int pump01 = lockEffective ? 0 : pump_raw;
    int p1 = lockEffective ? 0 : p1_raw;
    int p2 = p2_raw;
    int p3 = lockEffective ? 0 : p3_raw;
    int p4 = lockEffective ? 0 : p4_raw;

    // Keep stored fields aligned (debug/consistency only)
    FS_waitAckPump = (uint8_t)pump01;
    FS_waitAckP1 = (uint8_t)p1;
    FS_waitAckP2 = (uint8_t)p2;
    FS_waitAckP3 = (uint8_t)p3;
    FS_waitAckP4 = (uint8_t)p4;

    rs485SendRelayCmd(FS_waitAckSeq, pump01, p1, p2, p3, p4);
    ar3FrameEndMs = 0;  // one transmission per window
  } else {
    FS_waitAck = false;  // give up
  }
}

// --- TX window dispatcher (Phase 5) ---
// Opens 5 ms after each AR3 telemetry frame; dispatches one pending frame per window, CMD > STAT > CFG.
static void tickTxWindow() {
  if (!txWindowOpen()) return;
  if (!pendingTxCmd && !pendingTxStat && !pendingTxCfg) return;

  if (pendingTxCmd) {
    pendingTxCmd = false;
    uint8_t seq = (JF_cmdSeq++ & 0x0F);
    rs485SendRelayCmd(seq, pendingCmdPump, pendingCmdP1, pendingCmdP2, pendingCmdP3, pendingCmdP4);
    JF_lastSentPump = pendingCmdPump;
    JF_lastSentP1   = pendingCmdP1;
    JF_lastSentP2   = pendingCmdP2;
    JF_lastSentP3   = pendingCmdP3;
    JF_lastSentP4   = pendingCmdP4;
    JF_cmdFreezeUntilMs = millis() + JF_CMD_FREEZE_MS;
    FS_waitAck     = true;
    FS_waitAckSeq  = seq;
    FS_waitAckPump = (uint8_t)pendingCmdPump;
    FS_waitAckP1   = (uint8_t)pendingCmdP1;
    FS_waitAckP2   = (uint8_t)pendingCmdP2;
    FS_waitAckP3   = (uint8_t)pendingCmdP3;
    FS_waitAckP4   = (uint8_t)pendingCmdP4;
    FS_ackT0       = millis();
    FS_ackRetries  = 0;
    if (RS485Debug) {
      Serial.print("[TX-WIN CMD] seq="); Serial.print((int)seq);
      Serial.print(" pump="); Serial.print(pendingCmdPump);
      Serial.print(" P1="); Serial.print(pendingCmdP1);
      Serial.print(" P2="); Serial.print(pendingCmdP2);
      Serial.print(" P3="); Serial.print(pendingCmdP3);
      Serial.print(" P4="); Serial.println(pendingCmdP4);
    }
    ar3FrameEndMs = 0;
    return;
  }

  if (pendingTxStat) {
    pendingTxStat = false;
    rs485SendStatusSnapshot();
    ar3FrameEndMs = 0;
    return;
  }

  if (pendingTxCfg) {
    pendingTxCfg = false;
    rs485SendP1IntermittentCfg();
    ar3FrameEndMs = 0;
  }
}

// --- AR1 → AR3: send full relay snapshot (PUMP + P1..P4) ---
// --- JF: STATUS snapshot (no PUMP field; AR3 must not treat as a command) ---
// P1 uses actual coil state (relay1On) so AR3 mirrors manual intermittent recirc.
// P2 uses actual coil state (relay2On) so AR3 mirrors shed cooling timing.
static inline void rs485SendStatusSnapshot() {
  // HB:1 means AR1 is actively driving REMOTE mode (not autonomous).
  // HB:0 means AR1 is not driving REMOTE (autonomous active / heartbeat inactive).
  String msg = String("STAT:")
               + "HB:" + String(heartbeatActive ? 1 : 0)
               // Explicitly drive AR3 HA auto flags OFF unless we later add real HA-controlled AUTO.
               + "|HA_AUTO:0"
               + "|HA_RESUME:0"
               + "|P1:" + String(relay1On ? 1 : 0)
               + "|P2:" + String(relay2On ? 1 : 0)
               + "|P3:" + String(peripheral3State ? 1 : 0)
               + "|P4:" + String(peripheral4State ? 1 : 0)
               + "|LOCK:" + String((lockState && heartbeatActive) ? 1 : 0)
               + "|SR:" + String(selfRestartEnabled ? 1 : 0);
  RS485Send(msg);
}

// --- AR1 → AR3: Manual-mode intermittent config for P1 (HA dial, minutes) ---
// NOTE: AR1 does *not* use these timings locally for pump control. They are
// forwarded to AR3/TAR3 as metadata only. Autonomous modes on TAR3 still use
// their own baked-in timing table.
static inline void rs485SendP1IntermittentCfg() {
  // timeOn/timeOff are stored in ms; export whole minutes to AR3
  unsigned long onMin = timeOn / 60000UL;
  unsigned long offMin = timeOff / 60000UL;

  String msg = String("CFG:INT")
               + "|ON_MIN:" + String(onMin)
               + "|OFF_MIN:" + String(offMin);

  RS485Send(msg);

  if (RS485Debug) {
    Serial.print("[RS485 CFG] ");
    Serial.println(msg);
  }
}

// --- JF: COMMAND sender (edge only) ---
// Pump + P1..P4 in a single idempotent CMD frame.
// P2 uses actual coil state (relay2On) so AR3 mirrors shed cooling timing.
static inline void rs485SendRelayCmd(uint8_t seq,
                                     int pump01,
                                     int p1,
                                     int p2,
                                     int p3,
                                     int p4) {
  // Compact, checksum-protected command frame. AR3 should act only on CMD: frames.
  String msg = String("CMD:")
               + "SEQ:" + String((int)(seq & 0x0F))
               + "|LOCK:" + String((lockState && heartbeatActive) ? 1 : 0)
               + "|PUMP:" + String(pump01 ? 1 : 0)
               + "|P1:" + String(p1 ? 1 : 0)
               + "|P2:" + String(p2 ? 1 : 0)
               + "|P3:" + String(p3 ? 1 : 0)
               + "|P4:" + String(p4 ? 1 : 0);
  RS485Send(msg);
}

// --- JF: Edge+ACK command arbiter ---
// Call whenever desired pump *or any P1..P4 state* changes (e.g., from MQTT)
// and periodically from loop() if you ever want background retries.
static inline bool maybeSendPumpCmd() {
  const unsigned long now = millis();

  // Do not send any pump/relay commands until AR3 is clearly alive and sending telemetry.
  if (!ar3Ready) {
    return false;
  }

  // If we have no explicit intent yet, treat desiredPump as "current pump state".
  if (JF_desiredPump < 0) {
    JF_desiredPump = pumpState ? 1 : 0;
  }

  // Raw intent derived from AR1 local state
  int pump01_raw = (JF_desiredPump > 0) ? 1 : 0;
  int p1_raw = relay1On ? 1 : 0;  // actual sprinkler/recirc coil state
  int p2_raw = relay2On ? 1 : 0;  // actual shed-cooling coil state
  int p3_raw = peripheral3State ? 1 : 0;
  int p4_raw = peripheral4State ? 1 : 0;

  // LOCK policy (AR1 truth source):
  //  - LOCK=1 forces Pump/P1/P3/P4 OFF
  //  - P2 (shed cooling) is not masked here directly, but relay2On is itself
  //    forced to the disabled state by the peripheral2/set handler and the
  //    loop() cooling state machine whenever sysLocked() is true, so p2_raw
  //    already reflects LOCK's effect by the time it reaches this frame.
  const bool lock = lockState;
  int pump01 = lock ? 0 : pump01_raw;
  int p1 = lock ? 0 : p1_raw;
  int p2 = p2_raw;
  int p3 = lock ? 0 : p3_raw;
  int p4 = lock ? 0 : p4_raw;

  bool patternChanged =
    (pump01 != JF_lastSentPump) || (p1 != JF_lastSentP1) || (p2 != JF_lastSentP2) || (p3 != JF_lastSentP3) || (p4 != JF_lastSentP4);

  if (!patternChanged) { pendingTxCmd = false; return false; }

  // Opposite pump edges are rate-limited to protect the contactor.
  if (now < JF_cmdFreezeUntilMs && JF_lastSentPump >= 0 && pump01 != JF_lastSentPump) {
    return false;
  }

  // Defer actual send to TX window (dispatched by tickTxWindow after next AR3 telemetry + guard).
  pendingCmdPump = pump01;
  pendingCmdP1   = p1;
  pendingCmdP2   = p2;
  pendingCmdP3   = p3;
  pendingCmdP4   = p4;
  pendingTxCmd   = true;
  return true;
}

// Debug helper: warn periodically if we are gating relay commands
// because AR3 has not been seen as "ready" yet.
static inline void debugAr3Wait() {
  if (ar3Ready) return;  // nothing to say once ready

  // Only complain if RS485 link is nominally "Connected".
  if (!RS485_Status) return;

  unsigned long now = millis();
  if (now - ar3WaitLastPrintMs < 5000UL) return;  // every 5 seconds max
  ar3WaitLastPrintMs = now;

  Serial.print(F("[AR3 WAIT] RS485 link OK but no telemetry yet; relay commands gated. millis="));
  Serial.println(now);
}

// Decls
void setup_wifi();
void publishSensorData();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void turnPumpOn();
void turnPumpOff();
void handleDirectPumpStart();
float getTemperature(int sensorPin, float simulatedValue, bool simulateSensor);
float readWaterLevel();
void calculateFlowRate(bool simulateFlow);
bool getOilPressureState();
bool getWaterPressureState();

// Sensors & helpers
bool getOilPressureState() {
  return RX_OIL_OK;  // received from AR2 telemetry; oilPressureSwitchPin (A1) removed
}
bool getWaterPressureState() {
  return digitalRead(waterPressureSwitchPin);
}
float readWaterLevel() {
  return RX_WATER_LVL;
}
float getTemperature(int sensorPin, float simulatedValue, bool simulateSensor) {
  if (sensorPin == tempSensor1Pin) return RX_INT_TEMP;
  if (sensorPin == tempSensor2Pin) return RX_EXT_TEMP;
  return RX_EXT2_TEMP;
}
void calculateFlowRate(bool simulateFlow) {}

// MQTT snapshot publish
void publishSensorData() {
  // Snapshot RS485 values
  float ext = RX_EXT_TEMP;
  float inT = RX_INT_TEMP;
  float ext2 = RX_EXT2_TEMP;
  float flow = RX_FLOW_LPM;
  float wl = RX_WATER_LVL;
  int emb = RX_EMBER;

  // Pump pressure telemetry from AR2/AR3
  bool oilOk = RX_OIL_OK;            // true = oil pressure high (engine running)
  float waterPv = RX_WATER_PRESS_V;  // raw node voltage 0–5 V
  float fuelLvl = RX_FUEL_LEVEL;     // 0-100%

  // Battery voltage as decoded from RS485 BAT: field
  float batV = batteryVoltage;

  if (MQTT_SUMMARY_DEBUG) {
    Serial.print("RS485→ AR1  EXT:");
    Serial.print(ext, 2);
    Serial.print("  INT:");
    Serial.print(inT, 2);
    Serial.print("  EXT2:");
    Serial.print(ext2, 2);
    Serial.print("  FLOW(Lpm):");
    Serial.print(flow, 2);
    Serial.print("  WL(m):");
    Serial.print(wl, 2);
    Serial.print("  EMBER:");
    Serial.print(emb);
    Serial.print("  OIL_OK:");
    Serial.print(oilOk ? 1 : 0);
    Serial.print("  WATER_PV(V):");
    Serial.print(waterPv, 2);
    Serial.print("  BAT(V):");
    Serial.println(batV, 2);
  }

  char mExt[16], mInt[16], mExt2[16], mFlow[16], mWL[16], mEmber[2];
  char mOil[2];
  char mWaterPv[16];
  char mFuel[16];
  char mBat[16];

  dtostrf(ext, 0, 2, mExt);
  dtostrf(inT, 0, 2, mInt);
  dtostrf(ext2, 0, 2, mExt2);
  dtostrf(flow, 0, 2, mFlow);
  dtostrf(wl, 0, 2, mWL);

  mEmber[0] = emb ? '1' : '0';
  mEmber[1] = '\0';

  mOil[0] = oilOk ? '1' : '0';  // 1 = oil pressure high
  mOil[1] = '\0';

  if (!isnan(waterPv)) {
    dtostrf(waterPv, 0, 2, mWaterPv);
  } else {
    strncpy(mWaterPv, "nan", sizeof(mWaterPv));
    mWaterPv[sizeof(mWaterPv) - 1] = '\0';
  }

  if (!isnan(fuelLvl)) {
    dtostrf(fuelLvl, 0, 2, mFuel);
  } else {
    strncpy(mFuel, "nan", sizeof(mFuel));
    mFuel[sizeof(mFuel) - 1] = '\0';
  }

  if (!isnan(batV)) {
    dtostrf(batV, 0, 2, mBat);
  } else {
    strncpy(mBat, "nan", sizeof(mBat));
    mBat[sizeof(mBat) - 1] = '\0';
  }

  if (mqttClient.connected()) {
    // Existing temperature / flow / WL / flame topics
    mqttClient.publish("home/sensor/ext_temp", mExt, true);
    mqttClient.publish("home/sensor/int_temp", mInt, true);
    mqttClient.publish("home/sensor/temp1", mInt, true);
    mqttClient.publish("home/sensor/temp2", mExt, true);
    mqttClient.publish("home/sensor/ext2_temp", mExt2, true);
    mqttClient.publish("home/sensor/flow_rate", mFlow, true);
    mqttClient.publish("home/sensor/water_level", mWL, true);
    mqttClient.publish("home/sensor/flame_status", mEmber, true);

    // Pump health telemetry
    mqttClient.publish("home/sensor/pump_oil_ok", mOil, true);
    if (!isnan(waterPv)) {
      mqttClient.publish("home/sensor/pump_water_press_v", mWaterPv, true);
    }
    if (!isnan(fuelLvl)) {
      mqttClient.publish("home/sensor/fuel_level", mFuel, true);
    }

    // New: battery voltage
    mqttClient.publish("home/sensor/battery_voltage", mBat, true);
  }
}

// Publish Autonomous (retained). HA mapping: payload "OFF" means autonomous ACTIVE.
static inline void publishAutonomousState(bool autonomousActive) {
  const char* payload = autonomousActive ? "OFF" : "ON";
  static char last[4] = "";
  if (strcmp(last, payload) != 0) {
    mqttClient.publish("heartbeat/control/state", payload, true);
    strncpy(last, payload, sizeof(last));
    Serial.print("[HB] State publish: ");
    Serial.println(payload);
  }
}

// True when relay/pump commands must be blocked: operator locked, HB inactive, or AR3 in AUTO.
static inline bool sysLocked() {
  return lockState || !heartbeatActive || ar3InAutoMode;
}

// MQTT callback with Heartbeat adoption
void callback(char* topic, byte* payload, unsigned int length) {
  String messageTemp;
  for (unsigned int i = 0; i < length; i++) messageTemp += (char)payload[i];
  //  if (MQTT_SUMMARY_DEBUG) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(messageTemp);
  // }

  // --- Remote reboot command (T41) ---
  // Payload: "1" triggers immediate hardware reset via NVIC_SystemReset()
  if (String(topic) == "home/ar1/reboot") {
    String m = messageTemp;
    m.trim();
    if (m == "1") {
      Serial.println(F("[REBOOT] MQTT reboot command received — resetting now."));
      mqttClient.publish("ar1/status", "rebooting", false);
      mqttClient.loop();
      selfRestart(5);
    }
    return;
  }

  // --- HA "Connection self-restarts" switch (all three boards) ---
  if (String(topic) == "home/selfrestart/set") {
    String m = messageTemp;
    m.trim();
    m.toUpperCase();
    selfRestartEnabled = (m == "1" || m == "ON");
    EEPROM.update(selfRestartEnAddr, selfRestartEnabled ? 1 : 0);
    mqttClient.publish("home/selfrestart/state", selfRestartEnabled ? "1" : "0", true);
    pendingTxStat = true;  // tell AR3 (it passes it on to AR2)
    Serial.print(F("[RESTART] connection restarts "));
    Serial.println(selfRestartEnabled ? F("on") : F("off"));
    return;
  }

  // --- Dev/Prod boot policy from HA ---
  // Payload accepted: "0"/"OFF"/"PRODUCTION" or "1"/"ON"/"DEVELOPMENT"
  if (String(topic) == "home/system/devmode/set") {
    String m = messageTemp;
    m.trim();
    m.toUpperCase();

    bool wantDev = (m == "1" || m == "ON" || m == "DEVELOPMENT" || m == "DEV");
    devMode = wantDev;

    EEPROM.write(devModeAddr, devMode ? 1 : 0);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif

    // Publish retained state for HA UI
    mqttClient.publish("home/system/devmode/state", devMode ? "1" : "0", true);

    Serial.print("[DEV MODE] Set via HA to: ");
    Serial.println(devMode ? "DEVELOPMENT" : "PRODUCTION");

    // NOTE: EEPROM clearing happens on next reboot when devMode==true (per your setup() logic)
    return;
  }

  if (String(topic) == "home/lock/control") {
    String m = messageTemp;
    m.trim();
    m.toUpperCase();
    if (m == "0" || m == "OFF" || m == "UNLOCK") lockState = false;
    else if (m == "1" || m == "ON" || m == "LOCK") lockState = true;
    // Publish effective state: while AUTO is active the switch snaps back to OFF
    // even if lockState was just set to false. lockState still stores operator intent
    // so the system unlocks automatically when AUTO exits.
    mqttClient.publish("home/lock/state",
        (lockState || !heartbeatActive || ar3InAutoMode) ? "1" : "0", true);

    if (lockState) {
      // LOCK=ON: force OFF Pump + P1 + P3 + P4.  (P2 is allowed; do NOT force it off.)
      if (pumpState) {
        turnPumpOff();
      }

      // Force OFF sprinkler/recirc (P1)
      if (peripheral1State || relay1On) {
        peripheral1State = false;
        EEPROM.write(peripheral1StateAddr, peripheral1State);

        relay1On = false;
        digitalWrite(relayPeripheral1, HIGH);

        sprinklerPumpOnPhase = false;
        sprinklerPumpPhaseStartMs = 0;

        mqttClient.publish("home/peripheral1/state", "0", true);
        mqttClient.publish("home/relay/peripheral1", "0", true);
        mqttClient.publish("home/peripheral1/info", "LOCKED", true);
      }

      // Force OFF P3
      if (peripheral3State) {
        peripheral3State = false;
        EEPROM.write(peripheral3StateAddr, peripheral3State);
        digitalWrite(relayPeripheral3, HIGH);
        mqttClient.publish("home/peripheral3/state", "0", true);
        mqttClient.publish("home/relay/peripheral3", "0", true);
      }

      // Force OFF P4
      if (peripheral4State) {
        peripheral4State = false;
        EEPROM.write(peripheral4StateAddr, peripheral4State);
        digitalWrite(relayPeripheral4, HIGH);
        mqttClient.publish("home/peripheral4/state", "0", true);
        mqttClient.publish("home/relay/peripheral4", "0", true);
      }

      // Reflect pump state after forced-off (and ensure AR3 gets LOCK immediately)
      publishPumpState();
      pendingTxStat = true;
      maybeSendPumpCmd();  // sends masked CMD including LOCK
    } else {
      // UNLOCK: just refresh HA + AR3 mirrors
      publishPumpState();
      pendingTxStat = true;
      maybeSendPumpCmd();
    }
  }

  if (String(topic) == "home/flame/acknowledge") {
    if (messageTemp == "1" || messageTemp == "ON") {
      flameAcknowledged = true;
      mqttClient.publish("home/flame/alert", "ACKNOWLEDGED", true);
      Serial.println("✅ Flame alert acknowledged from Home Assistant");
    }
  }

  // JF PATCH: unify pump mirror and force RS-485 snapshot once
  // JF PATCH: Edge command via RS-485 (no local coil pulse here)
  if (String(topic) == "home/pump/inTopic") {
    int want = (messageTemp == "1" || messageTemp == "ON") ? 1 : 0;

    // ---- Pump Lock: block MQTT "ON" while system is locked/AUTO ----
    if (sysLocked() && want == 1) {
      Serial.println("Pump LOCK is active. Ignoring MQTT pump ON command.");
      // Reflect the *actual* pump state back to HA
      publishPumpState();         // uses pumpState → home/relay/pump
      pendingTxStat = true;
      return;
    }

    // Allow OFF commands even when locked (safe to stop pump).
    manualPumpOverride = (want == 0);  // previous semantics retained

    // Safety: if the operator explicitly turns the pump OFF while the sprinkler cycle
    // owns the pump, stop the sprinkler cycle immediately so loop() does not re-assert
    // the pump on the next tick. The operator can re-enable the sprinkler when ready.
    if (want == 0 && sprinklerOwnsPump) {
      sprinklerOwnsPump = false;
      peripheral1State = false;
      EEPROM.write(peripheral1StateAddr, 0);
      relay1On = false;
      lastToggleTime = 0;
      sprinklerPumpOnPhase = false;
      sprinklerPumpPhaseStartMs = 0;
      digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH (safe valve position)
      mqttClient.publish("home/peripheral1/state", "0", true);
      mqttClient.publish("home/relay/peripheral1", "0", true);
      mqttClient.publish("home/peripheral1/info", "STOPPED: pump OFF override", true);
      Serial.println("[PUMP OFF] Sprinkler cycle stopped by operator pump-OFF command");
    }

    JF_desiredPump = want;
    maybeSendPumpCmd();  // sends CMD only on edge; freezes opposite for a few seconds

    // Arm or cancel the stop-retry check
    if (want == 0) {
      stopRetryPending  = true;
      stopRetryCheckMs  = millis() + PUMP_STOP_RETRY_MS;
    } else {
      stopRetryPending  = false;  // operator turned pump ON — no retry needed
    }

    // Publish intended (optimistic) UI state immediately; it will be corrected by ACK/telemetry if needed
    char p[2];
    snprintf(p, sizeof(p), "%d", want);
    mqttClient.publish("home/relay/pump", p, true);

    // Send STATUS (no PUMP field) to keep AR3 mirrors P1..P4; AR3 must ignore STAT frames for pump
    pendingTxStat = true;
  }


  if (strcmp(topic, "heartbeat/cmd") == 0 || strcmp(topic, "heartbeat/control") == 0) {
    static char buf[16];
    unsigned int n = (length < sizeof(buf) - 1) ? length : (sizeof(buf) - 1);
    memcpy(buf, payload, n);
    buf[n] = '\0';

    const bool isLegacy = (strcmp(topic, "heartbeat/control") == 0);

    // ── Retain-safety: ignore legacy ON/OFF reflected from HA during boot window ──
    if (isLegacy && (millis() - mqttBootMs) < HB_LEGACY_IGNORE_MS) {
      if (strcasecmp(buf, "ON") == 0 || strcasecmp(buf, "OFF") == 0) {
        Serial.println("[HB] Ignored retained legacy ON/OFF at boot");
        return;
      }
    }

    // New command policy:
    // - Preferred topic: heartbeat/cmd accepts START/STOP/1/0/TRUE/FALSE
    // - Legacy topic: heartbeat/control accepts START/STOP only (ON/OFF for legacy allowed AFTER boot window)
    bool wantOn = false;
    if (!isLegacy) {
      wantOn = (strcasecmp(buf, "START") == 0 || strcasecmp(buf, "1") == 0 || strcasecmp(buf, "TRUE") == 0 || strcasecmp(buf, "ON") == 0);
    } else {
      // Legacy: allow START/STOP always; ON/OFF only after boot window
      const bool legacyOnOffOk = ((millis() - mqttBootMs) >= HB_LEGACY_IGNORE_MS);
      wantOn = (strcasecmp(buf, "START") == 0) || (legacyOnOffOk && (strcasecmp(buf, "ON") == 0 || strcasecmp(buf, "1") == 0 || strcasecmp(buf, "TRUE") == 0));
    }

    if (wantOn) {
      bool wasActive = heartbeatActive;
      heartbeatActive = true;
      digitalWrite(heartbeatPin, HIGH);
      Serial.println("[HB] MQTT → START");
      // STAT frame carries HB:1, so no separate HB announce needed.
      // Push a fresh relay snapshot so AR3 has current P/P1..P4 as Remote begins
      pendingTxStat = true;
      // Also push current P1 intermittent config (HA dial, minutes) for Manual mode
      pendingTxCfg = true;
      // AUTO → MANUAL: start manual control from a clean, safe state - pump, sprinkler (P1),
      // high flow (P3) and gel (P4) OFF; the Disable Shed Cooling setting (P2) is the
      // operator's and is left alone. Until 2026-11-01 AR1 copied AR3's last relay states
      // here; the system is still locked at that moment, so the pump stopped anyway, but
      // AR1 kept "pump ON" and showed it in HA on unlock while AR3 stayed off (and AR3's
      // shed-cooling relay state was copied into the Disable switch, inverted).
      if (!wasActive) {
        JF_desiredPump = 0;
        pumpState = false;
        relay1On = false;
        peripheral1State = false;
        digitalWrite(relayPeripheral1, HIGH);
        peripheral3State = false;
        digitalWrite(relayPeripheral3, HIGH);
        peripheral4State = false;
        digitalWrite(relayPeripheral4, HIGH);
        EEPROM.write(peripheral1StateAddr, 0);
        EEPROM.write(peripheral3StateAddr, 0);
        EEPROM.write(peripheral4StateAddr, 0);
        publishPumpState();
        mqttClient.publish("home/peripheral1/state", "0", true);
        mqttClient.publish("home/relay/peripheral1", "0", true);
        mqttClient.publish("home/peripheral3/state", "0", true);
        mqttClient.publish("home/relay/peripheral3", "0", true);
        mqttClient.publish("home/peripheral4/state", "0", true);
        mqttClient.publish("home/relay/peripheral4", "0", true);
        noInterrupts();
        ar3HasLiveState = false;
        interrupts();
        Serial.println("[HB→MANUAL] Manual starts with pump, P1, P3, P4 OFF (P2 setting kept).");
        maybeSendPumpCmd();
        pendingTxStat = true;
      } else {
        Serial.println("[HB] Already active; skipping AR3 state adoption (would overwrite live commands).");
      }
    } else {
      heartbeatActive = false;
      digitalWrite(heartbeatPin, LOW);
      stopRetryPending = false;  // AR3 owns the pump in AUTO; cancel any pending retry
      Serial.println("[HB] MQTT → STOP");
    }

    EEPROM.write(autonomousStateAddr, heartbeatActive ? 1 : 0);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif
    publishAutonomousState(!heartbeatActive);  // retained state (OFF ⇒ autonomous ACTIVE)
    mqttClient.publish("home/lock/state",
        (lockState || !heartbeatActive || ar3InAutoMode) ? "1" : "0", true);
    return;
  }


  if (String(topic) == "home/peripheral1/set") {
    String m = messageTemp;
    m.trim();
    m.toUpperCase();
    bool wantOn = (m == "1" || m == "ON");

    // Under LOCK=ON, block P1 enable (but allow OFF).
    if (sysLocked() && wantOn) {
      peripheral1State = false;
      EEPROM.write(peripheral1StateAddr, peripheral1State);

      // Ensure coil is OFF and FSM reset
      mqttClient.publish("home/peripheral1/state", "0", true);
      mqttClient.publish("home/relay/peripheral1", "0", true);
      mqttClient.publish("home/peripheral1/info", "LOCKED", true);

      maybeSendPumpCmd();
      pendingTxStat = true;
      Serial.println("Peripheral1 ON ignored due to LOCK");
      return;
    }

    peripheral1State = wantOn;
    EEPROM.write(peripheral1StateAddr, peripheral1State);

    if (peripheral1State) {
      // Enable manual sprinkler mode.
      // Decide driver based on sprinklerUsesPump and safety constraints.
      bool pumpModeNow = isSprinklerPumpModeAllowed();
      sprinklerPumpEffective = pumpModeNow;

      // Valve: coil OFF (logic HIGH) = PASS-THROUGH to sprinklers; coil ON = recirculate.
      // Start in PASS-THROUGH and let loop() manage any cycling.
      relay1On = false;                      // coil OFF = PASS-THROUGH
      lastToggleTime = 0;                    // let loop() start the cycle cleanly
      digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH

      if (!pumpModeNow) {
        // VALVE-mode: P1 cycles PASS-THROUGH/RECIRC; pump ON continuously from the start.
        // Must call turnPumpOn() HERE (before maybeSendPumpCmd below) so JF_desiredPump=1
        // is included in the first CMD frame. Without this, the MQTT callback sends CMD with
        // pump=0, starts a 400ms freeze, and loop()'s turnPumpOn() is then frozen out.
        if (timeOn > 0) {
          turnPumpOn();
          sprinklerOwnsPump = true;
        }
        Serial.println("Peripheral1 (sprinkler) ENABLED → VALVE driver, starting ON phase");
      } else {
        // PUMP-mode: valve held PASS-THROUGH (coil off), pump cycled by loop() FSM
        sprinklerPumpOnPhase = true;
        sprinklerPumpPhaseStartMs = millis();
        turnPumpOn();
        Serial.println("Peripheral1 (sprinkler) ENABLED → PUMP driver, valve PASS-THROUGH, pump ON phase");
      }

      // Publish effective driver mode for HA
      const char* modePayload = pumpModeNow ? "PUMP" : "RECIRC";
      mqttClient.publish("home/sprinkler/driver/state", modePayload, true);

    } else {
      // Disable sprinkler/recirc. Coil OFF (= PASS-THROUGH) and reset PUMP FSM.
      if (relay1On) {
        relay1On = false;
        digitalWrite(relayPeripheral1, HIGH);
      }
      sprinklerPumpOnPhase = false;
      sprinklerPumpPhaseStartMs = 0;
      Serial.println("Peripheral1 (sprinkler) DISABLED → OFF");
    }

    // Full relay CMD (pump + P1..P4) for sprinkler/recirc bit as well.
    maybeSendPumpCmd();
    pendingTxStat = true;

    char v1[2];
    snprintf(v1, sizeof(v1), "%d", peripheral1State ? 1 : 0);
    // HA still sees peripheral1State as the mode enable bit.
    mqttClient.publish("home/peripheral1/state", v1, true);
    mqttClient.publish("home/relay/peripheral1", relay1On ? "1" : "0", true);  // valve coil (1 = recirc)

  } else if (String(topic) == "home/peripheral2/set") {
    // P2 (Disable Shed Cooling): while sysLocked(), the switch is held ON
    // (cooling stays disabled) — attempts to turn it OFF (re-enable cooling)
    // are ignored until Enable Controls is restored. Unlike P1/P3/P4, LOCK
    // blocks the OFF command here, not the ON command.
    String m = messageTemp;
    m.trim();
    m.toUpperCase();
    bool wantOn = (m == "1" || m == "ON");

    if (sysLocked() && !wantOn) {
      peripheral2State = true;
      EEPROM.write(peripheral2StateAddr, peripheral2State);
      digitalWrite(relayPeripheral2, HIGH);  // cooling relay forced off
      relay2On = false;

      mqttClient.publish("home/peripheral2/state", "1", true);
      mqttClient.publish("home/relay/peripheral2", "0", true);
      mqttClient.publish("home/peripheral2/info", "LOCKED", true);

      maybeSendPumpCmd();
      pendingTxStat = true;
      Serial.println("Peripheral2 OFF ignored due to LOCK (Disable Shed Cooling held ON)");
      return;
    }

    peripheral2State = wantOn;

    EEPROM.write(peripheral2StateAddr, peripheral2State);
    // Disable ON → coil off now. Disable OFF → leave the coil to the loop() cycle, which
    // restarts from its ON phase (lastToggleTime2 = 0). The switch never energises the coil.
    if (peripheral2State) {
      digitalWrite(relayPeripheral2, HIGH);
      relay2On = false;
    }
    lastToggleTime2 = 0;

    mqttClient.publish("home/peripheral2/info", "", true);

    // Send masked CMD (LOCK forces Pump/P1/P3/P4 OFF; P2 now respects LOCK too).
    maybeSendPumpCmd();
    pendingTxStat = true;

    char v2[2];
    snprintf(v2, sizeof(v2), "%d", peripheral2State ? 1 : 0);
    mqttClient.publish("home/peripheral2/state", v2, true);
    mqttClient.publish("home/relay/peripheral2", relay2On ? "1" : "0", true);

  } else if (String(topic) == "home/peripheral1/TimeON") {
    // Payload is minutes; store in ms for local manual intermittent logic
    int mins = messageTemp.toInt();
    if (mins < 0) mins = 0;
    if (mins > 240) mins = 240;  // safety clamp

    timeOn = (unsigned long)mins * 60000UL;
    EEPROM.write(peripheral1TimeOnAddr, (uint8_t)mins);

    Serial.print("Peripheral1 TimeON set to ");
    Serial.print(mins);
    Serial.println(" min (MQTT)");

    pendingTxCfg = true;

  } else if (String(topic) == "home/peripheral1/TimeOFF" || String(topic) == "home/peripheral1/TimeOff") {
    int mins = messageTemp.toInt();
    if (mins < 0) mins = 0;
    if (mins > 240) mins = 240;  // safety clamp

    timeOff = (unsigned long)mins * 60000UL;
    EEPROM.write(peripheral1TimeOffAddr, (uint8_t)mins);

    Serial.print("Peripheral1 TimeOFF set to ");
    Serial.print(mins);
    Serial.println(" min (MQTT)");

    mqttClient.publish("home/peripheral1/TimeOFF_rx", String(mins).c_str(), true);  // JF: debug receipt echo

    pendingTxCfg = true;

  } else if (String(topic) == "home/peripheral2/TimeON") {
    int mins = messageTemp.toInt();
    timeOn2 = (unsigned long)mins * 60000UL;
    EEPROM.write(peripheral2TimeOnAddr, (uint8_t)mins);
    lastToggleTime2 = 0;  // restart cycle so new timing takes effect immediately

    Serial.print("Peripheral2 (shed cooling) TimeON set to ");
    Serial.print(mins);
    Serial.println(" min");

  } else if (String(topic) == "home/peripheral2/TimeOFF") {
    int mins = messageTemp.toInt();
    timeOff2 = (unsigned long)mins * 60000UL;
    EEPROM.write(peripheral2TimeOffAddr, (uint8_t)mins);
    lastToggleTime2 = 0;  // restart cycle so new timing takes effect immediately

    Serial.print("Peripheral2 (shed cooling) TimeOFF set to ");
    Serial.print(mins);
    Serial.println(" min");

  } else if (String(topic) == "home/peripheral3/set") {
    String m3 = messageTemp;
    m3.trim();
    m3.toUpperCase();
    bool wantOn = (m3 == "1" || m3 == "ON");

    if (sysLocked() && wantOn) {
      peripheral3State = false;
      EEPROM.write(peripheral3StateAddr, peripheral3State);
      digitalWrite(relayPeripheral3, HIGH);

      mqttClient.publish("home/peripheral3/state", "0", true);
      mqttClient.publish("home/relay/peripheral3", "0", true);
      Serial.println("Peripheral3 ON ignored due to LOCK");

      maybeSendPumpCmd();
      pendingTxStat = true;
      return;
    }

    // Gel needs reduced flow (Dosatron): refuse high flow while gel is on.
    if (wantOn && peripheral4State) {
      peripheral3State = false;
      EEPROM.write(peripheral3StateAddr, peripheral3State);
      digitalWrite(relayPeripheral3, HIGH);

      mqttClient.publish("home/peripheral3/state", "0", true);
      mqttClient.publish("home/relay/peripheral3", "0", true);
      mqttClient.publish("home/peripheral3/info", "GEL ON - reduced flow required", true);
      Serial.println("Peripheral3 (high flow) ON refused: gel is on, reduced flow required");

      maybeSendPumpCmd();
      pendingTxStat = true;
      return;
    }
    mqttClient.publish("home/peripheral3/info", "", true);

    peripheral3State = wantOn;
    digitalWrite(relayPeripheral3, peripheral3State ? LOW : HIGH);
    EEPROM.write(peripheral3StateAddr, peripheral3State);

    // Full relay CMD for P3.
    maybeSendPumpCmd();
    pendingTxStat = true;

    mqttClient.publish("home/peripheral3/state", peripheral3State ? "1" : "0", true);
    mqttClient.publish("home/relay/peripheral3", peripheral3State ? "1" : "0", true);

  } else if (String(topic) == "home/peripheral4/set") {
    String m4 = messageTemp;
    m4.trim();
    m4.toUpperCase();
    bool wantOn = (m4 == "1" || m4 == "ON");

    if (sysLocked() && wantOn) {
      peripheral4State = false;
      EEPROM.write(peripheral4StateAddr, peripheral4State);
      digitalWrite(relayPeripheral4, HIGH);

      mqttClient.publish("home/peripheral4/state", "0", true);
      mqttClient.publish("home/relay/peripheral4", "0", true);
      Serial.println("Peripheral4 ON ignored due to LOCK");

      maybeSendPumpCmd();
      pendingTxStat = true;
      return;
    }

    // Gel needs reduced flow (Dosatron): switching gel on drops high flow first.
    if (wantOn && peripheral3State) {
      peripheral3State = false;
      EEPROM.write(peripheral3StateAddr, peripheral3State);
      digitalWrite(relayPeripheral3, HIGH);
      mqttClient.publish("home/peripheral3/state", "0", true);
      mqttClient.publish("home/relay/peripheral3", "0", true);
      Serial.println("Gel ON: flow switched to reduced (needed for the Dosatron)");
    }
    if (!wantOn) mqttClient.publish("home/peripheral3/info", "", true);

    peripheral4State = wantOn;
    digitalWrite(relayPeripheral4, peripheral4State ? LOW : HIGH);
    EEPROM.write(peripheral4StateAddr, peripheral4State);

    // Full relay CMD for P4 (e.g. high-flow / gel).
    maybeSendPumpCmd();
    pendingTxStat = true;

    mqttClient.publish("home/peripheral4/state", peripheral4State ? "1" : "0", true);
    mqttClient.publish("home/relay/peripheral4", peripheral4State ? "1" : "0", true);

  } else if (String(topic) == "home/pump/TempAutoTrigger") {
    TempAutoTrigger = (messageTemp == "1" || messageTemp == "ON");
  } else if (String(topic) == "home/battery/voltage") {
    batteryVoltage = messageTemp.toFloat();
    Serial.print("Battery voltage updated: ");
    Serial.println(batteryVoltage);

  } else if (String(topic) == "home/sprinkler/driver/set") {
    // JF: Manual sprinkler driver selection (RECIRC vs PUMP)
    // Accept: "RECIRC"/"0"/"OFF" and "PUMP"/"1"/"ON"
    String m = messageTemp;
    m.trim();
    m.toUpperCase();

    bool wantPump = (m == "1" || m == "ON" || m == "PUMP");
    sprinklerUsesPump = wantPump;

    // Reset PUMP-mode FSM; loop will honour new selection
    sprinklerPumpOnPhase = false;
    sprinklerPumpPhaseStartMs = 0;

    // Compute effective mode under current safety constraints
    bool effectivePump = isSprinklerPumpModeAllowed();
    sprinklerPumpEffective = effectivePump;

    // ---- DEBUG: show what the gate used right now ----
    unsigned long onMinDbg = (timeOn == 0) ? 0 : (timeOn / 60000UL);
    unsigned long offMinDbg = (timeOff == 0) ? 0 : (timeOff / 60000UL);
    Serial.print("Sprinkler driver request=");
    Serial.print(wantPump ? "PUMP" : "RECIRC");
    Serial.print("  BAT=");
    Serial.print(batteryVoltage, 2);
    Serial.print("  TimeON=");
    Serial.print(onMinDbg);
    Serial.print("min  TimeOFF=");
    Serial.print(offMinDbg);
    Serial.print("min  Tmin=");
    Serial.print(MAN_PUMP_MODE_MIN_OFF_MIN);
    Serial.print("  effective=");
    Serial.println(effectivePump ? "PUMP" : "RECIRC");

    const char* modePayload = effectivePump ? "PUMP" : "RECIRC";
    mqttClient.publish("home/sprinkler/driver/state", modePayload, true);

    if (!effectivePump && wantPump) {
      mqttClient.publish("home/sprinkler/driver/info", "DOWNGRADED", true);
      Serial.println("Sprinkler driver PUMP request downgraded to RECIRC (battery or Tmin/phase constraint).");
    } else {
      mqttClient.publish("home/sprinkler/driver/info", "", true);
      Serial.print("Sprinkler driver mode set to ");
      Serial.println(modePayload);
    }

  } else if (String(topic) == "home/switch3/inTopic") {
    if (messageTemp == "1" || messageTemp == "ON") {
      heartbeatEnabled = true;
      publishHeartbeat = true;
      pinMode(heartbeatPin, OUTPUT);
      listenStart = 0;
      Serial.println("Heartbeat ENABLED (1 Hz).");
    } else if (messageTemp == "0" || messageTemp == "OFF") {
      heartbeatEnabled = false;
      publishHeartbeat = false;
      confirmationReceived = false;
      listenStart = 0;
      pinMode(heartbeatPin, INPUT_PULLUP);
      Serial.println("Heartbeat DISABLED → Awaiting Arduino 3 takeover confirmation.");
    }

  } else if (String(topic) == "home/ar3/comms_path/force") {
    // Test hook: force comms path without waiting for watchdog timeout.
    // Payloads: "backup" | "primary" | "auto" (releases forced mode)
    String m = messageTemp;
    m.trim();
    m.toUpperCase();
    if (m == "BACKUP") {
      ar3PathForced = true;
      ar3ActivePath = PATH_BACKUP;
      ar3Ready = false;  // re-arm gate so backup telemetry must arrive before commands flow
      primaryPathLastTelMs = 0;  // prevent immediate watchdog recovery
      Serial.println(F("[PATH TEST] Forced → BACKUP path"));
      if (mqttClient.connected()) {
        mqttClient.publish("home/ar3/comms_path", "backup", true);
        mqttClient.publish("home/ar3/comms_alert", "1", true);
      }
    } else if (m == "PRIMARY") {
      ar3PathForced = true;
      ar3ActivePath = PATH_PRIMARY;
      Serial.println(F("[PATH TEST] Forced → PRIMARY path"));
      if (mqttClient.connected()) {
        mqttClient.publish("home/ar3/comms_path", "primary", true);
        mqttClient.publish("home/ar3/comms_alert", "0", true);
      }
    } else if (m == "AUTO") {
      ar3PathForced = false;
      // Do NOT reset primaryPathLastTelMs here: zeroing it while swPathLastTelMs is
      // non-zero causes an immediate spurious backup switch. The timestamp is real
      // data — if primary was recently alive the watchdog will stay on primary; if
      // it is stale the watchdog will correctly switch to backup on the next tick.
      Serial.println(F("[PATH TEST] Released — watchdog now controls path"));
    }
  }
}

// MQTT reconnect
void reconnect() {
  if (testNoMqtt) return;
  static unsigned long lastWiFiWarn = 0;
  // UNO R4 WiFi library can transiently report WL_DISCONNECTED while still
  // associated. Require 3 consecutive bad reads before acting.
  static uint8_t wifiDownCount = 0;

  if (WiFi.status() != WL_CONNECTED) {
    wifiDownCount = (wifiDownCount < 10) ? wifiDownCount + 1 : 10;
    if (wifiDownCount < 10) return;  // transient blip — ignore

    unsigned long now = millis();

    // Periodic log so you can see what's happening on Serial
    if (now - lastWiFiWarn >= 5000) {
      lastWiFiWarn = now;
      Serial.println("WiFi not connected — will retry WiFi + MQTT.");
    }

    // Throttled WiFi re-connect using existing setup_wifi()
    if (now - lastWifiRetryMs >= WIFI_RETRY_INTERVAL_MS) {
      lastWifiRetryMs = now;
      Serial.println("WiFi lost → re-running setup_wifi() for reconnect.");
      WDT.refresh();
      WiFi.disconnect();  // drop any half-dead association
      setup_wifi();       // blocks up to ~20 s per known SSID; sets ACTIVE_SSID/MQTT

      // If WiFi came back, allow immediate MQTT retry on next loop()
      if (WiFi.status() == WL_CONNECTED) {
        wifiDownCount = 0;
        lastMqttRetryMs = 0;
      }
    }
    return;
  }
  wifiDownCount = 0;  // confirmed connected — reset debounce

  if (!mqttClient.connected()) {
    Serial.print("MQTT connect… ");
    if (MQTT_CLIENT_ID[0] == '\0') {
      uint8_t mac[6] = { 0 };
      WiFi.macAddress(mac);
      if (mac[0] | mac[1] | mac[2] | mac[3] | mac[4] | mac[5])
        snprintf(MQTT_CLIENT_ID, sizeof(MQTT_CLIENT_ID),
                 "uno_R4_01-%02X%02X%02X", mac[3], mac[4], mac[5]);
      else
        snprintf(MQTT_CLIENT_ID, sizeof(MQTT_CLIENT_ID),
                 "uno_R4_01-%lu", (unsigned long)millis());
    }
    // Bounded so the 5 s watchdog can't fire: TCP connect 2 s (modem allowed 2.5 s to
    // answer), then up to 2 s for the broker's reply (setSocketTimeout in setup()).
    WDT.refresh();
    espClient.setConnectionTimeout(2000);
    modem.timeout(2500);
    unsigned long t0 = millis();
    bool ok = mqttClient.connect(MQTT_CLIENT_ID,
                                 mqtt_user,
                                 mqtt_password,
                                 "ar1/status",
                                 1,
                                 true,
                                 "offline");
    modem.timeout(1000);
    WDT.refresh();
    unsigned long dt = millis() - t0;
    if (ok) {
      Serial.print("ok in ");
      Serial.print(dt);
      Serial.println(" ms");

      mqttClient.subscribe("home/battery/voltage");
      mqttClient.subscribe("home/sprinkler/driver/set");  // JF: manual sprinkler driver mode (RECIRC/PUMP)
      mqttClient.subscribe("home/pump/inTopic");
      mqttClient.subscribe("home/peripheral1/set");
      mqttClient.subscribe("home/peripheral1/TimeON");
      mqttClient.subscribe("home/peripheral1/TimeOFF");
      mqttClient.subscribe("home/peripheral1/TimeOff");  // JF: defensive subscribe (case/variant)
      mqttClient.subscribe("home/peripheral2/set");
      mqttClient.subscribe("home/peripheral2/TimeON");
      mqttClient.subscribe("home/peripheral2/TimeOFF");
      mqttClient.subscribe("home/peripheral3/set");
      mqttClient.subscribe("home/peripheral4/set");
      mqttClient.subscribe("home/pump/TempAutoTrigger");
      mqttClient.subscribe("home/switch3/inTopic");
      mqttClient.subscribe("home/lock/control");
      mqttClient.subscribe("heartbeat/control");
      mqttClient.subscribe("home/flame/acknowledge");
      mqttClient.publish("ar1/status", "online", true);
      mqttClient.subscribe("home/system/devmode/set");  // HA Dev/Prod toggle (0=prod,1=dev)
      mqttClient.subscribe("home/ar3/comms_path/force");  // test hook: "backup"|"primary"|"auto"
      mqttClient.subscribe("home/ar1/reboot");           // T41: remote reboot command
      mqttClient.subscribe("home/selfrestart/set");      // HA "Connection self-restarts" switch
      mqttClient.publish("home/selfrestart/state", selfRestartEnabled ? "1" : "0", true);
      if (!resetInfoPublished) {
        mqttClient.publish("home/ar1/last_reset", resetCauseText(ar1ResetCause), true);
        resetInfoPublished = true;
      }

      publishAutonomousState(!heartbeatActive);
      char hb[16];
      snprintf(hb, sizeof(hb), "%ld", heartbeatValue);
      mqttClient.publish("heartbeat/value", hb, true);
      WDT.refresh();
    } else {
      Serial.print("fail rc=");
      Serial.print(mqttClient.state());
      Serial.print(" in ");
      Serial.print(dt);
      Serial.println(" ms");
    }
  }
}

// Direct Pump Start (unchanged core behavior)
bool directPumpStart = false;
unsigned long directPumpStartStartTime = 0;
void handleDirectPumpStart() {
  if (sysLocked()) {
    Serial.println("Pump is locked. Direct Pump Start not allowed.");
    return;
  }
  if (RX_OIL_OK) {
    Serial.println("Oil pressure high. Pump already started.");
    return;
  }
  digitalWrite(directPumpPin, HIGH);
  directPumpStartStartTime = millis();
  Serial.println("Direct Pump Start initiated.");
  WDT.refresh();  // the wait below is up to 3 s
  while (millis() - directPumpStartStartTime < 3000) {
    if (RX_OIL_OK) {
      Serial.println("Oil pressure detected. Stopping Direct Pump Start.");
      break;
    }
  }
  digitalWrite(directPumpPin, LOW);
  Serial.println("Direct Pump Start completed.");
}

// Pump control
static void publishPumpState() {
  char p[2];
  snprintf(p, sizeof(p), "%d", pumpState ? 1 : 0);
  mqttClient.publish("home/relay/pump", p, true);
}
void turnPumpOn() {
  if (sysLocked()) {
    Serial.println("Error: Pump is locked. Unlock first.");
    publishPumpState();
    pendingTxStat = true;
    return;
  }
  JF_desiredPump = 1;
  bool sent = maybeSendPumpCmd();
  mqttClient.publish("home/relay/pump", "1", true);
  pendingTxStat = true;
  if (sent) {
    Serial.println("Pump ON command sent to AR3");
  } else if (!ar3Ready) {
    Serial.println(F("[AR3 WAIT] Pump ON deferred — AR3 not yet ready"));
  }
}

void turnPumpOff() {
  JF_desiredPump = 0;
  bool sent = maybeSendPumpCmd();
  mqttClient.publish("home/relay/pump", "0", true);
  pendingTxStat = true;
  if (sent) {
    Serial.println("Pump OFF command sent to AR3");
  } else if (!ar3Ready) {
    Serial.println(F("[AR3 WAIT] Pump OFF deferred — AR3 not yet ready"));
  }
}

// WiFi setup
void setup_wifi() {
  Serial.println("Connecting to known Wi-Fi…");
  for (size_t i = 0; i < NETS_LEN; ++i) {
    Serial.print("→ Trying SSID: ");
    Serial.println(NETS[i].ssid);
    // Join command allowed 3 s (it is only answered once the join finishes); other
    // modem commands stay capped at 1 s (setup()). Watchdog refreshed throughout.
    WDT.refresh();
    modem.timeout(3000);
    WiFi.setTimeout(200);
    WiFi.begin(NETS[i].ssid, NETS[i].pass);
    modem.timeout(1000);
    WDT.refresh();
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000UL) {
      WDT.refresh();
      // Service RS485 receive during the wait so primaryPathLastTelMs stays
      // fresh and the path watchdog does not fire during a WiFi reconnect.
      unsigned long sliceEnd = millis() + 500;
      while (millis() < sliceEnd) {
        String d = RS485Receive();
        if (d.length() > 0 &&
            (d.indexOf("INT_TEMP:") >= 0 || d.indexOf("WATER_LEVEL:") >= 0)) {
          primaryPathLastTelMs = millis();
        }
      }
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      // Wait for DHCP to assign a non-zero IP (R4 WiFi reports WL_CONNECTED before DHCP completes).
      unsigned long dhcpT0 = millis();
      while (WiFi.localIP() == IPAddress(0, 0, 0, 0) && (millis() - dhcpT0) < 5000UL) {
        WDT.refresh();
        delay(200);
      }
      ACTIVE = &NETS[i];
      strncpy(ACTIVE_SSID, NETS[i].ssid, sizeof(ACTIVE_SSID) - 1);
      strncpy(ACTIVE_MQTT, resolveMqttHost(NETS[i].mqttHost), sizeof(ACTIVE_MQTT) - 1);
      Serial.print("Wi-Fi connected → SSID: ");
      Serial.print(ACTIVE_SSID);
      Serial.print("  IP: ");
      Serial.print(WiFi.localIP());
      Serial.print("  RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      Serial.print("MQTT broker for this site: ");
      Serial.println(ACTIVE_MQTT);
      mqttClient.setServer(ACTIVE_MQTT, 1883);
      return;
    } else {
      Serial.println("  (timeout)");
    }
  }
  Serial.println("No known SSID found; MQTT will remain disconnected.");
}

// Read an integer field "KEY<value>|" from a telemetry piece; false if absent or cut off.
static bool autoField(const String& msg, const char* key, int& out) {
  int i = msg.indexOf(key);
  if (i < 0) return false;
  int st = i + strlen(key);
  int j = msg.indexOf('|', st);
  if (j <= st) return false;
  out = msg.substring(st, j).toInt();
  return true;
}

// One-line Auto status for HA (home/auto/status), built from AR3's FSMODE, PHASE, MODE,
// TST and SPR fields. Fields may arrive in different chunks, so the last value of each
// is cached; nothing is published until FSMODE has been seen.
static void publishAutoStatus(const String& msg, int sprA) {
  static int fsm = -1, fsmPending = -1, phase = 0, mode = 0, tst = 0, spr = 0, fb = -1;
  int v;
  // ON/OFF changes only after two matching readings in a row: a single corrupted
  // RS-485 frame showed a 2 s false "OFF" during a test (2026-10-31).
  if (autoField(msg, "|FSMODE:", v) && (v == 0 || v == 1)) {
    if (v == fsm) fsmPending = -1;
    else if (v == fsmPending) { fsm = v; fsmPending = -1; }
    else fsmPending = v;
  }
  if (autoField(msg, "|PHASE:", v)) phase = v;
  // MODE also needs two matching readings: a corrupted frame showed a 2 s false
  // "watching, no trigger" during a fallback test (2026-11-01).
  static int modePending = -1;
  if (autoField(msg, "|MODE:", v)) {
    if (v == mode) modePending = -1;
    else if (v == modePending) { mode = v; modePending = -1; }
    else modePending = v;
  }
  if (autoField(msg, "|TST:", v)) tst = v;
  if (autoField(msg, "|FB:", v)) fb = v;
  if (autoField(msg, "|WLMETHOD:", v) && v >= 1 && v <= 3) {
    static int lastWm = -1;
    if (v != lastWm) {
      const char* t = (v == 1) ? "Gauge"
                    : (v == 2) ? "ESTIMATED from flow meters (tank gauge failed)"
                               : "ESTIMATED - gauge and flow meter failed, assuming 147 L/min while pumping";
      if (mqttClient.publish("home/sensor/wl_source", t, true)) lastWm = v;
    }
  }
  if (autoField(msg, "|TFAIL:", v) && v >= 0 && v <= 7) {
    static int lastTf = -1;
    if (v != lastTf) {
      String t;
      if (v == 0) t = "OK";
      else {
        if (v & 1) t += "EXT";
        if (v & 4) t += (t.length() ? ", EXT2" : "EXT2");
        if (v & 2) t += (t.length() ? ", INT" : "INT");
        t += " failed";
      }
      if (mqttClient.publish("home/sensor/temp_fail", t.c_str(), true)) lastTf = v;
    }
  }
  if (sprA != -99) spr = sprA;
  if (fsm < 0) return;
  String txt;
  if (fsm != 1) {
    txt = "OFF (remote control from HA)";
  } else {
    txt = tst ? "ON (button test) - " : "ON - ";
    if (spr < 0) {
      txt += "starting, waiting for sensors";
    } else {
      switch (mode) {
        case 0: txt += phase ? "cool-down, no action needed" : "watching, no trigger"; break;
        case 1: txt += "tank dry (<=0.05 m), all outputs off"; break;
        case 2: txt += "tank low (<0.30 m), gel priority"; break;
        case 3: txt += "tank 0.30-0.50 m, water-saving cycle"; break;
        case 4: txt += "warm-up, outside >80 C, continuous"; break;
        case 5: txt += "warm-up, outside 60-75 C"; break;
        case 6: txt += "warm-up, outside 75-80 C"; break;
        case 7: txt += "cool-down, outside 70-80 C"; break;
        case 8: txt += "cool-down, outside 60-70 C"; break;
        case 9: txt += "cool-down, outside <=60 C, sprinklers stopped"; break;
        case 10: txt += "cool-down, outside >80 C, continuous reduced"; break;
        case 11: {
          static const char* const fbName[8] = { "B 3/7", "C 5/3", "continuous reduced",
            "continuous high", "continuous reduced", "G 3/5", "H 3/8", "L 2/20" };
          if (fb < 0) return;  // fallback just ended; MODE catches up on the next report
          txt += "ALL temp sensors failed - ";
          if (fb <= 7) txt += "timed fallback stage " + String(fb + 1) + "/8: " + fbName[fb];
          else txt += "no rise seen, staying idle";
          break;
        }
        default: txt += "mode " + String(mode); break;
      }
    }
  }
  static String lastTxt = "";
  if (txt != lastTxt && mqttClient.publish("home/auto/status", txt.c_str(), true)) {
    lastTxt = txt;
  }
}

// Autonomous cycle report from AR3 (SPR:a,on,off,ph | SHD:s) → readable text for HA,
// retained, published only on change. Called from processRS485(), which sees both whole
// frames and the partial chunks AR1 often reads a long telemetry frame in; a field is
// used only if it is terminated by '|' within this piece, so a cut-off value is ignored.
static void publishAutoCycle(const String& msg) {
  int sprSeen = -99;
  int iS = msg.indexOf("SPR:");
  if (iS >= 0) {
    int jS = msg.indexOf('|', iS + 4);
    if (jS > iS + 4) {
      String v = msg.substring(iS + 4, jS);
      int c1 = v.indexOf(','), c2 = v.indexOf(',', c1 + 1), c3 = v.indexOf(',', c2 + 1);
      if (c1 > 0 && c2 > c1 && c3 > c2 && c3 + 1 < (int)v.length()) {
        int a = v.substring(0, c1).toInt();
        int on = v.substring(c1 + 1, c2).toInt();
        int off = v.substring(c2 + 1, c3).toInt();
        int ph = v.substring(c3 + 1).toInt();
        sprSeen = a;
        String txt;
        if (a < 0) txt = "Auto not running";
        else if (a == 0) txt = "No sprinkling";
        else if (a == 1) txt = "Continuous (pump)";
        else txt = String(on) + " min on / " + String(off) + " min off via "
                   + (a == 2 ? "pump" : "recirc") + " - now " + (ph ? "ON" : "OFF");
        static String lastSprTxt = "";
        if (txt != lastSprTxt && mqttClient.publish("home/auto/sprinkler_cycle", txt.c_str(), true)) {
          lastSprTxt = txt;
        }
      }
    }
  }
  int iH = msg.indexOf("SHD:");
  if (iH >= 0) {
    int jH = msg.indexOf('|', iH + 4);
    if (jH > iH + 4) {
      int s = msg.substring(iH + 4, jH).toInt();
      // Plain ON/OFF: AR3's thermostat pulses are a legacy detail; in a fire the
      // shed never cools enough to matter, so cooling is effectively just ON.
      const char* txt = (s < 0) ? "Auto not running"
                      : (s == 0) ? "Cooling OFF"
                                 : "Cooling ON";
      static int lastShd = -99;
      if (s > 1) s = 1;  // all ON states read the same
      if (s != lastShd && mqttClient.publish("home/auto/shed_cooling", txt, true)) {
        lastShd = s;
      }
    }
  }
  publishAutoStatus(msg, sprSeen);
}

// Process RS-485 frames and update caches; reflect HB:0/1 if present
static void processRS485(String rs) {
  rs.trim();
  lastRS485Activity = millis();
  int lt = rs.indexOf('<');
  int gt = rs.lastIndexOf('>');
  String payload = (lt >= 0 && gt > lt) ? rs.substring(lt + 1, gt) : rs;
  payload.trim();
  if (payload.length() == 0) return;
  publishAutoCycle(payload);
  if (payload.startsWith("PONG_AR3")) {
    lastRS485Seen = millis();
    RS485_Status = true;
    return;
  }
  int bar = payload.lastIndexOf('|');
  if (bar >= 0) {
    bool digits = true;
    for (int i = bar + 1; i < (int)payload.length(); ++i) {
      char ch = payload[i];
      if (ch < '0' || ch > '9') {
        digits = false;
        break;
      }
    }
    if (digits) payload = payload.substring(0, bar);
  }

  float ext = 0, inT = 0, ext2 = 0, flow = 0, wl = 0;
  int ember = 0, pumpN = -1, p1N = -1, p2N = -1, p3N = -1, p4N = -1, hbN = -1;
  int phaseN = -1, modeN = -1;
  int oilOkN = -1;
  float waterPv = 0;
  float fuelLvl = 0;

  bool haveExt = false, haveInt = false, haveExt2 = false, haveFlow = false, haveWL = false, haveEmber = false;
  bool havePump = false, haveP1 = false, haveP2 = false, haveP3 = false, haveP4 = false, haveHB = false;
  bool havePhase = false, haveMode = false;
  bool haveOilOk = false, haveWaterPv = false;
  bool haveFuel = false;

  int start = 0;
  while (start < payload.length()) {
    int sep = payload.indexOf(':', start);
    if (sep < 0) break;
    int next = payload.indexOf('|', sep + 1);
    if (next < 0) next = payload.length();
    String key = payload.substring(start, sep);
    key.trim();
    key.toUpperCase();
    String val = payload.substring(sep + 1, next);
    val.trim();

    if (key == "EXT_TEMP" || key == "T2") {
      ext = val.toFloat();
      haveExt = true;
    } else if (key == "INT_TEMP" || key == "T1") {
      inT = val.toFloat();
      haveInt = true;
    } else if (key == "EXT2_TEMP" || key == "T3" || key == "TANK_TEMP") {
      ext2 = val.toFloat();
      haveExt2 = true;
    } else if (key == "FLOW" || key == "FLOW_RATE") {
      flow = val.toFloat();
      haveFlow = true;
    } else if (key == "WATER_LEVEL" || key == "TANK" || key == "WL") {
      wl = val.toFloat();
      haveWL = true;
    } else if (key == "EMBER" || key == "FLAME") {
      ember = val.toInt();
      haveEmber = true;
    } else if (key == "PUMP") {
      pumpN = val.toInt();
      havePump = true;
    } else if (key == "P1") {
      p1N = val.toInt();
      haveP1 = true;
    } else if (key == "P2") {
      p2N = val.toInt();
      haveP2 = true;
    } else if (key == "P3") {
      p3N = val.toInt();
      haveP3 = true;
    } else if (key == "P4") {
      p4N = val.toInt();
      haveP4 = true;
    } else if (key == "OIL_OK") {
      oilOkN = val.toInt();
      haveOilOk = true;
    } else if (key == "WATER_PV") {
      waterPv = val.toFloat();
      haveWaterPv = true;
    } else if (key == "FUEL") {
      fuelLvl = val.toFloat();
      haveFuel = true;
    } else if (key == "HB") {
      hbN = val.toInt();
      haveHB = true;
    } else if (key == "PHASE") {
      phaseN = val.toInt();
      havePhase = true;
    } else if (key == "MODE") {
      modeN = val.toInt();
      haveMode = true;
    } else if (key == "RST" || key == "RST2") {
      // Why AR3 (RST) / AR2 (RST2) last restarted: publish when it changes.
      const int c = val.toInt();
      int &seen = (key == "RST") ? ar3ResetCause : ar2ResetCause;
      const bool valid = val.length() > 0 && isDigit(val[0]);  // ignore a damaged frame's empty value
      if (valid && c >= 0 && c != seen && mqttClient.connected()) {
        seen = c;
        mqttClient.publish(key == "RST" ? "home/ar3/last_reset" : "home/ar2/last_reset",
                           resetCauseText(c), true);
      }
    } else if (key == "FOK" || key == "FBAD" || key == "FNC") {
      // AR3's frame-check counts (frames it received from AR1)
      if (val.length() > 0 && isDigit(val[0])) {
        long &dst = (key == "FOK") ? ar3FcOk : (key == "FBAD") ? ar3FcBad : ar3FcNoChk;
        dst = val.toInt();
      }
    } else if (key == "FSMODE") {
      bool newAutoMode = (val.toInt() != 0);
      if (newAutoMode != ar3InAutoMode) {
        ar3InAutoMode = newAutoMode;
        // Reflect AUTO state in HA lock switch: locked while AR3 is autonomous so the
        // operator sees nothing can happen until the system exits AUTO mode.
        bool effLock = lockState || !heartbeatActive || ar3InAutoMode;
        mqttClient.publish("home/lock/state", effLock ? "1" : "0", true);
        if (!ar3InAutoMode) {
          maybeSendPumpCmd();  // push any queued relay state now that AUTO has ended
        }
      }
    }


    start = next + 1;
  }

  // Early ar3Ready detection (belt-and-suspenders alongside onRS485FrameFromAR3)
  if (!ar3Ready && haveInt && haveWL) {
    ar3Ready = true;
    Serial.println(F("[AR3 READY] Telemetry confirmed via processRS485."));
  }

  // Retry any pump/relay intent that was blocked by the 400 ms contactor-freeze when it
  // was first requested.  By the time the next telemetry frame arrives (every ~2 s) the
  // freeze has always expired, so this is a safe unconditional catch-all.
  if (ar3Ready) maybeSendPumpCmd();

  // --- Battery voltage: parse BAT: (and BAT(V):) field into batteryVoltage ---
  {
    int batPos = payload.indexOf("BAT(V):");
    int keyLen = 7;  // strlen("BAT(V):")

    if (batPos < 0) {
      batPos = payload.indexOf("BAT:");
      keyLen = 4;  // strlen("BAT:")
    }

    if (batPos >= 0) {
      int batEnd = payload.indexOf('|', batPos + keyLen);
      if (batEnd < 0) batEnd = payload.length();
      String batStr = payload.substring(batPos + keyLen, batEnd);
      batStr.trim();
      float bat = batStr.toFloat();

      // Defensive range check: ignore obviously bogus values
      if (bat > 5.0f && bat < 16.5f) {
        batteryVoltage = bat;
      }
    }
  }

  if (haveExt) RX_EXT_TEMP = ext;
  if (haveInt) RX_INT_TEMP = inT;
  if (haveExt2) RX_EXT2_TEMP = ext2;
  if (haveFlow) RX_FLOW_LPM = flow;
  if (haveWL) RX_WATER_LVL = wl;
  if (haveEmber) RX_EMBER = ember;
  if (haveOilOk) RX_OIL_OK = (oilOkN != 0);
  if (haveWaterPv) RX_WATER_PRESS_V = waterPv;
  if (haveFuel) RX_FUEL_LEVEL = fuelLvl;
  if (havePhase) {
    RX_PHASE = phaseN;
    mqttClient.publish("home/auto/phase", (phaseN == 0 ? "0" : "1"), true);
  }
  if (haveMode) {
    RX_MODE = modeN;
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", modeN);
    mqttClient.publish("home/auto/mode", buf, true);
  }


  if (haveWL || havePump || haveP1 || haveP2 || haveP3 || haveP4) {
    noInterrupts();
    if (haveWL) ar3WL_m = wl;
    if (havePump) ar3Pump = (pumpN != 0);
    if (haveP1) ar3P1 = (p1N != 0);
    if (haveP2) ar3P2 = (p2N != 0);
    if (haveP3) ar3P3 = (p3N != 0);
    if (haveP4) ar3P4 = (p4N != 0);
    ar3HasLiveState = true;
    ar3StateMs = millis();
    interrupts();
  }

  if (haveHB) {
    bool wantOn = (hbN != 0);
    heartbeatActive = wantOn;
    digitalWrite(heartbeatPin, wantOn ? HIGH : LOW);
    EEPROM.write(autonomousStateAddr, heartbeatActive ? 1 : 0);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif
    publishAutonomousState(!heartbeatActive);
    mqttClient.publish("home/lock/state",
        (lockState || !heartbeatActive || ar3InAutoMode) ? "1" : "0", true);

    // NEW: publish current relay states immediately so HA refreshes
    onHBControlFromAR3(wantOn);
  }

  bool newFlame = (RX_EMBER != 0);
  if (newFlame && !flameDetected) {
    flameDetectionTime = millis();
    flameAcknowledged = false;
  }
  flameDetected = newFlame;

  publishSensorData();
  lastRS485Seen = millis();
  lastRS485Activity = lastRS485Seen;
  RS485_Status = true;
}

// Setup
void setup() {
  ar1ResetCause = readResetCause();
  selfRestartEnabled = (EEPROM.read(selfRestartEnAddr) != 0);
  selfRestartCount = EEPROM.read(selfRestartCountAddr);
  if (selfRestartCount == 0xFF) selfRestartCount = 0;
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1);
  swSerial.begin(9600);
  Serial.println("ARDUINO 1");
  Serial.print(F("[BOOT] reset cause: "));
  Serial.print(resetCauseText(ar1ResetCause));
  Serial.print(F(", connection restarts "));
  Serial.print(selfRestartEnabled ? F("on") : F("off"));
  Serial.print(F(", in a row "));
  Serial.println(selfRestartCount);
  Serial.println(F("[BOOT] T25: Dual-path AR1↔AR3. Primary=RS485/Serial1, Backup=SoftwareSerial RX=A5 TX=A4"));

  // TestSys detection — JF_TESTSYS_PIN (A3) LOW (strapped to GND) = TestSys; floating HIGH = Production.
  // (Not D13 — D13's onboard LED circuit loads it, giving ~1.9V not 3.3V on INPUT_PULLUP, see above.)
  pinMode(JF_TESTSYS_PIN, INPUT_PULLUP);
  JF_isTestSys = (digitalRead(JF_TESTSYS_PIN) == LOW);
  Serial.print(F("[BOOT] Hardware mode: "));
  Serial.println(JF_isTestSys ? F("TESTSYS") : F("PRODUCTION"));
  if (JF_isTestSys) {  // short connection-check times so TestSys can be tested quickly
    SELF_RESTART_NO_MQTT_MS = 3UL * 60000UL;
    SELF_RESTART_MIN_UPTIME_MS = 2UL * 60000UL;
  }

  pinMode(relayPumpOn, OUTPUT);
  pinMode(relayPumpOff, OUTPUT);
  digitalWrite(relayPumpOn, HIGH);
  digitalWrite(relayPumpOff, HIGH);
  pinMode(relayPeripheral1, OUTPUT);
  pinMode(relayPeripheral2, OUTPUT);
  pinMode(relayPeripheral3, OUTPUT);
  pinMode(relayPeripheral4, OUTPUT);
  pinMode(directPumpPin, OUTPUT);
  digitalWrite(directPumpPin, LOW);
  pinMode(TANK_SENSOR_PIN, INPUT);
  // oilPressureSwitchPin (A1) pinMode removed — A0/A1 now used as SoftwareSerial RX/TX
  pinMode(waterPressureSwitchPin, INPUT);
  pinMode(heartbeatPin, OUTPUT);
  heartbeatEnabled = true;

#if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(64);
#endif
  {
    byte v = EEPROM.read(autonomousStateAddr);
    if (v == 0xFF) {
      heartbeatActive = true;
      EEPROM.write(autonomousStateAddr, 1);  // Note: we override this later to MANUAL on every boot, regardless of EEPROM.
#if defined(ESP8266) || defined(ESP32)
      EEPROM.commit();
#endif
    } else {
      heartbeatActive = (v == 1);           // 0 or 2 = Autonomous
      hbStoppedByMqttLoss = (v == 2);       // MQTT back -> Manual, as before the restart
    }
  }
  digitalWrite(heartbeatPin, heartbeatActive ? HIGH : LOW);
  confirmationReceived = false;
  mqttFailed = false;
  mqttFailStartTime = 0;

  // Cap every WiFi-module command at 1 s (library default 10 s) so no single call can
  // outlast the 5 s watchdog; the WiFi join and MQTT connect get a little longer.
  modem.timeout(1000);
  setup_wifi();
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(2);  // was 10 s; bounded for the 5 s watchdog
  mqttClient.setCallback(callback);
  reconnect();

  // --- Restore the mode saved before the restart (Auto or Manual) and publish it to HA ---
  // Until 2026-11-02 AR1 forced MANUAL here on every boot, which sent HB:1 and took AR3 out of
  // Auto; Auto then only came back ~5 min later. heartbeatActive was read from EEPROM above.
  digitalWrite(heartbeatPin, heartbeatActive ? HIGH : LOW);
  Serial.print(F("[BOOT] mode restored: "));
  Serial.println(heartbeatActive ? F("Manual") : F("Autonomous"));
  publishAutonomousState(!heartbeatActive);


  // Mark start of post-connect window (used by legacy ON/OFF ignore)
  mqttBootMs = millis();

  // JF: retain-safety window to ignore legacy ON/OFF echoed from HA at boot
  // Prefer a new, explicit command topic that HA should publish to (non-retained)
  mqttClient.subscribe("heartbeat/cmd", 1);

  lockState = true;
  mqttClient.publish("home/lock/state", "1", true);

  // Publish initial comms path state
  if (mqttClient.connected()) {
    mqttClient.publish("home/ar3/comms_path", "primary", true);
    mqttClient.publish("home/ar3/comms_alert", "0", true);
  }

  if (WDT.begin(wdtInterval)) {
    Serial.print("Watchdog initialized with interval: ");
    Serial.print(WDT.getTimeout());
    Serial.println(" ms");
    WDT.refresh();
  } else {
    Serial.println("Error initializing watchdog");
  }
  delay(100);

  lastRS485Activity = millis() - RS485_ACTIVITY_TIMEOUT - 1;
  RS485_Status = false;
  RS485_BootMs = millis();

  digitalWrite(relayPumpOn, HIGH);
  digitalWrite(relayPumpOff, HIGH);
  digitalWrite(relayPeripheral1, HIGH);
  digitalWrite(relayPeripheral2, HIGH);
  digitalWrite(relayPeripheral3, HIGH);
  digitalWrite(relayPeripheral4, HIGH);
  Serial.println("Initial relay states set");
  Serial.println("[HB] Tip: Use topic 'heartbeat/cmd' (non-retained) with payload START/STOP.");

  // --- Development / Production boot mode ---
#ifdef FORCE_DEV_MODE
  devMode = true;
  EEPROM.write(devModeAddr, 1);
  Serial.println(F("[BOOT] FORCE_DEV_MODE active — devMode forced ON, written to EEPROM."));
#else
  uint8_t dm = EEPROM.read(devModeAddr);
  if (dm == 0xFF) {
    // Fresh/undefined EEPROM -> default to PRODUCTION
    devMode = false;
    EEPROM.write(devModeAddr, 0);
  } else {
    devMode = (dm != 0);
  }
#endif

  // Publish retained DEV/PROD mode so HA switch matches persisted setting
  if (mqttClient.connected()) {
    mqttClient.publish("home/system/devmode/state", devMode ? "1" : "0", true);
  }


  // If in DEVELOPMENT: clear EEPROM-stored relay/timer state bytes on every reboot.
  // NOTE: do NOT clear devModeAddr itself, otherwise the setting won't persist.
  if (devMode) {
    Serial.println("[DEV MODE] Clearing EEPROM relay/timer states on boot...");

    EEPROM.write(pumpStateAddr, 0);
    EEPROM.write(peripheral1StateAddr, 0);
    EEPROM.write(peripheral2StateAddr, 0);
    EEPROM.write(peripheral3StateAddr, 0);
    EEPROM.write(peripheral4StateAddr, 0);

    // IMPORTANT: Do NOT write 0xFF here, because restore logic maps 0xFF -> legacy 2 min defaults.
    // In DEV maintenance we want a SAFE/IDLE reset: TimeON=0 and TimeOFF=0.
    EEPROM.write(peripheral1TimeOnAddr, 0);
    EEPROM.write(peripheral1TimeOffAddr, 0);
    EEPROM.write(peripheral2TimeOnAddr, 0);
    EEPROM.write(peripheral2TimeOffAddr, 0);
  }

  pumpState = EEPROM.read(pumpStateAddr);
  peripheral1State = EEPROM.read(peripheral1StateAddr);
  peripheral2State = (EEPROM.read(peripheral2StateAddr) == 1);  // 0xFF unwritten → 0 (cooling ON)
  peripheral3State = EEPROM.read(peripheral3StateAddr);
  peripheral4State = EEPROM.read(peripheral4StateAddr);
  if (peripheral3State && peripheral4State) {  // gel needs reduced flow
    peripheral3State = false;
    EEPROM.write(peripheral3StateAddr, 0);
  }
  timeOn2 = EEPROM.read(peripheral2TimeOnAddr) * 60000;
  timeOff2 = EEPROM.read(peripheral2TimeOffAddr) * 60000;
  if (timeOn2 == 0xFF * 60000) timeOn2 = 300000;
  if (timeOff2 == 0xFF * 60000) timeOff2 = 300000;
  timeOn = EEPROM.read(peripheral1TimeOnAddr) * 60000UL;
  timeOff = EEPROM.read(peripheral1TimeOffAddr) * 60000UL;

  // Treat 0xFF as "uninitialised" -> SAFE IDLE (0/0), not legacy 2 min / 2 min.
  if (EEPROM.read(peripheral1TimeOnAddr) == 0xFF) timeOn = 0;
  if (EEPROM.read(peripheral1TimeOffAddr) == 0xFF) timeOff = 0;

  Serial.print("Restored pump state: ");
  Serial.println(pumpState ? "1" : "0");
  Serial.print("Restored Peripheral 1 state: ");
  Serial.println(peripheral1State ? "1" : "0");
  Serial.print("Restored Peripheral 2 state: ");
  Serial.println(peripheral2State ? "1" : "0");
  Serial.print("Restored Peripheral 3 state: ");
  Serial.println(peripheral3State ? "1" : "0");
  Serial.print("Restored Peripheral 4 state: ");
  Serial.println(peripheral4State ? "1" : "0");

  if (pumpState) turnPumpOn();
  else turnPumpOff();

  // BOOT SAFETY: P1/valve always starts PASS-THROUGH.
  // Sprinkler logic in loop() will take control ONLY if peripheral1State is enabled.
  digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH (no inadvertent recirc after reboot)
  // Shed cooling starts OFF; loop() switches it on if the cycle asks for it.
  // (peripheral2State is the "Disable Shed Cooling" switch, so it must not drive the coil.)
  digitalWrite(relayPeripheral2, HIGH);
  digitalWrite(relayPeripheral3, peripheral3State ? LOW : HIGH);
  digitalWrite(relayPeripheral4, peripheral4State ? LOW : HIGH);

  // Track actual relay states
  relay1On = false;
  relay2On = false;

  // Reset sprinkler driver FSMs at boot
  lastToggleTime = 0;
  sprinklerPumpOnPhase = false;
  sprinklerPumpPhaseStartMs = 0;
  sprinklerOwnsPump = false;

  {
    char p[2], v[2];
    snprintf(p, sizeof(p), "%d", pumpState ? 1 : 0);
    mqttClient.publish("home/relay/pump", p, true);
    pendingTxStat = true;
    snprintf(v, sizeof(v), "%d", peripheral1State ? 1 : 0);
    mqttClient.publish("home/peripheral1/state", v, true);
    mqttClient.publish("home/relay/peripheral1", relay1On ? "1" : "0", true);  // valve coil
    snprintf(v, sizeof(v), "%d", peripheral2State ? 1 : 0);
    mqttClient.publish("home/peripheral2/state", v, true);  // Disable switch
    mqttClient.publish("home/relay/peripheral2", "0", true);  // cooling relay starts off
    snprintf(v, sizeof(v), "%d", peripheral3State ? 1 : 0);
    mqttClient.publish("home/peripheral3/state", v, true);
    mqttClient.publish("home/relay/peripheral3", v, true);
    snprintf(v, sizeof(v), "%d", peripheral4State ? 1 : 0);
    mqttClient.publish("home/peripheral4/state", v, true);
    mqttClient.publish("home/relay/peripheral4", v, true);
  }
}

// Main loop
void loop() {
  static bool wasConnected = false;
  bool nowConn = mqttClient.connected();

  // USB test command: WDTTEST freezes the loop so the watchdog restarts AR1.
  {
    static char ucmd[12];
    static uint8_t un = 0;
    while (Serial.available()) {
      const char c = Serial.read();
      if (c == '\r' || c == '\n') {
        ucmd[un] = 0;
        if (un && strcmp(ucmd, "WDTTEST") == 0) {
          Serial.println(F("[USB] WDTTEST: freezing loop - the watchdog should restart AR1"));
          while (true) {}
        } else if (un && strcmp(ucmd, "NOMQTT") == 0) {
          Serial.println(F("[USB] NOMQTT: MQTT dropped until the next restart (tests the connection check)"));
          testNoMqtt = true;
          mqttClient.disconnect();
        }
        un = 0;
      } else if (un < sizeof(ucmd) - 1) {
        ucmd[un++] = (c >= 'a' && c <= 'z') ? c - 32 : c;
      }
    }
  }
  if (!wasConnected && nowConn && hbStoppedByMqttLoss) {
    heartbeatActive = true;
    digitalWrite(heartbeatPin, HIGH);
    EEPROM.update(autonomousStateAddr, 1);  // restart safety: remember Manual again
    Serial.println("MQTT reconnected → heartbeat resumed (was stopped due to MQTT loss).");
    publishAutonomousState(!heartbeatActive);
    hbStoppedByMqttLoss = false;
  }
  if (!nowConn) {
    if (mqttFailStartTime == 0) mqttFailStartTime = millis();
    else if (!mqttFailed && millis() - mqttFailStartTime >= MQTT_FAIL_TIMEOUT) {
      mqttFailed = true;
      if (heartbeatActive) {
        heartbeatActive = false;
        hbStoppedByMqttLoss = true;
        EEPROM.update(autonomousStateAddr, 2);  // restart safety: a restart stays in Auto until MQTT is back
      }
      digitalWrite(heartbeatPin, LOW);
      Serial.println("MQTT unreachable for 5 minutes → heartbeat stopped (Autonomous ACTIVE).");
      publishAutonomousState(!heartbeatActive);
    }
    // Restart safety: still no MQTT after 15 min -> restart (resets the WiFi chip too).
    if (mqttFailStartTime != 0 && selfRestartEnabled &&
        millis() >= SELF_RESTART_MIN_UPTIME_MS &&
        millis() - mqttFailStartTime >= SELF_RESTART_NO_MQTT_MS &&
        selfRestartCount < SELF_RESTART_MAX_IN_ROW) {
      Serial.println(F("[RESTART] no MQTT for too long -> restarting"));
      selfRestart(4);
    }
  } else {
    mqttFailStartTime = 0;
    mqttFailed = false;
    if (selfRestartCount != 0) {
      selfRestartCount = 0;
      EEPROM.update(selfRestartCountAddr, 0);
    }
  }
  wasConnected = nowConn;

  if (mqttClient.connected()) mqttClient.loop();

  static unsigned long _jf_lastSnapshot = 0;
  unsigned long _jf_now = millis();
  if (mqttClient.connected() && (_jf_now - _jf_lastSnapshot >= 15000UL)) {
    _jf_lastSnapshot = _jf_now;
    publishSensorData();
  }

  // (Phase 6) Periodic 1 Hz RS485 HB removed: STAT frames carry HB:1/0 and are
  // dispatched via the TX window after each AR3 telemetry frame (every ~2 s).

  static unsigned long lastSsidCheck = 0;
  if (millis() - lastSsidCheck >= 5000) {
    lastSsidCheck = millis();
    if (WiFi.status() == WL_CONNECTED) {
      String ss = WiFi.SSID();
      if (ss.length() > 0 && ss != String(ACTIVE_SSID)) {
        const NetProfile* np = nullptr;
        for (size_t i = 0; i < NETS_LEN; ++i)
          if (ss == NETS[i].ssid) {
            np = &NETS[i];
            break;
          }
        if (np) {
          strncpy(ACTIVE_SSID, np->ssid, sizeof(ACTIVE_SSID) - 1);
          strncpy(ACTIVE_MQTT, resolveMqttHost(np->mqttHost), sizeof(ACTIVE_MQTT) - 1);
          mqttClient.disconnect();
          mqttClient.setServer(ACTIVE_MQTT, 1883);
          Serial.print("SSID changed → now on ");
          Serial.print(ACTIVE_SSID);
          Serial.print(", MQTT → ");
          Serial.println(ACTIVE_MQTT);
        }
      }
    }
  }

  WDT.refresh();
  unsigned long currentMillis = millis();

  if (currentMillis - lastRelay3Update >= relay3Interval) {
    lastRelay3Update = currentMillis;
    if (digitalRead(relayPeripheral3) != (peripheral3State ? LOW : HIGH)) {
      digitalWrite(relayPeripheral3, peripheral3State ? LOW : HIGH);
      Serial.print("Updated relayPeripheral3 state to: ");
      Serial.println(peripheral3State ? "ON" : "OFF");
    }
  }
  if (currentMillis - lastRelay4Update >= relay4Interval) {
    lastRelay4Update = currentMillis;
    if (digitalRead(relayPeripheral4) != (peripheral4State ? LOW : HIGH)) {
      digitalWrite(relayPeripheral4, peripheral4State ? LOW : HIGH);
      Serial.print("Updated relayPeripheral4 state to: ");
      Serial.println(peripheral4State ? "ON" : "OFF");
    }
  }
  tickFastAckRetry();

  // Primary path receive — track telemetry timestamps for watchdog and TX window
  String rs485Data = RS485Receive();
  if (rs485Data.length() > 0) {
    if (rs485Data.indexOf("INT_TEMP:") >= 0 || rs485Data.indexOf("WATER_LEVEL:") >= 0) {
      primaryPathLastTelMs = millis();
      // Time the TX window from this path only when we transmit on it. On backup,
      // AR3 sends its swSerial copy right after the RS485 one; timing from RS485
      // made AR1 transmit over that copy and lose its '<'.
      if (ar3ActivePath == PATH_PRIMARY) ar3FrameEndMs = millis();
    }
    processRS485(rs485Data);
  }

  // Backup path receive — always poll so swPathLastTelMs is updated even on boot
  // (when ar3ActivePath==PATH_PRIMARY). This allows tickPathWatchdog() to detect
  // that backup is alive and switch paths when RS485 has never delivered. T42.
  // Frame processing is only done when backup path is active to avoid duplicates.
  {
    String swData = SwSerialReceive();
    if (swData.length() > 0) {
      if (swData.indexOf("INT_TEMP:") >= 0 || swData.indexOf("WATER_LEVEL:") >= 0) {
        swPathLastTelMs = millis();  // always track backup liveness
        // A late frame completes only as AR3's next burst begins, so its end is
        // ~300 ms away (267 chars at 9600 baud): open the window after that.
        if (ar3ActivePath == PATH_BACKUP) {
          ar3FrameEndMs = millis() + (swFrameLate ? 300UL : 0UL);
        }
      }
      if (ar3ActivePath == PATH_BACKUP) {
        processRS485(swData);
      }
    }
  }

  // Path watchdog: detect primary failure / recovery
  tickPathWatchdog();
  tickFrameStats();

  // TX window: dispatch pending CMD/STAT/CFG frames after guard delay
  tickTxWindow();

  unsigned long now = millis();
  RS485_Status = (now - lastRS485Activity < RS485_ACTIVITY_TIMEOUT);
  if (!RS485_AnnouncedAtStartup && (now - RS485_BootMs >= RS485_STARTUP_ANNOUNCE_DELAY)) {
    Serial.print("RS485 link to AR3: ");
    Serial.println(RS485_Status ? "Connected" : "Disconnected");
    RS485_AnnouncedAtStartup = true;
    RS485_LastState = RS485_Status;
  }
  if (RS485_AnnouncedAtStartup && (RS485_Status != RS485_LastState)) {
    Serial.println(RS485_Status ? "RS485 link to AR3: RESTORED" : "RS485 link to AR3: LOST");
    RS485_LastState = RS485_Status;
  }

  if (!mqttClient.connected()) {
    unsigned long nowMs = millis();
    if (nowMs - lastMqttRetryMs >= MQTT_RETRY_INTERVAL_MS) {
      lastMqttRetryMs = nowMs;
      reconnect();
    }
  }

  debugAr3Wait();  //Check whether still waiting on AR1

  if (manualPumpOverride) {
    if (!lastManualPumpOverride) turnPumpOff();
    lastManualPumpOverride = true;
  } else {
    lastManualPumpOverride = false;
  }

  // NOTE (JF 2025-11-20, updated 2025-12-XX):
  // In the REMOTE design (heartbeatActive == true), AR1 implements local
  // intermittent logic for:
  //   - peripheral1 (P1) = sprinkler/recirc valve, using timeOn/timeOff
  //     from HA to cycle the recirc valve on/off while the pump is
  //     manually controlled.
  //   - peripheral2 (P2) = shed cooling, using timeOn2/timeOff2.
  //
  // Pump ON/OFF itself is still not cycled locally in AR1; in AUTO it is
  // owned by AR3/TAR3 according to FS_TEMP_LOGIC modes, and in REMOTE it
  // follows direct MQTT commands via home/pump/inTopic.

  // ---------------- Sprinkler / Recirc (P1) intermittent control ----------------
  //
  // Semantics in REMOTE (heartbeatActive == true):
  //  - peripheral1State == 1  → sprinkler/recirc mode enabled from HA
  //  - timeOn == 0 && timeOff == 0 → no intermittent, valve PASS-THROUGH (coil off)
  //  - timeOn > 0 && timeOff > 0  → TimeON: PASS-THROUGH (coil off) → sprinklers;
  //                                   TimeOFF: RECIRC (coil on) → back to the tank
  //  - relay1On tracks actual coil state, and rs485SendStatusSnapshot()
  //    keeps AR3 in sync.
  //
  // We do nothing here when heartbeatActive == false (AUTO mode),
  // so that P1 is fully owned by AR3/TAR3.

  // HA's home/relay/peripheral1 = the recirculation valve coil (1 = recirc to tank, 0 = to the
  // sprinklers), published whenever it changes in manual mode - the valve cycle toggles it
  // without telling HA otherwise. (In AUTO it comes from AR3's telemetry instead.)
  {
    static int lastRelay1Pub = -1;
    if (heartbeatActive && (int)relay1On != lastRelay1Pub &&
        mqttClient.publish("home/relay/peripheral1", relay1On ? "1" : "0", true)) {
      lastRelay1Pub = relay1On;
    }
  }

  if (heartbeatActive) {
    // Evaluate effective driver mode under current config + battery
    bool pumpModeNow = isSprinklerPumpModeAllowed();
    if (pumpModeNow != sprinklerPumpEffective) {
      sprinklerPumpEffective = pumpModeNow;

      // Reset PUMP-mode FSM timing whenever mode flips
      sprinklerPumpOnPhase = false;
      sprinklerPumpPhaseStartMs = 0;
      sprinklerOwnsPump = false;

      const char* modePayload = pumpModeNow ? "PUMP" : "RECIRC";
      mqttClient.publish("home/sprinkler/driver/state", modePayload, true);

      if (!pumpModeNow && sprinklerUsesPump && (timeOn != 0 || timeOff != 0)) {
        mqttClient.publish("home/sprinkler/driver/info", "DOWNGRADED", true);
        Serial.println("Sprinkler driver: PUMP request downgraded to RECIRC (battery or Tmin/phase constraint).");
      } else {
        mqttClient.publish("home/sprinkler/driver/info", "", true);
      }
    }

    if (peripheral1State) {

      // --- Safety/idle mode: TimeON=0 and TimeOFF=0 ---
      // Valve must remain PASS-THROUGH and sprinkler logic must not assert the pump.
      // If the VALVE driver previously claimed the pump, release it now.
      if (timeOn == 0 && timeOff == 0) {
        if (relay1On) {  // valve to the sprinklers
          relay1On = false;
          digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH
          maybeSendPumpCmd();
          pendingTxStat = true;
        }
        if (sprinklerOwnsPump && !lockState) {
          turnPumpOff();
          sprinklerOwnsPump = false;
          Serial.println("Sprinkler VALVE driver: released pump (0/0 idle mode)");
        }
      } else if (!pumpModeNow) {
        // ---------------- VALVE-CYCLING driver (RECIRC selected or forced) ----------------
        // Pump ON continuously while TimeON>0.
        if (timeOn > 0 && !pumpState && JF_desiredPump != 1 && !lockState) {
          turnPumpOn();
          sprinklerOwnsPump = true;
          Serial.println("Sprinkler VALVE driver: pump forced ON (continuous)");
        }

        // If we don't have a proper ON/OFF pair, hold PASS-THROUGH.
        // Fairhaven wiring: PASS-THROUGH = coil OFF (logic HIGH); coil ON = recirculate.
        if (timeOn == 0 || timeOff == 0) {
          if (relay1On) {
            relay1On = false;
            digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH
            maybeSendPumpCmd();
            pendingTxStat = true;
            Serial.println("Peripheral1 (sprinkler, VALVE) hold → PASS-THROUGH");
          }
          lastToggleTime = 0;

        } else {
          // Both > 0: PASS-THROUGH during TimeON, RECIRC during TimeOFF.
          if (lastToggleTime == 0) {
            relay1On = false;
            digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH
            maybeSendPumpCmd();
            pendingTxStat = true;
            lastToggleTime = currentMillis;
            Serial.println("Peripheral1 (sprinkler, VALVE) cycle start → PASS-THROUGH");
          }

          if (!relay1On && (currentMillis - lastToggleTime >= timeOn)) {
            // End of PASS-THROUGH (TimeON) → go RECIRC (TimeOFF)
            relay1On = true;
            lastToggleTime = currentMillis;
            digitalWrite(relayPeripheral1, LOW);  // RECIRC (coil on)
            maybeSendPumpCmd();
            pendingTxStat = true;
            Serial.println("Peripheral1 (sprinkler, VALVE) cycle → RECIRC");
          } else if (relay1On && (currentMillis - lastToggleTime >= timeOff)) {
            // End of RECIRC (TimeOFF) → go PASS-THROUGH (TimeON)
            relay1On = false;
            lastToggleTime = currentMillis;
            digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH (coil off)
            maybeSendPumpCmd();
            pendingTxStat = true;
            Serial.println("Peripheral1 (sprinkler, VALVE) cycle → PASS-THROUGH");
          }
        }

        // PUMP-cycling FSM not used in valve-cycling mode
        sprinklerPumpOnPhase = false;
        sprinklerPumpPhaseStartMs = 0;

      } else {
        // PUMP driver: valve held PASS-THROUGH (coil off); the pump itself is cycled below
        if (relay1On) {
          relay1On = false;
          digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH
          maybeSendPumpCmd();
          pendingTxStat = true;
          Serial.println("Peripheral1 (sprinkler, PUMP driver) → valve PASS-THROUGH");
        }
        lastToggleTime = 0;  // valve FSM not used

        // Safe guards (pumpModeNow implies both >0 & >Tmin, but keep this robust)
        if (timeOn == 0) {
          if (pumpState && !lockState) {
            turnPumpOff();
          }
          sprinklerPumpOnPhase = false;
          sprinklerPumpPhaseStartMs = 0;

        } else if (timeOff == 0) {
          // Should not happen if pumpModeNow is true, but keep safe
          if (!pumpState && !lockState) {
            turnPumpOn();
            sprinklerOwnsPump = true;
          }
          sprinklerPumpOnPhase = true;
          sprinklerPumpPhaseStartMs = currentMillis;

        } else {
          unsigned long nowMs = currentMillis;
          unsigned long onMs = timeOn;
          unsigned long offMs = timeOff;

          if (!sprinklerPumpOnPhase && sprinklerPumpPhaseStartMs == 0) {
            sprinklerPumpOnPhase = true;
            sprinklerPumpPhaseStartMs = nowMs;
            if (!pumpState && !lockState) {
              turnPumpOn();
              sprinklerOwnsPump = true;
            }
            Serial.println("Sprinkler PUMP driver: cycle start → pump ON");

          } else if (sprinklerPumpOnPhase && (nowMs - sprinklerPumpPhaseStartMs >= onMs)) {
            sprinklerPumpOnPhase = false;
            sprinklerPumpPhaseStartMs = nowMs;
            if (!lockState) {
              turnPumpOff();
            }
            Serial.println("Sprinkler PUMP driver: cycle → pump OFF");

          } else if (!sprinklerPumpOnPhase && (nowMs - sprinklerPumpPhaseStartMs >= offMs)) {
            sprinklerPumpOnPhase = true;
            sprinklerPumpPhaseStartMs = nowMs;
            if (!pumpState && !lockState) {
              turnPumpOn();
              sprinklerOwnsPump = true;
            }
            Serial.println("Sprinkler PUMP driver: cycle → pump ON");
          }
        }
      }

    } else {
      // Disabled from HA: valve PASS-THROUGH and stop any sprinkler-driven pump cycling
      if (relay1On) {
        relay1On = false;
        digitalWrite(relayPeripheral1, HIGH);  // PASS-THROUGH
        maybeSendPumpCmd();
        pendingTxStat = true;
        Serial.println("Peripheral1 (sprinkler) disabled → valve PASS-THROUGH");
      }

      lastToggleTime = 0;
      sprinklerPumpOnPhase = false;
      sprinklerPumpPhaseStartMs = 0;

      if (sprinklerOwnsPump && !lockState) {
        turnPumpOff();
        sprinklerOwnsPump = false;
        Serial.println("Sprinkler: released pump → OFF");
      }
    }
  }  // end heartbeatActive

  // ---------------- Periodic STAT heartbeat ----------------
  // Ensures AR3 receives HB:1 at least every STAT_HB_PERIOD_MS (5 s) while in REMOTE mode.
  // Without this, idle periods longer than HB_ARM_MS (5 min) cause AR3 to enter AUTO unexpectedly.
  // Also republishes effective lock state so HA lock switch stays ON while AR3 is autonomous.
  {
    static unsigned long lastStatHbMs = 0;
    if (heartbeatActive && currentMillis - lastStatHbMs >= STAT_HB_PERIOD_MS) {
      lastStatHbMs = currentMillis;
      pendingTxStat = true;
      bool effLock = lockState || !heartbeatActive || ar3InAutoMode;
      mqttClient.publish("home/lock/state", effLock ? "1" : "0", true);
    }
  }

  // ---------------- Pump stop retry ----------------
  // 45 s after a Remote-mode stop command, if both OIL_OK and WATER_PV still indicate
  // the pump is running, send one further stop and alert the operator via MQTT.
  // Only one retry: the flag is cleared on first check regardless of outcome.
  // The check is suppressed if the operator has since turned the pump back ON, or if
  // the system has entered AUTO mode (AR3 owns the pump).
  if (stopRetryPending && currentMillis >= stopRetryCheckMs) {
    stopRetryPending = false;
    if (JF_desiredPump == 0 && heartbeatActive && !ar3InAutoMode) {
      bool oilShowsRunning   = RX_OIL_OK;
      bool waterShowsRunning = (!isnan(RX_WATER_PRESS_V) && RX_WATER_PRESS_V < 0.5f);
      if (oilShowsRunning && waterShowsRunning) {
        maybeSendPumpCmd();
        mqttClient.publish("home/pump/alert",
            "AUTO-RETRY: pump still running 45s after stop command — second stop sent", true);
        Serial.println(F("[PUMP STOP RETRY] Both sensors show pump still running; second stop sent"));
      } else {
        mqttClient.publish("home/pump/alert", "", true);  // clear any previous alert
      }
    }
  }

  // ---------------- Shed cooling (P2) intermittent / thermostat control ----------------
  //
  // Manual (heartbeat ON) only, with or without the pump, so P2 can operate with gravity feed.
  // In Auto, AR3 owns shed cooling and ignores AR1's P2, so AR1 keeps its coil off and
  // publishes nothing (AR3's telemetry reports the real relay). Running the cycle here in Auto
  // made HA show "cooling ON" for ~2 s every 2 min (found 2026-11-02).
  //
  //  - peripheral2State == 1  → "Disable Shed Cooling" switch is ON → P2 forced OFF
  //  - timeOn2 == 0 && timeOff2 == 0 → follows pump (REMOTE only; off otherwise)
  //  - timeOn2 > 0 && timeOff2 == 0 → explicit continuous ON
  //  - timeOn2 == 0 && timeOff2 > 0 → explicit continuous OFF
  //  - timeOn2 > 0 && timeOff2 > 0  → timed intermittent cycle; starts ON immediately
  //
  // relay2On tracks actual coil state; rs485SendStatusSnapshot() keeps AR3 in sync.

  if (!heartbeatActive || ar3InAutoMode) {
    if (relay2On) {
      relay2On = false;
      digitalWrite(relayPeripheral2, HIGH);
    }
    lastToggleTime2 = 0;  // back in manual, the cycle restarts from its ON phase
  } else if (!peripheral2State) {
    // "Disable Shed Cooling" switch is OFF → cooling enabled (default).
    if (timeOn2 == 0 && timeOff2 == 0) {
      // No timing configured: cooling follows pump (REMOTE mode only).
      bool wantOn = heartbeatActive && (JF_desiredPump == 1);
      if (wantOn && !relay2On) {
        relay2On = true;
        digitalWrite(relayPeripheral2, LOW);
        mqttClient.publish("home/relay/peripheral2", "1", true);
        pendingTxStat = true;
        maybeSendPumpCmd();
        Serial.println("Peripheral2 (shed cooling) auto-ON with pump");
      } else if (!wantOn && relay2On) {
        relay2On = false;
        digitalWrite(relayPeripheral2, HIGH);
        mqttClient.publish("home/relay/peripheral2", "0", true);
        pendingTxStat = true;
        maybeSendPumpCmd();
        Serial.println("Peripheral2 (shed cooling) auto-OFF (pump stopped)");
      }
    } else if (timeOn2 > 0 && timeOff2 == 0) {
      // Explicit continuous ON
      if (!relay2On) {
        relay2On = true;
        digitalWrite(relayPeripheral2, LOW);
        mqttClient.publish("home/relay/peripheral2", "1", true);
        pendingTxStat = true;
        maybeSendPumpCmd();
        Serial.println("Peripheral2 (shed cooling) forced continuous ON");
      }
    } else if (timeOn2 == 0 && timeOff2 > 0) {
      // Explicit continuous OFF
      if (relay2On) {
        relay2On = false;
        digitalWrite(relayPeripheral2, HIGH);
        mqttClient.publish("home/relay/peripheral2", "0", true);
        pendingTxStat = true;
        maybeSendPumpCmd();
        Serial.println("Peripheral2 (shed cooling) forced continuous OFF");
      }
    } else {
      // Both > 0: intermittent timing cycle.
      // lastToggleTime2 == 0 means never cycled → start ON immediately.
      if (relay2On && currentMillis - lastToggleTime2 >= timeOn2) {
        relay2On = false;
        lastToggleTime2 = currentMillis;
        digitalWrite(relayPeripheral2, HIGH);
        mqttClient.publish("home/relay/peripheral2", "0", true);
        pendingTxStat = true;
        maybeSendPumpCmd();
        Serial.println("Peripheral2 (shed cooling) cycle → OFF");
      } else if (!relay2On && (lastToggleTime2 == 0 || currentMillis - lastToggleTime2 >= timeOff2)) {
        relay2On = true;
        lastToggleTime2 = currentMillis;
        digitalWrite(relayPeripheral2, LOW);
        mqttClient.publish("home/relay/peripheral2", "1", true);
        pendingTxStat = true;
        maybeSendPumpCmd();
        Serial.println("Peripheral2 (shed cooling) cycle → ON");
      }
    }
  } else {
    // "Disable Shed Cooling" switch is ON → turn relay off.
    if (relay2On) {
      relay2On = false;
      digitalWrite(relayPeripheral2, HIGH);
      mqttClient.publish("home/relay/peripheral2", "0", true);
      pendingTxStat = true;
      maybeSendPumpCmd();
      Serial.println("Peripheral2 (shed cooling) disabled → OFF");
    }
  }
}  //end loop()
