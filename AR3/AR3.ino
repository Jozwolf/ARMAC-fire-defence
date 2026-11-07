// LAST MODIFIED: 2026-11-05 AEST — FRAME_REJECT switch (default 0 = count only). Set to 1 to ignore
//   frames from AR1 with a bad/missing checksum. Tested ON at TestSys 5 Nov. Enable with AR1.
// LAST MODIFIED: 2026-11-04 AEST — ACKs and HB:0 to AR1 are now framed <...|CRC> like telemetry,
//   so AR1 sees the ACK and stops re-sending commands. Needs AR1 4 Nov or later (HB parse fix).
//   An applied pump change is now ACKed at once (was only on AR1's duplicate retry).
// LAST MODIFIED: 2026-11-02 AEST — Restart safety (build 2026-11-02a). A restart no longer drops
//   Auto: AR3 keeps "in Auto", the fallback stage and the tank estimate in EEPROM and resumes them
//   at boot (it used to wait 5 min and restart the fallback as "no rise seen"). Connection check:
//   no AR2 data for 15 min -> restart (not in the first 30 min, never while the pump runs or Auto
//   is busy, max 4 in a row). HA switch arrives as STAT SR:0/1, passed to AR2 as SRCTRL:0/1 every
//   minute. Reset causes sent to AR1 in telemetry as RST (AR3) and RST2 (AR2).
// LAST MODIFIED: 2026-11-01 AEST — Gel needs reduced flow: the constriction makes the pressure
//   difference that drives the Dosatron. P3 (energised = high flow) is now forced off whenever
//   P4 (gel) is on, in AUTO and when applying AR1's commands. AUTO already gave reduced flow in
//   the gel band; this is a last-step guard so no rule change can break it.
// LAST MODIFIED: 2026-11-01 AEST — A test now ends as soon as AR2's scenario finishes (AR2's
//   packets stop carrying SIM:1 for ~1.5 s after having carried it). Before, the test ran on
//   until HA, the button or the 30 min limit ended it, and in that gap AR3 mixed real readings
//   into the simulated fire - on TestSys (no sensors) it started the fallback at stage L with
//   the pump running. After the end AR3 stays in AUTO, watching, until HA switches Auto off.
// LAST MODIFIED: 2026-11-01 AEST — Tank gauge fallback: when AR2 flags the gauge failed (WLFAIL),
//   the level is estimated from the last good reading and the flow meters (DN50 out, DN15 in;
//   147 L/min assumed if the DN50 meter reads nothing with the pump on and the valve to the
//   sprinklers). Used for all water rules and sent as WATER_LEVEL, with WLMETHOD 2/3. A dead
//   gauge used to read 0 m and force the dry cutoff. USB TEST4 = AR2 scenario 4.
// LAST MODIFIED: 2026-11-01 AEST — Recirculation-valve intermittent cycles fixed: pump runs
//   throughout, P1 off (sprinklers) in the ON phase and energised (tank) in the OFF phase. It was
//   inverted (P1 energised while "spraying", pump off otherwise), so the 3/7, 5/3, 3/5, 3/8 cycles
//   never reached the sprinklers. USB TEST3 = AR2 scenario 3 (steady 3/7 cycle).
// LAST MODIFIED: 2026-11-01 AEST — Cool-down now starts when, after outside has reached 60 C,
//   outside is >= 5 C below its peak and inside >= 2 C below its peak. The old fixed 5-min window
//   (1 min on TestSys) could miss the fall when the two drops landed in different windows.
// LAST MODIFIED: 2026-11-01 AEST — Temperature-sensor fallback restored (lost in the FS_TEMP_LOGIC
//   rewrite; the Aug 2025 AR3 had it, as described on the wiki). AR2 now flags failed sensors
//   (TFAIL). Outside temp = EXT, else EXT2, else INT + last outside-inside offset. If all three
//   fail for 1 min: timed stages from the mode in force, B 3/7 -> C 5/3 -> cont reduced -> cont
//   high -> cont reduced -> G 3/5 -> H 3/8 -> L 2/20 (held), 5 min each (1 min on TestSys); no
//   rise seen -> stay idle. Water-level rules still apply; shed cooling follows the stage.
//   Sensors good for 2 min -> normal control. Also: cool-down above 80 C now runs continuous
//   reduced flow (it stopped sprinkling), high flow only in warm-up. USB serial commands
//   TEST1 / TEST2 / TESTSTOP start a scenario from a computer (TEST2 = sensor-failure run).
//   While the failure is being confirmed the current plan is held (sprinklers don't stop).
//   Ending a test clears the simulated fire history (phase, peaks, fallback, offset), and AR2's
//   simulated packets (SIM:1) are ignored whenever no test is running.
//   Test start repeats HB:0 to AR1 every 5 s until AR1 confirms (a lost frame left AR1 remote).
//   Telemetry adds TFAIL and FB (fallback stage).
// LAST MODIFIED: 2026-10-31 AEST — HB_FRESH_FOR_EXIT_MS 10 s -> 12 s: AR1's STAT frames in AUTO
//   are sometimes 10 s apart, which restarted the 15 s exit timer (exit took ~40 s, now ~15-20 s).
//   Still below HB_STABLE_MS (15 s).
// LAST MODIFIED: 2026-10-31 AEST — HA can now end a button test: while a test runs, a heartbeat
//   that goes OFF then ON (2 reports in a row) - i.e. Autonomous Mode switched off in HA - stops
//   the test, and the normal 15 s exit returns AR3 to REMOTE. Button and 30 min limit unchanged.
// LAST MODIFIED: 2026-10-31 AEST — This is the former AR3B sketch, renamed AR3B -> AR3 (no code
//   change; boot messages still say AR3B). Old AR3 retired to AR3_OLD.
// LAST MODIFIED: 2026-10-31 AEST — LCD probe at boot: if no LCD answers at I2C 0x27 the LCD is
//   disabled (warning printed) instead of hanging into the watchdog; I2C transfers bounded at
//   25 ms. Guards against the TestSys D11 strap falling out (no LCD there).
// LAST MODIFIED: 2026-10-31 AEST — WiFi join (begin) now allowed 3 s; the 1 s cap made every
//   fresh join to Arduino2_AP fail. Other modem commands stay capped at 1 s.
// LAST MODIFIED: 2026-10-30 AEST — WiFi retry can no longer trip the 5 s watchdog: each modem
//   command (all of them, set in setup) capped at 1 s, begin() waits 1 s; AR3B reboot-looped when
//   Arduino2_AP was slow/absent. Boot prints a build line.
// LAST MODIFIED: 2026-10-31 AEST — Telemetry adds TST:0/1 (button test running) so HA can
//   show why Auto is on.
// LAST MODIFIED: 2026-10-30 AEST — Telemetry now reports the autonomous sprinkler cycle
//   and shed-cooling state (SPR:/SHD: fields) so AR1 can show them read-only in HA.
// LAST MODIFIED: 2026-10-25 AEST — LOCK safety kick no longer applies in autonomous mode
//   (it fought tickAuto: OFF/ON latch pulses every 0.5 s whenever the firmware lock was left on).
// LAST MODIFIED: 2026-10-21 AEST — LATCH_FB_FITTED switch (default 0): until the D10
//   read-back wire is fitted, D10 floats HIGH (= "latch OFF"), which would block every
//   OFF pulse. With 0, AR3B pulses unconditionally exactly like production AR3, so
//   AR3B can be the standard sketch before the self-control rewiring (box in charge,
//   D9 open = sequencer idle, D8 unconnected). Set 1 after fitting D10, then run L1-L5.
// LAST MODIFIED: 2026-10-10 AEST — Latch read-back on D10 (pump latch output / core 1
//   at M4): pumpPulseIfNeeded() only pulses when the latch is not already in the
//   requested state, because the latch module toggles on a repeated pulse (confirmed
//   on the spare). Also fixes enterAuto() pulsing PIN_COIL_OFF regardless of
//   FS_POLARITY_INVERT.
// LAST MODIFIED: 2026-10-08 AEST — Added D9 mode input (PIN_MODE_BYPASS, INPUT_PULLUP,
//   via new 3-core cable core B): LOW = BYPASS (ARMAC start sequencer active), open = BOX /
//   disabled (sequencer idle, crank held off; fails safe). Step 1 install: link to earth.
// LAST MODIFIED: 2026-09-27 AEST — AR3B.ino created (Option B variant of AR3.ino;
//   AR3/AR3.ino remains the SFS01-V2-box version). Engine start sequencer replaces the
//   SFS01-V2 box. New PIN_CRANK (D8, spare relay ch1, active LOW) drives the
//   starter AutoRelay (pin 86); fuel/RUN is held by the existing pump latch relay.
//   Starts on a pumpState OFF->ON edge (REMOTE or AUTO): up to 5 attempts, crank
//   <=5 s (ends on oil pressure), 20 s rest. Logic interlock: crank only while pump
//   demanded, AR2 data fresh (<1.5 s; needs AR2B's true 10 Hz UDP) and oil pressure
//   absent (attempts 2-5; attempt 1 is a fixed 2 s crank that ignores oil pressure,
//   per SFS practice). Stall = report only. D9 mode input (2026-10-08): LOW = BYPASS
//   (sequencer active), open = BOX/disabled (idle, crank held off). Watchdog fixed (8192 ms was invalid -> 5000 ms);
//   WiFi.begin() timeout 3 s and crank forced off before each WiFi retry.
//   Test button (A3): START now needs a debounced >= 3 s hold; any press ends a test;
//   tests auto-end after 30 min (an accidental brush started a test on 26 Sep).
// LAST MODIFIED: 2026-08-22 14:57 AEST — Added diesel fuel level sender relay:
//   parse FUEL: from AR2's UDP line into ar2Fuel, forward it on the existing
//   checksummed telemetry frame to AR1 (both RS-485 primary and serial
//   fallback paths, same as WATER_PV/OIL_OK — no new command tag needed).
// LAST MODIFIED: 2026-07-15 22:15 AEST — Added a clarifying comment above
//   PIN_TEST_BTN (A3) distinguishing it from JF_TESTSYS_PIN (D11) — the two
//   were conflated during today's debugging (D11 mistakenly believed to
//   trigger scenario tests; it doesn't, PIN_TEST_BTN/A3 does).
//
//   RS485 flapping on TestSys is RESOLVED (previous entry, kept for
//   context). Root cause was tickLCD() hanging 8-18s on every call, freezing
//   loop() and starving RS485 telemetry — because D11 (JF_TESTSYS_PIN)
//   wasn't reliably grounded, so JF_isTestSys read false at boot and AR3
//   assumed a real LCD was present (attempting genuine I2C traffic to a
//   floating, absent bus). Fixed by physically strapping D11 to GND
//   permanently. Confirmed stable on AR1's serial monitor after the fix. All
//   diagnostic instrumentation added while hunting this has been stripped
//   out — it did its job. Two real, unrelated firmware bugs found and fixed
//   along the way: frame-parser resync on '<' in feedByte()/feedByteSW()
//   (matches AR1's RS485Receive()), and a missing ACK on the first command
//   after boot in applyPumpEdgeRaw(). Neither was the main cause. See
//   project_dn50_flow_sensor.md memory for the full writeup.
//T-AR3B
// T41-AR3 (2026-05-31): Add RA4M1 hardware watchdog (8192ms) — auto-reboot on loop freeze.
// T40 (2026-05-18): Dynamic swSerial.begin()/end() to fix WiFi instability.
//   - A2 is a valid RA4M1 SoftwareSerial RX pin; its interrupt handler fires
//     whenever AR1 sends HB on swSerial, disrupting WiFi SPI to the ESP32-S3.
//   - tickSwSerial() calls swSerial.end() while RS-485 is active (disabling
//     the A2 interrupt) and swSerial.begin() after SW_BACKUP_ACTIVATE_MS of
//     RS-485 silence (activating backup path).
//   - At boot swSerial is disabled for SW_BACKUP_ACTIVATE_MS to give AR1 time
//     to send its first RS-485 frame before declaring the primary path dead.
//   - fsSendFramedBackup() and sendLineOnBothPaths() now gate on swSerialEnabled.
// T39 (2026-05-18): Gate swSerial TX on RS-485 silence to fix WiFi dropout.
//   - lastRs485RxMs tracks last RS-485 frame receipt
//   - fsSendFramedBackup() and sendLineOnBothPaths() suppress swSerial TX
//     while RS-485 frames have arrived within SW_BACKUP_ACTIVATE_MS (10 s)
//   - Prevents SoftwareSerial interrupt-disable from breaking AR2 WiFi link
//   - Failover to swSerial TX still occurs after 10 s RS-485 silence (same
//     threshold as AR1 PRIMARY_FAIL_MS)
// T38 (2026-05-12): Reduce AUTO exit thresholds for faster REMOTE recovery.
//   - HB_FRESH_FOR_EXIT_MS: 35 s → 10 s
//   - HB_STABLE_MS: 60 s → 15 s
//   EXIT AUTO now triggers ~15-20 s after user enables REMOTE (was ~65 s).
// T31 (2026-05-08): Add SoftwareSerial backup path for AR1↔AR3 comms.
//   - Backup path: PIN_SW_RX=A2, PIN_SW_TX=A1
//   - Cable: AR1.TX(A4) → AR3.RX(A2), AR3.TX(A1) → AR1.RX(A5)
//   - AR3 listens on both paths; responds on whichever received the command
//   - AR3 sends telemetry on both paths so AR1 can detect primary recovery
//   - All ACKs (pump edge, same-dir, non-pump CMD) now sent on both paths

#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <WDT.h>                // T41-AR3: RA4M1 hardware watchdog
#include <EEPROM.h>

// ---------------- Restart safety (2026-11-02) ----------------
// A restart (watchdog, power cut, connection check) must not drop Auto. AR3 keeps in EEPROM
// whether it is in Auto, the fallback stage and the tank estimate, and on boot carries on with
// them (it used to wait 5 min before Auto and start the fallback again as "no rise seen").
// Connection check: no data from AR2 for 15 min -> restart (also resets the WiFi chip). Never in
// the first 30 min after boot, never while the pump runs or Auto is spraying/cooling/in a
// fallback stage, at most 4 in a row (then it waits for AR2 to come back). HA's "Connection
// self-restarts" switch arrives from AR1 as STAT ...|SR:0/1 and is passed on to AR2 (SRCTRL).
// Why it last restarted is sent to AR1 as RST (AR2's as RST2):
//   0 power-on, 1 watchdog, 2 software, 3 reset pin / upload, 4 connection check, 5 HA command
static const int EE3_AUTO = 0;         // 0xA5 = in Auto
static const int EE3_FB = 1;           // fallback: 0xFF none, 0..7 stage, 9 = active, no rise seen
static const int EE3_EST = 4;          // float: tank estimate while the gauge has failed (NaN = none)
static const int EE3_GOOD = 8;         // float: last good gauge reading (NaN = none)
static const int EE3_RST_PENDING = 12; // reason for a restart AR3 asked for itself
static const int EE3_RST_COUNT = 13;   // connection restarts in a row
static const int EE3_SR_ENABLED = 14;  // 0 = connection restarts off (0xFF/other = on)
static unsigned long SR3_NO_AR2_MS = 15UL * 60000UL;      // TestSys: 3 min (set in setup)
static unsigned long SR3_MIN_UPTIME_MS = 30UL * 60000UL;  // TestSys: 2 min
static const uint8_t SR3_MAX_IN_ROW = 4;
static uint8_t resetCause3 = 3;
static int8_t ar2ResetCause = -1;      // from AR2's RST field
static bool selfRestartEnabled = true;
static uint8_t selfRestartCount = 0;
static unsigned long lastSrCtrlMs = 0;

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
    const uint8_t p = EEPROM.read(EE3_RST_PENDING);
    if (p == 4 || p == 5) c = p;
  }
  EEPROM.update(EE3_RST_PENDING, 0xFF);
  return c;
}

static void selfRestart(uint8_t reason) {
  EEPROM.update(EE3_RST_PENDING, reason);
  if (reason == 4) EEPROM.update(EE3_RST_COUNT, (uint8_t)min(255, selfRestartCount + 1));
  Serial.print(F("[RESTART] reason "));
  Serial.println(reason);
  delay(200);
  NVIC_SystemReset();
}

/* ---------- TestSys / FireSys detect ----------
   LOW on JF_TESTSYS_PIN (11 strapped to GND) => TestSys (no LCD hardware)
   HIGH / floating                             => FireSys (LCD present)
*/
// --- JF: TestSys detector (ground A0 on TestSys; leave open on FireSys) ---
const uint8_t JF_TESTSYS_PIN = 11;  // tie to GND on TestSys
static bool JF_isTestSys = false;

// Boot time — set in setup(); used by RS-485 quiet period and HB arm logic.
static unsigned long bootMs = 0;
// RS-485 transmit quiet period after boot — no TX until bus has settled.

// RS-485 receive timestamp — updated whenever a complete RS-485 frame arrives.
// Used to gate SoftwareSerial TX: only transmit on swSerial when RS-485 has
// been silent for SW_BACKUP_ACTIVATE_MS, avoiding WiFi dropout caused by
// SoftwareSerial TX disabling interrupts on the RA4M1.
static unsigned long lastRs485RxMs = 0;
static const unsigned long SW_BACKUP_ACTIVATE_MS = 10000UL;  // match AR1 PRIMARY_FAIL_MS
static const unsigned long RS485_BOOT_QUIET_MS = 3000UL;
// Backup path enable state — managed by tickSwSerial().
// false = swSerial.end() called (A2 RX interrupt off, WiFi safe)
// true  = swSerial.begin() called (backup path receiving and transmitting)
static bool swSerialEnabled = false;

// ---------------- Build-time options ----------------
// FS_DISABLE_AUTONOMOUS:
//   0 = normal behaviour (AUTO allowed based on HB/HA_AUTO)
//   1 = field-safe: NEVER enter Autonomous on this node (except TestSys)
#ifndef FS_DISABLE_AUTONOMOUS
#define FS_DISABLE_AUTONOMOUS 0
#endif

static inline bool fsAutoAllowed() {
  // Allow full AUTO on TestSys jig even if field builds compile with
  // FS_DISABLE_AUTONOMOUS=1, so we can keep testing the logic.
  if (JF_isTestSys) return true;
  return (FS_DISABLE_AUTONOMOUS == 0);
}


// ---------------- Serial & RS-485 ----------------
#define USB_BAUD 9600
#define RS485_BAUD 9600
#define RS485_PORT Serial1

// Backup SoftwareSerial path
// Cable: AR1.TX(A4) → AR3.RX(A2), AR3.TX(A1) → AR1.RX(A5)
#define PIN_SW_RX  A2
#define PIN_SW_TX  A1
SoftwareSerial swSerial(PIN_SW_RX, PIN_SW_TX);

static inline void rs485SendLine(const String &s) {
  if (millis() - bootMs < RS485_BOOT_QUIET_MS) return;
  RS485_PORT.print(s);
  RS485_PORT.flush();
}

// CRC-8 (polynomial 0x07), matching AR1. Replaced the old XOR byte checksum on
// 4 Nov 2026: CRC-8 also catches double-bit errors, short noise bursts and swapped bytes.
static uint8_t crc8(const String &s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c ^= (uint8_t)s[i];
    for (uint8_t b = 0; b < 8; ++b) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static String fsAddChecksum(const String &msg) {
  char hex[3];
  snprintf(hex, sizeof(hex), "%02X", crc8(msg, msg.length()));
  return msg + "|" + hex;
}

// Frame check, COUNT ONLY (4 Nov 2026): frames from AR1 are checked and counted but
// still handled exactly as before. Counts go to AR1 in telemetry (FOK/FBAD/FNC) and
// to the serial log every 10 min. Rejection comes later.
// FRAME_REJECT 1 = ignore frames from AR1 with a bad or missing checksum. 0 = count only.
// Committed as 0 until tested on TestSys and loaded at Fairhaven on site.
#ifndef FRAME_REJECT
#define FRAME_REJECT 0
#endif
static unsigned long fcOk = 0, fcBad = 0, fcNoChk = 0;
static unsigned long fcLastBadLogMs = 0, fcLastNoChkLogMs = 0;
// Returns true if the frame should be handled.
static bool frameCheck(const String &f, bool viaBackup) {
  const int bar = f.lastIndexOf('|');
  const String chk = (bar >= 0) ? f.substring(bar + 1) : String();
  if (bar < 0 || chk.length() != 2 || !isHexadecimalDigit(chk[0]) || !isHexadecimalDigit(chk[1])) {
    fcNoChk++;
    if (FRAME_REJECT && (fcLastNoChkLogMs == 0 || millis() - fcLastNoChkLogMs >= 10000UL)) {
      fcLastNoChkLogMs = millis();
      Serial.print(F("[FRAME] no checksum ("));
      Serial.print(viaBackup ? F("backup") : F("RS485"));
      Serial.print(F("), rejected: "));
      Serial.println(f.substring(0, 80));
    }
    return !FRAME_REJECT;
  }
  if (crc8(f, bar) == (uint8_t)strtoul(chk.c_str(), nullptr, 16)) {
    fcOk++;
    return true;
  }
  fcBad++;
  if (fcLastBadLogMs == 0 || millis() - fcLastBadLogMs >= 10000UL) {
    fcLastBadLogMs = millis();
    Serial.print(F("[FRAME] bad checksum ("));
    Serial.print(viaBackup ? F("backup") : F("RS485"));
    Serial.print(FRAME_REJECT ? F("), rejected: ") : F("), still accepted: "));
    Serial.println(f.substring(0, 80));
  }
  return !FRAME_REJECT;
}

static void fsSendFramed(const String &payload) {
  if (millis() - bootMs < RS485_BOOT_QUIET_MS) return;
  String frame = "<" + fsAddChecksum(payload) + ">";
  RS485_PORT.print(frame);
  RS485_PORT.flush();
}

// Send framed telemetry on backup SoftwareSerial path.
// Suppressed when swSerial is disabled (RS-485 active) — avoids WiFi dropout
// from SoftwareSerial TX disabling RA4M1 interrupts during normal operation.
static void fsSendFramedBackup(const String &payload) {
  if (!swSerialEnabled) return;
  String frame = "<" + fsAddChecksum(payload) + ">";
  swSerial.print(frame);
}

// Send ACKs and HB:0 on both RS485 and backup SoftwareSerial, framed with a CRC-8
// like telemetry (4 Nov 2026; before that they were raw text, which AR1's command
// logic never saw, so AR1 re-sent every command 4 times). Backup is suppressed while
// swSerial is disabled (RS-485 active) — same WiFi protection as above.
static inline void sendLineOnBothPaths(const String &s) {
  fsSendFramed(s);
  fsSendFramedBackup(s);
}

// ---------------- WiFi / UDP to AR2 ----------------
static const uint16_t UDP_PORT = 5000;
WiFiUDP udp;

#include "config.h"
const char *WIFI_SSID = AR2_AP_SSID;
const char *WIFI_PASSWORD = AR2_AP_PASSWORD;

String lastUdpLine;
unsigned long lastUdpMs = 0;
const unsigned long UDP_STALE_MS = 3000;

// Track an in-progress connection attempt
static bool wifiConnecting = false;
static unsigned long wifiConnectStartMs = 0;
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;  // give each attempt up to 10 s
static uint8_t wifiFailCount = 0;          // consecutive failed attempts
static unsigned long wifiBackoffMs = 0;    // current retry interval (doubles on each failure)

// ---------------- Outputs ----------------
#define PIN_COIL_ON 2
#define PIN_COIL_OFF 3
static const uint8_t PIN_P1 = 4;
static const uint8_t PIN_P2 = 5;
static const uint8_t PIN_P3 = 6;
static const uint8_t PIN_P4 = 7;
// Option B (2026-09-27): starter command via new 3-core cable core A → relay R1 → AutoRelay pin 86.
// Active LOW like P1–P4. Driven ONLY by tickStartSeq().
static const uint8_t PIN_CRANK = 8;
// Mode input (2026-10-08): D9 via core B of a new 3-core cable to the relay cabinet (D8 crank on core A).
//   LOW  (linked to earth)  = BYPASS: ARMAC controls pump starting (start sequencer active)
//   HIGH (open, pull-up)    = BOX / disabled: sequencer idle, crank output held OFF
// Step 1 installation: fixed link to earth. Step 2: a pole of the BOX/BYPASS selector.
// Open circuit (broken wire, no link) = idle, i.e. fails safe.
static const uint8_t PIN_MODE_BYPASS = 9;
// Latch read-back (2026-10-10): D10 wired to the pump latch module's output line
// (core 1 at the M4 end). LOW = latch ON (pulling R8/R2 inputs low). The latch
// module is in "interlock + shutdown" mode, so a repeated pulse on the same input
// TOGGLES it - confirmed on the spare 10 Oct 2026. AR3 therefore only pulses when
// the latch is not already in the requested state (also after a reboot).
static const uint8_t PIN_LATCH_FB = 10;
// 0 = D10 read-back wire NOT fitted: always pulse (production AR3 behaviour).
// 1 = fitted (section 3.3 step 18): pulse only when the latch differs from the request.
#define LATCH_FB_FITTED 0

// ---------------- TEST button ----------------
// A3 = momentary auto-scenario-test trigger. AR3B (2026-09-27): a test now
// starts only on a deliberate HOLD of >= 3 s (debounced); any press while a
// test runs ends it; a test auto-ends after 30 min. (Previously a short press
// started a test - a brush of the button on 26 Sep 2026 did exactly that and
// locked HA out of pump control.) NOT the same as JF_TESTSYS_PIN (D11, above) —
// that's a separate, boot-time-only TestSys/Production hardware detect pin
// with nothing to do with scenario testing. Easy to conflate; don't.
#ifndef PIN_TEST_BTN
#define PIN_TEST_BTN A3
#endif

static bool testBtnPrev = true;

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
static unsigned long lcdLastMs = 0;
static const unsigned long LCD_PERIOD_MS = 2500;
static uint8_t lcdPage = 1;

// Set at boot: false on TestSys (D11 strap), or when no LCD answers at 0x27.
static bool lcdPresent = false;

// Helper: true only when an LCD is physically present
static inline bool fsHasLcd() {
  return lcdPresent;
}

// ---------------- Timing / Guards ----------------
#define FS_POLARITY_INVERT 1

static const unsigned long COIL_PULSE_MS = 300;
static const unsigned long EDGE_GUARD_OPP_MS = 400;
static const unsigned long CMD_SILENCE_MS = 400;
static const unsigned long SAME_DIR_MIN_MS = 400;

// ---------------- Pump state ----------------
static bool havePumpState = false;
static bool pumpState = false;
static unsigned long lastChangeMs = 0;
static unsigned long cmdIgnoreUntilMs = 0;
static unsigned long lastSameDirMs = 0;
static int lastSeqApplied = -1;

// ---------------- Mode & heartbeat ----------------
enum FsMode : uint8_t { FS_MODE_REMOTE = 0,
                        FS_MODE_AUTONOMOUS = 1 };

static FsMode fsMode = FS_MODE_REMOTE;
static bool fsInAuto = false;
static bool haAutoEnable = false;
static bool haAutoResume = true;
static unsigned long modeFlipMs = 0;

static unsigned long lastHbMs = 0;
static const unsigned long HB_FRESH_MS = 4000;
// AR1 sends STAT:HB:1 every 5 s (STAT_HB_PERIOD_MS).
// HB_FRESH_FOR_EXIT_MS spans 2x that interval so one missed frame does not abort exit.
// HB_STABLE_MS > HB_FRESH_FOR_EXIT_MS so a single spurious HB:1 cannot trigger EXIT AUTO.
static const unsigned long HB_FRESH_FOR_EXIT_MS = 12000UL;  // 10 s = AR1's occasional STAT gap in AUTO, which reset the exit timer
static const unsigned long HB_WARN_MS = 60000;
static const unsigned long HB_ARM_MS = 300000;
static const unsigned long HB_STABLE_MS = 15000;

// Track boot time so lastHbMs==0 does NOT force immediate AUTO on power-up.
// bootMs is declared at the top of the file alongside RS485_BOOT_QUIET_MS.


// ---------------- LOCK (from AR1 / HA) ----------------
// LOCK=1 forces Pump/P1/P3/P4 OFF. P2 is allowed.
static bool remoteLockActive = false;

// If lock becomes active while pump is ON, keep kicking OFF until it drops.
static unsigned long lockPumpKickMs = 0;
static const unsigned long LOCK_PUMP_KICK_MS = 500;


// ---------------- AR2 link state ----------------
static unsigned long lastAr2Ms = 0;
static const unsigned long AR2_TIMEOUT_MS = 25000;  // 25 s: tolerates infrequent AR2 UDP sends + brief WiFi glitches
static bool ar2EverSeen = false;
static bool ar2LastState = false;

// ---------------- WiFi retry (AR2 AP) ----------------
static unsigned long lastWifiRetryMs = 0;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;  // 15 s between attempts
static bool wifiLastState = false;


// ---------------- AR2 sensor snapshot ----------------
static float ar2IntTemp = NAN;
static float ar2ExtTemp = NAN;
static float ar2Ext2Temp = NAN;
static float ar2WaterLvl = NAN;
static float ar2FlowOut = NAN;
static float ar2FlowIn = NAN;
static float ar2Flow = NAN;
static float ar2Bat = NAN;
static int ar2Ember = 0;
// Pump pressure from AR2
static bool ar2OilOk = false;       // true when PC817 OUT == LOW
static float ar2WaterPressV = NAN;  // raw voltage at water-pressure node
static float ar2Fuel = NAN;         // 0-100%, calibrated on AR2
static uint8_t ar2TFail = 0;        // AR2 TFAIL bits: 1 EXT, 2 INT, 4 EXT2 failed (0 if AR2 doesn't send it)
static uint8_t ar2WlFail = 0;       // AR2 WLFAIL: tank gauge failed (0 if AR2 doesn't send it)
static float ar2WlLastGood = NAN;   // last tank level read while the gauge was working
static float fsExtIntOffset = NAN;  // last good outside minus inside temperature (for the INT proxy)

// ---------------- Autonomous spec (JF 2025-11) ----------------
//
// This section encodes the autonomous behaviour exactly as per the
// JF Fire Defence System spec. All numeric thresholds and cycle
// timings are collected here for easy review/tuning.
//

// High-level phase: which external-temp branch we are in.
enum FsPhase : uint8_t {
  FS_PHASE_WARMUP = 0,
  FS_PHASE_COOLDOWN = 1
};

static FsPhase fsPhase = FS_PHASE_WARMUP;

// Exportable autonomous mode code for HA (numeric; map to names in HA).
enum FsModeCode : uint8_t {
  FS_MODE_IDLE = 0,
  FS_MODE_DRY_CUTOFF = 1,
  FS_MODE_GEL_PRIORITY = 2,
  FS_MODE_RECIRC_BAND = 3,
  FS_MODE_WARM_CONT = 4,
  FS_MODE_WARM_B = 5,
  FS_MODE_WARM_C = 6,
  FS_MODE_CD1 = 7,
  FS_MODE_CD2 = 8,
  FS_MODE_CD_OFF = 9,
  FS_MODE_CD_CONT = 10,   // cool-down, outside still > 80 C: continuous reduced flow
  FS_MODE_FALLBACK = 11   // all temperature sensors failed: timed stages (see FB telemetry)
};

static uint8_t fsModeCode = FS_MODE_IDLE;


// Simple on/off cycle expressed in whole minutes.
struct FsCycleMinutes {
  uint8_t on;
  uint8_t off;
};

// -------- Water level bands (m) --------
static const float FS_WL_DRY_M = 0.05f;      // <= 0.05  → all OFF
static const float FS_WL_GEL_M = 0.30f;      // (0.05,0.30) → gel priority
static const float FS_WL_RECIRC_HI = 0.50f;  // [0.30,0.50] → recirc cycle
// > 0.50 → normal temp-based behaviour

// -------- Internal temperature thresholds (°C) --------
static const float FS_T_INT_SHED_ON_C = 50.0f;    // >50 → start shed cycle
static const float FS_T_INT_SHED_CONT_C = 60.0f;  // >60 → shed continuous

// -------- External temperature thresholds (°C) --------
// Warm-up:
static const float FS_T_EXT_WARM_INT_LO_C = 60.0f;  // start intermittent
static const float FS_T_EXT_WARM_B_HI_C = 75.0f;    // 60–75 → [3,7]
static const float FS_T_EXT_WARM_C_HI_C = 80.0f;    // 75–80 → [5,3]
static const float FS_T_EXT_CONT_C = 80.0f;         // >80 → continuous sprinklers

// Cool-down bands (used once fsPhase==FS_PHASE_COOLDOWN):
//  (70,80] → [3,5]
//  (60,70] → [3,8]
//  <=60    → no further sprinkler cycles (system winds down)
static const float FS_T_EXT_CD1_LO_C = 70.0f;
static const float FS_T_EXT_CD2_LO_C = 60.0f;

// -------- Cool-down entry definition --------
// Cooldown is entered once, after outside has reached at least 60 C (a fire-level rise), both
//   EXT_TEMP is >= 5°C below its peak AND INT_TEMP is >= 2°C below its peak.
// (Until 2026-11-01 this compared readings one fixed 5-min window apart, which could miss the
//  fall when the two drops landed in different windows.)
static const float FS_CD_EXT_DROP_C = 5.0f;
static const float FS_CD_INT_DROP_C = 2.0f;

// -------- TRISE / flow selection --------
static const float FS_TRISE_HIGH_FLOW_C_PER_MIN = 1.0f;  // >1°C/min → HIGH (P3=ON)

// -------- Battery rule (intermittent only) --------
static const float FS_BAT_LOW_V = 12.0f;  // <=12V → intermittent always on P1

// -------- Canonical cycles (Ton,Toff in minutes) --------
static const FsCycleMinutes FS_CYCLE_RECIRC_BAND = { 3, 8 };  // water 0.30–0.50 m
static const FsCycleMinutes FS_CYCLE_WARM_B = { 3, 7 };       // EXT 60–75 // For test can set 3 to 1.
static const FsCycleMinutes FS_CYCLE_WARM_C = { 5, 3 };       // EXT 75–80
static const FsCycleMinutes FS_CYCLE_CD1 = { 3, 5 };          // EXT 70–80
static const FsCycleMinutes FS_CYCLE_CD2 = { 3, 8 };          // EXT 50–70
static const FsCycleMinutes FS_CYCLE_CD3 = { 3, 30 };         // EXT <=50
static const FsCycleMinutes FS_CYCLE_SHED = { 4, 4 };         // INT 50–60 shed cycle

// -------- Shed thermostat mode (INT-driven) --------
//
// Thresholds:
//   - FS_T_INT_SHED_THERM_C        = 47°C (start pulses)
//   - FS_T_INT_SHED_THERM_RESET_C  = 45°C (re-arm threshold)
// Timing:
//   - FS_SHED_THERM_PULSE_MS       = 1 minute ON
//   - FS_SHED_THERM_MIN_GAP_MS     = 2 minutes between pulse starts
//
// Behaviour:
//   - When INT rises to >= 47°C after cooling below 45°C, start a 1-minute
//     shed cooling pulse (P2 ON).
//   - If the time between the start of the current pulse and the previous
//     pulse is < 2 minutes, latch into continuous ON.
//   - INT > FS_T_INT_SHED_CONT_C (60°C) still forces immediate continuous ON.
static const float FS_T_INT_SHED_THERM_C = 47.0f;
static const float FS_T_INT_SHED_THERM_RESET_C = 45.0f;
static const unsigned long FS_SHED_THERM_PULSE_MS = 60000UL;     // 1 min
static const unsigned long FS_SHED_THERM_MIN_GAP_MS = 120000UL;  // 2 min

// -------- Derived TRISE state (degC/min) --------
static float fsLastExtForRise = NAN;
static unsigned long fsLastExtRiseMs = 0;
static float fsTrise = 0.0f;

// -------- Cool-down detection: peaks since this AUTO run / test began --------
static float fsPeakExt = NAN;
static float fsPeakInt = NAN;

// -------- Sprinkler driver for intermittent cycles --------
enum FsSprinklerDriver : uint8_t {
  FS_SPRINKLER_NONE = 0,
  FS_SPRINKLER_PUMP_P0,   // drive main pump (P0)
  FS_SPRINKLER_RECIRC_P1  // drive recirc only (P1)
};

// Per-loop autonomous outputs & cycle configuration.
// These are "what we want", tickAuto() will turn them into actual
// coil/relay states and handle the timers.
struct FsAutoOutputs {
  bool pumpContinuous;    // P0 continuous ON
  bool recircContinuous;  // P1 continuous ON
  bool shedContinuous;    // P2 continuous ON
  bool gelOn;             // P4 ON
  bool flowHigh;          // P3 high-flow (TRISE > 1)

  bool sprinklerIntermittent;  // true → use sprinklerCycle & sprinklerDrv
  FsCycleMinutes sprinklerCycle;
  FsSprinklerDriver sprinklerDrv;

  bool shedIntermittent;  // true → use shedCycle for P2
  FsCycleMinutes shedCycle;
};

static FsAutoOutputs fsAutoOut;

// Runtime FSM state for intermittent sprinkler and shed cycles (ms).
static unsigned long fsSprPhaseMs = 0;
static bool fsSprOnPhase = false;
static unsigned long fsShedPhaseMs = 0;
static bool fsShedOnPhase = false;

// Thermostat-mode runtime state for shed cooling (P2).
static bool fsShedThermContinuous = false;   // latched continuous ON
static bool fsShedThermPulseActive = false;  // currently in a 1-min pulse window
static bool fsShedThermArmed = false;        // ready to trigger at 47°C
static unsigned long fsShedThermPulseStartMs = 0;
static unsigned long fsShedThermLastStartMs = 0;

// ---------------- Autonomous helpers ----------------

// Compute TRISE (degC/min) from external temperature history.
// We update on each new sample; caller decides how often to call
// ---------------- Autonomous helpers ----------------

// Compute TRISE (degC/min) from external temperature history.
// We update on each new sample; caller decides how often to call.
static void fsUpdateTrise(float tExt) {
  if (isnan(tExt)) return;

  unsigned long now = millis();
  if (isnan(fsLastExtForRise)) {
    fsLastExtForRise = tExt;
    fsLastExtRiseMs = now;
    fsTrise = 0.0f;
    return;
  }

  unsigned long dtMs = now - fsLastExtRiseMs;
  if (dtMs == 0) return;

  float dtMin = dtMs / 60000.0f;
  if (dtMin <= 0.0f) return;

  fsTrise = (tExt - fsLastExtForRise) / dtMin;

  fsLastExtForRise = tExt;
  fsLastExtRiseMs = now;
}

// Track cooldown phase: WARMUP → COOLDOWN once outside has reached 60 C and both
// temperatures have fallen from their peaks (EXT by >= 5 C, INT by >= 2 C).
static void fsUpdatePhase(float tExt, float tInt) {
  if (fsPhase == FS_PHASE_COOLDOWN) return;
  if (isnan(tExt) || isnan(tInt)) return;

  if (isnan(fsPeakExt) || tExt > fsPeakExt) fsPeakExt = tExt;
  if (isnan(fsPeakInt) || tInt > fsPeakInt) fsPeakInt = tInt;
  if (fsPeakExt < FS_T_EXT_WARM_INT_LO_C) return;  // no fire-level rise yet

  if (fsPeakExt - tExt >= FS_CD_EXT_DROP_C && fsPeakInt - tInt >= FS_CD_INT_DROP_C) {
    fsPhase = FS_PHASE_COOLDOWN;
    fsLogTs();
    Serial.print(F("[AUTO] Phase -> COOLDOWN (EXT peak "));
    Serial.print(fsPeakExt, 1);
    Serial.print(F(" now "));
    Serial.print(tExt, 1);
    Serial.print(F(", INT peak "));
    Serial.print(fsPeakInt, 1);
    Serial.print(F(" now "));
    Serial.print(tInt, 1);
    Serial.println(F(")"));
  }
}

// ---------------- Temperature-sensor fallback ----------------
// Stages as on the wiki / Aug 2025 AR3: B, C, CONT reduced, CONT high, CONT reduced, G, H, L.
struct FsFbStage { uint8_t on; uint8_t off; bool cont; bool high; const char* name; };
static const FsFbStage FS_FB_STAGES[8] = {
  { 3, 7,  false, false, "B 3/7" },
  { 5, 3,  false, false, "C 5/3" },
  { 0, 0,  true,  false, "continuous reduced" },
  { 0, 0,  true,  true,  "continuous high" },
  { 0, 0,  true,  false, "continuous reduced" },
  { 3, 5,  false, false, "G 3/5" },
  { 3, 8,  false, false, "H 3/8" },
  { 2, 20, false, false, "L 2/20" },
};
static const unsigned long FS_FB_STAGE_MS_FIELD = 5UL * 60000UL, FS_FB_STAGE_MS_TESTSYS = 60000UL;
static const unsigned long FS_FB_ENTER_MS_FIELD = 60000UL,       FS_FB_ENTER_MS_TESTSYS = 10000UL;
static const unsigned long FS_FB_RECOVER_MS_FIELD = 120000UL,    FS_FB_RECOVER_MS_TESTSYS = 20000UL;
static unsigned long fsFbStageIntMs = FS_FB_STAGE_MS_FIELD;
static unsigned long fsFbEnterMs = FS_FB_ENTER_MS_FIELD;
static unsigned long fsFbRecoverMs = FS_FB_RECOVER_MS_FIELD;

static bool fsFbActive = false;        // all sensors failed (latched until they recover)
static int8_t fsFbStage = -1;          // 0..7, or -1 = no rise seen (stay idle)
static unsigned long fsFbStageMs = 0;
static unsigned long fsAllFailSinceMs = 0;
static unsigned long fsAnyOkSinceMs = 0;
static uint8_t fsLastTempMode = 0;     // last temperature-driven mode (FS_MODE_*), for the start stage
static bool fsExtProxy = false;        // outside temp is INT + offset

// Keep the fallback state in EEPROM so a restart mid-fire resumes the same stage.
static void fsSaveFb() {
  const uint8_t v = !fsFbActive ? 0xFF : (fsFbStage < 0 ? 9 : (uint8_t)fsFbStage);
  EEPROM.update(EE3_FB, v);
}

static void fsFbReset() {
  fsFbActive = false;
  fsFbStage = -1;
  fsFbStageMs = 0;
  fsAllFailSinceMs = 0;
  fsAnyOkSinceMs = 0;
  fsLastTempMode = FS_MODE_IDLE;
  fsSaveFb();
}

static int8_t fsFbStartStage(uint8_t m) {
  switch (m) {
    case FS_MODE_WARM_B:    return 0;
    case FS_MODE_WARM_C:    return 1;
    case FS_MODE_WARM_CONT: return 3;
    case FS_MODE_CD_CONT:   return 4;
    case FS_MODE_CD1:       return 5;
    case FS_MODE_CD2:       return 6;
    case FS_MODE_CD_OFF:    return 7;
    default:                return -1;  // no rise seen
  }
}

static void fsLogFbStage(const char* why) {
  fsLogTs();
  Serial.print(F("[FALLBACK] "));
  Serial.print(why);
  if (fsFbStage < 0) {
    Serial.println(F(": no temperature rise seen -> stay idle"));
  } else {
    Serial.print(F(": stage "));
    Serial.print(fsFbStage);
    Serial.print(F(" = "));
    Serial.println(FS_FB_STAGES[fsFbStage].name);
  }
}

// Pick the temperatures to use (skipping failed sensors) and run the fallback state machine.
static void fsSelectTemps(float &tExt, float &tInt) {
  const bool extOk = !isnan(ar2ExtTemp) && !(ar2TFail & 1);
  const bool intOk = !isnan(ar2IntTemp) && !(ar2TFail & 2);
  const bool ext2Ok = !isnan(ar2Ext2Temp) && !(ar2TFail & 4);
  tInt = intOk ? ar2IntTemp : NAN;
  fsExtProxy = false;
  if (extOk) tExt = ar2ExtTemp;
  else if (ext2Ok) tExt = ar2Ext2Temp;
  else if (intOk && !isnan(fsExtIntOffset)) { tExt = ar2IntTemp + fsExtIntOffset; fsExtProxy = true; }
  else tExt = NAN;

  const unsigned long now = millis();
  const bool allFail = !extOk && !ext2Ok && !intOk;
  if (allFail) { fsAnyOkSinceMs = 0; if (fsAllFailSinceMs == 0) fsAllFailSinceMs = now; }
  else         { fsAllFailSinceMs = 0; if (fsAnyOkSinceMs == 0) fsAnyOkSinceMs = now; }

  if (!fsFbActive) {
    if (allFail && now - fsAllFailSinceMs >= fsFbEnterMs) {
      fsFbActive = true;
      fsFbStage = fsFbStartStage(fsLastTempMode);
      fsFbStageMs = now;
      fsSaveFb();
      fsLogFbStage("ALL temperature sensors failed");
    }
  } else if (!allFail && now - fsAnyOkSinceMs >= fsFbRecoverMs) {
    fsFbActive = false;
    fsFbStage = -1;
    fsSaveFb();
    fsLogTs();
    Serial.println(F("[FALLBACK] temperature sensors back -> normal control"));
  } else if (fsFbStage >= 0 && fsFbStage < 7 && now - fsFbStageMs >= fsFbStageIntMs) {
    fsFbStage++;
    fsFbStageMs = now;
    fsSaveFb();
    fsLogFbStage("next");
  }
  if (fsFbActive) { tExt = NAN; tInt = NAN; }  // stay on the timed stages until sensors are back for good
}

// Temperature-driven sprinkler mode for an outside temperature (no water-level rules).
static uint8_t fsTempModeFor(float tExt) {
  if (isnan(tExt)) return FS_MODE_IDLE;
  if (fsPhase == FS_PHASE_WARMUP) {
    if (tExt > FS_T_EXT_CONT_C) return FS_MODE_WARM_CONT;
    if (tExt >= FS_T_EXT_WARM_INT_LO_C && tExt < FS_T_EXT_WARM_B_HI_C) return FS_MODE_WARM_B;
    if (tExt >= FS_T_EXT_WARM_B_HI_C && tExt <= FS_T_EXT_WARM_C_HI_C) return FS_MODE_WARM_C;
    return FS_MODE_IDLE;
  }
  if (tExt > FS_T_EXT_CONT_C) return FS_MODE_CD_CONT;
  if (tExt > FS_T_EXT_CD1_LO_C) return FS_MODE_CD1;
  if (tExt > FS_T_EXT_CD2_LO_C) return FS_MODE_CD2;
  return FS_MODE_CD_OFF;
}

static float bestWaterLevel();  // tank level: gauge, or flow-meter estimate if the gauge has failed

// Build the high-level autonomous "plan" for this loop.
// This sets fsAutoOut: what should P0–P4 do, and what cycles apply.
static void fsComputeAutoPlan() {
  FsAutoOutputs out{};
  out.sprinklerCycle = { 0, 0 };
  out.shedCycle = { 0, 0 };
  out.sprinklerDrv = FS_SPRINKLER_NONE;
  uint8_t mode = FS_MODE_IDLE;

  const float wl = bestWaterLevel();  // gauge, or estimate from the flow meters if it has failed
  float tExt, tInt;
  fsSelectTemps(tExt, tInt);  // skips failed sensors; NAN for both while in fallback
  const float bat = ar2Bat;

  const bool haveWL = !isnan(wl);
  const bool haveExt = !isnan(tExt);
  const bool haveInt = !isnan(tInt);
  unsigned long now = millis();

  // Maintain TRISE and PHASE whenever we have data.
  if (haveExt) fsUpdateTrise(tExt);
  if (haveExt && haveInt) fsUpdatePhase(tExt, tInt);

  // Flow select (P3):
  //   HIGH whenever EXT is in the upper band (>80°C).
  //   (TRISE is still tracked but no longer gates P3.)
  out.flowHigh = (haveExt && fsPhase == FS_PHASE_WARMUP && tExt > FS_T_EXT_CONT_C);

  // 1) Water level bands: hard priority.
  if (haveWL && wl <= FS_WL_DRY_M) {
    // Dry cutoff: everything off.
    fsModeCode = FS_MODE_DRY_CUTOFF;
    fsAutoOut = out;
    return;
  }

  // All temperature sensors just failed, not yet confirmed: keep the current plan rather
  // than stopping the sprinklers while the fallback timer runs.
  if (!fsFbActive && fsAllFailSinceMs != 0) return;

  if (haveWL && wl > FS_WL_DRY_M && wl < FS_WL_GEL_M) {
    if (mode == FS_MODE_IDLE) mode = FS_MODE_GEL_PRIORITY;
    // Gel priority band: P0 ON, gel ON, low flow; shed cooling still allowed.
    //
    // We no longer return here so that shed thermostat logic (and any other
    // non-conflicting fire logic) can continue to run while gel is injected.
    out.pumpContinuous = true;
    out.gelOn = true;
    out.flowHigh = false;
  }

  bool inRecircBand = false;

  if (haveWL && wl >= FS_WL_GEL_M && wl <= FS_WL_RECIRC_HI) {
    if (mode == FS_MODE_IDLE) mode = FS_MODE_RECIRC_BAND;
    // Recirculating conservation: fixed sprinkler cycle [3,8], mapped by battery.
    inRecircBand = true;
    out.sprinklerIntermittent = true;
    out.sprinklerCycle = FS_CYCLE_RECIRC_BAND;
  }

  // 2) Shed cooling (P2) from internal temperature — thermostat mode.
  //
  //   Global instantaneous rule:
  //     INT > FS_T_INT_SHED_CONT_C (60°C) → P2 continuous ON,
  //       and latch thermostat into continuous.
  //
  //   Thermostat mode (sub-threshold region):
  //     - When INT rises to >= FS_T_INT_SHED_THERM_C (47°C) after having
  //       cooled below FS_T_INT_SHED_THERM_RESET_C (45°C), start a 1-minute
  //       shed pulse (P2 ON).
  //     - If the time between the start of the current pulse and the start
  //       of the previous pulse is < FS_SHED_THERM_MIN_GAP_MS (2 min),
  //       escalate to continuous ON.
  //     - While a thermostat pulse is active, P2 is driven via shedContinuous.
  //
  //   INT below 47°C (and no active pulse) → P2 OFF (subject to water band
  //   overrides handled earlier, e.g. dry cutoff disabling all outputs).
  if (haveInt) {
    // Critical over-temperature: force continuous ON and latch thermostat.
    if (tInt > FS_T_INT_SHED_CONT_C) {
      out.shedContinuous = true;
      fsShedThermContinuous = true;
      fsShedThermPulseActive = false;
    } else {
      // If we are already latched continuous from thermostat, keep P2 ON.
      if (fsShedThermContinuous) {
        out.shedContinuous = true;
      } else {
        // Re-arm thermostat once sufficiently cooled below reset threshold.
        if (!fsShedThermArmed && tInt <= FS_T_INT_SHED_THERM_RESET_C) {
          fsShedThermArmed = true;
        }

        if (fsShedThermPulseActive) {
          // Maintain a single 1-minute thermostat pulse window.
          if (now - fsShedThermPulseStartMs >= FS_SHED_THERM_PULSE_MS) {
            fsShedThermPulseActive = false;
          } else {
            out.shedContinuous = true;
          }
        } else {
          // No active pulse: check for new thermostat trigger at INT ≥ 47°C.
          if (fsShedThermArmed && tInt >= FS_T_INT_SHED_THERM_C) {
            unsigned long gapMs;
            if (fsShedThermLastStartMs == 0) {
              // First pulse: never escalate based on gap.
              gapMs = FS_SHED_THERM_MIN_GAP_MS + 1;
            } else {
              gapMs = now - fsShedThermLastStartMs;
            }

            if (fsShedThermLastStartMs != 0 && gapMs < FS_SHED_THERM_MIN_GAP_MS) {
              // Escalate: pulses are less than 2 minutes apart → go continuous.
              fsShedThermContinuous = true;
              out.shedContinuous = true;
            } else {
              // Start a new 1-minute thermostat pulse.
              fsShedThermPulseActive = true;
              fsShedThermArmed = false;
              fsShedThermPulseStartMs = now;
              fsShedThermLastStartMs = now;
              out.shedContinuous = true;
            }
          }
        }
      }
    }
  }

  // 3) External temperature → sprinkler behaviour (only when not in gel band).
  const uint8_t tempMode = fsTempModeFor(tExt);
  if (haveExt) fsLastTempMode = tempMode;  // remembered as the fallback start point
  if (haveWL && wl > FS_WL_RECIRC_HI && haveExt) {
    switch (tempMode) {
      case FS_MODE_WARM_CONT:  // warm-up, EXT > 80 → continuous (high flow, set above)
      case FS_MODE_CD_CONT:    // cool-down, EXT still > 80 → continuous reduced flow
        out.pumpContinuous = true;
        break;
      case FS_MODE_WARM_B: out.sprinklerIntermittent = true; out.sprinklerCycle = FS_CYCLE_WARM_B; break;  // [3,7]
      case FS_MODE_WARM_C: out.sprinklerIntermittent = true; out.sprinklerCycle = FS_CYCLE_WARM_C; break;  // [5,3]
      case FS_MODE_CD1:    out.sprinklerIntermittent = true; out.sprinklerCycle = FS_CYCLE_CD1;    break;  // [3,5]
      case FS_MODE_CD2:    out.sprinklerIntermittent = true; out.sprinklerCycle = FS_CYCLE_CD2;    break;  // [3,8]
      default: break;      // IDLE (warm-up < 60) or CD_OFF (cool-down <= 60): no sprinklers
    }
    if (mode == FS_MODE_IDLE) mode = tempMode;
  }

  // 3b) All temperature sensors failed: timed fallback stage drives sprinklers and shed cooling.
  if (fsFbActive) {
    if (fsFbStage >= 0) {
      const FsFbStage &st = FS_FB_STAGES[fsFbStage];
      if (st.cont) out.shedContinuous = true;
      else { out.shedIntermittent = true; out.shedCycle = { st.on, st.off }; }
      if (haveWL && wl > FS_WL_RECIRC_HI) {
        if (st.cont) {
          out.pumpContinuous = true;
          out.flowHigh = st.high;
        } else {
          out.sprinklerIntermittent = true;
          out.sprinklerCycle = { st.on, st.off };
        }
      }
    }
    if (mode == FS_MODE_IDLE) mode = FS_MODE_FALLBACK;
  }

  // In the [0.30,0.50] m band, we *always* down-rate to the recirc cycle,
  // so any continuous request from EXT is ignored.
  if (inRecircBand) {
    out.pumpContinuous = false;
  }

  // 4) Battery rule → map intermittent sprinkler cycles to P0 or P1.
  out.sprinklerDrv = FS_SPRINKLER_NONE;
  if (out.sprinklerIntermittent && (out.sprinklerCycle.on > 0 || out.sprinklerCycle.off > 0)) {
    float vb = bat;
    if (!isnan(vb) && vb <= FS_BAT_LOW_V) {
      // <= 12V → always use P1 (recirc) for intermittent.
      out.sprinklerDrv = FS_SPRINKLER_RECIRC_P1;
    } else {
      // >12V or unknown → choose by Toff (minutes).
      uint8_t offMin = out.sprinklerCycle.off;
      if (offMin > 15) {
        out.sprinklerDrv = FS_SPRINKLER_PUMP_P0;
      } else {
        out.sprinklerDrv = FS_SPRINKLER_RECIRC_P1;
      }
    }
  }

  // Continuous sprinklers always use P0 (pumpContinuous flag).
  // We do not currently down-rate continuous modes by battery.

  fsAutoOut = out;
  fsModeCode = mode;
}

// Turn fsAutoOut into actual pump/valve states, handling the
// intermittent timers and the P0 latch relay.
static void fsApplyAutoOutputs() {
  unsigned long now = millis();

  // Desired instantaneous states for this loop.
  bool wantPumpOn = false;  // P0 (pump)
  bool wantP1On = false;    // P1 energised = recirculate to tank (off = to sprinklers)
  bool wantP2On = false;    // shed
  bool wantP3On = fsAutoOut.flowHigh && !fsAutoOut.gelOn;  // gel needs reduced flow (Dosatron)
  bool wantP4On = fsAutoOut.gelOn;

  // Continuous sprinklers drive P0.
  if (fsAutoOut.pumpContinuous) {
    wantPumpOn = true;
  }

  // Intermittent sprinkler cycle (P0 or P1 depending on sprinklerDrv).
  if (fsAutoOut.sprinklerIntermittent && fsAutoOut.sprinklerDrv != FS_SPRINKLER_NONE && (fsAutoOut.sprinklerCycle.on > 0 || fsAutoOut.sprinklerCycle.off > 0)) {

    const unsigned long onMs = (unsigned long)fsAutoOut.sprinklerCycle.on * 60000UL;
    const unsigned long offMs = (unsigned long)fsAutoOut.sprinklerCycle.off * 60000UL;

    if (fsSprPhaseMs == 0) {
      fsSprPhaseMs = now;
      fsSprOnPhase = true;
    }

    unsigned long phaseDur = fsSprOnPhase ? onMs : offMs;
    if (phaseDur == 0) {
      fsSprOnPhase = false;  // degenerate: treat as OFF
    } else if (now - fsSprPhaseMs >= phaseDur) {
      fsSprOnPhase = !fsSprOnPhase;
      fsSprPhaseMs = now;
      // B) Log sprinkler cycle flip.
      fsLogTs();
      Serial.print(F("[AUTO] Spr -> "));
      if (fsSprOnPhase) {
        Serial.print(F("ON  ("));
        Serial.print(fsAutoOut.sprinklerCycle.on);
        Serial.println(F("min)"));
      } else {
        Serial.print(F("OFF ("));
        Serial.print(fsAutoOut.sprinklerCycle.off);
        Serial.println(F("min)"));
      }
    }

    bool sprActive = fsSprOnPhase && (onMs > 0);

    if (fsAutoOut.sprinklerDrv == FS_SPRINKLER_PUMP_P0) {
      // Intermittent driven directly by pump (P0).
      if (sprActive) wantPumpOn = true;
    } else if (fsAutoOut.sprinklerDrv == FS_SPRINKLER_RECIRC_P1) {
      // Intermittent via the recirculation valve: the pump runs throughout. P1 de-energised
      // = water to the sprinklers (ON phase); P1 energised = recirculate to the tank (OFF
      // phase). Until 2026-11-01 this energised P1 in the ON phase and stopped the pump in
      // the OFF phase, so these cycles never reached the sprinklers.
      wantPumpOn = true;
      if (!sprActive) wantP1On = true;
    }
  } else {
    // No intermittent sprinkler request: reset the FSM.
    fsSprPhaseMs = 0;
    fsSprOnPhase = false;
  }

  // Shed cooling on P2: continuous or intermittent.
  if (fsAutoOut.shedContinuous) {
    wantP2On = true;
  } else if (fsAutoOut.shedIntermittent && (fsAutoOut.shedCycle.on > 0 || fsAutoOut.shedCycle.off > 0)) {
    const unsigned long onMs = (unsigned long)fsAutoOut.shedCycle.on * 60000UL;
    const unsigned long offMs = (unsigned long)fsAutoOut.shedCycle.off * 60000UL;

    if (fsShedPhaseMs == 0) {
      fsShedPhaseMs = now;
      fsShedOnPhase = true;
    }

    unsigned long phaseDur = fsShedOnPhase ? onMs : offMs;
    if (phaseDur == 0) {
      fsShedOnPhase = false;
    } else if (now - fsShedPhaseMs >= phaseDur) {
      fsShedOnPhase = !fsShedOnPhase;
      fsShedPhaseMs = now;
    }

    bool shedActive = fsShedOnPhase && (onMs > 0);
    if (shedActive) wantP2On = true;
  } else {
    fsShedPhaseMs = 0;
    fsShedOnPhase = false;
  }

  // Apply pump latch relay (P0).
  if (wantPumpOn && !pumpState) {
    applyPumpEdge(1, -1);
  } else if (!wantPumpOn && pumpState) {
    applyPumpEdge(0, -1);
  }

  // Apply P1–P4 (active-low outputs).
  digitalWrite(PIN_P1, wantP1On ? LOW : HIGH);
  digitalWrite(PIN_P2, wantP2On ? LOW : HIGH);
  digitalWrite(PIN_P3, wantP3On ? LOW : HIGH);
  digitalWrite(PIN_P4, wantP4On ? LOW : HIGH);
}



// ---------------- Manual-mode intermittent config from AR1 (HA dial) ----------------
// NOTE: These are only for Manual/Remote semantics. Autonomous modes in TAR3
// continue to use baked-in on/off timings and ignore these.
static unsigned long manIntOnMs = 0;
static unsigned long manIntOffMs = 0;

// ---------------- TEST system ----------------
static bool testActive = false;
static bool ignoreHbInTest = false;
// HA stop for a button test: heartbeat seen OFF, then ON in 2 reports in a row.
static bool testHbOffSeen = false;
static uint8_t testHbOnCount = 0;
static bool testStopByHa = false;
// Scenario finished: AR2 sent SIM:1 during this test, then TEST_SIM_END_PKTS real packets in a row.
static const uint8_t TEST_SIM_END_PKTS = 10;  // ~1.5 s at AR2's ~7 packets/s
static bool testSimSeen = false;
static uint8_t testRealPkts = 0;
static bool testScenDone = false;
static void testHbSample(int hb) {
  if (!testActive || hb < 0) return;
  if (hb == 0) { testHbOffSeen = true; testHbOnCount = 0; return; }
  if (testHbOffSeen && testHbOnCount < 2 && ++testHbOnCount >= 2) testStopByHa = true;
}
static bool testAwaitUdp = false;
static unsigned long testStartMs = 0;
static const unsigned long TEST_WARMUP_MS = 1200;
static uint8_t testScenarioId = 1;
static float testSpeed = 1.00f;

// AR3B: debounced hold-to-start test button + test-mode timeout.
static const unsigned long BTN_DEBOUNCE_MS   = 50;        // level must be stable this long
static const unsigned long BTN_START_HOLD_MS = 3000;      // hold to START a test
static const unsigned long TEST_MAX_MS       = 1800000UL; // auto-END a test after 30 min
static bool btnRawPrev = true;          // last raw reading (HIGH = released)
static unsigned long btnRawChangeMs = 0;
static bool btnStable = true;           // debounced level (HIGH = released)
static unsigned long btnDownMs = 0;     // debounced press start
static bool btnPressConsumed = false;   // this press already acted on

// ---------------- Utility helpers ----------------
static inline void fsLogTs() {
  Serial.print('[');
  Serial.print(millis());
  Serial.print("] ");
}

static const char* fsModeCodeName(uint8_t code) {
  switch (code) {
    case FS_MODE_IDLE:         return "IDLE";
    case FS_MODE_DRY_CUTOFF:   return "DRY_CUTOFF";
    case FS_MODE_GEL_PRIORITY: return "GEL_PRIORITY";
    case FS_MODE_RECIRC_BAND:  return "RECIRC_BAND";
    case FS_MODE_WARM_CONT:    return "WARM_CONT";
    case FS_MODE_WARM_B:       return "WARM_B";
    case FS_MODE_WARM_C:       return "WARM_C";
    case FS_MODE_CD1:          return "CD1";
    case FS_MODE_CD2:          return "CD2";
    case FS_MODE_CD_OFF:       return "CD_OFF";
    case FS_MODE_CD_CONT:      return "CD_CONT";
    case FS_MODE_FALLBACK:     return "FALLBACK";
    default:                   return "?";
  }
}

static uint8_t fsLastLoggedMode = 0xFF;  // sentinel: force log on first AUTO tick

static inline void coilPulse(uint8_t pin, bool requestOn) {
  // FireSys: real latch relay coils → keep existing pulse semantics
  if (!JF_isTestSys) {
    digitalWrite(pin, LOW);
    delay(COIL_PULSE_MS);
    digitalWrite(pin, HIGH);
    return;
  }

  // TestSys: no latch relay → mimic latch behaviour on D2/D3 as LEDs
  //
  // We still give a short "activation" pulse on the requested pin for
  // testing, then leave the pins latched to represent pump ON/OFF:
  //
  //  - requestOn == true  → pump ON  → D3 (PIN_COIL_OFF) LED ON, D2 OFF
  //  - requestOn == false → pump OFF → D2 (PIN_COIL_ON) LED ON, D3 OFF
  //
  digitalWrite(pin, LOW);
  delay(COIL_PULSE_MS);

  if (requestOn) {
    // Pump ON: ON-side coil line active, OFF-side idle
    digitalWrite(PIN_COIL_OFF, LOW);  // D3 "ON" (active-low LED)
    digitalWrite(PIN_COIL_ON, HIGH);  // D2 "OFF"
  } else {
    // Pump OFF: OFF-side coil line active, ON-side idle
    digitalWrite(PIN_COIL_OFF, HIGH);  // D3 "OFF"
    digitalWrite(PIN_COIL_ON, LOW);    // D2 "ON" (pump off indicator)
  }
}

static inline void setP1234(int p1, int p2, int p3, int p4) {
  // LOCK policy: force P1/P3/P4 OFF, allow P2.
  if (remoteLockActive) {
    p1 = 0;
    p3 = 0;
    p4 = 0;
  }
  if (p4) p3 = 0;  // gel needs reduced flow: the constriction drives the Dosatron

  digitalWrite(PIN_P1, p1 ? LOW : HIGH);
  digitalWrite(PIN_P2, p2 ? LOW : HIGH);
  digitalWrite(PIN_P3, p3 ? LOW : HIGH);
  digitalWrite(PIN_P4, p4 ? LOW : HIGH);
}

static inline int rdP(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

static int readFieldInt(const String &src, const String &key, int defVal) {
  int idx = src.indexOf(key + ":");
  if (idx < 0) return defVal;
  idx += key.length() + 1;
  int end = idx;
  while (end < (int)src.length() && (isDigit(src[end]) || src[end] == '-' || src[end] == '+')) end++;
  String v = src.substring(idx, end);
  if (!v.length()) return defVal;
  return v.toInt();
}

static float readFieldFloat(const String &src, const String &key, float defVal) {
  int idx = src.indexOf(key + ":");
  if (idx < 0) return defVal;
  idx += key.length() + 1;
  int end = idx;
  while (end < (int)src.length() && (isDigit(src[end]) || src[end] == '.' || src[end] == '-' || src[end] == '+')) end++;
  String v = src.substring(idx, end);
  if (!v.length()) return defVal;
  return v.toFloat();
}

// ---------------- RS-485 frame ingest ----------------
static String frameBuf;
static bool frameOpen = false;

// Separate state for swSerial path — prevents RS485 and swSerial frames from
// corrupting each other when both streams are active simultaneously.
static String swFrameBuf;
static bool swFrameOpen = false;

static void handleFrame(const String &frame);

static void feedByte(uint8_t b) {
  char c = (char)b;
  // Resync unconditionally on '<', even mid-frame — a single dropped/corrupted
  // byte can otherwise strand frameOpen=true indefinitely, silently swallowing
  // every subsequent real frame's '<' as payload data until a stray '>' shows
  // up. Matches AR1's RS485Receive() resync behaviour (T44, 2026-07-15).
  if (c == '<') {
    frameOpen = true;
    frameBuf = "";
    return;
  }
  if (!frameOpen) {
    return;
  }
  if (c == '>') {
    frameOpen = false;
    String f = frameBuf;
    frameBuf = "";
    f.trim();
    if (f.length()) {
      lastRs485RxMs = millis();  // RS-485 link alive — suppress swSerial TX
      if (frameCheck(f, false)) handleFrame(f);
    }
    return;
  }
  frameBuf += c;
}

static void feedByteSW(uint8_t b) {
  char c = (char)b;
  // Same unconditional resync-on-'<' fix as feedByte() (T44, 2026-07-15).
  if (c == '<') {
    swFrameOpen = true;
    swFrameBuf = "";
    return;
  }
  if (!swFrameOpen) {
    return;
  }
  if (c == '>') {
    swFrameOpen = false;
    String f = swFrameBuf;
    swFrameBuf = "";
    f.trim();
    if (f.length()) {
      if (frameCheck(f, true)) handleFrame(f);
    }
    return;
  }
  swFrameBuf += c;
}

// ---------------- LCD helpers ----------------
static inline char modeChar() {
  return (fsMode == FS_MODE_AUTONOMOUS) ? 'A' : 'R';
}

static inline bool hbFresh() {
  return (lastHbMs != 0) && (millis() - lastHbMs < HB_FRESH_MS);
}

static inline bool hbWarn() {
  if (lastHbMs == 0) return true;
  unsigned long age = millis() - lastHbMs;
  return (age >= HB_WARN_MS) && (age < HB_ARM_MS);
}

static inline char pinOnChar(uint8_t pin) {
  return digitalRead(pin) == LOW ? '1' : '0';
}

static void drawPage1() {
  lcd.setCursor(0, 0);
  lcd.print("LVL ");
  lcd.print(bestWaterLevel(), 2);
  lcd.print("m M:");
  lcd.print(modeChar());

  lcd.setCursor(0, 1);
  if (fsMode == FS_MODE_AUTONOMOUS) {
    lcd.print("AUTO ON  HB:");
    lcd.print(hbFresh() ? '1' : '0');
  } else {
    if (hbWarn()) {
      lcd.print("REMOTE  HB:WARN");
    } else {
      lcd.print("REMOTE   HB:");
      lcd.print(hbFresh() ? '1' : '0');
    }
  }
}

static void drawPage2() {
  lcd.setCursor(0, 0);
  lcd.print("HEARTBEAT");
  lcd.setCursor(0, 1);
  lcd.print(hbFresh() ? "   ON" : "  OFF");
}

static void drawPage3() {
  lcd.setCursor(0, 0);
  lcd.print("P=");
  lcd.print(pumpState ? '1' : '0');
  lcd.print(" HF=");
  lcd.print(pinOnChar(PIN_P3));
  lcd.print(" SHD=");
  lcd.print(pinOnChar(PIN_P2));

  lcd.setCursor(0, 1);
  lcd.print("REC=");
  lcd.print(pinOnChar(PIN_P1));
  lcd.print(" GEL=");
  lcd.print(pinOnChar(PIN_P4));
}

static void tickLCD() {
  // No LCD on TestSys: skip all I²C LCD traffic
  if (!fsHasLcd()) return;

  unsigned long now = millis();
  if (now - lcdLastMs < LCD_PERIOD_MS) return;
  lcdLastMs = now;
  lcdPage++;
  if (lcdPage > 3) lcdPage = 1;

  lcd.clear();
  switch (lcdPage) {
    case 1: drawPage1(); break;
    case 2: drawPage2(); break;
    case 3: drawPage3(); break;
  }
}

// Select latch coil pin from the requested logical state.
// FS_POLARITY_INVERT swaps the coil mapping.
static inline uint8_t jfPumpCoilPinForRequest(int requestOn) {
  const bool wantOn = (requestOn != 0);
  uint8_t pin = wantOn ? PIN_COIL_ON : PIN_COIL_OFF;
  if (FS_POLARITY_INVERT) {
    pin = wantOn ? PIN_COIL_OFF : PIN_COIL_ON;
  }
  return pin;
}

// ---------------- Option B: engine start sequencer (replaces SFS01-V2) ----------------
// Fuel/RUN is held by the pump latch relay itself (follows pumpState, survives resets).
// This sequencer only drives the starter (PIN_CRANK → AutoRelay pin 86).
// Starts on a pumpState OFF->ON edge, whatever the source (HA/REMOTE or AUTO).
// Logic interlock: the crank output may be ON only in SS_CRANK, with the pump
// demanded, AR2 data fresh and oil pressure absent; hard cap per attempt.
// A reset/power loss drops the (non-latching) crank relay; WDT (5 s) bounds a hang.
static const unsigned long SS_PREFUEL_MS     = 1000;   // fuel solenoid pull-in before cranking
static const unsigned long SS_CRANK_MAX_MS   = 5000;   // max crank, attempts 2-5 (end early on oil pressure)
// Attempt 1 ignores oil pressure (per Graham Lund, SFS: the oil switch often opens
// while cranking before the engine has actually fired). It runs for this fixed time,
// kept short (as the SFS box's 2 s first crank) so a quick start is not over-cranked.
static const unsigned long SS_FIRST_CRANK_MS = 2000;
static const unsigned long SS_REST_MS        = 20000;  // rest between attempts
static const uint8_t       SS_MAX_ATTEMPTS   = 5;
static const unsigned long SS_OIL_CONFIRM_MS = 300;    // oil OK held this long = engine fired
static const unsigned long SS_RUN_CONFIRM_MS = 5000;   // after crank, oil must stay OK this long
static const unsigned long SS_OIL_DROP_MS    = 1000;   // oil lost this long in confirm = failed attempt
static const unsigned long SS_OIL_LOST_MS    = 2000;   // oil lost this long while running = stall
static const unsigned long SS_AR2_FRESH_MS   = 1500;   // AR2 UDP (10 Hz with AR2B) must be newer than this;
                                                       // margin for lost WiFi broadcast packets
static const unsigned long SS_BOOT_HOLD_MS   = 20000;  // never crank in the first 20 s after boot
static const unsigned long SS_NODATA_FAIL_MS = 60000;  // give up if AR2 data never becomes fresh

enum SsState : uint8_t {
  SS_IDLE = 0,   // pump OFF
  SS_PREFUEL,    // pump ON, fuel on, waiting to crank
  SS_CRANK,      // starter energised
  SS_CONFIRM,    // crank ended on oil pressure; confirming engine keeps running
  SS_REST,       // between attempts
  SS_RUNNING,    // engine running
  SS_FAILED,     // attempts exhausted or no AR2 data; no more cranking until pump OFF
  SS_STALLED     // oil pressure lost after a good start; report only, no restart
};
static SsState ssState = SS_IDLE;
static unsigned long ssStateMs = 0;
static uint8_t ssAttempt = 0;
static bool ssPrevPump = false;
static bool ssOilPrev = false;
static unsigned long ssOilChangeMs = 0;
static bool ssCrankOn = false;

static inline bool ssAr2Fresh() {
  return lastUdpMs != 0 && (millis() - lastUdpMs) < SS_AR2_FRESH_MS;
}

static inline void ssCrank(bool on) {
  if (on == ssCrankOn) return;
  ssCrankOn = on;
  digitalWrite(PIN_CRANK, on ? LOW : HIGH);  // active LOW
  fsLogTs();
  Serial.println(on ? F("[START] CRANK ON") : F("[START] CRANK OFF"));
}

static inline void crankForceOff() {
  ssCrank(false);
}

// Takes uint8_t (not SsState): the Arduino builder's auto-generated prototypes
// are placed above the enum, so an enum parameter type would not compile.
static void ssEnter(uint8_t s) {
  ssState = (SsState)s;
  ssStateMs = millis();
  fsLogTs();
  Serial.print(F("[START] state="));
  Serial.print((int)s);
  Serial.print(F(" attempt="));
  Serial.println(ssAttempt);
}

static bool ssBypassMode = false;          // debounced D9 state (true = BYPASS)
static bool ssModeRaw = false;
static unsigned long ssModeRawMs = 0;
static const unsigned long SS_MODE_DEBOUNCE_MS = 200;

static void tickStartSeq() {
  const unsigned long now = millis();

  // Mode input D9 (debounced). Not BYPASS = sequencer idle, crank forced off.
  const bool modeRawNow = (digitalRead(PIN_MODE_BYPASS) == LOW);
  if (modeRawNow != ssModeRaw) {
    ssModeRaw = modeRawNow;
    ssModeRawMs = now;
  }
  if (ssModeRaw != ssBypassMode && now - ssModeRawMs >= SS_MODE_DEBOUNCE_MS) {
    ssBypassMode = ssModeRaw;
    fsLogTs();
    Serial.println(ssBypassMode ? F("[START] mode = BYPASS (ARMAC starts pump)")
                                : F("[START] mode = BOX/disabled (sequencer idle)"));
  }
  if (!ssBypassMode) {
    crankForceOff();
    if (ssState != SS_IDLE) ssEnter(SS_IDLE);
    ssPrevPump = pumpState;  // no start edge is "remembered" across a mode change
    return;
  }

  // Track how long the oil state has been held (for confirm / drop / stall timing).
  if (ar2OilOk != ssOilPrev) {
    ssOilPrev = ar2OilOk;
    ssOilChangeMs = now;
  }
  const unsigned long oilHeldMs = now - ssOilChangeMs;
  const bool fresh = ssAr2Fresh();
  const bool oilConfirmed = fresh && ar2OilOk && oilHeldMs >= SS_OIL_CONFIRM_MS;

  const bool edgeOn = pumpState && !ssPrevPump;
  ssPrevPump = pumpState;

  // Pump OFF always wins.
  if (!pumpState) {
    crankForceOff();
    if (ssState != SS_IDLE) ssEnter(SS_IDLE);
    return;
  }

  switch (ssState) {
    case SS_IDLE:
      if (edgeOn) {
        ssAttempt = 0;
        ssEnter(SS_PREFUEL);
      }
      break;

    case SS_PREFUEL:
      if (now - ssStateMs < SS_PREFUEL_MS || now - bootMs < SS_BOOT_HOLD_MS) break;
      if (!fresh) {
        if (now - ssStateMs >= SS_NODATA_FAIL_MS) {
          fsLogTs();
          Serial.println(F("[START] no fresh AR2 data - giving up"));
          ssEnter(SS_FAILED);
        }
        break;  // never crank blind
      }
      if (oilConfirmed) {
        ssEnter(SS_RUNNING);  // already running (e.g. key-started): no crank
      } else if (!ar2OilOk) {
        ssAttempt++;
        ssEnter(SS_CRANK);
        ssCrank(true);
      }
      break;

    case SS_CRANK:
      if (!fresh) {
        crankForceOff();
        fsLogTs();
        Serial.println(F("[START] AR2 data stale - crank aborted"));
        ssEnter(SS_REST);
      } else if (ssAttempt == 1) {
        // First attempt: oil pressure is NOT used to end the crank (it can read
        // OK before the engine fires). Fixed-length crank, then judge the result.
        if (now - ssStateMs >= SS_FIRST_CRANK_MS) {
          crankForceOff();
          ssEnter(SS_CONFIRM);  // CONFIRM decides: running, or rest and retry
        }
      } else if (oilConfirmed) {
        crankForceOff();
        ssEnter(SS_CONFIRM);
      } else if (now - ssStateMs >= SS_CRANK_MAX_MS) {
        crankForceOff();
        ssEnter(SS_REST);
      }
      break;

    case SS_CONFIRM:
      if (!ar2OilOk && oilHeldMs >= SS_OIL_DROP_MS) {
        ssEnter(SS_REST);  // fired briefly but did not keep running
      } else if (now - ssStateMs >= SS_RUN_CONFIRM_MS) {
        ssEnter(SS_RUNNING);
      }
      break;

    case SS_REST:
      if (oilConfirmed && oilHeldMs >= SS_RUN_CONFIRM_MS) {
        ssEnter(SS_RUNNING);  // caught late on the previous attempt
      } else if (now - ssStateMs >= SS_REST_MS) {
        if (ssAttempt >= SS_MAX_ATTEMPTS) {
          fsLogTs();
          Serial.println(F("[START] FAILED - attempts exhausted"));
          ssEnter(SS_FAILED);
        } else {
          ssEnter(SS_PREFUEL);
        }
      }
      break;

    case SS_RUNNING:
      if (!ar2OilOk && oilHeldMs >= SS_OIL_LOST_MS) {
        fsLogTs();
        Serial.println(F("[START] STALLED - oil pressure lost (report only)"));
        ssEnter(SS_STALLED);
      }
      break;

    case SS_FAILED:
    case SS_STALLED:
      break;  // no action until pump OFF
  }

  // Logic interlock: whatever happened above, the crank may only be ON in SS_CRANK
  // with fresh AR2 data, and (from attempt 2 on) with no oil pressure. Attempt 1 is
  // time-limited to SS_FIRST_CRANK_MS instead of being cut by oil pressure.
  if (ssCrankOn && !(ssState == SS_CRANK && fresh && (ssAttempt == 1 || !ar2OilOk))) {
    crankForceOff();
  }
}

// Latch state from read-back (FireSys). TestSys has no latch hardware: use pumpState.
static inline bool latchIsOn() {
  if (JF_isTestSys || !LATCH_FB_FITTED) return pumpState;
  return digitalRead(PIN_LATCH_FB) == LOW;
}

// Pulse the latch only if it is not already in the requested state. Prevents the
// "interlock + shutdown" module toggling OFF on a repeated ON pulse (e.g. the first
// command after an AR3 reboot while the pump is running).
static void pumpPulseIfNeeded(int requestOn) {
  const bool want = (requestOn != 0);
  if (!LATCH_FB_FITTED || JF_isTestSys) {  // no read-back: always pulse
    coilPulse(jfPumpCoilPinForRequest(requestOn), want);
    return;
  }
  if (latchIsOn() == want) {
    fsLogTs();
    Serial.println(want ? F("[LATCH] already ON - no pulse") : F("[LATCH] already OFF - no pulse"));
    return;
  }
  coilPulse(jfPumpCoilPinForRequest(requestOn), want);
  if (latchIsOn() != want) {
    fsLogTs();
    Serial.println(F("[LATCH] WARNING: read-back does not match request after pulse"));
  }
}

static void applyPumpEdgeRaw(int requestOn, int seqVal) {
  // Option B: drop the starter immediately on any OFF request (before the
  // blocking coil pulse below), rather than waiting for the next tickStartSeq().
  if (requestOn == 0) crankForceOff();

  // "phys" represents the physical coil mapping domain (not the logical state).
  // Keep it for compatibility with existing timing logic, but do NOT use it to set pumpState.
  const int phys = FS_POLARITY_INVERT ? (requestOn ? 0 : 1) : (requestOn ? 1 : 0);

  if (!havePumpState) {
    // First command after boot:
    //  - Define logical pumpState from the REQUEST (not from phys)
    //  - Pulse the correct coil based on request + invert mapping
    havePumpState = true;
    pumpState = (requestOn != 0);
    lastChangeMs = millis();
    if (seqVal >= 0) lastSeqApplied = seqVal;

    pumpPulseIfNeeded(requestOn);  // read-back guarded (no toggle after reboot)

    cmdIgnoreUntilMs = millis() + CMD_SILENCE_MS;

    // T44 (2026-07-15): send the ACK here too — previously this branch never
    // acked, so AR1's fast-ack-retry kept resending the same first command
    // every ~95ms, and every retry landed inside the CMD_SILENCE_MS window
    // above and got dropped by the "silence" guard before ever reaching the
    // duplicate-SEQ ACK-resend logic further down. Net effect: a real
    // 400ms-ish command/ACK stall on every boot.
    if (seqVal >= 0) {
      String ack = "ACK:SEQ:" + String(seqVal & 0x0F) + "|PUMP:" + (pumpState ? "1" : "0");
      sendLineOnBothPaths(ack);
    }
    return;
  }

  unsigned long now = millis();

  if (now < cmdIgnoreUntilMs) {
    fsLogTs();
    Serial.println("[CMD] drop: silence");
    return;
  }

  if (seqVal >= 0 && lastSeqApplied >= 0 && ((seqVal & 0x0F) == (lastSeqApplied & 0x0F))) {
    String ack = "ACK:SEQ:" + String(seqVal & 0x0F) + "|PUMP:" + (pumpState ? "1" : "0");
    sendLineOnBothPaths(ack);
    return;
  }

  bool sameDir = (requestOn != 0) == pumpState;
  if (sameDir) {
    if (now - lastSameDirMs < SAME_DIR_MIN_MS) {
      fsLogTs();
      Serial.println("[CMD] drop: same-dir min");
      return;
    }
    lastSameDirMs = now;
    if (seqVal >= 0) lastSeqApplied = seqVal;
    String ack = "ACK:SEQ:" + String(seqVal & 0x0F) + "|PUMP:" + (pumpState ? "1" : "0");
    sendLineOnBothPaths(ack);
    return;
  }

  if (now - lastChangeMs < EDGE_GUARD_OPP_MS) {
    fsLogTs();
    Serial.println("[CMD] drop: opp guard");
    return;
  }

  if (seqVal >= 0) lastSeqApplied = seqVal;

  // Pulse the correct coil for the requested state (respecting invert mapping),
  // only if the latch read-back says it is not already there.
  pumpPulseIfNeeded(requestOn);

  pumpState = (requestOn != 0);
  lastChangeMs = now;
  cmdIgnoreUntilMs = now + CMD_SILENCE_MS;

  // ACK the change straight away (4 Nov 2026). Before, an applied pump change was only
  // ACKed when AR1's retry arrived as a duplicate SEQ, so every pump change went twice.
  // Dropped commands (silence / opposite-direction guard) still get no ACK, so AR1 retries.
  if (seqVal >= 0) {
    String ack = "ACK:SEQ:" + String(seqVal & 0x0F) + "|PUMP:" + (pumpState ? "1" : "0");
    sendLineOnBothPaths(ack);
  }
}

static void applyPumpEdge(int requestOn, int seqVal) {
  // LOCK policy: pump must never be turned ON while locked.
  if (remoteLockActive && !fsInAuto) {
    requestOn = 0;
  }
  applyPumpEdgeRaw(requestOn, seqVal);
}

// ---------------- RS-485 frame handler ----------------
static void handleFrame(const String &frame) {
  if (frame.startsWith("HB:")) {
    int v = readFieldInt(frame, "HB", 0);
    testHbSample(v);
    if (ignoreHbInTest) return;
    if (v > 0) lastHbMs = millis();
    fsLogTs();
    Serial.print("[HB] ");
    Serial.println(frame);
    return;
  }

  if (frame.startsWith("STAT:")) {
    // Existing HA auto flags + heartbeat
    // IMPORTANT: if AR1 does not include HA_AUTO / HA_RESUME fields, default them to 0
    // so we do not "stick" in AUTO from an old value.
    haAutoEnable = (readFieldInt(frame, "HA_AUTO", 0) != 0);
    haAutoResume = (readFieldInt(frame, "HA_RESUME", 0) != 0);
    int hb = readFieldInt(frame, "HB", -1);
    testHbSample(hb);

    // Backward-compat: if older AR1 builds send STAT without HB,
    // treat STAT as "link alive" so REMOTE does not immediately collapse into AUTO.
    if (!ignoreHbInTest) {
      if (hb > 0) {
        lastHbMs = millis();
      } else if (hb < 0) {
        // HB field absent
        lastHbMs = millis();
      }
    }

    // HA "Connection self-restarts" switch (optional field) from AR1
    const int sr = readFieldInt(frame, "SR", -1);
    if (sr >= 0 && (sr != 0) != selfRestartEnabled) {
      selfRestartEnabled = (sr != 0);
      EEPROM.update(EE3_SR_ENABLED, selfRestartEnabled ? 1 : 0);
      lastSrCtrlMs = 0;  // pass it on to AR2 straight away
      fsLogTs();
      Serial.print(F("[RESTART] connection restarts "));
      Serial.println(selfRestartEnabled ? F("on") : F("off"));
    }

    // LOCK (optional field) from AR1
    int lock01 = readFieldInt(frame, "LOCK", remoteLockActive ? 1 : 0);
    bool newLock = (lock01 != 0);
    if (newLock != remoteLockActive) {
      remoteLockActive = newLock;
      fsLogTs();
      Serial.print("[LOCK] ");
      Serial.println(remoteLockActive ? "ON" : "OFF");
    }

    // NEW: in REMOTE mode, follow HA relay bits P1–P4 from STAT frame
    if (!fsInAuto) {
      int p1 = readFieldInt(frame, "P1", rdP(PIN_P1));
      int p2 = readFieldInt(frame, "P2", rdP(PIN_P2));
      int p3 = readFieldInt(frame, "P3", rdP(PIN_P3));
      int p4 = readFieldInt(frame, "P4", rdP(PIN_P4));

      setP1234(p1, p2, p3, p4);

      fsLogTs();
      Serial.print("[STAT] REMOTE P1-4 <- ");
      Serial.print(p1);
      Serial.print(p2);
      Serial.print(p3);
      Serial.print(p4);
      Serial.print(" frame=");
      Serial.println(frame);
    } else {
      // In AUTO we just log and ignore P1–P4, AUTOnomy owns the relays
      fsLogTs();
      Serial.print("[STAT] (AUTO) ");
      Serial.println(frame);
    }

    return;
  }

  if (frame.startsWith("CFG:")) {
    // Currently only expecting manual intermittent config from AR1:
    //   CFG:INT|ON_MIN:<mins>|OFF_MIN:<mins>|CHK
    int onMin = readFieldInt(frame, "ON_MIN", (manIntOnMs > 0) ? (int)(manIntOnMs / 60000UL) : 0);
    int offMin = readFieldInt(frame, "OFF_MIN", (manIntOffMs > 0) ? (int)(manIntOffMs / 60000UL) : 0);

    if (onMin < 0) onMin = 0;
    if (offMin < 0) offMin = 0;
    if (onMin > 1440) onMin = 1440;
    if (offMin > 1440) offMin = 1440;

    manIntOnMs = (unsigned long)onMin * 60000UL;
    manIntOffMs = (unsigned long)offMin * 60000UL;

    fsLogTs();
    Serial.print("[CFG] INT P1 ON_MIN=");
    Serial.print(onMin);
    Serial.print(" OFF_MIN=");
    Serial.println(offMin);
    return;
  }

  if (frame.startsWith("CMD:")) {
    fsLogTs();
    Serial.print("[CMD] ");
    Serial.println(frame);

    int seqVal = -1;
    int idx = frame.indexOf("SEQ:");
    if (idx >= 0) {
      int j = idx + 4;
      int k = j;
      while (k < (int)frame.length() && isDigit(frame[k])) k++;
      seqVal = frame.substring(j, k).toInt() & 0x0F;
    }

    // LOCK field is accepted in any mode (LOCK state is mode-independent).
    int lock01 = readFieldInt(frame, "LOCK", remoteLockActive ? 1 : 0);
    bool newLock = (lock01 != 0);
    if (newLock != remoteLockActive) {
      remoteLockActive = newLock;
      fsLogTs();
      Serial.print("[LOCK] ");
      Serial.println(remoteLockActive ? "ON" : "OFF");
    }

    // In AUTO mode, autonomy owns the relays — ignore relay/pump fields but ACK
    // so AR1 does not retry and cause RS485 contention.
    if (fsMode == FS_MODE_AUTONOMOUS) {
      fsLogTs();
      Serial.println("[CMD] AUTO guard: ignoring relay/pump fields");
      if (seqVal >= 0) {
        String ack = "ACK:SEQ:" + String(seqVal & 0x0F) + "|PUMP:" + (pumpState ? "1" : "0");
        sendLineOnBothPaths(ack);
      }
      return;
    }

    bool hasPump = frame.indexOf("PUMP:") >= 0;
    int pumpReq = hasPump ? readFieldInt(frame, "PUMP", 0) : 0;

    // LOCK forces pump request OFF
    if (remoteLockActive) {
      pumpReq = 0;
    }

    setP1234(
      readFieldInt(frame, "P1", rdP(PIN_P1)),
      readFieldInt(frame, "P2", rdP(PIN_P2)),
      readFieldInt(frame, "P3", rdP(PIN_P3)),
      readFieldInt(frame, "P4", rdP(PIN_P4)));

    if (hasPump) {
      applyPumpEdge(pumpReq ? 1 : 0, seqVal);
    } else if (seqVal >= 0) {
      String ack = "ACK:SEQ:" + String(seqVal & 0x0F) + "|PUMP:" + (pumpState ? "1" : "0");
      sendLineOnBothPaths(ack);
    }
    return;
  }

  fsLogTs();
  Serial.print("[RX] unknown: ");
  Serial.println(frame);
}

// ---------------- Telemetry up to AR1 ----------------
static const unsigned long TEL_PERIOD_MS = 2000;
static unsigned long telLastMs = 0;

static float bestFlow() {
  if (millis() - lastUdpMs <= UDP_STALE_MS) {
    if (!isnan(ar2FlowOut) && ar2FlowOut > 0) return ar2FlowOut;
    if (!isnan(ar2Flow)) return ar2Flow;
  }
  return 0.0f;
}

// ---------------- Tank level when the gauge fails (2026-11-01) ----------------
// From the moment AR2 flags the gauge as failed, the level is estimated from the last good
// reading by subtracting the water through the DN50 meter (after the recirculation valve, so
// only water actually used; shed cooling included) and adding mains refill (DN15). If the DN50
// meter itself reads nothing while the pump runs with the valve to the sprinklers, it is taken
// as failed and 147 L/min (measured reduced flow) is assumed whenever that is the case.
// Gauge readings good again for 2 min (20 s on TestSys) -> back to the gauge.
static const float TANK_L_PER_M = 9500.0f;          // 95 L per cm (house-meter refill, Oct 2026)
static const float TANK_FULL_M = 2.94f;             // full depth (Oct 2026 calibration)
static const float ASSUMED_FLOW_LPM = 147.0f;       // measured reduced flow
static const unsigned long WL_RECOVER_MS_FIELD = 120000UL, WL_RECOVER_MS_TESTSYS = 20000UL;
static const unsigned long METER_FAIL_MS_FIELD = 60000UL,  METER_FAIL_MS_TESTSYS = 15000UL;
static bool fsWlEstActive = false;
static float fsWlEst = NAN;
static bool fsMeterFailed = false;
static unsigned long fsWlLastTickMs = 0, fsWlGoodSinceMs = 0, fsMeterZeroSinceMs = 0;
static float fsSavedEst = NAN, fsSavedGood = NAN;  // what is in EEPROM (restart safety)

static void fsWlEstReset() {
  fsWlEstActive = false; fsWlEst = NAN; fsMeterFailed = false;
  fsWlGoodSinceMs = 0; fsMeterZeroSinceMs = 0; ar2WlLastGood = NAN;
}

static void fsWaterTick() {
  const unsigned long now = millis();
  unsigned long dt = fsWlLastTickMs ? now - fsWlLastTickMs : 0;
  fsWlLastTickMs = now;
  if (dt > 2000) dt = 2000;  // never integrate across a long stall

  const bool udpFresh = (now - lastUdpMs <= UDP_STALE_MS);
  const float flowOut = (udpFresh && !isnan(ar2FlowOut)) ? ar2FlowOut : 0.0f;
  const float flowIn = (udpFresh && !isnan(ar2FlowIn)) ? ar2FlowIn : 0.0f;
  const bool toSprinklers = pumpState && !rdP(PIN_P1);  // pump on, valve not recirculating

  // DN50 meter health: nothing while water should be going to the sprinklers.
  if (toSprinklers && flowOut < 1.0f) {
    if (!fsMeterZeroSinceMs) fsMeterZeroSinceMs = now;
    if (!fsMeterFailed && now - fsMeterZeroSinceMs >= (JF_isTestSys ? METER_FAIL_MS_TESTSYS : METER_FAIL_MS_FIELD)) {
      fsMeterFailed = true;
      fsLogTs(); Serial.println(F("[WL] flow meter reads 0 with pump on -> assuming 147 L/min"));
    }
  } else {
    fsMeterZeroSinceMs = 0;
    if (fsMeterFailed && flowOut >= 1.0f) {
      fsMeterFailed = false;
      fsLogTs(); Serial.println(F("[WL] flow meter reading again"));
    }
  }

  if (ar2WlFail) {
    fsWlGoodSinceMs = 0;
    if (!fsWlEstActive) {
      fsWlEstActive = true;
      fsWlEst = isnan(ar2WlLastGood) ? TANK_FULL_M : ar2WlLastGood;  // no good reading yet: assume full
      fsLogTs(); Serial.print(F("[WL] tank gauge FAILED -> estimating from flow, start "));
      Serial.print(fsWlEst, 2); Serial.println(F(" m"));
    }
  } else if (fsWlEstActive) {
    if (!fsWlGoodSinceMs) fsWlGoodSinceMs = now;
    if (now - fsWlGoodSinceMs >= (JF_isTestSys ? WL_RECOVER_MS_TESTSYS : WL_RECOVER_MS_FIELD)) {
      fsWlEstActive = false;
      fsLogTs(); Serial.print(F("[WL] tank gauge back -> using it (estimate was "));
      Serial.print(fsWlEst, 2); Serial.println(F(" m)"));
    }
  }

  if (fsWlEstActive && dt) {
    const float outLpm = fsMeterFailed ? (toSprinklers ? ASSUMED_FLOW_LPM : 0.0f) : flowOut;
    fsWlEst -= (outLpm - flowIn) * (dt / 60000.0f) / TANK_L_PER_M;
    if (fsWlEst < 0.0f) fsWlEst = 0.0f;
    if (fsWlEst > TANK_FULL_M) fsWlEst = TANK_FULL_M;
  }

  // Restart safety: while in Auto, keep the estimate and the last good gauge reading in EEPROM
  // so a restart mid-fire carries on from them. Rate-limited to spare the flash.
  static unsigned long lastEstSaveMs = 0, lastGoodSaveMs = 0;
  if (fsInAuto) {
    if (fsWlEstActive) {
      if (now - lastEstSaveMs >= 60000UL && (isnan(fsSavedEst) || fabsf(fsWlEst - fsSavedEst) >= 0.01f)) {
        EEPROM.put(EE3_EST, fsWlEst);
        fsSavedEst = fsWlEst;
        lastEstSaveMs = now;
      }
    } else if (!isnan(fsSavedEst)) {
      const float none = NAN;
      EEPROM.put(EE3_EST, none);
      fsSavedEst = NAN;
    }
    if (!isnan(ar2WlLastGood) && now - lastGoodSaveMs >= 300000UL &&
        (isnan(fsSavedGood) || fabsf(ar2WlLastGood - fsSavedGood) >= 0.10f)) {
      EEPROM.put(EE3_GOOD, ar2WlLastGood);
      fsSavedGood = ar2WlLastGood;
      lastGoodSaveMs = now;
    }
  }
}

static float bestWaterLevel() {
  return fsWlEstActive ? fsWlEst : ar2WaterLvl;
}

// 1 = gauge, 2 = estimated from the flow meters, 3 = estimated, flow meter failed (147 L/min assumed)
static int wlMethod() {
  if (!fsWlEstActive) return 1;
  return fsMeterFailed ? 3 : 2;
}

// ---- Autonomous cycle report for HA (read-only display via AR1) ----
// SPR:a,on,off,ph   a: -1 auto not running, 0 no sprinkler request,
//                      1 continuous (pump), 2 intermittent via pump (P0),
//                      3 intermittent via recirc (P1);
//                   on/off = cycle minutes; ph = 1 in ON phase, 0 in OFF phase.
// SHD:s             s: -1 auto not running, 0 off, 1 thermostat pulse (1 min),
//                      2 continuous, 3 timed cycle (not used by current plan).
// TST:t             t: 1 while a button-started test is running, else 0.
static String fsCycleTelemetry() {
  bool live = (fsMode == FS_MODE_AUTONOMOUS) &&
              !(testActive && (testAwaitUdp || (millis() - testStartMs) < TEST_WARMUP_MS));
  String tst = testActive ? "|TST:1" : "|TST:0";
  if (!live) return String("|SPR:-1,0,0,0|SHD:-1") + tst;
  int a = 0, on = 0, off = 0, ph = 0;
  if (fsAutoOut.pumpContinuous) {
    a = 1;
    ph = 1;
  } else if (fsAutoOut.sprinklerIntermittent && fsAutoOut.sprinklerDrv != FS_SPRINKLER_NONE &&
             (fsAutoOut.sprinklerCycle.on > 0 || fsAutoOut.sprinklerCycle.off > 0)) {
    a = (fsAutoOut.sprinklerDrv == FS_SPRINKLER_PUMP_P0) ? 2 : 3;
    on = fsAutoOut.sprinklerCycle.on;
    off = fsAutoOut.sprinklerCycle.off;
    ph = fsSprOnPhase ? 1 : 0;
  }
  int s = 0;
  if (fsAutoOut.shedContinuous) {
    s = (fsShedThermPulseActive && !fsShedThermContinuous) ? 1 : 2;
  } else if (fsAutoOut.shedIntermittent) {
    s = 3;
  }
  return "|SPR:" + String(a) + "," + String(on) + "," + String(off) + "," + String(ph)
         + "|SHD:" + String(s) + tst;
}

static void emitTelemetryIfDue() {
  unsigned long now = millis();
  if (now - telLastMs < TEL_PERIOD_MS) return;
  telLastMs = now;

  int pmp = pumpState ? 1 : 0;
  int p1 = rdP(PIN_P1);
  int p2 = rdP(PIN_P2);
  int p3 = rdP(PIN_P3);
  int p4 = rdP(PIN_P4);

  // ---- TestSys support: simulate battery voltage for functional testing ----
  // In production (JF_isTestSys==false) we use the measured/received ar2Bat value.
  float telemBatV = ar2Bat;
  if (JF_isTestSys) {
    telemBatV = 12.8f;
  }


  String msg = "INT_TEMP:" + String(ar2IntTemp, 2)
               + "|EXT_TEMP:" + String(ar2ExtTemp, 2)
               + "|EXT2_TEMP:" + String(ar2Ext2Temp, 2)
               + "|FLOW:" + String(bestFlow(), 2)
               + "|WATER_LEVEL:" + String(bestWaterLevel(), 2)
               + "|EMBER:" + String(ar2Ember ? 1 : 0)
               + "|WLMETHOD:" + String(wlMethod())
               + "|PHASE:" + String((int)fsPhase)
               + "|MODE:" + String((int)fsModeCode)
               + "|FSMODE:" + String((int)fsMode)
               + "|BAT:" + String(telemBatV, 2)
               + "|PUMP:" + String(pmp)
               + "|P1:" + String(p1)
               + "|P2:" + String(p2)
               + "|P3:" + String(p3)
               + "|P4:" + String(p4)
               + "|OIL_OK:" + String(ar2OilOk ? 1 : 0)
               + "|WATER_PV:" + String(ar2WaterPressV, 2)
               + "|FUEL:" + String(ar2Fuel, 2)
               + fsCycleTelemetry()
               + "|TFAIL:" + String(ar2TFail)
               + "|FB:" + String(!fsFbActive ? -1 : (fsFbStage < 0 ? 9 : fsFbStage))
               + "|RST:" + String(resetCause3)
               + "|RST2:" + String(ar2ResetCause)
               // Button test until AR1 confirms: HB:0 inside telemetry too. The raw HB:0 line
               // was lost on the backup path (bench 2026-11-03); telemetry gets through.
               + ((testActive && !testHbOffSeen) ? "|HB:0" : "")
               + "|FOK:" + String(fcOk) + "|FBAD:" + String(fcBad) + "|FNC:" + String(fcNoChk)
               + "|READY:1";

  static unsigned long fcLastLogMs = 0;
  if (now - fcLastLogMs >= 600000UL) {
    fcLastLogMs = now;
    fsLogTs();
    Serial.print(F("[FRAME] since boot from AR1: ok="));
    Serial.print(fcOk);
    Serial.print(F(" bad="));
    Serial.print(fcBad);
    Serial.print(F(" nochk="));
    Serial.println(fcNoChk);
  }

  fsSendFramed(msg);
  fsSendFramedBackup(msg);  // backup path: AR1 uses this to detect primary recovery
}

// ---------------- Mode manager ----------------
static void relaysSafe() {
  digitalWrite(PIN_P1, HIGH);
  digitalWrite(PIN_P2, HIGH);
  digitalWrite(PIN_P3, HIGH);
  digitalWrite(PIN_P4, HIGH);
}

static void enterAuto(bool resume) {
  if (!resume) {
    if (pumpState) {
      crankForceOff();  // Option B: starter off before the blocking coil pulse
      // 2026-10-10: was coilPulse(PIN_COIL_OFF, false), which ignored
      // FS_POLARITY_INVERT; now uses the mapped pin and the latch read-back.
      pumpPulseIfNeeded(0);
      pumpState = false;
    }
    relaysSafe();
  }

  // Reset autonomous state.
  fsPhase = FS_PHASE_WARMUP;

  // Reset autonomous state.
  fsPhase = FS_PHASE_WARMUP;

  fsLastExtForRise = NAN;
  fsLastExtRiseMs = 0;
  fsTrise = 0.0f;

  fsPeakExt = NAN;
  fsPeakInt = NAN;

  memset(&fsAutoOut, 0, sizeof(fsAutoOut));
  fsAutoOut.sprinklerCycle = { 0, 0 };
  fsAutoOut.shedCycle = { 0, 0 };

  fsSprPhaseMs = 0;
  fsSprOnPhase = false;
  fsLastLoggedMode = 0xFF;  // force mode log on first AUTO tick

  // Reset shed thermostat runtime state for a fresh AUTO run.
  fsShedThermContinuous = false;
  fsShedThermPulseActive = false;
  fsShedThermArmed = false;
  fsShedThermPulseStartMs = 0;
  fsShedThermLastStartMs = 0;

  fsFbReset();

  fsInAuto = true;
  fsMode = FS_MODE_AUTONOMOUS;
  modeFlipMs = millis();
  // Restart safety: resume Auto after a restart. Also write the tank values afresh, so only
  // values saved during this Auto run can ever be resumed (never stale bytes - 2026-11-02 bench).
  EEPROM.update(EE3_AUTO, 0xA5);
  {
    const float none = NAN;
    fsSavedEst = fsWlEstActive ? fsWlEst : NAN;
    fsSavedGood = ar2WlLastGood;
    EEPROM.put(EE3_EST, fsWlEstActive ? fsWlEst : none);
    EEPROM.put(EE3_GOOD, ar2WlLastGood);
  }

  fsLogTs();
  Serial.println("[MODE] ENTER AUTO");
}

static void exitAuto() {
  fsInAuto = false;
  fsMode = FS_MODE_REMOTE;
  modeFlipMs = millis();
  // Restart safety: nothing to resume any more.
  {
    const float none = NAN;
    EEPROM.update(EE3_AUTO, 0);
    EEPROM.update(EE3_FB, 0xFF);
    EEPROM.put(EE3_EST, none);
    EEPROM.put(EE3_GOOD, none);
    fsSavedEst = fsSavedGood = NAN;
  }

  fsLogTs();
  Serial.println("[MODE] EXIT AUTO");
}

static void tickMode() {
  if (testActive) {
    // TEST mode wants AUTO, but only if this build/hardware allows it
    if (fsAutoAllowed()) {
      if (!fsInAuto) enterAuto(haAutoResume);
    } else {
      // AUTO is forbidden in this build on this hardware: stay REMOTE
      if (fsInAuto) {
        exitAuto();
      }
    }
    return;
  }

  unsigned long now = millis();
  bool fresh = hbFresh();

  // Do not enter AUTO immediately at boot just because lastHbMs==0.
  // Only treat HB as "long missing" if either:
  //   - we had HB before and it's now stale, OR
  //   - we've been up HB_ARM_MS with no HB ever seen.
  bool hbLongMissing = false;
  if (lastHbMs != 0) {
    hbLongMissing = (now - lastHbMs >= HB_ARM_MS);
  } else {
    hbLongMissing = (bootMs != 0) && (now - bootMs >= HB_ARM_MS);
  }

  if (!fsInAuto) {
    // Only enter AUTO if both build and hardware allow it
    if (fsAutoAllowed() && (haAutoEnable || hbLongMissing)) {
      enterAuto(haAutoResume);
    }
  } else {
    // Already in AUTO: normal exit path, but this block is unreachable if
    // fsAutoAllowed() is false, because we never enter AUTO in that case.
    static unsigned long hbStableStart = 0;

    // Use HB_FRESH_FOR_EXIT_MS (35 s) not hbFresh() (4 s): in AUTO mode telemetry is
    // every 30 s, so STAT frames arrive every ~30 s and the 4 s window expires between them.
    bool freshForExit = (lastHbMs != 0) && (now - lastHbMs < HB_FRESH_FOR_EXIT_MS);
    if (!haAutoEnable && freshForExit) {
      if (hbStableStart == 0) hbStableStart = now;
      if (now - hbStableStart >= HB_STABLE_MS) {
        hbStableStart = 0;
        exitAuto();
      }
    } else {
      hbStableStart = 0;
    }
  }
}


// ---------------- AUTO tick ----------------
static void tickAuto() {
  if (fsMode != FS_MODE_AUTONOMOUS) return;

  // In TEST warm-up, hold everything idle until AR2 is sending.
  if (testActive && (testAwaitUdp || (millis() - testStartMs) < TEST_WARMUP_MS)) {
    if (pumpState) {
      // Ensure pump is OFF while we’re waiting for AR2
      applyPumpEdge(0, -1);
    }
    relaysSafe();
    // Reset intermittent FSMs so cycles restart cleanly after warm-up
    fsSprPhaseMs = 0;
    fsSprOnPhase = false;
    fsShedPhaseMs = 0;
    fsShedOnPhase = false;
    return;
  }

  // Normal autonomous tick: compute plan, then apply outputs.
  fsComputeAutoPlan();

  // A) Log mode transitions (named, with key sensor values).
  if (fsModeCode != fsLastLoggedMode) {
    fsLogTs();
    Serial.print(F("[AUTO] Mode "));
    if (fsLastLoggedMode != 0xFF) {
      Serial.print(fsModeCodeName(fsLastLoggedMode));
      Serial.print(F(" -> "));
    }
    Serial.print(fsModeCodeName(fsModeCode));
    Serial.print(F(" (EXT="));
    Serial.print(ar2ExtTemp, 1);
    Serial.print(F(" INT="));
    Serial.print(ar2IntTemp, 1);
    Serial.print(F(" WL="));
    Serial.print(bestWaterLevel(), 2);
    Serial.println(')');
    fsLastLoggedMode = fsModeCode;
  }

  // Apply outputs before the status line below, so it shows this tick's relays, not last tick's.
  fsApplyAutoOutputs();

  // C) Periodic status heartbeat every 30 s.
  static unsigned long fsAutoStatusMs = 0;
  if (millis() - fsAutoStatusMs >= 30000UL) {
    fsAutoStatusMs = millis();
    fsLogTs();
    Serial.print(F("[AUTO] phase="));
    Serial.print(fsPhase == FS_PHASE_COOLDOWN ? F("COOLDOWN") : F("WARMUP"));
    Serial.print(F(" mode="));
    Serial.print(fsModeCodeName(fsModeCode));
    Serial.print(F(" EXT="));   Serial.print(ar2ExtTemp, 1);
    Serial.print(F(" INT="));   Serial.print(ar2IntTemp, 1);
    Serial.print(F(" WL="));    Serial.print(bestWaterLevel(), 2);
    Serial.print(F(" P0="));    Serial.print(pumpState ? 1 : 0);
    Serial.print(F(" P1="));    Serial.print(digitalRead(PIN_P1) == LOW ? 1 : 0);
    Serial.print(F(" P2="));    Serial.println(digitalRead(PIN_P2) == LOW ? 1 : 0);
  }
}

// ---------------- TEST (SIMCTRL to AR2) ----------------
static void sendSimctrl(const String &cmd) {
  char buf[128];
  cmd.toCharArray(buf, sizeof(buf));
  udp.beginPacket(IPAddress(192, 168, 4, 1), UDP_PORT);
  udp.write((uint8_t *)buf, strlen(buf));
  udp.endPacket();
}

static void startTest() {
  // 1) Tell AR2 how fast to run the scenario
  sendSimctrl(String("SIMCTRL:SPEED:") + String(testSpeed, 3));

  // 2) Tell AR2 which scenario to run (Scenario 1 = extended fire run)
  sendSimctrl(String("SIMCTRL:START:") + String(testScenarioId));

  // 3) Force HB OFF on RS-485 so AUTO won’t drop out while we’re testing.
  //    Both paths: if RS-485 has failed, AR1 is listening on the backup path (2026-11-03).
  sendLineOnBothPaths("HB:0");
  lastHbMs = 0;

  // 4) Pin local test/autonomous state
  testActive = true;
  ignoreHbInTest = true;
  testHbOffSeen = false;
  testHbOnCount = 0;
  testStopByHa = false;
  testSimSeen = false;
  testRealPkts = 0;
  testScenDone = false;
  testAwaitUdp = true;
  testStartMs = millis();

  if (fsAutoAllowed()) {
    if (!fsInAuto) enterAuto(haAutoResume);
  } else {
    fsLogTs();
    Serial.println("[TEST] AUTO disabled by build-time flag; staying REMOTE");
  }

  // Reset autonomous state for the test run.
  fsPhase = FS_PHASE_WARMUP;

  fsLastExtForRise = NAN;
  fsLastExtRiseMs = 0;
  fsTrise = 0.0f;

  fsPeakExt = NAN;
  fsPeakInt = NAN;

  memset(&fsAutoOut, 0, sizeof(fsAutoOut));
  fsAutoOut.sprinklerCycle = { 0, 0 };
  fsAutoOut.shedCycle = { 0, 0 };

  // Reset shed thermostat state so each test run starts clean.
  fsShedThermContinuous = false;
  fsShedThermPulseActive = false;
  fsShedThermArmed = false;
  fsShedThermPulseStartMs = 0;
  fsShedThermLastStartMs = 0;

  fsSprPhaseMs = 0;
  fsSprOnPhase = false;
  fsShedPhaseMs = 0;
  fsShedOnPhase = false;
  fsFbReset();
  fsExtIntOffset = NAN;  // scenario temps are simulated: learn the offset afresh
  fsWlEstReset();

  fsLogTs();
  Serial.println("[TEST] START → sending SIMCTRL:SPEED then SIMCTRL:START");
}

static void endTest() {
  sendSimctrl("SIMCTRL:STOP");
  testActive = false;
  testScenarioId = 1;  // the button always runs scenario 1
  // Forget the simulated fire: AR3 stays in AUTO after a test until HA takes over, and the
  // test's history must not steer real control (it started the fallback at stage L).
  fsFbReset();
  fsPhase = FS_PHASE_WARMUP;
  fsPeakExt = NAN;
  fsPeakInt = NAN;
  fsExtIntOffset = NAN;
  fsWlEstReset();
  ar2WlFail = 0;  // the last simulated gauge/sensor flags too (real values arrive in ~0.1 s)
  ar2TFail = 0;
  // Drop the last simulated readings too; real AR2 values replace them within ~0.1 s.
  ar2IntTemp = ar2ExtTemp = ar2Ext2Temp = NAN;
  ar2WaterLvl = ar2FlowOut = ar2Flow = ar2Bat = NAN;
  memset(&fsAutoOut, 0, sizeof(fsAutoOut));  // and the test's plan, so nothing is "held"
  fsModeCode = FS_MODE_IDLE;
  ignoreHbInTest = false;
  testAwaitUdp = false;
  testSimSeen = false;
  testRealPkts = 0;
  testScenDone = false;

  fsLogTs();
  Serial.println("[TEST] END");
}

// ---------------- UDP intake (AR2) ----------------
static void parseUdpCsv(const String &csv) {
  // Simulated (test) packets outside a test - in flight at test end, or AR2 still simulating
  // after a lost STOP - must never drive real control.
  const bool sim = readFieldInt(csv, "SIM", 0) == 1;
  if (!testActive && sim) return;
  // During a test: once AR2 has simulated, real packets mean its scenario has finished. Don't
  // mix them into the simulated fire; after a short run of them the test ends (loop()).
  if (testActive) {
    if (sim) { testSimSeen = true; testRealPkts = 0; }
    else if (testSimSeen) {
      if (testRealPkts < 255 && ++testRealPkts >= TEST_SIM_END_PKTS) testScenDone = true;
      return;
    }
  }
  {
    const int r = readFieldInt(csv, "RST", -1);
    if (r >= 0) ar2ResetCause = (int8_t)r;
  }
  float v;
  v = readFieldFloat(csv, "INT_TEMP", NAN);
  if (!isnan(v)) ar2IntTemp = v;

  v = readFieldFloat(csv, "EXT_TEMP", NAN);
  if (!isnan(v)) ar2ExtTemp = v;

  v = readFieldFloat(csv, "EXT2_TEMP", NAN);
  if (!isnan(v)) ar2Ext2Temp = v;

  v = readFieldFloat(csv, "WATER_LEVEL", NAN);
  if (!isnan(v)) ar2WaterLvl = v;

  v = readFieldFloat(csv, "FLOW_OUT", NAN);
  if (!isnan(v)) ar2FlowOut = v;

  v = readFieldFloat(csv, "FLOW_IN", NAN);
  if (!isnan(v)) ar2FlowIn = v;

  v = readFieldFloat(csv, "FLOW", NAN);
  if (!isnan(v)) ar2Flow = v;

  v = readFieldFloat(csv, "BAT", NAN);
  if (!isnan(v)) ar2Bat = v;

  int e = readFieldInt(csv, "EMBER", -999);
  if (e != -999) ar2Ember = e;

  // Pump pressure fields from AR2
  int oil = readFieldInt(csv, "OIL_OK", -999);
  if (oil != -999) ar2OilOk = (oil != 0);

  v = readFieldFloat(csv, "WATER_PV", NAN);
  if (!isnan(v)) ar2WaterPressV = v;

  v = readFieldFloat(csv, "FUEL", NAN);
  if (!isnan(v)) ar2Fuel = v;

  int tf = readFieldInt(csv, "TFAIL", -1);
  ar2TFail = (tf >= 0) ? (uint8_t)tf : 0;
  int wf = readFieldInt(csv, "WLFAIL", -1);
  ar2WlFail = (wf > 0) ? 1 : 0;
  if (!ar2WlFail && !isnan(ar2WaterLvl)) ar2WlLastGood = ar2WaterLvl;

  // Learn outside-minus-inside while an outside sensor and the inside sensor both work.
  const bool extOk = !isnan(ar2ExtTemp) && !(ar2TFail & 1);
  const bool ext2Ok = !isnan(ar2Ext2Temp) && !(ar2TFail & 4);
  if ((extOk || ext2Ok) && !isnan(ar2IntTemp) && !(ar2TFail & 2)) {
    fsExtIntOffset = (extOk ? ar2ExtTemp : ar2Ext2Temp) - ar2IntTemp;
  }
}

static void udpPoll() {
  int len = udp.parsePacket();
  if (len <= 0) return;

  char buf[256];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = 0;

  String s(buf);
  s.trim();
  if (!s.length()) return;

  lastUdpLine = s;
  lastUdpMs = millis();
  lastAr2Ms = lastUdpMs;
  if (!ar2EverSeen) {
    ar2EverSeen = true;
    ar2LastState = true;
  }

  if (testActive && testAwaitUdp) {
    testAwaitUdp = false;
    fsLogTs();
    Serial.println("[TEST] first UDP → leaving warm-up hold");
  }

  parseUdpCsv(s);
}

// Ensure we are connected as STA to AR2 AP (Arduino2_AP).
// Called from setup() and loop(). Non-blocking: no long while() loops.
static void fsWifiEnsureApLink() {
  unsigned long nowMs = millis();
  int status = WiFi.status();
  // WL_CONNECTED fires before DHCP completes on R4 WiFi.
  // Only require a valid IP during the initial connection — once connected,
  // trust WL_CONNECTED alone to avoid spurious dropout if localIP() momentarily
  // returns 0.0.0.0 during DHCP renewal (which would call WiFi.begin() and
  // disrupt an active connection).
  bool connected;
  if (wifiLastState) {
    connected = (status == WL_CONNECTED);
  } else {
    connected = (status == WL_CONNECTED) &&
                (WiFi.localIP() != IPAddress(0, 0, 0, 0));
  }

  if (connected) {
    // We have a link with a valid IP.
    if (!wifiLastState) {
      // Transition: just became connected
      fsLogTs();
      Serial.print(F("[WiFi] Connected to "));
      Serial.print(WIFI_SSID);
      Serial.print(F(" IP "));
      Serial.println(WiFi.localIP());
    }
    wifiLastState = true;
    wifiConnecting = false;
    wifiFailCount = 0;    // reset backoff on success
    wifiBackoffMs = 0;
    return;
  }

  // Not connected
  wifiLastState = false;

  // If we're currently in a connect attempt, just watch for timeout.
  if (wifiConnecting) {
    if (nowMs - wifiConnectStartMs >= WIFI_CONNECT_TIMEOUT_MS) {
      // Timed out: apply exponential backoff (15 s → 30 s → 60 s → 120 s max)
      wifiConnecting = false;
      wifiFailCount++;
      unsigned long base = WIFI_RETRY_INTERVAL_MS;
      wifiBackoffMs = (wifiFailCount >= 4) ? 120000UL
                    : (base << (wifiFailCount - 1 < 3 ? wifiFailCount - 1 : 3));
      lastWifiRetryMs = nowMs;
      fsLogTs();
      Serial.print(F("[WiFi] still not connected to AR2 AP (timeout) — next retry in "));
      Serial.print(wifiBackoffMs / 1000);
      Serial.println(F(" s"));
    }
    // Either way, don't start another WiFi.begin() yet.
    return;
  }

  // Not connected and not currently trying: see if it's time to start a new attempt.
  unsigned long retryInterval = (wifiBackoffMs > 0) ? wifiBackoffMs : WIFI_RETRY_INTERVAL_MS;
  if (lastWifiRetryMs != 0 && (nowMs - lastWifiRetryMs) < retryInterval) {
    return;
  }
  lastWifiRetryMs = nowMs;

  fsLogTs();
  Serial.print(F("[WiFi] retry → "));
  Serial.println(WIFI_SSID);

  // Do NOT call WiFi.disconnect() here: if a previous begin() attempt is
  // still in progress the ESP32-S3 blocks for ~10 s waiting for it to cancel,
  // stalling the main loop and silencing RS-485 telemetry long enough to
  // trigger AR1's path watchdog.  WiFi.begin() resets state on its own.
  // AR3B: WiFi.begin() BLOCKS (up to WiFi.setTimeout(), set in setup()), so the
  // starter must never be left energised across it.
  crankForceOff();
  // Bound the blocking time so the 5 s watchdog can't fire when Arduino2_AP is
  // slow or absent (seen 2026-10-30: AR3B reboot-looped on every boot). Other
  // modem commands are capped at 1 s (setup()), but the join command is only
  // answered once the join finishes (normally 1-3 s), so a 1 s cap made every
  // fresh join fail (2026-10-31). Allow 3 s for begin() only; it then polls
  // status for 200 ms. Worst case stays inside the 5 s watchdog.
  WDT.refresh();
  modem.timeout(3000);
  WiFi.setTimeout(200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  modem.timeout(1000);
  WDT.refresh();

  wifiConnecting = true;
  wifiConnectStartMs = nowMs;
}


// Manage SoftwareSerial backup path enable/disable based on RS-485 activity.
//
// Rationale: A2 is a valid RA4M1 SoftwareSerial RX pin. When swSerial.begin()
// has been called, the library attaches an interrupt to A2. Every bit received
// from AR1's continuous HB stream fires that interrupt, which can preempt the
// SPI transfer between the RA4M1 and the ESP32-S3 WiFi module — causing WiFi
// connection drops. swSerial.end() detaches the interrupt, making A2 inert and
// letting WiFi SPI run undisturbed.
//
// State machine:
//   RS-485 recently active  → swSerial.end()   (swSerialEnabled = false)
//   RS-485 silent ≥ 10 s    → swSerial.begin() (swSerialEnabled = true)
//   Boot: hold disabled for SW_BACKUP_ACTIVATE_MS to give AR1 time to send
//   its first RS-485 frame before declaring the primary path dead.
static void tickSwSerial() {
  unsigned long now = millis();

  bool shouldEnable;
  if (lastRs485RxMs == 0) {
    // Never received an RS-485 frame: wait SW_BACKUP_ACTIVATE_MS from boot
    // before enabling, so AR1 has time to establish the primary link.
    shouldEnable = (now - bootMs >= SW_BACKUP_ACTIVATE_MS);
  } else {
    // Have received RS-485 before: enable only after prolonged silence.
    shouldEnable = (now - lastRs485RxMs >= SW_BACKUP_ACTIVATE_MS);
  }

  if (shouldEnable && !swSerialEnabled) {
    swSerial.begin(RS485_BAUD);
    swSerialEnabled = true;
    fsLogTs();
    Serial.println(F("[SW] RS-485 silent → backup path ENABLED (swSerial.begin)"));
  } else if (!shouldEnable && swSerialEnabled) {
    swSerial.end();
    swSerialEnabled = false;
    fsLogTs();
    Serial.println(F("[SW] RS-485 active → backup path DISABLED (swSerial.end)"));
  }
}

// Poll backup SoftwareSerial path — uses its own frame state to avoid corrupting RS485 frames.
// Guard on swSerialEnabled: if swSerial is disabled (swSerial.end() called) there is
// nothing to read and calling available() could misbehave.
static void swSerialPoll() {
  if (!swSerialEnabled) return;
  while (swSerial.available()) {
    feedByteSW((uint8_t)swSerial.read());
  }
}

// ---------------- setup / loop ----------------
void setup() {
  resetCause3 = readResetCause();
  selfRestartEnabled = (EEPROM.read(EE3_SR_ENABLED) != 0);
  selfRestartCount = EEPROM.read(EE3_RST_COUNT);
  if (selfRestartCount == 0xFF) selfRestartCount = 0;
  Serial.begin(USB_BAUD);
  { unsigned long _t = millis(); while (!Serial && millis() - _t < 3000UL) {} }

  Serial.println("AR3B Start (Option B start sequencer)");
  Serial.print(F("[BOOT] reset cause "));
  Serial.print(resetCause3);
  Serial.print(F(", connection restarts "));
  Serial.print(selfRestartEnabled ? F("on") : F("off"));
  Serial.print(F(", in a row "));
  Serial.println(selfRestartCount);
  bootMs = millis();
  
  RS485_PORT.begin(RS485_BAUD);
  // swSerial is NOT started here — tickSwSerial() manages begin()/end() at
  // runtime based on RS-485 activity to prevent A2 RX interrupt from
  // disrupting WiFi SPI.  swSerialEnabled starts false.
  Serial.println(F("[BOOT] T40: swSerial dynamic enable; backup RX=A2 TX=A1"));

  pinMode(PIN_COIL_ON, OUTPUT);
  digitalWrite(PIN_COIL_ON, HIGH);
  pinMode(PIN_COIL_OFF, OUTPUT);
  digitalWrite(PIN_COIL_OFF, HIGH);

  pinMode(PIN_P1, OUTPUT);
  digitalWrite(PIN_P1, HIGH);
  pinMode(PIN_P2, OUTPUT);
  digitalWrite(PIN_P2, HIGH);
  pinMode(PIN_P3, OUTPUT);
  digitalWrite(PIN_P3, HIGH);
  pinMode(PIN_P4, OUTPUT);
  digitalWrite(PIN_P4, HIGH);
  // Option B: starter relay. Write HIGH (off) BEFORE switching to OUTPUT so the
  // relay never sees a LOW glitch at boot.
  digitalWrite(PIN_CRANK, HIGH);
  pinMode(PIN_CRANK, OUTPUT);
  digitalWrite(PIN_CRANK, HIGH);
  // Mode input D9: LOW = BYPASS (ARMAC starts pump); open/HIGH = idle.
  pinMode(PIN_MODE_BYPASS, INPUT_PULLUP);
  pinMode(PIN_LATCH_FB, INPUT_PULLUP);  // latch read-back (LOW = latch ON)
  Serial.println(digitalRead(PIN_MODE_BYPASS) == LOW
                 ? F("[BOOT] D9 mode: BYPASS (ARMAC pump start enabled)")
                 : F("[BOOT] D9 mode: BOX/disabled (pump start sequencer idle)"));

  pinMode(PIN_TEST_BTN, INPUT_PULLUP);
  testBtnPrev = true;

  pinMode(PIN_TEST_BTN, INPUT_PULLUP);
  testBtnPrev = true;

  // --- TestSys / FireSys detect (D11 strap) ---
  pinMode(JF_TESTSYS_PIN, INPUT_PULLUP);
  {
    uint8_t lows = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      if (digitalRead(JF_TESTSYS_PIN) == LOW) lows++;
      delay(2);
    }
    // Majority vote for robustness
    JF_isTestSys = (lows >= 6);
  }

  // --- Cooldown detect window (TestSys only) ---
  fsFbStageIntMs = JF_isTestSys ? FS_FB_STAGE_MS_TESTSYS : FS_FB_STAGE_MS_FIELD;
  fsFbEnterMs    = JF_isTestSys ? FS_FB_ENTER_MS_TESTSYS : FS_FB_ENTER_MS_FIELD;
  fsFbRecoverMs  = JF_isTestSys ? FS_FB_RECOVER_MS_TESTSYS : FS_FB_RECOVER_MS_FIELD;
  if (JF_isTestSys) {  // short connection-check times so TestSys can be tested quickly
    SR3_NO_AR2_MS = 3UL * 60000UL;
    SR3_MIN_UPTIME_MS = 2UL * 60000UL;
  }

  //JF_isTestSys = true;
  Wire.begin();
  // Bound every I2C transfer (25 ms, bus reset on timeout) so a missing or stuck
  // LCD can never stall loop() into the 5 s watchdog.
  Wire.setWireTimeout(25000, true);

  if (JF_isTestSys) {
    // No LCD fitted: just log to Serial
    fsLogTs();
    Serial.println(F("[JF] TestSys detected via D11→GND: LCD disabled, using Serial only."));
  } else {
    // No strap = production, but check an LCD actually answers. If the D11 strap falls
    // out on TestSys there is no LCD, and driving one reboot-looped AR3B (2026-10-30).
    Wire.beginTransmission(0x27);
    lcdPresent = (Wire.endTransmission() == 0);
    if (!lcdPresent) {
      fsLogTs();
      Serial.println(F("[BOOT] WARNING: no LCD at I2C 0x27 - LCD disabled. On TestSys: D11 strap missing?"));
    }
  }
  if (lcdPresent) {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("AR3B READY");
    lcd.setCursor(0, 1);
    lcd.print("RS485+UDP+LCD");
  }

  fsLogTs();
  Serial.println("[BOOT] AR3B");
  Serial.println(LATCH_FB_FITTED ? F("[BOOT] latch read-back D10: FITTED") : F("[BOOT] latch read-back D10: not fitted (always pulse)"));
  Serial.println(F("[BOOT] AR3B ready; relays OFF (incl. CRANK D8), waiting for AR1."));

  // --- UDP listener (will only actually receive if Wi-Fi connected) ---
  if (udp.begin(UDP_PORT)) {
    fsLogTs();
    Serial.print("[UDP] listening ");
    Serial.println(UDP_PORT);
  }

  // AR3B: WiFiS3's WiFi.begin() blocks for up to its timeout (library default
  // 10 s). Keep each attempt well inside the watchdog period below.
  WiFi.setTimeout(1000);
  // Cap EVERY WiFi-module command at 1 s (library default 10 s). While the
  // ESP32 is busy joining (or failing to find) Arduino2_AP it can stop
  // answering; status()/parsePacket() then blocked up to 10 s and the 5 s
  // watchdog reboot-looped AR3B (2026-10-30). A late reply is skipped by the
  // next command's parser.
  modem.timeout(1000);
  Serial.println(F("[BOOT] build 2026-11-02a: restarts keep Auto + connection self-restart; gel forces reduced flow; test ends with scenario; temp-sensor fallback; HA can end button test; LCD probe; WiFi join 3 s"));

  // T41-AR3: Hardware watchdog — reboot if loop freezes.
  // AR3B: 8192 ms exceeds the RA4M1 maximum (16384 x 8192 / 24 MHz PCLKB =
  // 5592 ms), so WDT.begin(8192) always FAILED and AR3 had no watchdog at all.
  // 5000 ms is valid.
  if (WDT.begin(5000)) {
    Serial.println(F("[WDT] Watchdog initialized: 5000ms"));
    WDT.refresh();
  } else {
    Serial.println(F("[WDT] Watchdog init failed"));
  }

  // Restart safety: AR3 was in Auto when it restarted -> carry on in Auto now (it used to wait
  // 5 min), at the same fallback stage and tank estimate. AR1's heartbeat still ends Auto as
  // usual (steady for 15 s) if the system is really in Manual.
  if (EEPROM.read(EE3_AUTO) == 0xA5 && fsAutoAllowed()) {
    const uint8_t fb = EEPROM.read(EE3_FB);
    float est, good;
    EEPROM.get(EE3_EST, est);
    EEPROM.get(EE3_GOOD, good);
    enterAuto(true);  // relays are already safe at boot; this clears the saved fallback stage
    fsLogTs();
    Serial.println(F("[RESTART] was in Auto before the restart -> Auto resumed"));
    if (fb <= 7 || fb == 9) {
      fsFbActive = true;
      fsFbStage = (fb == 9) ? -1 : (int8_t)fb;
      fsFbStageMs = millis();
      fsSaveFb();
      fsLogFbStage("resumed after restart");
    }
    if (!isnan(good) && good >= 0.0f && good <= TANK_FULL_M) {
      ar2WlLastGood = good;
      fsSavedGood = good;
      EEPROM.put(EE3_GOOD, good);  // enterAuto() above rewrote it
    }
    if (!isnan(est) && est >= 0.0f && est <= TANK_FULL_M) {
      fsWlEstActive = true;
      fsWlEst = est;
      fsSavedEst = est;
      EEPROM.put(EE3_EST, est);
      fsLogTs();
      Serial.print(F("[RESTART] tank estimate resumed at "));
      Serial.print(est, 2);
      Serial.println(F(" m"));
    }
  }
}

// Restart safety: connection self-restart and passing HA's switch on to AR2. Called from loop().
static void tickRestartSafety() {
  const unsigned long now = millis();
  const bool ar2Fresh = ar2EverSeen && (now - lastAr2Ms < AR2_TIMEOUT_MS);

  if (ar2Fresh && selfRestartCount != 0) {
    selfRestartCount = 0;
    EEPROM.update(EE3_RST_COUNT, 0);
  }

  // After a restart, stop any simulation AR2 is still running from a test that the restart cut
  // short: AR3 ignores simulated packets outside a test, so it would see no usable data.
  static bool simStopSent = false;
  if (ar2Fresh && !simStopSent && !testActive) {
    simStopSent = true;
    sendSimctrl("SIMCTRL:STOP");
  }

  // Tell AR2 the switch setting every minute (AR2 also takes it as "AR3 is connected").
  if (ar2Fresh && (lastSrCtrlMs == 0 || now - lastSrCtrlMs >= 60000UL)) {
    lastSrCtrlMs = now;
    sendSimctrl(String("SRCTRL:") + (selfRestartEnabled ? "1" : "0"));
  }

  if (!selfRestartEnabled || now < SR3_MIN_UPTIME_MS || testActive) return;
  const unsigned long since = ar2EverSeen ? lastAr2Ms : bootMs;
  if (now - since < SR3_NO_AR2_MS) return;
  if (selfRestartCount >= SR3_MAX_IN_ROW) return;  // gave up until AR2 is back
  // Never while water is flowing or Auto has something running.
  const bool autoBusy = fsInAuto && ((fsFbActive && fsFbStage >= 0) || fsAutoOut.pumpContinuous ||
                                     fsAutoOut.sprinklerIntermittent || fsAutoOut.shedContinuous ||
                                     fsAutoOut.shedIntermittent || fsAutoOut.gelOn);
  if (pumpState || autoBusy) return;
  fsLogTs();
  Serial.println(F("[RESTART] no data from AR2 for too long -> restarting"));
  selfRestart(4);
}

// USB serial commands from a computer: TEST1 / TEST2 / TEST3 start a test scenario (TEST2 = temperature
// sensors fail, TEST3 = one steady 3/7 cycle), TESTSTOP ends it. Not needed in normal use; the button still runs scenario 1.
static void usbCommandPoll() {
  static char cmd[16];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      cmd[n] = 0;
      if (n) {
        fsLogTs();
        Serial.print(F("[USB] "));
        Serial.println(cmd);
        if (strncmp(cmd, "TEST", 4) == 0 && cmd[4] >= '1' && cmd[4] <= '4' && cmd[5] == 0 && !testActive) {
          testScenarioId = cmd[4] - '0';
          startTest();
        } else if (strcmp(cmd, "TESTSTOP") == 0 && testActive) {
          endTest();
        } else if (strcmp(cmd, "WDTTEST") == 0) {
          Serial.println(F("[USB] WDTTEST: freezing loop - the watchdog should restart AR3"));
          while (true) {}
        }
      }
      n = 0;
    } else if (n < sizeof(cmd) - 1) {
      cmd[n++] = (c >= 'a' && c <= 'z') ? c - 32 : c;
    }
  }
}

void loop() {
  usbCommandPoll();
  WDT.refresh();  // T41-AR3: pet the watchdog — must complete within 8192ms
  while (RS485_PORT.available()) {
    feedByte((uint8_t)RS485_PORT.read());
  }
  tickSwSerial();  // manage swSerial begin()/end() based on RS-485 activity
  swSerialPoll();  // backup path: same frame parser, responds on whichever path sent the command

  // Keep trying to join Arduino2_AP in the background
  fsWifiEnsureApLink();

  // LOCK safety: if locked while pump is ON, keep requesting OFF until it drops.
  // Not in autonomous mode: the lock guards against remote (pocket-dial) starts only,
  // and applyPumpEdge() already lets AUTO start the pump while locked. Without the
  // !fsInAuto test this kick fought tickAuto() with OFF/ON pulses every 0.5 s.
  if (remoteLockActive && pumpState && !fsInAuto) {
    unsigned long now = millis();
    if (now - lockPumpKickMs >= LOCK_PUMP_KICK_MS) {
      lockPumpKickMs = now;
      applyPumpEdgeRaw(0, -1);  // local safety kick; no SEQ ACK needed
    }
  }

  WDT.refresh();  // WiFi retry above may have used up to ~4 s
  udpPoll();

  if (ar2EverSeen) {
    unsigned long now = millis();
    bool ok = (now - lastAr2Ms) < AR2_TIMEOUT_MS;
    if (ok != ar2LastState) {
      fsLogTs();
      Serial.println(ok ? "AR2 link: RESTORED" : "AR2 link: LOST");
      ar2LastState = ok;
    }
  }

  fsWaterTick();
  tickMode();
  tickAuto();
  tickRestartSafety();
  tickStartSeq();  // Option B: engine start sequencer (after any pump edge this pass)

  // TEST button handling (AR3B): debounced; START needs a >= 3 s hold,
  // any press while a test is running ENDS it. A brush or glitch does nothing.
  bool curBtn = digitalRead(PIN_TEST_BTN);
  unsigned long nowMs = millis();
  if (curBtn != btnRawPrev) {
    btnRawPrev = curBtn;
    btnRawChangeMs = nowMs;
  }
  if (curBtn != btnStable && nowMs - btnRawChangeMs >= BTN_DEBOUNCE_MS) {
    btnStable = curBtn;
    if (!btnStable) {  // debounced press
      btnDownMs = nowMs;
      btnPressConsumed = false;
      if (testActive) {
        btnPressConsumed = true;
        fsLogTs();
        Serial.println("[TEST] press during test -> STOP");
        endTest();
      }
    }
  }
  if (!btnStable && !btnPressConsumed && !testActive &&
      nowMs - btnDownMs >= BTN_START_HOLD_MS) {
    btnPressConsumed = true;
    fsLogTs();
    Serial.println("[TEST] 3 s hold -> START");
    startTest();
  }
  testBtnPrev = curBtn;

  // Test-mode timeout: a forgotten or accidental test cannot hold AR3 in AUTO.
  // Fresh millis(): nowMs predates startTest() above, so nowMs - testStartMs
  // would underflow and end a just-started test at once (bench 2026-10-16).
  // At test start AR3 tells AR1 "HB:0" once; a lost frame left AR1 in remote (2026-11-01).
  // Repeat every 5 s until AR1's STAT confirms HB:0 - then stop, so HA can still end the test.
  static unsigned long lastHb0Ms = 0;
  if (testActive && !testHbOffSeen && millis() - lastHb0Ms >= 5000) {
    lastHb0Ms = millis();
    sendLineOnBothPaths("HB:0");
  }
  if (testActive && testStopByHa) {
    fsLogTs();
    Serial.println("[TEST] heartbeat back ON from HA -> STOP");
    endTest();
  }
  if (testActive && testScenDone) {
    fsLogTs();
    Serial.println("[TEST] AR2 scenario finished -> END");
    endTest();
  }
  if (testActive && millis() - testStartMs >= TEST_MAX_MS) {
    fsLogTs();
    Serial.println("[TEST] 30 min limit reached -> STOP");
    endTest();
  }

  emitTelemetryIfDue();
  tickLCD();
}
