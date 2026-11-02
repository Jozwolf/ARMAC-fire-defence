// LAST MODIFIED: 2026-11-02 AEST — Restart safety. Sends why it last restarted (RST: 0 power-on,
//   1 watchdog, 2 software, 3 reset pin, 4 connection check, 5 HA command). Restarts itself if
//   nothing has used its WiFi for 25 min (not in the first 30 min after boot, max 4 in a row;
//   HA switch relayed by AR3 as SRCTRL:0/1). Boot no longer waits forever for the access point.
// LAST MODIFIED: 2026-11-01 AEST — Temperature-sensor failures are now reported: UDP field
//   TFAIL:n (bit0 EXT, bit1 INT, bit2 EXT2; 1 = failed). Substitute values are still sent so
//   nothing else changes. A reading of exactly 85.0 C (DS18B20 power-on value) counts as failed
//   unless the sensor's last good reading was within 10 C of it. New test scenario 2
//   (SIMCTRL:START:2) simulates the sensors failing one by one to exercise AR3's fallback.
//   Simulated packets carry SIM:1 so AR3 can ignore them when no test is running.
//   A failed sensor's value is now sent as 0.0 (was 31/30/32 C) so HA shows it plainly.
//   Scenario 3 (SIMCTRL:START:3): outside held at 66 C for 11 min - one steady 3/7 cycle.
//   Tank gauge failure (voltage < 0.9 V or > 4.6 V for 2 s) now flagged as WLFAIL:1. Scenario 4
//   (SIMCTRL:START:4) simulates the gauge, then the flow meter, failing during a fire.
// LAST MODIFIED: 2026-10-31 AEST — This is the former AR2B sketch, renamed AR2B -> AR2 (no code
//   change; boot messages still say AR2B). Old AR2 retired to AR2_OLD.
// LAST MODIFIED: 2026-10-25 AEST — Tank level: two-point calibration replaces placeholder map.
// LAST MODIFIED: 2026-10-25 AEST — K_OUT_PPL 10.33 -> 16.2 (house water meter calibration).
// LAST MODIFIED: 2026-09-27 AEST — AR2B.ino created (variant of AR2.ino for Option B;
//   AR2/AR2.ino unchanged). DS18B20 temperatures now read WITHOUT blocking:
//   10-bit resolution (0.25 C, ~190 ms conversion), conversion started then
//   collected on a later pass; cached values sent meanwhile. Previously
//   requestTemperatures() blocked ~750 ms every cycle, so the "10 Hz" UDP line
//   actually went out ~1.2 Hz - too slow/irregular for AR3B's crank logic.
//   Watchdog fixed: WDT.begin(8192) was invalid (max 5592 ms) and always failed -> 5000 ms.
// LAST MODIFIED: 2026-08-26 AEST — Set FUEL_ADC_FULL/FUEL_ADC_EMPTY from
//   bench calibration (258 / 788) with the sender connected to A3. Fuel
//   level now reads real values instead of always 0%.
// LAST MODIFIED: 2026-08-22 14:57 AEST — Added diesel fuel level sender support:
//   Wema 25 sender (30-240 Ohm) on A3, 5V/100R divider, new readFuelLevel()
//   with two-point calibration, broadcast as FUEL: on the existing 10Hz UDP
//   telemetry line to AR3. FUEL_ADC_FULL/FUEL_ADC_EMPTY are still placeholder
//   999/999 — bench-measure and replace before flashing, until then this
//   always reads 0%.
// LAST MODIFIED: 2026-07-16 11:37 AEST — Verified/restored to correct content
//   after an unrelated tooling mix-up briefly overwrote this file's working
//   copy with AR3's content (caught and fixed same session, before any
//   reflash — no functional change to AR2's own firmware from this entry).
//   Last real functional change remains the K_OUT_PPL=10.33 flow calibration
//   and elapsed-time sampling fix below (2026-07-13, already committed).
// AR2B.ino (was AR2_SensorHub_AddressLocked.ino)
// Device: Arduino 2 (Sensor Aggregator)
// Purpose: Read sensors on AR2 and broadcast via UDP to AR3/TAR3 at 10 Hz.
// + Scenario Engine (sensor-only) with UDP control from TAR3
//   SIMCTRL:START:id / SIMCTRL:SPEED:x / SIMCTRL:STOP  (same UDP port)
//   Scenario overrides affect only the outgoing UDP payload; sensor ISRs remain untouched.
// 3 Dec 25 - changed Dalas Temp pin to D6.
// T1-AR2 (2026-05-31): Add RA4M1 hardware watchdog (8192ms) — auto-reboot on loop freeze.

#include <Arduino.h>
#include <WiFiS3.h>   // UNO R4 WiFi
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WDT.h>                // T1-AR2: RA4M1 hardware watchdog
#include <EEPROM.h>

// ====================== WiFi AP (AR2 acts as AP) =========================
#include "config.h"

// ====================== Restart safety (2026-11-02) ======================
// Why the board last restarted (sent to AR3 as RST, shown in HA):
//   0 power-on, 1 watchdog, 2 software, 3 reset pin / upload, 4 connection check, 5 HA command
// Connection check: if nothing has used AR2's WiFi for 25 min (AR3 sends SRCTRL every minute,
// so silence means AR3 is not connected), restart - this also resets the WiFi chip. AR3's own
// check is 15 min, so AR3 restarts first. Never in the first 30 min after a boot, and at most
// 4 in a row: after that AR2 stops trying until something connects again. HA's "Connection
// self-restarts" switch (relayed AR1 -> AR3 -> AR2 as SRCTRL:0/1) turns the check off.
static const int EE_RST_PENDING = 0;   // reason for a restart AR2 asked for itself
static const int EE_RST_COUNT = 1;     // connection restarts in a row
static const int EE_SR_ENABLED = 2;    // 0 = connection restarts off (0xFF/other = on)
static const unsigned long SR_NO_CLIENT_MS = 25UL * 60000UL;
static const unsigned long SR_MIN_UPTIME_MS = 30UL * 60000UL;
static const uint8_t SR_MAX_IN_ROW = 4;
static uint8_t resetCause = 3;
static bool selfRestartEnabled = true;
static unsigned long lastClientMs = 0;  // last sign that AR3 is connected
static bool testMute = false;           // USB test command MUTE: act as if AR3 were gone (until UNMUTE/restart)

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
    const uint8_t p = EEPROM.read(EE_RST_PENDING);
    if (p == 4 || p == 5) c = p;
  }
  EEPROM.update(EE_RST_PENDING, 0xFF);
  return c;
}

static void selfRestart(uint8_t reason) {
  EEPROM.update(EE_RST_PENDING, reason);
  if (reason == 4) EEPROM.update(EE_RST_COUNT, (uint8_t)min(255, EEPROM.read(EE_RST_COUNT) + 1));
  Serial.print(F("[RESTART] reason "));
  Serial.println(reason);
  delay(200);
  NVIC_SystemReset();
}

static void noteClient() {
  lastClientMs = millis();
  if (EEPROM.read(EE_RST_COUNT) != 0) EEPROM.update(EE_RST_COUNT, 0);
}

static void connectionCheck(unsigned long now) {
  static unsigned long lastPollMs = 0;
  if (now - lastPollMs < 30000UL) return;
  lastPollMs = now;
  // A WiFi-module call: if the module has hung it can block, and the watchdog then restarts AR2.
  if (!testMute && WiFi.status() == WL_AP_CONNECTED) noteClient();
  now = millis();  // noteClient() just stamped a later time; an older 'now' would wrap round
  if (!selfRestartEnabled || now < SR_MIN_UPTIME_MS) return;
  if (now - lastClientMs < SR_NO_CLIENT_MS) return;
  if (EEPROM.read(EE_RST_COUNT) >= SR_MAX_IN_ROW) return;  // gave up until AR3 is back
  Serial.println(F("[RESTART] no WiFi client for 25 min -> restarting"));
  selfRestart(4);
}
const char* apSSID     = AR2_AP_SSID;
const char* apPassword = AR2_AP_PASSWORD;  // if AP password unsupported, set to ""
const uint16_t UDP_PORT = 5000;         // broadcast port (match TAR3)
WiFiUDP udp;

// ====================== Simulation Flags ================================
#define SIMULATION_MODE            0   // 1 to force simulated values
#define SIMULATE_TANK_WATER_LEVEL  0
#define SIMULATED_WATER_LEVEL      2.90f
#define SIMULATE_FLOW              0
#define SIMULATED_FLOW_RATE        199.0f
#define SIMULATE_BATTERY           0
#define SIMULATED_BATTERY_VOLTAGE  13.8f

// ====================== Sensors: pins & objects ==========================
#define TANK_SENSOR_PIN A0
static const uint8_t TEMP_ONEWIRE_PIN = 6;  // Grey. D6 common bus for 3x DS18B20
static const uint8_t FLOW_IN_PIN      = 3;  // White. DN15 in-feed (mains → tank)
static const uint8_t FLOW_OUT_PIN     = 2;  // Blue. DN50 pump line (to sprinklers)
static const uint8_t EMBER_PIN        = 12; // Yellow. digital input (active-LOW assumed)
static const uint8_t FUEL_SENDER_PIN  = A3; // Wema 25 sender (30 Ohm full - 240 Ohm empty), 5V/100R divider to GND

OneWire oneWire(TEMP_ONEWIRE_PIN);
DallasTemperature dallas(&oneWire);

// ====================== DS18B20: Address-locked mapping ==================
// T1: 28 5B 83 57 00 00 00 13  -> EXT_TEMP
// T2: 28 2E 53 57 00 00 00 3E  -> INT_TEMP
// T3: 28 11 F1 35 00 00 00 D0  -> EXT2_TEMP
DeviceAddress ADDR_T1 = {0x28,0x5B,0x83,0x57,0x00,0x00,0x00,0x13};
DeviceAddress ADDR_T2 = {0x28,0x2E,0x53,0x57,0x00,0x00,0x00,0x3E};
DeviceAddress ADDR_T3 = {0x28,0x11,0xF1,0x35,0x00,0x00,0x00,0xD0};

// ====================== DS18B20: resilience state =======================
static bool    dallasReady      = false;
static uint8_t dallasFailCount  = 0;
static const uint8_t DALLAS_FAIL_LIMIT = 5;   // N consecutive all-bad reads triggers re-init

static void dallasInitBus(const char* reason) {
  Serial.print(F("[DAL] init: "));
  Serial.println(reason);

  oneWire.reset();                // defensive reset on the 1-Wire bus
  dallas.begin();
  dallas.setResolution(10);       // AR2B: 0.25 C steps (HA shows 1 dp), ~190 ms conversion
  dallas.setWaitForConversion(false);  // AR2B: non-blocking; see readThreeTemps()

  dallasReady     = true;
  dallasFailCount = 0;
}

// ====================== Flow measurement ================================
volatile unsigned long flowInPulseCount  = 0;
volatile unsigned long flowOutPulseCount = 0;
static const unsigned long debounceMicros = 8000;  // 8 ms min gap
static const float K_IN_PPL  = 450.0f;   // DN15 ≈450 pulses/L
static const float K_OUT_PPL = 16.2f;    // DN50. 2026-10-25: recalibrated against the house water meter
                                          // (10.3 min sprinkler run used 1,520 L by refill; HA at 10.33 showed
                                          // 2,380 L, so x1.566). The 2026-07-13 value 10.33 was set against the
                                          // manual in-line meter, which reads high.

// ====================== Pump pressure sensors =========================
// OIL: digital via PC817 optocoupler OUT (LOW = oil pressure present)
// WATER: analog tap on SFS water-pressure node (0 V ≈ pump running,
//        ~1–3 V when pump is NOT building pressure)
const int OIL_PRESS_PIN_DIGITAL   = 10;  // AR2 D10 (opto OUT)
const int WATER_PRESS_PIN_ANALOG  = A1;  // AR2 A1  (direct analog, high-Z, moved from D11)

void flowInISR() {
  static unsigned long lastMicrosIn = 0;
  unsigned long now = micros();
  if (now - lastMicrosIn >= debounceMicros) {
    flowInPulseCount++;
    lastMicrosIn = now;
  }
}
void flowOutISR() {
  static unsigned long lastMicrosOut = 0;
  unsigned long now = micros();
  if (now - lastMicrosOut >= debounceMicros) {
    flowOutPulseCount++;
    lastMicrosOut = now;
  }
}

// ====================== Defaults when sensors absent ====================
static const float DEF_EXT_TEMP  = 31.0f;
static const float DEF_INT_TEMP  = 30.0f;
static const float DEF_EXT2_TEMP = 32.0f;

// ====================== Helpers =========================================
static const float WL_M_PER_V = 1.478f;  // tank level sensor: metres per volt (2026-10-25)
static const float WL_ZERO_V  = 1.254f;  // sensor voltage at zero depth (2026-10-25; to confirm)
// Tank gauge failure: the 4-20 mA loop reads ~1.25 V at zero depth (4 mA). Well below that
// (cable cut / sensor dead -> ~0 V) or pinned near the top of the ADC (short circuit) cannot
// be a real level. Flagged after 2 s, cleared after 2 s good; sent as WLFAIL so AR3 switches
// to its flow-meter estimate instead of seeing an empty tank (which forced the dry cutoff).
static const float WL_FAIL_LOW_V  = 0.90f;
static const float WL_FAIL_HIGH_V = 4.60f;
static bool wlFail = false;
static unsigned long wlBadSinceMs = 0, wlGoodSinceMs = 0;

float readWaterLevel() {
  if (SIMULATION_MODE && SIMULATE_TANK_WATER_LEVEL) return SIMULATED_WATER_LEVEL;
  int raw = analogRead(TANK_SENSOR_PIN);
  float voltage = (raw / 1023.0f) * 5.0f;
  {
    const unsigned long nowMs = millis();
    const bool bad = (voltage < WL_FAIL_LOW_V || voltage > WL_FAIL_HIGH_V);
    if (bad) { wlGoodSinceMs = 0; if (!wlBadSinceMs) wlBadSinceMs = nowMs; if (nowMs - wlBadSinceMs >= 2000) wlFail = true; }
    else     { wlBadSinceMs = 0; if (!wlGoodSinceMs) wlGoodSinceMs = nowMs; if (nowMs - wlGoodSinceMs >= 2000) wlFail = false; }
  }
  // 2026-10-25 two-point calibration (replaces the "(V/3.3)*3.2" placeholder):
  //  - sensitivity: house-water-meter refill 1,520 L = 16.0 cm real (95 L/cm,
  //    3.44 m corrugated tank) for a 0.108 V change -> 1.478 m per volt;
  //  - full tank: real depth 2.94 m (3.08 m tank, 0.14 m freeboard) at 3.243 V.
  //  => zero depth at ~1.254 V. Confirm by lifting the sensor out of the water
  //     (HA read ~1.2 m on the old placeholder at that voltage).
  // The old map read ~1.2 m with an EMPTY tank, so AR3's 0.05/0.30/0.50 m
  // dry/gel/recirc thresholds could never trigger.
  float depthM  = WL_M_PER_V * (voltage - WL_ZERO_V);
  if (depthM < 0.0f) depthM = 0.0f;
  return depthM;
}

// Water pressure node: read raw voltage at SFS input
// Pump OFF → ~1–3 V, Pump ON → ~0 V (based on measured behaviour)
float readWaterPressureVoltage() {
  int raw = analogRead(WATER_PRESS_PIN_ANALOG);
  return (raw / 1023.0f) * 5.0f;
}

// Battery sense: A2 via resistor divider from 12 V battery.
// Divider: Vadc = Vbat * (Rbottom / (Rtop + Rbottom))
// Adjust BAT_RTOP_OHMS / BAT_RBOT_OHMS to match the actual hardware.
const int   BATTERY_PIN   = A2;
const float ADC_REF_VOLTS = 5.0f;
const float ADC_COUNTS_MAX = 1023.0f;

// Calibrated: at 12.0 V battery we measured 2.53 V on A2.
// So Vbat ≈ Vadc * (12.0 / 2.53) ≈ Vadc * 4.743
// Measured in system we have a voltage input to the modules of 13.3 V requiring a further correction of 0.55846
const float BAT_ADC_TO_VBAT =  0.55846f * (12.0f / 2.53f);  // (ie 12/4.5308)
// Simple low-pass filter on battery voltage to smooth ADC noise / load spikes.
// Time constant ≈ 30 s, assuming we call readBatteryVoltage() at ~10 Hz from loop().
static float        batFiltered          = 0.0f;
static bool         batFilterInitialised = false;
static unsigned long batFilterLastMs     = 0;
static const float  BAT_FILTER_TAU_S     = 45.0f;   // approximate averaging window (seconds)

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float vadc = (raw / ADC_COUNTS_MAX) * ADC_REF_VOLTS;
  float vbat = vadc * BAT_ADC_TO_VBAT;

  unsigned long now = millis();

  if (!batFilterInitialised) {
    // First sample: initialise filter directly.
    batFiltered          = vbat;
    batFilterInitialised = true;
    batFilterLastMs      = now;
  } else {
    // Exponential moving average with time constant BAT_FILTER_TAU_S.
    unsigned long dtMs = now - batFilterLastMs;
    batFilterLastMs    = now;

    float dtSec = dtMs / 1000.0f;
    if (dtSec < 0.0f) dtSec = 0.0f;

    // alpha ≈ dt / tau, capped at 1.0 for long gaps (e.g. if loop paused).
    float alpha = dtSec / BAT_FILTER_TAU_S;
    if (alpha > 1.0f) alpha = 1.0f;

    batFiltered += alpha * (vbat - batFiltered);
  }

  return batFiltered;
}


// Oil pressure via PC817:
//   Pump OFF  → opto LED OFF  → OUT pulled HIGH
//   Pump ON   → opto LED ON   → OUT pulled LOW
// We interpret oil pressure "OK" when OUT == LOW.
bool readOilPressureOk() {
  int raw = digitalRead(OIL_PRESS_PIN_DIGITAL);
  return (raw == LOW);
}

// Fuel level sender: Wema 25, 30-240 Ohm, top-mounted, 100mm travel.
// Powered from AR2's regulated 5V rail (not the 12V system rail — purely
// resistive/passive sender, no minimum operating voltage; 5V keeps current
// and spark energy low for a sender that lives inside the diesel tank).
// Divider: 5V --[R_ref 100R]-- A3 --[Sender]-- GND. (as wired 2026-08-26 —
// full reads LOWER ADC than empty, opposite of a sender-first divider.)
// ADC_FULL/ADC_EMPTY bench-measured directly against the sender's own
// mechanical limits (not calculated) — avoids trusting linearity
// assumptions near the ends of travel.
// Bench-calibrated 2026-08-26 with sender connected to A3.
const int FUEL_ADC_FULL  = 258;   // raw ADC, sender at 100mm (tank full)
const int FUEL_ADC_EMPTY = 788;   // raw ADC, sender at mechanical bottom (35mm above tank floor)

float readFuelLevel() {
  int raw = analogRead(FUEL_SENDER_PIN);
  float span = (float)(FUEL_ADC_FULL - FUEL_ADC_EMPTY);
  if (span == 0.0f) return 0.0f;
  float pct = (float)(raw - FUEL_ADC_EMPTY) * 100.0f / span;
  return constrain(pct, 0.0f, 100.0f);
}


// AR2B: non-blocking read. First call starts a conversion and returns the
// cached values; once DALLAS_CONV_MS has elapsed a later call collects the
// results, refreshes the cache and starts the next conversion. Temperatures
// therefore update about every 0.2-0.3 s without ever stalling loop().
static const unsigned long DALLAS_CONV_MS = 200;  // 10-bit max 187.5 ms
static bool dallasConvPending = false;
static unsigned long dallasConvStartMs = 0;
static float cacheExt1 = 0.0f;   // until the first read (flagged failed by cacheTFail)
static float cacheInt  = 0.0f;
static float cacheExt2 = 0.0f;
static bool  cacheOk   = false;
static uint8_t cacheTFail = 7;   // bit0 EXT, bit1 INT, bit2 EXT2 (1 = failed); all failed until first read
static float lastGoodT[3] = {NAN, NAN, NAN};

// Valid DS18B20 reading? Rejects disconnected (-127) etc., and 85.0 C (power-on reset value)
// unless the sensor was already reading close to 85 C.
static bool tempReadingOk(float v, uint8_t i) {
  if (!(v > -55.0f && v < 125.0f)) return false;
  if (v == 85.0f && (isnan(lastGoodT[i]) || fabsf(lastGoodT[i] - 85.0f) > 10.0f)) return false;
  lastGoodT[i] = v;
  return true;
}

bool readThreeTemps(float& ext1, float& internal, float& ext2) {
  // Ensure bus is initialised (in case we disabled it or never ran setup Dallas)
  if (!dallasReady) {
    dallasInitBus("lazy-init");
  }

  const unsigned long nowMs = millis();
  if (!dallasConvPending) {
    dallas.requestTemperatures();  // returns immediately (setWaitForConversion(false))
    dallasConvStartMs = nowMs;
    dallasConvPending = true;
  }
  if (nowMs - dallasConvStartMs < DALLAS_CONV_MS) {
    ext1 = cacheExt1; internal = cacheInt; ext2 = cacheExt2;
    return cacheOk;  // conversion still running: report last values
  }
  dallasConvPending = false;  // collect now; next call starts a new conversion

  bool ok = false;

  float v0 = dallas.getTempC(ADDR_T1);
  float v1 = dallas.getTempC(ADDR_T2);
  float v2 = dallas.getTempC(ADDR_T3);

  // Basic sanity window (covers DEVICE_DISCONNECTED_C etc.)
  bool v0ok = tempReadingOk(v0, 0);
  bool v1ok = tempReadingOk(v1, 1);
  bool v2ok = tempReadingOk(v2, 2);
  cacheTFail = (v0ok ? 0 : 1) | (v1ok ? 0 : 2) | (v2ok ? 0 : 4);

  // A failed sensor is sent as 0.0 so it stands out on the dashboard (it used to be a
  // believable 31/30/32 C). AR3 ignores it anyway because TFAIL flags it.
  if (v0ok) { ext1     = v0; ok = true; } else ext1     = 0.0f;
  if (v1ok) { internal = v1; ok = true; } else internal = 0.0f;
  if (v2ok) { ext2     = v2; ok = true; } else ext2     = 0.0f;

  bool allBad = !v0ok && !v1ok && !v2ok;

  if (allBad) {
    if (dallasFailCount < 255) dallasFailCount++;
    if (dallasFailCount == DALLAS_FAIL_LIMIT) {
      Serial.println(F("[DAL] all sensors invalid; attempting bus re-init"));
      dallasInitBus("all-bad");
    }
  } else {
    // Any valid sensor reading clears the failure streak
    dallasFailCount = 0;
  }

  cacheExt1 = ext1; cacheInt = internal; cacheExt2 = ext2; cacheOk = ok;
  return ok;
}

//  --- Timing ---
unsigned long lastSendMs  = 0;   // 10 Hz broadcast
unsigned long lastPrintMs = 0;   // 1 Hz debug prints

// --- DEBUG snapshot (1 Hz print) ---
#define DEBUG_TEMPS 1
static float dbg_ext1 = 0, dbg_int = 0, dbg_ext2 = 0, dbg_wl = 0;
static float dbg_flow_in = 0.0f, dbg_flow_out = 0.0f;
static int   dbg_ember = 0;
static bool  dbg_temp_ok = false;

// ====================== Scenario Engine (sensor-only) ====================
// Ported from AR3 AutoTest (effects only). TAR3 starts/stops via UDP "SIMCTRL:*".
static bool     FS_scenActive  = false;
static uint8_t  FS_scenId      = 0;
static uint32_t FS_scenStepMs  = 0;
static size_t   FS_scenIdx     = 0;
static float    FS_speed       = 1.00f;  // time-scale (AR3 set to ≈1.0f)

// Live simulated values (override outgoing UDP when scenario active)
static float FS_sim_EXT           = 58.0f;
static float FS_sim_INT           = 55.0f;
static float FS_sim_EXT2          = 60.0f;
static float FS_sim_WL_m          = 3.00f;
static float FS_sim_FLOW_OUT_Lpm  = 0.0f;
static int   FS_sim_EMBER         = 0;
static float FS_sim_BAT           = 13.5f;   // simulated battery voltage
static uint8_t FS_sim_TFAIL       = 0;       // simulated sensor-failure bits (as TFAIL)
static uint8_t FS_sim_WLFAIL      = 0;       // simulated tank-gauge failure

static void FS_startScenario(uint8_t id, float speed=0.05f) {
  FS_scenActive  = true;
  FS_scenId      = id;
  FS_speed       = speed;
  FS_scenIdx     = 0;
  FS_scenStepMs  = millis();

  FS_sim_EXT          = 58.0f;
  FS_sim_INT          = 30.0f;
  FS_sim_EXT2         = 60.0f;
  FS_sim_WL_m         = 3.00f;
  FS_sim_FLOW_OUT_Lpm = 0.0f;
  FS_sim_EMBER        = 0;
  FS_sim_BAT          = 13.5f;
  FS_sim_TFAIL        = 0;
  FS_sim_WLFAIL       = 0;

  Serial.print(F("[SCEN] START id=")); Serial.print(id);
  Serial.print(F(" speed=")); Serial.println(FS_speed,3);
}

static void FS_stopScenario() {
  FS_scenActive = false;
  FS_scenId     = 0;
  Serial.println(F("[SCEN] STOP"));
}

// ------------- Scenario 1 (AR3 AutoTest1 sensor-equivalent) -------------
static const float FS_SCEN_NC = -9999.0f;  // sentinel = "no change"

typedef struct {
  float dtSec;    // hold time in seconds (pre-scale by FS_speed)
  float ext;
  float internal;
  float wl;
  float batt;
  float flowOut;
} FS_Scen1Step;

// Scenario 1 – “Extended test” ported from AR3 AUTOTEST1_CMDS
static const FS_Scen1Step FS_scen1[] = {
  // dtSec,  EXT,          INT,          WL,         BAT,    FLOW_OUT
  { 30.0f,  35.0f,         35.0f,        FS_SCEN_NC, 13.5f,  165.0f },   // flow on, good batt
  { 30.0f,  35.0f,         47.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // INT crosses thermostat threshold
  { 30.0f,  49.0f,         37.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // INT at thermostat threshold
  { 30.0f,  66.0f,         47.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // EXT 66
  { 30.0f,  66.0f,         55.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // EXT 66
  { 30.0f,  66.0f,         55.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // EXT 66
  { 30.0f,  66.0f,         55.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // EXT 66 Inter off
  { 30.0f,  79.0f,         FS_SCEN_NC,   FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // EXT 79
  { 30.0f,  81.0f,         FS_SCEN_NC,   FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // EXT 81
  { 30.0f,  95.0f,         FS_SCEN_NC,   FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // EXT 95 (peak)
  { 30.0f,  88.0f,         53.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // cooldown start (force INT drop >=2)
  { 30.0f,  FS_SCEN_NC,    FS_SCEN_NC,   FS_SCEN_NC, 11.6f,  FS_SCEN_NC },     // batt drop
  { 30.0f,  75.0f,         FS_SCEN_NC,   FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // still hot
  { 30.0f,  65.0f,         FS_SCEN_NC,   FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // cooling
  { 30.0f,  58.0f,         FS_SCEN_NC,   FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // near baseline
  { 30.0f,  FS_SCEN_NC,    FS_SCEN_NC,   0.10f,      FS_SCEN_NC, FS_SCEN_NC }, // WL low (gel zone)
  { 30.0f,  FS_SCEN_NC,    FS_SCEN_NC,   0.04f,      FS_SCEN_NC, FS_SCEN_NC }, // WL very low (shutdown)
  { 30.0f,  54.0f,         FS_SCEN_NC,   FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }, // cool external
  { 30.0f,  FS_SCEN_NC,    52.0f,        FS_SCEN_NC, FS_SCEN_NC, FS_SCEN_NC }  // cool internal
};

static const size_t FS_SCEN1_LEN = sizeof(FS_scen1) / sizeof(FS_scen1[0]);

static void FS_tickScenario1(uint32_t now) {
  if (!FS_scenActive || FS_scenId != 1) return;
  if (FS_scenIdx >= FS_SCEN1_LEN) {
    Serial.println(F("[SCEN1] complete"));
    FS_stopScenario();
    return;
  }

  const FS_Scen1Step &step = FS_scen1[FS_scenIdx];
  float msF = step.dtSec * 1000.0f * FS_speed;
  if (msF < 50.0f) msF = 50.0f;
  uint32_t stepGapMs = (uint32_t)msF;

  if ((now - FS_scenStepMs) < stepGapMs) return;
  FS_scenStepMs = now;

  // Apply this step’s changes
  if (step.ext      > FS_SCEN_NC) FS_sim_EXT          = step.ext;
  if (step.internal > FS_SCEN_NC) FS_sim_INT          = step.internal;
  if (step.wl       > FS_SCEN_NC) FS_sim_WL_m         = step.wl;
  if (step.batt     > FS_SCEN_NC) FS_sim_BAT          = step.batt;
  if (step.flowOut  > FS_SCEN_NC) FS_sim_FLOW_OUT_Lpm = step.flowOut;

  // Mirror EXT into EXT2 to emulate AR3’s paired sensors
  FS_sim_EXT2 = FS_sim_EXT;

  Serial.print(F("[SCEN1] step "));
  Serial.print(FS_scenIdx);
  Serial.print(F(" t="));
  Serial.print(step.dtSec, 1);
  Serial.print(F("s EXT="));   Serial.print(FS_sim_EXT, 1);
  Serial.print(F(" INT="));    Serial.print(FS_sim_INT, 1);
  Serial.print(F(" WL="));     Serial.print(FS_sim_WL_m, 2);
  Serial.print(F(" BAT="));    Serial.print(FS_sim_BAT, 2);
  Serial.print(F(" FLOW="));   Serial.println(FS_sim_FLOW_OUT_Lpm, 1);

  FS_scenIdx++;
}

// ------------- Scenario 2: temperature sensors fail during a fire -------------
// Exercises AR3's sensor fallback: EXT fails (AR3 uses EXT2), both outside sensors fail
// (AR3 uses INT + last outside-inside offset), then all three fail (timed fallback
// stages), then the sensors recover. Hold times are seconds before FS_speed scaling.
typedef struct { float dtSec; float ext; float internal; uint8_t tfail; const char* note; } FS_Scen2Step;
static const FS_Scen2Step FS_scen2[] = {
  {  30.0f, 35.0f, 35.0f, 0, "all sensors OK, cool" },
  {  60.0f, 66.0f, 47.0f, 0, "EXT 66 -> warm-up B (3/7)" },
  {  60.0f, 66.0f, 47.0f, 1, "EXT fails -> EXT2 used" },
  {  60.0f, 66.0f, 47.0f, 5, "both outside fail -> INT+offset (66)" },
  {  60.0f, 66.0f, 58.0f, 5, "INT 58 -> proxy 77 -> warm-up C (5/3)" },
  { 600.0f, 77.0f, 58.0f, 7, "ALL fail -> timed fallback from C" },
  {  90.0f, 65.0f, 50.0f, 0, "sensors back -> normal control" },
};
static const size_t FS_SCEN2_LEN = sizeof(FS_scen2) / sizeof(FS_scen2[0]);

static void FS_tickScenario2(uint32_t now) {
  if (!FS_scenActive || FS_scenId != 2) return;
  if (FS_scenIdx > 0) {
    float msF = FS_scen2[FS_scenIdx - 1].dtSec * 1000.0f * FS_speed;
    if ((now - FS_scenStepMs) < (uint32_t)msF) return;
  }
  if (FS_scenIdx >= FS_SCEN2_LEN) {
    Serial.println(F("[SCEN2] complete"));
    FS_stopScenario();
    return;
  }
  const FS_Scen2Step &st = FS_scen2[FS_scenIdx];
  FS_scenStepMs = now;
  FS_sim_EXT = st.ext;  FS_sim_EXT2 = st.ext;  FS_sim_INT = st.internal;
  FS_sim_TFAIL = st.tfail;
  if (FS_scenIdx == 0) { FS_sim_FLOW_OUT_Lpm = 165.0f; FS_sim_BAT = 13.5f; }
  Serial.print(F("[SCEN2] step ")); Serial.print(FS_scenIdx);
  Serial.print(F(" TFAIL=")); Serial.print(st.tfail);
  Serial.print(F(" : ")); Serial.println(st.note);
  FS_scenIdx++;
}

// ------------- Scenario 3: one steady intermittent cycle -------------
// Outside held at 66 C (warm-up B, 3 min on / 7 min off, via the recirculation valve) for
// 11 minutes so a whole ON and OFF phase can be watched: pump on throughout, P1 off in the
// ON phase (sprinklers) and on in the OFF phase (recirculate).
static void FS_tickScenario3(uint32_t now) {
  if (!FS_scenActive || FS_scenId != 3) return;
  if (FS_scenIdx == 0) {
    FS_sim_EXT = 66.0f; FS_sim_EXT2 = 66.0f; FS_sim_INT = 40.0f;
    FS_sim_FLOW_OUT_Lpm = 147.0f; FS_sim_BAT = 13.5f; FS_sim_TFAIL = 0;
    FS_scenStepMs = now; FS_scenIdx = 1;
    Serial.println(F("[SCEN3] EXT 66 C held 11 min -> warm-up B 3/7 via recirc valve"));
    return;
  }
  if ((now - FS_scenStepMs) >= (uint32_t)(660000.0f * FS_speed)) {
    Serial.println(F("[SCEN3] complete"));
    FS_stopScenario();
  }
}

// ------------- Scenario 4: tank gauge (and flow meter) fail during a fire -------------
// Outside 85 C (continuous). Flow is exaggerated (1000 L/min) so the level falls through the
// water bands in minutes. Gauge dies at 0.80 m -> AR3 estimates from flow; flow meter dies
// with the pump on -> AR3 assumes 147 L/min; meter back; estimate reaches recirc, gel and
// dry cutoff; gauge recovers reading 0.04 m.
typedef struct { float dtSec; float wl; float flow; uint8_t wlfail; const char* note; } FS_Scen4Step;
static const FS_Scen4Step FS_scen4[] = {
  {  30.0f, 0.80f, 1000.0f, 0, "EXT 85 continuous, gauge OK at 0.80 m" },
  {  60.0f, 0.00f, 1000.0f, 1, "GAUGE FAILS (reads 0) -> estimate from flow" },
  {  60.0f, 0.00f,    0.0f, 1, "flow meter fails, pump on -> assume 147 L/min" },
  { 240.0f, 0.00f, 1000.0f, 1, "meter back -> estimate falls through 0.50 and 0.30 m" },
  { 180.0f, 0.00f, 1000.0f, 1, "estimate reaches 0.05 m -> dry cutoff" },
  {  60.0f, 0.04f,    0.0f, 0, "gauge recovers, reads 0.04 m" },
};
static const size_t FS_SCEN4_LEN = sizeof(FS_scen4) / sizeof(FS_scen4[0]);

static void FS_tickScenario4(uint32_t now) {
  if (!FS_scenActive || FS_scenId != 4) return;
  if (FS_scenIdx > 0) {
    float msF = FS_scen4[FS_scenIdx - 1].dtSec * 1000.0f * FS_speed;
    if ((now - FS_scenStepMs) < (uint32_t)msF) return;
  }
  if (FS_scenIdx >= FS_SCEN4_LEN) { Serial.println(F("[SCEN4] complete")); FS_stopScenario(); return; }
  const FS_Scen4Step &st = FS_scen4[FS_scenIdx];
  FS_scenStepMs = now;
  FS_sim_EXT = 85.0f; FS_sim_EXT2 = 85.0f; FS_sim_INT = 40.0f; FS_sim_BAT = 13.5f; FS_sim_TFAIL = 0;
  FS_sim_WL_m = st.wl; FS_sim_FLOW_OUT_Lpm = st.flow; FS_sim_WLFAIL = st.wlfail;
  Serial.print(F("[SCEN4] step ")); Serial.print(FS_scenIdx);
  Serial.print(F(" : ")); Serial.println(st.note);
  FS_scenIdx++;
}

// ------------- Scenario 5 (existing, sensor-focused) ---------------------
static void FS_tickScenario5(uint32_t now) {
  if (!FS_scenActive || FS_scenId != 5) return;
  const uint32_t stepGapMs = (FS_scenIdx < 6)
                           ? (uint32_t)(100 * FS_speed)
                           : (uint32_t)(30000 * FS_speed);
  if ((now - FS_scenStepMs) < stepGapMs) return;
  FS_scenStepMs = now;

  switch (FS_scenIdx++) {
    case 0:  Serial.println(F("[SCEN5] ON")); break;
    case 1:  Serial.println(F("[SCEN5] WL fallback path (sensor fail emulate)")); break;
    case 2:  FS_sim_FLOW_OUT_Lpm = 180.0f; Serial.println(F("[SCEN5] FLOW_OUT=180 L/min")); break;
    case 3:  /* wait */ break;
    case 4:  /* print */ break;
    case 5:  FS_sim_FLOW_OUT_Lpm = 0.0f; Serial.println(F("[SCEN5] FLOW_OUT=0")); break;
    case 6:  /* flow meter off → hold 0 */ break;
    case 7:  /* wait */ break;
    case 8:  /* print */ break;
    case 9:  Serial.println(F("[SCEN5] HB off is AR3/AR1 domain")); break;
    default: Serial.println(F("[SCEN5] complete")); FS_stopScenario(); break;
  }

  // Background trend each step: temp up, WL down proportional to flow
  FS_sim_EXT  += 1.0f;
  FS_sim_EXT2  = FS_sim_EXT;
  FS_sim_INT   = max(50.0f, FS_sim_INT - 0.2f);

  float dLpm = (FS_sim_FLOW_OUT_Lpm > 0.1f) ? FS_sim_FLOW_OUT_Lpm : 5.0f; // drip if no flow
  const float tankDiaM = 3.5f;
  const float areaM2   = 3.14159f * (tankDiaM*0.5f) * (tankDiaM*0.5f);
  float litresPerStep  = dLpm * (stepGapMs / 60000.0f);
  float mDrop          = (litresPerStep / 1000.0f) / areaM2;
  FS_sim_WL_m = max(0.0f, FS_sim_WL_m - mDrop);
}

// ====================== setup ====================
void setup() {
  resetCause = readResetCause();
  selfRestartEnabled = (EEPROM.read(EE_SR_ENABLED) != 0);
  Serial.begin(9600);
  delay(200);
  Serial.print(F("[BOOT] reset cause "));
  Serial.print(resetCause);
  Serial.print(F(", connection restarts "));
  Serial.println(selfRestartEnabled ? F("on") : F("off"));

  // I/O setup
  pinMode(TANK_SENSOR_PIN, INPUT);
  pinMode(FLOW_IN_PIN,  INPUT_PULLUP);
  pinMode(FLOW_OUT_PIN, INPUT_PULLUP);
  pinMode(EMBER_PIN,    INPUT_PULLUP);
  pinMode(OIL_PRESS_PIN_DIGITAL, INPUT_PULLUP);  // default HIGH if nothing connected

  attachInterrupt(digitalPinToInterrupt(FLOW_IN_PIN),  flowInISR,  FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_OUT_PIN), flowOutISR, FALLING);

  // OneWire/Dallas
  delay(500);                 // allow sensor 5V + bus to stabilise
  dallasInitBus("power-up");  // uses resilience state (dallasReady / failCount)
  Serial.print("Dallas bus on D");
  Serial.print(TEMP_ONEWIRE_PIN);
  Serial.println(" initialised");

  // WiFi AP + UDP
  // Wait at most 30 s for the access point (this used to wait forever, before the watchdog
  // starts, if the WiFi chip never came up). If it fails, the connection check restarts AR2.
  WiFi.beginAP(apSSID, apPassword);
  {
    const unsigned long t0 = millis();
    while (WiFi.status() != WL_AP_LISTENING && millis() - t0 < 30000UL) {
      delay(200);
      Serial.print('.');
    }
  }
  Serial.println();
  Serial.print("AP up: "); Serial.println(apSSID);
  Serial.print("AP IP: "); Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);
  Serial.print("UDP port: "); Serial.println(UDP_PORT);

  // T1-AR2: Hardware watchdog — reboot if loop freezes for >8192ms
  // AR2B: 8192 ms exceeds the RA4M1 maximum (5592 ms), so WDT.begin(8192)
  // always failed and AR2 had no watchdog. 5000 ms is valid.
  if (WDT.begin(5000)) {
    Serial.println(F("[WDT] Watchdog initialized: 5000ms"));
    WDT.refresh();
  } else {
    Serial.println(F("[WDT] Watchdog init failed"));
  }
}

// ====================== loop ====================
void loop() {
  WDT.refresh();  // T1-AR2: pet the watchdog — must complete within 5000ms (AR2B)
  unsigned long now = millis();
  connectionCheck(now);

  // USB test commands: WDTTEST freezes the loop (watchdog restart); NOCLIENT pretends AR3 has
  // been away for 25 min; MUTE / UNMUTE stop / restore all UDP with AR3 (tests both checks).
  {
    static char ucmd[12];
    static uint8_t un = 0;
    while (Serial.available()) {
      const char c = Serial.read();
      if (c == '\r' || c == '\n') {
        ucmd[un] = 0;
        if (un && strcmp(ucmd, "WDTTEST") == 0) {
          Serial.println(F("[USB] WDTTEST: freezing loop - the watchdog should restart AR2"));
          while (true) {}
        } else if (un && strcmp(ucmd, "NOCLIENT") == 0) {
          Serial.println(F("[USB] NOCLIENT: last client time set 25 min back"));
          lastClientMs = now - SR_NO_CLIENT_MS;
        } else if (un && strcmp(ucmd, "MUTE") == 0) {
          Serial.println(F("[USB] MUTE: no UDP to or from AR3 until UNMUTE or restart"));
          testMute = true;
        } else if (un && strcmp(ucmd, "UNMUTE") == 0) {
          Serial.println(F("[USB] UNMUTE"));
          testMute = false;
        }
        un = 0;
      } else if (un < sizeof(ucmd) - 1) {
        ucmd[un++] = (c >= 'a' && c <= 'z') ? c - 32 : c;
      }
    }
  }

  // ---- Control channel: TAR3→AR2 scenario control on same UDP port ----
  {
    int clen = udp.parsePacket();
    if (clen > 0 && testMute) {
      char junk[96];
      udp.read(junk, sizeof(junk));  // muted test: ignore AR3
    } else if (clen > 0) {
      char cbuf[96];
      int n = udp.read(cbuf, sizeof(cbuf) - 1);
      if (n > 0) {
        cbuf[n] = 0;
        String ctrl = String(cbuf);
        ctrl.trim();
        // Only AR3's commands count as "connected" (not our own broadcasts, if they echo back).
        if (ctrl.startsWith("SRCTRL:") || ctrl.startsWith("SIMCTRL:")) noteClient();
        if (ctrl.startsWith("SRCTRL:")) {
          const bool en = ctrl.substring(7).toInt() != 0;
          if (en != selfRestartEnabled) {
            selfRestartEnabled = en;
            EEPROM.update(EE_SR_ENABLED, en ? 1 : 0);
            Serial.print(F("[RESTART] connection restarts "));
            Serial.println(en ? F("on") : F("off"));
          }
        } else if (ctrl.startsWith("SIMCTRL:")) {
          if (ctrl.startsWith("SIMCTRL:START:")) {
            int id = ctrl.substring(14).toInt();
            FS_startScenario((uint8_t)id, FS_speed);
          } else if (ctrl.startsWith("SIMCTRL:SPEED:")) {
            float sp = ctrl.substring(14).toFloat();
            if (sp > 0.0f) {
              FS_speed = sp;
              Serial.print(F("[SCEN] speed="));
              Serial.println(FS_speed, 3);
            }
          } else if (ctrl.startsWith("SIMCTRL:STOP")) {
            FS_stopScenario();
          }
        }
      }
    }
  }

  // ---- Tick active scenario (updates simulated values & step machine) ----
  if (FS_scenActive) {
    if      (FS_scenId == 1) FS_tickScenario1(now);
    else if (FS_scenId == 2) FS_tickScenario2(now);
    else if (FS_scenId == 3) FS_tickScenario3(now);
    else if (FS_scenId == 4) FS_tickScenario4(now);
    else if (FS_scenId == 5) FS_tickScenario5(now);
    // else: other scenarios go here
  }

  // ===== 10 Hz sampling & UDP broadcast =====
  if (now - lastSendMs >= 100) {
    const unsigned long elapsedMs = now - lastSendMs;  // actual gap, not assumed 100ms
    lastSendMs = now;

    // Temps
    float tExt1 = DEF_EXT_TEMP;
    float tInt  = DEF_INT_TEMP;
    float tExt2 = DEF_EXT2_TEMP;
    if (!SIMULATION_MODE) {
      dbg_temp_ok = readThreeTemps(tExt1, tInt, tExt2);
    } else {
      dbg_temp_ok = true;
    }

    // Water level
    float wl = readWaterLevel();

    // Flow meters (convert pulses/0.1s → Hz → L/s → L/min)
    float flowInLpm  = 0.0f;
    float flowOutLpm = 0.0f;

    if (SIMULATION_MODE && SIMULATE_FLOW) {
      flowOutLpm = SIMULATED_FLOW_RATE;
    } else {
      noInterrupts();
      unsigned long pIn  = flowInPulseCount;  flowInPulseCount  = 0;
      unsigned long pOut = flowOutPulseCount; flowOutPulseCount = 0;
      interrupts();

      const float dtSec = elapsedMs / 1000.0f;
      const float hzIn  = pIn  / dtSec;
      const float hzOut = pOut / dtSec;

      if (K_IN_PPL  > 0.0f) flowInLpm  = (hzIn  / K_IN_PPL ) * 60.0f;
      if (K_OUT_PPL > 0.0f) flowOutLpm = (hzOut / K_OUT_PPL) * 60.0f;

      if (flowInLpm  < 0.05f) flowInLpm  = 0.0f;
      if (flowOutLpm < 0.05f) flowOutLpm = 0.0f;
    }

    // Ember (active LOW => ember present when LOW)
    int ember = (digitalRead(EMBER_PIN) == LOW) ? 1 : 0;

    // ---- Scenario overrides (apply to outgoing values only) ----
    uint8_t tfail = SIMULATION_MODE ? 0 : cacheTFail;
    uint8_t wlfail = SIMULATION_MODE ? 0 : (wlFail ? 1 : 0);
    if (FS_scenActive) {
      tfail      = FS_sim_TFAIL;
      wlfail     = FS_sim_WLFAIL;
      tExt1      = FS_sim_EXT;
      tInt       = FS_sim_INT;
      tExt2      = FS_sim_EXT2;
      wl         = FS_sim_WL_m;
      flowOutLpm = FS_sim_FLOW_OUT_Lpm;  // FLOW mirrors OUT below
      ember      = FS_sim_EMBER;
    }

    // Decide battery value:
    //  - FS_scenActive: take battery from scenario
    //  - SIMULATION_MODE+SIMULATE_BATTERY: fixed simulated battery
    //  - else: real measurement from A2 divider
    float bat;
    if (FS_scenActive) {
      bat = FS_sim_BAT;
    } else if (SIMULATION_MODE && SIMULATE_BATTERY) {
      bat = SIMULATED_BATTERY_VOLTAGE;
    } else {
      bat = readBatteryVoltage();
    }

    // Pump pressure sensors taken from SFS control box nodes via AR2
    float waterPressV = readWaterPressureVoltage();  // 0–5 V at analog WATER_PRESS_PIN_ANALOG
    bool  oilOk       = readOilPressureOk();         // true when oil pressure switch says "engine running"
    float fuelPct     = readFuelLevel();             // 0-100%, calibrated against sender's own travel limits

    // DEBUG snapshot (reflect final values)
    dbg_ext1     = tExt1;
    dbg_int      = tInt;
    dbg_ext2     = tExt2;
    dbg_wl       = wl;
    dbg_flow_in  = flowInLpm;
    dbg_flow_out = flowOutLpm;
    dbg_ember    = ember;

    // Build and send UDP line (TAR3 expects '|' KV tokens)
    String line = String("EXT_TEMP:")   + String(tExt1, 2)
                 + "|INT_TEMP:"         + String(tInt, 2)
                 + "|EXT2_TEMP:"        + String(tExt2, 2)
                 + "|FLOW_IN:"          + String(flowInLpm, 2)
                 + "|FLOW_OUT:"         + String(flowOutLpm, 2)
                 + "|FLOW:"             + String(flowOutLpm, 2)  // Legacy field = OUT
                 + "|EMBER:"            + String(ember)
                 + "|WATER_LEVEL:"      + String(wl, 2)
                 + "|BAT:"              + String(bat, 2)
                 + "|OIL_OK:"           + String(oilOk ? 1 : 0)
                 + "|WATER_PV:"         + String(waterPressV, 2)
                 + "|FUEL:"             + String(fuelPct, 2)
                 + "|TFAIL:"            + String(tfail)
                 + "|WLFAIL:"           + String(wlfail)
                 + "|RST:"              + String(resetCause)
                 + (FS_scenActive ? "|SIM:1" : "");  // AR3 ignores SIM packets outside a test

    if (!testMute) {
      udp.beginPacket(IPAddress(255,255,255,255), UDP_PORT);
      udp.print(line);
      udp.endPacket();
    }

#if DEBUG_TEMPS
    static uint8_t oncePer = 0;
    if (++oncePer >= 10) {  // ~1 Hz
      Serial.print(F("UDP SENT: "));
      Serial.println(line);
      oncePer = 0;
    }
#endif
  }

  // ===== 1 Hz console status =====
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    Serial.print("HB ");
    Serial.print(now / 1000);
    Serial.print("s, IP:");
    Serial.println(WiFi.localIP());

#if DEBUG_TEMPS == 1
    Serial.print(F("AR2: T1="));        Serial.print(dbg_ext1, 2);
    Serial.print(F("°C, T2="));         Serial.print(dbg_int, 2);
    Serial.print(F("°C, T3="));         Serial.print(dbg_ext2, 2);
    Serial.print(F("°C | WL="));        Serial.print(dbg_wl, 2);
    Serial.print(F(" m | FLOW_IN="));   Serial.print(dbg_flow_in, 2);
    Serial.print(F(" L/min | FLOW_OUT=")); Serial.print(dbg_flow_out, 2);
    Serial.print(F(" L/min | EMBER=")); Serial.print(dbg_ember);
    Serial.print(F(" | TEMP_OK="));     Serial.println(dbg_temp_ok ? F("Y") : F("N"));
#endif
  }
}
