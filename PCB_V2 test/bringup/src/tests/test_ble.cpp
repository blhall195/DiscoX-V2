// PCB V2 bring-up test: BLE (SAP6 protocol, long-range Coded PHY)
//
// Ports the hardware-proven long-range recipe from the V1 DiscoX firmware
// (github.com/blhall195/Mr_Zappy), which took real effort to get right — the
// order of operations matters:
//   1. Bluefruit.configPrphConn(..., 24, ...) BEFORE begin() — the default
//      event length of 3 (3.75 ms) is too short for Coded PHY; without this
//      sd_ble_gap_phy_update() fails with 0x0013 NRF_ERROR_RESOURCES.
//   2. The global Bluefruit library patch (BLEConnection.cpp ~line 392 answers
//      PHY update requests with BLE_GAP_PHY_CODED, not AUTO). Verified present
//      2026-07-07; a PlatformIO framework update silently reverts it — if the
//      status line says "1 Mbps" on a phone that did Coded before, check there
//      FIRST.
//   3. PHY request retried every 500 ms from loop() — the SoftDevice is busy
//      right after connection; calling from the connect callback always fails.
//   4. conn->setTxPower(+8) per connection — Bluefruit.setTxPower() only
//      covers advertising, data packets would stay at 0 dBm.
//
// New in this test (attacking the reported connectivity complaints):
//   - Supervision timeout raised to 6 s (PPCP + active request): at the edge
//     of Coded PHY range, brief signal fades outlive the ~2 s default and the
//     phone declares the link dead. 6 s rides them out.
//   - PHY watchdog: DiscoX requested Coded once per connection — if the phone
//     later renegotiated down to 1M it stayed there. Re-request on fallback,
//     capped at 3 attempts per connection (phones without Coded PHY — all
//     iPhones — would otherwise be spammed forever).
//   - RSSI + link status printed every 5 s while connected: walk away from
//     the board and watch the margin; distinguishes weak radio (RSSI < -90)
//     from protocol stalls (good RSSI, pending stuck > 0).
//
// Live mode — buttons on H3 send pretend readings (verify on the phone app):
//   BUTTON1: fixed leg  az=123.4  inc=-12.3  dist=4.56  (recognisable values)
//   BUTTON2: random leg
//   BUTTON3: burst of 5 random legs (exercises queue + sequence-bit ACK path)
//   BUTTON4: print status dump (PHY, RSSI, conn interval, counters)
//
// Boot self-test can't verify RF — [PASS] here means the stack accepted the
// long-range configuration. Real proof is the "Coded PHY: YES" line + legs
// arriving in the phone app (nRF Connect on Android for raw inspection;
// iOS connects but stays on 1 Mbps).

#include <Arduino.h>
#include <bluefruit.h>

#include "drivers/sap6_ble.h"
#include "pins_v2.h"

static const char DEVICE_NAME[] =
    "SAP6_V2_Test"; // SAP6_ prefix: some apps filter on it
static constexpr int8_t BLE_TX_POWER = 8;        // +8 dBm, nRF52840 maximum
static constexpr uint16_t SUP_TIMEOUT_MS = 6000; // ride out fades at range
static constexpr uint32_t PHY_RETRY_MS = 500;
static constexpr uint32_t PHY_WATCHDOG_MS = 10000;
static constexpr uint8_t PHY_WATCHDOG_MAX = 3; // re-requests per connection
static constexpr uint32_t STATUS_PERIOD_MS = 5000;

// --- Buttons (same debounce scheme as test_buttons.cpp) ---
struct Button {
  const char *name;
  uint8_t pin;
};
static const Button BUTTONS[] = {
    {"BUTTON1", PIN_BUTTON1},
    {"BUTTON2", PIN_BUTTON2},
    {"BUTTON3", PIN_BUTTON3},
    {"BUTTON4", PIN_BUTTON4},
};
static constexpr size_t NUM_BUTTONS = sizeof(BUTTONS) / sizeof(BUTTONS[0]);
static bool btnPressed[NUM_BUTTONS] = {false};

// --- Connection / PHY state machine (mirrors DiscoX main.cpp) ---
static bool lastConnected = false;
static uint32_t lastConnCheck = 0;
static bool phyRequestPending = false; // initial Coded PHY request w/ retry
static uint32_t phyRequestTime = 0;
static bool phyCheckPending = false; // delayed "which PHY did we get" report
static uint32_t phyCheckTime = 0;
static uint32_t phyWatchdogTime = 0;
static uint8_t phyWatchdogCount = 0;
static uint8_t lastReportedPhy = 0xFF;
static uint32_t lastStatusTime = 0;

static int passCount = 0;
static int failCount = 0;
static bool bleInited = false; // Bluefruit.begin() is once-only

static void report(const char *name, bool ok, const char *detail) {
  if (ok) {
    passCount++;
  } else {
    failCount++;
  }
  Serial.print(ok ? "[PASS] " : "[FAIL] ");
  Serial.print(name);
  if (detail && detail[0]) {
    Serial.print(" — ");
    Serial.print(detail);
  }
  Serial.println();
}

static const char *phyName(uint8_t phy) {
  switch (phy) {
  case BLE_GAP_PHY_1MBPS:
    return "1 Mbps";
  case BLE_GAP_PHY_2MBPS:
    return "2 Mbps";
  case BLE_GAP_PHY_CODED:
    return "Coded (long range)";
  default:
    return "unknown";
  }
}

static const char *commandName(int cmd) {
  switch (cmd) {
  case SAP6_CMD_ACK0:
    return "ACK0";
  case SAP6_CMD_ACK1:
    return "ACK1";
  case SAP6_CMD_STOP_CAL:
    return "STOP_CAL";
  case SAP6_CMD_START_CAL:
    return "START_CAL";
  case SAP6_CMD_DEVICE_OFF:
    return "DEVICE_OFF";
  case SAP6_CMD_LASER_ON:
    return "LASER_ON";
  case SAP6_CMD_LASER_OFF:
    return "LASER_OFF";
  case SAP6_CMD_TAKE_SHOT:
    return "TAKE_SHOT";
  default:
    return "unknown";
  }
}

static void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(sap6.service());
  Bluefruit.Advertising.addTxPower();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(false); // restarted from loop()
  Bluefruit.Advertising.setInterval(32, 244);       // fast, then slow
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0); // 0 = forever
}

static void runTests() {
  char detail[96];

  if (!bleInited) {
    // 1. Event length MUST go in before begin() — Coded PHY needs >= 24
    Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, 24,
                             BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT,
                             BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    bool ok = Bluefruit.begin();
    report("Bluefruit.begin (event length 24)", ok,
           ok ? "SoftDevice S140 up" : "begin() failed");
    if (!ok) {
      return; // nothing below can work
    }
    bleInited = true;

    Bluefruit.autoConnLed(false);
    Bluefruit.setName(DEVICE_NAME);
    Bluefruit.Periph.clearBonds(); // V1 lesson: stale bonds break re-pairing

    // 2. Connection event extension — lets the SoftDevice stretch a
    //    connection event beyond the nominal length, recovering throughput
    //    lost to Coded PHY's 8x on-air time
    ble_opt_t opt = {};
    opt.common_opt.conn_evt_ext.enable = 1;
    ok = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt) == NRF_SUCCESS;
    report("connection event extension", ok, "");

    // 3. Preferred supervision timeout (phone reads PPCP on connect)
    ok = Bluefruit.Periph.setConnSupervisionTimeoutMS(SUP_TIMEOUT_MS);
    snprintf(detail, sizeof(detail), "%u ms (default ~2 s drops at range)",
             SUP_TIMEOUT_MS);
    report("supervision timeout PPCP", ok, detail);

    // 4. Advertising TX power (connection TX power is per-conn, set later)
    ok = Bluefruit.setTxPower(BLE_TX_POWER);
    snprintf(detail, sizeof(detail), "+%d dBm advertising", BLE_TX_POWER);
    report("advertising TX power", ok, detail);

    // 5. SAP6 GATT service
    ok = sap6.begin();
    report("SAP6 GATT service", ok, "proto name + command + 17-byte leg data");

    // 6. Advertising
    startAdvertising();
    ok = Bluefruit.Advertising.isRunning();
    snprintf(detail, sizeof(detail), "\"%s\", SAP6 service UUID in adv data",
             DEVICE_NAME);
    report("advertising started", ok, detail);
  } else {
    // BLE stack can't re-begin — re-report live state instead
    report("Bluefruit stack", true,
           "already running (cannot re-init without reset)");
    bool connected = Bluefruit.connected();
    report(connected ? "connection" : "advertising",
           connected || Bluefruit.Advertising.isRunning(),
           connected ? "phone connected" : "waiting for phone");
  }

  Serial.println();
  Serial.print("Result: ");
  Serial.print(passCount);
  Serial.print(" passed, ");
  Serial.print(failCount);
  Serial.println(" failed");
  Serial.println(
      ">>> Stack checks done — RANGE/PHY NEEDS A PHONE TO CONFIRM <<<");
  Serial.println();
  Serial.print("Connect with nRF Connect (Android) or the survey app to \"");
  Serial.print(DEVICE_NAME);
  Serial.println("\".");
  Serial.println("Expect: 'PHY update request: OK' then 'Coded PHY: YES' "
                 "(Android w/ BLE 5).");
  Serial.println(
      "iOS has no Coded PHY — '1 Mbps' there is normal, not a failure.");
  Serial.println();
  Serial.println(
      "Buttons: 1=fixed leg  2=random leg  3=burst of 5  4=status dump");
  Serial.println("Send any character to re-print this report.");
  Serial.println();
}

static void printStatus() {
  Serial.print("  [status] ");
  if (!Bluefruit.connected()) {
    Serial.print("disconnected, advertising ");
    Serial.println(Bluefruit.Advertising.isRunning() ? "ON" : "OFF (!)");
    return;
  }
  BLEConnection *conn = Bluefruit.Connection(Bluefruit.connHandle());
  if (!conn) {
    Serial.println("connected but no connection object (!)");
    return;
  }
  Serial.print("PHY: ");
  Serial.print(phyName(conn->getPHY()));
  Serial.print(", RSSI: ");
  Serial.print(conn->getRssi());
  Serial.print(" dBm, interval: ");
  Serial.print(conn->getConnectionInterval() * 1.25f, 2);
  Serial.print(" ms, sent/acked/resent: ");
  Serial.print(sap6.sentCount());
  Serial.print("/");
  Serial.print(sap6.ackedCount());
  Serial.print("/");
  Serial.print(sap6.resendCount());
  Serial.print(", pending: ");
  Serial.println(sap6.pending());
}

static void sendLeg(float az, float inc, float dist) {
  Serial.print("  TX leg: az=");
  Serial.print(az, 1);
  Serial.print(" inc=");
  Serial.print(inc, 1);
  Serial.print(" dist=");
  Serial.print(dist, 2);
  Serial.print("  (pending ");
  Serial.print(sap6.pending() + 1);
  Serial.println(")");
  sap6.sendData(az, inc, dist);
}

static void randomLeg() {
  sendLeg(random(0, 3600) / 10.0f, random(-900, 900) / 10.0f,
          random(50, 3000) / 100.0f);
}

static void onButton(size_t idx) {
  if (!Bluefruit.connected() && idx < 3) {
    Serial.println(
        "  (not connected — reading will queue and send on connect)");
  }
  switch (idx) {
  case 0:
    sendLeg(123.4f, -12.3f, 4.56f);
    break;
  case 1:
    randomLeg();
    break;
  case 2:
    Serial.println("  burst of 5 (watch the ACK sequence bits):");
    for (int i = 0; i < 5; i++) {
      randomLeg();
    }
    break;
  case 3:
    printStatus();
    break;
  }
}

static void pollButtons() {
  static uint32_t lastChange[NUM_BUTTONS] = {0};
  static bool rawLast[NUM_BUTTONS] = {false};

  for (size_t i = 0; i < NUM_BUTTONS; i++) {
    bool rawPressed = (digitalRead(BUTTONS[i].pin) == LOW);
    if (rawPressed != rawLast[i]) {
      rawLast[i] = rawPressed;
      lastChange[i] = millis();
    } else if (rawPressed != btnPressed[i] && millis() - lastChange[i] >= 20) {
      btnPressed[i] = rawPressed;
      if (rawPressed) {
        Serial.print(BUTTONS[i].name);
        Serial.println(" pressed:");
        onButton(i);
      }
    }
  }
}

// Connection monitoring + the long-range PHY dance, ported from DiscoX loop()
static void pollConnection() {
  if (millis() - lastConnCheck >= 100) {
    lastConnCheck = millis();
    bool connected = Bluefruit.connected();
    if (connected != lastConnected) {
      if (connected) {
        Serial.println("BLE Connected");
        phyRequestTime = millis();
        phyRequestPending = true;
        phyWatchdogCount = 0;
        lastReportedPhy = 0xFF;
      } else {
        Serial.println("BLE Disconnected");
        phyRequestPending = false;
        phyCheckPending = false;
        Bluefruit.Periph.clearBonds();
        if (!Bluefruit.Advertising.isRunning()) {
          Serial.println("Restarting advertising at +8 dBm");
          Bluefruit.Advertising.start(0);
        }
      }
      lastConnected = connected;
    }
  }

  // Initial Coded PHY request, retried until the SoftDevice accepts it
  // (busy right after connect — can't be done from the connect callback)
  if (phyRequestPending && millis() - phyRequestTime >= PHY_RETRY_MS) {
    phyRequestTime = millis();
    uint16_t handle = Bluefruit.connHandle();
    ble_gap_phys_t phys = {BLE_GAP_PHY_CODED, BLE_GAP_PHY_CODED};
    uint32_t err = sd_ble_gap_phy_update(handle, &phys);
    if (err == 0) {
      Serial.println("PHY update request: OK");
      phyRequestPending = false;
      BLEConnection *conn = Bluefruit.Connection(handle);
      if (conn) {
        // Data-packet TX power (setTxPower() above was advertising only)
        if (conn->setTxPower(BLE_TX_POWER)) {
          Serial.println("Connection TX power set to +8 dBm");
        }
        // Actively push the 6 s supervision timeout (PPCP is only a
        // hint the phone may ignore); keep whatever interval it chose
        conn->requestConnectionParameter(conn->getConnectionInterval(), 0,
                                         SUP_TIMEOUT_MS / 10);
        conn->monitorRssi(); // enable RSSI sampling for status lines
      }
      phyCheckTime = millis();
      phyCheckPending = true;
      phyWatchdogTime = millis();
    } else {
      Serial.print("PHY update retry: 0x");
      Serial.println(err, HEX);
    }
  }

  // Delayed report — negotiation takes a moment after the request
  if (phyCheckPending && millis() - phyCheckTime >= 2000) {
    phyCheckPending = false;
    BLEConnection *conn = Bluefruit.Connection(Bluefruit.connHandle());
    if (conn) {
      uint8_t phy = conn->getPHY();
      lastReportedPhy = phy;
      if (phy == BLE_GAP_PHY_CODED) {
        Serial.println("Coded PHY: YES — long range active");
      } else {
        Serial.print("Coded PHY: NO — running at ");
        Serial.println(phyName(phy));
      }
    }
  }

  // PHY watchdog — phones can renegotiate down mid-connection; DiscoX never
  // recovered from that. Capped so Coded-less phones aren't hassled forever.
  if (lastConnected && !phyRequestPending && !phyCheckPending &&
      phyWatchdogCount < PHY_WATCHDOG_MAX &&
      millis() - phyWatchdogTime >= PHY_WATCHDOG_MS) {
    phyWatchdogTime = millis();
    BLEConnection *conn = Bluefruit.Connection(Bluefruit.connHandle());
    if (conn && conn->getPHY() != BLE_GAP_PHY_CODED) {
      phyWatchdogCount++;
      Serial.print("PHY watchdog: not Coded, re-requesting (");
      Serial.print(phyWatchdogCount);
      Serial.print("/");
      Serial.print(PHY_WATCHDOG_MAX);
      Serial.println(")");
      phyRequestTime = 0; // fire the retry path immediately
      phyRequestPending = true;
    }
  }

  // Periodic link status while connected
  if (lastConnected && millis() - lastStatusTime >= STATUS_PERIOD_MS) {
    lastStatusTime = millis();
    printStatus();
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 5000) {
    delay(10); // wait for USB host, but don't block forever
  }

  for (size_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTONS[i].pin, INPUT_PULLUP);
  }
  randomSeed(micros());

  Serial.println();
  Serial.println(
      "=== Mr Zappy PCB V2 — BLE (SAP6, long-range Coded PHY) test ===");
  Serial.println();

  runTests();
}

void loop() {
  // Any keypress re-prints the report (BLE stack stays up — no re-init)
  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    passCount = failCount = 0;
    Serial.println();
    Serial.println("--- re-running test sequence ---");
    runTests();
    return;
  }

  // Inbound SAP6 traffic: ACKs for our legs + commands from the phone
  int cmd = sap6.poll();
  if (cmd >= 0) {
    Serial.print("  RX cmd 0x");
    Serial.print(cmd, HEX);
    Serial.print(" (");
    Serial.print(commandName(cmd));
    Serial.println(")");
  }

  pollConnection();
  pollButtons();
  delay(5);
}
