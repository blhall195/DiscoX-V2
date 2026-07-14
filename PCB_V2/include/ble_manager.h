#pragma once

#include <Arduino.h>
#include <bluefruit.h>

#include "drivers/sap6_ble.h"

// ── BLE command codes received from the phone app ───────────────────
// Same enum as V1 (main.cpp's dispatch was written against it). On V1 these
// arrived as decimal strings over the DiscoX UART; on V2 they come straight
// from the SAP6 command characteristic via sap6.poll().
enum class BleCommand : uint8_t {
  NONE = 0,
  ACK_RECEIVED, // phone ACKed the in-flight leg (SAP6 0x55/0x56)
  READY,        // V1-only (DiscoX startup handshake) — never emitted on V2
  STOP_CAL,     // 0x30
  START_CAL,    // 0x31
  DEVICE_OFF,   // 0x34
  LASER_ON,     // 0x36
  LASER_OFF,    // 0x37
  TAKE_SHOT,    // 0x38
  UNKNOWN
};

// ── BLE Manager — in-process SAP6 stack (V2) ────────────────────────
// Replaces the V1 SERCOM UART bridge to the DiscoX board: the SAP6 GATT
// service (drivers/sap6_ble) now runs on this MCU, so sendSurveyData() feeds
// the send queue directly and update() polls for inbound commands.
//
// Also owns everything the DiscoX board's main.cpp used to do:
//   - connection monitoring (was the BLE_CONNECTED GPIO to the main board)
//   - clear bonds on disconnect + restart advertising
//   - the long-range Coded PHY recipe (event length 24 BEFORE begin(),
//     PHY request retried from the loop, per-connection +8 dBm, 6 s
//     supervision timeout, PHY fallback watchdog) — hardware-proven in
//     "../PCB_V2 test/bringup/src/tests/test_ble.cpp".
//
// ⚠ Coded PHY additionally needs the patched global Bluefruit library
// (BLEConnection.cpp answers PHY update requests with BLE_GAP_PHY_CODED);
// a framework update reverts it silently — see CLAUDE.md.
class BleManager {
public:
  // Full BLE bring-up. MUST be called before anything else touches
  // Bluefruit (configPrphConn only works before Bluefruit.begin()).
  bool begin(const char *name);

  // Non-blocking; call every loop tick. Drives the SAP6 ACK/retry state
  // machine, the connection monitor, and the Coded PHY dance.
  void update();

  // Outbound survey data (roll is not measured on this device — sent as 0)
  void sendSurveyData(float compass, float clino, float distance);
  void sendKeepAlive() {
  } // V1 UART keep-alive — nothing to keep alive in-process

  // Change advertised name (used by the settings menu / app rename flow)
  void setName(const char *name);

  // Inbound
  bool hasCommand() const { return _pendingCmd != BleCommand::NONE; }
  BleCommand readCommand(); // Returns & clears pending command

  // Status
  bool isConnected() const { return Bluefruit.connected(); }
  int pendingReadings() { return sap6.pending(); }

  static const char *commandName(BleCommand cmd);

private:
  static constexpr int8_t TX_POWER_DBM = 8;        // nRF52840 maximum
  static constexpr uint16_t SUP_TIMEOUT_MS = 6000; // ride out fades at range
  static constexpr uint32_t CONN_CHECK_MS = 100;
  static constexpr uint32_t PHY_RETRY_MS = 500;
  static constexpr uint32_t PHY_WATCHDOG_MS = 10000;
  static constexpr uint8_t PHY_WATCHDOG_MAX = 3; // re-requests per connection

  void startAdvertising();
  void pollConnection();
  static BleCommand mapSap6Command(int cmd);

  BleCommand _pendingCmd = BleCommand::NONE;

  // Connection / PHY state (mirrors the DiscoX loop)
  bool _lastConnected = false;
  uint32_t _lastConnCheck = 0;
  bool _phyRequestPending = false;
  uint32_t _phyRequestTime = 0;
  uint32_t _phyWatchdogTime = 0;
  uint8_t _phyWatchdogCount = 0;
};
