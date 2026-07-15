#pragma once
// SAP6 BLE survey protocol — port of the V1 DiscoX sap6_protocol.cpp
// (github.com/blhall195/Mr_Zappy; itself a faithful port of caveble.py
// SurveyProtocolService).
//
// This is the module destined for the V2 production firmware: on V2 there is
// no UART bridge, so sendData() is called directly from the measurement code.
// The reliable-delivery scheme is unchanged from V1 (phone apps depend on it):
// 17-byte leg packets <Bffff (seq bit + az/inc/roll/dist floats), sequence bit
// alternating 0/1, receiver ACKs with 0x55/0x56, resend after 5 s without ACK.
//
// NOTE (long range): Coded PHY on this stack additionally relies on the
// patched global Bluefruit library (BLEConnection.cpp responds to PHY update
// requests with BLE_GAP_PHY_CODED instead of AUTO) and on the host app calling
// Bluefruit.configPrphConn() with event length >= 24 BEFORE Bluefruit.begin().
// See the "Coded PHY" gotcha in PCB_V2/CLAUDE.md.

#include <Arduino.h>
#include <bluefruit.h>

// 128-bit UUIDs, little-endian byte order for Bluefruit
// (string form 137c4435-8a64-4bcb-93f1-3792c6bdc9XX, XX = 65/66/67/68)
extern const uint8_t SAP6_SERVICE_UUID[16];
extern const uint8_t SAP6_PROTO_NAME_UUID[16];
extern const uint8_t SAP6_COMMAND_UUID[16];
extern const uint8_t SAP6_LEG_DATA_UUID[16];

// SAP6 protocol command bytes
constexpr uint8_t SAP6_CMD_ACK0 = 0x55; // Acknowledge leg with sequence bit 0
constexpr uint8_t SAP6_CMD_ACK1 = 0x56; // Acknowledge leg with sequence bit 1
constexpr uint8_t SAP6_CMD_STOP_CAL = 0x30;   // Finish calibration
constexpr uint8_t SAP6_CMD_START_CAL = 0x31;  // Start calibration
constexpr uint8_t SAP6_CMD_DEVICE_OFF = 0x34; // Turn device off
constexpr uint8_t SAP6_CMD_LASER_ON = 0x36;   // Turn laser on
constexpr uint8_t SAP6_CMD_LASER_OFF = 0x37;  // Turn laser off
constexpr uint8_t SAP6_CMD_TAKE_SHOT = 0x38;  // Take a reading

// ACK lookup — mirrors Python's ACK = [0x56, 0x55]. After sending with bit X
// the code toggles _lastSentBit to X^1, so the expected ACK for the packet
// just sent is SAP6_ACK_TABLE[_lastSentBit] (indexed by the toggled value).
constexpr uint8_t SAP6_ACK_TABLE[2] = {0x56, 0x55};

constexpr uint32_t SAP6_ACK_TIMEOUT_MS = 5000; // resend after 5 s with no ACK
constexpr int SAP6_SEND_QUEUE_MAX = 20;        // max queued leg readings

// One queued survey measurement
struct LegReading {
  float azimuth;
  float inclination;
  float roll;
  float distance;
};

class SAP6Protocol {
public:
  // Create the GATT service and characteristics. Call AFTER Bluefruit.begin().
  // Returns false if any GATT registration failed.
  bool begin();

  // Queue a measurement for BLE transmission (drop if queue full).
  void sendData(float azimuth, float inclination, float distance,
                float roll = 0.0f);

  // Drive the state machine: check for inbound commands, handle ACK/retry.
  // Returns the received command byte (>= 0), or -1 if nothing actionable.
  // ACKs for the in-flight packet are consumed internally and also returned.
  int poll();

  // How many readings are waiting (queue + any in-flight packet).
  int pending();

  // Called from the static BLE write callback trampoline.
  void onCommandWrite(uint16_t conn_hdl, uint8_t *data, uint16_t len);

  // Expose service for advertising setup.
  BLEService &service() { return _service; }

  // Diagnostics (bring-up / status display)
  uint32_t sentCount() const { return _sentCount; }
  uint32_t ackedCount() const { return _ackedCount; }
  uint32_t resendCount() const { return _resendCount; }
  bool waitingForAck() const { return _waitingForAck; }

private:
  void pollOut();
  int pollIn();

  BLEService _service{SAP6_SERVICE_UUID};
  BLECharacteristic _protoNameChar{SAP6_PROTO_NAME_UUID};
  BLECharacteristic _commandChar{SAP6_COMMAND_UUID};
  BLECharacteristic _legDataChar{SAP6_LEG_DATA_UUID};

  LegReading _queue[SAP6_SEND_QUEUE_MAX];
  int _queueHead = 0;
  int _queueTail = 0;
  int _queueCount = 0;

  // Sequence-bit ACK/retry state
  uint8_t _lastSentBit = 0;
  bool _waitingForAck = false;
  uint32_t _lastSendTime = 0;
  uint8_t _currentPacket[17] = {}; // cached for resend

  // Inbound command from BLE write callback (set from BLE context)
  volatile uint8_t _pendingCmd = 0;
  volatile bool _cmdReceived = false;

  uint32_t _sentCount = 0;
  uint32_t _ackedCount = 0;
  uint32_t _resendCount = 0;
};

extern SAP6Protocol sap6;
