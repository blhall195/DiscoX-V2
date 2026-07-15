#include "ble_manager.h"

bool BleManager::begin(const char *name) {
    // 1. Event length MUST go in before begin() — Coded PHY needs >= 24
    //    (default 3 makes sd_ble_gap_phy_update fail with NRF_ERROR_RESOURCES)
    Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, 24, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT,
                             BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    if (!Bluefruit.begin()) {
        Serial.println("[BLE] Bluefruit.begin() failed");
        return false;
    }

    Bluefruit.autoConnLed(false);
    Bluefruit.setName(name);
    Bluefruit.Periph.clearBonds(); // V1 lesson: stale bonds break re-pairing

    // 2. Connection event extension — recovers throughput lost to Coded
    //    PHY's 8x on-air time
    ble_opt_t opt = {};
    opt.common_opt.conn_evt_ext.enable = 1;
    sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);

    // 3. Preferred supervision timeout (phone reads PPCP on connect); the
    //    default ~2 s drops the link on brief fades at Coded PHY range
    Bluefruit.Periph.setConnSupervisionTimeoutMS(SUP_TIMEOUT_MS);

    // 4. Advertising TX power (connection TX power is per-conn, set after
    //    the PHY request succeeds)
    Bluefruit.setTxPower(TX_POWER_DBM);

    // 5. SAP6 GATT service
    if (!sap6.begin()) {
        Serial.println("[BLE] SAP6 GATT registration failed");
        return false;
    }

    startAdvertising();
    return Bluefruit.Advertising.isRunning();
}

void BleManager::startAdvertising() {
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(sap6.service());
    Bluefruit.Advertising.addTxPower();
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(false); // restarted from pollConnection()
    Bluefruit.Advertising.setInterval(32, 244);       // fast, then slow
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0); // 0 = forever
}

void BleManager::update() {
    // Inbound SAP6 traffic: ACKs for our legs + commands from the phone.
    // Single pending slot, same as the V1 UART bridge — main loop consumes
    // it within 10 ms.
    int cmd = sap6.poll();
    if (cmd >= 0) {
        _pendingCmd = mapSap6Command(cmd);
    }

    pollConnection();
}

void BleManager::pollConnection() {
    uint32_t now = millis();

    if (now - _lastConnCheck >= CONN_CHECK_MS) {
        _lastConnCheck = now;
        bool connected = Bluefruit.connected();
        if (connected != _lastConnected) {
            if (connected) {
                Serial.println("[BLE] Connected");
                _phyRequestTime = now;
                _phyRequestPending = true;
                _phyWatchdogCount = 0;
            } else {
                Serial.println("[BLE] Disconnected");
                _phyRequestPending = false;
                Bluefruit.Periph.clearBonds();
                if (!Bluefruit.Advertising.isRunning()) {
                    Bluefruit.Advertising.start(0);
                }
            }
            _lastConnected = connected;
        }
    }

    // Initial Coded PHY request, retried until the SoftDevice accepts it
    // (busy right after connect — can't be done from the connect callback)
    if (_phyRequestPending && now - _phyRequestTime >= PHY_RETRY_MS) {
        _phyRequestTime = now;
        uint16_t handle = Bluefruit.connHandle();
        ble_gap_phys_t phys = {BLE_GAP_PHY_CODED, BLE_GAP_PHY_CODED};
        if (sd_ble_gap_phy_update(handle, &phys) == 0) {
            _phyRequestPending = false;
            BLEConnection *conn = Bluefruit.Connection(handle);
            if (conn) {
                // Data-packet TX power (setTxPower() in begin() was
                // advertising only)
                conn->setTxPower(TX_POWER_DBM);
                // Actively push the 6 s supervision timeout (PPCP is only a
                // hint the phone may ignore); keep whatever interval it chose
                conn->requestConnectionParameter(conn->getConnectionInterval(), 0, SUP_TIMEOUT_MS / 10);
            }
            _phyWatchdogTime = now;
        }
    }

    // PHY watchdog — phones can renegotiate down mid-connection; re-request
    // Coded, capped so Coded-less phones (all iPhones) aren't hassled forever
    if (_lastConnected && !_phyRequestPending && _phyWatchdogCount < PHY_WATCHDOG_MAX &&
        now - _phyWatchdogTime >= PHY_WATCHDOG_MS) {
        _phyWatchdogTime = now;
        BLEConnection *conn = Bluefruit.Connection(Bluefruit.connHandle());
        if (conn && conn->getPHY() != BLE_GAP_PHY_CODED) {
            _phyWatchdogCount++;
            _phyRequestTime = 0; // fire the retry path immediately
            _phyRequestPending = true;
        }
    }
}

void BleManager::sendSurveyData(float compass, float clino, float distance) {
    sap6.sendData(compass, clino, distance);
}

void BleManager::setName(const char *name) {
    Bluefruit.setName(name);
    // Rebuild advertising data so the new name is visible to scanners
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
    startAdvertising();
}

BleCommand BleManager::readCommand() {
    BleCommand cmd = _pendingCmd;
    _pendingCmd = BleCommand::NONE;
    return cmd;
}

BleCommand BleManager::mapSap6Command(int cmd) {
    switch (cmd) {
    case SAP6_CMD_ACK0:
    case SAP6_CMD_ACK1:
        return BleCommand::ACK_RECEIVED;
    case SAP6_CMD_STOP_CAL:
        return BleCommand::STOP_CAL;
    case SAP6_CMD_START_CAL:
        return BleCommand::START_CAL;
    case SAP6_CMD_DEVICE_OFF:
        return BleCommand::DEVICE_OFF;
    case SAP6_CMD_LASER_ON:
        return BleCommand::LASER_ON;
    case SAP6_CMD_LASER_OFF:
        return BleCommand::LASER_OFF;
    case SAP6_CMD_TAKE_SHOT:
        return BleCommand::TAKE_SHOT;
    default:
        return BleCommand::UNKNOWN;
    }
}

const char *BleManager::commandName(BleCommand cmd) {
    switch (cmd) {
    case BleCommand::NONE:
        return "NONE";
    case BleCommand::ACK_RECEIVED:
        return "ACK_RECEIVED";
    case BleCommand::READY:
        return "READY";
    case BleCommand::STOP_CAL:
        return "STOP_CAL";
    case BleCommand::START_CAL:
        return "START_CAL";
    case BleCommand::DEVICE_OFF:
        return "DEVICE_OFF";
    case BleCommand::LASER_ON:
        return "LASER_ON";
    case BleCommand::LASER_OFF:
        return "LASER_OFF";
    case BleCommand::TAKE_SHOT:
        return "TAKE_SHOT";
    case BleCommand::UNKNOWN:
        return "UNKNOWN";
    }
    return "?";
}
