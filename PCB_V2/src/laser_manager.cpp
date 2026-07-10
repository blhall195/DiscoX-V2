#include "laser_manager.h"

#include "sounds.h"

bool LaserManager::begin(Stream &serial) {
    _ldj.begin(serial);
    return _ldj.ping(300);
}

LaserError LaserManager::setLaser(bool on) {
    if (_ldj.setLaser(on)) {
        _lastError = LaserError::OK;
    } else {
        _lastError = (_ldj.lastStatus() != 0) ? mapStatus(_ldj.lastStatus()) : LaserError::TIMEOUT;
    }
    return _lastError;
}

LaserError LaserManager::setBuzzer(bool on) {
    if (on) {
        Sounds::click();
    }
    _lastError = LaserError::OK;
    return _lastError;
}

LaserError LaserManager::stopMeasuring() {
    _ldj.stopContinuous();
    _lastError = LaserError::OK;
    return _lastError;
}

void LaserManager::wibble() {
    // Same visual as V1: blink the laser dot four times.
    delay(25);
    for (int i = 0; i < 4; i++) {
        _ldj.setLaser(true);
        delay(150);
        _ldj.setLaser(false);
        delay(200);
    }
}

LaserError LaserManager::measure(int32_t &distanceMm) {
    LDJ100::Measurement m;
    if (_ldj.measure(m, LDJ100::SINGLE_AUTO)) {
        distanceMm = (int32_t)m.distanceMm;
        _lastError = LaserError::OK;
    } else {
        _lastError = (_ldj.lastStatus() != 0) ? mapStatus(_ldj.lastStatus()) : LaserError::TIMEOUT;
    }
    return _lastError;
}

LaserError LaserManager::mapStatus(uint16_t status) {
    // LDJ100 error frames carry a signed int16 status (datasheet table 6-1,
    // reported negative); see LDJ100::statusText for the full set.
    int16_t code = (int16_t)status;
    if (code < 0) {
        code = -code;
    }
    switch (code) {
    case 0x0005: // target out of range
    case 0x0006: // invalid measurement result
    case 0x000F: // laser signal unstable
        return LaserError::BAD_READING;
    case 0x0008: // laser signal too weak
        return LaserError::TOO_DIM;
    case 0x0007: // ambient light too strong
    case 0x0009: // laser signal too strong
        return LaserError::TOO_BRIGHT;
    default:
        return LaserError::COMMAND_FAILED;
    }
}

const char *LaserManager::errorString(LaserError err) {
    switch (err) {
    case LaserError::OK:
        return "OK";
    case LaserError::TIMEOUT:
        return "Timeout";
    case LaserError::COMMAND_FAILED:
        return "Command failed";
    case LaserError::TOO_DIM:
        return "Too dim";
    case LaserError::TOO_BRIGHT:
        return "Too bright";
    case LaserError::BAD_READING:
        return "Bad reading";
    }
    return "Unknown";
}
