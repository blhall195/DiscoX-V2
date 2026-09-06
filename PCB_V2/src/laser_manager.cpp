#include "laser_manager.h"

#include "defaults.h"
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

LaserError LaserManager::measure(int32_t &distanceMm, uint16_t *sq) {
    // SINGLE_SLOW prioritises accuracy over speed — right choice for survey
    // shots on marginal targets (SINGLE_AUTO picks speed from signal strength).
    LDJ100::Measurement m;
    if (_ldj.measure(m, LDJ100::SINGLE_SLOW)) {
        distanceMm = (int32_t)m.distanceMm;
        if (sq) {
            *sq = m.signalQuality;
        }
        _lastError = LaserError::OK;
    } else {
        _lastError = (_ldj.lastStatus() != 0) ? mapStatus(_ldj.lastStatus()) : LaserError::TIMEOUT;
    }
    return _lastError;
}

LaserError LaserManager::measureValidated(int32_t &distanceMm, uint8_t shots, uint16_t sqLimit,
                                          uint16_t spreadLimitMm) {
    if (shots < 1) {
        shots = 1;
    } else if (shots > Defaults::laserShotsMax) {
        shots = Defaults::laserShotsMax;
    }
    // 1-2 shots: every shot must survive. 3+: 3 survivors are enough.
    const uint8_t minValid = (shots < 3) ? shots : 3;
    uint32_t dist[Defaults::laserShotsMax];
    uint8_t valid = 0;
    bool sawWeak = false; // SQ over limit, or module reported weak signal
    LaserError firstHardErr = LaserError::TIMEOUT;
    bool haveHardErr = false;

    for (uint8_t shot = 0; shot < shots; shot++) {
        LDJ100::Measurement m;
        bool ok = _ldj.measure(m, LDJ100::SINGLE_SLOW);
        uint16_t st = _ldj.lastStatus();

        // One line per shot — the SQ threshold calibration data set.
        Serial.print(F("LZRSQ shot="));
        Serial.print(shot);
        Serial.print(F(" mm="));
        Serial.print(ok ? (int32_t)m.distanceMm : -1);
        Serial.print(F(" sq="));
        Serial.print(ok ? m.signalQuality : 0xFFFF);
        Serial.print(F(" (0x"));
        Serial.print(ok ? m.signalQuality : 0xFFFF, HEX);
        Serial.print(F(")"));
        Serial.print(F(" st=0x"));
        Serial.println(st, HEX);

        if (!ok) {
            LaserError e = (st != 0) ? mapStatus(st) : LaserError::TIMEOUT;
            if (e == LaserError::TOO_DIM) {
                sawWeak = true;
            } else if (!haveHardErr) {
                firstHardErr = e;
                haveHardErr = true;
            }
        } else if (sqLimit != 0 && m.signalQuality < sqLimit) {
            sawWeak = true;
        } else {
            dist[valid++] = m.distanceMm;
        }

        // Bail once enough survivors are impossible.
        uint8_t remaining = shots - shot - 1;
        if (valid + remaining < minValid) {
            break;
        }
    }

    if (valid < minValid) {
        // Weak signal is the actionable diagnosis (target card / better
        // angle), so report it over generic comm errors.
        _lastError = sawWeak ? LaserError::TOO_DIM
                             : (haveHardErr ? firstHardErr : LaserError::TIMEOUT);
        return _lastError;
    }

    // Insertion sort — at most Defaults::laserShots elements.
    for (uint8_t i = 1; i < valid; i++) {
        uint32_t v = dist[i];
        int8_t j = i - 1;
        while (j >= 0 && dist[j] > v) {
            dist[j + 1] = dist[j];
            j--;
        }
        dist[j + 1] = v;
    }

    if (valid > 1 && dist[valid - 1] - dist[0] > spreadLimitMm) {
        _lastError = LaserError::INCONSISTENT;
        return _lastError;
    }

    // Median: middle element, or mean of the two middles for even counts.
    uint32_t median = (valid & 1) ? dist[valid / 2]
                                  : (dist[valid / 2 - 1] + dist[valid / 2]) / 2;

    if (median < Defaults::laserMinDistanceMm || median > Defaults::laserMaxDistanceMm) {
        _lastError = LaserError::BAD_READING;
        return _lastError;
    }

    distanceMm = (int32_t)median;
    _lastError = LaserError::OK;
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
    case LaserError::INCONSISTENT:
        return "Inconsistent";
    }
    return "Unknown";
}
