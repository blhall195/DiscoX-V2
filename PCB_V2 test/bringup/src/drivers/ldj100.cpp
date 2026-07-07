#include "ldj100.h"

namespace {
constexpr uint8_t HEAD_OK = 0xAA;
constexpr uint8_t HEAD_ERR = 0xEE;
constexpr uint8_t BYTE_STOP_CONTINUOUS = 0x58; // 'X'
constexpr uint8_t BYTE_BAUD_HANDSHAKE = 0x55;

constexpr uint16_t REG_STATUS = 0x0000;
constexpr uint16_t REG_VOLTAGE = 0x0006;
constexpr uint16_t REG_HW_VERSION = 0x000A;
constexpr uint16_t REG_SW_VERSION = 0x000C;
constexpr uint16_t REG_SERIAL = 0x000E;
constexpr uint16_t REG_MEASURE = 0x0020;
constexpr uint16_t REG_RESULT = 0x0022;
constexpr uint16_t REG_LASER = 0x01BE;
} // namespace

void LDJ100::begin(Stream &serial, uint8_t address) {
    serial_ = &serial;
    addr_ = address & 0x7F;
}

void LDJ100::flushInput() {
    while (serial_->available()) {
        serial_->read();
    }
}

bool LDJ100::readByte(uint8_t &b, uint32_t deadline) {
    while ((int32_t)(deadline - millis()) > 0) {
        if (serial_->available()) {
            b = (uint8_t)serial_->read();
            return true;
        }
        yield();
    }
    return false;
}

bool LDJ100::readFrame(Frame &f, uint32_t deadline) {
    uint8_t b;
    while (readByte(b, deadline)) {
        if (b != HEAD_OK && b != HEAD_ERR) {
            continue; // resync on next head byte
        }
        f.head = b;
        uint8_t hdr[5];
        bool ok = true;
        for (int i = 0; i < 5 && ok; i++) {
            ok = readByte(hdr[i], deadline);
        }
        if (!ok) {
            return false;
        }
        f.addr = hdr[0];
        f.reg = ((uint16_t)hdr[1] << 8) | hdr[2];
        uint16_t words = ((uint16_t)hdr[3] << 8) | hdr[4];
        if (words == 0 || words > MAX_WORDS) {
            continue; // implausible length — treat as garbage
        }
        uint8_t sum = hdr[0] + hdr[1] + hdr[2] + hdr[3] + hdr[4];
        for (uint16_t i = 0; i < words; i++) {
            uint8_t hi, lo;
            if (!readByte(hi, deadline) || !readByte(lo, deadline)) {
                return false;
            }
            f.payload[i] = ((uint16_t)hi << 8) | lo;
            sum += hi + lo;
        }
        uint8_t ck;
        if (!readByte(ck, deadline)) {
            return false;
        }
        if (ck == sum) {
            f.words = (uint8_t)words;
            return true;
        }
    }
    return false;
}

bool LDJ100::waitForReg(uint16_t reg, Frame &f, uint32_t timeoutMs) {
    uint32_t deadline = millis() + timeoutMs;
    while (readFrame(f, deadline)) {
        if (f.head == HEAD_ERR) {
            lastStatus_ = f.payload[0];
            return false;
        }
        if (f.reg == reg) {
            return true;
        }
    }
    return false;
}

void LDJ100::sendRead(uint16_t reg) {
    uint8_t frame[5];
    frame[0] = HEAD_OK;
    frame[1] = 0x80 | addr_;
    frame[2] = (uint8_t)(reg >> 8);
    frame[3] = (uint8_t)reg;
    frame[4] = (uint8_t)(frame[1] + frame[2] + frame[3]);
    serial_->write(frame, sizeof(frame));
}

void LDJ100::sendWrite(uint16_t reg, uint16_t value) {
    uint8_t frame[9];
    frame[0] = HEAD_OK;
    frame[1] = addr_;
    frame[2] = (uint8_t)(reg >> 8);
    frame[3] = (uint8_t)reg;
    frame[4] = 0x00;
    frame[5] = 0x01;
    frame[6] = (uint8_t)(value >> 8);
    frame[7] = (uint8_t)value;
    uint8_t sum = 0;
    for (int i = 1; i < 8; i++) {
        sum += frame[i];
    }
    frame[8] = sum;
    serial_->write(frame, sizeof(frame));
}

bool LDJ100::readWord(uint16_t reg, uint16_t &value, uint32_t timeoutMs) {
    lastStatus_ = 0;
    flushInput();
    sendRead(reg);
    Frame f;
    if (!waitForReg(reg, f, timeoutMs)) {
        return false;
    }
    value = f.payload[0];
    return true;
}

bool LDJ100::ping(uint32_t timeoutMs) {
    uint16_t status;
    return readStatus(status, timeoutMs);
}

bool LDJ100::autoBaudHandshake(uint8_t &moduleAddr, uint32_t timeoutMs) {
    flushInput();
    serial_->write(BYTE_BAUD_HANDSHAKE);
    return readByte(moduleAddr, millis() + timeoutMs);
}

bool LDJ100::readStatus(uint16_t &status, uint32_t timeoutMs) {
    return readWord(REG_STATUS, status, timeoutMs);
}

bool LDJ100::readHardwareVersion(uint16_t &version, uint32_t timeoutMs) {
    return readWord(REG_HW_VERSION, version, timeoutMs);
}

bool LDJ100::readSoftwareVersion(uint16_t &version, uint32_t timeoutMs) {
    return readWord(REG_SW_VERSION, version, timeoutMs);
}

bool LDJ100::readSerialNumber(uint32_t &serialNo, uint32_t timeoutMs) {
    lastStatus_ = 0;
    flushInput();
    sendRead(REG_SERIAL);
    Frame f;
    if (!waitForReg(REG_SERIAL, f, timeoutMs) || f.words < 2) {
        return false;
    }
    serialNo = ((uint32_t)f.payload[0] << 16) | f.payload[1];
    return true;
}

bool LDJ100::readInputVoltageMv(uint16_t &millivolts, uint32_t timeoutMs) {
    uint16_t bcd;
    if (!readWord(REG_VOLTAGE, bcd, timeoutMs)) {
        return false;
    }
    // e.g. 0x3219 = 3219 mV
    millivolts = ((bcd >> 12) & 0xF) * 1000 + ((bcd >> 8) & 0xF) * 100 +
                 ((bcd >> 4) & 0xF) * 10 + (bcd & 0xF);
    return true;
}

bool LDJ100::setLaser(bool on, uint32_t timeoutMs) {
    lastStatus_ = 0;
    flushInput();
    sendWrite(REG_LASER, on ? 0x0001 : 0x0000);
    Frame f; // module echoes the command frame back
    return waitForReg(REG_LASER, f, timeoutMs) && f.payload[0] == (on ? 0x0001 : 0x0000);
}

bool LDJ100::measure(Measurement &m, MeasureMode mode, uint32_t timeoutMs) {
    lastStatus_ = 0;
    flushInput();
    sendWrite(REG_MEASURE, mode);
    Frame f;
    if (!waitForReg(REG_RESULT, f, timeoutMs) || f.words < 3) {
        return false;
    }
    m.distanceMm = ((uint32_t)f.payload[0] << 16) | f.payload[1];
    m.signalQuality = f.payload[2];
    return true;
}

void LDJ100::startContinuous(MeasureMode mode) {
    lastStatus_ = 0;
    flushInput();
    sendWrite(REG_MEASURE, mode); // result frames now stream — see poll()
}

void LDJ100::stopContinuous() {
    serial_->write(BYTE_STOP_CONTINUOUS);
    delay(50);
    flushInput();
}

bool LDJ100::poll(Measurement &m, uint32_t timeoutMs) {
    lastStatus_ = 0;
    Frame f;
    if (!waitForReg(REG_RESULT, f, timeoutMs) || f.words < 3) {
        return false;
    }
    m.distanceMm = ((uint32_t)f.payload[0] << 16) | f.payload[1];
    m.signalQuality = f.payload[2];
    return true;
}

// Datasheet table 6-1 order is ambiguous in the PDF; this follows the
// JRT-family register protocol the LDJ-100 speaks (verified on hardware:
// measurement failures report the code as a NEGATIVE int16, e.g. 0xFFFB =
// -5 = target out of range). Always show the raw code alongside the text.
const char *LDJ100::statusText(uint16_t status) {
    int16_t code = (int16_t)status;
    if (code < 0) {
        code = -code;
    }
    switch (code) {
    case 0x0000: return "no error";
    case 0x0001: return "input voltage low (needs >= 2.0V)";
    case 0x0002: return "internal error (ignorable)";
    case 0x0003: return "module temperature too low (<-20C)";
    case 0x0004: return "module temperature too high (>+60C)";
    case 0x0005: return "target out of range";
    case 0x0006: return "invalid measurement result";
    case 0x0007: return "ambient light too strong";
    case 0x0008: return "laser signal too weak";
    case 0x0009: return "laser signal too strong";
    case 0x000A: return "hardware error";
    case 0x000F: return "laser signal unstable";
    case 0x0081: return "invalid communication format";
    default: return "unknown status";
    }
}
