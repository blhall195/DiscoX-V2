#include "rm3100.h"
#include <math.h>

static constexpr uint8_t REG_POLL = 0x00;
static constexpr uint8_t REG_CMM = 0x01;
static constexpr uint8_t REG_CCX = 0x04;
static constexpr uint8_t REG_TMRC = 0x0B;
static constexpr uint8_t REG_MX = 0x24;
static constexpr uint8_t REG_STATUS = 0x34;

static constexpr float LN2 = 0.693147f;

bool RM3100::begin(TwoWire &wire, uint8_t addr, uint16_t cycleCount, int8_t drdyPin) {
    wire_ = &wire;
    addr_ = addr;
    cycleCount_ = cycleCount;
    drdyPin_ = drdyPin;
    continuous_ = false;

    if (drdyPin_ >= 0) {
        pinMode(drdyPin_, INPUT);
    }

    wire_->beginTransmission(addr_);
    if (wire_->endTransmission() != 0) {
        return false;
    }

    uint8_t cc[6] = {
        (uint8_t)(cycleCount_ >> 8),   (uint8_t)(cycleCount_ & 0xFF), (uint8_t)(cycleCount_ >> 8),
        (uint8_t)(cycleCount_ & 0xFF), (uint8_t)(cycleCount_ >> 8),   (uint8_t)(cycleCount_ & 0xFF),
    };
    writeReg(REG_CCX, cc, 6);
    return true;
}

void RM3100::startSingleReading() {
    uint8_t cmd = 0x70;
    writeReg(REG_POLL, &cmd, 1);
}

void RM3100::startContinuousReading(float frequency) {
    int exp = (int)roundf(logf(600.0f / frequency) / LN2);
    if (exp < 0) {
        exp = 0;
    }
    if (exp > 13) {
        exp = 13;
    }
    uint8_t tmrc = 0x92 + exp;
    writeReg(REG_TMRC, &tmrc, 1);

    uint8_t cmm = 0x79;
    writeReg(REG_CMM, &cmm, 1);
    continuous_ = true;
}

void RM3100::stop() {
    uint8_t cmm = 0x70;
    writeReg(REG_CMM, &cmm, 1);
    continuous_ = false;
}

bool RM3100::measurementComplete() const {
    if (drdyPin_ >= 0) {
        return digitalRead(drdyPin_) == HIGH;
    }

    uint8_t status = 0;
    const_cast<RM3100 *>(this)->readReg(REG_STATUS, &status, 1);
    return (status & 0x80) != 0;
}

RM3100::Reading RM3100::getLastReading() {
    uint8_t buf[9];
    readReg(REG_MX, buf, 9);

    Reading r;
    int32_t vals[3];
    for (uint8_t i = 0; i < 3; i++) {
        uint32_t raw = ((uint32_t)buf[i * 3] << 16) | ((uint32_t)buf[i * 3 + 1] << 8) | (uint32_t)buf[i * 3 + 2];
        if (raw & 0x800000) {
            raw |= 0xFF000000;
        }
        vals[i] = (int32_t)raw;
    }
    r.x = vals[0];
    r.y = vals[1];
    r.z = vals[2];
    return r;
}

float RM3100::measurementTime() const { return CYCLE_DURATION * cycleCount_; }

RM3100::Reading RM3100::readSingle() {
    startSingleReading();
    uint32_t timeout = millis() + 100;
    while (!measurementComplete()) {
        if (millis() > timeout) {
            break;
        }
        delayMicroseconds(100);
    }
    return getLastReading();
}

void RM3100::toMicroTesla(const Reading &raw, float &ux, float &uy, float &uz) const {
    float factor = UT_PER_CYCLE / cycleCount_;
    ux = raw.x * factor;
    uy = raw.y * factor;
    uz = raw.z * factor;
}

void RM3100::writeReg(uint8_t reg, const uint8_t *data, uint8_t len) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(data, len);
    wire_->endTransmission();
}

void RM3100::readReg(uint8_t reg, uint8_t *buf, uint8_t len) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->endTransmission(false);
    wire_->requestFrom(addr_, len);
    for (uint8_t i = 0; i < len && wire_->available(); i++) {
        buf[i] = wire_->read();
    }
}

uint8_t RM3100::readReg8(uint8_t reg) {
    uint8_t val = 0;
    readReg(reg, &val, 1);
    return val;
}
