#include "drivers/sca3300.h"

constexpr uint32_t SCA3300::CMD_MODE[4];

bool SCA3300::begin(SPIClass &spi, uint8_t csPin, Mode mode) {
    spi_ = &spi;
    csPin_ = csPin;
    mode_ = mode;

    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);
    spi_->begin();

    // Power-on start-up time is 1 ms; assume the rail is already stable by
    // the time begin() is called.
    delay(2);
    return startup();
}

// Table 10 start-up sequence, minus the SPI/pin setup begin() does once.
bool SCA3300::startup() {
    transfer(CMD_SW_RESET);
    delay(1); // memory read + signal path settling after reset

    transfer(CMD_MODE[static_cast<uint8_t>(mode_)]);
    // Signal path settling: 15 ms for modes 1-3, 100 ms for mode 4
    delay(mode_ == Mode::MODE_4 ? 100 : 15);

    if (!clearStatus()) {
        return false;
    }

    if (readWhoAmI() != WHOAMI_VALUE) {
        lastError_ = Error::WHOAMI;
        return false;
    }
    return true;
}

bool SCA3300::readAcceleration(float &gx, float &gy, float &gz) {
    Reading raw;
    if (!readRaw(raw)) {
        return false;
    }
    const float s = sensitivity();
    gx = raw.x / s;
    gy = raw.y / s;
    gz = raw.z / s;
    return true;
}

bool SCA3300::readRaw(Reading &out) { return readRawInternal(out, true); }

bool SCA3300::readRawInternal(Reading &out, bool allowRecovery) {
    // Off-frame protocol: each response belongs to the previous command, so
    // chain the three axis reads and use a trailing frame to collect Z.
    transfer(CMD_READ_ACC_X); // response to whatever came before — discard
    uint32_t rx = transfer(CMD_READ_ACC_Y);
    uint32_t ry = transfer(CMD_READ_ACC_Z);
    uint32_t rz = transfer(CMD_READ_ACC_X);
    if (!validate(rx) || !validate(ry) || !validate(rz)) {
        // A latched STATUS flag fails every future frame until STATUS is
        // read, so clear it and take one more look before giving up.
        if (allowRecovery) {
            Error fault = lastError_;
            if (recover()) {
                return readRawInternal(out, false);
            }
            lastError_ = fault; // report what faulted, not how recovery went
        }
        return false;
    }
    out.x = static_cast<int16_t>((rx >> 8) & 0xFFFF);
    out.y = static_cast<int16_t>((ry >> 8) & 0xFFFF);
    out.z = static_cast<int16_t>((rz >> 8) & 0xFFFF);
    return true;
}

bool SCA3300::readTemperature(float &celsius) {
    uint16_t raw;
    if (!readRegister(CMD_READ_TEMP, raw)) {
        return false;
    }
    celsius = -273.0f + static_cast<int16_t>(raw) / 18.9f;
    return true;
}

bool SCA3300::readSelfTest(int16_t &sto) {
    uint16_t raw;
    if (!readRegister(CMD_READ_STO, raw)) {
        return false;
    }
    sto = static_cast<int16_t>(raw);
    return true;
}

bool SCA3300::readStatus(uint16_t &status) { return readRegister(CMD_READ_STATUS, status); }

uint8_t SCA3300::readWhoAmI() {
    uint16_t raw;
    if (!readRegister(CMD_READ_WHOAMI, raw)) {
        return 0;
    }
    return raw & 0xFF;
}

bool SCA3300::readSerialNumber(char *buf, size_t len) {
    if (buf == nullptr || len < 14) {
        return false;
    }
    // Section 6.6: serial lives in bank 1 as two 16-bit words
    transfer(CMD_SELECT_BANK_1);
    transfer(CMD_READ_SERIAL1);
    uint32_t r1 = transfer(CMD_READ_SERIAL2);
    uint32_t r2 = transfer(CMD_SELECT_BANK_0);
    transfer(CMD_READ_STATUS); // flush so the bank switch completes cleanly
    if (!validate(r1) || !validate(r2)) {
        return false;
    }
    uint32_t serial = (((r2 >> 8) & 0xFFFF) << 16) | ((r1 >> 8) & 0xFFFF);
    snprintf(buf, len, "%luB33", static_cast<unsigned long>(serial));
    return true;
}

bool SCA3300::powerDown() {
    transfer(CMD_POWER_DOWN);
    uint32_t resp = transfer(CMD_READ_STATUS);
    return crc8(resp) == (resp & 0xFF);
}

bool SCA3300::wakeUp() {
    transfer(CMD_WAKE_UP);
    // Wake-up frame is the mode 1 command; restore the configured mode
    if (mode_ != Mode::MODE_1) {
        transfer(CMD_MODE[static_cast<uint8_t>(mode_)]);
    }
    delay(mode_ == Mode::MODE_4 ? 100 : 15);
    return clearStatus();
}

bool SCA3300::recover() {
    uint32_t now = millis();
    if (recoveryAttempted_ && (now - lastRecoveryMs_) < RECOVERY_RETRY_MS) {
        return false;
    }
    recoveryAttempted_ = true;
    lastRecoveryMs_ = now;
    ++faultCount_;
    lastRecoveryReset_ = false;

    uint16_t status = 0;
    if (!readAndClearStatus(status)) {
        // STATUS itself unreadable — bus or CRC trouble, not a sensor flag.
        // A reset is the only remaining move.
        lastStatus_ = 0xFFFF;
        lastRecoveryReset_ = true;
        return startup();
    }
    lastStatus_ = status;

    if (status & STATUS_NEEDS_RESET) {
        lastRecoveryReset_ = true;
        return startup();
    }
    if (status & STATUS_PD) {
        lastRecoveryReset_ = true;
        return wakeUp();
    }
    // SAT / TEMP_SAT only: the flag is transient and reading STATUS above
    // already cleared it, so the next frame should come back RS '01'.
    return true;
}

bool SCA3300::readAndClearStatus(uint16_t &status) {
    // Section 6.3.1: frame 1 answers the previous command, frame 2 carries the
    // flags but still reports RS '11', frame 3 confirms RS is back to '01'
    // with STATUS now zero. Validating frame 2 would discard the flags, so
    // only its CRC is checked here.
    transfer(CMD_READ_STATUS);
    uint32_t flagged = transfer(CMD_READ_STATUS);
    uint32_t cleared = transfer(CMD_READ_STATUS);
    if (crc8(flagged) != (flagged & 0xFF)) {
        lastError_ = Error::CRC;
        return false;
    }
    status = (flagged >> 8) & 0xFFFF;
    return validate(cleared);
}

const char *SCA3300::errorName(Error e) {
    switch (e) {
    case Error::NONE:
        return "none";
    case Error::CRC:
        return "CRC mismatch (bus/wiring)";
    case Error::STARTUP:
        return "sensor still starting up";
    case Error::SENSOR_FLAG:
        return "sensor error flag (see STATUS)";
    case Error::WHOAMI:
        return "WHOAMI mismatch";
    }
    return "unknown";
}

void SCA3300::statusBitNames(uint16_t status, char *buf, size_t len) {
    if (buf == nullptr || len == 0) {
        return;
    }
    if (status == 0xFFFF) {
        snprintf(buf, len, "unreadable");
        return;
    }
    static const struct {
        uint16_t mask;
        const char *name;
    } kBits[] = {
        {STATUS_DIGI1, "DIGI1"}, {STATUS_DIGI2, "DIGI2"},
        {STATUS_CLK, "CLK"},     {STATUS_SAT, "SAT"},
        {STATUS_TEMP_SAT, "TEMP_SAT"}, {STATUS_PWR, "PWR"},
        {STATUS_MEM, "MEM"},     {STATUS_PD, "PD"},
        {STATUS_MODE_CHANGE, "MODE_CHANGE"}, {STATUS_PIN_CONTINUITY, "PIN_CONTINUITY"},
    };
    buf[0] = '\0';
    size_t used = 0;
    for (const auto &bit : kBits) {
        if (!(status & bit.mask)) {
            continue;
        }
        int n = snprintf(buf + used, len - used, "%s%s", used ? "|" : "", bit.name);
        if (n <= 0 || static_cast<size_t>(n) >= len - used) {
            return; // truncated
        }
        used += n;
    }
    if (used == 0) {
        snprintf(buf, len, "none");
    }
}

float SCA3300::sensitivity() const {
    switch (mode_) {
    case Mode::MODE_1:
        return 2700.0f;
    case Mode::MODE_2:
        return 1350.0f;
    default:
        return 5400.0f; // modes 3 and 4
    }
}

uint32_t SCA3300::transfer(uint32_t frame) {
    spi_->beginTransaction(settings_);
    digitalWrite(csPin_, LOW);
    uint32_t resp = 0;
    resp |= static_cast<uint32_t>(spi_->transfer((frame >> 24) & 0xFF)) << 24;
    resp |= static_cast<uint32_t>(spi_->transfer((frame >> 16) & 0xFF)) << 16;
    resp |= static_cast<uint32_t>(spi_->transfer((frame >> 8) & 0xFF)) << 8;
    resp |= static_cast<uint32_t>(spi_->transfer(frame & 0xFF));
    digitalWrite(csPin_, HIGH);
    spi_->endTransaction();
    delayMicroseconds(TLH_US); // TLH: CSB must stay high ≥10 µs between frames
    return resp;
}

bool SCA3300::validate(uint32_t response) {
    if (crc8(response) != (response & 0xFF)) {
        lastError_ = Error::CRC;
        return false;
    }
    rs_ = (response >> 24) & 0x03;
    if (rs_ == RS_ERROR) {
        lastError_ = Error::SENSOR_FLAG;
        return false;
    }
    if (rs_ == RS_STARTUP) {
        lastError_ = Error::STARTUP;
        return false;
    }
    lastError_ = Error::NONE;
    return true;
}

bool SCA3300::readRegister(uint32_t cmd, uint16_t &data) {
    transfer(cmd); // response to previous command — discard
    uint32_t resp = transfer(cmd);
    if (!validate(resp)) {
        return false;
    }
    data = (resp >> 8) & 0xFFFF;
    return true;
}

bool SCA3300::clearStatus() {
    // Start-up steps 6-8: first read returns pre-clear summary (RS '11' is
    // expected after reset/mode change), the following response must be '01'.
    transfer(CMD_READ_STATUS);
    transfer(CMD_READ_STATUS);
    uint32_t resp = transfer(CMD_READ_STATUS);
    return validate(resp);
}

// CRC-8, poly 0x1D, init 0xFF, output inverted — over the 24 MSBs (section 5.2)
uint8_t SCA3300::crc8(uint32_t frame) {
    uint8_t crc = 0xFF;
    for (int8_t bit = 31; bit >= 8; --bit) {
        uint8_t in = (frame >> bit) & 0x01;
        uint8_t msb = (crc & 0x80) ? 1 : 0;
        crc <<= 1;
        if (in ^ msb) {
            crc ^= 0x1D;
        }
    }
    return static_cast<uint8_t>(~crc);
}
