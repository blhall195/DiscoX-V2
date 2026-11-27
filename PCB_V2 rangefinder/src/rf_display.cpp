#include "rf_display.h"

#include <string.h>

// ── Battery icon — top-right (= ../PCB_V2 display_manager.cpp) ──────
static constexpr int16_t BAT_X = 90;
static constexpr int16_t BAT_Y = 0;
static constexpr int16_t BAT_W = 32;
static constexpr int16_t BAT_H = 15;
static constexpr int16_t BAT_TIP_W = 3;
static constexpr int16_t BAT_TIP_H = 6;
static constexpr int16_t BAT_FILL_W = 30;
static constexpr int16_t BAT_FILL_H = 13;

// ── Main screen ─────────────────────────────────────────────────────
static constexpr int16_t DIST_Y = 44;      // size-4 text, 32 px tall: centred between battery and hint bar
static constexpr int16_t HINT_BAR_Y = 104; // inverted hint bar, 20 px
static constexpr int16_t HINT_BAR_H = 20;

// ── Error screen (= ../PCB_V2 display_manager.cpp) ──────────────────
static constexpr int16_t ERR_HEAD_Y = 22;
static constexpr uint8_t ERR_HEAD_MAX_CHARS = 7;
static constexpr int16_t ERR_RULE_Y = 56;
static constexpr int16_t ERR_RULE_INSET = 14;
static constexpr int16_t ERR_BODY_TOP = 64;
static constexpr int16_t ERR_BODY_LINE_H = 18;
static constexpr uint8_t ERR_BODY_COLS = 10;
static constexpr uint8_t ERR_BODY_LINES = 3;

// Greedy word-wrap, breaking on '\n' as well as spaces (copied from the
// full firmware's display_manager.cpp).
static uint8_t wrapWords(const char *text, char out[][ERR_BODY_COLS + 1], uint8_t maxLines) {
    for (uint8_t i = 0; i < maxLines; i++) {
        out[i][0] = '\0';
    }
    if (!text || maxLines == 0) {
        return 0;
    }
    uint8_t line = 0;
    uint8_t col = 0;
    while (*text) {
        bool forced = false;
        while (*text == ' ' || *text == '\n') {
            if (*text == '\n') {
                forced = true;
            }
            text++;
        }
        if (!*text) {
            break;
        }
        if (forced && col != 0) {
            if (line + 1 >= maxLines) {
                break;
            }
            line++;
            col = 0;
        }
        size_t wordLen = 0;
        while (text[wordLen] && text[wordLen] != ' ' && text[wordLen] != '\n') {
            wordLen++;
        }
        size_t take = (wordLen > ERR_BODY_COLS) ? ERR_BODY_COLS : wordLen;
        if (col != 0 && col + 1 + take > ERR_BODY_COLS) {
            if (line + 1 >= maxLines) {
                break;
            }
            line++;
            col = 0;
        }
        if (col != 0) {
            out[line][col++] = ' ';
        }
        for (size_t i = 0; i < take; i++) {
            out[line][col++] = text[i];
        }
        out[line][col] = '\0';
        text += wordLen;
    }
    return (out[0][0] != '\0') ? (uint8_t)(line + 1) : 0;
}

// Zero the panel RAM before begin() so a cold boot doesn't flash static
// (see blankPanelRam in the full firmware's display_manager.cpp).
static void blankPanelRam() {
    constexpr uint8_t CHUNK = 32;
    for (uint8_t page = 0; page < SH1107_HEIGHT / 8; page++) {
        Wire.beginTransmission(SH1107_ADDR);
        Wire.write(0x00);
        Wire.write((uint8_t)(0xB0 + page));
        Wire.write(0x10);
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) {
            return;
        }
        for (uint8_t col = 0; col < SH1107_WIDTH; col += CHUNK) {
            Wire.beginTransmission(SH1107_ADDR);
            Wire.write(0x40);
            for (uint8_t i = 0; i < CHUNK; i++) {
                Wire.write(0x00);
            }
            Wire.endTransmission();
        }
    }
}

bool RfDisplay::begin() {
    blankPanelRam();
    if (!_display.begin(SH1107_ADDR, true)) {
        return false;
    }
    _display.setRotation(2); // same orientation as the full firmware
    _display.clearDisplay();
    _display.display();
    _ok = true;
    return true;
}

void RfDisplay::setBrightness(uint8_t level) {
    if (_ok) {
        _display.setContrast(level);
    }
}

void RfDisplay::setBattery(float pct, BatteryMode mode) {
    _battery = constrain(pct, 0.0f, 100.0f);
    _batMode = mode;
}

void RfDisplay::centred(const char *text, uint8_t size, int16_t y) {
    int16_t w = (int16_t)strlen(text) * 6 * size;
    _display.setTextSize(size);
    _display.setCursor((SH1107_WIDTH - w) / 2, y);
    _display.print(text);
}

void RfDisplay::drawBattery() {
    float pct = _battery;
    // Charging: the charged part solid, the rest filling in five steps
    if (_batMode == BatteryMode::CHARGING && pct < 99.5f) {
        const uint32_t frame = (millis() / 250) % 5;
        pct += (100.0f - pct) * (float)frame / 4.0f;
    }
    _display.drawRect(BAT_X, BAT_Y, BAT_W, BAT_H, SH110X_WHITE);
    _display.fillRect(BAT_X + BAT_W, BAT_Y + (BAT_H - BAT_TIP_H) / 2, BAT_TIP_W, BAT_TIP_H, SH110X_WHITE);
    int16_t fillW = (int16_t)((pct / 100.0f) * BAT_FILL_W);
    if (fillW > 0) {
        _display.fillRect(BAT_X + 1, BAT_Y + 1, fillW, BAT_FILL_H, SH110X_WHITE);
    }
}

void RfDisplay::refresh() {
    if (!_ok) {
        return;
    }
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);

    drawBattery();

    // Distance with its unit: size 4 when it fits ("9.99m"), else size 3
    // ("12.34m" up to "100.00m", 126 px)
    char buf[12];
    if (_distance > 0.0f) {
        snprintf(buf, sizeof(buf), "%.2fm", (double)_distance);
    } else {
        strcpy(buf, "0.00m"); // no reading taken yet this session
    }
    uint8_t size = (strlen(buf) * 24 <= SH1107_WIDTH) ? 4 : 3;
    centred(buf, size, DIST_Y + (4 - size) * 4);

    // While the beam is on, an inverted bar says FIRE will measure. Nothing
    // when it's off. Max 21 chars.
    if (_hint != LaserHint::OFF) {
        _display.fillRect(0, HINT_BAR_Y, SH1107_WIDTH, HINT_BAR_H, SH110X_WHITE);
        _display.setTextColor(SH110X_BLACK);
        centred(_hint == LaserHint::AIMING ? "FIRE to measure" : "Measuring...", 1, HINT_BAR_Y + 6);
        _display.setTextColor(SH110X_WHITE);
    }
    _display.display();
}

void RfDisplay::showError(const char *headline, const char *detail) {
    if (!_ok) {
        return;
    }
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);
    drawBattery();

    size_t hlen = strlen(headline);
    uint8_t hsize = 3;
    if (hlen > ERR_HEAD_MAX_CHARS) {
        hsize = 2;
        if (hlen > ERR_BODY_COLS) {
            hlen = ERR_BODY_COLS;
        }
    }
    _display.setTextSize(hsize);
    _display.setCursor((SH1107_WIDTH - (int16_t)hlen * 6 * hsize) / 2, ERR_HEAD_Y + (3 - hsize) * 4);
    for (size_t i = 0; i < hlen; i++) {
        _display.write(headline[i]);
    }
    _display.drawFastHLine(ERR_RULE_INSET, ERR_RULE_Y, SH1107_WIDTH - 2 * ERR_RULE_INSET, SH110X_WHITE);

    char lines[ERR_BODY_LINES][ERR_BODY_COLS + 1];
    uint8_t n = wrapWords(detail, lines, ERR_BODY_LINES);
    if (n > 0) {
        int16_t blockH = (int16_t)n * ERR_BODY_LINE_H - (ERR_BODY_LINE_H - 16);
        int16_t y = ERR_BODY_TOP + ((SH1107_HEIGHT - ERR_BODY_TOP) - blockH) / 2;
        for (uint8_t i = 0; i < n; i++) {
            centred(lines[i], 2, y + (int16_t)i * ERR_BODY_LINE_H);
        }
    }
    _display.display();
}

void RfDisplay::showPowerPrompt(const char *const *lines, uint8_t count, float progress, const char *hint) {
    if (!_ok) {
        return;
    }
    constexpr int16_t LINE_H = 20;
    constexpr int16_t BAR_X = 14, BAR_W = 100, BAR_H = 14, BAR_GAP = 8;
    constexpr int16_t HINT_GAP = 10, HINT_LINE_H = 10;
    const bool bar = progress >= 0.0f;
    uint8_t hintLines = 0;
    if (hint) {
        hintLines = 1;
        for (const char *c = hint; *c; c++) {
            hintLines += (*c == '\n');
        }
    }
    const int16_t hintH = hintLines ? HINT_GAP + hintLines * HINT_LINE_H - 2 : 0;
    const int16_t blockH = count * LINE_H - 4 + (bar ? BAR_GAP + BAR_H : 0) + hintH;
    const int16_t y0 = (SH1107_HEIGHT - blockH) / 2;

    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);
    for (uint8_t i = 0; i < count; i++) {
        centred(lines[i], 2, y0 + i * LINE_H);
    }
    if (bar) {
        const int16_t by = y0 + count * LINE_H - 4 + BAR_GAP;
        float p = progress > 1.0f ? 1.0f : progress;
        _display.drawRoundRect(BAR_X, by, BAR_W, BAR_H, 4, SH110X_WHITE);
        int16_t fill = (int16_t)(p * (BAR_W - 4));
        if (fill > 0) {
            _display.fillRect(BAR_X + 2, by + 2, fill, BAR_H - 4, SH110X_WHITE);
        }
    }
    if (hint) {
        int16_t hy = y0 + blockH - hintH + HINT_GAP;
        _display.setTextSize(1);
        for (const char *line = hint; *line;) {
            const char *end = strchr(line, '\n');
            size_t len = end ? (size_t)(end - line) : strlen(line);
            _display.setCursor((SH1107_WIDTH - (int16_t)len * 6) / 2, hy);
            for (size_t i = 0; i < len; i++) {
                _display.write(line[i]);
            }
            hy += HINT_LINE_H;
            if (!end) {
                break;
            }
            line = end + 1;
        }
    }
    _display.display();
}

void RfDisplay::showCharging(float pct, bool havePct, const char *status, const char *hint1,
                             const char *hint2) {
    if (!_ok) {
        return;
    }
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);
    drawBattery();
    char buf[12];
    if (havePct) {
        snprintf(buf, sizeof(buf), "%.0f%%", (double)pct);
    } else {
        strcpy(buf, "--%");
    }
    centred(buf, 3, 36);
    centred(status, 2, 70);
    if (hint1) {
        centred(hint1, 1, 100);
    }
    if (hint2) {
        centred(hint2, 1, 112);
    }
    _display.display();
}

void RfDisplay::blank() {
    if (_ok) {
        _display.clearDisplay();
        _display.display();
    }
}

void RfDisplay::setSleep(bool sleep) {
    if (_ok) {
        _display.oled_command(sleep ? SH110X_DISPLAYOFF : SH110X_DISPLAYON);
    }
}
