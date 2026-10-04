#include "display_manager.h"

#include "splash.h"

// ── Layout constants (match Python display_manager.py positions) ────
// Battery icon — top-right
static constexpr int16_t BAT_X = 90;
static constexpr int16_t BAT_Y = 0;
static constexpr int16_t BAT_W = 32;
static constexpr int16_t BAT_H = 15;
static constexpr int16_t BAT_TIP_W = 3;
static constexpr int16_t BAT_TIP_H = 6;
static constexpr int16_t BAT_FILL_X = BAT_X + 1;
static constexpr int16_t BAT_FILL_Y = BAT_Y + 1;
static constexpr int16_t BAT_FILL_W = 30;
static constexpr int16_t BAT_FILL_H = 13;

// BT label — top-left (text size 2)
static constexpr int16_t BT_X = 0;
static constexpr int16_t BT_Y = 0;
static constexpr int16_t BT_NUM_X = 30;

// Data lines — text size 3
static constexpr int16_t DIST_Y = 34;
static constexpr int16_t AZ_Y = 68;
static constexpr int16_t INC_Y = 100;

// Error detail line — text size 1, in the 10 px gap between the distance row
// (34..57 at size 3) and the azimuth row. Only drawn while the distance field
// shows an error headline, so it never displaces a reading. At size 1 the 6 px
// cell gives 21 columns across the 128 px panel.
static constexpr int16_t DETAIL_Y = 59;
static constexpr uint8_t DETAIL_MAX_CHARS = 21;

// ── Full-screen error layout ────────────────────────────────────────
// An error owns the whole panel: the azimuth/inclination behind it are the
// frozen pre-shot values and mean nothing once the shot failed, so that
// space buys a headline you can read at arm's length plus a two-line remedy
// at size 2 instead of the old 21-char size-1 sliver.
// Columns: 128 px / 18 px cell = 7 at size 3, / 12 px = 10 at size 2.
static constexpr int16_t ERR_HEAD_Y = 22;
static constexpr uint8_t ERR_HEAD_MAX_CHARS = 7;
static constexpr int16_t ERR_RULE_Y = 56;
static constexpr int16_t ERR_RULE_INSET = 14;
static constexpr int16_t ERR_BODY_TOP = 64; // body occupies 64..128
static constexpr int16_t ERR_BODY_LINE_H = 18;
static constexpr uint8_t ERR_BODY_COLS = 10;
static constexpr uint8_t ERR_BODY_LINES = 3;

// Greedy word-wrap into at most maxLines of ERR_BODY_COLS, breaking on '\n'
// as well as spaces so the messages can pick their own line breaks. A word
// longer than a line is truncated rather than allowed to run off the panel.
// Returns lines used. Small fixed buffers only — the loop task has a 4 KB
// stack (see CLAUDE.md).
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
        // Skip run of separators, honouring an explicit break.
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

// Degree symbol drawn as a small circle (looks better than CP437 '\xF8' at size
// 3)
static constexpr int16_t DEG_RADIUS = 3;
static constexpr int16_t DEG_OFFSET_X = 2; // gap after text
static constexpr int16_t DEG_OFFSET_Y = 3; // down from top of text line

// ── Public API ─────────────────────────────────────────────────────

// Zero the panel's display RAM over raw I2C, before Adafruit's begin().
// The SH1107's RAM holds random data after a cold power-up, and begin()
// switches the display on (after a 100 ms wait) without writing it — so
// until our first display() lands, ~45 ms later, a cold boot showed a
// screenful of static. Warm resets never did: the panel stays powered and
// keeps its last image. RAM writes are accepted while the display is still
// off (its power-on state), so begin() now turns on a blank screen.
// Page addressing is the power-on default; the commands match
// Adafruit_SH110X::display(). Costs ~45 ms of boot; a NACK just skips it.
static void blankPanelRam() {
    constexpr uint8_t CHUNK = 32; // Wire's TX buffer is 64 bytes
    for (uint8_t page = 0; page < SH1107_HEIGHT / 8; page++) {
        Wire.beginTransmission(SH1107_ADDR);
        Wire.write(0x00);                 // control byte: commands follow
        Wire.write((uint8_t)(0xB0 + page)); // page address
        Wire.write(0x10);                 // column address high nibble = 0
        Wire.write(0x00);                 // column address low nibble = 0
        if (Wire.endTransmission() != 0) {
            return; // panel not answering — begin() will report it
        }
        for (uint8_t col = 0; col < SH1107_WIDTH; col += CHUNK) {
            Wire.beginTransmission(SH1107_ADDR);
            Wire.write(0x40); // control byte: display data follows
            for (uint8_t i = 0; i < CHUNK; i++) {
                Wire.write(0x00);
            }
            Wire.endTransmission();
        }
    }
}

bool DisplayManager::begin() {
    blankPanelRam();
    if (!_display.begin(SH1107_ADDR, true)) {
        return false;
    }
    _display.setRotation(2); // 180 deg to match Python layout
    _display.clearDisplay();
    _display.display();
    _initialized = true;
    return true;
}

void DisplayManager::setBrightness(uint8_t level) {
    if (!_initialized) {
        return;
    }
    _display.setContrast(level);
}

void DisplayManager::initScreen() {
    if (!_initialized) {
        return;
    }
    drawMainScreen();
    _display.display();
}

void DisplayManager::updateSensorReadings(float distance, float azimuth, float inclination) {
    _distance = distance;
    _distIsText = false;
    _azimuth = azimuth;
    _inclination = inclination;
}

void DisplayManager::updateDistance(float distance) {
    _distance = distance;
    _distIsText = false;
}

void DisplayManager::updateDistanceText(const char *text, const char *detail) {
    strncpy(_distText, text, sizeof(_distText) - 1);
    _distText[sizeof(_distText) - 1] = '\0';
    _distIsText = true;
    // No detail clears the previous one — a stale remedy line under an
    // unrelated headline is worse than no line at all.
    if (detail) {
        strncpy(_distDetail, detail, sizeof(_distDetail) - 1);
        _distDetail[sizeof(_distDetail) - 1] = '\0';
    } else {
        _distDetail[0] = '\0';
    }
}

void DisplayManager::updateAzimuth(float azimuth) { _azimuth = azimuth; }

void DisplayManager::updateInclination(float inclination) { _inclination = inclination; }

void DisplayManager::updateBattery(float percentage) { _battery = constrain(percentage, 0.0f, 100.0f); }

void DisplayManager::updateBTLabel(bool connected) { _btConnected = connected; }

void DisplayManager::updateBTNumber(uint16_t pending) { _btPending = pending; }

void DisplayManager::updateMeasureFrom(bool front) { _measureFromFront = front; }

void DisplayManager::showPowerPrompt(const char *const *lines, uint8_t count, float progress, const char *hint) {
    if (!_initialized) {
        return;
    }
    constexpr int16_t LINE_H = 20; // size-2 text is 16 px tall
    constexpr int16_t BAR_X = 14, BAR_W = 100, BAR_H = 14, BAR_GAP = 8;
    constexpr int16_t HINT_GAP = 10, HINT_LINE_H = 10; // size-1 text is 8 px tall
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
    _display.setTextSize(2);
    for (uint8_t i = 0; i < count; i++) {
        int16_t w = (int16_t)strlen(lines[i]) * 12;
        _display.setCursor((SH1107_WIDTH - w) / 2, y0 + i * LINE_H);
        _display.print(lines[i]);
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
    if (hint) { // '\n' separates lines, each centred
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

void DisplayManager::setSleep(bool sleep) {
    if (!_initialized) {
        return;
    }
    _display.oled_command(sleep ? SH110X_DISPLAYOFF : SH110X_DISPLAYON);
}

void DisplayManager::blankScreen() {
    if (!_initialized) {
        return;
    }
    _display.clearDisplay();
    _display.display();
}

void DisplayManager::showStartingMenu() {
    if (!_initialized) {
        return;
    }
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SH110X_WHITE);
    _display.setCursor(0, 10);
    _display.println(F("Starting"));
    _display.println(F("Menu"));
    _display.println();
    _display.println(F("If this takes longer"));
    _display.println(F(" than 10 seconds"));
    _display.println(F(" turn the device"));
    _display.println(F(" on/off again"));
    _display.println(F(" and reattempt"));
    _display.display();
}

void DisplayManager::showInitialisingMessage() {
    if (!_initialized) {
        return;
    }
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SH110X_WHITE);
    _display.setCursor(0, 10);
    _display.println(F("Device Initialising"));
    _display.println(F("Please wait..."));
    _display.display();
}

void DisplayManager::playSplash(uint8_t style, const char *nameSuffix) {
    if (!_initialized) {
        return;
    }
    Splash::play(_display, style, nameSuffix);
}

void DisplayManager::refresh() {
    if (!_initialized) {
        return;
    }
    if (_errorScreen) {
        drawErrorScreen();
    } else {
        drawMainScreen();
    }
    _display.display();
}

void DisplayManager::showErrorScreen(const char *headline, const char *detail) {
    updateDistanceText(headline, detail);
    _errorScreen = true;
}

void DisplayManager::clearErrorScreen() {
    _errorScreen = false;
    _distIsText = false;
    _distDetail[0] = '\0';
}

// ── Private drawing helpers ────────────────────────────────────────

void DisplayManager::drawMainScreen() {
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);

    // ── BT label (top-left, size 2) ─────────────────────────────
    _display.setTextSize(2);
    if (_btConnected) {
        _display.setCursor(BT_X, BT_Y);
        _display.print(F("BT"));
    }

    // ── BT pending count ────────────────────────────────────────
    if (_btPending > 0) {
        _display.setCursor(_btConnected ? BT_NUM_X : BT_X, BT_Y);
        _display.print(_btPending);
    }

    // ── Measure-from indicator (top-centre, size 2) ─────────────
    // "FR" only shown when measuring from front; blank = default back
    if (_measureFromFront) {
        _display.setTextSize(1);
        _display.setCursor((128 - 30) / 2, BT_Y + 4);
        _display.print(F("Front"));
        _display.setTextSize(2);
    }

    // ── Battery bar (top-right) ─────────────────────────────────
    drawBattery(_battery);

    // ── Distance (size 3) ───────────────────────────────────────
    _display.setTextSize(3);
    _display.setCursor(0, DIST_Y);
    if (_distIsText) {
        _display.print(_distText);
        // Remedy line under the headline — centred, size 1.
        if (_distDetail[0]) {
            size_t len = strlen(_distDetail);
            if (len > DETAIL_MAX_CHARS) {
                len = DETAIL_MAX_CHARS;
            }
            _display.setTextSize(1);
            _display.setCursor((SH1107_WIDTH - (int16_t)len * 6) / 2, DETAIL_Y);
            for (size_t i = 0; i < len; i++) {
                _display.write(_distDetail[i]);
            }
            _display.setTextSize(3);
        }
    } else if (_distance != 0.0f) {
        _display.print(_distance, 2);
        _display.print('m');
    }

    // ── Azimuth (size 3) ────────────────────────────────────────
    _display.setCursor(0, AZ_Y);
    _display.print(_azimuth, 1);
    drawDegreeSymbol(AZ_Y);

    // ── Inclination (size 3) ────────────────────────────────────
    _display.setCursor(0, INC_Y);
    if (_inclination >= 0.0f) {
        _display.print('+');
    }
    _display.print(_inclination, 1);
    drawDegreeSymbol(INC_Y);
}

void DisplayManager::drawErrorScreen() {
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);

    // Keep the status strip — battery and link state still matter mid-error,
    // and they sit above the space the readings used to take.
    _display.setTextSize(2);
    if (_btConnected) {
        _display.setCursor(BT_X, BT_Y);
        _display.print(F("BT"));
    }
    drawBattery(_battery);

    // Headline, size 3, centred. One too long for size 3 ("LOW SIGNAL") drops
    // to size 2 rather than being truncated, re-centred in the same band.
    size_t hlen = strlen(_distText);
    uint8_t hsize = 3;
    if (hlen > ERR_HEAD_MAX_CHARS) {
        hsize = 2;
        if (hlen > ERR_BODY_COLS) {
            hlen = ERR_BODY_COLS;
        }
    }
    _display.setTextSize(hsize);
    _display.setCursor((SH1107_WIDTH - (int16_t)hlen * 6 * hsize) / 2,
                       ERR_HEAD_Y + (3 - hsize) * 4);
    for (size_t i = 0; i < hlen; i++) {
        _display.write(_distText[i]);
    }

    _display.drawFastHLine(ERR_RULE_INSET, ERR_RULE_Y, SH1107_WIDTH - 2 * ERR_RULE_INSET,
                           SH110X_WHITE);

    // Remedy, size 2, wrapped and centred as a block in the lower half.
    char lines[ERR_BODY_LINES][ERR_BODY_COLS + 1];
    uint8_t n = wrapWords(_distDetail, lines, ERR_BODY_LINES);
    if (n == 0) {
        return;
    }
    int16_t blockH = (int16_t)n * ERR_BODY_LINE_H - (ERR_BODY_LINE_H - 16);
    int16_t y = ERR_BODY_TOP + ((SH1107_HEIGHT - ERR_BODY_TOP) - blockH) / 2;
    _display.setTextSize(2);
    for (uint8_t i = 0; i < n; i++) {
        int16_t len = (int16_t)strlen(lines[i]);
        _display.setCursor((SH1107_WIDTH - len * 12) / 2, y + (int16_t)i * ERR_BODY_LINE_H);
        _display.print(lines[i]);
    }
}

void DisplayManager::drawDegreeSymbol(int16_t y) {
    int16_t cx = _display.getCursorX() + DEG_OFFSET_X + DEG_RADIUS;
    int16_t cy = y + DEG_OFFSET_Y + DEG_RADIUS;
    _display.drawCircle(cx, cy, DEG_RADIUS, SH110X_WHITE);
    _display.drawCircle(cx, cy, DEG_RADIUS - 1, SH110X_WHITE);
}

void DisplayManager::drawBattery(float pct) {
    // Outline
    _display.drawRect(BAT_X, BAT_Y, BAT_W, BAT_H, SH110X_WHITE);

    // Positive terminal tip
    _display.fillRect(BAT_X + BAT_W, BAT_Y + (BAT_H - BAT_TIP_H) / 2, BAT_TIP_W, BAT_TIP_H, SH110X_WHITE);

    // Fill proportional to charge level
    int16_t fillW = (int16_t)((pct / 100.0f) * BAT_FILL_W);
    if (fillW > 0) {
        _display.fillRect(BAT_FILL_X, BAT_FILL_Y, fillW, BAT_FILL_H, SH110X_WHITE);
    }
}
