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

// Horizontal distance (optional) — the band between the status strip and
// the distance row: a size-1 "Horiz" label at the left, the value at size 2
// right-aligned. Glyph rows 18..31, clear of the distance row at 34.
static constexpr int16_t HORIZ_Y = 18;
static constexpr int16_t HORIZ_LABEL_Y = HORIZ_Y + 4;       // centred on the value
static constexpr int16_t HORIZ_KIND_LABEL_Y = HORIZ_Y + 8;  // history: under the kind

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

void DisplayManager::updateShowHorizontal(bool show) { _showHorizontal = show; }

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

void DisplayManager::showHistory(uint16_t n, uint16_t total, const char *kind, float distance,
                                 float azimuth, float inclination) {
    if (!_initialized) {
        return;
    }
    _display.clearDisplay();

    // Banner: inverted strip where BT / battery normally sit, "n of total"
    // centred at size 2 (10 columns: "500 of 500" just fits).
    char banner[12];
    snprintf(banner, sizeof(banner), "%u of %u", (unsigned)n, (unsigned)total);
    // 17 rows, not 18: leaves a black row above the horizontal distance
    _display.fillRect(0, 0, SH1107_WIDTH, 17, SH110X_WHITE);
    _display.setTextColor(SH110X_BLACK);
    _display.setTextSize(2);
    _display.setCursor((SH1107_WIDTH - (int16_t)strlen(banner) * 12) / 2, 2);
    _display.print(banner);

    // Kind, size 1, centred in the gap above the distance row. With the
    // horizontal distance in that gap it moves to the left, above "Horiz".
    _display.setTextColor(SH110X_WHITE);
    _display.setTextSize(1);
    if (_showHorizontal) {
        _display.setCursor(0, HORIZ_Y);
        _display.print(kind);
        drawHorizontal(distance, inclination, HORIZ_KIND_LABEL_Y);
    } else {
        _display.setCursor((SH1107_WIDTH - (int16_t)strlen(kind) * 6) / 2, 22);
        _display.print(kind);
    }

    _display.setTextSize(3);
    _display.setCursor(0, DIST_Y);
    _display.print(distance, 2);
    _display.print('m');

    _display.setCursor(0, AZ_Y);
    _display.print(azimuth, 1);
    drawDegreeSymbol(AZ_Y);

    _display.setCursor(0, INC_Y);
    if (inclination >= 0.0f) {
        _display.print('+');
    }
    _display.print(inclination, 1);
    drawDegreeSymbol(INC_Y);

    _display.display();
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
        // _inclination is the shot's: the display stays frozen while a
        // distance is shown (cleared to 0 on the next FIRE)
        if (_showHorizontal) {
            drawHorizontal(_distance, _inclination, HORIZ_LABEL_Y);
        }
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

// For plan-view sketching on paper. labelY places the size-1 "Horiz" label
// (the history screen stacks it under the reading's kind). Leaves text size 3.
void DisplayManager::drawHorizontal(float distance, float inclination, int16_t labelY) {
    // fabsf: cos(90 deg) in float is a hair below zero, which prints "-0.00"
    float horiz = fabsf(distance * cosf(inclination * (float)M_PI / 180.0f));
    char text[12];
    snprintf(text, sizeof(text), "%.2fm", (double)horiz);

    _display.setTextSize(1);
    _display.setCursor(0, labelY);
    _display.print(F("Horiz"));

    _display.setTextSize(2);
    _display.setCursor(SH1107_WIDTH - (int16_t)strlen(text) * 12, HORIZ_Y);
    _display.print(text);
    _display.setTextSize(3);
}

void DisplayManager::drawDegreeSymbol(int16_t y) {
    int16_t cx = _display.getCursorX() + DEG_OFFSET_X + DEG_RADIUS;
    int16_t cy = y + DEG_OFFSET_Y + DEG_RADIUS;
    _display.drawCircle(cx, cy, DEG_RADIUS, SH110X_WHITE);
    _display.drawCircle(cx, cy, DEG_RADIUS - 1, SH110X_WHITE);
}

void DisplayManager::showChargingScreen(float pct, bool havePct, const char *status, const char *note,
                                        const char *const *hints) {
    if (!_initialized) {
        return;
    }
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);
    drawBattery(pct); // same place as on the main screen, so any wear is shared
    drawChargingText(pct, havePct, status, SIZE_MAX, note, hints, true);
    _display.display();
}

void DisplayManager::drawChargingText(float pct, bool havePct, const char *status, size_t statusChars,
                                      const char *note, const char *const *hints, bool showHints) {
    // Same rows as the distance / azimuth lines of the main screen
    char text[12];
    if (havePct) {
        snprintf(text, sizeof(text), "%.2f%%", (double)pct);
    } else {
        strcpy(text, "--.--%");
    }
    _display.setTextSize(3);
    _display.setCursor((SH1107_WIDTH - (int16_t)strlen(text) * 18) / 2, DIST_Y);
    _display.print(text);

    if (status && statusChars > 0) {
        _display.setTextSize(2);
        _display.setCursor((SH1107_WIDTH - (int16_t)strlen(status) * 12) / 2, AZ_Y + 2);
        for (size_t i = 0; status[i] && i < statusChars; i++) {
            _display.write(status[i]);
        }
    }

    if (!showHints) {
        return;
    }
    _display.setTextSize(1);
    // Note: between the battery icon (rows 0-14) and the % (from row 34)
    if (note) {
        _display.setCursor((SH1107_WIDTH - (int16_t)strlen(note) * 6) / 2, 20);
        _display.print(note);
    }
    // Hints: under the status (rows 70-85), at rows 92, 104, 116
    for (uint8_t i = 0; hints && i < CHARGING_HINTS; i++) {
        if (hints[i]) {
            _display.setCursor((SH1107_WIDTH - (int16_t)strlen(hints[i]) * 6) / 2, 92 + i * 12);
            _display.print(hints[i]);
        }
    }
}

// ── Charging intro ──────────────────────────────────────────────────
// Timeline (ms). Each frame is a pure function of t, as for the boot splash,
// so it runs at whatever rate the I2C refresh allows (~20 fps).
static constexpr uint32_t CI_BOLT_END = 300;    // bolt grows from the top edge to the battery
static constexpr uint32_t CI_FLASH_END = 370;   // whole panel white
static constexpr uint32_t CI_RING_END = 750;    // shockwave from the strike
static constexpr uint32_t CI_POP_START = 330;   // big battery scales up...
static constexpr uint32_t CI_POP_END = 620;     // ...with a little overshoot
static constexpr uint32_t CI_FILL_START = 620;  // fills to the real level, % counts up
static constexpr uint32_t CI_FILL_END = 1450;
static constexpr uint32_t CI_FLY_START = 1500;  // shrinks into the icon's corner
static constexpr uint32_t CI_FLY_END = 1950;
static constexpr uint32_t CI_TYPE_START = 1800; // status types in
static constexpr uint32_t CI_TYPE_END = 2050;
static constexpr uint32_t CI_HINTS = 2080;      // hints appear

// Big battery, centred under the % row (its tip counts towards the width)
static constexpr float CI_BIG_W = 64.0f;
static constexpr float CI_BIG_H = 32.0f;
static constexpr float CI_BIG_X = (SH1107_WIDTH - CI_BIG_W - 6.0f) / 2.0f;
static constexpr float CI_BIG_Y = 70.0f;
static constexpr int16_t CI_CX = (int16_t)(CI_BIG_X + CI_BIG_W / 2);
static constexpr int16_t CI_CY = (int16_t)(CI_BIG_Y + CI_BIG_H / 2);

static float ciClamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
static float ciPhase(uint32_t t, uint32_t a, uint32_t b) {
    return ciClamp01((float)((int32_t)t - (int32_t)a) / (float)(b - a));
}
static float ciEaseOut(float v) {
    const float u = 1.0f - v;
    return 1.0f - u * u * u;
}
static float ciEaseInOut(float v) {
    if (v < 0.5f) {
        return 4.0f * v * v * v;
    }
    const float u = -2.0f * v + 2.0f;
    return 1.0f - u * u * u / 2.0f;
}
static float ciEaseOutBack(float v) {
    const float c1 = 1.70158f, c3 = c1 + 1.0f;
    const float u = v - 1.0f;
    return 1.0f + c3 * u * u * u + c1 * u * u;
}
static float ciLerp(float a, float b, float k) { return a + (b - a) * k; }
static uint32_t ciHash(uint32_t a, uint32_t b) {
    uint32_t h = a * 374761393u + b * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

void DisplayManager::drawBatteryAt(float x, float y, float w, float h, float frac, float wave, uint32_t t) {
    const int16_t bx = (int16_t)lroundf(x), by = (int16_t)lroundf(y);
    const int16_t bw = (int16_t)lroundf(w), bh = (int16_t)lroundf(h);
    if (bw < 4 || bh < 4) {
        return;
    }
    const bool big = bw > 44; // thick walls and a gap round the fill when big
    _display.drawRect(bx, by, bw, bh, SH110X_WHITE);
    if (big) {
        _display.drawRect(bx + 1, by + 1, bw - 2, bh - 2, SH110X_WHITE);
    }
    // Tip in the main icon's proportions (3x6 on 32x15)
    const int16_t tipW = (int16_t)lroundf(w * 3.0f / 32.0f);
    const int16_t tipH = (int16_t)lroundf(h * 6.0f / 15.0f);
    _display.fillRect(bx + bw, by + (bh - tipH) / 2, tipW < 1 ? 1 : tipW, tipH, SH110X_WHITE);

    const int16_t inset = big ? 4 : 1;
    const int16_t fx = bx + inset, fy = by + inset;
    const int16_t fw = bw - 2 * inset, fh = bh - 2 * inset;
    const float level = ciClamp01(frac) * (float)fw;
    if (fw <= 0 || fh <= 0 || level <= 0.0f) {
        return;
    }
    for (int16_t r = 0; r < fh; r++) {
        const float edge = level + wave * sinf((float)r * 0.55f + (float)t * 0.018f);
        int16_t len = (int16_t)lroundf(edge);
        if (len > fw) {
            len = fw;
        }
        if (len > 0) {
            _display.drawFastHLine(fx, fy + r, len, SH110X_WHITE);
        }
    }
}

// The charge symbol: top slab slanting down-left, a jog right, bottom slab.
// One span per row, drawn INVERSE so it reads both on the fill and off it.
static void ciChargeSymbol(Adafruit_SH1107 &d, int16_t cx, int16_t cy) {
    for (int16_t y = -11; y <= 11; y++) {
        int16_t left;
        int16_t w = 5;
        if (y < 0) {
            left = cx + 1 - (y + 11) / 2;
        } else if (y < 2) {
            left = cx - 5;
            w = 11;
        } else {
            left = cx + 1 - (y - 2) / 2;
        }
        d.drawFastHLine(left, cy + y, w, SH110X_INVERSE);
    }
}

void DisplayManager::showChargingIntro(uint32_t t, float pct, bool havePct, const char *status,
                                       const char *note, const char *const *hints) {
    if (!_initialized) {
        return;
    }
    if (t >= CHARGING_INTRO_MS) {
        showChargingScreen(pct, havePct, status, note, hints);
        return;
    }
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);
    const float frac = havePct ? pct / 100.0f : 0.0f;

    // Flash: the strike lights the whole panel for a frame or two
    if (t >= CI_BOLT_END && t < CI_FLASH_END) {
        _display.fillScreen(SH110X_WHITE);
        _display.display();
        return;
    }

    // Lightning bolt from above the top edge down onto the battery
    if (t < CI_BOLT_END) {
        static const int16_t kBolt[][2] = {{74, -4}, {56, 22}, {72, 28}, {52, 52}, {68, 56}, {CI_CX, CI_CY - 2}};
        constexpr uint8_t kN = sizeof(kBolt) / sizeof(kBolt[0]);
        float total = 0.0f;
        for (uint8_t i = 1; i < kN; i++) {
            total += hypotf((float)(kBolt[i][0] - kBolt[i - 1][0]), (float)(kBolt[i][1] - kBolt[i - 1][1]));
        }
        float remain = total * ciEaseOut(ciPhase(t, 0, CI_BOLT_END));
        for (uint8_t i = 1; i < kN && remain > 0.0f; i++) {
            const float x0 = kBolt[i - 1][0], y0 = kBolt[i - 1][1];
            const float dx = kBolt[i][0] - x0, dy = kBolt[i][1] - y0;
            const float len = hypotf(dx, dy);
            const float k = remain >= len ? 1.0f : remain / len;
            remain -= len;
            const int16_t x1 = (int16_t)lroundf(x0 + dx * k), y1 = (int16_t)lroundf(y0 + dy * k);
            for (int16_t o = -1; o <= 1; o++) {
                _display.drawLine((int16_t)x0 + o, (int16_t)y0, x1 + o, y1, SH110X_WHITE);
            }
        }
    }

    // Shockwave from the strike
    if (t >= CI_FLASH_END && t < CI_RING_END) {
        const float k = ciEaseOut(ciPhase(t, CI_FLASH_END, CI_RING_END));
        const int16_t r = (int16_t)lroundf(8.0f + 70.0f * k);
        _display.drawCircle(CI_CX, CI_CY, r, SH110X_WHITE);
        if (k < 0.6f) {
            _display.drawCircle(CI_CX, CI_CY, r - 3, SH110X_WHITE);
        }
    }

    // The % counts up with the fill; the status types in as the battery lands.
    // Drawn before the battery so the battery passes in front on its way up.
    if (t >= CI_FILL_START) {
        const float shown = havePct ? pct * ciEaseOut(ciPhase(t, CI_FILL_START, CI_FILL_END)) : 0.0f;
        size_t chars = 0;
        if (status && t >= CI_TYPE_START) {
            const size_t len = strlen(status);
            chars = (size_t)(ciPhase(t, CI_TYPE_START, CI_TYPE_END) * (float)len + 0.999f);
        }
        drawChargingText(shown, havePct, status, chars, note, hints, t >= CI_HINTS);
    }

    // The battery: pops in, fills, then flies into the corner
    if (t >= CI_POP_START) {
        float x, y, w, h, wave = 0.0f;
        const float level = frac * ciEaseOut(ciPhase(t, CI_FILL_START, CI_FILL_END));
        if (t < CI_FLY_START) {
            const float s = ciEaseOutBack(ciPhase(t, CI_POP_START, CI_POP_END));
            w = CI_BIG_W * s;
            h = CI_BIG_H * s;
            x = CI_CX - w / 2.0f;
            y = CI_CY - h / 2.0f;
            if (t >= CI_FILL_START) {
                wave = 2.0f;
            }
        } else {
            const float k = ciEaseInOut(ciPhase(t, CI_FLY_START, CI_FLY_END));
            x = ciLerp(CI_BIG_X, BAT_X, k);
            y = ciLerp(CI_BIG_Y, BAT_Y, k);
            w = ciLerp(CI_BIG_W, BAT_W, k);
            h = ciLerp(CI_BIG_H, BAT_H, k);
            wave = 2.0f * (1.0f - k);
        }
        if (t >= CI_FLY_END) {
            drawBattery(pct); // the real icon, already where it stays
        } else {
            if (t >= CI_FLY_START) { // opaque, so it slides over the % row cleanly
                _display.fillRect((int16_t)x - 2, (int16_t)y - 2, (int16_t)(w * 35.0f / 32.0f) + 5,
                                  (int16_t)h + 5, SH110X_BLACK);
            }
            drawBatteryAt(x, y, w, h, level, wave, t);
        }

        if (t >= CI_FILL_START && t < CI_FLY_START) {
            // Sparks spraying off the advancing fill edge, kept inside
            const int16_t fx = (int16_t)lroundf(x) + 4, fy = (int16_t)lroundf(y) + 4;
            const int16_t fw = (int16_t)lroundf(w) - 8, fh = (int16_t)lroundf(h) - 8;
            const float front = fx + level * fw;
            if (t < CI_FILL_END && fh > 0) {
                for (uint32_t i = 0; i < 7; i++) {
                    const uint32_t cyc = (t + i * 61) / 220;
                    const float age = (float)((t + i * 61) % 220) / 220.0f;
                    const uint32_t hsh = ciHash(i, cyc);
                    const float vx = 4.0f + (float)(hsh % 9);
                    const float vy = (float)((int32_t)((hsh >> 8) % 13) - 6);
                    const float sy = fy + (float)((hsh >> 16) % (uint32_t)fh);
                    const int16_t px = (int16_t)lroundf(front + vx * age);
                    const int16_t py = (int16_t)lroundf(sy + vy * age);
                    if (px >= fx && px < fx + fw - 1 && py >= fy && py < fy + fh) {
                        _display.fillRect(px, py, 2, 1, SH110X_INVERSE);
                    }
                }
            }
            ciChargeSymbol(_display, CI_CX, CI_CY);
        }
    }
    _display.display();
}

void DisplayManager::drawBattery(float pct) {
    // Charging: the charged part stays solid and the rest fills up in five
    // steps, one per 250 ms refresh (~1.25 s a sweep), then starts again.
    if (_batMode == BatteryMode::CHARGING && pct < 99.5f) {
        const uint32_t frame = (millis() / 250) % 5; // 0 = the real level, 4 = full
        pct += (100.0f - pct) * (float)frame / 4.0f;
    }

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
