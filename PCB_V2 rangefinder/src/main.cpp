// DiscoX rangefinder-only firmware — see ../CLAUDE.md in this folder.
//
// The device as a laser distance meter: FIRE aims, FIRE again measures.
// MENU opens a menu with the two timeouts, a firmware update and power off.
// No compass/inclinometer, calibration, BLE, reading history, disco, snake
// or USB settings drive. Everything that is here behaves as it does in the
// full firmware (../PCB_V2/src/main.cpp), and most of it is a trimmed copy:
// look there for the long-form reasoning behind each piece.

#include "button_manager.h"
#include "config.h"
#include "defaults.h"
#include "device_number.h"
#include "drivers/buzzer.h"
#include "drivers/charger.h"
#include "drivers/max17048.h"
#include "drivers/power.h"
#include "laser_manager.h"
#include "led.h"
#include "rf_display.h"
#include "rf_menu.h"
#include "settings.h"
#include "sounds.h"
#include "version.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <Wire.h>
#include <flash/flash_nrf5x.h>
#include <string.h>

// ── Hardware ────────────────────────────────────────────────────────
static ButtonManager buttons;
static RfDisplay display;
static LaserManager laser;
static Buzzer buzzer;
static PowerControl power;
static Charger charger;
static MAX17048_Persistent battery;
static RfMenu menu;
static Settings settings;

static bool dispOk = false;
static bool laserOk = false;
static bool batOk = false;

// ── State ───────────────────────────────────────────────────────────
static bool laserOn = false;          // beam on: the next FIRE measures
static float distance = 0.0f;         // last good reading, m (0 = none yet: shows 0.00)
static uint32_t lastActivity = 0;     // millis() of the last button press
static uint32_t errorUntil = 0;       // error screen showing until then (0 = none)
static uint32_t ledOffAt = 0;         // green reading LED goes out then (0 = not lit)
static bool screenDirty = true;
static BatteryMode shownBatMode = BatteryMode::NORMAL; // icon mode last drawn
static float lastBatPct = 0.0f;
static uint32_t lastBatRead = 0;
static uint32_t lastDisplayRefresh = 0;
static uint32_t lastAutoOffCheck = 0;
static uint32_t lastLaserCheck = 0;

static void doShutdown();
static void showMain();

// ── Boot marks in GPREGRET2 (same values as the full firmware) ──────
// GPREGRET2 survives resets and the UF2 bootloader; only a real power loss
// clears it. APP_RAN_MARK written at every boot means "a reboot, not a
// power-on" next time (so a UF2 update on USB doesn't open the charging
// screen). STORAGE_FAULT_MARK is left by a littlefs assert (below). Using
// the full firmware's values keeps the two firmwares' warm/cold logic in
// agreement when one is updated to the other.
static constexpr uint8_t APP_RAN_MARK = 0x5C;
static constexpr uint8_t STORAGE_FAULT_MARK = 0xA7;
static uint8_t bootGpregret2 = 0;
static uint32_t bootResetReas = 0;
static bool warmBoot() { return bootGpregret2 == APP_RAN_MARK || bootGpregret2 == STORAGE_FAULT_MARK; }
static bool coldPowerOn() {
    const uint32_t notPowerOn = POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_LOCKUP_Msk |
                                POWER_RESETREAS_RESETPIN_Msk;
    return !(bootResetReas & notPowerOn) && !warmBoot();
}

// A damaged LittleFS trips a littlefs assert rather than returning an
// error. Reboot and run this session without storage (defaults, nothing
// saved) instead of freezing. Repairing it is the full firmware's job: it
// has the factory reset and the backup on the FAT partition. The BLE
// SoftDevice is never started here, so POWER is ours to write directly.
extern "C" [[noreturn]] void __assert_func(const char *file, int line, const char *func, const char *expr) {
    Serial.print(F("assertion \""));
    Serial.print(expr ? expr : "?");
    Serial.print(F("\" failed: "));
    Serial.print(file ? file : "?");
    Serial.print(':');
    Serial.println(line);
    (void)func;
    if (file && (strstr(file, "littlefs") || strstr(file, "LittleFS"))) {
        for (int i = 0; i < 200; i++) {
            delayMicroseconds(1000); // let the log drain; no delay() in here
        }
        NRF_POWER->GPREGRET2 = STORAGE_FAULT_MARK;
        NVIC_SystemReset();
    }
    // Any other assert is a bug: halt, but keep the power button working
    while (true) {
        if (power.buttonPressed()) {
            power.powerOff();
        }
        for (int i = 0; i < 10; i++) {
            delayMicroseconds(1000);
        }
    }
}

// ── Power button (LTC2954 INT) ──────────────────────────────────────
// True once per confirmed press (20 ms LOW); disarmed until released, so
// the power-on press can't power straight back off. On 2026-10-18+ boards
// the USB auto-on circuit also pulls INT low at every plug-in: a press
// within 300 ms of VBUS rising is that pulse, not the user.
static bool pwrBtnArmed = false;
static uint32_t pwrBtnLowSince = 0;
static bool vbusWasPresent = false;
static uint32_t vbusRoseAt = 0;
static constexpr uint32_t PLUG_PULSE_WINDOW_MS = 300;

static bool powerButtonPressed(uint32_t now) {
    const bool pressed = power.buttonPressed();
    const bool vbus = Charger::usbPresent();
    if (vbus && !vbusWasPresent) {
        vbusRoseAt = now ? now : 1;
    }
    vbusWasPresent = vbus;

    if (!pwrBtnArmed) {
        if (!pressed) {
            pwrBtnArmed = true;
        }
        return false;
    }
    if (!pressed) {
        pwrBtnLowSince = 0;
        return false;
    }
    if (pwrBtnLowSince == 0) {
        pwrBtnLowSince = now;
    } else if (now - pwrBtnLowSince >= 20) {
        pwrBtnArmed = false;
        pwrBtnLowSince = 0;
        if (vbusRoseAt != 0 && now - vbusRoseAt < PLUG_PULSE_WINDOW_MS) {
            Serial.println(F("POWER: USB plug-in pulse ignored"));
            return false;
        }
        return true;
    }
    return false;
}

// Hold to power off, with a bar. There is no standby in this firmware, so a
// press released early just says how to switch off and carries on.
static bool pwrHolding = false;
static uint32_t pwrHoldStart = 0;
static uint32_t pwrHoldLastDraw = 0;

static void drainButtonEdges() {
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        buttons.wasPressed(static_cast<Button>(i));
        buttons.wasReleased(static_cast<Button>(i));
    }
}

static void beginPowerHold(uint32_t now) {
    Serial.println(F("POWER: button down - hold to power off"));
    pwrHolding = true;
    pwrHoldStart = now;
    pwrHoldLastDraw = 0;
    flash_nrf5x_flush(); // the LTC2954 can cut power by itself on a long hold
}

static void servicePowerHold(uint32_t now) {
    const uint32_t held = now - pwrHoldStart;
    if (power.buttonPressed()) {
        if (held >= Timing::POWER_HOLD_OFF_MS) {
            static const char *const kOff[] = {"Powering", "off"};
            display.showPowerPrompt(kOff, 2, 1.0f);
            doShutdown();
        }
        if (dispOk && held >= Timing::POWER_BAR_DELAY_MS && (pwrHoldLastDraw == 0 || now - pwrHoldLastDraw >= 40)) {
            static const char *const kHold[] = {"Hold to", "power off"};
            display.showPowerPrompt(kHold, 2, (float)held / Timing::POWER_HOLD_OFF_MS);
            pwrHoldLastDraw = now;
        }
        return;
    }
    pwrHolding = false;
    if (dispOk) {
        static const char *const kTap[] = {"Hold to", "power off"};
        display.showPowerPrompt(kTap, 2);
        delay(1000);
    }
    menu.close(); // the prompt covered it: back to the rangefinder screen
    errorUntil = 0;
    drainButtonEdges();
    lastActivity = millis();
    showMain();
}

// Cold power-on must be a hold, not a tap (a knock in a bag shouldn't turn
// it on). Skipped on USB and after any reset. Fail-safe: if INT is never
// seen LOW, boot normally rather than risk a device that never turns on.
static void bootHoldGate() {
    const bool usb = NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
    if (!dispOk || usb || !coldPowerOn()) {
        return;
    }
    const uint32_t t0 = millis();
    while (!power.buttonPressed()) {
        if (millis() - t0 > 30) {
            return;
        }
    }
    static const char *const kHold[] = {"Hold to", "power on"};
    uint32_t lastDraw = 0;
    for (;;) {
        uint32_t held = millis() - t0;
        if (held >= Timing::POWER_HOLD_ON_MS) {
            display.showPowerPrompt(kHold, 2, 1.0f);
            return;
        }
        if (!power.buttonPressed()) {
            static const char *const kTap[] = {"Hold the", "power", "button to", "turn on"};
            display.showPowerPrompt(kTap, 4);
            delay(Timing::POWER_TAP_NOTICE_MS);
            display.blank();
            power.powerOff(); // nothing has touched storage yet
        }
        if (lastDraw == 0 || millis() - lastDraw >= 40) {
            display.showPowerPrompt(kHold, 2, (float)held / Timing::POWER_HOLD_ON_MS);
            lastDraw = millis();
        }
    }
}

// ── Power off ───────────────────────────────────────────────────────
static void doShutdown() {
    Serial.println(F(">>> SHUTDOWN"));
    if (laserOk) {
        laser.setLaser(false);
    }
    Led::off();
    flash_nrf5x_flush(); // commit the flash HAL's page cache (settings saves)
    Serial.flush();
    delay(100);
    display.blank();
    power.powerOff(); // KILL LOW — does not return
}

// ── Battery ─────────────────────────────────────────────────────────
static BatteryMode batteryMode() {
    if (!Charger::usbPresent()) {
        return BatteryMode::NORMAL;
    }
    if (charger.hasControl()) { // 2026-10-18+ boards report CHG
        return charger.chargingSignal() ? BatteryMode::CHARGING : BatteryMode::FULL;
    }
    return lastBatPct < 99.0f ? BatteryMode::CHARGING : BatteryMode::FULL;
}

static void lowBatteryShutdown() {
    Serial.println(F("LOW BATTERY - shutting down"));
    static const char *const kLow[] = {"Battery", "low"};
    display.showPowerPrompt(kLow, 2, -1.0f, "Charge before use");
    Led::red();
    delay(3000);
    doShutdown();
}

static void pollBattery(uint32_t now) {
    if (!batOk || now - lastBatRead < Timing::BATTERY_CHECK_MS) {
        return;
    }
    lastBatRead = now;
    lastBatPct = battery.cellPercent();
    Serial.print(F("BAT: "));
    Serial.print(battery.cellVoltage(), 3);
    Serial.print(F("V "));
    Serial.print(lastBatPct, 1);
    Serial.println('%');
    screenDirty = true;
    // 0% = no cell fitted (USB only); on USB the cell doesn't run the device
    if (!Charger::usbPresent() && lastBatPct > 0.5f && lastBatPct <= Timing::BATTERY_SHUTDOWN_PCT) {
        lowBatteryShutdown();
    }
}

// ── Screens ─────────────────────────────────────────────────────────
static void showMain() {
    display.setDistance(distance);
    display.setHint(laserOn ? LaserHint::AIMING : LaserHint::OFF);
    shownBatMode = batteryMode();
    display.setBattery(lastBatPct, shownBatMode);
    display.refresh();
    lastDisplayRefresh = millis();
    screenDirty = false;
}

// Plugged in while off (2026-10-18+ boards switch on at plug-in): show the
// charge instead of the rangefinder, and don't burn the panel in. Any of
// the four buttons carries on to the rangefinder (or relights a dark
// screen); the power button or unplugging switches off. A trimmed version
// of the full firmware's runChargingScreen.
static void runChargingScreen() {
    Serial.println(F("Charging screen (cold power-on on USB)"));
    static constexpr uint32_t DIM_MS = 60000, OFF_MS = 120000;
    uint32_t litSince = millis();
    bool dark = false, dimmed = false;
    uint32_t lastDraw = 0;
    drainButtonEdges();
    for (;;) {
        const uint32_t now = millis();
        if (!Charger::usbPresent()) {
            Serial.println(F("Unplugged - powering off"));
            doShutdown();
        }
        if (powerButtonPressed(now)) {
            doShutdown();
        }
        buttons.update();
        bool any = false;
        for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
            any |= buttons.wasPressed(static_cast<Button>(i));
        }
        if (any) {
            if (!dark && !dimmed) {
                // Let the press go before the rangefinder sees the button
                while (buttons.isPressed(Button::FIRE) || buttons.isPressed(Button::MENU) ||
                       buttons.isPressed(Button::UP_DISCO) || buttons.isPressed(Button::DOWN)) {
                    buttons.update();
                    delay(10);
                }
                drainButtonEdges();
                display.setBrightness(settings.screenBrightness);
                return;
            }
            dark = dimmed = false;
            display.setSleep(false);
            display.setBrightness(settings.screenBrightness);
            litSince = now;
            lastDraw = 0;
        }
        if (!dark && now - litSince >= OFF_MS) {
            dark = true;
            display.setSleep(true);
        } else if (!dimmed && now - litSince >= DIM_MS) {
            dimmed = true;
            display.setBrightness(16);
        }
        if (!dark && (lastDraw == 0 || now - lastDraw >= 250)) {
            if (batOk && (lastBatRead == 0 || now - lastBatRead >= 2000)) {
                lastBatPct = battery.cellPercent();
                lastBatRead = now;
            }
            BatteryMode mode = batteryMode();
            display.setBattery(lastBatPct, mode);
            display.showCharging(lastBatPct, batOk, mode == BatteryMode::FULL ? "Charged" : "Charging",
                                 "Any button: use", "Power button: off");
            lastDraw = now;
        }
        delay(20);
    }
}

// ── Firmware update ─────────────────────────────────────────────────
// Reboots into the UF2 bootloader, which shows up on the PC as a USB drive
// to drop a .uf2 onto. The bootloader only waits ~3 s for USB after a
// software entry before falling back to the app, so wait for the PC link
// first (see the "Update Firmware needs USB" gotcha in ../PCB_V2/CLAUDE.md).
static void enterFirmwareUpdate() {
    Serial.println(F("Firmware update requested"));
    if (laserOk) {
        laser.setLaser(false);
    }
    laserOn = false;
    Led::off();
    if (!TinyUSBDevice.mounted()) {
        Serial.println(F("Waiting for USB before bootloader entry..."));
        auto &d = display.raw();
        if (dispOk) {
            d.clearDisplay();
            d.setTextSize(1);
            d.setTextColor(SH110X_WHITE);
            d.setCursor(0, 10);
            d.println(F("Firmware update"));
            d.println();
            d.println(F("Plug device into PC"));
            d.println(F("to continue..."));
            d.println();
            d.println(F("MENU: cancel"));
            d.display();
        }
        drainButtonEdges();
        const uint32_t start = millis();
        while (!TinyUSBDevice.mounted()) {
            const uint32_t now = millis();
            if (powerButtonPressed(now)) {
                doShutdown();
            }
            buttons.update();
            if (buttons.wasPressed(Button::MENU) || now - start > settings.autoShutdownTimeout * 1000UL) {
                Serial.println(F("Firmware update cancelled"));
                drainButtonEdges();
                lastActivity = millis();
                showMain();
                return;
            }
            delay(10);
        }
    }
    Serial.println(F("Entering UF2 bootloader..."));
    if (dispOk) {
        auto &d = display.raw();
        d.clearDisplay();
        d.setTextSize(1);
        d.setTextColor(SH110X_WHITE);
        d.setCursor(0, 10);
        d.println(F("Entered bootloader"));
        d.println();
        d.println(F("Copy new firmware"));
        d.println(F("(.uf2) onto the"));
        d.println(F("USB drive."));
        d.println();
        d.println(F("Hold power button"));
        d.println(F("to exit."));
        d.display();
        delay(1500);
    }
    flash_nrf5x_flush();
    Serial.flush();
    enterUf2Dfu(); // does not return
}

// ── Serial: 'u' = clean bootloader entry for a host-driven reflash ──
// Same command as the full firmware, so the upload skill / memory flow
// works unchanged: flush storage, then DFU with USB already up. Polled in
// every mode (the full firmware only takes it on the main screen).
static void pollSerial() {
    while (Serial.available()) {
        int c = Serial.read();
        if (c == 'u' || c == 'U') {
            Serial.println(F("Reflash: syncing storage, entering bootloader..."));
            flash_nrf5x_flush();
            Serial.flush();
            delay(100);
            enterUf2Dfu();
        } else if (c == 'v' || c == 'V') {
            char label[24];
            DeviceNumber::versionLabel(label, sizeof(label));
            Serial.print(F("Rangefinder firmware "));
            Serial.println(label);
        }
    }
}

// ── Laser ───────────────────────────────────────────────────────────
static void initLaser() {
    // LDJ-100 on Serial1; RX needs the pull-up (open-drain TXD). Probe
    // retried past the module's ~2.5 s auto-baud window on a cold boot — a
    // single early ping latches "no laser" for the session.
    Serial1.setPins(PIN_LASER_RX, PIN_LASER_TX);
    pinMode(PIN_LASER_RX, INPUT_PULLUP);
    Serial1.begin(LASER_UART_BAUD_LDJ100);
    delay(20);
    for (int attempt = 0; attempt < 15 && !laserOk; attempt++) {
        laserOk = laser.begin(Serial1);
        if (!laserOk) {
            delay(100);
        }
    }
    if (laserOk) {
        laser.setLaser(false);
    }
}

static void resetLaser() {
    if (!laserOk) {
        return;
    }
    Serial.println(F("LASER: resetting"));
    laser.setLaser(false);
    delay(100);
    while (Serial1.available()) {
        Serial1.read();
    }
    laser.setLaser(true);
    delay(200);
}

static void setLaserOn(bool on) {
    if (laserOk) {
        laser.setLaser(on);
    }
    laserOn = on && laserOk;
    screenDirty = true;
}

static void showError(const char *headline, const char *detail) {
    display.showError(headline, detail);
    // Four red flashes, then the womp (Feedback::failed in the full firmware)
    for (int i = 0; i < 4; i++) {
        Led::red();
        delay(100);
        Led::off();
        delay(100);
    }
    Sounds::error();
    errorUntil = millis() + Timing::ERROR_SCREEN_MS;
}

static void takeMeasurement() {
    display.setHint(LaserHint::MEASURING);
    display.refresh();
    Led::red();
    Sounds::shotStart();

    while (Serial1.available()) {
        Serial1.read(); // stale bytes
    }
    delay(30);
    int32_t mm = 0;
    LaserError err = laser.measureValidated(mm, Defaults::laserShots, Defaults::laserSqLimit,
                                            Defaults::laserSpreadLimitMm);
    Serial.print(F("MEAS: "));
    Serial.print(LaserManager::errorString(err));
    Serial.print(F(" mm="));
    Serial.println(mm);

    if (err != LaserError::OK) {
        // Same headlines and remedies as the full firmware's laser errors
        const char *code = "LASER";
        const char *detail = "Bad reply\nResetting";
        switch (err) {
        case LaserError::TOO_DIM:
            code = "LOW SIGNAL";
            detail = "Use\nreflective\ntarget";
            break;
        case LaserError::TOO_BRIGHT:
            code = "GLARE";
            detail = "Too bright\nShade it";
            break;
        case LaserError::UNSTABLE:
            code = "SURFACE";
            detail = "Dark/shiny\nUse card";
            break;
        case LaserError::BAD_READING:
            code = "RANGE";
            detail = "No target\nin range";
            break;
        case LaserError::TOO_NEAR:
            code = "NEAR";
            detail = "Too close\nMin 0.03m";
            break;
        case LaserError::TOO_FAR:
            code = "FAR";
            detail = "Too far\nMax 100m";
            break;
        case LaserError::TIMEOUT:
            detail = "No reply\nResetting";
            resetLaser();
            break;
        default:
            resetLaser();
            break;
        }
        showError(code, detail);
        setLaserOn(true); // still aiming: FIRE again re-measures
        return;
    }

    distance = mm / 1000.0f + (settings.measureFromFront ? -Defaults::laserFrontOffset : Defaults::laserDistanceOffset);
    Serial.print(F("MEAS OK: DIST="));
    Serial.println(distance, 3);
    setLaserOn(false); // as after a survey shot: FIRE wakes it for the next one
    showMain();        // the number first, then the (blocking) bleep
    Led::green();
    Sounds::readingOk();
    ledOffAt = millis() + 2000;
}

// ── Normal-mode buttons ─────────────────────────────────────────────
static void openMenu() {
    Serial.println(F("MENU pressed"));
    setLaserOn(false);
    Led::off();
    ledOffAt = 0;
    errorUntil = 0;
    menu.begin(display.raw(), settings);
}

static void pollButtons(uint32_t now) {
    if (buttons.wasPressed(Button::FIRE)) {
        lastActivity = now;
        errorUntil = 0;
        Led::off();
        ledOffAt = 0;
        if (!laserOk) {
            showError("LASER", "Not found\nRestart");
            return;
        }
        if (laserOn) {
            takeMeasurement();
        } else {
            setLaserOn(true);
            Serial.println(F("MEASURE: laser on, aiming"));
            showMain();
        }
        return;
    }
    if (buttons.wasPressed(Button::MENU)) {
        lastActivity = now;
        openMenu();
        return;
    }
    if (buttons.wasPressed(Button::DOWN)) { // beam off between shots, saves battery
        lastActivity = now;
        errorUntil = 0;
        if (laserOn) {
            Serial.println(F("DOWN: laser off"));
        }
        setLaserOn(false);
        return;
    }
    if (buttons.wasPressed(Button::UP_DISCO)) {
        lastActivity = now; // unused, but counts as activity
        errorUntil = 0;
        screenDirty = true;
    }
}

static void handleMenuExit(MenuExit e) {
    switch (e) {
    case MenuExit::UPDATE:
        enterFirmwareUpdate();
        break;
    case MenuExit::POWER_OFF:
    case MenuExit::TIMED_OUT:
        doShutdown();
        break;
    default:
        break;
    }
    drainButtonEdges();
    lastActivity = millis();
    showMain();
}

// ═══════════════════════════════════════════════════════════════════
void setup() {
    bootResetReas = NRF_POWER->RESETREAS;
    bootGpregret2 = NRF_POWER->GPREGRET2;
    NRF_POWER->GPREGRET2 = APP_RAN_MARK; // also consumes a storage-fault mark

    power.begin(PIN_KILL, PIN_PB_INT); // KILL stays high-Z
    charger.begin(PIN_BOARD_ID, PIN_CHG, PIN_PGOOD, PIN_CHG_EN);
    Led::begin();
    buzzer.begin(PIN_BUZZER_A, PIN_BUZZER_B);
    Sounds::begin(buzzer);

    // Display before anything slow, so the hold-to-power-on bar appears
    // while the button is still down. V2 I2C pins are not the variant's.
    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);
    dispOk = display.begin();

    bootHoldGate();
    initLaser();

    pinMode(PIN_BTN_FIRE, INPUT_PULLUP);
    pinMode(PIN_BTN_UP_DISCO, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_MENU, INPUT_PULLUP);
    delayMicroseconds(50);
    buttons.begin();

    Serial.begin(115200);
    char label[24];
    DeviceNumber::versionLabel(label, sizeof(label));
    Serial.println(F("=== DiscoX rangefinder (PCB V2) ==="));
    Serial.print(F("Firmware    "));
    Serial.println(label);
    Serial.print(F("SH1107:     "));
    Serial.println(dispOk ? F("OK") : F("FAILED"));
    Serial.print(F("Laser:      "));
    Serial.println(laserOk ? F("OK") : F("FAILED"));

    batOk = battery.begin(&Wire);
    if (batOk) {
        delay(100);
        lastBatPct = battery.cellPercent();
        lastBatRead = millis();
    }
    Serial.print(F("MAX17048:   "));
    Serial.println(batOk ? F("OK") : F("FAILED"));

    const bool storageFault = bootGpregret2 == STORAGE_FAULT_MARK;
    if (!storageFault) {
        SettingsStore::begin();
    }
    SettingsStore::load(settings); // defaults when there's no storage
    display.setBrightness(settings.screenBrightness);

    if (charger.hasControl() && Charger::usbPresent() && coldPowerOn()) {
        runChargingScreen();
    }

    if (storageFault) {
        Serial.println(F("Storage fault last boot - running without storage"));
        static const char *const kErr[] = {"Storage", "error"};
        display.showPowerPrompt(kErr, 2, -1.0f, "Settings won't save.\nFix it with the full\nfirmware (Update).");
        delay(4000);
    }

    if (batOk && !Charger::usbPresent() && lastBatPct > 0.5f && lastBatPct <= Timing::BATTERY_SHUTDOWN_PCT) {
        lowBatteryShutdown();
    }

    // MENU held through boot opens the menu (a way in if FIRE is stuck)
    buttons.update();
    drainButtonEdges();
    lastActivity = millis();
    if (buttons.isPressed(Button::MENU)) {
        openMenu();
    } else {
        showMain();
    }
    Serial.println(F("Running."));
}

void loop() {
    const uint32_t now = millis();
    pollSerial();

    // Power button, every mode: hold to power off
    if (pwrHolding) {
        servicePowerHold(now);
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }
    if (powerButtonPressed(now)) {
        beginPowerHold(now);
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    buttons.update();

    if (menu.isActive()) {
        MenuExit e = menu.update(buttons);
        if (e != MenuExit::NONE) {
            handleMenuExit(e);
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    pollButtons(now);
    if (menu.isActive()) { // MENU just opened it
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }
    pollBattery(now);

    // Auto power-off after no button press for the shutdown timeout
    if (now - lastAutoOffCheck >= Timing::AUTO_SHUTOFF_CHECK_MS) {
        lastAutoOffCheck = now;
        if ((now - lastActivity) / 1000 > settings.autoShutdownTimeout) {
            Serial.println(F("Inactivity timeout - powering off"));
            doShutdown();
        }
    }
    // Laser off after no button press for the laser timeout
    if (laserOn && now - lastLaserCheck >= Timing::LASER_TIMEOUT_CHECK_MS) {
        lastLaserCheck = now;
        if ((now - lastActivity) / 1000 > settings.laserTimeout) {
            Serial.println(F("Laser timeout - off"));
            setLaserOn(false);
        }
    }

    if (ledOffAt != 0 && (int32_t)(millis() - ledOffAt) >= 0) {
        ledOffAt = 0;
        Led::off();
    }

    if (errorUntil != 0) {
        if ((int32_t)(millis() - errorUntil) < 0) {
            delay(Timing::LOOP_INTERVAL_MS);
            return; // error screen holds the panel
        }
        errorUntil = 0;
        screenDirty = true;
    }

    // Redraw on change (plug/unplug included), and at 4 Hz while the
    // battery icon animates
    const BatteryMode mode = batteryMode();
    if (millis() - lastDisplayRefresh >= Timing::DISPLAY_REFRESH_MS &&
        (screenDirty || mode != shownBatMode || mode == BatteryMode::CHARGING)) {
        showMain();
    }

    delay(Timing::LOOP_INTERVAL_MS);
}
