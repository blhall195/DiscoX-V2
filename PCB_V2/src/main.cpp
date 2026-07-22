#include "ble_manager.h"
#include "button_manager.h"
#include "calibration_mode.h"
#include "config.h"
#include "config_manager.h"
#include "defaults.h"
#include "device_context.h"
#include "disco_manager.h"
#include "display_manager.h"
#include "drivers/buzzer.h"
#include "drivers/max17048.h"
#include "drivers/power.h"
#include "drivers/sca3300.h"
#include "laser_manager.h"
#include "mag_cal/calibration.h"
#include "math_utils.h"
#include "menu_manager.h"
#include "rm3100.h"
#include "sensor_manager.h"
#include "snake_game.h"
#include "sounds.h"
#include "usb_drive.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <strings.h>

// ── Embedded calibration data ──────────────────────────────────────
// From V1's calibration_dict.json (github.com/blhall195/Mr_Zappy) —
// loaded at startup.
// Axes updated to the V2 mappings (2026-07-10, see config.h); the
// transform/centre/rbf data is still V1's and only roughly valid — expect
// MagErr until a full V2 on-device calibration replaces this fallback.
static const char CALIBRATION_JSON[] PROGMEM = R"({
  "mag": {
    "axes": "+Y-X+Z",
    "transform": [[0.0225258, -0.000348105, -0.000709316],
                   [0.000171691, 0.0221358, -0.000679377],
                   [0.000447533, -0.000145672, 0.0225338]],
    "centre": [-0.205928, -1.49322, 3.39001],
    "rbfs": [[[0.003298], [-0.000433647], [0.000818552], [-0.00208113], [0.00252497]],
             [[0.0], [0.0], [0.0], [0.0], [0.0]],
             [[0.00067137], [0.000540657], [0.000939099], [0.00137892], [0.00192191]]],
    "field_avg": 44.6264,
    "field_std": 0.310694
  },
  "dip_avg": 68.9221,
  "grav": {
    "axes": "+Y-X-Z",
    "transform": [[0.101936, -0.00167875, 0.000143624],
                   [0.0015862, 0.10164, 0.000604115],
                   [0.000153489, -0.000228592, 0.102199]],
    "centre": [0.235023, 0.0249322, 0.100026],
    "rbfs": [],
    "field_avg": 9.81022,
    "field_std": 0.00509215
  }
})";

// ── Global state ────────────────────────────────────────────────────
// (MAX17048_Persistent now lives in drivers/max17048.h — same subclass,
// shared with the bring-up tests.)
DeviceContext ctx;
ButtonManager buttons;
RM3100 mag;
// SCA3300 needs its own SPI master: SPIM3 is the core's default `SPI`,
// SPIM0/1 overlap the TWI/UART peripherals — SPIM2 is the free one.
static SPIClass accelSpi(NRF_SPIM2, PIN_SCA3300_MISO, PIN_SCA3300_SCK, PIN_SCA3300_MOSI);
SCA3300 accel;
MAX17048_Persistent battery;
DisplayManager display;
LaserManager laser;
Buzzer buzzer;
PowerControl power;
BleManager ble;
MagCal::Calibration calibration;
SensorManager sensorMgr;
ConfigManager configMgr;
DiscoManager disco;
MenuManager menuMgr;
CalibrationMode calMode;
SnakeGame snakeGame;

bool magOk = false;
bool accelOk = false;
bool batOk = false;
bool dispOk = false;
bool laserOk = false;
bool bleOk = false;
bool calOk = false;
bool calFromFlash = false; // true = loaded from flash, false = PROGMEM fallback
bool flashOk = false;
static bool enterMenuMode = false;
static bool enterCalibMode = false;
static bool enterSnakeMode = false;

// Latest sensor readings
static float lastMagX = 0, lastMagY = 0, lastMagZ = 0;
static float lastAccX = 0, lastAccY = 0, lastAccZ = 0;

// ── Raw-axis snapshot (commissioning: determine MAG_AXES/GRAV_AXES) ──
// Type 'r' in the serial monitor to print one line of the RM3100 and
// SCA3300 axes exactly as the chips report them (before Axes::fixAxes),
// so the sensor→device mapping can be read off from known poses.
// Type 's' to silence/resume the continuous >azimuth/>inclination teleplot
// stream so the snapshot lines are easy to copy.
static bool teleplotEnabled = true;

// ── Buzzer test console (browser GUI over Web Serial) ───────────────
// Lines starting with '>' are buffered until '\n' and dispatched as buzzer
// test commands (see handleBuzzerTestLine) — kept behind a distinct prefix
// byte so it can never collide with the single-char commissioning commands
// above, which stay immediate/unbuffered for raw terminal use.
static char buzzTestLine[64];
static uint8_t buzzTestLen = 0;
static bool buzzTestActive = false;

// ── Accel-derived motion detection (V2 has no gyro) ────────────────
// V1 gated the fusion alpha and the display freeze on gyro magnitude; the
// SCA3300 is accel-only, so motion is inferred from the deviation of the
// raw accel vector from its own slow EMA. Tune the threshold on hardware.
static float accEmaX = 0, accEmaY = 0, accEmaZ = 0;
static bool accEmaSeeded = false;
static bool deviceMoving = false;
static constexpr float ACCEL_MOTION_EMA_ALPHA = 0.2f;  // smoothing for the reference vector
static constexpr float ACCEL_MOTION_THRESHOLD = 0.35f; // m/s² deviation = "moving"

static float lastDistance = 0;

// ── Boot-time field strength sanity check ────────────────────────────
// After N sensor readings, compare calibrated field strength to expected.
// Catches stale calibration AND sensor remagnetization.
static constexpr uint8_t FIELD_CHECK_AFTER_SAMPLES = 20; // let EMA settle
static constexpr float FIELD_CHECK_TOLERANCE = 0.15f;    // 15%
static uint8_t fieldCheckCounter = 0;
static bool fieldCheckDone = false;

// ── Display deadband (prevents ±0.1 flicker when stationary) ────────
static float dispAz = 0.0f;                    // currently displayed azimuth
static float dispInc = 0.0f;                   // currently displayed inclination
static constexpr float DEADBAND_ANGLE = 0.10f; // degrees

// ── Timing statics ──────────────────────────────────────────────────
static uint32_t lastSensorUpdate = 0;
static uint32_t lastBatRead = 0;
static float lastBatPct = 0.0f;
static uint32_t lastDisplayRefresh = 0;
static uint32_t lastBleCheck = 0;
static uint32_t lastBleUartPoll = 0;
static uint32_t lastAutoShutCheck = 0;
static uint32_t lastLaserTimeCheck = 0;

// ── Button hold detection ───────────────────────────────────────────
static uint32_t discoHoldStart = 0;
static bool discoHolding = false;
static bool discoTriggered = false; // true once disco toggled during this hold

// ── Toast notifications ─────────────────────────────────────────────
static uint32_t splaysToastTime = 0;

// ── BLE state ───────────────────────────────────────────────────────
static bool lastBleConnected = false;

// ── Power button (LTC2954 INT) edge detection ───────────────────────
// INT pulses LOW (<1 s) per press — edge-triggered with a 20 ms confirm.
// Boot-hold guard: arm only after INT has first been seen HIGH, so the
// power-ON press can't immediately power the board back off.
static bool pwrBtnArmed = false;    // set once INT first reads HIGH
static uint32_t pwrBtnLowSince = 0; // millis() when INT first read LOW (0 = idle)

// ── Forward declarations — init ────────────────────────────────────
static void initPins();
static void scanI2C();
static void initDisplay();
static void initLaser();
static void initBle();
static void initFlash();
static void initCalibration();
static void initDisco();

// ── Forward declarations — polling ──────────────────────────────────
static void pollPowerButton(uint32_t now);
static void readSensorsUpdate(uint32_t now);
static void pollButtons(uint32_t now);
static void pollMeasurement(uint32_t now);
static void pollBLEPin(uint32_t now);
static void pollBLECommands(uint32_t now);
static void pollBattery(uint32_t now);
static void checkAutoShutoff(uint32_t now);
static void checkLaserTimeout(uint32_t now);
static void updateDisplay(uint32_t now);

// ── Forward declarations — helpers ──────────────────────────────────
static void handleBuzzerTestLine(char *line);
static void showSplaysDisabledToast();
static void handleMeasurementSuccess();
static void alertError(const char *errCode);
static void resetLaser();
static void doShutdown();
static void enterUsbDriveMode(); // modal USB MSC settings drive — reboots on exit
static void onFlushReading(float az, float inc, float dist);

const bool REGULAR_SHOT = false;
const bool QUICK_SHOT = true;

void laserOn() {
    if (!laserOk) {
        return;
    }
    laser.setLaser(true);
    ctx.laserEnabled = true;
}

void laserOff() {
    if (!laserOk) {
        return;
    }
    laser.setLaser(false);
    ctx.laserEnabled = false;
}

// ═══════════════════════════════════════════════════════════════════
// ── Setup ─────────────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════
void setup() {
    // Power controller first: KILL stays high-Z so nothing can cut the rail,
    // INT/PGOOD become inputs. (The power-on button press is still in flight —
    // the boot-hold guard in pollPowerButton keeps it from powering us off.)
    power.begin(PIN_KILL, PIN_PB_INT, PIN_PGOOD);
    buzzer.begin(PIN_BUZZER_A, PIN_BUZZER_B);
    Sounds::begin(buzzer);

    // Init laser so it can be turned off immediately (rail is ENA-gated and
    // already up; the LDJ-100 boots with the diode off, so this is belt+braces).
    initLaser();

    // Get display up ASAP — before serial, which can block on USB enumeration.
    // V2 I2C pins are NOT the variant defaults — must be routed before begin().
    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);
    initDisplay();

    // Configure button pull-ups BEFORE reading them
    pinMode(PIN_BTN_FIRE, INPUT_PULLUP);
    pinMode(PIN_BTN_MENU, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    delayMicroseconds(50); // let pull-ups settle before reading

    // ── Early USB drive mode check ─────────────────────────────────
    // Must happen BEFORE any Serial use so the MSC interface is part of
    // the initial USB enumeration. Entered by EITHER the usb_drive flag
    // (set from the menu) OR by holding DOWN (B3) at power-on. The button
    // path needs no flash read/write, so it still works when LittleFS is
    // broken — it's the recovery route to reformat the partition from a PC.
    // (FIRE-at-boot is taken by the serial-debug hold below, hence DOWN.)
    {
        flashOk = configMgr.begin(); // idempotent — initFlash() re-checks later
        bool usbByFlag = flashOk && configMgr.hasFlag(Flags::USB_DRIVE);
        bool usbByButton = (digitalRead(PIN_BTN_DOWN) == LOW);
        if (usbByFlag || usbByButton) {
            if (usbByFlag) {
                configMgr.clearFlag(Flags::USB_DRIVE);
            }
            UsbDrive::beginMsc(); // register MSC BEFORE the host enumerates
            enterUsbDriveMode();  // modal — reboots or powers off, never returns
        }
    }

    // Normal boot — show splash screen
    display.showSplash(false);

    Serial.begin(115200);
    if (digitalRead(PIN_BTN_FIRE) == LOW) { // hold FIRE for serial debug
        while (!Serial && millis() < 3000) {
        }
        Serial.println(F("I2C bus scan:"));
        scanI2C();
        Serial.println();
    }

    Serial.println(F("=== Mr_Zappy PCB V2 ==="));
    Serial.println(F("Merged single-MCU firmware"));
    Serial.println();

    initPins();
    buttons.begin();

    // ── Sensors ──
    magOk = mag.begin(Wire, RM3100_I2C_ADDR, RM3100_CYCLE_COUNT, PIN_MAG_DRDY);
    if (magOk) {
        mag.startSingleReading(); // kick off first non-blocking read
    }
    Serial.print(F("RM3100:      "));
    Serial.println(magOk ? F("OK") : F("FAILED"));

    // MODE_1 (±3 g, 70 Hz LPF): closest dynamics to V1's 4G/104Hz setup, and
    // ±1.5 g modes would clip during disco shake detection (11 m/s² threshold)
    accelOk = accel.begin(accelSpi, PIN_SCA3300_CS, SCA3300::Mode::MODE_1);
    Serial.print(F("SCA3300:     "));
    Serial.println(accelOk ? F("OK") : F("FAILED"));

    batOk = battery.begin(&Wire);
    if (batOk) {
        delay(100); // let SOC register update after wake
        lastBatPct = battery.cellPercent();
        lastBatRead = millis();
    }
    Serial.print(F("MAX17048:    "));
    Serial.println(batOk ? F("OK") : F("FAILED"));
    if (batOk) {
        Serial.print(F("  Voltage:   "));
        Serial.print(battery.cellVoltage(), 3);
        Serial.println(F(" V"));
        Serial.print(F("  SOC:       "));
        Serial.print(lastBatPct, 1);
        Serial.println(F(" %"));
        Serial.print(F("  Hibernate: "));
        Serial.println(battery.isHibernating() ? F("yes") : F("no"));
    }
    Serial.println();

    // Flash/config BEFORE BLE — the advertised name comes from config.json
    initFlash();
    initBle();

    // Warn user if flash was auto-reformatted (all saved data lost)
    if (flashOk && configMgr.wasReformatted() && dispOk) {
        auto &disp = display.getDisplay();
        display.blankScreen();
        disp.setTextColor(SH110X_WHITE);
        disp.setTextSize(2);
        disp.setCursor(0, 10);
        disp.println(F("FLASH"));
        disp.println(F("RECOVERED"));
        disp.setTextSize(1);
        disp.println();
        disp.println(F("Storage was corrupt."));
        disp.println(F("Reformatted OK."));
        disp.println();
        disp.println(F("Calibration &"));
        disp.println(F("settings were lost."));
        disp.display();
        delay(5000);
    }

    initCalibration();
    initDisco();

    // ── Splash sequence: off(200) → on(300) → off(200) → on(750) = 1450ms
    // Timed from its own start, NOT from the early static splash: init work
    // and any boot warning screens (NOT CALIBRATED / FLASH RECOVERED /
    // STORAGE DEGRADED, up to several seconds) used to consume the shared
    // window, silently skipping the whole animation — and the BLE name,
    // which is only drawn here.
    {
        uint32_t animStart = millis();
        auto splashLaser = [&]() -> bool {
            uint32_t t = millis() - animStart;
            if (t < 200) {
                return false; // off  200ms
            }
            if (t < 500) {
                return true; // on   300ms
            }
            if (t < 700) {
                return false; // off  200ms
            }
            return true; // on   750ms (final hold)
        };
        // Extract suffix after '_' from BLE name for splash display
        const char *nameSuffix = strchr(ctx.config.bleName, '_');
        if (nameSuffix) {
            nameSuffix++; // skip the '_'
        }

        while (millis() - animStart < 1450) {
            display.showSplash(splashLaser(), nameSuffix);
        }
    }

    // ── Switch display from splash to main screen ────────────────
    if (dispOk) {
        display.updateBattery(lastBatPct);
        display.updateMeasureFrom(ctx.config.measureFromFront);
        display.initScreen();
    }

    // ── Low battery check on boot ────────────────────────────────
    // Skip if 0% — that means no LiPo connected (USB-only power)
    if (batOk && lastBatPct > 0.5f && lastBatPct <= Timing::BATTERY_SHUTDOWN_PCT) {
        Serial.println(F("LOW BATTERY — shutting down"));
        if (dispOk) {
            display.updateDistanceText("LOW");
            display.updateAzimuth(0);
            display.updateInclination(0);
            display.refresh();
        }
        disco.setRed();
        delay(3000);
        doShutdown();
    }

    // ── Pending readings display ─────────────────────────────────
    if (dispOk) {
        display.updateBTNumber(ctx.bleDisconnectionCounter);
    }

    // ── Check for boot mode overrides ────────────────────────────
    buttons.update();
    if (buttons.isPressed(Button::MENU)) {
        Serial.println(F("MENU held at boot — entering menu mode"));
        enterMenuMode = true;
    }

    // Enter menu mode if flag was set or MENU held
    if (enterMenuMode && dispOk) {
        display.showStartingMenu();
        delay(500);
        menuMgr.begin(display.getDisplay(), ctx, configMgr);
    }

    // Enter calibration mode if flag was set
    if (enterCalibMode && magOk && accelOk && dispOk && laserOk) {
        calMode.begin(buttons, display, disco, laser, mag, accel, configMgr, calibration, ctx.config);
    }

    // Enter snake mode if flag was set
    if (enterSnakeMode && dispOk) {
        if (laserOk) {
            laser.setLaser(false);
        }
        snakeGame.begin(display.getDisplay(), buttons, disco);
    }

    ctx.lastActivityTime = millis();

    if (snakeGame.isActive()) {
        Serial.println(F("Running. Snake mode."));
    } else if (calMode.isActive()) {
        Serial.println(F("Running. Calibration mode."));
    } else if (menuMgr.isActive()) {
        Serial.println(F("Running. Menu mode."));
    } else {
        Serial.println(F("Running. Normal mode."));
    }
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// ── Main Loop ─────────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    // Hardware power button (LTC2954 INT) — works in all modes
    // (normal, menu, cal, snake). Confirmed press → clean shutdown.
    pollPowerButton(now);

    buttons.update();

    // ── Snake mode: SnakeGame owns the loop ──
    if (snakeGame.isActive()) {
        if (snakeGame.update()) {
            // Game finished — return to normal operation
            Serial.println(F("Snake game ended — returning to normal mode"));
            disco.turnOff();
            display.initScreen();
            ctx.laserEnabled = false;
            ctx.lastActivityTime = millis();
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // ── Menu mode: hand off to MenuManager, skip everything else ──
    if (menuMgr.isActive()) {
        menuMgr.update(buttons);
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // ── Handle menu exit transitions (one-shot) ──
    if (menuMgr.exitAction() == MenuExitAction::ENTER_PART1_CALIB ||
        menuMgr.exitAction() == MenuExitAction::ENTER_PART2_CALIB) {
        CalMode cm = CalMode::PART1_ELLIPSOID;
        if (menuMgr.exitAction() == MenuExitAction::ENTER_PART2_CALIB) {
            cm = CalMode::PART2_ALIGNMENT;
        }
        menuMgr.clearExitAction();
        if (magOk && accelOk && dispOk && laserOk) {
            Serial.println(F("Transitioning: menu → calibration"));
            calMode.begin(buttons, display, disco, laser, mag, accel, configMgr, calibration, ctx.config, cm);
        } else {
            Serial.println(F("Cannot enter calibration — sensors not ready"));
            display.initScreen();
            ctx.lastActivityTime = millis();
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // Mag Field Check (Foreshot/Backshot)
    if (menuMgr.exitAction() == MenuExitAction::ENTER_FB_CHECK) {
        menuMgr.clearExitAction();
        if (magOk && accelOk && dispOk && laserOk && calOk) {
            Serial.println(F("Transitioning: menu → F/B field check"));
            calMode.beginFBCheck(buttons, display, disco, laser, mag, accel, configMgr, calibration,
                                 ctx.config);
        } else {
            Serial.println(F("Cannot enter F/B check — sensors/calibration not ready"));
            display.initScreen();
            ctx.lastActivityTime = millis();
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // Snake
    if (menuMgr.exitAction() == MenuExitAction::ENTER_SNAKE) {
        menuMgr.clearExitAction();
        if (dispOk) {
            Serial.println(F("Transitioning: menu → snake game"));
            laserOff();
            snakeGame.begin(display.getDisplay(), buttons, disco);
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // Update Firmware
    if (menuMgr.exitAction() == MenuExitAction::ENTER_BOOTLOADER) {
        menuMgr.clearExitAction();

        // The bootloader gives USB only 3 s to enumerate after enterUf2Dfu()
        // before rebooting back into the app, so DFU only sticks if the PC
        // link is already up — wait for USB before resetting (MENU cancels).
        // The bootloader's no-timeout DFU branch is NOT reachable from
        // software: its double-tap magic is gated on a reset-PIN reset,
        // which NVIC_SystemReset() is not (see CLAUDE.md gotcha).
        if (!TinyUSBDevice.mounted()) {
            Serial.println(F("Waiting for USB before bootloader entry..."));
            if (dispOk) {
                auto &d = display.getDisplay();
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
            while (!TinyUSBDevice.mounted()) {
                buttons.update();
                if (buttons.wasPressed(Button::MENU)) {
                    Serial.println(F("Bootloader entry cancelled"));
                    return;
                }
                delay(10);
            }
        }

        Serial.println(F("Entering UF2 bootloader..."));
        if (dispOk) {
            auto &d = display.getDisplay();
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

        // Adafruit nRF52 core: reboot into the UF2 bootloader (3 s USB window)
        enterUf2Dfu();
        // Does not return
    }

    // USB drive mode (edit settings/calibration on a PC)
    if (menuMgr.exitAction() == MenuExitAction::ENTER_USB_DRIVE) {
        menuMgr.clearExitAction();
        Serial.println(F("Rebooting into USB drive mode..."));
        bool flagged = flashOk && configMgr.writeFlag(Flags::USB_DRIVE);
        if (dispOk) {
            auto &d = display.getDisplay();
            d.clearDisplay();
            d.setTextSize(1);
            d.setTextColor(SH110X_WHITE);
            d.setCursor(0, 20);
            if (flagged) {
                d.println(F("Restarting into"));
                d.println(F("USB drive mode..."));
            } else {
                // Flag write failed (storage broken) — tell the user the
                // button route, which needs no flash writes at all.
                d.println(F("Storage error."));
                d.println();
                d.println(F("Hold DOWN (B3) while"));
                d.println(F("powering on for USB"));
                d.println(F("drive mode."));
            }
            d.display();
        }
        delay(flagged ? 1000 : 4000);
        if (flagged) {
            NVIC_SystemReset();
            // Does not return
        }
        display.initScreen();
        ctx.lastActivityTime = millis();
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // Reformat internal storage (recovery)
    if (menuMgr.exitAction() == MenuExitAction::REFORMAT_FLASH) {
        menuMgr.clearExitAction();
        Serial.println(F("Reformatting internal storage..."));
        if (dispOk) {
            auto &d = display.getDisplay();
            d.clearDisplay();
            d.setTextSize(1);
            d.setTextColor(SH110X_WHITE);
            d.setCursor(0, 40);
            d.println(F("Erasing storage..."));
            d.println();
            d.println(F("Device will restart."));
            d.display();
        }
        configMgr.reformat(); // formats LittleFS in place
        delay(1500);
        NVIC_SystemReset();
        // Does not return
    }
    if (menuMgr.exitAction() == MenuExitAction::RETURN_NORMAL) {
        menuMgr.clearExitAction();
        Serial.println(F("Returning to normal mode"));
        display.updateMeasureFrom(ctx.config.measureFromFront);
        display.initScreen();
        ctx.lastActivityTime = millis();
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    //  Calibration mode
    if (calMode.isActive()) {
        bool done = calMode.update();
        if (done) {
            Serial.println(F("Calibration mode finished."));
            // Drain buttons held during save/discard (UP_DISCO or FIRE+UP_DISCO)
            // to prevent pollButtons() treating the release as a disco toggle.
            while (buttons.isPressed(Button::UP_DISCO) || buttons.isPressed(Button::FIRE)) {
                buttons.update();
                delay(Timing::LOOP_INTERVAL_MS);
            }
            // Consume any residual edge flags so the next loop doesn't
            // see a stale wasPressed and trigger an unwanted measurement.
            buttons.update();
            buttons.wasPressed(Button::FIRE);
            buttons.wasPressed(Button::UP_DISCO);
            buttons.wasPressed(Button::DOWN);
            buttons.wasPressed(Button::MENU);
            calOk = calibration.isCalibrated();
            if (calOk) {
                sensorMgr.init(&calibration, ctx.config.emaAlphaStable, ctx.config.emaAlphaMoving,
                               ctx.config.stabilityBufferLength, Defaults::emaJumpThreshold);
            }
            bleRadioQuiet(false); // resume advertising silenced by calMode.begin()
            display.initScreen();
            laserOn();
            ctx.lastActivityTime = millis();
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // ── Normal operation: cooperative polling ─────────────────────
    readSensorsUpdate(now);
    pollButtons(now);
    pollMeasurement(now);
    pollBLEPin(now);
    pollBLECommands(now);
    pollBattery(now);
    checkAutoShutoff(now);
    checkLaserTimeout(now);
    updateDisplay(now);
    disco.update(lastAccX, lastAccY, lastAccZ);

    // ── Deferred flash write: sync RAM-buffered readings when idle ──
    if (ctx.currentState == SystemState::IDLE && flashOk && configMgr.hasPendingToSync()) {
        configMgr.syncPendingToFlash();
    }

    delay(Timing::LOOP_INTERVAL_MS);
}

// ═══════════════════════════════════════════════════════════════════
// ── Polling Functions ─────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

// ── Read sensors + update fusion at 50 Hz ───────────────────────
static void readSensorsUpdate(uint32_t now) {
    if (now - lastSensorUpdate < Timing::SENSOR_POLL_MS) {
        return;
    }

    lastSensorUpdate = now;

    if (magOk && mag.measurementComplete()) {
        RM3100::Reading mr = mag.getLastReading();
        mag.toMicroTesla(mr, lastMagX, lastMagY, lastMagZ);
        mag.startSingleReading(); // immediately kick off next measurement
    }

    if (accelOk) {
        float gx, gy, gz;
        if (accel.readAcceleration(gx, gy, gz)) {
            // SCA3300 reports g — convert to m/s² (V1 pipeline units)
            lastAccX = gx * GRAVITY_MS2;
            lastAccY = gy * GRAVITY_MS2;
            lastAccZ = gz * GRAVITY_MS2;

            // Motion detection without a gyro: deviation of the raw accel
            // vector from its slow EMA. Seeded on the first sample.
            if (!accEmaSeeded) {
                accEmaX = lastAccX;
                accEmaY = lastAccY;
                accEmaZ = lastAccZ;
                accEmaSeeded = true;
            }
            float dx = lastAccX - accEmaX;
            float dy = lastAccY - accEmaY;
            float dz = lastAccZ - accEmaZ;
            deviceMoving = sqrtf(dx * dx + dy * dy + dz * dz) > ACCEL_MOTION_THRESHOLD;
            accEmaX += ACCEL_MOTION_EMA_ALPHA * dx;
            accEmaY += ACCEL_MOTION_EMA_ALPHA * dy;
            accEmaZ += ACCEL_MOTION_EMA_ALPHA * dz;
        }
    }

    // Raw-axis snapshot for axis-mapping commissioning (press 'r')
    while (Serial.available()) {
        int c = Serial.read();

        if (buzzTestActive) {
            if (c == '\n' || c == '\r') {
                if (buzzTestLen > 0) {
                    buzzTestLine[buzzTestLen] = '\0';
                    handleBuzzerTestLine(buzzTestLine);
                    buzzTestLen = 0;
                }
                buzzTestActive = false;
            } else if (buzzTestLen < sizeof(buzzTestLine) - 1) {
                buzzTestLine[buzzTestLen++] = (char)c;
            }
            continue;
        }

        if (c == '>') {
            // Start of a buffered buzzer-test command line — see
            // handleBuzzerTestLine for the protocol.
            buzzTestActive = true;
            buzzTestLen = 0;
        } else if (c == 's' || c == 'S') {
            teleplotEnabled = !teleplotEnabled;
            Serial.println(teleplotEnabled ? F("Teleplot stream ON") : F("Teleplot stream OFF"));
        } else if (c == 'f' || c == 'F') {
            // Reset filter tuning to firmware defaults — a stored config.json
            // otherwise shadows new defaults forever (loadConfig prefers it).
            ctx.config.emaAlphaStable = Defaults::emaAlphaStable;
            ctx.config.emaAlphaMoving = Defaults::emaAlphaMoving;
            ctx.config.stabilityBufferLength = Defaults::stabilityBufferLength;
            bool saved = flashOk && configMgr.saveConfig(ctx.config);
            if (calOk) {
                sensorMgr.init(&calibration, ctx.config.emaAlphaStable, ctx.config.emaAlphaMoving,
                               ctx.config.stabilityBufferLength, Defaults::emaJumpThreshold);
            }
            Serial.print(F("Filter reset: EMA stable="));
            Serial.print(ctx.config.emaAlphaStable, 2);
            Serial.print(F(", moving="));
            Serial.print(ctx.config.emaAlphaMoving, 2);
            Serial.print(F(", stability buf="));
            Serial.print(ctx.config.stabilityBufferLength);
            Serial.println(saved ? F(" (saved)") : F(" (NOT saved — storage error)"));
        } else if (c == 'c' || c == 'C') {
            Serial.println(F("--- /config.json ---"));
            if (!configMgr.printConfig(Serial)) {
                Serial.println(F("(no config file — running on defaults)"));
            }
        } else if (c == 'u' || c == 'U') {
            // Clean bootloader entry for host-driven reflash. The 1200bps
            // touch resets at an arbitrary instant and can tear an in-flight
            // LittleFS commit (root cause of the 2026-07-19 storage
            // corruption) — this path syncs storage first, then enters DFU
            // with USB already up so the bootloader's 3 s window is safe.
            // The upload skill sends 'u' before running pio upload.
            Serial.println(F("Reflash: syncing storage, entering bootloader..."));
            if (flashOk && configMgr.hasPendingToSync()) {
                configMgr.syncPendingToFlash();
            }
            Serial.flush();
            delay(100);
            enterUf2Dfu();
            // Does not return
        } else if (c == 'r' || c == 'R') {
            Serial.print(F("RAW mag uT  X="));
            Serial.print(lastMagX, 2);
            Serial.print(F(" Y="));
            Serial.print(lastMagY, 2);
            Serial.print(F(" Z="));
            Serial.print(lastMagZ, 2);
            Serial.print(F("  |  acc m/s2  X="));
            Serial.print(lastAccX, 2);
            Serial.print(F(" Y="));
            Serial.print(lastAccY, 2);
            Serial.print(F(" Z="));
            Serial.println(lastAccZ, 2);
        }
    }

    if (calOk && magOk && accelOk) {
        Eigen::Vector3f rawMag(lastMagX, lastMagY, lastMagZ);
        Eigen::Vector3f rawGrav(lastAccX, lastAccY, lastAccZ);
        sensorMgr.update(rawMag, rawGrav, deviceMoving);

        // Boot-time sanity check: after EMA settles, verify field strength
        if (!fieldCheckDone && ++fieldCheckCounter >= FIELD_CHECK_AFTER_SAMPLES) {
            fieldCheckDone = true;
            float expectedMag = calibration.mag().fieldAvg();
            if (expectedMag > 0.0f) {
                float actualMag = calibration.mag().getFieldStrength(rawMag);
                float deviation = fabsf(actualMag - expectedMag) / expectedMag;
                Serial.print(F("Field check: expected="));
                Serial.print(expectedMag, 2);
                Serial.print(F(" actual="));
                Serial.print(actualMag, 2);
                Serial.print(F(" dev="));
                Serial.print(deviation * 100.0f, 1);
                Serial.println(F("%"));
                if (deviation > FIELD_CHECK_TOLERANCE) {
                    Serial.println(F("WARNING: field strength deviation >15% — "
                                     "calibration may be invalid"));
                    if (dispOk) {
                        display.updateDistanceText("CAL?");
                        display.refresh();
                        disco.setRed();
                        delay(3000);
                        disco.turnOff();
                    }
                }
            }
        }
    }
}

// ── Buzzer test console ───────────────────────────────────────────
// Protocol for the browser buzzer-tester GUI (Web Serial, 115200 8N1).
// Lines are ASCII, whitespace-separated, dispatched from
// handleBuzzerTestLine's caller once a trailing '\n' completes the line
// buffered after a leading '>':
//   >TONE <freqHz> <durationMs>
//   >SWEEP <fromHz> <toHz> <durationMs>
//   >MELODY <freq:ms>,<freq:ms>,...     (freq 0 = rest/silence)
//   >SOUND <name>                       click|shot|reading|leg|warning|
//                                        error|snakestart|snakeeat|snakecrash
//   >STOP                               silence immediately (best-effort —
//                                        the driver is a blocking bit-bang,
//                                        so this only takes effect between
//                                        notes, not mid-tone)
// Every command prints "OK" or "ERR <reason>" when it returns so the GUI
// can serialize sends and show status.
static void handleBuzzerTestLine(char *line) {
    char *save = nullptr;
    char *cmd = strtok_r(line, " ", &save);
    if (!cmd) {
        return;
    }

    if (strcasecmp(cmd, "TONE") == 0) {
        char *freqTok = strtok_r(nullptr, " ", &save);
        char *msTok = strtok_r(nullptr, " ", &save);
        if (!freqTok || !msTok) {
            Serial.println(F("ERR usage: TONE <freqHz> <durationMs>"));
            return;
        }
        buzzer.tone((uint32_t)atol(freqTok), (uint32_t)atol(msTok));
        Serial.println(F("OK"));
    } else if (strcasecmp(cmd, "SWEEP") == 0) {
        char *fromTok = strtok_r(nullptr, " ", &save);
        char *toTok = strtok_r(nullptr, " ", &save);
        char *msTok = strtok_r(nullptr, " ", &save);
        if (!fromTok || !toTok || !msTok) {
            Serial.println(F("ERR usage: SWEEP <fromHz> <toHz> <durationMs>"));
            return;
        }
        buzzer.sweep((uint32_t)atol(fromTok), (uint32_t)atol(toTok), (uint32_t)atol(msTok));
        Serial.println(F("OK"));
    } else if (strcasecmp(cmd, "MELODY") == 0) {
        char *notes = strtok_r(nullptr, " ", &save);
        if (!notes) {
            Serial.println(F("ERR usage: MELODY <freq:ms>,<freq:ms>,..."));
            return;
        }
        char *noteSave = nullptr;
        char *note = strtok_r(notes, ",", &noteSave);
        while (note) {
            char *colon = strchr(note, ':');
            if (colon) {
                *colon = '\0';
                uint32_t freq = (uint32_t)atol(note);
                uint32_t ms = (uint32_t)atol(colon + 1);
                if (freq == 0) {
                    delay(ms);
                } else {
                    buzzer.tone(freq, ms);
                }
            }
            note = strtok_r(nullptr, ",", &noteSave);
        }
        Serial.println(F("OK"));
    } else if (strcasecmp(cmd, "SOUND") == 0) {
        char *name = strtok_r(nullptr, " ", &save);
        if (!name) {
            Serial.println(F("ERR usage: SOUND <name>"));
            return;
        }
        if (strcasecmp(name, "click") == 0) {
            Sounds::click();
        } else if (strcasecmp(name, "shot") == 0) {
            Sounds::shotStart();
        } else if (strcasecmp(name, "reading") == 0) {
            Sounds::readingOk();
        } else if (strcasecmp(name, "leg") == 0) {
            Sounds::legComplete();
        } else if (strcasecmp(name, "warning") == 0) {
            Sounds::warning();
        } else if (strcasecmp(name, "error") == 0) {
            Sounds::error();
        } else if (strcasecmp(name, "snakestart") == 0) {
            Sounds::snakeStart();
        } else if (strcasecmp(name, "snakeeat") == 0) {
            Sounds::snakeEat(0);
        } else if (strcasecmp(name, "snakecrash") == 0) {
            Sounds::snakeCrash(0);
        } else {
            Serial.println(F("ERR unknown sound name"));
            return;
        }
        Serial.println(F("OK"));
    } else if (strcasecmp(cmd, "STOP") == 0) {
        buzzer.off();
        Serial.println(F("OK"));
    } else {
        Serial.println(F("ERR unknown command"));
    }
}

static void showSplaysDisabledToast() {
    if (dispOk) {
        auto &d = display.getDisplay();
        d.clearDisplay();
        d.setTextColor(SH110X_WHITE);
        d.setTextSize(2);
        d.setCursor(0, 44);
        d.println(F("Splays not"));
        d.println(F("enabled"));
        d.display();
    }
    splaysToastTime = millis();
}

static void startDisco() {
    disco.turnOn();
    ctx.discoOn = true;
    laser.setLaser(false);
    ctx.laserEnabled = false;
    Serial.println(F("DISCO: on"));
}

static void stopDisco() {
    disco.turnOff();
    ctx.discoOn = false;
    laser.setLaser(false);
    ctx.laserEnabled = false;
    Serial.println(F("DISCO: off"));
}

static bool heldLongEnoughForDisco(uint32_t now, uint32_t start) {
    return now - start >= Timing::DISCO_HOLD_MS;
}

static void prepareForShot() {
    // Setup laser
    laserOn();
    lastDistance = 0;

    ctx.displayFrozen = false;
    disco.turnOff();
    Serial.println(F("MEASURE: getting ready for a shot"));
}

// Call after prepare for shot
static void startShot(bool isQuickShot) {
    ctx.quickShot = isQuickShot;
    ctx.displayFrozen = false;
    ctx.currentState = SystemState::TAKING_MEASUREMENT;
    ctx.measurementTaken = false;
    lastDistance = 0;
    Serial.println(F("MEASURE: taking measurement"));

    if (!ctx.purpleLatched) {
        disco.setRed();
    }

    Sounds::shotStart();
}

// ── Button event handling ───────────────────────────────────────
static void pollButtons(uint32_t now) {
    // Button 1 (FIRE — the trigger)
    // Take measurement or wake laser
    if (buttons.wasPressed(Button::FIRE)) {
        ctx.lastActivityTime = now;
        ctx.purpleLatched = false;

        if (ctx.laserEnabled) {
            startShot(REGULAR_SHOT);
        } else {
            prepareForShot();
            delay(20);
        }
        return;
    }

    // Button 2 (UP_DISCO)
    // Long press to toggle disco, short press for splay/quick shot
    if (buttons.isPressed(Button::UP_DISCO)) {
        ctx.lastActivityTime = now;
        if (!discoHolding) {
            discoHolding = true;
            discoTriggered = false;
            discoHoldStart = now;
        } else if (!discoTriggered) {
            if (heldLongEnoughForDisco(now, discoHoldStart)) {
                discoTriggered = true;
                ctx.discoOn ? stopDisco() : startDisco();
            }
        }
        return;
    } else { // Short press -> splay shot
        if (discoHolding) {
            discoHolding = false;

            if (discoTriggered) {
                discoTriggered = false;
            } else {
                if (ctx.config.splaysEnabled) {
                    ctx.laserEnabled ? startShot(QUICK_SHOT) : prepareForShot();
                } else {
                    showSplaysDisabledToast();
                    Sounds::warning();
                }
            }
            return;
        }
    }

    // Button 4 (MENU)
    if (buttons.wasPressed(Button::MENU) && ctx.currentState == SystemState::IDLE) {
        ctx.lastActivityTime = now;
        Serial.println(F("MENU pressed — entering menu mode"));
        laser.setLaser(false);
        ctx.laserEnabled = false;
        disco.turnOff();
        ctx.discoOn = false;
        menuMgr.begin(display.getDisplay(), ctx, configMgr);
        return;
    }

    // Button 3 (DOWN) is unused in normal mode.
    // Power off is the hardware power button — see pollPowerButton().
}

// ── Measurement workflow ────────────────────────────────────────
static void pollMeasurement(uint32_t now) {
    if (ctx.currentState != SystemState::TAKING_MEASUREMENT) {
        return;
    }

    if (ctx.measurementTaken) {
        return;
    }

    if (!calOk || !magOk || !accelOk || !laserOk) {
        Serial.println(F("MEAS: sensors not ready"));
        ctx.quickShot = false;
        ctx.currentState = SystemState::IDLE;
        return;
    }

    // const ILegChecker &stabChecker = ctx.quickShot ?
    // ctx.quickShotStabilityChecker : ctx.stabilityChecker;
    const ILegChecker &stabChecker = ctx.stabilityChecker;

    // No stability checking for quick shots. (In Beta)
    if (!ctx.quickShot && !sensorMgr.isStable(stabChecker)) {
        static uint32_t lastStabDbg = 0;
        uint32_t n = millis();
        if (n - lastStabDbg > 1000) {
            lastStabDbg = n;
            Serial.print(F("MEAS: waiting stable  AZ="));
            Serial.print(sensorMgr.getAzimuth(), 1);
            Serial.print(F(" INC="));
            Serial.println(sensorMgr.getInclination(), 1);
        }
        return;
    }

    Serial.println(F("MEAS: taking measurement"));

    // ── Stable — take measurement ────────────────────────────
    ctx.readings.azimuth = sensorMgr.getAzimuth();
    ctx.readings.inclination = sensorMgr.getInclination();
    ctx.readings.roll = sensorMgr.getRoll();

    // Flush any stale UART data before talking to laser
    while (Serial1.available()) {
        Serial1.read();
    }
    delay(30); // let the UART line settle

    // Take laser distance
    Serial.println(F("MEAS: sending measure cmd..."));
    int32_t distMm = 0;
    LaserError lErr = laser.measure(distMm);
    Serial.print(F("MEAS: result="));
    Serial.print(LaserManager::errorString(lErr));
    Serial.print(F(" mm="));
    Serial.println(distMm);

    if (lErr != LaserError::OK) {
        Serial.print(F("MEAS: laser error: "));
        Serial.println(LaserManager::errorString(lErr));
        resetLaser();
        alertError("LzrERR");
        ctx.quickShot = false;
        ctx.currentState = SystemState::IDLE;
        return;
    }

    ctx.readings.distance =
        (distMm / 1000.0f) +
        (ctx.config.measureFromFront ? -Defaults::laserFrontOffset : ctx.config.laserDistanceOffset);

    lastDistance = ctx.readings.distance;
    Serial.print(F("MEAS: dist="));
    Serial.println(ctx.readings.distance, 3);

    // Anomaly detection
    Serial.println(F("MEAS: checking anomaly..."));
    if (ctx.config.anomalyDetection) {
        Eigen::Vector3f rawMag(lastMagX, lastMagY, lastMagZ);
        Eigen::Vector3f rawGrav(lastAccX, lastAccY, lastAccZ);
        MagCal::Strictness strict = {ctx.config.magTolerance, ctx.config.gravTolerance,
                                     ctx.config.dipTolerance};
        MagCal::AnomalyType anom = calibration.checkAnomaly(rawMag, rawGrav, strict);
        if (anom != MagCal::AnomalyType::NONE) {
            const char *errStr = "Err";
            switch (anom) {
            case MagCal::AnomalyType::MAGNETIC:
                errStr = "MagErr";
                break;
            case MagCal::AnomalyType::GRAVITY:
                errStr = "GravErr";
                break;
            case MagCal::AnomalyType::DIP:
                errStr = "DipErr";
                break;
            default:
                break;
            }
            Serial.print(F("MEAS: anomaly: "));
            Serial.println(errStr);
            alertError(errStr);
            // Still update display with the readings
            dispAz = ctx.readings.azimuth;
            dispInc = ctx.readings.inclination;
            if (dispOk) {
                display.updateSensorReadings(ctx.readings.distance, ctx.readings.azimuth,
                                             ctx.readings.inclination);
                display.refresh();
            }
            ctx.quickShot = false;
            ctx.currentState = SystemState::IDLE;
            return;
        }
    }

    // Success! Show distance immediately before blocking buzzer/wibble sequence
    if (dispOk) {
        display.updateSensorReadings(ctx.readings.distance, ctx.readings.azimuth, ctx.readings.inclination);
        display.refresh();
    }

    Serial.println(F("MEAS: calling handleSuccess..."));
    handleMeasurementSuccess();
    Serial.println(F("MEAS: handleSuccess done"));

    // Freeze display — stays showing shot readings until next button press
    dispAz = ctx.readings.azimuth;
    dispInc = ctx.readings.inclination;
    ctx.displayFrozen = true;

    Serial.print(F("MEAS OK: AZ="));
    Serial.print(ctx.readings.azimuth, 1);
    Serial.print(F(" INC="));
    Serial.print(ctx.readings.inclination, 1);
    Serial.print(F(" DIST="));
    Serial.println(ctx.readings.distance, 2);

    ctx.currentState = SystemState::IDLE;

    if (!ctx.purpleLatched) {
        disco.turnOff();
    }

    // If doing quick shots, prepare again immediately.
    if (ctx.quickShot) {
        prepareForShot();
    } else {
        laserOff();
    }
    ctx.quickShot = false;
}

// ── Handle successful measurement ───────────────────────────────
static void handleMeasurementSuccess() {
    ctx.lastMeasurementTime = millis();

    // Green disco to indicate successful reading.
    // Beep later after we know if there was a successful leg.
    disco.setGreen();

    Serial.print(F("  HS:2 bleConn="));
    Serial.print(ctx.bleConnected);
    Serial.print(F(" bleOk="));
    Serial.println(bleOk);
    Serial.flush();
    delay(50);

    // Send via BLE or queue
    if (ctx.bleConnected && bleOk) {
        Serial.println(F("  HS:2a ble send"));
        Serial.flush();
        delay(50);
        ble.sendSurveyData(ctx.readings.azimuth, ctx.readings.inclination, ctx.readings.distance);
        ctx.bleDisconnectionCounter = 0;
    } else {
        ctx.bleDisconnectionCounter++;
        if (dispOk) {
            display.updateBTNumber(ctx.bleDisconnectionCounter);
        }
        // Buffer reading in RAM — synced to flash during IDLE or shutdown
        configMgr.appendPendingReading(ctx.readings.azimuth, ctx.readings.inclination, ctx.readings.distance);
    }

    // ── Leg consistency buffer (skip for quick shots) ─────────
    if (ctx.quickShot) {
        Serial.println(F("  HS:3 quick shot — skipping leg buf"));
        ctx.measurementTaken = false;
        Sounds::readingOk();
        return;
    }

    Serial.println(F("  HS:3 leg buf"));
    Serial.flush();
    ctx.shotBuf.push(Shot(ctx.readings.azimuth, ctx.readings.inclination, ctx.readings.distance));

    Serial.println(F("  HS:4 leg check"));
    Serial.flush();

    if (ctx.shotBuf.hasValidLeg()) {
        // Rising fanfare under a white flash (was: triple buzz + flash)
        disco.setWhite();
        Sounds::legComplete();
        disco.turnOff();

        // Laser wibble to indicate leg detected
        if (ctx.config.laserWibble) {
            laser.wibble();
        }

        ctx.shotBuf.clear();

        // Latch purple
        disco.setPurple();
        ctx.purpleLatched = true;
        ctx.measurementTaken = true;

        Serial.println(F("LEG COMPLETE — 3 consistent readings"));
        return;
    }

    // Successful reading but leg not complete
    // (function already returned if leg complete)
    Sounds::readingOk();

    Serial.println(F("  HS:5 done"));
    Serial.flush();
    ctx.measurementTaken = false;
}

// ── Error alert — red flash sequence ────────────────────────────
static void alertError(const char *errCode) {
    if (dispOk) {
        display.updateDistanceText(errCode);
        display.refresh();
    }

    // Red failure disco
    disco.turnOff();
    for (int i = 0; i < 4; i++) {
        disco.setRed();
        delay(100);
        disco.turnOff();
        delay(100);
    }

    // Beep after error flashes
    Sounds::error();

    // Re-enable laser
    laser.setLaser(true);
    ctx.laserEnabled = true;
    ctx.measurementTaken = true;
}

// ── BLE pin monitoring (connection state + flush) ───────────────
static void pollBLEPin(uint32_t now) {
    if (!bleOk) {
        return;
    }
    if (now - lastBleCheck < Timing::BLE_PIN_CHECK_MS) {
        return;
    }
    lastBleCheck = now;

    bool connected = ble.isConnected();
    ctx.bleConnected = connected;

    // Transition: disconnected → connected
    if (connected && !lastBleConnected) {
        Serial.println(F("BLE: connected"));

        // Flush pending readings if any
        if (ctx.bleDisconnectionCounter > 0 && flashOk) {
            disco.setBlue();
            if (dispOk) {
                display.updateBTNumber(ctx.bleDisconnectionCounter);
                display.refresh();
            }
            delay(1000); // let BLE slave be ready

            configMgr.flushPendingReadings(onFlushReading);
            configMgr.clearPendingReadings();

            ctx.bleDisconnectionCounter = 0;
            ctx.bleReadingsTransferredFlag = false;
            disco.turnOff();
            Serial.println(F("BLE: pending readings flushed"));
        }
    }

    if (!connected && lastBleConnected) {
        Serial.println(F("BLE: disconnected"));
    }

    // Update display
    if (dispOk) {
        display.updateBTLabel(connected);
        if (connected) {
            if (ctx.bleReadingsTransferredFlag) {
                display.updateBTNumber(0);
                ctx.bleDisconnectionCounter = 0;
            } else {
                display.updateBTNumber(0);
            }
        } else {
            display.updateBTNumber(ctx.bleDisconnectionCounter);
        }
    }

    lastBleConnected = connected;
}

// ── BLE UART command processing ─────────────────────────────────
static void pollBLECommands(uint32_t now) {
    if (!bleOk) {
        return;
    }
    if (now - lastBleUartPoll < Timing::BLE_UART_POLL_MS) {
        return;
    }
    lastBleUartPoll = now;

    ble.update();
    if (!ble.hasCommand()) {
        return;
    }

    BleCommand cmd = ble.readCommand();
    Serial.print(F("BLE CMD: "));
    Serial.println(BleManager::commandName(cmd));

    switch (cmd) {
    case BleCommand::ACK_RECEIVED:
        ctx.bleReadingsTransferredFlag = true;
        break;

    case BleCommand::TAKE_SHOT:
        prepareForShot();
        startShot(REGULAR_SHOT);
        ctx.lastActivityTime = now;
        break;

    case BleCommand::LASER_ON:
        laserOn();
        break;

    case BleCommand::LASER_OFF:
        laserOff();
        break;

    case BleCommand::DEVICE_OFF:
        doShutdown();
        break;

    case BleCommand::START_CAL:
        Serial.println(F("BLE: entering menu mode"));
        laserOff();
        disco.turnOff();
        ctx.discoOn = false;
        menuMgr.begin(display.getDisplay(), ctx, configMgr);
        break;

    case BleCommand::STOP_CAL:
        ctx.currentState = SystemState::IDLE;
        break;

    default:
        break;
    }
}

// ── Hardware power button (LTC2954 INT, edge-triggered) ─────────────
// (Replaces both V1's SHUTDOWN GPIO button and the DiscoX UART name-sync
// handshake — the BLE name is now set locally at boot in initBle().)
static void pollPowerButton(uint32_t now) {
    bool pressed = power.buttonPressed(); // INT reads LOW

    // Boot-hold guard: don't arm until the power-on press has released
    if (!pwrBtnArmed) {
        if (!pressed) {
            pwrBtnArmed = true;
        }
        return;
    }

    if (!pressed) {
        pwrBtnLowSince = 0;
        return;
    }

    if (pwrBtnLowSince == 0) {
        pwrBtnLowSince = now;                // LOW edge — start confirm window
    } else if (now - pwrBtnLowSince >= 20) { // 20 ms confirm (INT pulses <1 s)
        Serial.println(F("SHUTDOWN: power button pressed"));
        doShutdown();
    }
}

// ── Battery check (every 30s) ───────────────────────────────────
static void pollBattery(uint32_t now) {
    if (!batOk) {
        return;
    }
    if (now - lastBatRead < Timing::BATTERY_CHECK_MS) {
        return;
    }
    lastBatRead = now;

    lastBatPct = battery.cellPercent();
    ctx.readings.batteryLevel = lastBatPct;
    Serial.print(F("BAT: "));
    Serial.print(battery.cellVoltage(), 3);
    Serial.print(F("V  "));
    Serial.print(lastBatPct, 1);
    Serial.println(F("%"));

    if (dispOk) {
        display.updateBattery(lastBatPct);
    }

    if (lastBatPct > 0.5f && lastBatPct <= Timing::BATTERY_SHUTDOWN_PCT) {
        Serial.println(F("LOW BATTERY — shutting down"));
        if (dispOk) {
            display.updateDistanceText("LOW");
            display.updateAzimuth(0);
            display.updateInclination(0);
            display.refresh();
        }
        disco.setRed();
        delay(3000);
        doShutdown();
    }
}

// ── Auto shutdown check (every 5s) ──────────────────────────────
static void checkAutoShutoff(uint32_t now) {
    if (now - lastAutoShutCheck < Timing::AUTO_SHUTOFF_CHECK_MS) {
        return;
    }
    lastAutoShutCheck = now;

    uint32_t inactiveSec = (now - ctx.lastActivityTime) / 1000;
    if (inactiveSec > ctx.config.autoShutdownTimeout) {
        Serial.println(F("Inactivity timeout — shutting down"));
        doShutdown();
    }
}

// ── Laser timeout check (every 1s) ─────────────────────────────
static void checkLaserTimeout(uint32_t now) {
    if (now - lastLaserTimeCheck < Timing::LASER_TIMEOUT_CHECK_MS) {
        return;
    }
    lastLaserTimeCheck = now;

    if (!ctx.laserEnabled) {
        return;
    }

    uint32_t inactiveSec = (now - ctx.lastActivityTime) / 1000;
    if (inactiveSec > ctx.config.laserTimeout) {
        Serial.println(F("Laser timeout — turning off"));
        if (laserOk) {
            laser.setLaser(false);
        }
        ctx.laserEnabled = false;
    }
}

// ── Display update (4 Hz) ───────────────────────────────────────
static void updateDisplay(uint32_t now) {
    if (!dispOk) {
        return;
    }

    // Show "Splays not enabled" toast for 2s, suppressing normal refresh.
    // Use millis() not now — now is stale from the top of loop() and may
    // predate splaysToastTime (set after a blocking beep), causing underflow.
    if (splaysToastTime != 0) {
        if (millis() - splaysToastTime < 1200) {
            if (now - lastDisplayRefresh >= Timing::DISPLAY_REFRESH_MS) {
                lastDisplayRefresh = now;
                auto &d = display.getDisplay();
                d.clearDisplay();
                d.setTextColor(SH110X_WHITE);
                d.setTextSize(2);
                d.setCursor(0, 44);
                d.println(F("Splays not"));
                d.println(F("enabled"));
                d.display();
            }
            return;
        } else {
            splaysToastTime = 0;
        }
    }

    if (now - lastDisplayRefresh < Timing::DISPLAY_REFRESH_MS) {
        return;
    }
    lastDisplayRefresh = now;

    // Only update live sensor readings when display is not frozen.
    // After a measurement the display stays frozen showing the shot
    // reading until the user presses MEASURE again to resume live mode.
    if (!ctx.displayFrozen) {
        float liveAz, liveInc;
        if (calOk) {
            liveAz = sensorMgr.getAzimuth();
            liveInc = sensorMgr.getInclination();
        } else {
            liveAz = wrapTo360(radiansToDegrees(atan2f(lastMagY, lastMagX)));
            liveInc = radiansToDegrees(atan2f(lastAccZ, sqrtf(lastAccX * lastAccX + lastAccY * lastAccY)));
        }

        // Live update with a small deadband against last-digit flicker.
        // V1's motion-gated freeze + anchor clamp (which hid the noisy
        // ISM330DHCX by locking the display to ±0.1° once still) is gone —
        // the SCA3300/RM3100 EMA output is steady enough to show live, and
        // the clamp could mask slow re-aims below the motion threshold.
        if (circularDiff(liveAz, dispAz) > DEADBAND_ANGLE) {
            dispAz = liveAz;
        }
        if (fabsf(liveInc - dispInc) > DEADBAND_ANGLE) {
            dispInc = liveInc;
        }

        display.updateSensorReadings(lastDistance, dispAz, dispInc);

        // Teleplot output ('s' in the serial monitor silences/resumes)
        if (teleplotEnabled) {
            Serial.print(F(">azimuth:"));
            Serial.println(dispAz, 1);
            Serial.print(F(">inclination:"));
            Serial.println(dispInc, 1);
        }
    }

    display.updateBattery(lastBatPct);
    display.updateBTLabel(bleOk ? ble.isConnected() : false);
    display.refresh();
}

// ═══════════════════════════════════════════════════════════════════
// ── Helper Functions ──────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

static void resetLaser() {
    if (!laserOk) {
        return;
    }
    Serial.println(F("LASER: resetting"));
    laser.setLaser(false);
    delay(100);
    while (Serial1.available()) {
        Serial1.read(); // flush UART
    }
    laser.setLaser(true);
    delay(200); // let module stabilise
    ctx.laserEnabled = true;
}

static void doShutdown() {
    Serial.println(F(">>> SHUTDOWN"));
    laserOff();
    // Flush any unsaved readings to flash before power dies
    if (flashOk && configMgr.hasPendingToSync()) {
        configMgr.syncPendingToFlash();
    }
    Serial.flush();
    delay(100); // let the LittleFS write settle before cutting the rail
    if (dispOk) {
        display.blankScreen();
    }
    power.powerOff(); // KILL LOW — never returns (hangs if USB keeps us alive)
}

// Shared power-off hook (declared in config.h) for menu/snake timeout paths
void systemPowerOff() { doShutdown(); }

// Radio-quiet hook (declared in config.h) — calibration mode silences
// advertising for its whole session so LittleFS commits can't collide with
// radio timeslots (the core's flash HAL ignores SoftDevice flash errors).
void bleRadioQuiet(bool quiet) {
    if (!bleOk) {
        return;
    }
    if (quiet) {
        ble.pauseAdvertising();
    } else {
        ble.resumeAdvertising();
    }
}

// ═══════════════════════════════════════════════════════════════════
// ── USB Mass Storage Drive Mode (V2: internal-flash FAT partition) ─
// ═══════════════════════════════════════════════════════════════════
// Called from setup() right after the MSC interface is registered.
// LittleFS → FAT staging, then the host owns the volume until MENU is
// pressed (import + reboot) or the device is powered off (the usb_import
// flag makes the next boot import instead).
static void enterUsbDriveMode() {
    bool fatOk = UsbDrive::mountOrFormat();
    bool exportOk = fatOk && flashOk && UsbDrive::exportFiles(configMgr);

    // Power-cycle exit must still pick up host edits — flag it before the
    // host can touch anything.
    if (flashOk) {
        configMgr.writeFlag(Flags::USB_IMPORT);
    }
    UsbDrive::setHostAccess(true);

    Serial.begin(115200);
    Serial.println(F("=== USB Drive Mode ==="));
    if (!fatOk) {
        Serial.println(F("FAT partition bad — format it (FAT) from the PC"));
    } else if (!exportOk) {
        Serial.println(F("WARNING: file staging incomplete"));
    }

    if (dispOk) {
        auto &d = display.getDisplay();
        d.clearDisplay();
        d.setTextSize(1);
        d.setTextColor(SH110X_WHITE);
        d.setCursor(0, 4);
        d.println(F("USB Drive Mode"));
        d.println();
        d.println(F("Edit CONFIG.JSON on"));
        d.println(F("the MRZAPPY drive."));
        d.println();
        d.println(F("Eject, then press"));
        d.println(F("MENU to save & exit"));
        d.println(F("(or just power off)."));
        d.display();
    }

    // Spin: MENU press exits, power button still powers off cleanly.
    uint32_t menuLowSince = 0;
    while (true) {
        uint32_t now = millis();
        pollPowerButton(now);
        if (digitalRead(PIN_BTN_MENU) == LOW) {
            if (menuLowSince == 0) {
                menuLowSince = now;
            } else if (now - menuLowSince >= 50) {
                break; // debounced MENU press — save & exit
            }
        } else {
            menuLowSince = 0;
        }
        delay(10);
    }

    // Revoke host access and let any in-flight host write drain before this
    // task touches the volume (MSC callbacks share the flash page cache).
    UsbDrive::setHostAccess(false);
    delay(500);

    Serial.println(F("USB drive closed — importing edits"));
    bool importOk = flashOk && UsbDrive::importFiles(configMgr);
    if (flashOk) {
        configMgr.clearFlag(Flags::USB_IMPORT);
    }

    if (dispOk) {
        auto &d = display.getDisplay();
        d.clearDisplay();
        d.setTextSize(1);
        d.setTextColor(SH110X_WHITE);
        d.setCursor(0, 20);
        if (importOk) {
            d.println(F("Settings saved."));
        } else {
            d.println(F("Some files invalid -"));
            d.println(F("old data kept."));
        }
        d.println();
        d.println(F("Restarting..."));
        d.display();
    }
    delay(1500);
    NVIC_SystemReset();
    // Does not return
}

static void onFlushReading(float az, float inc, float dist) {
    Serial.print(F("FLUSH: az="));
    Serial.print(az, 1);
    Serial.print(F(" inc="));
    Serial.print(inc, 1);
    Serial.print(F(" dist="));
    Serial.println(dist, 2);
    ble.sendSurveyData(az, inc, dist);
    delay(50); // pacing between BLE sends
}

// ═══════════════════════════════════════════════════════════════════
// ── Initialization Functions (unchanged from previous sessions) ───
// ═══════════════════════════════════════════════════════════════════

static void initPins() {
    // Power pins (KILL/INT/PGOOD) are configured in power.begin() at the very
    // top of setup(); buzzer pins in buzzer.begin(); laser UART in initLaser().
    pinMode(PIN_BTN_FIRE, INPUT_PULLUP);
    pinMode(PIN_BTN_UP_DISCO, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_MENU, INPUT_PULLUP);

    pinMode(PIN_MAG_DRDY, INPUT);

    Serial.println(F("Pins initialized."));
}

static void scanI2C() {
    uint8_t count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x"));
            if (addr < 16) {
                Serial.print('0');
            }
            Serial.print(addr, HEX);
            if (addr == RM3100_I2C_ADDR) {
                Serial.print(F(" (RM3100 mag)"));
            }
            if (addr == MAX17048_ADDR) {
                Serial.print(F(" (MAX17048 battery)"));
            }
            if (addr == SH1107_ADDR) {
                Serial.print(F(" (SH1107 OLED)"));
            }
            Serial.println();
            count++;
        }
    }
    Serial.print(F("  Found "));
    Serial.print(count);
    Serial.println(F(" device(s)."));
}

// initSensors() inlined into setup() for splash progress updates

static void initDisplay() {
    dispOk = display.begin();
    Serial.print(F("SH1107:      "));
    Serial.println(dispOk ? F("OK") : F("FAILED"));

    // Splash is shown from setup() after initDisplay() returns

    Serial.println();
}

static void initLaser() {
    // LDJ-100 on Serial1 — V2 pins are not the variant defaults, and the
    // module's TXD is open drain (RX needs the pull-up).
    Serial1.setPins(PIN_LASER_RX, PIN_LASER_TX);
    pinMode(PIN_LASER_RX, INPUT_PULLUP);
    Serial1.begin(LASER_UART_BAUD_LDJ100);
    delay(20);

    // Retry the probe instead of a single early ping. On a COLD power-button
    // boot the laser rail (ENA-gated) comes up with the MCU, and the module
    // spends its first ~2.5 s in an auto-baud window where it won't answer a
    // fixed-baud read. A single 20 ms-in ping fails there, latching
    // laserOk=false for the whole session — the symptom is FIRE only ever
    // printing "getting ready for a shot" and never measuring. Warm USB-reflash
    // resets happen to work because the module is already up. ~15 attempts of
    // (≤300 ms ping + 100 ms) covers the wake-up/auto-baud window (~4-6 s worst
    // case only when the laser is truly absent; a healthy module answers fast).
    laserOk = false;
    for (int attempt = 0; attempt < 15 && !laserOk; attempt++) {
        laserOk = laser.begin(Serial1);
        if (!laserOk) {
            delay(100);
        }
    }
    Serial.print(F("Laser:       "));
    Serial.println(laserOk ? F("OK (LDJ-100 @ 115200)") : F("FAILED (no response after retries)"));

    laserOff();

    Serial.println();
}

static void initBle() {
    // In-process SAP6 BLE stack (replaces the V1 UART bridge to DiscoX).
    // Must run AFTER initFlash() — the advertised name comes from config.
    bleOk = ble.begin(ctx.config.bleName);
    Serial.print(F("BLE (SAP6):  "));
    Serial.println(bleOk ? F("OK") : F("FAILED"));

    Serial.println();
}

static void initFlash() {
    flashOk = configMgr.begin();
    Serial.print(F("Flash/FS:    "));
    Serial.println(flashOk ? F("OK") : F("FAILED (using defaults)"));

    // Drive mode exited by power-cycle instead of MENU? Import host edits
    // now, BEFORE loadConfig/initCalibration read the LittleFS copies.
    if (flashOk && configMgr.hasFlag(Flags::USB_IMPORT)) {
        configMgr.clearFlag(Flags::USB_IMPORT);
        Serial.println(F("  ** USB import flag — checking drive for edits"));
        if (UsbDrive::mountOrFormat()) {
            UsbDrive::importFiles(configMgr);
        }
    }

    if (flashOk) {
        if (configMgr.loadConfig(ctx.config)) {
            Serial.println(F("  Config loaded from flash"));
        } else {
            Serial.println(F("  No saved config — writing defaults"));
            configMgr.saveConfig(ctx.config);
        }
        ctx.legChecker.setTolerance(ctx.config.cartesianTolerance);
        ctx.stabilityChecker.setTolerance(ctx.config.stabilityTolerance);

        if (dispOk) {
            display.setBrightness(ctx.config.screenBrightness);
        }

        uint16_t pending = configMgr.countPendingReadings();
        if (pending > 0) {
            Serial.print(F("  Pending readings: "));
            Serial.println(pending);
            ctx.bleDisconnectionCounter = pending;
        }

        if (configMgr.hasFlag(Flags::CALIBRATION)) {
            configMgr.clearFlag(Flags::CALIBRATION);
            Serial.println(F("  ** Calibration mode flag detected"));
            enterCalibMode = true;
        }
        if (configMgr.hasFlag(Flags::MENU)) {
            configMgr.clearFlag(Flags::MENU);
            Serial.println(F("  ** Menu mode flag detected"));
            enterMenuMode = true;
        }
        if (configMgr.hasFlag(Flags::SNAKE)) {
            configMgr.clearFlag(Flags::SNAKE);
            Serial.println(F("  ** Snake mode flag detected"));
            enterSnakeMode = true;
        }

        // Write self-test: a corrupt filesystem can mount and read fine while
        // every commit fails — warn at boot rather than at save time.
        // Retried before warning: single flash ops can hiccup transiently
        // (cold-boot rail settling etc.), and that must not cry wolf — the
        // genuine zombie-FS state fails deterministically on every attempt.
        bool storageOk = false;
        for (int i = 0; i < 3 && !storageOk; i++) {
            if (i > 0) {
                Serial.println(F("  self-test retrying..."));
                delay(50);
            }
            storageOk = configMgr.storageWriteTest();
        }
        if (!storageOk) {
            Serial.println(F("  ** STORAGE DEGRADED: write self-test FAILED — reformat advised"));
            if (dispOk) {
                display.blankScreen();
                auto &disp = display.getDisplay();
                disp.setTextColor(SH110X_WHITE);
                disp.setTextSize(2);
                disp.setCursor(0, 10);
                disp.println(F("STORAGE"));
                disp.println(F("DEGRADED"));
                disp.setTextSize(1);
                disp.println();
                disp.println(F("Saving will fail."));
                disp.println();
                disp.println(F("Fix: Menu > Settings"));
                disp.println(F("> Reformat Storage"));
                disp.display();
                delay(5000);
            }
        }
    }

    Serial.println();
}

static void initCalibration() {
    Serial.print(F("Calibration: "));

    bool loaded = false;
    calFromFlash = false;

    // 1. Try binary from flash (fastest — no JSON parse)
    if (flashOk) {
        MagCal::CalibrationBinary bin;
        if (configMgr.loadCalibrationBinary(bin)) {
            loaded = calibration.fromBinary(bin);
            if (loaded) {
                calFromFlash = true;
                Serial.println(F("OK (from binary)"));
            } else {
                Serial.println(F("binary file found but CRC/parse failed"));
            }
        } else {
            Serial.println(F("no binary file on flash"));
        }
    }

    // 2. Fall back to JSON from flash
    if (!loaded && flashOk) {
        char calBuf[2048];
        size_t calLen = 0;
        if (configMgr.loadCalibrationJson(calBuf, sizeof(calBuf), calLen)) {
            loaded = calibration.fromJson(calBuf, calLen);
            if (loaded) {
                calFromFlash = true;
                Serial.println(F("OK (from flash JSON)"));
            } else {
                Serial.print(F("JSON parse failed ("));
                Serial.print(calLen);
                Serial.println(F(" bytes)"));
            }
        } else {
            Serial.println(F("no JSON file or too large for buffer"));
        }
    }

    // 3. Fall back to compiled-in PROGMEM JSON
    if (!loaded) {
        loaded = calibration.fromJson(CALIBRATION_JSON, sizeof(CALIBRATION_JSON) - 1);
        if (loaded) {
            Serial.println(F("WARNING: using PROGMEM fallback — calibration may be stale!"));
        }
    }

    calOk = loaded;

    if (calOk) {
        Serial.print(F("  Mag axes:  "));
        Serial.println(calibration.mag().axes().toString());
        Serial.print(F("  Grav axes: "));
        Serial.println(calibration.grav().axes().toString());
        Serial.print(F("  Dip avg:   "));
        Serial.print(calibration.dipAvg(), 1);
        Serial.println(F(" deg"));

        sensorMgr.init(&calibration, ctx.config.emaAlphaStable, ctx.config.emaAlphaMoving,
                       ctx.config.stabilityBufferLength);
        Serial.print(F("  Filter:    EMA stable="));
        Serial.print(ctx.config.emaAlphaStable, 2);
        Serial.print(F(", moving="));
        Serial.print(ctx.config.emaAlphaMoving, 2);
        Serial.print(F(", stability buf="));
        Serial.println(ctx.config.stabilityBufferLength);

        // Warn user if calibration came from PROGMEM (stale compile-time data)
        if (!calFromFlash && dispOk) {
            display.blankScreen();
            auto &disp = display.getDisplay();
            disp.setTextColor(SH110X_WHITE);
            disp.setTextSize(2);
            disp.setCursor(0, 10);
            disp.println(F("NOT"));
            disp.println(F("CALIBRATED"));
            disp.setTextSize(1);
            disp.println();
            disp.println(F("Using stale built-in"));
            disp.println(F("calibration data."));
            disp.println();
            disp.println(F("Recalibrate before"));
            disp.println(F("surveying!"));
            disp.display();
            disco.setRed();
            delay(4000);
            disco.turnOff();
        }
    } else {
        Serial.println(F("FAILED — using raw angles"));
    }

    Serial.println();
}

static void initDisco() {
    // V2: no power-gate pin — the WS2812 rail is hardware-gated by ENA
    disco.begin();
    Serial.println(F("WS2812:      OK"));
    Serial.println();
}
