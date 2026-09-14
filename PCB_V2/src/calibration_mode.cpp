#include "calibration_mode.h"
#include "math_utils.h"
#include <math.h>

// ── Initialization ──────────────────────────────────────────────────

void CalibrationMode::begin(ButtonManager &btns, DisplayManager &disp, DiscoManager &disco,
                            LaserManager &laser, RM3100 &magSensor, SCA3300 &accel, ConfigManager &cfgMgr,
                            MagCal::Calibration &cal, const Config &config, CalMode mode) {
    btns_ = &btns;
    disp_ = &disp;
    disco_ = &disco;
    laser_ = &laser;
    magSensor_ = &magSensor;
    accel_ = &accel;
    cfgMgr_ = &cfgMgr;
    cal_ = &cal;

    // Apply consistency settings from config
    bufferLen_ = min((int)config.calBufferLength, MAX_BUFFER_SIZE);
    magThreshold_ = config.calMagConsistency;
    gravThreshold_ = config.calGravConsistency;
    settleMs_ = config.calSettleMs;
    calEmaAlpha_ = config.calEmaAlpha;
    calTimeoutMs_ = config.calTimeoutMs;

    // Clear data
    magArray_.clear();
    gravArray_.clear();
    bufferCount_ = 0;
    bufferIdx_ = 0;
    emaInitialized_ = false;
    waitingForStable_ = false;
    settleStart_ = 0;
    accumMag_ = Eigen::Vector3f::Zero();
    accumGrav_ = Eigen::Vector3f::Zero();
    accumCount_ = 0;
    iteration_ = 0;
    holdCounter_ = 0.0f;
    beepActive_ = false;
    laserWibbleActive_ = false;
    laserOnTime_ = 0;
    resultMagAcc_ = 0.0f;
    resultGravAcc_ = 0.0f;
    resultAccuracy_ = 0.0f;

    memset(coverageZones_, 0, sizeof(coverageZones_));

    // Turn on laser for calibration
    laser_->setLaser(true);

    // Silence BLE advertising for the whole session: SoftDevice flash ops
    // fail under radio contention (core HAL ignores the error event), which
    // made calibration saves fail transiently. Resumed by main.cpp when
    // calibration mode finishes; reboot exits reset the radio anyway.
    bleRadioQuiet(true);

    calMode_ = mode;

    if (mode == CalMode::PART1_ELLIPSOID) {
        Serial.println(F("Calibration mode: Part 1 (Ellipsoid)"));
        state_ = CalibState::INTRO_ELLIPSOID;
        showEllipsoidIntro();
    } else {
        Serial.println(F("Calibration mode: Part 2 (Alignment)"));
        state_ = CalibState::INTRO_ALIGNMENT;
        showAlignmentIntro();
    }
}

// ── Main update loop ────────────────────────────────────────────────

bool CalibrationMode::update() {
    if (state_ == CalibState::INACTIVE || state_ == CalibState::DONE) {
        return true;
    }

    // Note: btns_->update() is called by loop() before calMode.update(),
    // so we must NOT call it again here — that would clear edge flags.
    updateBeep();

    // V2: power-off is the hardware power button (LTC2954 INT), handled
    // globally in loop() before mode dispatch — no shutdown GPIO button here.
    // The SHUTDOWN_CONFIRM state below is retained but never entered.

    switch (state_) {
    case CalibState::INTRO_ELLIPSOID:
    case CalibState::INTRO_ALIGNMENT:
        updateIntro();
        break;
    case CalibState::COLLECTING_ELLIPSOID:
    case CalibState::COLLECTING_ALIGNMENT:
        updateCollecting();
        break;
    case CalibState::CALCULATING_ELLIPSOID:
        updateCalculatingEllipsoid();
        break;
    case CalibState::CALCULATING_ALIGNMENT:
        updateCalculatingAlignment();
        break;
    case CalibState::SHOW_RESULTS:
        updateShowResults();
        break;
    case CalibState::SAVING:
        updateSaving();
        break;
    case CalibState::FB_INTRO:
        updateFBIntro();
        break;
    case CalibState::FB_WAIT_FORESIGHT:
    case CalibState::FB_WAIT_BACKSIGHT:
        updateFBWaitShot();
        break;
    case CalibState::FB_PAIR_RESULT:
        updateFBPairResult();
        break;
    case CalibState::FB_CALCULATING:
        updateFBCalculating();
        break;
    case CalibState::FB_RESULTS:
        updateFBResults();
        break;
    case CalibState::FB_SAVING:
        updateFBSaving();
        break;
    case CalibState::SHUTDOWN_CONFIRM:
        updateShutdownConfirm();
        break;
    default:
        break;
    }

    return (state_ == CalibState::DONE);
}

// ── State: INTRO (ellipsoid or alignment) ───────────────────────────

void CalibrationMode::updateIntro() {
    // Dismiss intro screen on any button press
    if (btns_->wasPressed(Button::FIRE) || btns_->wasPressed(Button::UP_DISCO) ||
        btns_->wasPressed(Button::DOWN) || btns_->wasPressed(Button::MENU)) {

        if (state_ == CalibState::INTRO_ELLIPSOID) {
            // Start ellipsoid collection
            targetCount_ = 56;
            iteration_ = 0;
            magArray_.clear();
            gravArray_.clear();
            magArray_.reserve(56);
            gravArray_.reserve(56);
            bufferCount_ = 0;
            bufferIdx_ = 0;
            waitingForStable_ = false;

            state_ = CalibState::COLLECTING_ELLIPSOID;
            showEllipsoidScreen();
            Serial.println(F("Starting ellipsoid collection (56 points)..."));
        } else {
            // Start alignment collection
            targetCount_ = 24;
            iteration_ = 0;
            magArray_.clear();
            gravArray_.clear();
            magArray_.reserve(24);
            gravArray_.reserve(24);
            bufferCount_ = 0;
            bufferIdx_ = 0;
            waitingForStable_ = false;

            state_ = CalibState::COLLECTING_ALIGNMENT;
            showAlignmentProgress();
            Serial.println(F("Starting alignment collection (24 points: 3 dirs x 8 "
                             "rotations)..."));
        }
    }
}

// ── State: COLLECTING ───────────────────────────────────────────────

void CalibrationMode::updateCollecting() {
    // Sample sensors at ~100 Hz
    uint32_t now = millis();
    if (now - lastSampleTime_ < 10) {
        return;
    }
    lastSampleTime_ = now;

    // Read raw sensor data
    Eigen::Vector3f magReading, gravReading;
    readSensors(magReading, gravReading);

    // EMA pre-filter: smooth noise before consistency check
    if (!emaInitialized_) {
        emaMag_ = magReading;
        emaGrav_ = gravReading;
        emaInitialized_ = true;
    } else {
        emaMag_ = calEmaAlpha_ * magReading + (1.0f - calEmaAlpha_) * emaMag_;
        emaGrav_ = calEmaAlpha_ * gravReading + (1.0f - calEmaAlpha_) * emaGrav_;
    }

    // Push smoothed readings into rolling consistency buffer
    magBuffer_[bufferIdx_] = emaMag_;
    gravBuffer_[bufferIdx_] = emaGrav_;
    bufferIdx_ = (bufferIdx_ + 1) % bufferLen_;
    if (bufferCount_ < bufferLen_) {
        bufferCount_++;
    }

    // FIRE button starts a capture attempt
    if (btns_->wasPressed(Button::FIRE) && !waitingForStable_) {
        waitingForStable_ = true;
        captureStart_ = now;
        settleStart_ = 0;
        accumMag_ = Eigen::Vector3f::Zero();
        accumGrav_ = Eigen::Vector3f::Zero();
        accumCount_ = 0;
        disco_->setRed();
    }

    // DOWN (B3, as shown on screen) or MENU (B4) undoes the last recorded
    // point. Read both edges unconditionally — wasPressed() consumes the
    // flag, so short-circuiting would leave a stale edge to fire later.
    bool undoDown = btns_->wasPressed(Button::DOWN);
    bool undoMenu = btns_->wasPressed(Button::MENU);
    bool undoPressed = undoDown || undoMenu;

    if (undoPressed && waitingForStable_) {
        // Mid-capture: cancel the attempt (otherwise the 4 s stability
        // timeout force-records a junk point and the press is lost)
        waitingForStable_ = false;
        settleStart_ = 0;
        disco_->turnOff();
        Serial.println(F("Capture attempt cancelled"));
    } else if (undoPressed && iteration_ > 0) {
        magArray_.pop_back();
        gravArray_.pop_back();
        iteration_--;

        // Brief red flash to indicate deletion
        disco_->setRed();
        beep();
        delay(150);
        disco_->turnOff();

        Serial.print(F("Undo: back to "));
        Serial.print(iteration_);
        Serial.print(F("/"));
        Serial.println(targetCount_);

        // Recalculate coverage bar for ellipsoid mode
        if (state_ == CalibState::COLLECTING_ELLIPSOID) {
            memset(coverageZones_, 0, sizeof(coverageZones_));
            for (size_t i = 0; i < gravArray_.size(); i++) {
                updateCoverageBar(gravArray_[i]);
            }
            showEllipsoidScreen();
        } else {
            showAlignmentProgress();
        }

        // Reset stability buffer so next capture starts fresh
        bufferCount_ = 0;
        bufferIdx_ = 0;
    }

    // Check stability when waiting and buffer is full
    if (waitingForStable_ && bufferCount_ >= bufferLen_) {
        bool magConsistent = isConsistent(magBuffer_, bufferLen_, magThreshold_);
        bool gravConsistent = isConsistent(gravBuffer_, bufferLen_, gravThreshold_);

        if (magConsistent && gravConsistent) {
            // Accumulate this sample into the averaging pool
            accumMag_ += magReading;
            accumGrav_ += gravReading;
            accumCount_++;

            // Start settle timer on first consistent frame
            if (settleStart_ == 0) {
                settleStart_ = now;
            }

            // Wait for settle duration before accepting
            if (now - settleStart_ < settleMs_) {
                return; // keep accumulating
            }

            // Settle complete — record the averaged point
            Eigen::Vector3f avgMag = accumMag_ / (float)accumCount_;
            Eigen::Vector3f avgGrav = accumGrav_ / (float)accumCount_;
            acceptPoint(avgMag, avgGrav);
        } else {
            // Consistency lost — reset settle timer and accumulator
            if (settleStart_ != 0) {
                settleStart_ = 0;
                accumMag_ = Eigen::Vector3f::Zero();
                accumGrav_ = Eigen::Vector3f::Zero();
                accumCount_ = 0;
            }

            // Timeout — accept the current EMA-smoothed reading if we've waited too
            // long
            if (calTimeoutMs_ > 0 && (now - captureStart_) >= calTimeoutMs_) {
                Serial.println(F("Stability timeout — accepting EMA reading"));
                acceptPoint(emaMag_, emaGrav_);
            }
        }
    }
}

// ── State: CALCULATING_ELLIPSOID ────────────────────────────────────

void CalibrationMode::updateCalculatingEllipsoid() {
    // Show calculating message
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(0, 50);
    d.print(F("Calculating..."));
    d.display();

    calculateEllipsoid();

    // Part 1 ends here — show results and allow save
    Serial.println(F("Ellipsoid fit complete."));
    state_ = CalibState::SHOW_RESULTS;
    showResultsScreen();
}

// ── State: CALCULATING_ALIGNMENT ────────────────────────────────────

void CalibrationMode::updateCalculatingAlignment() {
    // Show calculating message
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(0, 50);
    d.print(F("Calculating..."));
    d.display();

    calculateAlignment();

    state_ = CalibState::SHOW_RESULTS;
    showResultsScreen();
}

// ── State: SHOW_RESULTS ─────────────────────────────────────────────

void CalibrationMode::updateShowResults() {
    // Hold UP_DISCO+DOWN to save, hold DOWN alone to discard.
    // A failed ellipsoid fit can only be discarded — never saved.
    bool up = btns_->isPressed(Button::UP_DISCO);
    bool down = btns_->isPressed(Button::DOWN);

    if (up && down && !ellipsoidFitFailed()) {
        holdCounter_ += 0.01f;
        if (holdCounter_ >= HOLD_TIME) {
            Serial.println(F("Saving calibration..."));
            state_ = CalibState::SAVING;
            showSavingScreen();
        }
    } else if (!up && down) {
        holdCounter_ += 0.01f;
        if (holdCounter_ >= HOLD_TIME) {
            Serial.println(F("Calibration discarded — rebooting to restore saved cal."));
            auto &d = disp_->getDisplay();
            d.clearDisplay();
            d.setTextColor(SH110X_WHITE);
            d.setTextSize(2);
            d.setCursor(0, 40);
            d.println(F("Discarded."));
            d.setCursor(0, 70);
            d.print(F("Restarting..."));
            d.display();
            delay(2000);
            NVIC_SystemReset();
        }
    } else {
        holdCounter_ = 0.0f;
    }
}

// ── State: SAVING ───────────────────────────────────────────────────

void CalibrationMode::updateSaving() {
    auto &d = disp_->getDisplay();
    if (saveCalibration()) {
        Serial.println(F("Calibration saved to flash."));

        d.clearDisplay();
        d.setTextColor(SH110X_WHITE);
        d.setTextSize(2);
        d.setCursor(20, 50);
        d.print(F("Saved!"));
        d.display();
        delay(1500);
    } else {
        Serial.println(F("Failed to save calibration!"));

        d.clearDisplay();
        d.setTextColor(SH110X_WHITE);
        d.setTextSize(2);
        d.setCursor(10, 40);
        d.print(F("Save FAIL"));
        d.setTextSize(1);
        d.setCursor(0, 70);
        d.println(F("Storage error - fit"));
        d.println(F("kept. Retry save, or"));
        d.println(F("reformat storage via"));
        d.println(F("Settings menu."));
        d.display();
        delay(3000);

        // Keep the fitted calibration and collected data — go back to the
        // results screen so the user can retry the save or discard, rather
        // than silently marching on (Part 1 → Part 2) or rebooting.
        holdCounter_ = 0.0f;
        state_ = CalibState::SHOW_RESULTS;
        showResultsScreen();
        return;
    }
    // After Part 1 save, automatically transition into Part 2 (alignment)
    if (calMode_ == CalMode::PART1_ELLIPSOID) {
        Serial.println(F("Part 1 complete — starting Part 2 (Alignment)"));
        calMode_ = CalMode::PART2_ALIGNMENT;
        magArray_.clear();
        gravArray_.clear();
        bufferCount_ = 0;
        bufferIdx_ = 0;
        emaInitialized_ = false;
        waitingForStable_ = false;
        settleStart_ = 0;
        accumMag_ = Eigen::Vector3f::Zero();
        accumGrav_ = Eigen::Vector3f::Zero();
        accumCount_ = 0;
        iteration_ = 0;
        holdCounter_ = 0.0f;
        resultMagAcc_ = 0.0f;
        resultGravAcc_ = 0.0f;
        resultAccuracy_ = 0.0f;
        state_ = CalibState::INTRO_ALIGNMENT;
        showAlignmentIntro();
        return;
    }

    // Final save complete — reboot to cleanly load new calibration
    auto &d2 = disp_->getDisplay();
    d2.clearDisplay();
    d2.setTextColor(SH110X_WHITE);
    d2.setTextSize(2);
    d2.setCursor(0, 50);
    d2.print(F("Restarting..."));
    d2.display();
    Serial.println(F("Rebooting to apply new calibration..."));
    delay(2000);
    NVIC_SystemReset();
}

// ── Sensor reading ──────────────────────────────────────────────────

void CalibrationMode::readSensors(Eigen::Vector3f &mag, Eigen::Vector3f &grav) {
    // Read magnetometer
    RM3100::Reading mr = magSensor_->readSingle();
    float mx, my, mz;
    magSensor_->toMicroTesla(mr, mx, my, mz);
    mag = Eigen::Vector3f(mx, my, mz);

    // Read accelerometer — SCA3300 reports g; convert to m/s² so the
    // calibration pipeline sees the same units as V1's ISM330DHCX
    float gx, gy, gz;
    if (accel_->readAcceleration(gx, gy, gz)) {
        grav = Eigen::Vector3f(gx * GRAVITY_MS2, gy * GRAVITY_MS2, gz * GRAVITY_MS2);
    } else {
        grav = Eigen::Vector3f::Zero(); // failed read — consistency check will
                                        // reject it
    }
}

// ── Consistency check ───────────────────────────────────────────────

bool CalibrationMode::isConsistent(const Eigen::Vector3f *buffer, int count, float threshold) const {
    // Angular consistency: threshold is in degrees.
    // Orientation-independent — same hand wobble passes/fails regardless of
    // heading.
    Eigen::Vector3f ref = buffer[0].normalized();
    for (int i = 1; i < count; i++) {
        float dot = ref.dot(buffer[i].normalized());
        float angleDeg = acosf(fminf(dot, 1.0f)) * 57.2958f;
        if (angleDeg > threshold) {
            return false;
        }
    }
    return true;
}

Eigen::Vector3f CalibrationMode::average(const Eigen::Vector3f *buffer, int count) const {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for (int i = 0; i < count; i++) {
        sum += buffer[i];
    }
    return sum / (float)count;
}

void CalibrationMode::recordPoint(const Eigen::Vector3f &mag, const Eigen::Vector3f &grav) {
    magArray_.push_back(mag);
    gravArray_.push_back(grav);
}

// ── Coverage bar ────────────────────────────────────────────────────

void CalibrationMode::updateCoverageBar(const Eigen::Vector3f &grav) {
    // Map gravity vector to coverage zone
    float magnitude = grav.norm();
    if (magnitude < 0.001f) {
        return;
    }

    float nz = grav[2] / magnitude;
    float elevation = radiansToDegrees(asinf(fmaxf(-1.0f, fminf(1.0f, nz))));
    float azimuth = wrapTo360(radiansToDegrees(atan2f(grav[1], grav[0])));

    int row = (int)((elevation + 90.0f) / 45.0f);
    row = max(0, min(COV_ROWS - 1, row));
    int col = (int)(azimuth / 45.0f);
    col = max(0, min(COV_COLS - 1, col));

    coverageZones_[row][col] = true;
}

void CalibrationMode::acceptPoint(const Eigen::Vector3f &mag, const Eigen::Vector3f &grav) {
    recordPoint(mag, grav);

    Serial.print(F("Accepted point "));
    disco_->setGreen();
    beep();

    iteration_++;
    waitingForStable_ = false;
    bufferCount_ = 0;
    bufferIdx_ = 0;

    // Update display
    if (state_ == CalibState::COLLECTING_ELLIPSOID) {
        updateCoverageBar(grav);
        showEllipsoidScreen();
    } else {
        showAlignmentProgress();

        // Single beep at direction changes (every 8 points)
        if (iteration_ % 8 == 0 && iteration_ < targetCount_) {
            beep();
            Serial.println(F("Change direction..."));
        }
    }

    Serial.print(iteration_);
    Serial.print(F("/"));
    Serial.println(targetCount_);

    // Brief green flash then turn off LED
    delay(200);
    disco_->turnOff();

    // Alignment: laser off for 500ms then back on (visual feedback)
    if (state_ == CalibState::COLLECTING_ALIGNMENT) {
        laser_->setLaser(false);
        laserWibbleActive_ = true;
        laserOnTime_ = millis() + 500;
    }

    // Check if collection is complete
    if (iteration_ >= targetCount_) {
        Serial.println(F("Collection complete. Calculating..."));
        if (state_ == CalibState::COLLECTING_ELLIPSOID) {
            state_ = CalibState::CALCULATING_ELLIPSOID;
        } else {
            state_ = CalibState::CALCULATING_ALIGNMENT;
        }
    }
}

// ── Display helpers ─────────────────────────────────────────────────
// All calibration screens draw directly to the OLED via getDisplay()
// to avoid the measurement-layout template from DisplayManager::refresh().

void CalibrationMode::showEllipsoidIntro() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Calibrate"));
    d.println(F("Phase 1"));

    // Instructions
    d.setTextSize(1);
    d.setCursor(0, 44);
    d.println(F("Take 56 readings in a"));
    d.println(F("diverse range of"));
    d.println(F("directions and"));
    d.println(F("orientations."));

    // Dismiss prompt
    d.setCursor(0, 110);
    d.print(F("Press any button..."));

    d.display();
}

void CalibrationMode::showAlignmentIntro() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Calibrate"));
    if (calMode_ == CalMode::PART2_ALIGNMENT) {
        d.println(F("Part 2"));
    } else {
        d.println(F("short mode"));
    }

    // Instructions
    d.setTextSize(1);
    d.setCursor(0, 44);
    d.println(F("Take 3 sets of 8"));
    d.println(F("readings, barrel roll"));
    d.println(F("the laser around a"));
    d.println(F("single point for each"));
    d.println(F("set of 8."));

    // Dismiss prompt
    d.setCursor(0, 110);
    d.print(F("Press any button..."));

    d.display();
}

void CalibrationMode::showEllipsoidScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Ellipsoid"));

    // Counter
    char buf[16];
    snprintf(buf, sizeof(buf), "%d/56", iteration_);
    d.setTextSize(3);
    d.setCursor(0, 30);
    d.print(buf);

    // Instructions
    d.setTextSize(1);
    d.setCursor(0, 68);
    d.print(F("B1:record  B3:undo"));

    // Coverage bar
    showCoverageBar();

    d.display();
}

void CalibrationMode::showCoverageBar() {
    // Draw an 8-column coverage bar at bottom of display
    // Each column fills from bottom based on how many of 4 rows are covered
    // Bar area: x=8..119, y=84..111 (28 pixels tall, 4px per row + outline)
    auto &d = disp_->getDisplay();

    const int barX = 8, barY = 84, barW = 112, barH = 28;
    const int colW = barW / COV_COLS; // 14px per column
    const int rowH = 6;               // height per coverage row

    // Outline
    d.drawRect(barX, barY, barW, barH, SH110X_WHITE);

    // Fill columns based on coverage
    for (int c = 0; c < COV_COLS; c++) {
        int filled = 0;
        for (int r = 0; r < COV_ROWS; r++) {
            if (coverageZones_[r][c]) {
                filled++;
            }
        }

        if (filled > 0) {
            int fillH = filled * rowH;
            int fillY = barY + barH - 1 - fillH;
            d.fillRect(barX + 1 + c * colW, fillY, colW - 1, fillH, SH110X_WHITE);
        }

        // Column dividers
        if (c > 0) {
            d.drawFastVLine(barX + c * colW, barY, barH, SH110X_WHITE);
        }
    }

    // Print coverage to serial periodically
    if (iteration_ > 0 && iteration_ % 8 == 0) {
        int total = 0;
        for (int r = 0; r < COV_ROWS; r++) {
            for (int cc = 0; cc < COV_COLS; cc++) {
                if (coverageZones_[r][cc]) {
                    total++;
                }
            }
        }
        Serial.print(F("Coverage: "));
        Serial.print(total);
        Serial.println(F("/32 zones"));
    }
}

void CalibrationMode::showAlignmentProgress() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Alignment"));

    int stage = (iteration_ > 0) ? ((iteration_ - 1) / 8 + 1) : 1;
    int inStage = (iteration_ > 0) ? (((iteration_ - 1) % 8) + 1) : 0;

    // Stage label
    char buf[16];
    snprintf(buf, sizeof(buf), "Stage %d", stage);
    d.setTextSize(2);
    d.setCursor(0, 30);
    d.print(buf);

    // Progress counter
    snprintf(buf, sizeof(buf), "%d/8", inStage);
    d.setTextSize(3);
    d.setCursor(0, 56);
    d.print(buf);

    // Instructions
    d.setTextSize(1);
    d.setCursor(0, 100);
    d.print(F("B1:record  B3:undo"));

    d.display();
}

void CalibrationMode::showResultsScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    char buf[24];
    d.setTextSize(1);

    if (ellipsoidFitFailed()) {
        d.setTextSize(2);
        d.setCursor(0, 0);
        d.println(F("Fit FAILED"));

        d.setTextSize(1);
        d.setCursor(0, 30);
        d.println(F("Point coverage too"));
        d.println(F("poor to fit an"));
        d.println(F("ellipsoid. Redo with"));
        d.println(F("more spread poses."));

        d.setCursor(0, 84);
        d.println(F("Hold DOWN: Discard"));
        d.display();

        Serial.println(F("Ellipsoid fit FAILED — hold DOWN to discard"));
        return;
    }

    if (calMode_ == CalMode::PART1_ELLIPSOID) {
        // Title
        d.setTextSize(2);
        d.setCursor(0, 0);
        d.println(F("Results"));

        d.setTextSize(1);
        // Ellipsoid-only results: uniformity metrics
        d.setCursor(0, 28);
        snprintf(buf, sizeof(buf), "Mag:  %.5f", (double)resultMagAcc_);
        d.println(buf);
        d.setCursor(0, 40);
        snprintf(buf, sizeof(buf), "Grav: %.5f", (double)resultGravAcc_);
        d.println(buf);

        Serial.print(F("Results — Mag: "));
        Serial.print(resultMagAcc_, 4);
        Serial.print(F("  Grav: "));
        Serial.println(resultGravAcc_, 4);

        d.setCursor(0, 72);
        d.println(F("Hold UP+DOWN: Save"));
        d.setCursor(0, 84);
        d.println(F("Hold DOWN: Discard"));
        d.setCursor(0, 100);
        d.println(F("Lower = Better"));
    } else {
        // Alignment / short cal results: accuracy only
        d.setTextSize(1);
        d.setCursor(0, 10);
        d.println(F("Accuracy:"));
        d.setTextSize(2);
        d.setCursor(0, 22);
        snprintf(buf, sizeof(buf), "%.3f deg", (double)resultAccuracy_);
        d.println(buf);
        d.setTextSize(1);
        d.setCursor(0, 46);
        d.println(F("< 0.5 is acceptable"));

        Serial.print(F("Results — Accuracy: "));
        Serial.print(resultAccuracy_, 3);
        Serial.println(F(" deg"));

        d.setCursor(0, 94);
        d.println(F("Hold UP+DOWN: Save"));
        d.setCursor(0, 106);
        d.println(F("Hold DOWN: Discard"));
    }

    d.display();

    Serial.println(F("Hold UP+DOWN to SAVE"));
    Serial.println(F("Hold DOWN to DISCARD"));
}

void CalibrationMode::showSavingScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(10, 50);
    d.print(F("Saving..."));
    d.display();
}

// ── Beep control ────────────────────────────────────────────────────

void CalibrationMode::beep() {
    // Match normal measurement success beep pattern
    delay(25);
    laser_->setBuzzer(true);
    delay(100);
    laser_->setBuzzer(false);
    delay(25);
}

void CalibrationMode::beepTriple() {
    for (int i = 0; i < 3; i++) {
        laser_->setBuzzer(true);
        delay(100);
    }
}

void CalibrationMode::updateBeep() {
    if (beepActive_ && millis() >= beepEndTime_) {
        beepActive_ = false;
        // Buzzer auto-stops on Egismos, but explicit off is harmless
    }
    // Laser wibble: turn laser back on after 500ms off
    if (laserWibbleActive_ && millis() >= laserOnTime_) {
        laser_->setLaser(true);
        laserWibbleActive_ = false;
    }
}

// ── Calculation ─────────────────────────────────────────────────────

void CalibrationMode::calculateEllipsoid() {
    Serial.println(F("Running ellipsoid fit..."));
    uint32_t t0 = millis();

    // Create fresh calibration with correct axis mappings
    *cal_ = MagCal::Calibration(MAG_AXES, GRAV_AXES);

    auto result = cal_->fitEllipsoid(magArray_, gravArray_);
    resultMagAcc_ = result.first;
    resultGravAcc_ = result.second;

    if (ellipsoidFitFailed()) {
        Serial.println(F("ERROR: ellipsoid fit degenerate — point coverage too poor"));
        return;
    }

    // Set field characteristics for anomaly detection
    cal_->setFieldCharacteristics(magArray_, gravArray_);

    uint32_t dt = millis() - t0;
    Serial.print(F("Ellipsoid fit done in "));
    Serial.print(dt);
    Serial.println(F(" ms"));
    Serial.print(F("  Mag uniformity:  "));
    Serial.println(resultMagAcc_, 4);
    Serial.print(F("  Grav uniformity: "));
    Serial.println(resultGravAcc_, 4);
}

void CalibrationMode::calculateAlignment() {
    Serial.println(F("Running alignment fit..."));
    uint32_t t0 = millis();

    // Alignment refines existing calibration — cal_ should already be loaded

    // Find similar shots (runs of aligned readings)
    auto runs = cal_->findSimilarShots(magArray_, gravArray_);
    Serial.print(F("  Found "));
    Serial.print(runs.size());
    Serial.println(F(" aligned run(s)"));

    if (runs.empty()) {
        Serial.println(F("  ERROR: No aligned runs found!"));
        resultAccuracy_ = 999.0f;
        return;
    }

    // Build paired data from runs
    MagCal::PairedData pairedData;
    for (const auto &run : runs) {
        std::vector<Eigen::Vector3f> magSub(magArray_.begin() + run.first, magArray_.begin() + run.second);
        std::vector<Eigen::Vector3f> gravSub(gravArray_.begin() + run.first, gravArray_.begin() + run.second);
        pairedData.push_back({magSub, gravSub});
    }

    // Step 1: Fit to axis
    float accAfterAxis = cal_->fitToAxis(pairedData, 'Y');
    Serial.print(F("  After axis fit: "));
    Serial.print(accAfterAxis, 3);
    Serial.println(F(" deg"));

    // Step 2: Non-linear quick (5 params, magnetometer only)
    float accAfterNL = cal_->fitNonLinearQuick(pairedData, 5);
    Serial.print(F("  After non-linear: "));
    Serial.print(accAfterNL, 3);
    Serial.println(F(" deg"));

    // Step 3: Roll alignment
    cal_->alignSensorRoll(magArray_, gravArray_);

    // Final accuracy
    resultAccuracy_ = cal_->accuracy(pairedData);
    Serial.print(F("  Final accuracy: "));
    Serial.print(resultAccuracy_, 3);
    Serial.println(F(" deg"));

    // Update field characteristics
    cal_->setFieldCharacteristics(magArray_, gravArray_);

    uint32_t dt = millis() - t0;
    Serial.print(F("Alignment fit done in "));
    Serial.print(dt);
    Serial.println(F(" ms"));
}

// ── F/B Check: Initialization ────────────────────────────────────

void CalibrationMode::beginFBCheck(ButtonManager &btns, DisplayManager &disp, DiscoManager &disco,
                                   LaserManager &laser, RM3100 &magSensor, SCA3300 &accel,
                                   ConfigManager &cfgMgr, MagCal::Calibration &cal, const Config &config) {
    btns_ = &btns;
    disp_ = &disp;
    disco_ = &disco;
    laser_ = &laser;
    magSensor_ = &magSensor;
    accel_ = &accel;
    cfgMgr_ = &cfgMgr;
    cal_ = &cal;

    // Apply consistency settings from config
    bufferLen_ = min((int)config.calBufferLength, MAX_BUFFER_SIZE);
    magThreshold_ = config.calMagConsistency;
    gravThreshold_ = config.calGravConsistency;
    settleMs_ = config.calSettleMs;
    calEmaAlpha_ = config.calEmaAlpha;
    calTimeoutMs_ = config.calTimeoutMs;

    // Store config values for stability/leg checks
    fbStabilityTol_ = config.stabilityTolerance;
    fbLegAngleTol_ = config.legAngleTolerance;
    fbLaserWibble_ = config.laserWibble;

    // Clear FB data
    fbCount_ = 0;
    fbHasForesight_ = false;
    fbCurrentFwd_ = 0.0f;
    fbAmplitude_ = 0.0f;
    holdCounter_ = 0.0f;
    bufferCount_ = 0;
    bufferIdx_ = 0;
    emaInitialized_ = false;
    waitingForStable_ = false;
    settleStart_ = 0;
    accumMag_ = Eigen::Vector3f::Zero();
    accumGrav_ = Eigen::Vector3f::Zero();
    accumCount_ = 0;
    beepActive_ = false;

    // Clear leg-style shot buffers
    fbStabHead_ = 0;
    fbStabCount_ = 0;
    fbLegCount_ = 0;
    fbTakingShot_ = false;
    fbLaserOn_ = true;
    fbCurrentBearing_ = 0.0f;
    fbLastDisplayTime_ = 0;
    fbEmaAz_ = 0.0f;
    fbEmaAlpha_ = config.emaAlphaMoving;
    fbEmaSeeded_ = false;

    // Turn on laser
    laser_->setLaser(true);

    // Same radio quiesce as begin() — the F/B save writes flash too
    bleRadioQuiet(true);

    Serial.println(F("F/B field check mode started"));
    state_ = CalibState::FB_INTRO;
    showFBIntroScreen();
}

// ── F/B Check: State handlers ───────────────────────────────────

void CalibrationMode::updateFBIntro() {
    if (btns_->wasPressed(Button::FIRE) || btns_->wasPressed(Button::UP_DISCO) ||
        btns_->wasPressed(Button::DOWN) || btns_->wasPressed(Button::MENU)) {

        state_ = CalibState::FB_WAIT_FORESIGHT;
        fbHasForesight_ = false;
        fbTakingShot_ = false;
        fbStabHead_ = 0;
        fbStabCount_ = 0;
        fbLegCount_ = 0;
        showFBLiveScreen();
        Serial.println(F("FB: waiting for foresight shot 1"));
    }
}

void CalibrationMode::updateFBWaitShot() {
    // Sample sensors at ~100 Hz and compute bearing
    uint32_t now = millis();
    if (now - lastSampleTime_ < 10) {
        return;
    }
    lastSampleTime_ = now;

    Eigen::Vector3f magReading, gravReading;
    readSensors(magReading, gravReading);
    float rawBearing = getBearing(magReading, gravReading);

    // Circular EMA smoothing (matches SensorManager pipeline)
    if (!fbEmaSeeded_) {
        fbEmaAz_ = rawBearing;
        fbEmaSeeded_ = true;
    } else {
        float diff = wrapTo180(rawBearing - fbEmaAz_);
        fbEmaAz_ = wrapTo360(fbEmaAz_ + fbEmaAlpha_ * diff);
    }
    fbCurrentBearing_ = fbEmaAz_;

    // Push EMA-smoothed bearing into stability ring buffer
    fbStabBuf_[fbStabHead_] = fbCurrentBearing_;
    fbStabHead_ = (fbStabHead_ + 1) % FB_STAB_LEN;
    if (fbStabCount_ < FB_STAB_LEN) {
        fbStabCount_++;
    }

    // Refresh display every 250ms
    if (now - fbLastDisplayTime_ >= 250) {
        fbLastDisplayTime_ = now;
        showFBLiveScreen();
    }

    // FIRE button: if laser is off, first press just turns it on;
    // next press starts capture so user can aim before taking a shot
    if (btns_->wasPressed(Button::FIRE) && !fbTakingShot_) {
        if (!fbLaserOn_) {
            // Re-enable laser with beep (same as main measurement flow)
            laser_->setBuzzer(true);
            delay(25);
            laser_->setLaser(true);
            delay(200);
            laser_->setBuzzer(false);
            delay(25);
            fbLaserOn_ = true;
        } else {
            fbTakingShot_ = true;
            fbStabHead_ = 0;
            fbStabCount_ = 0;
            disco_->setRed();
            // Entry click buzzer (same as main measurement flow)
            laser_->setBuzzer(true);
            delay(100);
            laser_->setBuzzer(false);
            delay(25);
        }
    }

    // UP_DISCO button: finish collection early (≥2 pairs, not mid-foresight)
    if (btns_->wasPressed(Button::UP_DISCO) && fbCount_ >= 2 && !fbHasForesight_) {
        Serial.println(F("FB: finishing collection early"));
        state_ = CalibState::FB_CALCULATING;
        return;
    }

    // Check bearing stability when taking a shot
    if (fbTakingShot_ && fbBearingStable(fbStabilityTol_)) {
        // Stable reading — green LED + success beep (same as main measurement)
        float bearing = fbCurrentBearing_;
        fbTakingShot_ = false;
        disco_->setGreen();
        delay(25);
        laser_->setBuzzer(true);
        delay(100);
        laser_->setBuzzer(false);
        delay(25);

        // Laser off after shot (same as main measurement)
        laser_->setLaser(false);
        fbLaserOn_ = false;
        disco_->turnOff();

        Serial.print(F("FB shot: "));
        Serial.print(bearing, 1);
        Serial.print(F("  leg "));
        Serial.print(fbLegCount_ + 1);
        Serial.println(F("/3"));

        // Push to leg consistency buffer (sliding window)
        if (fbLegCount_ < FB_LEG_LEN) {
            fbLegBuf_[fbLegCount_++] = bearing;
        } else {
            fbLegBuf_[0] = fbLegBuf_[1];
            fbLegBuf_[1] = fbLegBuf_[2];
            fbLegBuf_[2] = bearing;
        }

        // Check leg completion (3 consistent bearings)
        if (fbLegCount_ >= FB_LEG_LEN) {
            bool azOk = true;
            for (int i = 0; i < FB_LEG_LEN && azOk; i++) {
                for (int j = i + 1; j < FB_LEG_LEN && azOk; j++) {
                    if (circularDiff(fbLegBuf_[i], fbLegBuf_[j]) > fbLegAngleTol_) {
                        azOk = false;
                    }
                }
            }

            if (azOk) {
                // LEG COMPLETE — triple buzz + white flash + wibble + purple
                // (same sequence as main measurement flow)
                float finalBearing = fbCircularAverage(fbLegBuf_, FB_LEG_LEN);

                // Compute spread (max circular diff among the 3 legs)
                float spread = 0.0f;
                for (int i = 0; i < FB_LEG_LEN; i++) {
                    for (int j = i + 1; j < FB_LEG_LEN; j++) {
                        spread = max(spread, circularDiff(fbLegBuf_[i], fbLegBuf_[j]));
                    }
                }

                fbLegCount_ = 0;

                // Triple buzz + white flash
                for (int i = 0; i < 3; i++) {
                    laser_->setBuzzer(true);
                    disco_->setWhite();
                    delay(100);
                    laser_->setBuzzer(false);
                    disco_->turnOff();
                    delay(100);
                }

                // Laser wibble to indicate leg detected
                if (fbLaserWibble_) {
                    laser_->wibble();
                }

                // Laser fully off after leg
                laser_->setLaser(false);
                fbLaserOn_ = false;

                disco_->setPurple();

                if (state_ == CalibState::FB_WAIT_FORESIGHT) {
                    // Foresight accepted
                    fbCurrentFwd_ = finalBearing;
                    fbCurrentFwdSpread_ = spread;
                    fbHasForesight_ = true;
                    Serial.print(F("FB foresight accepted: "));
                    Serial.println(finalBearing, 1);

                    state_ = CalibState::FB_WAIT_BACKSIGHT;
                    fbStabHead_ = 0;
                    fbStabCount_ = 0;
                } else {
                    // Backsight accepted — pair complete
                    fbFwd_[fbCount_] = fbCurrentFwd_;
                    fbBwd_[fbCount_] = finalBearing;
                    fbFwdSpread_[fbCount_] = fbCurrentFwdSpread_;
                    fbBwdSpread_[fbCount_] = spread;
                    fbCount_++;
                    fbHasForesight_ = false;

                    Serial.print(F("FB backsight accepted: "));
                    Serial.print(finalBearing, 1);
                    Serial.print(F("  pair "));
                    Serial.print(fbCount_);
                    Serial.println(F(" complete"));

                    // Show pair error — wait for button press
                    float pairError = wrapTo180(fbCurrentFwd_ - finalBearing - 180.0f) / 2.0f;
                    showFBPairResult(pairError, fbCurrentFwd_);
                    state_ = CalibState::FB_PAIR_RESULT;
                }
            }
        }
    }
}

void CalibrationMode::updateFBPairResult() {
    // Wait for any button press to continue
    if (btns_->wasPressed(Button::FIRE) || btns_->wasPressed(Button::UP_DISCO) ||
        btns_->wasPressed(Button::DOWN) || btns_->wasPressed(Button::MENU)) {

        disco_->turnOff();

        if (fbCount_ >= FB_MAX_PAIRS) {
            Serial.println(F("FB: max pairs reached, calculating"));
            state_ = CalibState::FB_CALCULATING;
        } else {
            // Turn laser back on so user can aim, but don't start capture
            laser_->setLaser(true);
            fbLaserOn_ = true;
            state_ = CalibState::FB_WAIT_FORESIGHT;
            fbStabHead_ = 0;
            fbStabCount_ = 0;
        }
    }
}

void CalibrationMode::updateFBCalculating() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(0, 50);
    d.print(F("Calculating..."));
    d.display();

    fbAmplitude_ = cal_->applyFBCorrection(fbFwd_, fbBwd_, fbCount_);

    state_ = CalibState::FB_RESULTS;
    showFBResultsScreen();
}

void CalibrationMode::updateFBResults() {
    // Hold DOWN to exit (correction is display-only for now, not applied)
    bool down = btns_->isPressed(Button::DOWN);

    if (down) {
        holdCounter_ += 0.01f;
        if (holdCounter_ >= HOLD_TIME) {
            Serial.println(F("FB: discarded — rebooting to restore saved cal."));
            auto &d = disp_->getDisplay();
            d.clearDisplay();
            d.setTextColor(SH110X_WHITE);
            d.setTextSize(2);
            d.setCursor(0, 40);
            d.println(F("Discarded."));
            d.setCursor(0, 70);
            d.print(F("Restarting..."));
            d.display();
            delay(2000);
            NVIC_SystemReset();
        }
    } else {
        holdCounter_ = 0.0f;
    }
}

void CalibrationMode::updateFBSaving() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(10, 50);
    d.print(F("Saving..."));
    d.display();

    if (saveCalibration()) {
        Serial.println(F("FB: corrected calibration saved."));
        d.clearDisplay();
        d.setTextColor(SH110X_WHITE);
        d.setTextSize(2);
        d.setCursor(20, 50);
        d.print(F("Saved!"));
        d.display();
        delay(1500);
    } else {
        Serial.println(F("FB: save FAILED!"));
        d.clearDisplay();
        d.setTextColor(SH110X_WHITE);
        d.setTextSize(2);
        d.setCursor(10, 40);
        d.print(F("Save FAIL"));
        d.setTextSize(1);
        d.setCursor(0, 70);
        d.println(F("Storage error - kept"));
        d.println(F("in RAM. Retry save"));
        d.println(F("or discard."));
        d.display();
        delay(3000);

        // Keep the corrected calibration in RAM and return to the results
        // screen — rebooting here would discard the F/B correction.
        holdCounter_ = 0.0f;
        state_ = CalibState::FB_RESULTS;
        showFBResultsScreen();
        return;
    }

    // Reboot to cleanly load corrected calibration
    auto &d2 = disp_->getDisplay();
    d2.clearDisplay();
    d2.setTextColor(SH110X_WHITE);
    d2.setTextSize(2);
    d2.setCursor(0, 50);
    d2.print(F("Restarting..."));
    d2.display();
    Serial.println(F("Rebooting to apply corrected calibration..."));
    delay(2000);
    NVIC_SystemReset();
}

// ── F/B Check: Display helpers ──────────────────────────────────

void CalibrationMode::showFBIntroScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Mag Field"));
    d.println(F("Check"));

    d.setTextSize(1);
    d.setCursor(0, 44);
    d.println(F("Shoot a target, walk"));
    d.println(F("to it, shoot back."));
    d.println(F("Repeat 3+ times in"));
    d.println(F("different directions."));

    d.setCursor(0, 110);
    d.print(F("Press any button..."));

    d.display();
}

void CalibrationMode::showFBLiveScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    char buf[24];

    // Title + state
    d.setTextSize(2);
    d.setCursor(0, 0);
    if (state_ == CalibState::FB_WAIT_FORESIGHT) {
        snprintf(buf, sizeof(buf), "Fwd #%d", fbCount_ + 1);
    } else {
        snprintf(buf, sizeof(buf), "Bwd #%d", fbCount_ + 1);
    }
    d.println(buf);

    // Live bearing (large)
    d.setTextSize(3);
    d.setCursor(0, 28);
    snprintf(buf, sizeof(buf), "%.1f", (double)fbCurrentBearing_);
    d.print(buf);
    d.setTextSize(1);
    d.print(F(" deg"));

    // Shot progress
    d.setTextSize(1);
    d.setCursor(0, 60);
    snprintf(buf, sizeof(buf), "Shots: %d/3", min(fbLegCount_, 3));
    d.println(buf);

    // Foresight bearing if recorded
    if (fbHasForesight_) {
        d.setCursor(0, 72);
        snprintf(buf, sizeof(buf), "Fwd: %.1f deg", (double)fbCurrentFwd_);
        d.println(buf);
    }

    // Pairs completed
    d.setCursor(0, 84);
    snprintf(buf, sizeof(buf), "Pairs: %d", fbCount_);
    d.println(buf);

    // Status/hints
    d.setCursor(0, 100);
    if (fbTakingShot_) {
        d.println(F("Hold steady..."));
    } else if (fbCount_ >= 2 && !fbHasForesight_) {
        d.println(F("B1:shoot  B2:finish"));
    } else {
        d.println(F("Press B1 to shoot"));
    }

    d.display();
}

void CalibrationMode::showFBPairResult(float error, float bearing) {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    char buf[32];

    d.setTextSize(2);
    d.setCursor(0, 10);
    snprintf(buf, sizeof(buf), "Pair %d", fbCount_);
    d.println(buf);

    d.setTextSize(2);
    d.setCursor(0, 40);
    d.println(F("Error:"));
    d.setCursor(0, 60);
    snprintf(buf, sizeof(buf), "%.1f deg", (double)error);
    d.println(buf);

    d.setTextSize(1);
    d.setCursor(0, 90);
    snprintf(buf, sizeof(buf), "@ bearing %.0f", (double)bearing);
    d.println(buf);

    d.display();
}

void CalibrationMode::showFBResultsScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Mag Check"));

    char buf[32];

    d.setTextSize(1);
    d.setCursor(0, 28);
    snprintf(buf, sizeof(buf), "Pairs: %d", fbCount_);
    d.println(buf);

    d.setCursor(0, 42);
    snprintf(buf, sizeof(buf), "Correction: %.1f", (double)fbAmplitude_);
    d.println(buf);

    // Show per-pair: error and sensor spread (noise)
    d.setCursor(0, 56);
    for (int i = 0; i < fbCount_ && i < 5; i++) {
        float err = wrapTo180(fbFwd_[i] - fbBwd_[i] - 180.0f) / 2.0f;
        float maxSpread = max(fbFwdSpread_[i], fbBwdSpread_[i]);
        snprintf(buf, sizeof(buf), "%d: err:%.1f sprd:%.1f", i + 1, (double)err, (double)maxSpread);
        d.println(buf);
    }

    // Exit hint
    int yHint = 56 + min(fbCount_, 5) * 10 + 6;
    if (yHint > 100) {
        yHint = 100;
    }
    d.setCursor(0, yHint);
    d.println(F("Hold DOWN: Discard"));

    d.display();

    Serial.print(F("FB result: amplitude = "));
    Serial.print(fbAmplitude_, 2);
    Serial.print(F(" deg from "));
    Serial.print(fbCount_);
    Serial.println(F(" pairs"));
}

// ── F/B Check: Bearing helper ───────────────────────────────────

float CalibrationMode::getBearing(const Eigen::Vector3f &mag, const Eigen::Vector3f &grav) {
    MagCal::Angles angles = cal_->getAngles(mag, grav);
    return angles.azimuth;
}

bool CalibrationMode::fbBearingStable(float tolerance) const {
    if (fbStabCount_ < FB_STAB_LEN) {
        return false;
    }
    for (int i = 0; i < FB_STAB_LEN; i++) {
        for (int j = i + 1; j < FB_STAB_LEN; j++) {
            if (circularDiff(fbStabBuf_[i], fbStabBuf_[j]) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

float CalibrationMode::fbCircularAverage(const float *buf, int count) const {
    if (count <= 0) {
        return 0.0f;
    }
    // Use first element as reference to handle wraparound
    float ref = buf[0];
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += wrapTo180(buf[i] - ref);
    }
    return wrapTo360(ref + sum / count);
}

// ── Shutdown confirmation ────────────────────────────────────────

void CalibrationMode::requestShutdown() {
    if (state_ == CalibState::SHUTDOWN_CONFIRM) {
        // SHUTDOWN pressed again while confirm screen is showing — reset hold timer
        // so the user needs a deliberate continuous hold from scratch.
        shutdownHoldStart_ = 0;
        return;
    }
    Serial.println(F("Calibration: shutdown requested, showing confirm screen"));
    preShutdownState_ = state_;
    state_ = CalibState::SHUTDOWN_CONFIRM;
    shutdownHoldStart_ = 0;
    shutdownLastDisplayMs_ = 0;
    showShutdownConfirmScreen();
}

void CalibrationMode::updateShutdownConfirm() {
    // V2: unreachable — power-off is the hardware power button (LTC2954),
    // handled globally in loop(). If we somehow land here, bounce back.
    shutdownHoldStart_ = 0;
    state_ = preShutdownState_;
    redrawScreen(state_);
}

void CalibrationMode::showShutdownConfirmScreen(uint8_t holdPct) {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Power Off?"));

    d.setTextSize(1);
    d.setCursor(0, 36);
    d.println(F("Calibration progress"));
    d.println(F("will be lost."));
    d.println();
    d.println(F("Hold SHUTDOWN to"));
    d.println(F("power off."));
    d.println();
    d.println(F("Any other button"));
    d.println(F("to cancel."));

    // Progress bar (only visible while holding)
    if (holdPct > 0) {
        const int barX = 0, barY = 118, barW = 128, barH = 8;
        d.drawRect(barX, barY, barW, barH, SH110X_WHITE);
        int fillW = (int)(barW * holdPct / 100);
        if (fillW > 2) {
            d.fillRect(barX + 1, barY + 1, fillW - 2, barH - 2, SH110X_WHITE);
        }
    }

    d.display();
}

void CalibrationMode::redrawScreen(CalibState s) {
    switch (s) {
    case CalibState::INTRO_ELLIPSOID:
        showEllipsoidIntro();
        break;
    case CalibState::INTRO_ALIGNMENT:
        showAlignmentIntro();
        break;
    case CalibState::COLLECTING_ELLIPSOID:
        showEllipsoidScreen();
        break;
    case CalibState::COLLECTING_ALIGNMENT:
        showAlignmentProgress();
        break;
    case CalibState::SHOW_RESULTS:
        showResultsScreen();
        break;
    case CalibState::FB_INTRO:
        showFBIntroScreen();
        break;
    case CalibState::FB_WAIT_FORESIGHT:
    case CalibState::FB_WAIT_BACKSIGHT:
        showFBLiveScreen();
        break;
    case CalibState::FB_RESULTS:
        showFBResultsScreen();
        break;
    default:
        break;
    }
}

// ── Calibration save ────────────────────────────────────────────

bool CalibrationMode::saveCalibration() {
    // Serialize calibration to JSON
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    cal_->toJson(root);

    // Pre-check required size before serializing — serializeJson silently
    // truncates when the buffer is too small and returns sizeof(buf)-1, so
    // the naive "len >= sizeof(buf)" check never fires on overflow.
    size_t required = measureJson(doc);
    if (required == 0) {
        Serial.println(F("JSON serialization failed (empty)"));
        return false;
    }

    static constexpr size_t JSON_BUF_SIZE = 2048;
    if (required >= JSON_BUF_SIZE) {
        Serial.print(F("JSON too large: "));
        Serial.print(required);
        Serial.println(F(" bytes"));
        return false;
    }

    // static: the loop task has a 4 KB stack and the LittleFS commit path
    // below this frame needs ~1.5 KB more — with these as locals the frame
    // was 2.5 KB and the deepest write (the metrics rename) overflowed the
    // stack, smashing littlefs's heap buffers so it committed CRC-valid
    // garbage metadata; the next boot then hung traversing it (bricked
    // device, 2026-07-22). Same fix as MenuManager::testCalSave.
    static char jsonBuf[JSON_BUF_SIZE];
    size_t len = serializeJson(doc, jsonBuf, JSON_BUF_SIZE);
    if (len != required) {
        Serial.println(F("JSON serialization error"));
        return false;
    }

    // Save JSON to flash (human-readable backup)
    bool jsonOk = cfgMgr_->saveCalibrationJson(jsonBuf, len);
    Serial.print(F("  JSON save: "));
    Serial.println(jsonOk ? F("OK") : F("FAILED"));
    Serial.print(F("  JSON size: "));
    Serial.print(len);
    Serial.println(F(" bytes"));

    // Save binary to flash (fast boot path)
    static MagCal::CalibrationBinary bin; // static — see stack note above
    cal_->toBinary(bin);
    bool binOk = cfgMgr_->saveCalibrationBinary(bin);
    Serial.print(F("  Binary save: "));
    Serial.println(binOk ? F("OK") : F("FAILED"));

    // Save quality metrics for "View Last Cal" menu
    ConfigManager::CalMetrics metrics = {resultMagAcc_, resultGravAcc_, resultAccuracy_};
    cfgMgr_->saveCalMetrics(metrics);

    return jsonOk && binOk;
}
