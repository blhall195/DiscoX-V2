#include "calibration_mode.h"
#include "math_utils.h"
#include "shot_vector.h"
#include "feedback.h"
#include "sounds.h"
#include <algorithm>
#include <math.h>

namespace {
// Median and robust sigma (1.4826 x MAD) of x over the kept entries
void robustStats(const std::vector<float> &x, const std::vector<bool> &keep, float &med, float &sigma) {
    std::vector<float> v;
    for (size_t i = 0; i < x.size(); i++) {
        if (keep[i]) {
            v.push_back(x[i]);
        }
    }
    auto median = [](std::vector<float> w) {
        size_t n = w.size() / 2;
        std::nth_element(w.begin(), w.begin() + n, w.end());
        return w[n];
    };
    med = median(v);
    for (float &a : v) {
        a = fabsf(a - med);
    }
    sigma = 1.4826f * median(v);
}

float sdOf(const std::vector<float> &x) {
    double m = 0.0, s = 0.0;
    for (float a : x) {
        m += a;
    }
    m /= (double)x.size();
    for (float a : x) {
        s += (a - m) * (a - m);
    }
    return (float)sqrt(s / (double)x.size());
}

} // namespace

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
    case CalibState::FB_RESULTS:
        updateFBResults();
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
        Feedback::shotStart(*disco_); // as for a survey shot
    }

    // DOWN (as shown on screen) undoes the last recorded point. MENU is
    // deliberately inert while collecting — but still consume its edge:
    // wasPressed() clears the flag, and leaving it set would let the press
    // fire later, once calibration hands control back to the menu.
    bool undoPressed = btns_->wasPressed(Button::DOWN);
    (void)btns_->wasPressed(Button::MENU);

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
            // Timeout — the device never held still long enough. This used to
            // record the EMA reading anyway, feeding an unsettled point to the
            // fit unmarked; now the attempt is dropped and the user retries.
            if (calTimeoutMs_ > 0 && (now - captureStart_) >= calTimeoutMs_) {
                Serial.println(F("Stability timeout — point not taken, press FIRE to retry"));
                waitingForStable_ = false;
                settleStart_ = 0;
                Feedback::failed(*disco_); // as for a failed laser shot
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
    Feedback::readingOk(*disco_); // as for a survey reading

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

    disco_->turnOff(); // green ends with the reading, as in survey use

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
    d.print(F("FIRE:record DOWN:undo"));

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
    d.print(F("FIRE:record DOWN:undo"));

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
        d.setCursor(0, 52);
        snprintf(buf, sizeof(buf), "Dip spread: %.1f deg", (double)resultDipSpread_);
        d.println(buf);
        if (rejectedCount_ > 0) {
            d.setCursor(0, 62);
            snprintf(buf, sizeof(buf), "Dropped %d bad pt%s", rejectedCount_, rejectedCount_ == 1 ? "" : "s");
            d.println(buf);
        }

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
        if (envWarn_) {
            // Accuracy alone can't see this — say it in words
            d.setCursor(0, 60);
            d.println(F("! Field differs from"));
            d.println(F("  Part 1: metal near?"));
            d.println(F("  Redo both parts in"));
            d.println(F("  one clear spot."));
        } else {
            d.setCursor(0, 60);
            snprintf(buf, sizeof(buf), "Dip spread: %.1f deg", (double)resultDipSpread_);
            d.println(buf);
        }

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
    // Calibration-only cues (undo, Part 2 "change direction"): a click.
    // Shot-like events use Feedback:: so they match survey use.
    Sounds::click();
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

    rejectEllipsoidOutliers();

    // Set field characteristics for anomaly detection
    cal_->setFieldCharacteristics(magArray_, gravArray_);

    std::vector<float> dipsDeg(magArray_.size());
    for (size_t i = 0; i < magArray_.size(); i++) {
        dipsDeg[i] = cal_->getDip(magArray_[i], gravArray_[i]);
    }
    resultDipSpread_ = sdOf(dipsDeg);
    Serial.print(F("  Dip spread: "));
    Serial.print(resultDipSpread_, 2);
    Serial.print(F(" deg, points dropped: "));
    Serial.println(rejectedCount_);

    uint32_t dt = millis() - t0;

    Serial.print(F("Ellipsoid fit done in "));
    Serial.print(dt);
    Serial.println(F(" ms"));
    Serial.print(F("  Mag uniformity:  "));
    Serial.println(resultMagAcc_, 4);
    Serial.print(F("  Grav uniformity: "));
    Serial.println(resultGravAcc_, 4);
}

// Drop Part 1 points that disagree with the rest and refit. A point is scored
// on two physical invariants: its distance from the fitted unit sphere (field
// strength) and its dip angle (field-to-gravity angle), each against the
// median and robust spread of this calibration's own points — so it adapts to
// whatever the local field is, anywhere in the world. Thresholds were tuned
// offline on the 2026-09-26 logs with injected disturbances: no clean point
// dropped, and 5-10% disturbances cut from ~1.8 deg to ~0.3 deg worst-case
// azimuth error. Never drops more than 15% (coverage), and keeps the plain
// fit if a refit degenerates.
void CalibrationMode::rejectEllipsoidOutliers() {
    constexpr float Z_MAX = 4.0f;
    constexpr float RADIUS_FLOOR = 0.015f; // 1.5% of field strength
    constexpr float DIP_FLOOR = 2.0f;      // degrees
    const size_t n = magArray_.size();
    const size_t maxDrop = n * 15 / 100;

    std::vector<bool> keep(n, true);
    std::vector<float> radius(n), dip(n);
    rejectedCount_ = 0;

    for (int pass = 0; pass < 2; pass++) {
        for (size_t i = 0; i < n; i++) {
            radius[i] = cal_->mag().apply(magArray_[i]).norm() - 1.0f;
            dip[i] = cal_->getDip(magArray_[i], gravArray_[i]);
        }
        float rMed, rSig, dMed, dSig;
        robustStats(radius, keep, rMed, rSig);
        robustStats(dip, keep, dMed, dSig);
        float rLim = fmaxf(Z_MAX * rSig, RADIUS_FLOOR);
        float dLim = fmaxf(Z_MAX * dSig, DIP_FLOOR);

        // score > 1 = outlier; keep only the worst maxDrop if there are more
        std::vector<float> score(n);
        for (size_t i = 0; i < n; i++) {
            score[i] = fmaxf(fabsf(radius[i] - rMed) / rLim, fabsf(dip[i] - dMed) / dLim);
        }
        std::vector<size_t> order(n);
        for (size_t i = 0; i < n; i++) {
            order[i] = i;
        }
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return score[a] > score[b]; });
        std::vector<bool> newKeep(n, true);
        for (size_t k = 0; k < maxDrop && score[order[k]] > 1.0f; k++) {
            newKeep[order[k]] = false;
        }
        if (newKeep == keep) {
            break;
        }

        std::vector<Eigen::Vector3f> m, g;
        for (size_t i = 0; i < n; i++) {
            if (newKeep[i]) {
                m.push_back(magArray_[i]);
                g.push_back(gravArray_[i]);
            }
        }
        static MagCal::Calibration previous; // static: ~0.5 KB, 4 KB loop stack
        previous = *cal_;
        auto refit = cal_->fitEllipsoid(m, g);
        if (refit.first < 0.0f || refit.second < 0.0f) {
            *cal_ = previous; // refit degenerated — keep the fit we had
            break;
        }
        keep = newKeep;
        resultMagAcc_ = refit.first;
        resultGravAcc_ = refit.second;
    }

    std::vector<Eigen::Vector3f> m, g;
    for (size_t i = 0; i < n; i++) {
        if (keep[i]) {
            m.push_back(magArray_[i]);
            g.push_back(gravArray_[i]);
        } else {
            rejectedCount_++;
            Serial.print(F("  Outlier dropped: point "));
            Serial.print(i);
            Serial.print(F(" (field "));
            Serial.print(radius[i] * 100.0f, 2);
            Serial.print(F("%, dip "));
            Serial.print(dip[i], 2);
            Serial.println(F(" deg)"));
        }
    }
    magArray_ = m;
    gravArray_ = g;
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

    // Field references (strength + dip, for CAL? and the anomaly checks)
    // come from Part 1's 56 points spread over the whole sphere; Part 2's
    // 24 near-level shots used to overwrite them. Rotations don't change
    // either, so Part 1's values stay valid through the alignment. Only
    // fill them from Part 2 if the stored calibration has none.
    if (cal_->mag().fieldAvg() <= 0.0f || cal_->dipAvg() == 0.0f) {
        cal_->setFieldCharacteristics(magArray_, gravArray_);
    }

    // Environment check: Part 2 must see the same field as Part 1. A
    // 2026-09-26 calibration done half a metre from a radiator read 9%
    // stronger and 3 deg shallower in Part 2 while reporting a clean accuracy
    // figure. Relative to Part 1, so it holds anywhere in the world.
    std::vector<float> fields(magArray_.size()), dipsDeg(magArray_.size());
    for (size_t i = 0; i < magArray_.size(); i++) {
        fields[i] = cal_->mag().apply(magArray_[i]).norm();
        dipsDeg[i] = cal_->getDip(magArray_[i], gravArray_[i]);
    }
    std::vector<bool> all(magArray_.size(), true);
    float fieldMed, dipMed, unused;
    robustStats(fields, all, fieldMed, unused);
    robustStats(dipsDeg, all, dipMed, unused);
    envFieldRatio_ = fieldMed; // Part 1 fits the field to the unit sphere
    envDipDev_ = (cal_->dipAvg() != 0.0f) ? dipMed - cal_->dipAvg() : 0.0f;
    envWarn_ = fabsf(envFieldRatio_ - 1.0f) > 0.03f || fabsf(envDipDev_) > 2.0f;
    resultDipSpread_ = sdOf(dipsDeg);
    Serial.print(F("  Part 2 vs Part 1: field x"));
    Serial.print(envFieldRatio_, 3);
    Serial.print(F(", dip "));
    Serial.print(envDipDev_, 2);
    Serial.print(F(" deg"));
    Serial.println(envWarn_ ? F("  — DIFFERENT ENVIRONMENT") : F(""));

    uint32_t dt = millis() - t0;

    Serial.print(F("Alignment fit done in "));
    Serial.print(dt);
    Serial.println(F(" ms"));
}

// ── F/B Check ───────────────────────────────────────────────────
// Foresight/backsight check: shoot a leg, walk to the target, shoot back. A
// good compass reads the two 180° apart and the inclinations cancel. Each
// pair's disagreement is shown, then the mean with a verdict. It never
// changes the calibration.
//
// It used to fit a sinusoidal residual hard-iron correction and apply it.
// On a clean outdoor calibration (2026-09-26) the residual that correction
// could fix was ~0.1° RMS (≤0.25° worst heading) — below hand-aiming noise,
// so a fit from a handful of pairs mostly modelled noise, and any other F/B
// error (laser/sensor misalignment, a car in the car park) would have been
// baked into the calibration as hard iron. Disturbed calibrations are now
// caught at calibration time instead (outlier rejection, Part 2 check).

namespace {
// Verdict thresholds on the mean absolute disagreement. Real F/B pairs also
// carry station-marking error — a few cm on a 5 m leg is already ~0.5° — so
// "Good" allows 1°; beyond 2° the instrument, not the stations, is the
// likely cause.
constexpr float FB_GOOD_DEG = 1.0f;
constexpr float FB_OK_DEG = 2.0f;
// Azimuth is ill-defined on near-vertical legs; they count for inclination only
constexpr float FB_STEEP_DEG = 70.0f;

// Vector mean of n directions (wrap- and near-vertical-safe)
void meanDirection(const float *az, const float *inc, int n, float &outAz, float &outInc) {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    for (int i = 0; i < n; i++) {
        float a = degreesToRadians(az[i]);
        float e = degreesToRadians(inc[i]);
        x += cosf(e) * cosf(a);
        y += cosf(e) * sinf(a);
        z += sinf(e);
    }
    float r = sqrtf(x * x + y * y + z * z);
    outAz = wrapTo360(radiansToDegrees(atan2f(y, x)));
    outInc = radiansToDegrees(asinf(fmaxf(-1.0f, fminf(1.0f, z / r))));
}

float angleBetweenDeg(float az1, float inc1, float az2, float inc2) {
    return radiansToDegrees(Shot(az1, inc1, 1.0f).angleTo(Shot(az2, inc2, 1.0f)));
}

// Signed disagreement of a pair: azimuth off 180°, and inclinations' sum
float pairAzError(float fwdAz, float bwdAz) { return wrapTo180(bwdAz - fwdAz - 180.0f); }
float pairIncError(float fwdInc, float bwdInc) { return fwdInc + bwdInc; }
bool pairAzValid(float fwdInc) { return fabsf(fwdInc) <= FB_STEEP_DEG; }

const char *fbVerdict(float meanAz, float meanInc) {
    float worst = fmaxf(meanAz, meanInc);
    if (worst <= FB_GOOD_DEG) {
        return "Good";
    }
    return (worst <= FB_OK_DEG) ? "OK" : "Recal"; // size-2 headline: 10 chars max
}
} // namespace

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

    fbSensor_.init(cal_, config.emaAlphaStable, config.emaAlphaMoving, config.stabilityBufferLength,
                   Defaults::emaJumpThreshold);
    fbSteadyTol_ = config.stabilityTolerance;
    fbLegAngleTol_ = config.legAngleTolerance;
    fbLaserWibble_ = config.laserWibble;

    fbCount_ = 0;
    fbHasForesight_ = false;
    fbLegCount_ = 0;
    fbTakingShot_ = false;
    fbLaserOn_ = true;
    fbLastDisplayTime_ = 0;
    holdCounter_ = 0.0f;
    beepActive_ = false;

    laser_->setLaser(true);

    // Calibration modes silence the radio; main resumes it when we finish
    bleRadioQuiet(true);

    Serial.println(F("F/B check started"));
    state_ = CalibState::FB_INTRO;
    showFBIntroScreen();
}

void CalibrationMode::updateFBIntro() {
    if (btns_->wasPressed(Button::FIRE) || btns_->wasPressed(Button::UP_DISCO) ||
        btns_->wasPressed(Button::DOWN) || btns_->wasPressed(Button::MENU)) {
        state_ = CalibState::FB_WAIT_FORESIGHT;
        fbHasForesight_ = false;
        fbTakingShot_ = false;
        fbLegCount_ = 0;
        showFBLiveScreen();
        Serial.println(F("FB: waiting for foresight 1"));
    }
}

void CalibrationMode::updateFBWaitShot() {
    uint32_t now = millis();
    if (now - lastSampleTime_ < 10) {
        return;
    }
    lastSampleTime_ = now;

    Eigen::Vector3f magReading, gravReading;
    readSensors(magReading, gravReading);
    fbSensor_.update(magReading, gravReading, false);
    fbCurrentBearing_ = fbSensor_.getAzimuth();
    fbCurrentInc_ = fbSensor_.getInclination();

    if (now - fbLastDisplayTime_ >= 250) {
        fbLastDisplayTime_ = now;
        showFBLiveScreen();
    }

    // FIRE: first press wakes the laser to aim, the next takes a shot —
    // the survey two-press flow, with the survey feedback (Feedback::)
    if (btns_->wasPressed(Button::FIRE) && !fbTakingShot_) {
        if (!fbLaserOn_) {
            disco_->turnOff(); // wake is silent and clears purple, as in survey use
            laser_->setLaser(true);
            fbLaserOn_ = true;
        } else {
            fbTakingShot_ = true;
            Feedback::shotStart(*disco_);
        }
    }

    // UP: finish (at least one pair, not between foresight and backsight)
    if (btns_->wasPressed(Button::UP_DISCO) && fbCount_ >= 1 && !fbHasForesight_) {
        Serial.println(F("FB: finished"));
        disco_->turnOff();
        state_ = CalibState::FB_RESULTS;
        showFBResultsScreen();
        return;
    }

    float az, inc;
    if (!fbTakingShot_ || !fbSensor_.stableAverage(fbSteadyTol_, az, inc)) {
        return;
    }

    // Steady — shot taken
    fbTakingShot_ = false;
    Serial.print(F("FB shot: az "));
    Serial.print(az, 2);
    Serial.print(F(" inc "));
    Serial.println(inc, 2);

    // Sliding window of the last three shots
    if (fbLegCount_ < FB_LEG_LEN) {
        fbLegAz_[fbLegCount_] = az;
        fbLegInc_[fbLegCount_] = inc;
        fbLegCount_++;
    } else {
        for (int i = 0; i < FB_LEG_LEN - 1; i++) {
            fbLegAz_[i] = fbLegAz_[i + 1];
            fbLegInc_[i] = fbLegInc_[i + 1];
        }
        fbLegAz_[FB_LEG_LEN - 1] = az;
        fbLegInc_[FB_LEG_LEN - 1] = inc;
    }
    bool legDone = fbLegCount_ >= FB_LEG_LEN;
    for (int i = 0; legDone && i < FB_LEG_LEN; i++) {
        for (int j = i + 1; legDone && j < FB_LEG_LEN; j++) {
            legDone = angleBetweenDeg(fbLegAz_[i], fbLegInc_[i], fbLegAz_[j], fbLegInc_[j]) <= fbLegAngleTol_;
        }
    }

    // As in survey use: a lone reading bleeps green and ends with the laser
    // and LED off; a completed leg plays the fanfare instead and stays purple
    if (!legDone) {
        Feedback::readingOk(*disco_);
        laser_->setLaser(false);
        fbLaserOn_ = false;
        disco_->turnOff();
        return;
    }

    float legAz, legInc;
    meanDirection(fbLegAz_, fbLegInc_, FB_LEG_LEN, legAz, legInc);
    fbLegCount_ = 0;
    disco_->setGreen(); // the reading itself, as a survey shot shows it
    Feedback::legComplete(*disco_, *laser_, fbLaserWibble_);
    laser_->setLaser(false);
    fbLaserOn_ = false;

    if (state_ == CalibState::FB_WAIT_FORESIGHT) {
        fbCurrentFwdAz_ = legAz;
        fbCurrentFwdInc_ = legInc;
        fbHasForesight_ = true;
        state_ = CalibState::FB_WAIT_BACKSIGHT;
        Serial.print(F("FB foresight: "));
        Serial.print(legAz, 2);
        Serial.print(F(" / "));
        Serial.println(legInc, 2);
        return;
    }

    fbFwdAz_[fbCount_] = fbCurrentFwdAz_;
    fbFwdInc_[fbCount_] = fbCurrentFwdInc_;
    fbBwdAz_[fbCount_] = legAz;
    fbBwdInc_[fbCount_] = legInc;
    fbCount_++;
    fbHasForesight_ = false;
    Serial.print(F("FB pair "));
    Serial.print(fbCount_);
    Serial.print(F(": az error "));
    Serial.print(pairAzError(fbCurrentFwdAz_, legAz), 2);
    Serial.print(F(", inc error "));
    Serial.println(pairIncError(fbCurrentFwdInc_, legInc), 2);
    showFBPairResult();
    state_ = CalibState::FB_PAIR_RESULT;
}

void CalibrationMode::updateFBPairResult() {
    if (btns_->wasPressed(Button::FIRE) || btns_->wasPressed(Button::UP_DISCO) ||
        btns_->wasPressed(Button::DOWN) || btns_->wasPressed(Button::MENU)) {
        disco_->turnOff();
        if (fbCount_ >= FB_MAX_PAIRS) {
            state_ = CalibState::FB_RESULTS;
            showFBResultsScreen();
        } else {
            laser_->setLaser(true); // aim for the next foresight
            fbLaserOn_ = true;
            state_ = CalibState::FB_WAIT_FORESIGHT;
            showFBLiveScreen();
        }
    }
}

void CalibrationMode::updateFBResults() {
    // Hold DOWN to leave. Nothing was changed, so no save and no reboot.
    if (btns_->isPressed(Button::DOWN)) {
        holdCounter_ += 0.01f;
        if (holdCounter_ >= HOLD_TIME) {
            Serial.println(F("FB: check finished"));
            state_ = CalibState::DONE;
        }
    } else {
        holdCounter_ = 0.0f;
    }
}

// ── F/B Check: Display helpers ──────────────────────────────────

void CalibrationMode::showFBIntroScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("F/B Check"));

    d.setTextSize(1);
    d.setCursor(0, 28);
    d.println(F("Shoot a leg (3 shots)"));
    d.println(F("to a target, go there"));
    d.println(F("and shoot back to the"));
    d.println(F("start. Legs of 5 m+,"));
    d.println(F("a few directions."));
    d.println(F("Nothing is changed."));

    d.setCursor(0, 110);
    d.print(F("Press any button..."));
    d.display();
}

void CalibrationMode::showFBLiveScreen() {
    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    char buf[24];

    d.setTextSize(2);
    d.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "%s #%d", state_ == CalibState::FB_WAIT_FORESIGHT ? "Fwd" : "Back",
             fbCount_ + 1);
    d.println(buf);

    d.setTextSize(3);
    d.setCursor(0, 24);
    snprintf(buf, sizeof(buf), "%.1f", (double)fbCurrentBearing_);
    d.print(buf);
    d.setTextSize(1);
    d.setCursor(0, 50);
    snprintf(buf, sizeof(buf), "inc %+.1f", (double)fbCurrentInc_);
    d.println(buf);

    d.setCursor(0, 64);
    snprintf(buf, sizeof(buf), "Shots: %d/3", min(fbLegCount_, 3));
    d.println(buf);
    if (fbHasForesight_) {
        d.setCursor(0, 76);
        snprintf(buf, sizeof(buf), "Fwd: %.1f %+.1f", (double)fbCurrentFwdAz_, (double)fbCurrentFwdInc_);
        d.println(buf);
    }
    d.setCursor(0, 88);
    snprintf(buf, sizeof(buf), "Pairs: %d", fbCount_);
    d.println(buf);

    d.setCursor(0, 104);
    if (fbTakingShot_) {
        d.println(F("Hold steady..."));
    } else if (fbCount_ >= 1 && !fbHasForesight_) {
        d.println(F("FIRE:shoot  UP:finish"));
    } else {
        d.println(F("Press FIRE to shoot"));
    }
    d.display();
}

void CalibrationMode::showFBPairResult() {
    int i = fbCount_ - 1;
    float azErr = pairAzError(fbFwdAz_[i], fbBwdAz_[i]);
    float incErr = pairIncError(fbFwdInc_[i], fbBwdInc_[i]);
    bool azOk = pairAzValid(fbFwdInc_[i]);

    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    char buf[24];

    d.setTextSize(2);
    d.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "Pair %d", fbCount_);
    d.println(buf);

    d.setCursor(0, 30);
    if (azOk) {
        snprintf(buf, sizeof(buf), "Az %+.1f", (double)azErr);
    } else {
        snprintf(buf, sizeof(buf), "Az steep");
    }
    d.println(buf);
    d.setCursor(0, 52);
    snprintf(buf, sizeof(buf), "Inc %+.1f", (double)incErr);
    d.println(buf);

    d.setTextSize(1);
    d.setCursor(0, 80);
    snprintf(buf, sizeof(buf), "Leg at %.0f deg", (double)fbFwdAz_[i]);
    d.println(buf);
    d.setCursor(0, 104);
    d.println(F("Any button: next"));
    d.display();
}

void CalibrationMode::showFBResultsScreen() {
    float sumAz = 0.0f, sumInc = 0.0f;
    int nAz = 0;
    for (int i = 0; i < fbCount_; i++) {
        if (pairAzValid(fbFwdInc_[i])) {
            sumAz += fabsf(pairAzError(fbFwdAz_[i], fbBwdAz_[i]));
            nAz++;
        }
        sumInc += fabsf(pairIncError(fbFwdInc_[i], fbBwdInc_[i]));
    }
    float meanAz = nAz > 0 ? sumAz / nAz : 0.0f;
    float meanInc = fbCount_ > 0 ? sumInc / fbCount_ : 0.0f;
    const char *verdict = fbVerdict(meanAz, meanInc);

    auto &d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    char buf[24];

    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(verdict);

    d.setTextSize(1);
    d.setCursor(0, 22);
    snprintf(buf, sizeof(buf), "%d pair%s, mean error:", fbCount_, fbCount_ == 1 ? "" : "s");
    d.println(buf);
    snprintf(buf, sizeof(buf), " az %.1f  inc %.1f deg", (double)meanAz, (double)meanInc);
    d.println(buf);

    d.setCursor(0, 46);
    for (int i = 0; i < fbCount_ && i < 5; i++) {
        float incErr = pairIncError(fbFwdInc_[i], fbBwdInc_[i]);
        if (pairAzValid(fbFwdInc_[i])) {
            snprintf(buf, sizeof(buf), "%d: az%+.1f inc%+.1f", i + 1,
                     (double)pairAzError(fbFwdAz_[i], fbBwdAz_[i]), (double)incErr);
        } else {
            snprintf(buf, sizeof(buf), "%d: steep  inc%+.1f", i + 1, (double)incErr);
        }
        d.println(buf);
    }

    d.setCursor(0, 104);
    if (strcmp(verdict, "Recal") == 0) {
        d.println(F("Recal away from metal"));
    }
    d.setCursor(0, 116);
    d.print(F("Hold DOWN: exit"));
    d.display();

    Serial.print(F("FB result: "));
    Serial.print(verdict);
    Serial.print(F("  mean az "));
    Serial.print(meanAz, 2);
    Serial.print(F("  mean inc "));
    Serial.print(meanInc, 2);
    Serial.print(F("  pairs "));
    Serial.println(fbCount_);
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
