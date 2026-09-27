#pragma once
// Calibration Mode — on-device calibration data collection and processing
// Session 11: port of Python calibration_manager.py + calibration_mode.py
// Session 13: combined ellipsoid + alignment into single "long calibration"
// flow

#include "button_manager.h"
#include "config.h"
#include "config_manager.h"
#include "device_context.h"
#include "disco_manager.h"
#include "display_manager.h"
#include "drivers/sca3300.h"
#include "laser_manager.h"
#include "mag_cal/calibration.h"
#include "rm3100.h"
#include "sensor_manager.h"
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <vector>

/// Which calibration workflow to run
enum class CalMode : uint8_t {
    PART1_ELLIPSOID, // 56-pt ellipsoid only → results → save
    PART2_ALIGNMENT, // 24-pt alignment only (requires existing ellipsoid cal)
};

/// Calibration state machine states
enum class CalibState : uint8_t {
    INACTIVE,              // not in calibration mode
    INTRO_ELLIPSOID,       // showing ellipsoid instruction screen
    COLLECTING_ELLIPSOID,  // 56-point ellipsoid collection
    CALCULATING_ELLIPSOID, // running ellipsoid fitting math
    INTRO_ALIGNMENT,       // showing alignment instruction screen
    COLLECTING_ALIGNMENT,  // 24-point alignment collection (3 stages × 8)
    CALCULATING_ALIGNMENT, // running alignment fitting math
    SHOW_RESULTS,          // displaying accuracy, waiting for save/discard
    SAVING,                // writing calibration to flash
    FB_INTRO,              // F/B check: showing instructions
    FB_WAIT_FORESIGHT,     // F/B check: waiting for user to take foresight shot
    FB_WAIT_BACKSIGHT,     // F/B check: waiting for user to take backsight shot
    FB_PAIR_RESULT,        // F/B check: showing pair error, waiting for button press
    FB_RESULTS,            // F/B check: summary + verdict, hold DOWN to exit
    SHUTDOWN_CONFIRM,      // power-off confirmation screen (press again to confirm)
    DONE                   // finished, caller should exit calibration mode
};

class CalibrationMode {
  public:
    /// Initialize with references to all required peripherals.
    void begin(ButtonManager &btns, DisplayManager &disp, DiscoManager &disco, LaserManager &laser,
               RM3100 &magSensor, SCA3300 &accel, ConfigManager &cfgMgr, MagCal::Calibration &cal,
               const Config &config, CalMode mode = CalMode::PART1_ELLIPSOID);

    /// Enter the foresight/backsight check. Requires an existing calibration.
    /// Shows how far each foresight/backsight pair disagrees and gives a
    /// verdict; it never changes the calibration.
    void beginFBCheck(ButtonManager &btns, DisplayManager &disp, DiscoManager &disco, LaserManager &laser,
                      RM3100 &magSensor, SCA3300 &accel, ConfigManager &cfgMgr, MagCal::Calibration &cal,
                      const Config &config);

    /// Call every loop iteration. Returns true when calibration is complete.
    bool update();

    /// Called by main loop when the power button is pressed during calibration.
    /// First call shows the confirmation screen; second call executes shutdown.
    void requestShutdown();

    bool isActive() const { return state_ != CalibState::INACTIVE && state_ != CalibState::DONE; }
    CalibState state() const { return state_; }

  private:
    // ── Peripheral references (not owned) ──
    ButtonManager *btns_ = nullptr;
    DisplayManager *disp_ = nullptr;
    DiscoManager *disco_ = nullptr;
    LaserManager *laser_ = nullptr;
    RM3100 *magSensor_ = nullptr;
    SCA3300 *accel_ = nullptr;
    ConfigManager *cfgMgr_ = nullptr;
    MagCal::Calibration *cal_ = nullptr;

    // ── State machine ──
    CalibState state_ = CalibState::INACTIVE;
    CalibState preShutdownState_ = CalibState::INACTIVE; // state to restore on cancel
    uint32_t shutdownHoldStart_ = 0;                     // millis() when SHUTDOWN hold began (0 = not held)
    uint32_t shutdownLastDisplayMs_ = 0;

    // ── Data collection ──
    std::vector<Eigen::Vector3f> magArray_;
    std::vector<Eigen::Vector3f> gravArray_;

    // Rolling consistency buffer (runtime-sized, max 16)
    static constexpr int MAX_BUFFER_SIZE = 16;
    Eigen::Vector3f magBuffer_[MAX_BUFFER_SIZE];
    Eigen::Vector3f gravBuffer_[MAX_BUFFER_SIZE];
    int bufferLen_ = Defaults::calBufferLength; // runtime length from config
    float magThreshold_ = Defaults::calMagConsistency;
    float gravThreshold_ = Defaults::calGravConsistency;
    int bufferCount_ = 0;
    int bufferIdx_ = 0; // circular write index

    bool waitingForStable_ = false;
    uint32_t captureStart_ = 0; // millis() when MEASURE pressed (for timeout)
    uint16_t calTimeoutMs_ = Defaults::calTimeoutMs;

    // Rolling EMA pre-filter to smooth noise before consistency check
    float calEmaAlpha_ = Defaults::calEmaAlpha; // lower = smoother (0.3 = ~3 sample lag)
    Eigen::Vector3f emaMag_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f emaGrav_ = Eigen::Vector3f::Zero();
    bool emaInitialized_ = false;

    // Settle time: device must stay consistent for this duration before accepting
    uint16_t settleMs_ = Defaults::calSettleMs;
    uint32_t settleStart_ = 0; // millis() when consistency first detected (0 = not settling)
    Eigen::Vector3f accumMag_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f accumGrav_ = Eigen::Vector3f::Zero();
    int accumCount_ = 0;
    int iteration_ = 0;
    int targetCount_ = 56; // 56 for ellipsoid, 24 for alignment

    // ── Coverage bar (ellipsoid only) ──
    static constexpr int COV_COLS = 8;
    static constexpr int COV_ROWS = 4;
    bool coverageZones_[COV_ROWS][COV_COLS];

    // ── Save/discard hold detection ──
    float holdCounter_ = 0.0f;
    static constexpr float HOLD_TIME = 0.5f; // seconds to hold for save/discard

    // ── Calibration mode ──
    CalMode calMode_ = CalMode::PART1_ELLIPSOID;

    // ── Results ──
    float resultMagAcc_ = 0.0f;
    float resultGravAcc_ = 0.0f;
    float resultAccuracy_ = 0.0f;
    int rejectedCount_ = 0;       // Part 1: points dropped as outliers
    float resultDipSpread_ = 0.0f; // SD of per-point dip (deg) — the axis/
                                   // environment health check
    float envFieldRatio_ = 1.0f;  // Part 2 field / Part 1 field
    float envDipDev_ = 0.0f;      // Part 2 dip - Part 1 dip (deg)
    bool envWarn_ = false;        // Part 2 done somewhere magnetically different

    // Negative uniformity = fitEllipsoid rejected the data as degenerate;
    // the results screen then only offers discard, never save.
    bool ellipsoidFitFailed() const { return resultMagAcc_ < 0.0f || resultGravAcc_ < 0.0f; }

    // ── Timing ──
    uint32_t lastSampleTime_ = 0;
    uint32_t beepEndTime_ = 0;
    bool beepActive_ = false;
    uint32_t laserOnTime_ = 0; // millis() when laser should turn back on (0 = inactive)
    bool laserWibbleActive_ = false;

    // ── F/B check data ──
    static constexpr int FB_MAX_PAIRS = 10;
    float fbFwdAz_[FB_MAX_PAIRS]; // foresight direction per pair (degrees)
    float fbFwdInc_[FB_MAX_PAIRS];
    float fbBwdAz_[FB_MAX_PAIRS]; // backsight direction per pair (degrees)
    float fbBwdInc_[FB_MAX_PAIRS];
    int fbCount_ = 0;             // number of completed pairs
    bool fbHasForesight_ = false; // true if foresight recorded for current pair
    float fbCurrentFwdAz_ = 0.0f; // current pair's foresight
    float fbCurrentFwdInc_ = 0.0f;

    // Shots use the same pipeline as survey shots (median filter, averaged
    // steady hold, steady_tolerance) — a private instance, since the main
    // loop's SensorManager is not updated while calibration mode runs.
    SensorManager fbSensor_;
    float fbSteadyTol_ = Defaults::stabilityTolerance;

    // Leg consistency buffer (3 shots must agree, as for a survey leg)
    static constexpr int FB_LEG_LEN = 3;
    float fbLegAz_[FB_LEG_LEN];
    float fbLegInc_[FB_LEG_LEN];
    int fbLegCount_ = 0;
    float fbLegAngleTol_ = Defaults::legAngleTolerance;

    bool fbTakingShot_ = false; // true = red LED, waiting for a steady hold
    bool fbLaserOn_ = true;     // tracks laser state during FB collection
    bool fbLaserWibble_ = false;

    // Live display
    uint32_t fbLastDisplayTime_ = 0;
    float fbCurrentBearing_ = 0.0f;
    float fbCurrentInc_ = 0.0f;

    // ── State handlers ──
    void updateIntro();
    void updateCollecting();
    void updateCalculatingEllipsoid();
    void updateCalculatingAlignment();
    void updateShowResults();
    void updateSaving();
    void updateFBIntro();
    void updateFBWaitShot();
    void updateFBPairResult();
    void updateFBResults();
    void updateShutdownConfirm();

    // ── Helpers ──
    void readSensors(Eigen::Vector3f &mag, Eigen::Vector3f &grav);
    void acceptPoint(const Eigen::Vector3f &mag, const Eigen::Vector3f &grav);
    bool isConsistent(const Eigen::Vector3f *buffer, int count, float threshold) const;
    Eigen::Vector3f average(const Eigen::Vector3f *buffer, int count) const;
    void recordPoint(const Eigen::Vector3f &mag, const Eigen::Vector3f &grav);
    void updateCoverageBar(const Eigen::Vector3f &grav);

    // ── Display helpers ──
    void showEllipsoidIntro();
    void showAlignmentIntro();
    void showEllipsoidScreen();
    void showAlignmentProgress();
    void showCoverageBar();
    void showResultsScreen();
    void showSavingScreen();
    void showFBIntroScreen();
    void showFBLiveScreen();
    void showFBPairResult();
    void showFBResultsScreen();
    void showShutdownConfirmScreen(uint8_t holdPct = 0);
    void redrawScreen(CalibState s);

    // ── Beep control ──
    void beep();
    void beepTriple();
    void updateBeep();

    // ── Calculation ──
    void calculateEllipsoid();
    void rejectEllipsoidOutliers();
    void calculateAlignment();
    bool saveCalibration();
};
