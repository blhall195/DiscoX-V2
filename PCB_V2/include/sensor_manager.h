#pragma once
// Sensor fusion: direct angle calculation from mag + accel, EMA output
// smoothing.

#include "mag_cal/calibration.h"
#include <ArduinoEigenDense.h>

class SensorManager {
  public:
    /// Initialise with calibration reference and tuning parameters
    void init(const MagCal::Calibration *cal, float emaAlphaStable, float emaAlphaMoving,
              uint8_t stabilityLen, float jumpThreshold = 8.0f);

    /// Feed raw sensor readings; updates angles and EMA.
    /// moving: true if the device is in motion (use fast alpha). V1 derived
    /// this from the gyro; V2's SCA3300 has no gyro, so the caller derives
    /// it from accel deltas instead (see readSensorsUpdate in main.cpp).
    void update(const Eigen::Vector3f &rawMag, const Eigen::Vector3f &rawAccel, bool moving);

    // ── Filtered angles (from calibration + EMA) ─────────────────────
    float getAzimuth() const { return emaAz_; }      // [0, 360)
    float getInclination() const { return emaInc_; } // [-90, +90]
    float getRoll() const { return roll_; }

    // ── Stability detection ──────────────────────────────────────────
    /// True once every reading in the stability window (the last
    /// stabilityLen x 100 ms of median-filtered angles) lies within
    /// toleranceDeg of the window's mean direction. az/inc receive that mean
    /// — the averaged reading to record. spreadDeg (optional) receives the
    /// largest deviation from the mean, for tuning.
    bool stableAverage(float toleranceDeg, float &az, float &inc, float *spreadDeg = nullptr) const;
    void resetStability();

  private:
    // EMA-smoothed output angles
    float emaAz_ = 0.0f;
    float emaInc_ = 0.0f;
    float roll_ = 0.0f;
    float emaAlphaStable_ = 0.05f;
    float emaAlphaMoving_ = 0.3f;
    float jumpThreshold_ = 8.0f;
    bool emaSeeded_ = false;

    // Stability ring buffer of median-filtered (pre-EMA) angles, decimated to
    // one push per stabIntervalMs_. Its length is stabilityLen x 100 ms.
    static constexpr uint32_t STAB_SAMPLES_PER_100MS = 4;
    static constexpr uint8_t MAX_STAB_LEN = 10; // 1 s window
    static constexpr uint8_t MAX_STAB_BUF = MAX_STAB_LEN * STAB_SAMPLES_PER_100MS;
    float azBuf_[MAX_STAB_BUF] = {};
    float incBuf_[MAX_STAB_BUF] = {};
    uint8_t stabHead_ = 0;
    uint8_t stabCount_ = 0;
    uint8_t stabLen_ = 20;
    uint32_t stabIntervalMs_ = 100 / STAB_SAMPLES_PER_100MS; // ms between stability samples
    uint32_t lastStabPushMs_ = 0;

    // Median pre-filter (3-sample, kills single-sample spikes)
    static constexpr uint8_t MEDIAN_LEN = 3;
    float medAzBuf_[MEDIAN_LEN] = {};
    float medIncBuf_[MEDIAN_LEN] = {};
    uint8_t medHead_ = 0;
    uint8_t medCount_ = 0;

    // Calibration (not owned)
    const MagCal::Calibration *cal_ = nullptr;

    // ── Internal helpers ─────────────────────────────────────────────
    void pushStability(float az, float inc);

    /// Push raw angles into median buffer; returns true when buffer is full
    bool pushMedian(float az, float inc);

    /// Get median azimuth from buffer (circular-aware)
    float medianAzimuth() const;

    /// Get median inclination from buffer
    float medianInclination() const;

    /// Median of a small array (sorts in place)
    static float medianN(float *vals, uint8_t n);

    /// Direction of (azimuth, inclination) as a unit vector
    static void unitVector(float azDeg, float incDeg, float &x, float &y, float &z);

    /// Circular EMA for azimuth (handles 0/360 wraparound)
    static float circularEma(float prev, float next, float alpha);
};
