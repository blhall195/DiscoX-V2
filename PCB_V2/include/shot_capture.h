#pragma once
// Shot capture: the compass/tilt reading for a shot is the average of the
// raw sensor vectors over the whole time the laser spends ranging, not a
// short steady window taken before it.
//
// The user has to hold the dot on the target while the laser measures
// anyway (three low-speed shots, up to ~4 s each), so that hold is sampled
// instead of thrown away. A long average takes out hand tremor and, unlike
// the old 0.3 s window, slow hand sway too; and the angles then describe
// the same moment as the distance. The steadiness test becomes a sanity
// check (did the device clearly move?) rather than the thing the accuracy
// rests on.
//
// Samples are pooled into 100 ms bins as they arrive (cheap enough to do
// from the laser driver's wait loop). finish() turns each bin into a
// direction, drops the bins furthest from the rest (the button-press jolt,
// a twitch), averages the raw vectors of what is left and computes the
// angles once from that average.

#include "mag_cal/calibration.h"
#include <ArduinoEigenDense.h>
#include <stdint.h>

class ShotCapture {
  public:
    enum class Status : uint8_t {
        OK,
        NO_DATA, // too few bins with both sensors in them
        MOVED,   // kept bins still spread wider than the tolerance
    };

    struct Result {
        float azimuth = 0.0f;
        float inclination = 0.0f;
        float roll = 0.0f;
        float spreadDeg = 0.0f; // largest deviation of a kept bin from the result
        uint16_t binsUsed = 0;
        uint16_t binsTotal = 0;
        Eigen::Vector3f meanMag = Eigen::Vector3f::Zero();   // raw, as fed in
        Eigen::Vector3f meanAccel = Eigen::Vector3f::Zero(); // raw, as fed in
    };

    void begin(uint32_t nowMs);
    void addMag(uint32_t nowMs, float x, float y, float z);
    void addAccel(uint32_t nowMs, float x, float y, float z);
    uint32_t elapsedMs(uint32_t nowMs) const { return nowMs - startMs_; }

    /// toleranceDeg: largest deviation any kept 100 ms bin may have from the
    /// averaged direction.
    Status finish(const MagCal::Calibration &cal, float toleranceDeg, Result &out);

  private:
    static constexpr uint32_t BIN_MS = 100;
    static constexpr uint16_t MAX_BINS = 220; // 22 s: 5 laser shots at ~4 s each
    static constexpr uint16_t MIN_BINS = 4;
    static constexpr uint16_t TRIM_DIVISOR = 5; // drop the worst 1/5 of bins

    struct Bin {
        float mag[3];
        float acc[3];
        uint8_t nMag;
        uint8_t nAcc;
    };

    Bin *binFor(uint32_t nowMs);

    // ~8 KB in all: instances must be static, never a local (4 KB loop stack)
    Bin bins_[MAX_BINS];
    float dir_[MAX_BINS][3];
    bool keep_[MAX_BINS];
    uint16_t binCount_ = 0;
    uint32_t startMs_ = 0;
};
