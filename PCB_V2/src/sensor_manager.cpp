#include "sensor_manager.h"
#include "math_utils.h"
#include "shot_vector.h"
#include <cmath>

// ── Initialisation ──────────────────────────────────────────────────

void SensorManager::init(const MagCal::Calibration *cal, float emaAlphaStable, float emaAlphaMoving,
                         uint8_t stabilityLen, float jumpThreshold) {
    cal_ = cal;
    emaAlphaStable_ = emaAlphaStable;
    emaAlphaMoving_ = emaAlphaMoving;
    jumpThreshold_ = jumpThreshold;
    if (stabilityLen < 2) {
        stabilityLen = 2;
    } else if (stabilityLen > MAX_STAB_LEN) {
        stabilityLen = MAX_STAB_LEN;
    }
    stabLen_ = stabilityLen * STAB_SAMPLES_PER_100MS;
    resetStability();

    emaSeeded_ = false;
    emaAz_ = 0.0f;
    emaInc_ = 0.0f;
    roll_ = 0.0f;
    medHead_ = 0;
    medCount_ = 0;
}

// ── Main update ─────────────────────────────────────────────────────

void SensorManager::update(const Eigen::Vector3f &rawMag, const Eigen::Vector3f &rawAccel, bool moving) {
    if (!cal_) {
        return;
    }

    // 1) Compute angles directly from calibrated mag + raw accel
    //    calibration.getAngles() does: axis remap, ellipsoid correction,
    //    orientation matrix, ZXY Euler extraction
    MagCal::Angles a = cal_->getAngles(rawMag, rawAccel);
    roll_ = a.roll; // roll not smoothed (matches Python)

    // 2) Median pre-filter — kills single-sample spikes
    if (!pushMedian(a.azimuth, a.inclination)) {
        return; // buffer not full yet
    }
    float filtAz = medianAzimuth();
    float filtInc = medianInclination();

    // Stability window takes the median-filtered angles, not the EMA: the
    // window is averaged for the recorded reading, and the EMA's lag would
    // make a settling device look like it is still moving.
    uint32_t now = millis();
    if ((now - lastStabPushMs_) >= stabIntervalMs_) {
        pushStability(filtAz, filtInc);
        lastStabPushMs_ = now;
    }

    // 3) EMA smooth — adaptive alpha: low when the device is still (max
    //    smoothing), high when it is moving (fast tracking).
    //    Jump detection: snap EMA directly when error exceeds threshold
    float alpha = moving ? emaAlphaMoving_ : emaAlphaStable_;
    if (!emaSeeded_) {
        emaAz_ = filtAz;
        emaInc_ = filtInc;
        emaSeeded_ = true;
    } else {
        Shot prev(emaAz_, emaInc_, 1.0f);
        Shot next(filtAz, filtInc, 1.0f);
        if (radiansToDegrees(prev.angleTo(next)) > jumpThreshold_) {
            // Large discontinuity — snap to new value immediately
            emaAz_ = filtAz;
            emaInc_ = filtInc;
            resetStability();
        } else {
            emaAz_ = circularEma(emaAz_, filtAz, alpha);
            emaInc_ = alpha * filtInc + (1.0f - alpha) * emaInc_;
        }
    }
}

// ── Circular EMA for azimuth ────────────────────────────────────────
// Handles 0/360 wraparound by working in the shortest-arc direction.

float SensorManager::circularEma(float prev, float next, float alpha) {
    return wrapTo360(prev + alpha * wrapTo180(next - prev));
}

// ── Stability ring buffer ───────────────────────────────────────────

void SensorManager::pushStability(float az, float inc) {
    azBuf_[stabHead_] = az;
    incBuf_[stabHead_] = inc;
    stabHead_ = (stabHead_ + 1) % stabLen_;
    if (stabCount_ < stabLen_) {
        stabCount_++;
    }
}

void SensorManager::unitVector(float azDeg, float incDeg, float &x, float &y, float &z) {
    float a = degreesToRadians(azDeg);
    float e = degreesToRadians(incDeg);
    x = cosf(e) * cosf(a);
    y = cosf(e) * sinf(a);
    z = sinf(e);
}

bool SensorManager::stableAverage(float toleranceDeg, float &az, float &inc, float *spreadDeg) const {
    if (stabCount_ < stabLen_) {
        return false;
    }
    if (!(toleranceDeg > 0.1f) || !std::isfinite(toleranceDeg)) {
        toleranceDeg = 0.5f; // same guard AngularLegChecker::setTolerance applied
    }

    // Average as unit vectors: immune to the 0/360 wrap and to azimuth
    // becoming meaningless near vertical.
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    for (uint8_t i = 0; i < stabCount_; i++) {
        float x, y, z;
        unitVector(azBuf_[i], incBuf_[i], x, y, z);
        mx += x;
        my += y;
        mz += z;
    }
    float norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm < 1e-6f) {
        return false;
    }
    mx /= norm;
    my /= norm;
    mz /= norm;

    float minDot = 1.0f;
    for (uint8_t i = 0; i < stabCount_; i++) {
        float x, y, z;
        unitVector(azBuf_[i], incBuf_[i], x, y, z);
        float dot = x * mx + y * my + z * mz;
        if (dot < minDot) {
            minDot = dot;
        }
    }
    if (spreadDeg) {
        *spreadDeg = radiansToDegrees(acosf(fminf(minDot, 1.0f)));
    }
    if (minDot < cosf(degreesToRadians(toleranceDeg))) {
        return false;
    }

    az = wrapTo360(radiansToDegrees(atan2f(my, mx)));
    inc = radiansToDegrees(asinf(mz));
    return true;
}

void SensorManager::resetStability() {
    stabHead_ = 0;
    stabCount_ = 0;
    lastStabPushMs_ = millis();
}

// ── Median pre-filter ──────────────────────────────────────────────

bool SensorManager::pushMedian(float az, float inc) {
    medAzBuf_[medHead_] = az;
    medIncBuf_[medHead_] = inc;
    medHead_ = (medHead_ + 1) % MEDIAN_LEN;
    if (medCount_ < MEDIAN_LEN) {
        medCount_++;
    }
    return medCount_ >= MEDIAN_LEN;
}

float SensorManager::medianInclination() const {
    float vals[MEDIAN_LEN];
    for (uint8_t i = 0; i < MEDIAN_LEN; i++) {
        vals[i] = medIncBuf_[i];
    }
    return medianN(vals, MEDIAN_LEN);
}

float SensorManager::medianAzimuth() const {
    // Unwrap relative to first sample to handle 0/360 boundary
    float ref = medAzBuf_[0];
    float vals[MEDIAN_LEN];
    for (uint8_t i = 0; i < MEDIAN_LEN; i++) {
        vals[i] = ref + wrapTo180(medAzBuf_[i] - ref);
    }
    return wrapTo360(medianN(vals, MEDIAN_LEN));
}

float SensorManager::medianN(float *vals, uint8_t n) {
    // Insertion sort (tiny array)
    for (uint8_t i = 1; i < n; i++) {
        float key = vals[i];
        int8_t j = i - 1;
        while (j >= 0 && vals[j] > key) {
            vals[j + 1] = vals[j];
            j--;
        }
        vals[j + 1] = key;
    }
    return vals[n / 2];
}
