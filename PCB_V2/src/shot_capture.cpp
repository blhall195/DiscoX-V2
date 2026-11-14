#include "shot_capture.h"
#include "math_utils.h"
#include <cmath>
#include <cstring>

void ShotCapture::begin(uint32_t nowMs) {
    startMs_ = nowMs;
    binCount_ = 0;
}

ShotCapture::Bin *ShotCapture::binFor(uint32_t nowMs) {
    uint32_t idx = (nowMs - startMs_) / BIN_MS;
    if (idx >= MAX_BINS) {
        return nullptr; // longer than any real shot: ignore the excess
    }
    while (binCount_ <= idx) {
        memset(&bins_[binCount_], 0, sizeof(Bin));
        binCount_++;
    }
    return &bins_[idx];
}

void ShotCapture::addMag(uint32_t nowMs, float x, float y, float z) {
    Bin *b = binFor(nowMs);
    if (!b || b->nMag == 255) {
        return;
    }
    b->mag[0] += x;
    b->mag[1] += y;
    b->mag[2] += z;
    b->nMag++;
}

void ShotCapture::addAccel(uint32_t nowMs, float x, float y, float z) {
    Bin *b = binFor(nowMs);
    if (!b || b->nAcc == 255) {
        return;
    }
    b->acc[0] += x;
    b->acc[1] += y;
    b->acc[2] += z;
    b->nAcc++;
}

ShotCapture::Status ShotCapture::finish(const MagCal::Calibration &cal, float toleranceDeg, Result &out) {
    // 1) Each usable bin -> its mean raw vectors and the direction they give
    uint16_t n = 0;
    for (uint16_t i = 0; i < binCount_; i++) {
        Bin &b = bins_[i];
        keep_[i] = b.nMag > 0 && b.nAcc > 0;
        if (!keep_[i]) {
            continue;
        }
        for (int k = 0; k < 3; k++) {
            b.mag[k] /= b.nMag;
            b.acc[k] /= b.nAcc;
        }
        b.nMag = b.nAcc = 1; // now holds means (finish() is called once per capture)
        MagCal::Angles a = cal.getAngles(Eigen::Vector3f(b.mag[0], b.mag[1], b.mag[2]),
                                         Eigen::Vector3f(b.acc[0], b.acc[1], b.acc[2]));
        float az = degreesToRadians(a.azimuth);
        float el = degreesToRadians(a.inclination);
        dir_[i][0] = cosf(el) * cosf(az);
        dir_[i][1] = cosf(el) * sinf(az);
        dir_[i][2] = sinf(el);
        n++;
    }
    out.binsTotal = n;
    out.binsUsed = n;
    if (n < MIN_BINS) {
        return Status::NO_DATA;
    }

    // 2) Drop the bins furthest from the mean direction of them all
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    for (uint16_t i = 0; i < binCount_; i++) {
        if (keep_[i]) {
            mx += dir_[i][0];
            my += dir_[i][1];
            mz += dir_[i][2];
        }
    }
    float norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm < 1e-6f) {
        return Status::MOVED;
    }
    mx /= norm;
    my /= norm;
    mz /= norm;
    for (uint16_t drop = n / TRIM_DIVISOR; drop > 0; drop--) {
        int worst = -1;
        float worstDot = 2.0f;
        for (uint16_t i = 0; i < binCount_; i++) {
            if (!keep_[i]) {
                continue;
            }
            float dot = dir_[i][0] * mx + dir_[i][1] * my + dir_[i][2] * mz;
            if (dot < worstDot) {
                worstDot = dot;
                worst = i;
            }
        }
        if (worst < 0) {
            break;
        }
        keep_[worst] = false;
        out.binsUsed--;
    }

    // 3) Average the raw vectors of the kept bins; angles once, from that
    Eigen::Vector3f mag = Eigen::Vector3f::Zero();
    Eigen::Vector3f acc = Eigen::Vector3f::Zero();
    for (uint16_t i = 0; i < binCount_; i++) {
        if (keep_[i]) {
            mag += Eigen::Vector3f(bins_[i].mag[0], bins_[i].mag[1], bins_[i].mag[2]);
            acc += Eigen::Vector3f(bins_[i].acc[0], bins_[i].acc[1], bins_[i].acc[2]);
        }
    }
    mag /= (float)out.binsUsed;
    acc /= (float)out.binsUsed;
    MagCal::Angles a = cal.getAngles(mag, acc);
    out.azimuth = a.azimuth;
    out.inclination = a.inclination;
    out.roll = a.roll;
    out.meanMag = mag;
    out.meanAccel = acc;

    // 4) Sanity check: did the device clearly move during the hold?
    float az = degreesToRadians(a.azimuth);
    float el = degreesToRadians(a.inclination);
    float rx = cosf(el) * cosf(az), ry = cosf(el) * sinf(az), rz = sinf(el);
    float minDot = 1.0f;
    for (uint16_t i = 0; i < binCount_; i++) {
        if (keep_[i]) {
            float dot = dir_[i][0] * rx + dir_[i][1] * ry + dir_[i][2] * rz;
            if (dot < minDot) {
                minDot = dot;
            }
        }
    }
    out.spreadDeg = radiansToDegrees(acosf(fminf(minDot, 1.0f)));
    return (out.spreadDeg > toleranceDeg) ? Status::MOVED : Status::OK;
}
