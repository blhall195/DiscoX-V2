// Synthetic-data validation of the mag_cal fitting pipeline.
//
// These are acceptance tests, not just regression tests: they verify that the
// C++ port recovers known ground truth (centre, soft-iron, injected hard-iron
// residual) from generated data. applyFBCorrection especially has no Python
// reference — a wrong sign there would *corrupt* a good calibration in the
// field, so its sign convention is pinned down here.

#include "mag_cal/calibration.h"
#include "mag_cal/sensor.h"
#include <math.h>
#include <string.h>
#include <unity.h>
#include <vector>

void setUp() {}
void tearDown() {}

static constexpr float DEG2RADF = (float)M_PI / 180.0f;

// ── Synthetic data helpers ──────────────────────────────────────────

/// n roughly-uniform directions on the unit sphere (Fibonacci spiral)
static std::vector<Eigen::Vector3f> spherePoints(int n) {
    std::vector<Eigen::Vector3f> pts;
    const float goldenAngle = 2.39996323f;
    for (int i = 0; i < n; i++) {
        float z = 1.0f - 2.0f * (i + 0.5f) / (float)n;
        float r = sqrtf(1.0f - z * z);
        float th = goldenAngle * (float)i;
        pts.push_back(Eigen::Vector3f(r * cosf(th), r * sinf(th), z));
    }
    return pts;
}

/// Device-frame magnetometer reading for a level device at the given heading.
/// Earth field: horizontal component north, vertical component down (dip).
/// Device axes: X right, Y forward, Z up (matches getOrientationMatrix
/// conventions — validated by test_getAngles_conventions below).
static Eigen::Vector3f magAtHeading(float headingDeg, float dipDeg) {
    float th = headingDeg * DEG2RADF;
    float d = dipDeg * DEG2RADF;
    return Eigen::Vector3f(-cosf(d) * sinf(th), cosf(d) * cosf(th), -sinf(d));
}

/// Gravity reading for a level device (sensed such that upward = -grav)
static Eigen::Vector3f gravLevel() { return Eigen::Vector3f(0.0f, 0.0f, -1.0f); }

/// Identity calibration (unit transform, zero centre) with a known dip
static const char IDENTITY_CAL_JSON[] = R"({
  "mag":  {"axes": "+X+Y+Z",
           "transform": [[1,0,0],[0,1,0],[0,0,1]],
           "centre": [0,0,0], "rbfs": [],
           "field_avg": 1.0, "field_std": 0.001},
  "dip_avg": 30.0,
  "grav": {"axes": "+X+Y+Z",
           "transform": [[1,0,0],[0,1,0],[0,0,1]],
           "centre": [0,0,0], "rbfs": [],
           "field_avg": 1.0, "field_std": 0.001}
})";

static void loadIdentityCal(MagCal::Calibration &cal) {
    bool ok = cal.fromJson(IDENTITY_CAL_JSON, strlen(IDENTITY_CAL_JSON));
    TEST_ASSERT_TRUE_MESSAGE(ok, "identity calibration JSON failed to parse");
}

static float wrap180(float deg) {
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

// ── getAngles conventions (validates the helpers above) ─────────────

void test_getAngles_conventions() {
    MagCal::Calibration cal;
    loadIdentityCal(cal);

    for (float heading : {0.0f, 45.0f, 90.0f, 210.0f, 300.0f}) {
        MagCal::Angles a = cal.getAngles(magAtHeading(heading, 30.0f), gravLevel());
        TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, wrap180(a.azimuth - heading));
        TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, a.inclination);
    }
}

// ── fitEllipsoid: ground-truth recovery ─────────────────────────────

void test_fitEllipsoid_recovers_centre_and_unit_sphere() {
    // True field radius 50 (µT-ish), known soft-iron + hard-iron
    Eigen::Matrix3f soft;
    soft << 1.10f, 0.02f, 0.00f, //
        0.02f, 0.95f, 0.01f,     //
        0.00f, 0.01f, 1.03f;
    Eigen::Vector3f centre(3.0f, -5.0f, 10.0f);
    const float R = 50.0f;

    std::vector<Eigen::Vector3f> data;
    for (const auto &u : spherePoints(56)) {
        data.push_back(soft * (R * u) + centre);
    }

    MagCal::Sensor s("+X+Y+Z");
    float uni = s.fitEllipsoid(data);

    TEST_ASSERT_TRUE_MESSAGE(uni >= 0.0f, "fit reported failure on clean data");
    TEST_ASSERT_TRUE_MESSAGE(uni < 0.01f, "uniformity too poor on clean data");
    TEST_ASSERT_TRUE(s.isCalibrated());

    // Hard-iron centre recovered
    TEST_ASSERT_FLOAT_WITHIN(0.05f, centre[0], s.centre()[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, centre[1], s.centre()[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, centre[2], s.centre()[2]);

    // Every calibrated point lands on the unit sphere
    for (const auto &d : data) {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, s.apply(d).norm());
    }
}

void test_fitEllipsoid_rejects_planar_data() {
    // All points on a circle in the z-plane — not an ellipsoid
    std::vector<Eigen::Vector3f> data;
    for (int i = 0; i < 56; i++) {
        float th = 0.37f * (float)i;
        data.push_back(Eigen::Vector3f(50.0f * cosf(th), 50.0f * sinf(th), 12.0f));
    }

    MagCal::Sensor s("+X+Y+Z");
    float uni = s.fitEllipsoid(data);

    TEST_ASSERT_TRUE_MESSAGE(uni < 0.0f, "degenerate fit was not rejected");
    TEST_ASSERT_FALSE(s.isCalibrated());
    // Calibration state untouched: transform still identity, centre zero
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, s.transform()(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.centre()[0]);
}

void test_fitEllipsoid_rejects_too_few_points() {
    std::vector<Eigen::Vector3f> data;
    for (const auto &u : spherePoints(5)) {
        data.push_back(50.0f * u);
    }
    MagCal::Sensor s("+X+Y+Z");
    TEST_ASSERT_TRUE(s.fitEllipsoid(data) < 0.0f);
    TEST_ASSERT_FALSE(s.isCalibrated());
}

// ── alignSensorRoll: never poisons the transform ────────────────────

void test_alignSensorRoll_transform_stays_finite() {
    MagCal::Calibration cal;
    loadIdentityCal(cal);

    // Level sweep of headings — plus a couple of rolled poses
    std::vector<Eigen::Vector3f> mags, gravs;
    for (float h = 0.0f; h < 360.0f; h += 30.0f) {
        mags.push_back(magAtHeading(h, 30.0f));
        gravs.push_back(gravLevel());
    }
    cal.alignSensorRoll(mags, gravs);

    const Eigen::Matrix3f &t = cal.mag().transform();
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            TEST_ASSERT_TRUE_MESSAGE(isfinite(t(r, c)), "NaN/inf in mag transform");
        }
    }
}

// ── applyFBCorrection: sign convention + ground-truth recovery ──────

// Bearings observed when a residual hard-iron offset delta (device frame,
// calibrated units) contaminates the readings.
static void observedFB(const MagCal::Calibration &cal, const Eigen::Vector3f &delta, float dip,
                       const float *stations, int n, float *fwd, float *bwd) {
    for (int i = 0; i < n; i++) {
        fwd[i] = cal.getAngles(magAtHeading(stations[i], dip) + delta, gravLevel()).azimuth;
        bwd[i] = cal.getAngles(magAtHeading(stations[i] + 180.0f, dip) + delta, gravLevel()).azimuth;
    }
}

void test_applyFBCorrection_recovers_injected_offset() {
    MagCal::Calibration cal;
    loadIdentityCal(cal);

    const float dip = 30.0f; // must match dip_avg in the JSON
    const Eigen::Vector3f delta(0.03f, 0.02f, 0.0f);
    const float stations[] = {10.0f, 75.0f, 150.0f, 230.0f, 310.0f};
    const int n = 5;
    float fwd[n], bwd[n];
    observedFB(cal, delta, dip, stations, n, fwd, bwd);

    float amp = cal.applyFBCorrection(fwd, bwd, n);

    // Amplitude ≈ |delta_horizontal| / cos(dip), in degrees
    float expectedAmp = sqrtf(delta[0] * delta[0] + delta[1] * delta[1]) / cosf(dip * DEG2RADF) / DEG2RADF;
    TEST_ASSERT_FLOAT_WITHIN(0.3f, expectedAmp, amp);

    // Centre absorbed the offset — SIGN is the critical assertion here:
    // centre must move TOWARD +delta so that (raw - centre) removes it
    TEST_ASSERT_FLOAT_WITHIN(0.005f, delta[0], cal.mag().centre()[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, delta[1], cal.mag().centre()[1]);

    // Gold check: with the corrected calibration, the same physical setup
    // now yields near-zero foresight/backsight disagreement
    observedFB(cal, delta, dip, stations, n, fwd, bwd);
    for (int i = 0; i < n; i++) {
        float err = wrap180(fwd[i] - bwd[i] - 180.0f);
        TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, err);
    }
}

void test_applyFBCorrection_not_applied_with_two_pairs() {
    MagCal::Calibration cal;
    loadIdentityCal(cal);

    const Eigen::Vector3f delta(0.03f, 0.02f, 0.0f);
    const float stations[] = {10.0f, 100.0f};
    float fwd[2], bwd[2];
    observedFB(cal, delta, 30.0f, stations, 2, fwd, bwd);

    cal.applyFBCorrection(fwd, bwd, 2);

    // Guard must leave the calibration untouched
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cal.mag().centre()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cal.mag().centre()[1]);
}

void test_applyFBCorrection_not_applied_with_clustered_bearings() {
    MagCal::Calibration cal;
    loadIdentityCal(cal);

    // All shots within a 30° sector — fit would be an extrapolation
    const Eigen::Vector3f delta(0.03f, 0.02f, 0.0f);
    const float stations[] = {10.0f, 18.0f, 25.0f, 33.0f, 40.0f};
    float fwd[5], bwd[5];
    observedFB(cal, delta, 30.0f, stations, 5, fwd, bwd);

    cal.applyFBCorrection(fwd, bwd, 5);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cal.mag().centre()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cal.mag().centre()[1]);
}

void test_applyFBCorrection_not_applied_when_not_sinusoidal() {
    MagCal::Calibration cal;
    loadIdentityCal(cal);

    // Hand-built bearing errors: alternating ±2° — large amplitude in the
    // data but nothing a one-cycle sinusoid can explain (residual RMS ≈
    // fitted amplitude). Simulates a blunder pair / non-hard-iron error.
    const float stations[] = {10.0f, 80.0f, 150.0f, 220.0f, 290.0f};
    float fwd[5], bwd[5];
    for (int i = 0; i < 5; i++) {
        float err = (i % 2 == 0) ? 2.0f : -2.0f;
        fwd[i] = stations[i];
        float b = stations[i] - 180.0f - 2.0f * err;
        while (b < 0.0f) {
            b += 360.0f;
        }
        bwd[i] = b;
    }

    cal.applyFBCorrection(fwd, bwd, 5);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cal.mag().centre()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cal.mag().centre()[1]);
}

// ── Runner ──────────────────────────────────────────────────────────

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_getAngles_conventions);
    RUN_TEST(test_fitEllipsoid_recovers_centre_and_unit_sphere);
    RUN_TEST(test_fitEllipsoid_rejects_planar_data);
    RUN_TEST(test_fitEllipsoid_rejects_too_few_points);
    RUN_TEST(test_alignSensorRoll_transform_stays_finite);
    RUN_TEST(test_applyFBCorrection_recovers_injected_offset);
    RUN_TEST(test_applyFBCorrection_not_applied_with_two_pairs);
    RUN_TEST(test_applyFBCorrection_not_applied_with_clustered_bearings);
    RUN_TEST(test_applyFBCorrection_not_applied_when_not_sinusoidal);
    return UNITY_END();
}
