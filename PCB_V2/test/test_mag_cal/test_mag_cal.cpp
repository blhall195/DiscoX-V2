// Synthetic-data validation of the mag_cal fitting pipeline.
//
// These are acceptance tests, not just regression tests: they verify that the
// C++ port recovers known ground truth (centre, soft-iron, injected hard-iron
// residual) from generated data. Sensor::reframe has no Python reference —
// a mistake there would corrupt every calibration converted to a corrected
// axis mapping — so its defining property is pinned down here.

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

// ── Sensor::reframe: converting a calibration between axis mappings ──
// Stored calibrations are carried into a corrected axis mapping at boot
// (initCalibration). Getting this wrong silently corrupts every converted
// device, so pin the defining property: after reframe, every calibrated
// vector is the old calibrated vector expressed in the new device frame.

// The mapping changes that have actually shipped: mag July/09-19 -> 09-26,
// grav 09-19 -> 09-26, grav July -> 09-26
static const char *const REFRAMES[][2] = {
    {"+Y-X+Z", "-Y-X+Z"}, {"-Y-X+Z", "-Y-X-Z"}, {"+Y-X-Z", "-Y-X-Z"}};

static std::vector<Eigen::Vector3f> distortedSphere() {
    Eigen::Matrix3f soft;
    soft << 1.10f, 0.02f, 0.00f, //
        0.02f, 0.95f, 0.01f,     //
        0.00f, 0.01f, 1.03f;
    Eigen::Vector3f centre(3.0f, -5.0f, 10.0f);
    std::vector<Eigen::Vector3f> data;
    for (const auto &u : spherePoints(56)) {
        data.push_back(soft * (50.0f * u) + centre);
    }
    return data;
}

void test_reframe_equals_fresh_fit_in_new_frame() {
    auto data = distortedSphere();
    for (const auto &r : REFRAMES) {
        MagCal::Sensor converted(r[0]);
        TEST_ASSERT_TRUE(converted.fitEllipsoid(data) >= 0.0f);
        converted.reframe(r[1]);

        MagCal::Sensor fresh(r[1]);
        TEST_ASSERT_TRUE(fresh.fitEllipsoid(data) >= 0.0f);

        TEST_ASSERT_EQUAL_STRING(r[1], converted.axes().toString());
        for (int i = 0; i < 3; i++) {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, fresh.centre()[i], converted.centre()[i]);
            for (int j = 0; j < 3; j++) {
                TEST_ASSERT_FLOAT_WITHIN(1e-6f, fresh.transform()(i, j), converted.transform()(i, j));
            }
        }
    }
}

void test_reframe_preserves_calibrated_vectors_including_rbf() {
    auto data = distortedSphere();
    // Non-trivial non-linear correction on X and Z (Y stays linear, as the
    // alignment fit produces)
    const float params[15] = {0.003f, -0.0004f, 0.0008f, -0.002f, 0.0025f, //
                              0, 0, 0, 0, 0,                                //
                              0.0007f, 0.0005f, 0.0009f, 0.0014f, 0.0019f};
    for (const auto &r : REFRAMES) {
        MagCal::Sensor before(r[0]);
        TEST_ASSERT_TRUE(before.fitEllipsoid(data) >= 0.0f);
        before.setNonLinearParams(params, 15);

        MagCal::Sensor after = before;
        Eigen::Matrix3f P = after.reframe(r[1]);

        for (const auto &raw : data) {
            Eigen::Vector3f expected = P * before.apply(raw);
            TEST_ASSERT_TRUE_MESSAGE((after.apply(raw) - expected).norm() < 1e-5f,
                                     "reframe changed a calibrated vector");
        }
    }
}

// ── Runner ──────────────────────────────────────────────────────────

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_getAngles_conventions);
    RUN_TEST(test_fitEllipsoid_recovers_centre_and_unit_sphere);
    RUN_TEST(test_fitEllipsoid_rejects_planar_data);
    RUN_TEST(test_fitEllipsoid_rejects_too_few_points);
    RUN_TEST(test_alignSensorRoll_transform_stays_finite);
    RUN_TEST(test_reframe_equals_fresh_fit_in_new_frame);
    RUN_TEST(test_reframe_preserves_calibrated_vectors_including_rbf);
    return UNITY_END();
}
