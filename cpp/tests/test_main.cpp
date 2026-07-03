// Unit tests for the tracking core. Self-contained assert-based harness:
// each test function registers checks; main() reports PASS/FAIL per test and
// exits nonzero if anything failed (CTest-compatible).

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "adsb/csv_replay.hpp"
#include "adsb/kalman_filter.hpp"
#include "adsb/measurement.hpp"
#include "adsb/track_manager.hpp"

namespace {

int g_failed_checks = 0;
int g_checks = 0;
bool g_current_test_ok = true;

#define CHECK(cond)                                                        \
    do {                                                                   \
        ++g_checks;                                                        \
        if (!(cond)) {                                                     \
            ++g_failed_checks;                                             \
            g_current_test_ok = false;                                     \
            std::printf("    CHECK failed at %s:%d: %s\n", __FILE__,       \
                        __LINE__, #cond);                                  \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                              \
    do {                                                                   \
        ++g_checks;                                                        \
        const double va = (a), vb = (b), vtol = (tol);                     \
        if (!(std::fabs(va - vb) <= vtol)) {                               \
            ++g_failed_checks;                                             \
            g_current_test_ok = false;                                     \
            std::printf("    CHECK_NEAR failed at %s:%d: %s=%.6g vs "      \
                        "%s=%.6g (tol %.6g)\n",                            \
                        __FILE__, __LINE__, #a, va, #b, vb, vtol);         \
        }                                                                  \
    } while (0)

void runTest(const char* name, void (*fn)()) {
    g_current_test_ok = true;
    fn();
    std::printf("[%s] %s\n", g_current_test_ok ? "PASS" : "FAIL", name);
}

// Deterministic pseudo-noise in [-1, 1) so the filter-convergence test is
// repeatable across platforms (no <random> distribution differences).
struct Lcg {
    unsigned long long s = 12345;
    double next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>((s >> 17) & 0xFFFFF) / 524288.0 - 1.0;
    }
};

// ---------------------------------------------------------------- LocalFrame

void testLocalFrameRoundtrip() {
    const adsb::LocalFrame frame(40.0, -83.0);

    // 0.01 deg of latitude is ~1111.95 m of northing.
    double x = 0.0, y = 0.0;
    frame.toLocal(40.01, -83.0, x, y);
    CHECK_NEAR(y, 1111.95, 1.0);
    CHECK_NEAR(x, 0.0, 1e-6);

    // Round trip returns the original coordinates.
    double lat = 0.0, lon = 0.0;
    frame.toGeodetic(5000.0, -3000.0, lat, lon);
    frame.toLocal(lat, lon, x, y);
    CHECK_NEAR(x, 5000.0, 1e-6);
    CHECK_NEAR(y, -3000.0, 1e-6);
}

// -------------------------------------------------------------- KalmanFilter

void testKalmanPredict() {
    adsb::KalmanFilter kf(2.0, 50.0);
    kf.init(0.0, 0.0, 100.0, -20.0, 2500.0, 100.0);

    const double var_before = kf.positionVariance();
    kf.predict(10.0);

    // Constant-velocity motion: position moves by v*dt, velocity unchanged.
    CHECK_NEAR(kf.x(), 1000.0, 1e-9);
    CHECK_NEAR(kf.y(), -200.0, 1e-9);
    CHECK_NEAR(kf.vx(), 100.0, 1e-9);
    CHECK_NEAR(kf.vy(), -20.0, 1e-9);

    // Coasting without measurements must grow position uncertainty.
    CHECK(kf.positionVariance() > var_before);

    // Non-positive dt is a no-op (out-of-order measurement protection).
    const double x_before = kf.x();
    kf.predict(-5.0);
    CHECK_NEAR(kf.x(), x_before, 1e-12);
}

void testKalmanConvergesOnConstantVelocityTarget() {
    // Truth: target starts at origin moving 150 m/s east. The filter is
    // initialized with ZERO velocity knowledge and must learn the velocity
    // from noisy position reports alone.
    adsb::KalmanFilter kf(2.0, 50.0);
    kf.init(0.0, 0.0, 0.0, 0.0, 2500.0, 10000.0);
    const double initial_pos_var = kf.positionVariance();

    Lcg noise;
    const double vx_true = 150.0, vy_true = 0.0;
    double final_pos_err = 0.0;
    for (int k = 1; k <= 25; ++k) {
        const double t = static_cast<double>(k);
        const double zx = vx_true * t + 50.0 * noise.next();
        const double zy = vy_true * t + 50.0 * noise.next();
        kf.predict(1.0);
        kf.update(zx, zy);
        const double ex = kf.x() - vx_true * t;
        const double ey = kf.y() - vy_true * t;
        final_pos_err = std::sqrt(ex * ex + ey * ey);
    }

    // Velocity learned from positions only, despite +/-50 m noise.
    CHECK_NEAR(kf.vx(), vx_true, 20.0);
    CHECK_NEAR(kf.vy(), vy_true, 20.0);
    // Position estimate tracks truth to well under the raw noise level.
    CHECK(final_pos_err < 75.0);
    // Fusing measurements must shrink uncertainty below its initial value.
    CHECK(kf.positionVariance() < initial_pos_var);
}

void testMahalanobisGate() {
    adsb::KalmanFilter kf(2.0, 50.0);
    kf.init(1000.0, 2000.0, 0.0, 0.0, 2500.0, 100.0);

    // A measurement at the predicted position scores ~0.
    CHECK_NEAR(kf.mahalanobisSq(1000.0, 2000.0), 0.0, 1e-9);
    // A measurement 10 km away is far outside any plausible 2-DOF gate.
    CHECK(kf.mahalanobisSq(11000.0, 2000.0) > 100.0);
}

// -------------------------------------------------------------- TrackManager

adsb::Measurement makeMeas(const std::string& id, double t, double lat,
                           double lon, double alt = 10000.0,
                           double vel = 200.0, double hdg = 90.0) {
    adsb::Measurement m;
    m.aircraft_id = id;
    m.timestamp = t;
    m.latitude = lat;
    m.longitude = lon;
    m.altitude = alt;
    m.velocity = vel;
    m.heading = hdg;
    return m;
}

void testManagerCreatesTrack() {
    const adsb::LocalFrame frame(40.0, -83.0);
    adsb::TrackManager manager(frame);

    manager.processMeasurement(makeMeas("a1b2c3", 0.0, 40.1, -82.9));

    CHECK(manager.tracks().size() == 1);
    CHECK(manager.tracksCreated() == 1);
    const adsb::Track& t = manager.tracks().front();
    CHECK(t.aircraft_id == "a1b2c3");
    CHECK(t.hit_count == 1);
    CHECK(t.id == 1);

    // New track initializes at the measurement position.
    double lat = 0.0, lon = 0.0;
    manager.trackPosition(t, lat, lon);
    CHECK_NEAR(lat, 40.1, 1e-6);
    CHECK_NEAR(lon, -82.9, 1e-6);
}

void testManagerUpdatesExistingTrack() {
    const adsb::LocalFrame frame(40.0, -83.0);
    adsb::TrackManager manager(frame);

    // Eastbound at ~200 m/s: 5 s => ~1000 m => ~0.0117 deg lon at 40N.
    manager.processMeasurement(makeMeas("a1b2c3", 0.0, 40.0, -83.0, 10000.0));
    manager.processMeasurement(
        makeMeas("a1b2c3", 5.0, 40.0, -82.9883, 10200.0));

    CHECK(manager.tracks().size() == 1);  // same aircraft, same track
    const adsb::Track& t = manager.tracks().front();
    CHECK(t.hit_count == 2);
    CHECK_NEAR(t.last_update_time, 5.0, 1e-12);

    // Fused position moved most of the way to the second report.
    double lat = 0.0, lon = 0.0;
    manager.trackPosition(t, lat, lon);
    CHECK(lon > -83.0 + 0.5 * 0.0117);
    // Altitude smoothing moved toward the new report but not all the way.
    CHECK(t.altitude > 10000.0);
    CHECK(t.altitude < 10200.0);
}

void testManagerSeparateTracksPerAircraft() {
    const adsb::LocalFrame frame(40.0, -83.0);
    adsb::TrackManager manager(frame);

    manager.processMeasurement(makeMeas("a1b2c3", 0.0, 40.1, -82.9));
    manager.processMeasurement(makeMeas("c3f5a7", 1.0, 39.9, -83.1));
    manager.processMeasurement(makeMeas("a1b2c3", 2.0, 40.1, -82.895));

    CHECK(manager.tracks().size() == 2);
    CHECK(manager.tracksCreated() == 2);
    CHECK(manager.tracks()[0].id != manager.tracks()[1].id);
    CHECK(manager.tracks()[0].hit_count == 2);  // a1b2c3 got both reports
    CHECK(manager.tracks()[1].hit_count == 1);
}

void testManagerPrunesStaleTracks() {
    const adsb::LocalFrame frame(40.0, -83.0);
    adsb::TrackManager manager(frame);  // default coast limit: 30 s

    manager.processMeasurement(makeMeas("alive1", 0.0, 40.1, -82.9));
    manager.processMeasurement(makeMeas("stale1", 0.0, 39.9, -83.1));
    manager.processMeasurement(makeMeas("alive1", 40.0, 40.1, -82.85));
    // "stale1" last reported at t=0; at t=40 its age (40 s) exceeds 30 s.

    const int removed = manager.pruneStale(40.0);

    CHECK(removed == 1);
    CHECK(manager.staleTracksRemoved() == 1);
    CHECK(manager.tracks().size() == 1);
    CHECK(manager.tracks().front().aircraft_id == "alive1");

    // Nothing else is stale: pruning again removes nothing.
    CHECK(manager.pruneStale(40.0) == 0);
}

// ---------------------------------------------------------------- CSV replay

void testCsvLoaderParsesSortsAndSkipsBadRows() {
    std::istringstream csv(
        "# recorded sample\r\n"
        "aircraft_id,timestamp,latitude,longitude,altitude,velocity,heading\r\n"
        "a1b2c3,10.0,40.10,-82.90,10500,220,90\r\n"
        "c3f5a7,5.0,39.90,-83.10,3000,150,225\n"
        "brokenrow,notanumber,40,-83,1,2,3\n"
        "shortrow,1.0,40.0\n"
        "\n"
        "a1b2c3,12.0,40.10,-82.88,10500,220,90\n");

    const auto ms = adsb::loadMeasurementsFromStream(csv);

    CHECK(ms.size() == 3);  // bad rows skipped, not fatal
    // Sorted by timestamp regardless of file order.
    CHECK_NEAR(ms[0].timestamp, 5.0, 1e-12);
    CHECK_NEAR(ms[1].timestamp, 10.0, 1e-12);
    CHECK_NEAR(ms[2].timestamp, 12.0, 1e-12);
    CHECK(ms[0].aircraft_id == "c3f5a7");
    CHECK_NEAR(ms[0].heading, 225.0, 1e-12);
    CHECK_NEAR(ms[1].latitude, 40.10, 1e-12);  // CRLF stripped cleanly
}

}  // namespace

int main() {
    std::printf("adsb-tracker unit tests\n-----------------------\n");
    runTest("local_frame_roundtrip", testLocalFrameRoundtrip);
    runTest("kalman_predict", testKalmanPredict);
    runTest("kalman_converges_on_cv_target",
            testKalmanConvergesOnConstantVelocityTarget);
    runTest("mahalanobis_gate", testMahalanobisGate);
    runTest("manager_creates_track", testManagerCreatesTrack);
    runTest("manager_updates_existing_track", testManagerUpdatesExistingTrack);
    runTest("manager_separate_tracks_per_aircraft",
            testManagerSeparateTracksPerAircraft);
    runTest("manager_prunes_stale_tracks", testManagerPrunesStaleTracks);
    runTest("csv_loader_parses_sorts_and_skips_bad_rows",
            testCsvLoaderParsesSortsAndSkipsBadRows);

    std::printf("-----------------------\n%d checks, %d failed\n", g_checks,
                g_failed_checks);
    return g_failed_checks == 0 ? 0 : 1;
}
