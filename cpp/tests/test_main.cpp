// Unit tests for the tracking core. Self-contained assert-based harness:
// each test function registers checks; main() reports PASS/FAIL per test and
// exits nonzero if anything failed (CTest-compatible).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "adsb/association.hpp"
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

// --------------------------------------------------------------- Association

void testGreedyNearestNeighborRespectsGate() {
    // Measurement 0 is closest to track 0, measurement 1 to track 1;
    // track 2 is inside nobody's gate.
    const std::vector<std::vector<double>> cost = {{1.0, 5.0, 100.0},
                                                   {4.0, 2.0, 100.0}};
    const auto a = adsb::greedyNearestNeighbor(cost, adsb::kDefaultGateChi2);
    CHECK(a.size() == 2);
    CHECK(a[0] == 0);
    CHECK(a[1] == 1);

    // Everything outside the gate -> unassigned.
    const auto b = adsb::greedyNearestNeighbor({{10.0, 10.0, 10.0}},
                                               adsb::kDefaultGateChi2);
    CHECK(b.size() == 1);
    CHECK(b[0] == -1);
}

void testHungarianBeatsGreedyOnCrossingCosts() {
    // The classic trap: greedy grabs the global minimum (0,0)=1 and is then
    // forced into (1,1)=100 (total 101). The optimum is (0,1)+(1,0) = 4.
    const std::vector<std::vector<double>> cost = {{1.0, 2.0},
                                                   {2.0, 100.0}};
    const double gate = 1000.0;  // everything admissible for this unit test

    const auto greedy = adsb::greedyNearestNeighbor(cost, gate);
    CHECK(greedy[0] == 0);
    CHECK(greedy[1] == 1);  // suboptimal, by construction

    const auto optimal = adsb::hungarianAssign(cost, gate);
    CHECK(optimal[0] == 1);
    CHECK(optimal[1] == 0);
}

void testHungarianMatchesBruteForceOptimum() {
    const std::vector<std::vector<double>> cost = {{7.0, 5.0, 11.0, 8.0},
                                                   {5.0, 6.0, 9.0, 7.0},
                                                   {9.0, 10.0, 3.0, 2.0},
                                                   {6.0, 4.0, 8.0, 5.0}};
    // Brute force: best total over all 24 permutations.
    std::vector<int> perm = {0, 1, 2, 3};
    double best = 1e18;
    do {
        double total = 0.0;
        for (int i = 0; i < 4; ++i) total += cost[i][perm[i]];
        best = std::min(best, total);
    } while (std::next_permutation(perm.begin(), perm.end()));

    const auto a = adsb::hungarianAssign(cost, 1e9);
    double total = 0.0;
    for (int i = 0; i < 4; ++i) {
        CHECK(a[i] >= 0);
        total += cost[i][a[i]];
    }
    CHECK_NEAR(total, best, 1e-9);
}

void testProcessScanAssociatesWithoutIdentity() {
    const adsb::LocalFrame frame(40.0, -83.0);
    adsb::TrackManagerConfig cfg;
    cfg.association = adsb::AssociationMode::kNearestNeighbor;
    adsb::TrackManager manager(frame, cfg);

    // Two aircraft ~70 km apart, identities blank (non-cooperative case).
    // A: eastbound 200 m/s at 40.0N. B: southbound 150 m/s at 40.5N.
    std::vector<adsb::Measurement> scan1 = {
        makeMeas("", 0.0, 40.0, -83.0, 10000.0, 200.0, 90.0),
        makeMeas("", 0.0, 40.5, -82.5, 8000.0, 150.0, 180.0)};
    const auto r1 = manager.processScan(scan1);
    CHECK(r1.size() == 2);
    CHECK(r1[0].created_new_track);
    CHECK(r1[1].created_new_track);
    const int track_a = r1[0].track_id;
    const int track_b = r1[1].track_id;
    CHECK(track_a != track_b);

    // 5 s later, both moved along their velocity vectors:
    // A: +1000 m east (~0.0117 deg lon at 40N); B: -750 m north (~0.00674 deg).
    std::vector<adsb::Measurement> scan2 = {
        makeMeas("", 5.0, 40.0, -82.9883, 10000.0, 200.0, 90.0),
        makeMeas("", 5.0, 40.49326, -82.5, 8000.0, 150.0, 180.0)};
    const auto r2 = manager.processScan(scan2);
    CHECK(r2.size() == 2);
    CHECK(!r2[0].created_new_track);
    CHECK(!r2[1].created_new_track);
    CHECK(r2[0].track_id == track_a);  // continuity purely from kinematics
    CHECK(r2[1].track_id == track_b);
    CHECK(manager.tracks().size() == 2);
    CHECK(manager.tracks()[0].hit_count == 2);
    CHECK(manager.tracks()[1].hit_count == 2);
}

void testProcessScanOutOfGateSpawnsNewTrack() {
    const adsb::LocalFrame frame(40.0, -83.0);
    adsb::TrackManagerConfig cfg;
    cfg.association = adsb::AssociationMode::kNearestNeighbor;
    adsb::TrackManager manager(frame, cfg);

    std::vector<adsb::Measurement> scan1 = {
        makeMeas("", 0.0, 40.0, -83.0, 10000.0, 200.0, 90.0)};
    manager.processScan(scan1);

    // 5 s later a report 20+ km away: kinematically impossible for the
    // existing track, so it must fail the chi^2 gate and spawn a new track.
    std::vector<adsb::Measurement> scan2 = {
        makeMeas("", 5.0, 40.2, -83.0, 5000.0, 100.0, 0.0)};
    const auto r = manager.processScan(scan2);
    CHECK(r.size() == 1);
    CHECK(r[0].created_new_track);
    CHECK(manager.tracks().size() == 2);
}

void testRepeatedPredictEqualsSinglePredict() {
    // Per-scan predicts must compose: predict(5)+predict(5) == predict(10)
    // for the linear CV model, or geometric association would corrupt
    // coasting tracks. Guards the filter_time bookkeeping design.
    adsb::KalmanFilter a(2.0, 50.0), b(2.0, 50.0);
    a.init(100.0, -50.0, 120.0, 30.0, 2500.0, 400.0);
    b.init(100.0, -50.0, 120.0, 30.0, 2500.0, 400.0);

    a.predict(5.0);
    a.predict(5.0);
    b.predict(10.0);

    CHECK_NEAR(a.x(), b.x(), 1e-9);
    CHECK_NEAR(a.y(), b.y(), 1e-9);
    CHECK_NEAR(a.vx(), b.vx(), 1e-9);
    CHECK_NEAR(a.vy(), b.vy(), 1e-9);
    CHECK_NEAR(a.positionVariance(), b.positionVariance(), 1e-6);
    CHECK_NEAR(a.mahalanobisSq(1300.0, 250.0), b.mahalanobisSq(1300.0, 250.0),
               1e-9);
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
    runTest("greedy_nn_respects_gate", testGreedyNearestNeighborRespectsGate);
    runTest("hungarian_beats_greedy_on_crossing_costs",
            testHungarianBeatsGreedyOnCrossingCosts);
    runTest("hungarian_matches_brute_force_optimum",
            testHungarianMatchesBruteForceOptimum);
    runTest("process_scan_associates_without_identity",
            testProcessScanAssociatesWithoutIdentity);
    runTest("process_scan_out_of_gate_spawns_new_track",
            testProcessScanOutOfGateSpawnsNewTrack);
    runTest("repeated_predict_equals_single_predict",
            testRepeatedPredictEqualsSinglePredict);
    runTest("csv_loader_parses_sorts_and_skips_bad_rows",
            testCsvLoaderParsesSortsAndSkipsBadRows);

    std::printf("-----------------------\n%d checks, %d failed\n", g_checks,
                g_failed_checks);
    return g_failed_checks == 0 ? 0 : 1;
}
