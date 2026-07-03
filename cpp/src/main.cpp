// adsb_replay: feed a recorded ADS-B CSV through the tracking core and print
// periodic track-table snapshots plus end-of-run statistics.

#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "adsb/csv_replay.hpp"
#include "adsb/measurement.hpp"
#include "adsb/track_manager.hpp"

namespace {

void printSnapshot(const adsb::TrackManager& manager, double now, double t0) {
    // Display time relative to replay start: live sessions carry unix-epoch
    // timestamps, which are unreadable in a table header.
    std::printf("\n=== t=+%.0fs | active tracks: %zu ===\n", now - t0,
                manager.tracks().size());
    std::printf("%-4s %-8s %10s %11s %8s %7s %6s %5s %5s\n", "TRK", "ICAO24",
                "LAT", "LON", "ALT[m]", "SPD[m/s]", "HDG", "AGE", "HITS");
    for (const adsb::Track& t : manager.tracks()) {
        double lat = 0.0, lon = 0.0;
        manager.trackPosition(t, lat, lon);
        std::printf("%-4d %-8s %10.4f %11.4f %8.0f %7.1f %6.0f %5.1f %5d\n",
                    t.id, t.aircraft_id.c_str(), lat, lon, t.altitude,
                    t.filter.speed(), t.reported_heading, t.age(now),
                    t.hit_count);
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string path =
        (argc > 1) ? argv[1] : std::string("data/sample_adsb.csv");
    const double snapshot_interval = 10.0;  // seconds of replay time

    std::vector<adsb::Measurement> measurements;
    try {
        measurements = adsb::loadMeasurementsFromFile(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        std::fprintf(stderr, "usage: %s [path\\to\\measurements.csv]\n", argv[0]);
        return 1;
    }
    if (measurements.empty()) {
        std::fprintf(stderr, "error: no measurements in %s\n", path.c_str());
        return 1;
    }

    // Center the tracking frame on the first report.
    const adsb::LocalFrame frame(measurements.front().latitude,
                                 measurements.front().longitude);
    adsb::TrackManager manager(frame);

    std::printf("adsb_replay: %zu measurements from %s\n", measurements.size(),
                path.c_str());
    std::printf("frame origin: %.4f, %.4f | coast limit: 30s\n",
                frame.refLatitude(), frame.refLongitude());

    const double t0 = measurements.front().timestamp;
    double next_snapshot = t0 + snapshot_interval;
    for (const adsb::Measurement& m : measurements) {
        if (m.timestamp >= next_snapshot) {
            manager.pruneStale(next_snapshot);
            printSnapshot(manager, next_snapshot, t0);
            // Jump to the first boundary after this measurement: a session
            // with a long quiet gap gets one snapshot, not one per interval.
            const double intervals_past =
                std::floor((m.timestamp - next_snapshot) / snapshot_interval);
            next_snapshot += (intervals_past + 1.0) * snapshot_interval;
        }
        manager.processMeasurement(m);
    }
    const double t_end = measurements.back().timestamp;
    manager.pruneStale(t_end);
    printSnapshot(manager, t_end, t0);

    std::printf("\n--- replay summary ---\n");
    std::printf("measurements processed : %d\n", manager.measurementsProcessed());
    std::printf("tracks created         : %d\n", manager.tracksCreated());
    std::printf("stale tracks removed   : %d\n", manager.staleTracksRemoved());
    std::printf("tracks active at end   : %zu\n", manager.tracks().size());
    return 0;
}
