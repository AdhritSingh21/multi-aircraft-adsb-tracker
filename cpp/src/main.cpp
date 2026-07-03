// adsb_replay: feed a recorded ADS-B CSV through the tracking core and print
// periodic track-table snapshots plus end-of-run statistics.
//
// Association modes (--assoc):
//   id         associate by ADS-B identity (default; cooperative baseline)
//   nn         gated greedy nearest neighbor — identities are HIDDEN from the
//              tracker and used only as ground truth to score association
//   hungarian  like nn, but with global-optimal (Kuhn–Munkres) assignment

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <map>
#include <string>
#include <vector>

#include "adsb/csv_replay.hpp"
#include "adsb/measurement.hpp"
#include "adsb/track_manager.hpp"

namespace {

struct AssociationScore {
    long associations = 0;  // measurements fused into an existing track
    long correct = 0;       // ...whose true id matched the track's seed id
    long new_tracks = 0;

    // track_id -> ADS-B id the track was created from (ground truth label)
    std::map<int, std::string> track_truth;

    void scoreScan(const std::vector<adsb::ScanResult>& results,
                   const std::vector<std::string>& true_ids) {
        for (const adsb::ScanResult& r : results) {
            const std::string& truth =
                true_ids[static_cast<std::size_t>(r.measurement_index)];
            if (r.created_new_track) {
                track_truth[r.track_id] = truth;
                ++new_tracks;
            } else {
                ++associations;
                if (track_truth[r.track_id] == truth) ++correct;
            }
        }
    }
};

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
        const char* icao = t.aircraft_id.empty() ? "-" : t.aircraft_id.c_str();
        std::printf("%-4d %-8s %10.4f %11.4f %8.0f %7.1f %6.0f %5.1f %5d\n",
                    t.id, icao, lat, lon, t.altitude, t.filter.speed(),
                    t.reported_heading, t.age(now), t.hit_count);
    }
}

// Group time-sorted measurements into scans: a new bucket starts when a
// measurement is more than `width` after the bucket's first one. Live
// aggregator reports are not synchronized scans, so this is the standard
// approximation for scan-based association.
std::vector<std::vector<adsb::Measurement>> bucketScans(
    const std::vector<adsb::Measurement>& measurements, double width) {
    std::vector<std::vector<adsb::Measurement>> scans;
    for (const adsb::Measurement& m : measurements) {
        if (scans.empty() ||
            m.timestamp - scans.back().front().timestamp > width) {
            scans.emplace_back();
        }
        scans.back().push_back(m);
    }
    return scans;
}

}  // namespace

int main(int argc, char** argv) {
    std::string path = "data/sample_adsb.csv";
    adsb::AssociationMode mode = adsb::AssociationMode::kById;
    const char* mode_name = "id";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--assoc") == 0 && i + 1 < argc) {
            mode_name = argv[++i];
            if (std::strcmp(mode_name, "id") == 0) {
                mode = adsb::AssociationMode::kById;
            } else if (std::strcmp(mode_name, "nn") == 0) {
                mode = adsb::AssociationMode::kNearestNeighbor;
            } else if (std::strcmp(mode_name, "hungarian") == 0) {
                mode = adsb::AssociationMode::kHungarian;
            } else {
                std::fprintf(stderr, "error: unknown --assoc mode '%s' "
                             "(expected id, nn, or hungarian)\n", mode_name);
                return 1;
            }
        } else {
            path = argv[i];
        }
    }

    const double snapshot_interval = 10.0;  // seconds of replay time
    const double scan_width = 1.0;          // scan bucketing window [s]

    std::vector<adsb::Measurement> measurements;
    try {
        measurements = adsb::loadMeasurementsFromFile(path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        std::fprintf(stderr,
                     "usage: %s [measurements.csv] [--assoc id|nn|hungarian]\n",
                     argv[0]);
        return 1;
    }
    if (measurements.empty()) {
        std::fprintf(stderr, "error: no measurements in %s\n", path.c_str());
        return 1;
    }

    // Center the tracking frame on the first report.
    const adsb::LocalFrame frame(measurements.front().latitude,
                                 measurements.front().longitude);
    adsb::TrackManagerConfig cfg;
    cfg.association = mode;
    adsb::TrackManager manager(frame, cfg);

    const bool geometric = (mode != adsb::AssociationMode::kById);
    std::printf("adsb_replay: %zu measurements from %s | association: %s%s\n",
                measurements.size(), path.c_str(), mode_name,
                geometric ? " (identities hidden from tracker)" : "");
    std::printf("frame origin: %.4f, %.4f | coast limit: 30s\n",
                frame.refLatitude(), frame.refLongitude());

    AssociationScore score;
    const double t0 = measurements.front().timestamp;
    double next_snapshot = t0 + snapshot_interval;

    const auto snapshotUpTo = [&](double timestamp) {
        if (timestamp < next_snapshot) return;
        manager.pruneStale(next_snapshot);
        printSnapshot(manager, next_snapshot, t0);
        // Jump to the first boundary after this time: a session with a long
        // quiet gap gets one snapshot, not one per interval.
        const double intervals_past =
            std::floor((timestamp - next_snapshot) / snapshot_interval);
        next_snapshot += (intervals_past + 1.0) * snapshot_interval;
    };

    for (std::vector<adsb::Measurement>& scan :
         bucketScans(measurements, scan_width)) {
        snapshotUpTo(scan.front().timestamp);
        if (geometric) {
            // Keep the true identities aside, hide them from the tracker.
            std::vector<std::string> true_ids;
            true_ids.reserve(scan.size());
            for (adsb::Measurement& m : scan) {
                true_ids.push_back(m.aircraft_id);
                m.aircraft_id.clear();
            }
            score.scoreScan(manager.processScan(scan), true_ids);
        } else {
            manager.processScan(scan);
        }
    }
    const double t_end = measurements.back().timestamp;
    manager.pruneStale(t_end);
    printSnapshot(manager, t_end, t0);

    std::printf("\n--- replay summary ---\n");
    std::printf("measurements processed : %d\n", manager.measurementsProcessed());
    std::printf("tracks created         : %d\n", manager.tracksCreated());
    std::printf("stale tracks removed   : %d\n", manager.staleTracksRemoved());
    std::printf("tracks active at end   : %zu\n", manager.tracks().size());

    if (geometric) {
        const double accuracy =
            score.associations > 0
                ? 100.0 * static_cast<double>(score.correct) /
                      static_cast<double>(score.associations)
                : 0.0;
        std::printf("\n--- association score (%s, ids hidden) ---\n", mode_name);
        std::printf("associations scored    : %ld\n", score.associations);
        std::printf("correct (vs ADS-B id)  : %ld\n", score.correct);
        std::printf("association accuracy   : %.2f%%\n", accuracy);
        std::printf("new tracks spawned     : %ld\n", score.new_tracks);
    }
    return 0;
}
