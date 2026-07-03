#pragma once

#include <string>
#include <vector>

#include "adsb/association.hpp"
#include "adsb/measurement.hpp"
#include "adsb/track.hpp"

namespace adsb {

// How incoming measurements are matched to existing tracks.
enum class AssociationMode {
    kById,             // trust the ADS-B identity (cooperative-sensor baseline)
    kNearestNeighbor,  // gated greedy nearest neighbor on Mahalanobis distance
    kHungarian,        // gated global-optimal assignment (Kuhn–Munkres)
};

struct TrackManagerConfig {
    double max_coast_seconds = 30.0;  // prune tracks not updated for this long
    double sigma_accel = 2.0;         // process noise, unmodeled accel [m/s^2]
    double sigma_pos = 50.0;          // measurement position noise [m]
    double init_pos_var = 100.0 * 100.0;   // new-track position variance [m^2]
    double init_vel_var = 25.0 * 25.0;     // new-track velocity variance [(m/s)^2]
    double altitude_alpha = 0.3;      // altitude exponential-smoothing gain
    std::size_t max_history = 200;    // trail points kept per track

    AssociationMode association = AssociationMode::kById;
    double gate_chi2 = kDefaultGateChi2;  // association gate (chi^2, 2 DOF)
};

// What happened to one measurement: which track absorbed it, and whether
// that track was newly created for it. Lets callers (tests, the replay
// evaluator) score association decisions against ground truth.
struct ScanResult {
    int measurement_index = 0;
    int track_id = 0;
    bool created_new_track = false;
};

// Maintains the set of active tracks. Two operating styles:
//  - processMeasurement(): streaming, associates by ADS-B aircraft_id.
//  - processScan(): batch of (roughly) simultaneous measurements, associated
//    per the configured AssociationMode — geometric modes ignore identity
//    entirely, as a non-cooperative sensor would have to.
class TrackManager {
public:
    TrackManager(const LocalFrame& frame, TrackManagerConfig cfg = {});

    // Fuse one measurement by aircraft_id: update the matching track or
    // start a new one.
    ScanResult processMeasurement(const Measurement& m);

    // Fuse one scan. All tracks are first predicted to the scan time; in
    // geometric modes measurements are then gated (chi^2 on Mahalanobis
    // distance) and assigned (greedy NN or Hungarian); unassigned
    // measurements spawn new tracks. Results are per input measurement.
    std::vector<ScanResult> processScan(const std::vector<Measurement>& scan);

    // Remove tracks whose last update is older than max_coast_seconds.
    // Returns the number of tracks removed.
    int pruneStale(double now);

    const std::vector<Track>& tracks() const { return tracks_; }
    const LocalFrame& frame() const { return frame_; }

    // Current filter estimate of a track as lat/lon.
    void trackPosition(const Track& t, double& lat, double& lon) const;

    // Lifetime statistics.
    int measurementsProcessed() const { return measurements_processed_; }
    int tracksCreated() const { return tracks_created_; }
    int staleTracksRemoved() const { return stale_removed_; }

private:
    Track* findByAircraftId(const std::string& aircraft_id);
    Track makeTrack(const Measurement& m, double x, double y);
    // Filter update + display fields + staleness clock (no predict — the
    // caller is responsible for having predicted the filter to ~m.timestamp).
    void fuseMeasurement(Track& t, const Measurement& m, double zx, double zy);
    void appendHistory(Track& t, double timestamp);

    LocalFrame frame_;
    TrackManagerConfig cfg_;
    std::vector<Track> tracks_;
    int next_track_id_ = 1;
    int measurements_processed_ = 0;
    int tracks_created_ = 0;
    int stale_removed_ = 0;
};

}  // namespace adsb
