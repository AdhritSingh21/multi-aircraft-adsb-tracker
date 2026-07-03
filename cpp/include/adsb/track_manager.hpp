#pragma once

#include <string>
#include <vector>

#include "adsb/measurement.hpp"
#include "adsb/track.hpp"

namespace adsb {

struct TrackManagerConfig {
    double max_coast_seconds = 30.0;  // prune tracks not updated for this long
    double sigma_accel = 2.0;         // process noise, unmodeled accel [m/s^2]
    double sigma_pos = 50.0;          // measurement position noise [m]
    double init_pos_var = 100.0 * 100.0;   // new-track position variance [m^2]
    double init_vel_var = 25.0 * 25.0;     // new-track velocity variance [(m/s)^2]
    double altitude_alpha = 0.3;      // altitude exponential-smoothing gain
    std::size_t max_history = 200;    // trail points kept per track
};

// Maintains the set of active tracks. Milestone 1 associates measurements to
// tracks by ADS-B aircraft_id (the honest baseline — ADS-B is a cooperative
// sensor). Milestone 3 replaces this with gated geometric association so the
// pipeline also works when identity is unreliable or withheld.
class TrackManager {
public:
    TrackManager(const LocalFrame& frame, TrackManagerConfig cfg = {});

    // Fuse one measurement: update the matching track or start a new one.
    void processMeasurement(const Measurement& m);

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

    LocalFrame frame_;
    TrackManagerConfig cfg_;
    std::vector<Track> tracks_;
    int next_track_id_ = 1;
    int measurements_processed_ = 0;
    int tracks_created_ = 0;
    int stale_removed_ = 0;
};

}  // namespace adsb
