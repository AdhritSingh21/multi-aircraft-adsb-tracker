#pragma once

#include <string>
#include <vector>

#include "adsb/kalman_filter.hpp"

namespace adsb {

struct TrackPoint {
    double timestamp = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
};

// One tracked aircraft: filter state plus bookkeeping used for display,
// staleness pruning, and (later) association scoring.
struct Track {
    int id = 0;                    // track number assigned by TrackManager
    std::string aircraft_id;       // ICAO24 hex from ADS-B
    KalmanFilter filter;

    double first_seen_time = 0.0;
    double last_update_time = 0.0;
    int hit_count = 0;             // measurements fused into this track

    double altitude = 0.0;         // smoothed barometric/GNSS altitude [m]
    double reported_velocity = 0.0;  // last reported ground speed [m/s]
    double reported_heading = 0.0;   // last reported heading [deg]

    std::vector<TrackPoint> history;  // bounded position trail for display

    double age(double now) const { return now - last_update_time; }
};

}  // namespace adsb
