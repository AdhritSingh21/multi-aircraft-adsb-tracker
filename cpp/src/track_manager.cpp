#include "adsb/track_manager.hpp"

#include <algorithm>
#include <cmath>

namespace adsb {

TrackManager::TrackManager(const LocalFrame& frame, TrackManagerConfig cfg)
    : frame_(frame), cfg_(cfg) {}

void TrackManager::processMeasurement(const Measurement& m) {
    ++measurements_processed_;

    double zx = 0.0, zy = 0.0;
    frame_.toLocal(m.latitude, m.longitude, zx, zy);

    Track* track = findByAircraftId(m.aircraft_id);
    if (track == nullptr) {
        tracks_.push_back(makeTrack(m, zx, zy));
        ++tracks_created_;
        track = &tracks_.back();
    } else {
        // Out-of-order or duplicate timestamps: fuse without predicting
        // backwards (predict() ignores dt <= 0).
        track->filter.predict(m.timestamp - track->last_update_time);
        track->filter.update(zx, zy);

        track->altitude += cfg_.altitude_alpha * (m.altitude - track->altitude);
        track->reported_velocity = m.velocity;
        track->reported_heading = m.heading;
        track->last_update_time = std::max(track->last_update_time, m.timestamp);
        ++track->hit_count;
    }

    double lat = 0.0, lon = 0.0;
    trackPosition(*track, lat, lon);
    track->history.push_back({m.timestamp, lat, lon});
    if (track->history.size() > cfg_.max_history) {
        track->history.erase(track->history.begin());
    }
}

int TrackManager::pruneStale(double now) {
    const auto is_stale = [&](const Track& t) {
        return t.age(now) > cfg_.max_coast_seconds;
    };
    const auto it = std::remove_if(tracks_.begin(), tracks_.end(), is_stale);
    const int removed = static_cast<int>(tracks_.end() - it);
    tracks_.erase(it, tracks_.end());
    stale_removed_ += removed;
    return removed;
}

void TrackManager::trackPosition(const Track& t, double& lat,
                                 double& lon) const {
    frame_.toGeodetic(t.filter.x(), t.filter.y(), lat, lon);
}

Track* TrackManager::findByAircraftId(const std::string& aircraft_id) {
    for (Track& t : tracks_) {
        if (t.aircraft_id == aircraft_id) return &t;
    }
    return nullptr;
}

Track TrackManager::makeTrack(const Measurement& m, double x, double y) {
    Track t;
    t.id = next_track_id_++;
    t.aircraft_id = m.aircraft_id;
    t.first_seen_time = m.timestamp;
    t.last_update_time = m.timestamp;
    t.hit_count = 1;
    t.altitude = m.altitude;
    t.reported_velocity = m.velocity;
    t.reported_heading = m.heading;

    // Seed velocity from the reported ground speed + heading (deg clockwise
    // from north): east component = v*sin(h), north component = v*cos(h).
    const double h = m.heading * LocalFrame::kDegToRad;
    const double vx = m.velocity * std::sin(h);
    const double vy = m.velocity * std::cos(h);

    t.filter = KalmanFilter(cfg_.sigma_accel, cfg_.sigma_pos);
    t.filter.init(x, y, vx, vy, cfg_.init_pos_var, cfg_.init_vel_var);
    return t;
}

}  // namespace adsb
