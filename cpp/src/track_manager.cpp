#include "adsb/track_manager.hpp"

#include <algorithm>
#include <cmath>

namespace adsb {

TrackManager::TrackManager(const LocalFrame& frame, TrackManagerConfig cfg)
    : frame_(frame), cfg_(cfg) {}

ScanResult TrackManager::processMeasurement(const Measurement& m) {
    ++measurements_processed_;

    double zx = 0.0, zy = 0.0;
    frame_.toLocal(m.latitude, m.longitude, zx, zy);

    Track* track = findByAircraftId(m.aircraft_id);
    bool created = false;
    if (track == nullptr) {
        tracks_.push_back(makeTrack(m, zx, zy));
        ++tracks_created_;
        track = &tracks_.back();
        created = true;
    } else {
        // Out-of-order or duplicate timestamps: fuse without predicting
        // backwards (predict() ignores dt <= 0).
        track->filter.predict(m.timestamp - track->filter_time);
        track->filter_time = std::max(track->filter_time, m.timestamp);
        fuseMeasurement(*track, m, zx, zy);
    }
    appendHistory(*track, m.timestamp);
    return ScanResult{0, track->id, created};
}

std::vector<ScanResult> TrackManager::processScan(
    const std::vector<Measurement>& scan) {
    std::vector<ScanResult> results;
    if (scan.empty()) return results;
    results.reserve(scan.size());

    if (cfg_.association == AssociationMode::kById) {
        for (std::size_t i = 0; i < scan.size(); ++i) {
            ScanResult r = processMeasurement(scan[i]);
            r.measurement_index = static_cast<int>(i);
            results.push_back(r);
        }
        return results;
    }

    // --- Geometric association: identity field is ignored entirely. ---
    measurements_processed_ += static_cast<int>(scan.size());

    double scan_time = scan.front().timestamp;
    for (const Measurement& m : scan) {
        scan_time = std::max(scan_time, m.timestamp);
    }

    // 1) Predict every track to the scan time (coasting tracks keep the
    //    grown covariance, which correctly widens their gates).
    for (Track& t : tracks_) {
        t.filter.predict(scan_time - t.filter_time);
        t.filter_time = std::max(t.filter_time, scan_time);
    }

    // 2) Gated cost matrix: squared Mahalanobis distance measurement->track.
    const std::size_t n_meas = scan.size();
    const std::size_t n_tracks = tracks_.size();
    std::vector<double> zx(n_meas), zy(n_meas);
    std::vector<std::vector<double>> cost(
        n_meas, std::vector<double>(n_tracks, 0.0));
    for (std::size_t i = 0; i < n_meas; ++i) {
        frame_.toLocal(scan[i].latitude, scan[i].longitude, zx[i], zy[i]);
        for (std::size_t j = 0; j < n_tracks; ++j) {
            cost[i][j] = tracks_[j].filter.mahalanobisSq(zx[i], zy[i]);
        }
    }

    // 3) Assign.
    const std::vector<int> assignment =
        (cfg_.association == AssociationMode::kHungarian)
            ? hungarianAssign(cost, cfg_.gate_chi2)
            : greedyNearestNeighbor(cost, cfg_.gate_chi2);

    // 4) Fuse matched measurements; unmatched ones spawn new tracks
    //    afterwards (so creation cannot invalidate the j-indices above).
    for (std::size_t i = 0; i < n_meas; ++i) {
        if (assignment[i] < 0) continue;
        Track& t = tracks_[static_cast<std::size_t>(assignment[i])];
        fuseMeasurement(t, scan[i], zx[i], zy[i]);
        appendHistory(t, scan[i].timestamp);
        results.push_back(
            ScanResult{static_cast<int>(i), t.id, /*created=*/false});
    }
    for (std::size_t i = 0; i < n_meas; ++i) {
        if (assignment[i] >= 0) continue;
        tracks_.push_back(makeTrack(scan[i], zx[i], zy[i]));
        ++tracks_created_;
        appendHistory(tracks_.back(), scan[i].timestamp);
        results.push_back(
            ScanResult{static_cast<int>(i), tracks_.back().id,
                       /*created=*/true});
    }
    return results;
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

void TrackManager::fuseMeasurement(Track& t, const Measurement& m, double zx,
                                   double zy) {
    t.filter.update(zx, zy);
    t.altitude += cfg_.altitude_alpha * (m.altitude - t.altitude);
    t.reported_velocity = m.velocity;
    t.reported_heading = m.heading;
    t.last_update_time = std::max(t.last_update_time, m.timestamp);
    ++t.hit_count;
}

void TrackManager::appendHistory(Track& t, double timestamp) {
    double lat = 0.0, lon = 0.0;
    trackPosition(t, lat, lon);
    t.history.push_back({timestamp, lat, lon});
    if (t.history.size() > cfg_.max_history) {
        t.history.erase(t.history.begin());
    }
}

Track TrackManager::makeTrack(const Measurement& m, double x, double y) {
    Track t;
    t.id = next_track_id_++;
    t.aircraft_id = m.aircraft_id;
    t.first_seen_time = m.timestamp;
    t.last_update_time = m.timestamp;
    t.filter_time = m.timestamp;
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
