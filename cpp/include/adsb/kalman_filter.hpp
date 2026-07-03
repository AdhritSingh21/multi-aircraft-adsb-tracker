#pragma once

namespace adsb {

// Linear Kalman filter with a constant-velocity motion model.
//
//   state x = [x_east, y_north, vx_east, vy_north]   (meters, m/s)
//   measurement z = [x_east, y_north]                (meters)
//
// Process noise follows the continuous white-noise-acceleration model:
// unmodeled accelerations (turns, speed changes) enter as white noise of
// intensity sigma_accel^2 [(m/s^2)^2 * s]. This discretization composes
// exactly — predict(a) then predict(b) equals predict(a+b) — so covariance
// growth is independent of prediction cadence. Measurement noise is
// isotropic position noise of sigma_pos [m], appropriate for ADS-B
// GNSS-derived positions.
class KalmanFilter {
public:
    static constexpr int kStateDim = 4;

    // sigma_accel: 1-sigma unmodeled acceleration [m/s^2]
    // sigma_pos:   1-sigma measurement position error [m]
    explicit KalmanFilter(double sigma_accel = 2.0, double sigma_pos = 50.0);

    // Initialize state and diagonal covariance.
    void init(double x, double y, double vx, double vy, double pos_var,
              double vel_var);

    // Propagate state and covariance forward by dt seconds (dt >= 0).
    void predict(double dt);

    // Fuse a position measurement (meters east/north).
    void update(double zx, double zy);

    // Squared Mahalanobis distance of a measurement from the predicted
    // position, using the innovation covariance S = HPH' + R. Used for
    // association gating (chi-squared with 2 DOF).
    double mahalanobisSq(double zx, double zy) const;

    double x() const { return x_[0]; }
    double y() const { return x_[1]; }
    double vx() const { return x_[2]; }
    double vy() const { return x_[3]; }
    double speed() const;

    // Trace of the position block of P — a scalar measure of position
    // uncertainty [m^2].
    double positionVariance() const { return P_[0][0] + P_[1][1]; }

private:
    double x_[kStateDim];              // state estimate
    double P_[kStateDim][kStateDim];   // state covariance
    double q_;                         // sigma_accel^2
    double r_;                         // sigma_pos^2
};

}  // namespace adsb
