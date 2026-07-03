#include "adsb/kalman_filter.hpp"

#include <cmath>

namespace adsb {

namespace {
constexpr int N = KalmanFilter::kStateDim;
}

KalmanFilter::KalmanFilter(double sigma_accel, double sigma_pos)
    : q_(sigma_accel * sigma_accel), r_(sigma_pos * sigma_pos) {
    for (int i = 0; i < N; ++i) {
        x_[i] = 0.0;
        for (int j = 0; j < N; ++j) P_[i][j] = 0.0;
    }
}

void KalmanFilter::init(double x, double y, double vx, double vy,
                        double pos_var, double vel_var) {
    x_[0] = x;
    x_[1] = y;
    x_[2] = vx;
    x_[3] = vy;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) P_[i][j] = 0.0;
    P_[0][0] = pos_var;
    P_[1][1] = pos_var;
    P_[2][2] = vel_var;
    P_[3][3] = vel_var;
}

void KalmanFilter::predict(double dt) {
    if (dt <= 0.0) return;

    // Constant-velocity transition: F = I with F[0][2] = F[1][3] = dt.
    const double F[N][N] = {{1, 0, dt, 0},
                            {0, 1, 0, dt},
                            {0, 0, 1, 0},
                            {0, 0, 0, 1}};

    // x = F x
    double xnew[N];
    for (int i = 0; i < N; ++i) {
        xnew[i] = 0.0;
        for (int k = 0; k < N; ++k) xnew[i] += F[i][k] * x_[k];
    }
    for (int i = 0; i < N; ++i) x_[i] = xnew[i];

    // P = F P F' + Q
    double FP[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            FP[i][j] = 0.0;
            for (int k = 0; k < N; ++k) FP[i][j] += F[i][k] * P_[k][j];
        }
    }
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            double s = 0.0;
            for (int k = 0; k < N; ++k) s += FP[i][k] * F[j][k];
            P_[i][j] = s;
        }
    }

    // Continuous white-noise-acceleration process noise per axis:
    //   Q = q * [dt^3/3  dt^2/2]
    //           [dt^2/2  dt    ]
    // (exact discretization of continuous accel noise — unlike the
    // piecewise-constant dt^4/4 form, predict(a)+predict(b) == predict(a+b),
    // so uncertainty growth doesn't depend on how often we predict; the
    // geometric associator predicts every scan at whatever cadence data
    // arrives.)
    const double dt2 = dt * dt;
    const double q11 = q_ * dt2 * dt / 3.0;
    const double q12 = q_ * dt2 / 2.0;
    const double q22 = q_ * dt;
    P_[0][0] += q11;
    P_[0][2] += q12;
    P_[2][0] += q12;
    P_[2][2] += q22;
    P_[1][1] += q11;
    P_[1][3] += q12;
    P_[3][1] += q12;
    P_[3][3] += q22;
}

void KalmanFilter::update(double zx, double zy) {
    // Innovation y = z - Hx with H = [I2 0].
    const double yx = zx - x_[0];
    const double yy = zy - x_[1];

    // Innovation covariance S = HPH' + R (2x2).
    const double s00 = P_[0][0] + r_;
    const double s01 = P_[0][1];
    const double s10 = P_[1][0];
    const double s11 = P_[1][1] + r_;
    const double det = s00 * s11 - s01 * s10;
    if (det == 0.0) return;  // degenerate; skip update rather than divide by 0
    const double i00 = s11 / det;
    const double i01 = -s01 / det;
    const double i10 = -s10 / det;
    const double i11 = s00 / det;

    // Kalman gain K = P H' S^-1 (4x2). P H' is just the first two columns.
    double K[N][2];
    for (int i = 0; i < N; ++i) {
        K[i][0] = P_[i][0] * i00 + P_[i][1] * i10;
        K[i][1] = P_[i][0] * i01 + P_[i][1] * i11;
    }

    // State update.
    for (int i = 0; i < N; ++i) x_[i] += K[i][0] * yx + K[i][1] * yy;

    // Covariance update P = (I - K H) P.
    double newP[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            newP[i][j] = P_[i][j] - (K[i][0] * P_[0][j] + K[i][1] * P_[1][j]);
        }
    }
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) P_[i][j] = newP[i][j];
}

double KalmanFilter::mahalanobisSq(double zx, double zy) const {
    const double yx = zx - x_[0];
    const double yy = zy - x_[1];
    const double s00 = P_[0][0] + r_;
    const double s01 = P_[0][1];
    const double s10 = P_[1][0];
    const double s11 = P_[1][1] + r_;
    const double det = s00 * s11 - s01 * s10;
    if (det == 0.0) return 0.0;
    // y' S^-1 y with the 2x2 inverse written out.
    return (yx * (s11 * yx - s01 * yy) + yy * (-s10 * yx + s00 * yy)) / det;
}

double KalmanFilter::speed() const {
    return std::sqrt(x_[2] * x_[2] + x_[3] * x_[3]);
}

}  // namespace adsb
