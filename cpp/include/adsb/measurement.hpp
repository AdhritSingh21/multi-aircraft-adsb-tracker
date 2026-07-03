#pragma once

#include <cmath>
#include <string>

namespace adsb {

// Normalized ADS-B position report. Every layer of the system (ingestion,
// tracking core, API) speaks this format.
struct Measurement {
    std::string aircraft_id;  // ICAO24 hex address, e.g. "a1b2c3"
    double timestamp = 0.0;   // seconds (unix epoch or seconds since replay t0)
    double latitude = 0.0;    // degrees
    double longitude = 0.0;   // degrees
    double altitude = 0.0;    // meters
    double velocity = 0.0;    // m/s ground speed
    double heading = 0.0;     // degrees clockwise from true north
};

// Local east-north tangent plane centered on a reference lat/lon.
// Equirectangular approximation: accurate to well under measurement noise for
// regional scopes (a few hundred km), and keeps the Kalman filter linear.
class LocalFrame {
public:
    static constexpr double kEarthRadiusM = 6371000.0;
    static constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    LocalFrame() = default;

    LocalFrame(double ref_lat_deg, double ref_lon_deg)
        : ref_lat_(ref_lat_deg),
          ref_lon_(ref_lon_deg),
          cos_ref_lat_(std::cos(ref_lat_deg * kDegToRad)) {}

    // Geodetic -> meters east (x) / north (y) of the reference point.
    void toLocal(double lat_deg, double lon_deg, double& x_east,
                 double& y_north) const {
        x_east = (lon_deg - ref_lon_) * kDegToRad * kEarthRadiusM * cos_ref_lat_;
        y_north = (lat_deg - ref_lat_) * kDegToRad * kEarthRadiusM;
    }

    // Meters east/north -> geodetic.
    void toGeodetic(double x_east, double y_north, double& lat_deg,
                    double& lon_deg) const {
        lat_deg = ref_lat_ + (y_north / kEarthRadiusM) / kDegToRad;
        lon_deg = ref_lon_ + (x_east / (kEarthRadiusM * cos_ref_lat_)) / kDegToRad;
    }

    double refLatitude() const { return ref_lat_; }
    double refLongitude() const { return ref_lon_; }

private:
    double ref_lat_ = 0.0;
    double ref_lon_ = 0.0;
    double cos_ref_lat_ = 1.0;
};

}  // namespace adsb
