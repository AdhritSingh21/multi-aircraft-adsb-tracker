#include "adsb/json_out.hpp"

#include <cstdio>

namespace adsb {

namespace {

void appendFmt(std::string& out, const char* fmt, double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, v);
    out += buf;
}

void appendInt(std::string& out, long v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%ld", v);
    out += buf;
}

}  // namespace

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (const char ch : s) {
        const unsigned char c = static_cast<unsigned char>(ch);
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += ch;
                }
        }
    }
    return out;
}

std::string snapshotToJson(const TrackManager& manager, double now,
                           std::size_t max_trail) {
    std::string j;
    j.reserve(4096);

    j += "{\"time\":";
    appendFmt(j, "%.1f", now);
    j += ",\"tracks\":[";

    bool first_track = true;
    for (const Track& t : manager.tracks()) {
        if (!first_track) j += ',';
        first_track = false;

        double lat = 0.0, lon = 0.0;
        manager.trackPosition(t, lat, lon);

        j += "{\"id\":";
        appendInt(j, t.id);
        j += ",\"icao\":\"";
        j += jsonEscape(t.aircraft_id);
        j += "\",\"lat\":";
        appendFmt(j, "%.6f", lat);
        j += ",\"lon\":";
        appendFmt(j, "%.6f", lon);
        j += ",\"alt_m\":";
        appendFmt(j, "%.1f", t.altitude);
        j += ",\"speed_mps\":";
        appendFmt(j, "%.1f", t.filter.speed());
        j += ",\"heading_deg\":";
        appendFmt(j, "%.1f", t.reported_heading);
        j += ",\"age_s\":";
        appendFmt(j, "%.1f", t.age(now));
        j += ",\"hits\":";
        appendInt(j, t.hit_count);
        j += ",\"trail\":[";

        const std::size_t n = t.history.size();
        const std::size_t start = n > max_trail ? n - max_trail : 0;
        for (std::size_t i = start; i < n; ++i) {
            if (i != start) j += ',';
            j += '[';
            appendFmt(j, "%.6f", t.history[i].latitude);
            j += ',';
            appendFmt(j, "%.6f", t.history[i].longitude);
            j += ']';
        }
        j += "]}";
    }

    j += "],\"stats\":{\"measurements\":";
    appendInt(j, manager.measurementsProcessed());
    j += ",\"tracks_created\":";
    appendInt(j, manager.tracksCreated());
    j += ",\"stale_removed\":";
    appendInt(j, manager.staleTracksRemoved());
    j += ",\"active\":";
    appendInt(j, static_cast<long>(manager.tracks().size()));
    j += "}}";
    return j;
}

}  // namespace adsb
