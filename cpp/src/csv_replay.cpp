#include "adsb/csv_replay.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace adsb {

namespace {

// Split one CSV line into fields (no quoting needed for this format).
std::vector<std::string> splitFields(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    std::istringstream ss(line);
    while (std::getline(ss, field, ',')) fields.push_back(field);
    return fields;
}

}  // namespace

bool parseMeasurementLine(const std::string& raw, Measurement& out) {
    std::string line = raw;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') return false;

    const auto fields = splitFields(line);
    if (fields.size() < 7) return false;

    try {
        Measurement m;
        m.aircraft_id = fields[0];
        m.timestamp = std::stod(fields[1]);
        m.latitude = std::stod(fields[2]);
        m.longitude = std::stod(fields[3]);
        m.altitude = std::stod(fields[4]);
        m.velocity = std::stod(fields[5]);
        m.heading = std::stod(fields[6]);
        out = std::move(m);
        return true;
    } catch (const std::exception&) {
        return false;  // header row or unparsable numeric field
    }
}

std::vector<Measurement> loadMeasurementsFromStream(std::istream& in) {
    std::vector<Measurement> out;
    std::string line;
    Measurement m;
    while (std::getline(in, line)) {
        if (parseMeasurementLine(line, m)) out.push_back(m);
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const Measurement& a, const Measurement& b) {
                         return a.timestamp < b.timestamp;
                     });
    return out;
}

std::vector<Measurement> loadMeasurementsFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open measurement file: " + path);
    }
    return loadMeasurementsFromStream(in);
}

}  // namespace adsb
