#pragma once

#include <istream>
#include <string>
#include <vector>

#include "adsb/measurement.hpp"

namespace adsb {

// Parse one CSV line in the normalized format:
//   aircraft_id,timestamp,latitude,longitude,altitude,velocity,heading
// Returns false (never throws) for header, comment, blank, or malformed
// lines. Shared by the file loader and the adsb_stream stdin bridge.
bool parseMeasurementLine(const std::string& line, Measurement& out);

// Loads measurements from CSV (same format; bad rows skipped, not fatal).
// Rows are returned sorted by timestamp so they can be replayed in order.
std::vector<Measurement> loadMeasurementsFromStream(std::istream& in);
std::vector<Measurement> loadMeasurementsFromFile(const std::string& path);

}  // namespace adsb
