#pragma once

#include <istream>
#include <string>
#include <vector>

#include "adsb/measurement.hpp"

namespace adsb {

// Loads measurements from CSV with the header:
//   aircraft_id,timestamp,latitude,longitude,altitude,velocity,heading
// Lines starting with '#' and blank lines are skipped. Rows are returned
// sorted by timestamp so they can be replayed in order.
std::vector<Measurement> loadMeasurementsFromStream(std::istream& in);
std::vector<Measurement> loadMeasurementsFromFile(const std::string& path);

}  // namespace adsb
