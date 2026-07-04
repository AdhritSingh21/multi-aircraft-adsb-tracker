#pragma once

#include <cstddef>
#include <string>

#include "adsb/track_manager.hpp"

namespace adsb {

// Escape a string for embedding in a JSON string literal.
std::string jsonEscape(const std::string& s);

// Serialize the manager's active tracks and lifetime stats as one JSON
// object on a single line (NDJSON-friendly):
//   {"time":T,"tracks":[{"id",...,"trail":[[lat,lon],...]}],"stats":{...}}
// Each track's trail is capped to its most recent `max_trail` points to
// bound message size for streaming consumers.
std::string snapshotToJson(const TrackManager& manager, double now,
                           std::size_t max_trail = 20);

}  // namespace adsb
