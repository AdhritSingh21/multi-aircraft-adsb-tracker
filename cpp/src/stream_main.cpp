// adsb_stream: line-protocol tracker bridge for the backend.
//
//   stdin:  <measurement CSV line>  -> buffered into the current scan
//           TICK <t>                -> process buffered scan, prune stale
//                                      tracks at t, emit one JSON snapshot
//   stdout: one NDJSON snapshot per TICK (flushed immediately)
//
// The tracking frame is centered on the first measurement received, exactly
// like adsb_replay. Association mode via --assoc id|nn|hungarian (default id).
// EOF on stdin terminates cleanly.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "adsb/csv_replay.hpp"
#include "adsb/json_out.hpp"
#include "adsb/measurement.hpp"
#include "adsb/track_manager.hpp"

int main(int argc, char** argv) {
    adsb::AssociationMode mode = adsb::AssociationMode::kById;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--assoc") == 0 && i + 1 < argc) {
            const char* name = argv[++i];
            if (std::strcmp(name, "id") == 0) {
                mode = adsb::AssociationMode::kById;
            } else if (std::strcmp(name, "nn") == 0) {
                mode = adsb::AssociationMode::kNearestNeighbor;
            } else if (std::strcmp(name, "hungarian") == 0) {
                mode = adsb::AssociationMode::kHungarian;
            } else {
                std::fprintf(stderr, "error: unknown --assoc mode '%s'\n",
                             name);
                return 1;
            }
        }
    }

    std::unique_ptr<adsb::TrackManager> manager;  // created on first data
    std::vector<adsb::Measurement> scan_buffer;
    std::string line;
    adsb::Measurement m;

    while (std::getline(std::cin, line)) {
        if (line.rfind("TICK ", 0) == 0) {
            const double t = std::strtod(line.c_str() + 5, nullptr);
            if (manager == nullptr && !scan_buffer.empty()) {
                const adsb::LocalFrame frame(scan_buffer.front().latitude,
                                             scan_buffer.front().longitude);
                adsb::TrackManagerConfig cfg;
                cfg.association = mode;
                manager = std::make_unique<adsb::TrackManager>(frame, cfg);
            }
            if (manager == nullptr) {
                // No data yet: a well-formed empty snapshot keeps the
                // consumer's parse loop uniform.
                std::printf("{\"time\":%.1f,\"tracks\":[],\"stats\":"
                            "{\"measurements\":0,\"tracks_created\":0,"
                            "\"stale_removed\":0,\"active\":0}}\n", t);
                std::fflush(stdout);
                continue;
            }
            manager->processScan(scan_buffer);
            scan_buffer.clear();
            manager->pruneStale(t);
            const std::string json = adsb::snapshotToJson(*manager, t);
            std::fputs(json.c_str(), stdout);
            std::fputc('\n', stdout);
            std::fflush(stdout);
        } else if (adsb::parseMeasurementLine(line, m)) {
            scan_buffer.push_back(m);
        }
        // Unparsable non-TICK lines are ignored, same as the file loader.
    }
    return 0;
}
