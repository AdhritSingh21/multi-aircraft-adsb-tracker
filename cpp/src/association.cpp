#include "adsb/association.hpp"

#include <algorithm>
#include <limits>

namespace adsb {

namespace {
// Sentinel cost for padded / gated entries: far above any admissible cost,
// but finite so the Hungarian potentials stay well-behaved.
constexpr double kInadmissible = 1e9;
}

std::vector<int> greedyNearestNeighbor(
    const std::vector<std::vector<double>>& cost, double gate) {
    const int rows = static_cast<int>(cost.size());
    const int cols = rows > 0 ? static_cast<int>(cost[0].size()) : 0;
    std::vector<int> assignment(rows, -1);
    std::vector<char> row_done(rows, 0), col_done(cols, 0);

    for (int step = 0; step < std::min(rows, cols); ++step) {
        double best = std::numeric_limits<double>::infinity();
        int bi = -1, bj = -1;
        for (int i = 0; i < rows; ++i) {
            if (row_done[i]) continue;
            for (int j = 0; j < cols; ++j) {
                if (col_done[j]) continue;
                if (cost[i][j] <= gate && cost[i][j] < best) {
                    best = cost[i][j];
                    bi = i;
                    bj = j;
                }
            }
        }
        if (bi < 0) break;  // no admissible pair left
        assignment[bi] = bj;
        row_done[bi] = 1;
        col_done[bj] = 1;
    }
    return assignment;
}

std::vector<int> hungarianAssign(
    const std::vector<std::vector<double>>& cost, double gate) {
    const int rows = static_cast<int>(cost.size());
    const int cols = rows > 0 ? static_cast<int>(cost[0].size()) : 0;
    std::vector<int> assignment(rows, -1);
    if (rows == 0 || cols == 0) return assignment;

    // Pad to square; gated pairs become inadmissible so the optimizer only
    // routes through them when a row cannot be matched otherwise (filtered
    // out again below).
    const int n = std::max(rows, cols);
    std::vector<std::vector<double>> a(
        n + 1, std::vector<double>(n + 1, kInadmissible));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            a[i + 1][j + 1] = (cost[i][j] <= gate) ? cost[i][j] : kInadmissible;
        }
    }

    // Potentials formulation (u, v) with shortest augmenting paths.
    // p[j] = row matched to column j; way[j] = previous column on the path.
    const double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<int> p(n + 1, 0), way(n + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, kInf);
        std::vector<char> used(n + 1, 0);
        do {
            used[j0] = 1;
            const int i0 = p[j0];
            int j1 = 0;
            double delta = kInf;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = a[i0][j] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        // Augment along the found path.
        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    for (int j = 1; j <= n; ++j) {
        const int i = p[j];
        if (i >= 1 && i <= rows && j <= cols && cost[i - 1][j - 1] <= gate) {
            assignment[i - 1] = j - 1;
        }
    }
    return assignment;
}

}  // namespace adsb
