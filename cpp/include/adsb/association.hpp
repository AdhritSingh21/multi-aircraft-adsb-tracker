#pragma once

#include <vector>

namespace adsb {

// Chi-squared 99% quantile with 2 degrees of freedom. A measurement whose
// squared Mahalanobis distance from a track's predicted position exceeds
// this is statistically implausible for that track and may not be assigned
// to it (the pair is "outside the gate").
constexpr double kDefaultGateChi2 = 9.21;

// cost[i][j]: cost of assigning measurement i to track j (squared
// Mahalanobis distance). Both solvers return assignment[i] = track index,
// or -1 if measurement i is unassigned. Pairs with cost > gate are never
// assigned. Each track receives at most one measurement.

// Greedy global nearest neighbor: repeatedly commit the cheapest admissible
// (measurement, track) pair. Fast and simple; can be suboptimal when
// targets are close (the classic crossing case).
std::vector<int> greedyNearestNeighbor(
    const std::vector<std::vector<double>>& cost, double gate);

// Hungarian algorithm (Kuhn–Munkres, O(n^3) potentials formulation):
// globally minimizes total assignment cost. Rectangular matrices are padded
// internally; gated pairs are excluded from the result.
std::vector<int> hungarianAssign(
    const std::vector<std::vector<double>>& cost, double gate);

}  // namespace adsb
