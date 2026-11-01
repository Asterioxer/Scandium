#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "scandium/navigation/astar.hpp"
#include "scandium/navigation/jps.hpp"
#include "scandium/profiling/benchmark_harness.hpp"

using Clock = std::chrono::steady_clock;
using namespace scandium::navigation;
using scandium::bench::compute_stats;
using scandium::bench::do_not_optimize;

template <typename Search>
struct SearchResultSummary {
    double median_ms;
    double mean_ms;
    double stddev_ms;
    std::size_t nodes_explored;
    float path_cost;
    std::size_t path_length;
    bool path_found;
};

template <typename Search>
SearchResultSummary<Search> benchmark_search_topology(
    const Grid& grid, GridNode start, GridNode goal, int iterations, int trials = 5) {

    // Warm-up pass
    for (int i = 0; i < 5; ++i) {
        const auto res = Search::find_path_with_metrics(grid, start, goal);
        do_not_optimize(res.metrics.path_found);
    }

    std::vector<double> trial_times;
    trial_times.reserve(trials);
    std::size_t last_nodes = 0;
    float last_cost = 0.0f;
    std::size_t last_len = 0;
    bool found = false;

    for (int t = 0; t < trials; ++t) {
        const auto begin = Clock::now();
        std::size_t found_count = 0;

        for (int i = 0; i < iterations; ++i) {
            const auto res = Search::find_path_with_metrics(grid, start, goal);
            if (res.metrics.path_found) {
                ++found_count;
            }
            last_nodes = res.metrics.nodes_explored;
            last_cost = res.metrics.path_cost;
            last_len = res.path.size();
            found = res.metrics.path_found;
        }

        const auto end = Clock::now();
        const double duration = std::chrono::duration<double, std::milli>(end - begin).count() / iterations;
        trial_times.push_back(duration);
        do_not_optimize(found_count);
    }

    const auto stats = compute_stats(trial_times);
    return {stats.median_ms, stats.mean_ms, stats.stddev_ms, last_nodes, last_cost, last_len, found};
}

void run_navigation_benchmarks() {
    std::cout << "SCANDIUM NAVIGATION COMPREHENSIVE BENCHMARK (A* vs JPS)\n";
    std::cout << "=======================================================\n";
    std::cout << "topology,grid_size,astar_ms,jps_ms,speedup_astar_vs_jps,astar_nodes,jps_nodes,astar_cost,jps_cost,costs_match\n";

    constexpr int iterations = 100;

    // Topology 1: Open Grid 128x128 (No obstacles)
    {
        Grid grid{128, 128};
        const auto astar = benchmark_search_topology<AStar>(grid, {2, 2}, {125, 125}, iterations);
        const auto jps = benchmark_search_topology<JumpPointSearch>(grid, {2, 2}, {125, 125}, iterations);
        const double speedup = jps.median_ms > 0.0 ? astar.median_ms / jps.median_ms : 0.0;
        const bool match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "OpenGrid,128x128,"
                  << std::fixed << std::setprecision(4)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << std::setprecision(2) << (astar.median_ms > 0 ? jps.median_ms / astar.median_ms : 0.0) << "x A* faster,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << std::setprecision(1) << astar.path_cost << "," << jps.path_cost << ","
                  << (match ? "true" : "false") << "\n";
    }

    // Topology 2: Corridors 96x96 (Winding structured hallways)
    {
        Grid grid{96, 96};
        for (int y = 8; y < 88; y += 10) {
            for (int x = 12; x < 84; ++x) {
                if ((x + y) % 7 != 0) {
                    grid.set_blocked({x, y}, true);
                }
            }
        }
        const auto astar = benchmark_search_topology<AStar>(grid, {1, 1}, {94, 94}, iterations);
        const auto jps = benchmark_search_topology<JumpPointSearch>(grid, {1, 1}, {94, 94}, iterations);
        const bool match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "Corridors,96x96,"
                  << std::fixed << std::setprecision(4)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << std::setprecision(2) << (astar.median_ms > 0 ? jps.median_ms / astar.median_ms : 0.0) << "x A* faster,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << std::setprecision(1) << astar.path_cost << "," << jps.path_cost << ","
                  << (match ? "true" : "false") << "\n";
    }

    // Topology 3: Dense Labyrinth 64x64 (Alternating maze walls)
    {
        Grid grid{64, 64};
        for (int y = 4; y < 60; y += 4) {
            for (int x = 4; x < 60; ++x) {
                if (x % 6 != 0) {
                    grid.set_blocked({x, y}, true);
                }
            }
        }
        const auto astar = benchmark_search_topology<AStar>(grid, {1, 1}, {62, 62}, iterations);
        const auto jps = benchmark_search_topology<JumpPointSearch>(grid, {1, 1}, {62, 62}, iterations);
        const bool match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "DenseLabyrinth,64x64,"
                  << std::fixed << std::setprecision(4)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << std::setprecision(2) << (astar.median_ms > 0 ? jps.median_ms / astar.median_ms : 0.0) << "x A* faster,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << std::setprecision(1) << astar.path_cost << "," << jps.path_cost << ","
                  << (match ? "true" : "false") << "\n";
    }

    // Topology 4: Random Obstacles 128x128 (20% obstacle density)
    {
        Grid grid{128, 128};
        std::mt19937 rng{1337};
        std::uniform_real_distribution<float> prob{0.0f, 1.0f};

        for (int y = 0; y < 128; ++y) {
            for (int x = 0; x < 128; ++x) {
                if ((x < 5 && y < 5) || (x > 120 && y > 120)) continue;
                if (prob(rng) < 0.20f) {
                    grid.set_blocked({x, y}, true);
                }
            }
        }

        const auto astar = benchmark_search_topology<AStar>(grid, {2, 2}, {125, 125}, iterations);
        const auto jps = benchmark_search_topology<JumpPointSearch>(grid, {2, 2}, {125, 125}, iterations);
        const bool match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "RandomObstacles20%,128x128,"
                  << std::fixed << std::setprecision(4)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << std::setprecision(2) << (astar.median_ms > 0 ? jps.median_ms / astar.median_ms : 0.0) << "x A* faster,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << std::setprecision(1) << astar.path_cost << "," << jps.path_cost << ","
                  << (match ? "true" : "false") << "\n";
    }

    // Topology 5: Sparse Obstacles 128x128 (5% obstacle density)
    {
        Grid grid{128, 128};
        std::mt19937 rng{2026};
        std::uniform_real_distribution<float> prob{0.0f, 1.0f};

        for (int y = 0; y < 128; ++y) {
            for (int x = 0; x < 128; ++x) {
                if ((x < 5 && y < 5) || (x > 120 && y > 120)) continue;
                if (prob(rng) < 0.05f) {
                    grid.set_blocked({x, y}, true);
                }
            }
        }

        const auto astar = benchmark_search_topology<AStar>(grid, {2, 2}, {125, 125}, iterations);
        const auto jps = benchmark_search_topology<JumpPointSearch>(grid, {2, 2}, {125, 125}, iterations);
        const bool match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "SparseObstacles5%,128x128,"
                  << std::fixed << std::setprecision(4)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << std::setprecision(2) << (astar.median_ms > 0 ? jps.median_ms / astar.median_ms : 0.0) << "x A* faster,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << std::setprecision(1) << astar.path_cost << "," << jps.path_cost << ","
                  << (match ? "true" : "false") << "\n";
    }
}

int main() {
    run_navigation_benchmarks();
    return 0;
}
