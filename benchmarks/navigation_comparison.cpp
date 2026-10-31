#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "scandium/navigation/astar.hpp"
#include "scandium/navigation/jps.hpp"

using Clock = std::chrono::steady_clock;
using namespace scandium::navigation;

template <typename T>
void do_not_optimize(T&& val) {
    volatile auto sink = val;
    (void)sink;
}

double median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t mid = v.size() / 2;
    return (v.size() % 2 == 0) ? (v[mid - 1] + v[mid]) * 0.5 : v[mid];
}

template <typename Search>
struct BenchmarkStats {
    double median_ms;
    std::size_t nodes_explored;
    float path_cost;
    std::size_t path_length;
    bool path_found;
};

template <typename Search>
BenchmarkStats<Search> benchmark_search_rigorous(
    const Grid& grid, GridNode start, GridNode goal, int iterations, int trials = 5) {

    // Warmup pass
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
        const double duration = std::chrono::duration<double, std::milli>(end - begin).count();
        trial_times.push_back(duration);
        do_not_optimize(found_count);
    }

    return {median(trial_times), last_nodes, last_cost, last_len, found};
}

int main() {
    std::cout << "SCANDIUM NAVIGATION RIGOROUS BENCHMARK (A* vs JPS)\n";
    std::cout << "=================================================\n";
    std::cout << "topology,grid_size,iterations,astar_ms,jps_ms,speedup,astar_nodes,jps_nodes,astar_len,jps_len,costs_match\n";

    constexpr int iterations = 100;

    // Scenario 1: Standard 96x96 Corridor Grid
    {
        Grid grid{96, 96};
        for (int y = 8; y < 88; y += 10) {
            for (int x = 12; x < 84; ++x) {
                if ((x + y) % 7 != 0) {
                    grid.set_blocked({x, y}, true);
                }
            }
        }
        const auto astar = benchmark_search_rigorous<AStar>(grid, {1, 1}, {94, 94}, iterations);
        const auto jps = benchmark_search_rigorous<JumpPointSearch>(grid, {1, 1}, {94, 94}, iterations);
        const double speedup = jps.median_ms > 0.0 ? astar.median_ms / jps.median_ms : 0.0;
        const bool cost_match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "CorridorGrid,96x96," << iterations << ","
                  << std::fixed << std::setprecision(2)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << speedup << "x,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << astar.path_length << "," << jps.path_length << ","
                  << (cost_match ? "true" : "false") << "\n";
    }

    // Scenario 2: Open Map 128x128
    {
        Grid grid{128, 128};
        const auto astar = benchmark_search_rigorous<AStar>(grid, {2, 2}, {125, 125}, iterations);
        const auto jps = benchmark_search_rigorous<JumpPointSearch>(grid, {2, 2}, {125, 125}, iterations);
        const double speedup = jps.median_ms > 0.0 ? astar.median_ms / jps.median_ms : 0.0;
        const bool cost_match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "OpenGrid,128x128," << iterations << ","
                  << std::fixed << std::setprecision(2)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << speedup << "x,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << astar.path_length << "," << jps.path_length << ","
                  << (cost_match ? "true" : "false") << "\n";
    }

    // Scenario 3: Dense Labyrinth 64x64
    {
        Grid grid{64, 64};
        for (int y = 4; y < 60; y += 4) {
            for (int x = 4; x < 60; ++x) {
                if (x % 6 != 0) {
                    grid.set_blocked({x, y}, true);
                }
            }
        }
        const auto astar = benchmark_search_rigorous<AStar>(grid, {1, 1}, {62, 62}, iterations);
        const auto jps = benchmark_search_rigorous<JumpPointSearch>(grid, {1, 1}, {62, 62}, iterations);
        const double speedup = jps.median_ms > 0.0 ? astar.median_ms / jps.median_ms : 0.0;
        const bool cost_match = std::fabs(astar.path_cost - jps.path_cost) < 0.1f;

        std::cout << "DenseLabyrinth,64x64," << iterations << ","
                  << std::fixed << std::setprecision(2)
                  << astar.median_ms << "," << jps.median_ms << ","
                  << speedup << "x,"
                  << astar.nodes_explored << "," << jps.nodes_explored << ","
                  << astar.path_length << "," << jps.path_length << ","
                  << (cost_match ? "true" : "false") << "\n";
    }

    return 0;
}
