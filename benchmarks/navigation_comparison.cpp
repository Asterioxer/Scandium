#include <chrono>
#include <cstddef>
#include <iostream>

#include "scandium/navigation/astar.hpp"
#include "scandium/navigation/jps.hpp"

namespace {
template <typename Search>
double benchmark(const scandium::navigation::Grid& grid,
                 scandium::navigation::GridNode start,
                 scandium::navigation::GridNode goal,
                 int iterations) {
    using clock = std::chrono::steady_clock;
    std::size_t path_cells = 0;
    const auto begin = clock::now();

    for (int i = 0; i < iterations; ++i) {
        const auto path = Search::find_path(grid, start, goal);
        path_cells += path.size();
    }

    const auto end = clock::now();
    const auto elapsed =
        std::chrono::duration<double, std::milli>(end - begin).count();

    // Keep the result observable so the optimizer cannot remove the searches.
    if (path_cells == 0 && iterations > 0) {
        std::cerr << "no path found\n";
    }
    return elapsed;
}
}

int main() {
    using namespace scandium::navigation;

    Grid grid{96, 96};
    for (int y = 8; y < 88; y += 10) {
        for (int x = 12; x < 84; ++x) {
            if ((x + y) % 7 != 0) {
                grid.set_blocked({x, y}, true);
            }
        }
    }

    constexpr int iterations = 100;
    const GridNode start{1, 1};
    const GridNode goal{94, 94};

    const double astar_ms = benchmark<AStar>(grid, start, goal, iterations);
    const double jps_ms =
        benchmark<JumpPointSearch>(grid, start, goal, iterations);

    std::cout << "navigation,astar_ms,jps_ms\n";
    std::cout << iterations << "," << astar_ms << "," << jps_ms << "\n";
    return 0;
}
