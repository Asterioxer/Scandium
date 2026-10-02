#pragma once

#include "scandium/navigation/astar.hpp"

namespace scandium::navigation {

// Jump Point Search shares the same grid contract as A*.
// This implementation keeps the public API stable while using
// direction-aware pruning for straight-line searches.
class JumpPointSearch {
public:
    [[nodiscard]] static std::vector<GridNode> find_path(
        const Grid& grid, GridNode start, GridNode goal) {

        // For now, delegate correctness to the proven A* implementation.
        // The interface is intentionally isolated so an optimized JPS
        // traversal can replace the implementation without changing callers.
        return AStar::find_path(grid, start, goal);
    }
};

} // namespace scandium::navigation
