#include <cassert>

#include "scandium/navigation/astar.hpp"

int main() {
    using scandium::navigation::AStar;
    using scandium::navigation::Grid;
    using scandium::navigation::GridNode;

    Grid grid{8, 8};
    grid.set_blocked({2, 0}, true);
    grid.set_blocked({2, 1}, true);
    grid.set_blocked({2, 2}, true);

    const auto path = AStar::find_path(grid, {0, 0}, {5, 0});
    assert(!path.empty());
    const GridNode expected_start{0, 0};
    const GridNode expected_goal{5, 0};
    assert(path.front() == expected_start);
    assert(path.back() == expected_goal);

    for (const auto node : path) {
        assert(grid.walkable(node));
    }

    Grid impossible{3, 3};
    impossible.set_blocked({1, 0}, true);
    impossible.set_blocked({1, 1}, true);
    impossible.set_blocked({1, 2}, true);
    assert(AStar::find_path(impossible, {0, 1}, {2, 1}).empty());

    return 0;
}
