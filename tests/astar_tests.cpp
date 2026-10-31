#include <cassert>
#include <iostream>

#include "scandium/navigation/astar.hpp"
#include "scandium/navigation/jps.hpp"

using namespace scandium::navigation;

void test_basic_pathfinding() {
    Grid grid{8, 8};
    grid.set_blocked({2, 0}, true);
    grid.set_blocked({2, 1}, true);
    grid.set_blocked({2, 2}, true);

    const auto astar_res = AStar::find_path_with_metrics(grid, {0, 0}, {5, 0});
    assert(astar_res.metrics.path_found);
    assert(!astar_res.path.empty());
    assert(astar_res.path.front() == GridNode(0, 0));
    assert(astar_res.path.back() == GridNode(5, 0));
    assert(astar_res.metrics.nodes_explored > 0);
    assert(astar_res.metrics.path_cost > 0.0f);

    for (const auto node : astar_res.path) {
        assert(grid.walkable(node));
    }

    const auto jps_res = JumpPointSearch::find_path_with_metrics(grid, {0, 0}, {5, 0});
    assert(jps_res.metrics.path_found);
    assert(!jps_res.path.empty());
    assert(jps_res.path.front() == GridNode(0, 0));
    assert(jps_res.path.back() == GridNode(5, 0));
    assert(jps_res.path.size() == astar_res.path.size());

    for (const auto node : jps_res.path) {
        assert(grid.walkable(node));
    }
}

void test_unreachable_and_blocked() {
    Grid grid{10, 10};

    // Completely surround the goal
    grid.set_blocked({4, 5}, true);
    grid.set_blocked({6, 5}, true);
    grid.set_blocked({5, 4}, true);
    grid.set_blocked({5, 6}, true);

    const auto astar_res = AStar::find_path_with_metrics(grid, {0, 0}, {5, 5});
    assert(!astar_res.metrics.path_found);
    assert(astar_res.path.empty());

    const auto jps_res = JumpPointSearch::find_path_with_metrics(grid, {0, 0}, {5, 5});
    assert(!jps_res.metrics.path_found);
    assert(jps_res.path.empty());

    // Blocked start
    grid.set_blocked({0, 0}, true);
    assert(AStar::find_path(grid, {0, 0}, {9, 9}).empty());
    assert(JumpPointSearch::find_path(grid, {0, 0}, {9, 9}).empty());

    // Out of bounds start/goal
    assert(AStar::find_path(grid, {-1, 0}, {5, 5}).empty());
    assert(AStar::find_path(grid, {0, 0}, {15, 15}).empty());
}

void test_obstacle_changes_and_dynamic_repath() {
    Grid grid{20, 20};
    const GridNode start{2, 2};
    const GridNode goal{18, 18};

    // Free path
    auto path = AStar::find_path(grid, start, goal);
    assert(!path.empty());

    // Add wall blocking the path completely
    for (int y = 0; y < 20; ++y) {
        grid.set_blocked({10, y}, true);
    }
    assert(AStar::find_path(grid, start, goal).empty());

    // Open a single doorway in the wall
    grid.set_blocked({10, 10}, false);
    path = AStar::find_path(grid, start, goal);
    assert(!path.empty());

    bool passed_doorway = false;
    for (const auto node : path) {
        if (node == GridNode(10, 10)) {
            passed_doorway = true;
            break;
        }
    }
    assert(passed_doorway);

    // Clear obstacles
    grid.clear_obstacles();
    const auto clear_path = AStar::find_path(grid, start, goal);
    assert(!clear_path.empty());
    assert(clear_path.size() < path.size());
}

void test_path_smoothing() {
    Grid grid{15, 15};
    // No obstacles: A* will produce Manhattan staircase path
    const auto raw_path = AStar::find_path(grid, {1, 1}, {10, 10});
    assert(!raw_path.empty());

    const auto smoothed = AStar::smooth_path(grid, raw_path);
    assert(!smoothed.empty());
    assert(smoothed.front() == GridNode(1, 1));
    assert(smoothed.back() == GridNode(10, 10));
    // Since direct line of sight is clear, smoothed path should have far fewer waypoints
    assert(smoothed.size() <= raw_path.size());
}

void test_large_map() {
    Grid grid{128, 128};
    // Sparse obstacle field
    for (int y = 10; y < 118; y += 8) {
        for (int x = 10; x < 118; ++x) {
            if ((x + y) % 5 != 0) {
                grid.set_blocked({x, y}, true);
            }
        }
    }

    const auto result = AStar::find_path_with_metrics(grid, {2, 2}, {125, 125});
    assert(result.metrics.path_found);
    assert(!result.path.empty());
    assert(result.path.front() == GridNode(2, 2));
    assert(result.path.back() == GridNode(125, 125));
}

int main() {
    test_basic_pathfinding();
    test_unreachable_and_blocked();
    test_obstacle_changes_and_dynamic_repath();
    test_path_smoothing();
    test_large_map();
    std::cout << "All navigation tests passed successfully.\n";
    return 0;
}
