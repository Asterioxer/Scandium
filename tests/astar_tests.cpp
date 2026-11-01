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

void test_eight_way_diagonal_navigation() {
    Grid grid{15, 15};

    // 1. Diagonal open search: from (0, 0) to (10, 10)
    const auto four_way = AStar::find_path_with_metrics(grid, {0, 0}, {10, 10}, GridConnectivity::FourWay);
    const auto eight_way = AStar::find_path_with_metrics(grid, {0, 0}, {10, 10}, GridConnectivity::EightWay);

    assert(four_way.metrics.path_found);
    assert(eight_way.metrics.path_found);

    // 4-way path is staircase (20 steps, cost 20.0)
    assert(std::fabs(four_way.metrics.path_cost - 20.0f) < 1e-4f);
    assert(four_way.path.size() == 21);

    // 8-way path is true straight diagonal (10 diagonal steps, cost ~14.142)
    const float expected_8way_cost = 10.0f * 1.41421356f;
    assert(std::fabs(eight_way.metrics.path_cost - expected_8way_cost) < 0.05f);
    assert(eight_way.path.size() == 11);

    // 2. Corner cutting disallowance test
    // Place walls at (2, 1) and (1, 2)
    Grid maze{6, 6};
    maze.set_blocked({2, 1}, true);
    maze.set_blocked({1, 2}, true);

    const auto corner_path = AStar::find_path_with_metrics(maze, {1, 1}, {2, 2}, GridConnectivity::EightWay);
    assert(corner_path.metrics.path_found);
    // Should NOT cut corner: path cost must exceed direct diagonal (must step around or take safe detour)
    assert(corner_path.metrics.path_cost > 1.5f);

    // 3. Grid 8-way neighbors helper
    const auto n8 = grid.neighbors_8way({5, 5});
    assert(n8.size() == 8);
}


#include "scandium/navigation/flow_field.hpp"

void test_flow_field_navigation() {
    using namespace scandium::navigation;

    FlowField flow{20, 20, 1.0f};

    // Place goal at (10, 10)
    flow.compute({{10, 10}});

    assert(flow.get_distance_to_goal(10, 10) == 0.0f);
    assert(flow.get_distance_to_goal(10, 9) == 1.0f);
    assert(flow.get_distance_to_goal(9, 10) == 1.0f);

    // Vector at (10, 5) should point towards y+ (south/downhill)
    const auto v_north = flow.get_flow_at_cell(10, 5);
    assert(v_north.z > 0.9f);

    // Vector at (5, 10) should point towards x+ (east/downhill)
    const auto v_west = flow.get_flow_at_cell(5, 10);
    assert(v_west.x > 0.9f);

    // Test with wall obstacle
    FlowField maze_flow{10, 10, 1.0f};
    // Wall across row y = 5 except door at x = 2
    for (int x = 0; x < 10; ++x) {
        if (x != 2) {
            maze_flow.set_blocked(x, 5, true);
        }
    }
    maze_flow.compute({{2, 8}});

    // Cell at (8, 2) must route through door at (2, 5) to reach (2, 8)
    const float dist_through_door = maze_flow.get_distance_to_goal(8, 2);
    assert(dist_through_door < FlowField::UNREACHABLE);
    assert(dist_through_door > 6.0f);

    // Bilinear continuous sample
    const auto sample_vec = flow.sample({10.0f, 0.0f, 5.0f});
    assert(sample_vec.length() > 0.9f);
}

int main() {
    test_basic_pathfinding();
    test_unreachable_and_blocked();
    test_obstacle_changes_and_dynamic_repath();
    test_path_smoothing();
    test_large_map();
    test_eight_way_diagonal_navigation();
    test_flow_field_navigation();
    std::cout << "All navigation tests passed successfully.\n";
    return 0;
}
