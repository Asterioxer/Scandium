#include <cassert>
#include <cmath>
#include <iostream>

#include "scandium/navigation/astar.hpp"
#include "scandium/navigation/jps.hpp"
#include "scandium/navigation/hpa.hpp"
#include "scandium/navigation/flow_field.hpp"

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
    const auto raw_path = AStar::find_path(grid, {1, 1}, {10, 10});
    assert(!raw_path.empty());

    const auto smoothed = AStar::smooth_path(grid, raw_path);
    assert(!smoothed.empty());
    assert(smoothed.front() == GridNode(1, 1));
    assert(smoothed.back() == GridNode(10, 10));
    assert(smoothed.size() <= raw_path.size());
}

void test_large_map() {
    Grid grid{128, 128};
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

    const auto four_way = AStar::find_path_with_metrics(grid, {0, 0}, {10, 10}, GridConnectivity::FourWay);
    const auto eight_way = AStar::find_path_with_metrics(grid, {0, 0}, {10, 10}, GridConnectivity::EightWay);

    assert(four_way.metrics.path_found);
    assert(eight_way.metrics.path_found);

    assert(std::fabs(four_way.metrics.path_cost - 20.0f) < 1e-4f);
    assert(four_way.path.size() == 21);

    const float expected_8way_cost = 10.0f * 1.41421356f;
    assert(std::fabs(eight_way.metrics.path_cost - expected_8way_cost) < 0.05f);
    assert(eight_way.path.size() == 11);

    Grid maze{6, 6};
    maze.set_blocked({2, 1}, true);
    maze.set_blocked({1, 2}, true);

    const auto corner_path = AStar::find_path_with_metrics(maze, {1, 1}, {2, 2}, GridConnectivity::EightWay);
    assert(corner_path.metrics.path_found);
    assert(corner_path.metrics.path_cost > 1.5f);

    const auto n8 = grid.neighbors_8way({5, 5});
    assert(n8.size() == 8);
}

void test_flow_field_navigation() {
    FlowField flow{20, 20, 1.0f};
    flow.compute({{10, 10}});

    assert(flow.get_distance_to_goal(10, 10) == 0.0f);
    assert(flow.get_distance_to_goal(10, 9) == 1.0f);
    assert(flow.get_distance_to_goal(9, 10) == 1.0f);

    const auto v_north = flow.get_flow_at_cell(10, 5);
    assert(v_north.z > 0.9f);

    const auto v_west = flow.get_flow_at_cell(5, 10);
    assert(v_west.x > 0.9f);

    FlowField maze_flow{10, 10, 1.0f};
    for (int x = 0; x < 10; ++x) {
        if (x != 2) {
            maze_flow.set_blocked(x, 5, true);
        }
    }
    maze_flow.compute({{2, 8}});

    const float dist_through_door = maze_flow.get_distance_to_goal(8, 2);
    assert(dist_through_door < FlowField::UNREACHABLE);
    assert(dist_through_door > 6.0f);

    const auto sample_vec = flow.sample({10.0f, 0.0f, 5.0f});
    assert(sample_vec.length() > 0.9f);
}

void test_hierarchical_pathfinding() {
    std::cout << "Starting test_hierarchical_pathfinding..." << std::endl;

    // 1. 64x64 Grid with cluster size 16 (4x4 = 16 clusters)
    Grid grid{64, 64};
    HierarchicalPathfinding hpa{16};
    hpa.build_graph(grid);

    assert(hpa.abstract_node_count() > 0);
    assert(hpa.cluster_size() == 16);

    const GridNode start{2, 2};
    const GridNode goal{60, 60};

    const auto astar_res = AStar::find_path_with_metrics(grid, start, goal);
    const auto hpa_res = hpa.find_path_with_metrics(grid, start, goal);

    assert(astar_res.metrics.path_found);
    assert(hpa_res.metrics.path_found);
    assert(!hpa_res.path.empty());
    assert(hpa_res.path.front() == start);
    assert(hpa_res.path.back() == goal);

    // Verify all steps are walkable and contiguous
    for (std::size_t i = 1; i < hpa_res.path.size(); ++i) {
        assert(grid.walkable(hpa_res.path[i]));
        const int dist = std::abs(hpa_res.path[i].x - hpa_res.path[i - 1].x) +
                         std::abs(hpa_res.path[i].y - hpa_res.path[i - 1].y);
        assert(dist == 1);
    }

    // Bounded sub-optimality check: HPA* path cost must be within 15% of exact A*
    const float ratio = hpa_res.metrics.path_cost / astar_res.metrics.path_cost;
    assert(ratio >= 0.99f && ratio <= 1.15f);

    // 2. Doorway cluster wall test
    Grid walled{64, 64};
    // Wall across x = 32 with single doorway at y = 16
    for (int y = 0; y < 64; ++y) {
        if (y != 16) {
            walled.set_blocked(32, y, true);
        }
    }
    HierarchicalPathfinding hpa_walled{16};
    hpa_walled.build_graph(walled);

    const auto walled_path = hpa_walled.find_path(walled, {10, 16}, {50, 16});
    assert(!walled_path.empty());
    assert(walled_path.front() == GridNode(10, 16));
    assert(walled_path.back() == GridNode(50, 16));

    bool passed_door = false;
    for (const auto n : walled_path) {
        if (n == GridNode(32, 16)) {
            passed_door = true;
            break;
        }
    }
    assert(passed_door);

    std::cout << "Passed test_hierarchical_pathfinding." << std::endl;
}

int main() {
    test_basic_pathfinding();
    test_unreachable_and_blocked();
    test_obstacle_changes_and_dynamic_repath();
    test_path_smoothing();
    test_large_map();
    test_eight_way_diagonal_navigation();
    test_flow_field_navigation();
    test_hierarchical_pathfinding();
    std::cout << "All navigation tests passed successfully.\n";
    return 0;
}
