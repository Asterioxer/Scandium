#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "scandium/navigation/astar.hpp"

namespace scandium::navigation {

struct HpaMetrics {
    std::size_t abstract_nodes_explored{0};
    std::size_t concrete_nodes_explored{0};
    float path_cost{0.0f};
    double duration_us{0.0};
    bool path_found{false};
};

struct HpaPathResult {
    std::vector<GridNode> path;
    HpaMetrics metrics;
};

/**
 * Hierarchical Pathfinding A* (HPA*) for scalable navigation on large grid maps.
 * Decomposes grids into clusters, precomputes an abstract transition graph,
 * and executes hierarchical routing with local path refinement.
 */
class HierarchicalPathfinding {
public:
    struct AbstractEdge {
        std::uint32_t target_node_id{0};
        float cost{0.0f};
        std::vector<GridNode> concrete_path;
    };

    struct AbstractNode {
        std::uint32_t id{0};
        GridNode position{};
        int cluster_x{0};
        int cluster_y{0};
        std::vector<AbstractEdge> edges;
    };

    explicit HierarchicalPathfinding(int cluster_size = 16)
        : cluster_size_(cluster_size >= 4 ? cluster_size : 16) {}

    void build_graph(const Grid& grid) {
        grid_width_ = grid.width();
        grid_height_ = grid.height();
        clusters_x_ = (grid_width_ + cluster_size_ - 1) / cluster_size_;
        clusters_y_ = (grid_height_ + cluster_size_ - 1) / cluster_size_;

        nodes_.clear();
        cluster_nodes_.assign(static_cast<std::size_t>(clusters_x_ * clusters_y_), {});

        // 1. Identify inter-cluster transitions (entrances)
        build_entrances(grid);

        // 2. Compute intra-cluster paths between all pairs of entrance nodes in each cluster
        build_intra_cluster_edges(grid);
    }

    [[nodiscard]] HpaPathResult find_path_with_metrics(
        const Grid& grid, GridNode start, GridNode goal) const {

        const auto start_time = std::chrono::steady_clock::now();

        if (!grid.walkable(start) || !grid.walkable(goal)) {
            return {{}, {0, 0, 0.0f, 0.0, false}};
        }

        if (start == goal) {
            return {{start}, {0, 1, 0.0f, 0.0, true}};
        }

        // If start and goal share the same cluster, try direct local search first
        const int start_cx = start.x / cluster_size_;
        const int start_cy = start.y / cluster_size_;
        const int goal_cx = goal.x / cluster_size_;
        const int goal_cy = goal.y / cluster_size_;

        if (start_cx == goal_cx && start_cy == goal_cy) {
            const auto local_res = local_cluster_search(grid, start, goal, start_cx, start_cy);
            if (local_res.has_value()) {
                const auto end_time = std::chrono::steady_clock::now();
                const double dur = std::chrono::duration<double, std::micro>(end_time - start_time).count();
                return {*local_res, {0, local_res->size(), static_cast<float>(local_res->size() - 1), dur, true}};
            }
        }

        // Build ephemeral start and goal abstract nodes
        const std::uint32_t start_id = static_cast<std::uint32_t>(nodes_.size());
        const std::uint32_t goal_id = static_cast<std::uint32_t>(nodes_.size() + 1);

        AbstractNode temp_start{start_id, start, start_cx, start_cy, {}};
        AbstractNode temp_goal{goal_id, goal, goal_cx, goal_cy, {}};

        std::size_t concrete_explored = 0;

        // Connect temporary start node to all entrance nodes in start cluster
        const auto& start_cluster_entrances = get_cluster_nodes(start_cx, start_cy);
        for (const auto entrance_id : start_cluster_entrances) {
            const auto& ent = nodes_[entrance_id];
            const auto p = local_cluster_search(grid, start, ent.position, start_cx, start_cy);
            if (p.has_value()) {
                concrete_explored += p->size();
                const float cost = static_cast<float>(p->size() - 1);
                temp_start.edges.push_back({entrance_id, cost, *p});
            }
        }

        // Connect entrance nodes in goal cluster to temporary goal node
        const auto& goal_cluster_entrances = get_cluster_nodes(goal_cx, goal_cy);
        for (const auto entrance_id : goal_cluster_entrances) {
            const auto& ent = nodes_[entrance_id];
            const auto p = local_cluster_search(grid, ent.position, goal, goal_cx, goal_cy);
            if (p.has_value()) {
                concrete_explored += p->size();
                const float cost = static_cast<float>(p->size() - 1);
                temp_goal.edges.push_back({entrance_id, cost, *p});
            }
        }

        // A* search on the abstract graph
        struct SearchItem {
            std::uint32_t node_id;
            float f_score;
        };

        struct Compare {
            bool operator()(const SearchItem& a, const SearchItem& b) const noexcept {
                return a.f_score > b.f_score;
            }
        };

        std::priority_queue<SearchItem, std::vector<SearchItem>, Compare> open;
        std::unordered_map<std::uint32_t, float> g_scores;
        std::unordered_map<std::uint32_t, std::uint32_t> came_from;
        std::unordered_map<std::uint64_t, std::vector<GridNode>> edge_paths;

        auto heuristic = [&](GridNode a, GridNode b) noexcept -> float {
            return static_cast<float>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
        };

        g_scores[start_id] = 0.0f;
        open.push({start_id, heuristic(start, goal)});

        std::size_t abstract_explored = 0;
        bool found = false;

        auto get_node_pos = [&](std::uint32_t id) -> GridNode {
            if (id == start_id) return start;
            if (id == goal_id) return goal;
            return nodes_[id].position;
        };

        while (!open.empty()) {
            const auto [curr_id, f] = open.top();
            open.pop();

            if (curr_id == goal_id) {
                found = true;
                break;
            }

            ++abstract_explored;
            const float curr_g = g_scores[curr_id];

            // Determine outgoing edges
            std::vector<AbstractEdge> outgoing;
            if (curr_id == start_id) {
                outgoing = temp_start.edges;
            } else {
                outgoing = nodes_[curr_id].edges;
                // If in goal cluster, edge directly to goal node
                for (const auto& g_edge : temp_goal.edges) {
                    if (g_edge.target_node_id == curr_id) {
                        outgoing.push_back({goal_id, g_edge.cost, g_edge.concrete_path});
                    }
                }
            }

            for (const auto& edge : outgoing) {
                const float tent_g = curr_g + edge.cost;
                const auto next_id = edge.target_node_id;

                if (!g_scores.contains(next_id) || tent_g < g_scores[next_id]) {
                    g_scores[next_id] = tent_g;
                    came_from[next_id] = curr_id;

                    const std::uint64_t key = (static_cast<std::uint64_t>(curr_id) << 32) | next_id;
                    edge_paths[key] = edge.concrete_path;

                    const float next_f = tent_g + heuristic(get_node_pos(next_id), goal);
                    open.push({next_id, next_f});
                }
            }
        }

        const auto end_time = std::chrono::steady_clock::now();
        const double duration_us =
            std::chrono::duration<double, std::micro>(end_time - start_time).count();

        if (!found) {
            // Fall back to direct flat A* if coarse abstraction partitioned start/goal unreachable entrances
            const auto flat = AStar::find_path_with_metrics(grid, start, goal);
            if (flat.metrics.path_found) {
                return {flat.path, {abstract_explored, flat.metrics.nodes_explored, flat.metrics.path_cost, duration_us, true}};
            }
            return {{}, {abstract_explored, concrete_explored, 0.0f, duration_us, false}};
        }

        // Reconstruct concrete path from abstract path
        std::vector<std::uint32_t> path_nodes;
        std::uint32_t curr = goal_id;
        while (curr != start_id) {
            path_nodes.push_back(curr);
            curr = came_from[curr];
        }
        path_nodes.push_back(start_id);
        std::reverse(path_nodes.begin(), path_nodes.end());

        std::vector<GridNode> full_path;
        for (std::size_t i = 0; i + 1 < path_nodes.size(); ++i) {
            const std::uint32_t u = path_nodes[i];
            const std::uint32_t v = path_nodes[i + 1];
            const std::uint64_t key = (static_cast<std::uint64_t>(u) << 32) | v;

            const auto& segment = edge_paths[key];
            if (!segment.empty()) {
                if (full_path.empty()) {
                    full_path.insert(full_path.end(), segment.begin(), segment.end());
                } else {
                    full_path.insert(full_path.end(), segment.begin() + 1, segment.end());
                }
            }
        }

        const float cost = g_scores[goal_id];
        return {std::move(full_path), {abstract_explored, concrete_explored, cost, duration_us, true}};
    }

    [[nodiscard]] std::vector<GridNode> find_path(
        const Grid& grid, GridNode start, GridNode goal) const {
        return find_path_with_metrics(grid, start, goal).path;
    }

    [[nodiscard]] std::size_t abstract_node_count() const noexcept {
        return nodes_.size();
    }

    [[nodiscard]] int cluster_size() const noexcept {
        return cluster_size_;
    }

private:
    [[nodiscard]] const std::vector<std::uint32_t>& get_cluster_nodes(int cx, int cy) const noexcept {
        static const std::vector<std::uint32_t> empty;
        if (cx < 0 || cx >= clusters_x_ || cy < 0 || cy >= clusters_y_) {
            return empty;
        }
        return cluster_nodes_[static_cast<std::size_t>(cy * clusters_x_ + cx)];
    }

    void build_entrances(const Grid& grid) {
        // Horizontal cluster borders
        for (int cy = 0; cy < clusters_y_; ++cy) {
            for (int cx = 0; cx < clusters_x_ - 1; ++cx) {
                const int x1 = (cx + 1) * cluster_size_ - 1;
                const int x2 = x1 + 1;
                const int y_start = cy * cluster_size_;
                const int y_end = std::min(grid_height_, (cy + 1) * cluster_size_);

                int seg_start = -1;
                for (int y = y_start; y < y_end; ++y) {
                    if (grid.walkable(x1, y) && grid.walkable(x2, y)) {
                        if (seg_start == -1) seg_start = y;
                    } else {
                        if (seg_start != -1) {
                            add_entrance_pair(x1, x2, seg_start, y - 1, cx, cx + 1, cy, cy);
                            seg_start = -1;
                        }
                    }
                }
                if (seg_start != -1) {
                    add_entrance_pair(x1, x2, seg_start, y_end - 1, cx, cx + 1, cy, cy);
                }
            }
        }

        // Vertical cluster borders
        for (int cx = 0; cx < clusters_x_; ++cx) {
            for (int cy = 0; cy < clusters_y_ - 1; ++cy) {
                const int y1 = (cy + 1) * cluster_size_ - 1;
                const int y2 = y1 + 1;
                const int x_start = cx * cluster_size_;
                const int x_end = std::min(grid_width_, (cx + 1) * cluster_size_);

                int seg_start = -1;
                for (int x = x_start; x < x_end; ++x) {
                    if (grid.walkable(x, y1) && grid.walkable(x, y2)) {
                        if (seg_start == -1) seg_start = x;
                    } else {
                        if (seg_start != -1) {
                            add_entrance_pair_vertical(x_start, x - 1, y1, y2, cx, cx, cy, cy + 1);
                            seg_start = -1;
                        }
                    }
                }
                if (seg_start != -1) {
                    add_entrance_pair_vertical(seg_start, x_end - 1, y1, y2, cx, cx, cy, cy + 1);
                }
            }
        }
    }

    void add_entrance_pair(int x1, int x2, int y_start, int y_end,
                           int cx1, int cx2, int cy1, int cy2) {
        const int mid_y = (y_start + y_end) / 2;

        const std::uint32_t id1 = static_cast<std::uint32_t>(nodes_.size());
        nodes_.push_back({id1, {x1, mid_y}, cx1, cy1, {}});
        cluster_nodes_[static_cast<std::size_t>(cy1 * clusters_x_ + cx1)].push_back(id1);

        const std::uint32_t id2 = static_cast<std::uint32_t>(nodes_.size());
        nodes_.push_back({id2, {x2, mid_y}, cx2, cy2, {}});
        cluster_nodes_[static_cast<std::size_t>(cy2 * clusters_x_ + cx2)].push_back(id2);

        // Inter-edge cost 1.0 (adjacent cells)
        nodes_[id1].edges.push_back({id2, 1.0f, {{x1, mid_y}, {x2, mid_y}}});
        nodes_[id2].edges.push_back({id1, 1.0f, {{x2, mid_y}, {x1, mid_y}}});
    }

    void add_entrance_pair_vertical(int x_start, int x_end, int y1, int y2,
                                    int cx1, int cx2, int cy1, int cy2) {
        const int mid_x = (x_start + x_end) / 2;

        const std::uint32_t id1 = static_cast<std::uint32_t>(nodes_.size());
        nodes_.push_back({id1, {mid_x, y1}, cx1, cy1, {}});
        cluster_nodes_[static_cast<std::size_t>(cy1 * clusters_x_ + cx1)].push_back(id1);

        const std::uint32_t id2 = static_cast<std::uint32_t>(nodes_.size());
        nodes_.push_back({id2, {mid_x, y2}, cx2, cy2, {}});
        cluster_nodes_[static_cast<std::size_t>(cy2 * clusters_x_ + cx2)].push_back(id2);

        // Inter-edge cost 1.0 (adjacent cells)
        nodes_[id1].edges.push_back({id2, 1.0f, {{mid_x, y1}, {mid_x, y2}}});
        nodes_[id2].edges.push_back({id1, 1.0f, {{mid_x, y2}, {mid_x, y1}}});
    }

    void build_intra_cluster_edges(const Grid& grid) {
        for (int cy = 0; cy < clusters_y_; ++cy) {
            for (int cx = 0; cx < clusters_x_; ++cx) {
                const auto& ents = get_cluster_nodes(cx, cy);
                const std::size_t n = ents.size();
                for (std::size_t i = 0; i < n; ++i) {
                    for (std::size_t j = i + 1; j < n; ++j) {
                        const auto u = ents[i];
                        const auto v = ents[j];
                        const auto path = local_cluster_search(grid, nodes_[u].position, nodes_[v].position, cx, cy);
                        if (path.has_value()) {
                            const float cost = static_cast<float>(path->size() - 1);
                            nodes_[u].edges.push_back({v, cost, *path});

                            auto rev_path = *path;
                            std::reverse(rev_path.begin(), rev_path.end());
                            nodes_[v].edges.push_back({u, cost, std::move(rev_path)});
                        }
                    }
                }
            }
        }
    }

    [[nodiscard]] std::optional<std::vector<GridNode>> local_cluster_search(
        const Grid& grid, GridNode from, GridNode to, int cx, int cy) const {

        const int min_x = cx * cluster_size_;
        const int max_x = std::min(grid_width_ - 1, (cx + 1) * cluster_size_ - 1);
        const int min_y = cy * cluster_size_;
        const int max_y = std::min(grid_height_ - 1, (cy + 1) * cluster_size_ - 1);

        if (!grid.walkable(from) || !grid.walkable(to)) return std::nullopt;
        if (from == to) return std::vector<GridNode>{from};

        // Bounded cluster local A*
        struct LItem { GridNode node; float f; };
        struct LComp {
            bool operator()(const LItem& a, const LItem& b) const noexcept { return a.f > b.f; }
        };

        std::priority_queue<LItem, std::vector<LItem>, LComp> open;
        std::unordered_map<GridNode, float, GridNodeHash> g_score;
        std::unordered_map<GridNode, GridNode, GridNodeHash> came_from;

        const auto heuristic = [](GridNode a, GridNode b) noexcept {
            return static_cast<float>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
        };

        g_score[from] = 0.0f;
        open.push({from, heuristic(from, to)});

        static constexpr GridNode cardinal_dirs[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        while (!open.empty()) {
            const auto [curr, f] = open.top();
            open.pop();

            if (curr == to) {
                std::vector<GridNode> path;
                GridNode step = to;
                while (step != from) {
                    path.push_back(step);
                    step = came_from[step];
                }
                path.push_back(from);
                std::reverse(path.begin(), path.end());
                return path;
            }

            const float curr_g = g_score[curr];

            for (const auto d : cardinal_dirs) {
                const GridNode next{curr.x + d.x, curr.y + d.y};
                if (next.x >= min_x && next.x <= max_x &&
                    next.y >= min_y && next.y <= max_y &&
                    grid.walkable(next)) {

                    const float tent_g = curr_g + 1.0f;
                    if (!g_score.contains(next) || tent_g < g_score[next]) {
                        g_score[next] = tent_g;
                        came_from[next] = curr;
                        open.push({next, tent_g + heuristic(next, to)});
                    }
                }
            }
        }

        return std::nullopt;
    }

    int cluster_size_{16};
    int grid_width_{0};
    int grid_height_{0};
    int clusters_x_{0};
    int clusters_y_{0};
    std::vector<AbstractNode> nodes_;
    std::vector<std::vector<std::uint32_t>> cluster_nodes_;
};

} // namespace scandium::navigation
