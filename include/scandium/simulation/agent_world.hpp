#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "scandium/ai/steering.hpp"
#include "scandium/core/job_system.hpp"
#include "scandium/math/vec3.hpp"
#include "scandium/navigation/astar.hpp"
#include "scandium/navigation/jps.hpp"
#include "scandium/physics/aabb.hpp"
#include "scandium/simulation/simulation_stats.hpp"
#include "scandium/spatial/spatial_hash.hpp"

namespace scandium::simulation {

enum class AgentState : std::uint8_t {
    Idle,
    Seek,
    Flee,
    Navigating,
    Evading,
    Arrived
};

struct Agent {
    std::uint32_t id{};
    math::Vec3f position{};
    math::Vec3f previous_position{};
    math::Vec3f velocity{};
    AgentState state{AgentState::Idle};
    float max_speed{5.0f};
    float radius{0.5f};
    math::Vec3f target{};
    std::vector<navigation::GridNode> waypoints{};
    std::size_t current_waypoint_index{0};
    bool has_active_path{false};
    bool alive{true};
};

struct Obstacle {
    math::Vec3f position{};
    float radius{1.0f};
};

struct AgentWorldSnapshot {
    std::uint64_t tick{0};
    std::vector<Agent> agents{};
    std::vector<Obstacle> obstacles{};
};

struct WorldConfig {
    bool enable_collision_avoidance{false};
    bool enable_obstacle_avoidance{false};
    float separation_radius{2.0f};
    float separation_weight{1.5f};
    float waypoint_reach_distance{1.0f};
    float grid_cell_world_size{1.0f};
    math::Vec3f grid_origin{0.0f, 0.0f, 0.0f};
    spatial::BroadphaseStrategy broadphase_strategy{spatial::BroadphaseStrategy::Auto};
    std::size_t spatial_hash_threshold{150};
    std::size_t parallel_threshold{2000};
    std::size_t min_agents_per_chunk{512};
};

class AgentWorld {
public:
    explicit AgentWorld(float cell_size)
        : spatial_hash_(cell_size) {}

    void add_agent(Agent agent) {
        agents_.push_back(agent);
        next_agents_.push_back(agent);
    }

    void clear_agents() noexcept {
        agents_.clear();
        next_agents_.clear();
        spatial_hash_.clear();
    }

    void add_obstacle(Obstacle obstacle) {
        obstacles_.push_back(obstacle);
    }

    void clear_obstacles() noexcept {
        obstacles_.clear();
    }

    [[nodiscard]] const std::vector<Obstacle>& obstacles() const noexcept {
        return obstacles_;
    }

    [[nodiscard]] AgentWorldSnapshot capture_snapshot(std::uint64_t tick = 0) const {
        return AgentWorldSnapshot{tick, agents_, obstacles_};
    }

    void restore_snapshot(const AgentWorldSnapshot& snapshot) {
        agents_ = snapshot.agents;
        next_agents_ = snapshot.agents;
        obstacles_ = snapshot.obstacles;
    }

    WorldConfig& config() noexcept {
        return config_;
    }

    [[nodiscard]] const WorldConfig& config() const noexcept {
        return config_;
    }

    void set_navigation_grid(navigation::Grid grid) {
        grid_ = std::move(grid);
    }

    [[nodiscard]] const std::optional<navigation::Grid>& navigation_grid() const noexcept {
        return grid_;
    }

    bool navigate_agent_to(std::uint32_t agent_id, const math::Vec3f& world_target, bool use_jps = false) {
        if (!grid_.has_value()) {
            return false;
        }

        auto* agent = find_agent(agent_id);
        if (!agent) {
            return false;
        }

        const auto start_node = world_to_grid(agent->position);
        const auto goal_node = world_to_grid(world_target);

        std::vector<navigation::GridNode> path = use_jps
            ? navigation::JumpPointSearch::find_path(*grid_, start_node, goal_node)
            : navigation::AStar::find_path(*grid_, start_node, goal_node);

        if (path.empty()) {
            return false;
        }

        agent->waypoints = navigation::AStar::smooth_path(*grid_, path);
        agent->current_waypoint_index = 0;
        agent->has_active_path = true;
        agent->target = world_target;
        agent->state = AgentState::Navigating;
        return true;
    }

    void update(float dt, const math::Vec3f& target) {
        sync_buffers();
        rebuild_spatial_hash();

        for (std::size_t i = 0; i < agents_.size(); ++i) {
            update_agent(agents_[i], next_agents_[i], dt, target, agents_, spatial_hash_, obstacles_, config_);
        }

        agents_.swap(next_agents_);
    }

    void update_parallel(float dt, const math::Vec3f& target, core::JobSystem& jobs) {
        sync_buffers();
        rebuild_spatial_hash();

        const auto& read_agents = agents_;
        const auto& spatial = spatial_hash_;
        const auto& obs = obstacles_;
        const auto& cfg = config_;

        if (agents_.size() < config_.parallel_threshold || jobs.worker_count() <= 1) {
            for (std::size_t index = 0; index < agents_.size(); ++index) {
                update_agent(read_agents[index], next_agents_[index], dt, target, read_agents, spatial, obs, cfg);
            }
        } else {
            const std::size_t chunk_size = std::max(config_.min_agents_per_chunk,
                (agents_.size() + jobs.worker_count() - 1) / jobs.worker_count());
            jobs.parallel_for_chunked(agents_.size(), chunk_size, [&](std::size_t index) {
                update_agent(read_agents[index], next_agents_[index], dt, target, read_agents, spatial, obs, cfg);
            });
        }

        agents_.swap(next_agents_);
    }

    [[nodiscard]] const std::vector<Agent>& agents() const noexcept {
        return agents_;
    }

    [[nodiscard]] std::vector<Agent>& agents_mutable() noexcept {
        return agents_;
    }

    [[nodiscard]] math::Vec3f interpolated_position(
        const Agent& agent, float alpha) const noexcept {
        return agent.previous_position +
               (agent.position - agent.previous_position) * alpha;
    }

    [[nodiscard]] std::vector<std::uint32_t> nearby(
        const math::Vec3f& position) const {
        return spatial_hash_.query(position);
    }

    [[nodiscard]] const spatial::SpatialHash& spatial_hash() const noexcept {
        return spatial_hash_;
    }

    [[nodiscard]] SimulationStats stats() const noexcept {
        SimulationStats result;
        result.agent_count = agents_.size();

        for (const auto& agent : agents_) {
            if (agent.state == AgentState::Seek || agent.state == AgentState::Navigating) {
                ++result.seek_count;
            } else if (agent.state == AgentState::Idle || agent.state == AgentState::Arrived) {
                ++result.idle_count;
            }
        }

        return result;
    }

    [[nodiscard]] math::Vec3f grid_to_world(navigation::GridNode node) const noexcept {
        return {
            config_.grid_origin.x + (static_cast<float>(node.x) + 0.5f) * config_.grid_cell_world_size,
            config_.grid_origin.y,
            config_.grid_origin.z + (static_cast<float>(node.y) + 0.5f) * config_.grid_cell_world_size
        };
    }

    [[nodiscard]] navigation::GridNode world_to_grid(const math::Vec3f& pos) const noexcept {
        return {
            static_cast<int>(std::floor((pos.x - config_.grid_origin.x) / config_.grid_cell_world_size)),
            static_cast<int>(std::floor((pos.z - config_.grid_origin.z) / config_.grid_cell_world_size))
        };
    }

private:
    Agent* find_agent(std::uint32_t id) noexcept {
        for (auto& a : agents_) {
            if (a.id == id) return &a;
        }
        return nullptr;
    }

    void sync_buffers() {
        if (next_agents_.size() != agents_.size()) {
            next_agents_.resize(agents_.size());
        }
    }

    void rebuild_spatial_hash() {
        if (!config_.enable_collision_avoidance) {
            return;
        }
        if (config_.broadphase_strategy == spatial::BroadphaseStrategy::Naive ||
            (config_.broadphase_strategy == spatial::BroadphaseStrategy::Auto &&
             agents_.size() < config_.spatial_hash_threshold)) {
            return;
        }
        spatial_hash_.clear();
        for (const auto& agent : agents_) {
            spatial_hash_.insert(agent.id, agent.position);
        }
    }

    static void update_agent(const Agent& current, Agent& next, float dt,
                             const math::Vec3f& default_target,
                             const std::vector<Agent>& all_agents,
                             const spatial::SpatialHash& spatial,
                             const std::vector<Obstacle>& obstacles,
                             const WorldConfig& config) noexcept {
        next = current;
        next.previous_position = current.position;

        math::Vec3f desired_velocity{};
        bool active_movement = false;

        // Path navigation takes priority if active
        if (current.has_active_path && !current.waypoints.empty()) {
            math::Vec3f target_wp{};
            const auto wp_node = current.waypoints[current.current_waypoint_index];
            target_wp = {
                config.grid_origin.x + (static_cast<float>(wp_node.x) + 0.5f) * config.grid_cell_world_size,
                current.position.y,
                config.grid_origin.z + (static_cast<float>(wp_node.y) + 0.5f) * config.grid_cell_world_size
            };

            const float dist_to_wp = (target_wp - current.position).length();
            if (dist_to_wp <= config.waypoint_reach_distance) {
                if (current.current_waypoint_index + 1 < current.waypoints.size()) {
                    next.current_waypoint_index = current.current_waypoint_index + 1;
                    const auto next_wp_node = current.waypoints[next.current_waypoint_index];
                    target_wp = {
                        config.grid_origin.x + (static_cast<float>(next_wp_node.x) + 0.5f) * config.grid_cell_world_size,
                        current.position.y,
                        config.grid_origin.z + (static_cast<float>(next_wp_node.y) + 0.5f) * config.grid_cell_world_size
                    };
                } else {
                    // Reached destination
                    next.has_active_path = false;
                    next.state = AgentState::Arrived;
                    next.velocity = {};
                    return;
                }
            }

            next.state = AgentState::Navigating;
            math::Vec3f steer_dir = (target_wp - current.position).normalized();
            desired_velocity = steer_dir * current.max_speed;
            active_movement = true;
        } else {
            // Standard goal seek
            const math::Vec3f delta = default_target - current.position;
            const float distance_sq = delta.length_squared();

            next.state = distance_sq < 4.0f ? AgentState::Idle : AgentState::Seek;

            if (next.state == AgentState::Seek) {
                desired_velocity = delta.normalized() * current.max_speed;
                active_movement = true;
            } else {
                next.velocity = {};
                return;
            }
        }

        if (active_movement) {
            // Optional agent-agent collision avoidance
            if (config.enable_collision_avoidance) {
                math::Vec3f separation_force{};
                int count = 0;
                const float sep_sq = config.separation_radius * config.separation_radius;

                const bool use_spatial = (config.broadphase_strategy == spatial::BroadphaseStrategy::SpatialHash) ||
                                         (config.broadphase_strategy == spatial::BroadphaseStrategy::Auto &&
                                          all_agents.size() >= config.spatial_hash_threshold);

                if (use_spatial) {
                    spatial.query(current.position, [&](std::uint32_t neighbor_id) {
                        if (neighbor_id != current.id && neighbor_id < all_agents.size()) {
                            const auto& other = all_agents[neighbor_id];
                            const math::Vec3f diff = current.position - other.position;
                            const float d_sq = diff.length_squared();
                            if (d_sq > 1e-5f && d_sq < sep_sq) {
                                const float d = std::sqrt(d_sq);
                                separation_force += (diff / d) * (config.separation_radius - d);
                                ++count;
                            }
                        }
                    });
                } else {
                    for (const auto& other : all_agents) {
                        if (other.id != current.id) {
                            const math::Vec3f diff = current.position - other.position;
                            const float d_sq = diff.length_squared();
                            if (d_sq > 1e-5f && d_sq < sep_sq) {
                                const float d = std::sqrt(d_sq);
                                separation_force += (diff / d) * (config.separation_radius - d);
                                ++count;
                            }
                        }
                    }
                }

                if (count > 0) {
                    separation_force = separation_force.normalized() * current.max_speed;
                    desired_velocity = desired_velocity + separation_force * config.separation_weight;
                    if (desired_velocity.length_squared() > current.max_speed * current.max_speed) {
                        desired_velocity = desired_velocity.normalized() * current.max_speed;
                    }
                }
            }

            // Optional dynamic obstacle avoidance
            if (config.enable_obstacle_avoidance && !obstacles.empty()) {
                math::Vec3f obs_avoid{};
                for (const auto& obs : obstacles) {
                    const math::Vec3f diff = current.position - obs.position;
                    const float avoid_dist = obs.radius + current.radius + 1.0f;
                    const float d_sq = diff.length_squared();
                    if (d_sq > 1e-5f && d_sq < avoid_dist * avoid_dist) {
                        const float d = std::sqrt(d_sq);
                        obs_avoid += (diff / d) * (avoid_dist - d);
                    }
                }
                if (obs_avoid.length_squared() > 1e-5f) {
                    desired_velocity = (desired_velocity + obs_avoid.normalized() * current.max_speed * 2.0f).normalized() * current.max_speed;
                }
            }

            next.velocity = desired_velocity;
            next.position = current.position + next.velocity * dt;
        }
    }

    WorldConfig config_;
    std::vector<Agent> agents_;
    std::vector<Agent> next_agents_;
    std::vector<Obstacle> obstacles_;
    spatial::SpatialHash spatial_hash_;
    std::optional<navigation::Grid> grid_;
};

} // namespace scandium::simulation
