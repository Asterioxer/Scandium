#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "scandium/core/job_system.hpp"
#include "scandium/math/vec3.hpp"
#include "scandium/simulation/agent_world.hpp"

namespace scandium::simulation {

/**
 * 16-byte aligned StructuredBuffer representation matching standard HLSL/GLSL compute shaders.
 */
struct alignas(16) GPUAgent {
    // float4 pos_radius: x, y, z = position; w = radius
    float pos_x{0.0f};
    float pos_y{0.0f};
    float pos_z{0.0f};
    float radius{0.5f};

    // float4 vel_speed: x, y, z = velocity; w = max_speed
    float vel_x{0.0f};
    float vel_y{0.0f};
    float vel_z{0.0f};
    float max_speed{5.0f};

    // float4 target_state: x, y, z = target; w = state (cast to uint)
    float target_x{0.0f};
    float target_y{0.0f};
    float target_z{0.0f};
    std::uint32_t state{0}; // 0 = Idle, 1 = Seek, 2 = Arrived, etc.

    // uint4 metadata: x = agent_id, y, z, w = reserved
    std::uint32_t agent_id{0};
    std::uint32_t padding[3]{0, 0, 0};
};

struct alignas(16) GPUSimulationParams {
    float dt{1.0f / 60.0f};
    float separation_radius{2.0f};
    float separation_weight{1.5f};
    float waypoint_reach_distance{1.0f};

    std::uint32_t agent_count{0};
    std::uint32_t enable_separation{1};
    std::uint32_t reserved0{0};
    std::uint32_t reserved1{0};
};

/**
 * High-Throughput Batch / Compute Shader Pipeline for multi-agent simulation.
 * Employs hardware-aligned memory layouts (DirectX 12 / Vulkan / WebGPU compatible)
 * with a high-throughput CPU wavefront emulator and HLSL compute shader export.
 */
class BatchSimulationPipeline {
public:
    static constexpr std::size_t WORKGROUP_SIZE = 64;

    BatchSimulationPipeline() = default;

    void upload(const std::vector<Agent>& agents) {
        current_agents_.resize(agents.size());
        next_agents_.resize(agents.size());

        for (std::size_t i = 0; i < agents.size(); ++i) {
            const auto& src = agents[i];
            auto& dst = current_agents_[i];

            dst.pos_x = src.position.x;
            dst.pos_y = src.position.y;
            dst.pos_z = src.position.z;
            dst.radius = src.radius;

            dst.vel_x = src.velocity.x;
            dst.vel_y = src.velocity.y;
            dst.vel_z = src.velocity.z;
            dst.max_speed = src.max_speed;

            dst.target_x = src.target.x;
            dst.target_y = src.target.y;
            dst.target_z = src.target.z;
            dst.state = static_cast<std::uint32_t>(src.state);

            dst.agent_id = src.id;
        }
    }

    void download(std::vector<Agent>& agents) const {
        if (agents.size() != current_agents_.size()) {
            agents.resize(current_agents_.size());
        }

        for (std::size_t i = 0; i < current_agents_.size(); ++i) {
            const auto& src = current_agents_[i];
            auto& dst = agents[i];

            dst.id = src.agent_id;
            dst.position = {src.pos_x, src.pos_y, src.pos_z};
            dst.velocity = {src.vel_x, src.vel_y, src.vel_z};
            dst.radius = src.radius;
            dst.max_speed = src.max_speed;
            dst.target = {src.target_x, src.target_y, src.target_z};
            dst.state = static_cast<AgentState>(src.state);
        }
    }

    [[nodiscard]] std::size_t agent_count() const noexcept {
        return current_agents_.size();
    }

    [[nodiscard]] const std::vector<GPUAgent>& agent_buffer() const noexcept {
        return current_agents_;
    }

    /**
     * Executes the compute shader kernel across all agents.
     * Uses parallel threadgroup dispatch (WORKGROUP_SIZE = 64 threads per group)
     * matching GPU execution model exactly.
     */
    void step(const GPUSimulationParams& params, core::JobSystem* jobs = nullptr) {
        const std::size_t count = current_agents_.size();
        if (count == 0) return;

        auto execute_group = [&](std::size_t group_start, std::size_t group_end) {
            for (std::size_t i = group_start; i < group_end; ++i) {
                step_kernel(current_agents_[i], next_agents_[i], current_agents_.data(), count, params);
            }
        };

        if (jobs && jobs->worker_count() > 1 && count >= 512) {
            const std::size_t num_groups = (count + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;
            jobs->parallel_for_chunked(num_groups, 4, [&](std::size_t group_idx) {
                const std::size_t start = group_idx * WORKGROUP_SIZE;
                const std::size_t end = std::min(start + WORKGROUP_SIZE, count);
                execute_group(start, end);
            });
        } else {
            execute_group(0, count);
        }

        current_agents_.swap(next_agents_);
    }

    /**
     * Returns standard HLSL Compute Shader code corresponding to this compute kernel.
     * Ready for compilation with D3DCompile or DXC to SPIR-V / DXIL.
     */
    [[nodiscard]] static std::string_view get_hlsl_compute_shader() noexcept {
        return R"(// Scandium Engine - Multi-Agent GPU Simulation Compute Shader
struct GPUAgent {
    float4 pos_radius;
    float4 vel_speed;
    float4 target_state;
    uint4 metadata;
};

cbuffer SimulationParams : register(b0) {
    float dt;
    float separation_radius;
    float separation_weight;
    float waypoint_reach_dist;
    uint agent_count;
    uint enable_separation;
    uint reserved0;
    uint reserved1;
};

StructuredBuffer<GPUAgent> InputAgents : register(t0);
RWStructuredBuffer<GPUAgent> OutputAgents : register(u0);

[numthreads(64, 1, 1)]
void CSMain(uint3 dispatch_id : SV_DispatchThreadID) {
    uint id = dispatch_id.x;
    if (id >= agent_count) return;

    GPUAgent current = InputAgents[id];
    GPUAgent next = current;

    float3 pos = current.pos_radius.xyz;
    float3 target = current.target_state.xyz;
    float max_speed = current.vel_speed.w;

    float3 delta = target - pos;
    float dist_sq = dot(delta, delta);

    float3 desired_vel = float3(0, 0, 0);
    if (dist_sq > 4.0f) {
        desired_vel = normalize(delta) * max_speed;
        next.target_state.w = 1.0f; // Seek
    } else {
        next.target_state.w = 0.0f; // Idle
    }

    if (enable_separation != 0 && dist_sq > 4.0f) {
        float3 sep_force = float3(0, 0, 0);
        float sep_r = separation_radius;
        float sep_r_sq = sep_r * sep_r;
        int count = 0;

        for (uint j = 0; j < agent_count; ++j) {
            if (j == id) continue;
            float3 diff = pos - InputAgents[j].pos_radius.xyz;
            float d2 = dot(diff, diff);
            if (d2 > 1e-5f && d2 < sep_r_sq) {
                float d = sqrt(d2);
                sep_force += (diff / d) * (sep_r - d);
                count++;
            }
        }

        if (count > 0 && dot(sep_force, sep_force) > 1e-5f) {
            desired_vel = normalize(desired_vel + normalize(sep_force) * max_speed * separation_weight) * max_speed;
        }
    }

    next.vel_speed.xyz = desired_vel;
    next.pos_radius.xyz = pos + desired_vel * dt;
    OutputAgents[id] = next;
}
)";
    }

private:
    static void step_kernel(const GPUAgent& current, GPUAgent& next,
                            const GPUAgent* all_agents, std::size_t total_count,
                            const GPUSimulationParams& params) noexcept {
        next = current;

        const math::Vec3f pos{current.pos_x, current.pos_y, current.pos_z};
        const math::Vec3f target{current.target_x, current.target_y, current.target_z};

        const math::Vec3f delta = target - pos;
        const float dist_sq = delta.length_squared();

        math::Vec3f desired_velocity{};
        if (dist_sq > 4.0f) {
            desired_velocity = delta.normalized() * current.max_speed;
            next.state = 1; // Seek
        } else {
            next.vel_x = 0.0f;
            next.vel_y = 0.0f;
            next.vel_z = 0.0f;
            next.state = 0; // Idle
            return;
        }

        if (params.enable_separation) {
            math::Vec3f sep_force{};
            int count = 0;
            const float sep_sq = params.separation_radius * params.separation_radius;

            for (std::size_t j = 0; j < total_count; ++j) {
                if (j == current.agent_id) continue;
                const auto& other = all_agents[j];
                const math::Vec3f diff{pos.x - other.pos_x, pos.y - other.pos_y, pos.z - other.pos_z};
                const float d_sq = diff.length_squared();
                if (d_sq > 1e-5f && d_sq < sep_sq) {
                    const float d = std::sqrt(d_sq);
                    sep_force += (diff / d) * (params.separation_radius - d);
                    ++count;
                }
            }

            if (count > 0 && sep_force.length_squared() > 1e-5f) {
                desired_velocity = (desired_velocity + sep_force.normalized() * current.max_speed * params.separation_weight).normalized() * current.max_speed;
            }
        }

        next.vel_x = desired_velocity.x;
        next.vel_y = desired_velocity.y;
        next.vel_z = desired_velocity.z;

        next.pos_x = pos.x + desired_velocity.x * params.dt;
        next.pos_y = pos.y + desired_velocity.y * params.dt;
        next.pos_z = pos.z + desired_velocity.z * params.dt;
    }

    std::vector<GPUAgent> current_agents_;
    std::vector<GPUAgent> next_agents_;
};

} // namespace scandium::simulation
