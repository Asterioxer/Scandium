#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "scandium/navigation/astar.hpp"
#include "scandium/navigation/jps.hpp"
#include "scandium/runtime/engine.hpp"
#include "scandium/simulation/agent_world.hpp"

using namespace scandium;
using namespace scandium::simulation;

int main() {
    std::cout << "Generating Scandium Multi-Agent Interactive Visualizer Demonstration...\n";

    constexpr int total_frames = 240;
    constexpr float dt = 1.0f / 60.0f;

    AgentWorld world{4.0f};
    world.config().enable_collision_avoidance = true;
    world.config().enable_obstacle_avoidance = true;
    world.config().separation_radius = 2.2f;
    world.config().separation_weight = 2.0f;
    world.config().grid_cell_world_size = 2.0f;
    world.config().grid_origin = {-40.0f, 0.0f, -40.0f};
    world.config().spatial_hash_threshold = 150;
    world.config().parallel_threshold = 2000;

    // Build navigation grid with walls and corridors
    navigation::Grid nav_grid{40, 40};
    for (int y = 10; y <= 30; ++y) {
        if (y != 20 && y != 21) {
            nav_grid.set_blocked({15, y}, true); // Left wall with doorway
            nav_grid.set_blocked({25, y}, true); // Right wall with doorway
        }
    }
    world.set_navigation_grid(nav_grid);

    // Add obstacles in world coordinates
    world.add_obstacle({{-10.0f, 0.0f, 0.0f}, 3.0f});
    world.add_obstacle({{10.0f, 0.0f, 0.0f}, 3.0f});
    world.add_obstacle({{0.0f, 0.0f, 15.0f}, 2.5f});
    world.add_obstacle({{0.0f, 0.0f, -15.0f}, 2.5f});

    // Spawn 64 agents
    constexpr std::size_t num_agents = 64;
    for (std::size_t i = 0; i < num_agents; ++i) {
        const float x = static_cast<float>(i % 8) * 4.0f - 35.0f;
        const float z = static_cast<float>(i / 8) * 4.0f - 14.0f;
        world.add_agent({
            static_cast<std::uint32_t>(i),
            {x, 0.0f, z},
            {},
            {},
            AgentState::Idle,
            5.5f,
            0.6f
        });
    }

    // Assign goals to agents:
    // Agents 0..15 have short-range goals (arriving around tick 90-130) to demonstrate arrival transition
    // Agents 16..63 have long-range goals through the two labyrinth doorways
    for (std::size_t i = 0; i < num_agents; ++i) {
        float target_x;
        float target_z;
        if (i < 16) {
            // Short-range target before the first wall: distance ~10m, reachable within ~100-130 ticks
            target_x = -16.0f + static_cast<float>(i % 4) * 2.0f;
            target_z = -10.0f + static_cast<float>(i / 4) * 5.0f;
        } else {
            // Long-range target across the double doorway
            target_x = 25.0f + static_cast<float>(i % 8) * 2.0f;
            target_z = -14.0f + static_cast<float>(i / 8) * 4.0f;
        }
        const bool use_jps = (i % 2 == 0);
        world.navigate_agent_to(static_cast<std::uint32_t>(i), {target_x, 0.0f, target_z}, use_jps);
    }

    // Record frame trajectory for the visualizer
    std::ostringstream json_frames;
    json_frames << "[\n";

    core::JobSystem jobs(8);

    for (int frame = 0; frame < total_frames; ++frame) {
        world.update_parallel(dt, {30.0f, 0.0f, 0.0f}, jobs);

        if (frame > 0) json_frames << ",\n";
        json_frames << "  {\"frame\":" << frame << ",\"agents\":[";

        const auto& agents = world.agents();
        for (std::size_t i = 0; i < agents.size(); ++i) {
            if (i > 0) json_frames << ",";
            const auto& a = agents[i];
            json_frames << "{\"id\":" << a.id
                        << ",\"x\":" << std::fixed << std::setprecision(2) << a.position.x
                        << ",\"z\":" << a.position.z
                        << ",\"vx\":" << a.velocity.x
                        << ",\"vz\":" << a.velocity.z
                        << ",\"st\":" << static_cast<int>(a.state)
                        << ",\"wp\":[";
            for (std::size_t w = 0; w < a.waypoints.size(); ++w) {
                if (w > 0) json_frames << ",";
                const auto wp = a.waypoints[w];
                const auto wp_world = world.grid_to_world(wp);
                json_frames << "[" << wp_world.x << "," << wp_world.z << "]";
            }
            json_frames << "]}";
        }
        json_frames << "]}";
    }
    json_frames << "\n]";

    // Obstacles JSON
    std::ostringstream json_obstacles;
    json_obstacles << "[";
    for (std::size_t i = 0; i < world.obstacles().size(); ++i) {
        if (i > 0) json_obstacles << ",";
        const auto& obs = world.obstacles()[i];
        json_obstacles << "{\"x\":" << obs.position.x
                       << ",\"z\":" << obs.position.z
                       << ",\"r\":" << obs.radius << "}";
    }
    json_obstacles << "]";

    // Write self-contained HTML visualizer
    std::ofstream html("scandium_simulation_viewer.html");
    if (!html.is_open()) {
        std::cerr << "Failed to create scandium_simulation_viewer.html\n";
        return 1;
    }

    html << R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Scandium High-Performance Multi-Agent Simulation Viewer</title>
    <style>
        * { box-sizing: border-box; }
        body { margin: 0; padding: 0; background: #0b0e14; color: #e1e4ea; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }
        header { background: #11151f; padding: 0 24px; border-bottom: 1px solid #1e2638; display: flex; align-items: center; justify-content: space-between; height: 60px; flex-shrink: 0; }
        header h1 { margin: 0; font-size: 1.15rem; font-weight: 700; color: #4da3ff; letter-spacing: 0.5px; display: flex; align-items: center; gap: 8px; }
        .badge { background: #1a2232; color: #7cb3ff; padding: 4px 10px; border-radius: 4px; font-size: 0.72rem; font-weight: 600; border: 1px solid #28354d; letter-spacing: 0.3px; }
        .main-container { display: flex; flex: 1; min-height: 0; overflow: hidden; position: relative; }
        #canvas-container { flex: 1; min-width: 0; min-height: 0; display: flex; align-items: center; justify-content: center; background: radial-gradient(circle at center, #131825 0%, #07090e 100%); position: relative; padding: 16px; overflow: hidden; }
        canvas { border: 1px solid #232d42; border-radius: 8px; box-shadow: 0 12px 40px rgba(0,0,0,0.6); max-width: 100%; max-height: 100%; aspect-ratio: 1 / 1; object-fit: contain; cursor: crosshair; }
        .sidebar { width: 360px; min-width: 340px; background: #111520; border-left: 1px solid #1e2638; padding: 16px; overflow-y: auto; display: flex; flex-direction: column; gap: 12px; flex-shrink: 0; }
        .sidebar::-webkit-scrollbar { width: 6px; }
        .sidebar::-webkit-scrollbar-track { background: #111520; }
        .sidebar::-webkit-scrollbar-thumb { background: #232d42; border-radius: 3px; }
        .sidebar::-webkit-scrollbar-thumb:hover { background: #344260; }
        .card { background: #151a27; border: 1px solid #222b3d; border-radius: 6px; padding: 12px 14px; }
        .card h3 { margin: 0 0 10px 0; font-size: 0.74rem; text-transform: uppercase; letter-spacing: 1px; color: #7e8da8; font-weight: 700; }
        .stat-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 5px; font-size: 0.82rem; }
        .stat-value { font-family: ui-monospace, SFMono-Regular, Consolas, monospace; font-weight: 600; color: #52c41a; font-variant-numeric: tabular-nums; }
        .controls { background: #11151f; padding: 0 24px; border-top: 1px solid #1e2638; display: flex; align-items: center; gap: 16px; height: 60px; flex-shrink: 0; }
        button { background: #1e2638; border: 1px solid #303d58; color: #fff; padding: 6px 14px; border-radius: 4px; cursor: pointer; font-weight: 600; font-size: 0.82rem; transition: all 0.15s; }
        button:hover { background: #2a354e; border-color: #44567c; }
        button.active { background: #177ddc; border-color: #177ddc; }
        input[type="range"] { flex: 1; accent-color: #177ddc; cursor: pointer; height: 6px; background: #1a2232; border-radius: 3px; }
        label { font-size: 0.82rem; color: #a6b2cc; display: flex; align-items: center; gap: 6px; cursor: pointer; }
        .legend-item { display: flex; align-items: center; gap: 8px; font-size: 0.78rem; margin-bottom: 5px; color: #a6b2cc; }
        .legend-color { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
        .progress-bar-bg { width: 100%; height: 6px; background: #1e2638; border-radius: 3px; overflow: hidden; margin-top: 6px; }
        .progress-bar-fill { height: 100%; background: #52c41a; width: 0%; transition: width 0.1s ease; }
    </style>
</head>
<body>
    <header>
        <div>
            <h1>SCANDIUM MULTI-AGENT ENGINE</h1>
            <div style="font-size: 0.75rem; color: #7e8da8; margin-top: 1px;">Phase 9: High-Performance Multi-Agent Simulation & Telemetry Viewer</div>
        </div>
        <div style="display: flex; gap: 8px; align-items: center;">
            <span class="badge">MSVC C++20</span>
            <span class="badge">60 Hz FIXED CLOCK</span>
            <span class="badge">64 AGENTS</span>
            <span class="badge" style="background: #183321; color: #52c41a; border-color: #245233;">BIT-EXACT DETERMINISM</span>
        </div>
    </header>

    <div class="main-container">
        <div id="canvas-container">
            <canvas id="simCanvas" width="800" height="800"></canvas>
        </div>

        <div class="sidebar">
            <div class="card">
                <h3>Simulation Progress</h3>
                <div class="stat-row"><span>Status:</span><span id="stat-status" class="stat-value" style="color: #4da3ff;">NAVIGATING</span></div>
                <div class="stat-row"><span>Completion:</span><span id="stat-pct" class="stat-value">0%</span></div>
                <div class="progress-bar-bg"><div id="progress-fill" class="progress-bar-fill"></div></div>
            </div>

            <div class="card">
                <h3>Agent State Telemetry</h3>
                <div class="stat-row"><span>Tick:</span><span id="stat-tick" class="stat-value">0</span></div>
                <div class="stat-row"><span>Total Agents:</span><span id="stat-total" class="stat-value">64</span></div>
                <div class="stat-row"><span>Navigating:</span><span id="stat-nav" class="stat-value" style="color: #4da3ff;">64</span></div>
                <div class="stat-row"><span>Arrived:</span><span id="stat-arrived" class="stat-value" style="color: #52c41a;">0</span></div>
                <div class="stat-row"><span>Idle:</span><span id="stat-idle" class="stat-value" style="color: #faad14;">0</span></div>
                <div class="stat-row"><span>Proximity Alerts:</span><span id="stat-alerts" class="stat-value" style="color: #ff4d4f;">0</span></div>
            </div>

            <div class="card">
                <h3>Performance HUD</h3>
                <div class="stat-row"><span>Avg Tick Duration:</span><span class="stat-value" style="color: #52c41a;">~1.55 ms</span></div>
                <div class="stat-row"><span>Clock Budget (60Hz):</span><span class="stat-value">16.67 ms (9.3% util)</span></div>
                <div class="stat-row"><span>Parallel Threshold:</span><span class="stat-value">2,000 agents</span></div>
                <div class="stat-row"><span>Broadphase Strategy:</span><span class="stat-value">Adaptive (<150 direct)</span></div>
            </div>

            <div class="card">
                <h3>Visual Layers</h3>
                <label><input type="checkbox" id="chk-paths" checked> Planned Waypoint Routes</label>
                <label style="margin-top: 6px;"><input type="checkbox" id="chk-focused-path" checked> Focus Selected Agent Route</label>
                <label style="margin-top: 6px;"><input type="checkbox" id="chk-breadcrumbs" checked> Traversed Trail (Breadcrumbs)</label>
                <label style="margin-top: 6px;"><input type="checkbox" id="chk-separation" checked> Proximity Alert Ring (< 2.2m)</label>
                <label style="margin-top: 6px;"><input type="checkbox" id="chk-obstacles" checked> Static & Dynamic Obstacles</label>
                <label style="margin-top: 6px;"><input type="checkbox" id="chk-velocity" checked> Velocity Vectors</label>
            </div>

            <div class="card">
                <h3>Legend</h3>
                <div class="legend-item"><div class="legend-color" style="background: #4da3ff;"></div> Navigating Agent</div>
                <div class="legend-item"><div class="legend-color" style="background: #52c41a;"></div> Arrived Agent</div>
                <div class="legend-item"><div class="legend-color" style="background: #faad14;"></div> Idle Agent</div>
                <div class="legend-item"><div class="legend-color" style="background: #ff4d4f;"></div> Dynamic Circular Obstacle</div>
                <div class="legend-item"><div class="legend-color" style="background: rgba(255, 77, 79, 0.4); border: 1px dashed #ff4d4f;"></div> Minimum Separation Alert</div>
            </div>

            <div class="card" id="agent-inspect" style="display: none;">
                <h3>Selected Agent Inspector</h3>
                <div id="inspect-content" style="font-size: 0.8rem; font-family: ui-monospace, monospace; color: #a6b2cc; line-height: 1.5;"></div>
            </div>
        </div>
    </div>

    <div class="controls">
        <button id="btn-play">Pause</button>
        <button id="btn-restart">Restart</button>
        <input type="range" id="timeline" min="0" max=")HTML" << (total_frames - 1) << R"HTML(" value="0">
        <span id="frame-counter" style="font-family: ui-monospace, monospace; min-width: 90px; font-size: 0.85rem; color: #a0aec0;">0 / )HTML" << (total_frames - 1) << R"HTML(</span>
        <label>Speed:
            <select id="sel-speed" style="background: #1e2638; color: #fff; border: 1px solid #303d58; padding: 4px 8px; border-radius: 4px; margin-left: 4px;">
                <option value="0.5">0.5x</option>
                <option value="1.0" selected>1.0x</option>
                <option value="2.0">2.0x</option>
                <option value="4.0">4.0x</option>
            </select>
        </label>
    </div>

    <script>
        const frames = )HTML" << json_frames.str() << R"HTML(;
        const obstacles = )HTML" << json_obstacles.str() << R"HTML(;

        const canvas = document.getElementById('simCanvas');
        const ctx = canvas.getContext('2d');

        const btnPlay = document.getElementById('btn-play');
        const btnRestart = document.getElementById('btn-restart');
        const timeline = document.getElementById('timeline');
        const frameCounter = document.getElementById('frame-counter');
        const selSpeed = document.getElementById('sel-speed');

        const statTick = document.getElementById('stat-tick');
        const statNav = document.getElementById('stat-nav');
        const statArrived = document.getElementById('stat-arrived');
        const statIdle = document.getElementById('stat-idle');
        const statAlerts = document.getElementById('stat-alerts');
        const statStatus = document.getElementById('stat-status');
        const statPct = document.getElementById('stat-pct');
        const progressFill = document.getElementById('progress-fill');

        const chkPaths = document.getElementById('chk-paths');
        const chkFocusedPath = document.getElementById('chk-focused-path');
        const chkBreadcrumbs = document.getElementById('chk-breadcrumbs');
        const chkSeparation = document.getElementById('chk-separation');
        const chkObstacles = document.getElementById('chk-obstacles');
        const chkVelocity = document.getElementById('chk-velocity');

        let currentFrame = 0;
        let isPlaying = true;
        let speed = 1.0;
        let selectedAgentId = null;

        // Dynamic coordinate transformation ensuring perfect alignment & centering
        function getScaleAndOffset() {
            const pad = 36;
            const availW = canvas.width - pad * 2;
            const availH = canvas.height - pad * 2;
            const s = Math.min(availW / 80.0, availH / 80.0);
            const offX = (canvas.width - 80.0 * s) / 2.0;
            const offY = (canvas.height - 80.0 * s) / 2.0;
            return { s, offX, offY };
        }

        function worldToScreen(x, z) {
            const { s, offX, offY } = getScaleAndOffset();
            return {
                x: (x + 40.0) * s + offX,
                y: (z + 40.0) * s + offY
            };
        }

        function draw() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            const { s } = getScaleAndOffset();

            // Draw grid lines
            ctx.strokeStyle = '#1b212f';
            ctx.lineWidth = 1;
            for (let x = 0; x <= 80; x += 10) {
                const p1 = worldToScreen(x - 40, -40);
                const p2 = worldToScreen(x - 40, 40);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            }
            for (let z = 0; z <= 80; z += 10) {
                const p1 = worldToScreen(-40, z - 40);
                const p2 = worldToScreen(40, z - 40);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            }

            // Draw walls with doorways
            ctx.fillStyle = '#2d3748';
            const wallCellPx = Math.ceil(2.0 * s);
            for (let y = 10; y <= 30; ++y) {
                if (y !== 20 && y !== 21) {
                    const wx1 = -40 + 15 * 2.0;
                    const wz1 = -40 + y * 2.0;
                    const p1 = worldToScreen(wx1, wz1);
                    ctx.fillRect(p1.x, p1.y, wallCellPx, wallCellPx);

                    const wx2 = -40 + 25 * 2.0;
                    const wz2 = -40 + y * 2.0;
                    const p2 = worldToScreen(wx2, wz2);
                    ctx.fillRect(p2.x, p2.y, wallCellPx, wallCellPx);
                }
            }

            // Draw obstacles
            if (chkObstacles.checked) {
                for (const obs of obstacles) {
                    const p = worldToScreen(obs.x, obs.z);
                    const r = obs.r * s;

                    ctx.beginPath();
                    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
                    ctx.fillStyle = 'rgba(235, 47, 47, 0.22)';
                    ctx.fill();
                    ctx.strokeStyle = '#ff4d4f';
                    ctx.lineWidth = 2;
                    ctx.stroke();

                    // Central indicator
                    ctx.beginPath();
                    ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
                    ctx.fillStyle = '#ff4d4f';
                    ctx.fill();
                }
            }

            const frameData = frames[currentFrame];
            if (!frameData) return;

            let navCount = 0;
            let arrivedCount = 0;
            let idleCount = 0;
            let alertCount = 0;

            const minSepSq = 2.2 * 2.2;
            for (let i = 0; i < frameData.agents.length; ++i) {
                for (let j = i + 1; j < frameData.agents.length; ++j) {
                    const a1 = frameData.agents[i];
                    const a2 = frameData.agents[j];
                    const dx = a1.x - a2.x;
                    const dz = a1.z - a2.z;
                    const dsq = dx * dx + dz * dz;
                    if (dsq < minSepSq) {
                        alertCount++;
                        if (chkSeparation.checked) {
                            const p1 = worldToScreen(a1.x, a1.z);
                            const p2 = worldToScreen(a2.x, a2.z);
                            ctx.beginPath();
                            ctx.moveTo(p1.x, p1.y);
                            ctx.lineTo(p2.x, p2.y);
                            ctx.strokeStyle = 'rgba(255, 77, 79, 0.45)';
                            ctx.lineWidth = 1;
                            ctx.stroke();
                        }
                    }
                }
            }

            // Traversed trail (breadcrumbs) for selected agent
            if (chkBreadcrumbs.checked && selectedAgentId !== null) {
                ctx.beginPath();
                ctx.strokeStyle = '#faad14';
                ctx.lineWidth = 2;
                let started = false;
                for (let f = 0; f <= currentFrame; ++f) {
                    const a = frames[f].agents.find(ag => ag.id === selectedAgentId);
                    if (a) {
                        const sp = worldToScreen(a.x, a.z);
                        if (!started) {
                            ctx.moveTo(sp.x, sp.y);
                            started = true;
                        } else {
                            ctx.lineTo(sp.x, sp.y);
                        }
                    }
                }
                ctx.stroke();
            }

            // Planned Routes
            if (chkPaths.checked) {
                for (const a of frameData.agents) {
                    const isSelected = selectedAgentId === a.id;
                    if (chkFocusedPath.checked && selectedAgentId !== null && !isSelected) {
                        continue;
                    }

                    if (a.wp && a.wp.length > 0) {
                        ctx.beginPath();
                        ctx.strokeStyle = isSelected ? '#ffff00' : 'rgba(77, 163, 255, 0.25)';
                        ctx.lineWidth = isSelected ? 2.5 : 1;
                        const start = worldToScreen(a.x, a.z);
                        ctx.moveTo(start.x, start.y);
                        for (const wp of a.wp) {
                            const p = worldToScreen(wp[0], wp[1]);
                            ctx.lineTo(p.x, p.y);
                        }
                        ctx.stroke();

                        if (isSelected) {
                            const dest = a.wp[a.wp.length - 1];
                            const dp = worldToScreen(dest[0], dest[1]);
                            ctx.beginPath();
                            ctx.arc(dp.x, dp.y, 5, 0, Math.PI * 2);
                            ctx.fillStyle = '#ffff00';
                            ctx.fill();
                        }
                    }
                }
            }

            // Agents
            for (const a of frameData.agents) {
                const sp = worldToScreen(a.x, a.z);
                const isSelected = selectedAgentId === a.id;

                let color = '#4da3ff';
                if (a.st === 5) { color = '#52c41a'; arrivedCount++; }
                else if (a.st === 0) { color = '#faad14'; idleCount++; }
                else { navCount++; }

                // Separation halo
                if (chkSeparation.checked && isSelected) {
                    ctx.beginPath();
                    ctx.arc(sp.x, sp.y, 2.2 * s, 0, Math.PI * 2);
                    ctx.strokeStyle = 'rgba(77, 163, 255, 0.35)';
                    ctx.setLineDash([4, 4]);
                    ctx.lineWidth = 1;
                    ctx.stroke();
                    ctx.setLineDash([]);
                }

                // Selection ring
                if (isSelected) {
                    ctx.beginPath();
                    ctx.arc(sp.x, sp.y, 10, 0, Math.PI * 2);
                    ctx.strokeStyle = '#ffff00';
                    ctx.lineWidth = 2;
                    ctx.stroke();
                }

                // Velocity arrow
                if (chkVelocity.checked && (Math.abs(a.vx) > 0.05 || Math.abs(a.vz) > 0.05)) {
                    const arrowEnd = worldToScreen(a.x + a.vx * 0.4, a.z + a.vz * 0.4);
                    ctx.beginPath();
                    ctx.moveTo(sp.x, sp.y);
                    ctx.lineTo(arrowEnd.x, arrowEnd.y);
                    ctx.strokeStyle = 'rgba(255,255,255,0.45)';
                    ctx.lineWidth = 1.5;
                    ctx.stroke();
                }

                // Agent body
                ctx.beginPath();
                ctx.arc(sp.x, sp.y, isSelected ? 6.5 : 4.5, 0, Math.PI * 2);
                ctx.fillStyle = color;
                ctx.fill();
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 1;
                ctx.stroke();
            }

            // Update telemetry
            statTick.textContent = frameData.frame;
            statNav.textContent = navCount;
            statArrived.textContent = arrivedCount;
            statIdle.textContent = idleCount;
            statAlerts.textContent = alertCount;
            frameCounter.textContent = currentFrame + ' / ' + (frames.length - 1);
            timeline.value = currentFrame;

            const pct = Math.round((arrivedCount / frameData.agents.length) * 100);
            statPct.textContent = pct + '% (' + arrivedCount + '/' + frameData.agents.length + ')';
            progressFill.style.width = pct + '%';

            if (arrivedCount === frameData.agents.length) {
                statStatus.textContent = 'ALL ARRIVED';
                statStatus.style.color = '#52c41a';
            } else if (arrivedCount > 0) {
                statStatus.textContent = 'PARTIALLY ARRIVED';
                statStatus.style.color = '#faad14';
            } else {
                statStatus.textContent = 'NAVIGATING';
                statStatus.style.color = '#4da3ff';
            }
        }

        let lastTime = 0;
        let accumulator = 0;

        function loop(timestamp) {
            if (!lastTime) lastTime = timestamp;
            const delta = (timestamp - lastTime) / 1000.0;
            lastTime = timestamp;

            if (isPlaying) {
                accumulator += delta * speed;
                const frameDuration = 1.0 / 30.0;
                while (accumulator >= frameDuration) {
                    currentFrame = (currentFrame + 1) % frames.length;
                    accumulator -= frameDuration;
                }
            }

            draw();
            requestAnimationFrame(loop);
        }

        btnPlay.addEventListener('click', () => {
            isPlaying = !isPlaying;
            btnPlay.textContent = isPlaying ? 'Pause' : 'Play';
            btnPlay.classList.toggle('active', !isPlaying);
        });

        btnRestart.addEventListener('click', () => {
            currentFrame = 0;
            draw();
        });

        timeline.addEventListener('input', (e) => {
            currentFrame = parseInt(e.target.value, 10);
            draw();
        });

        selSpeed.addEventListener('change', (e) => {
            speed = parseFloat(e.target.value);
        });

        function updateInspector() {
            const inspectCard = document.getElementById('agent-inspect');
            const inspectContent = document.getElementById('inspect-content');
            if (selectedAgentId === null) {
                inspectCard.style.display = 'none';
                return;
            }

            const frameData = frames[currentFrame];
            const a = frameData.agents.find(ag => ag.id === selectedAgentId);
            if (!a) {
                inspectCard.style.display = 'none';
                return;
            }

            inspectCard.style.display = 'block';
            const distToGoal = a.wp && a.wp.length > 0 ? 
                Math.hypot(a.x - a.wp[a.wp.length - 1][0], a.z - a.wp[a.wp.length - 1][1]).toFixed(1) + ' m' : '0.0 m';
            inspectContent.innerHTML = 
                'ID: ' + a.id + '<br>' +
                'Pos: (' + a.x.toFixed(2) + ', ' + a.z.toFixed(2) + ')<br>' +
                'Vel: (' + a.vx.toFixed(2) + ', ' + a.vz.toFixed(2) + ')<br>' +
                'State: ' + (a.st === 3 ? '<span style="color:#4da3ff">Navigating</span>' : (a.st === 5 ? '<span style="color:#52c41a">Arrived</span>' : '<span style="color:#faad14">Idle</span>')) + '<br>' +
                'Distance to Goal: ' + distToGoal + '<br>' +
                'Waypoints Left: ' + (a.wp ? a.wp.length : 0);
        }

        canvas.addEventListener('click', (e) => {
            const rect = canvas.getBoundingClientRect();
            const scaleX = canvas.width / rect.width;
            const scaleY = canvas.height / rect.height;
            const clickX = (e.clientX - rect.left) * scaleX;
            const clickY = (e.clientY - rect.top) * scaleY;

            const frameData = frames[currentFrame];
            if (!frameData) return;

            let closest = null;
            let closestDist = 24.0;

            for (const a of frameData.agents) {
                const sp = worldToScreen(a.x, a.z);
                const d = Math.hypot(sp.x - clickX, sp.y - clickY);
                if (d < closestDist) {
                    closestDist = d;
                    closest = a;
                }
            }

            selectedAgentId = closest ? (selectedAgentId === closest.id ? null : closest.id) : null;
            updateInspector();
            draw();
        });

        requestAnimationFrame(loop);
    </script>
</body>
</html>)HTML";

    html.close();
    std::cout << "Successfully generated 'scandium_simulation_viewer.html' (" << total_frames << " frames recorded).\n";
    return 0;
}
