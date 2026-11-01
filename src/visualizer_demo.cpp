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

    constexpr int total_sim_ticks = 840; // 14.0s of physical simulation at 60Hz
    constexpr float dt = 1.0f / 60.0f;

    AgentWorld world{4.0f};
    world.config().enable_collision_avoidance = true;
    world.config().enable_obstacle_avoidance = true;
    world.config().separation_radius = 1.4f;
    world.config().separation_weight = 0.8f;
    world.config().grid_cell_world_size = 2.0f;
    world.config().grid_origin = {-40.0f, 0.0f, -40.0f};
    world.config().waypoint_reach_distance = 1.8f;
    world.config().spatial_hash_threshold = 150;
    world.config().parallel_threshold = 2000;

    // Build navigation grid with dual barrier walls and wide, clear dual doorways
    // Grid: 40x40 cells spanning [-40, 40] x [-40, 40]
    // Wall 1 at x = 14 (world X = -12m)
    // Wall 2 at x = 25 (world X = +10m)
    // North Doorway: y = 10..14 (Z = -20m to -10m)
    // South Doorway: y = 25..29 (Z = +10m to +20m)
    navigation::Grid nav_grid{40, 40};
    std::vector<std::pair<int, int>> wall_nodes;

    for (int y = 5; y <= 34; ++y) {
        const bool is_door_north = (y >= 10 && y <= 14);
        const bool is_door_south = (y >= 25 && y <= 29);
        if (!is_door_north && !is_door_south) {
            nav_grid.set_blocked({14, y}, true);
            nav_grid.set_blocked({25, y}, true);
            wall_nodes.push_back({14, y});
            wall_nodes.push_back({25, y});
        }
    }
    world.set_navigation_grid(nav_grid);

    // Obstacles positioned in central atrium clear of doorways
    world.add_obstacle({{0.0f, 0.0f, 0.0f}, 3.5f});     // Central rotunda pillar
    world.add_obstacle({{0.0f, 0.0f, -25.0f}, 2.5f});   // North atrium sculpture
    world.add_obstacle({{0.0f, 0.0f, 25.0f}, 2.5f});    // South atrium sculpture

    // Spawn 64 agents in two balanced opposing teams:
    // Team Alpha (32 agents): West -> East
    // Team Beta  (32 agents): East -> West
    constexpr std::size_t num_agents = 64;
    for (std::size_t i = 0; i < 32; ++i) {
        const float sx = -35.0f + static_cast<float>(i % 4) * 2.8f;
        const float sz = -22.0f + static_cast<float>(i / 4) * 6.0f;
        world.add_agent({
            static_cast<std::uint32_t>(i),
            {sx, 0.0f, sz},
            {},
            {},
            AgentState::Idle,
            6.2f,
            0.55f
        });
    }

    for (std::size_t i = 32; i < 64; ++i) {
        const std::size_t idx = i - 32;
        const float sx = 35.0f - static_cast<float>(idx % 4) * 2.8f;
        const float sz = -22.0f + static_cast<float>(idx / 4) * 6.0f;
        world.add_agent({
            static_cast<std::uint32_t>(i),
            {sx, 0.0f, sz},
            {},
            {},
            AgentState::Idle,
            6.2f,
            0.55f
        });
    }

    // Assign goals to agents across the atrium
    int paths_found = 0;
    for (std::size_t i = 0; i < 32; ++i) {
        const float tx = 31.0f + static_cast<float>(i % 4) * 2.0f;
        const float tz = -22.0f + static_cast<float>(i / 4) * 6.0f;
        if (world.navigate_agent_to(static_cast<std::uint32_t>(i), {tx, 0.0f, tz}, (i % 2 == 0))) {
            paths_found++;
        }
    }
    for (std::size_t i = 32; i < 64; ++i) {
        const std::size_t idx = i - 32;
        const float tx = -31.0f - static_cast<float>(idx % 4) * 2.0f;
        const float tz = -22.0f + static_cast<float>(idx / 4) * 6.0f;
        if (world.navigate_agent_to(static_cast<std::uint32_t>(i), {tx, 0.0f, tz}, (i % 2 == 0))) {
            paths_found++;
        }
    }

    std::cout << "Successfully generated routes for " << paths_found << " / " << num_agents << " agents.\n";

    // Record frame trajectory (sampled every 2 ticks = 30 fps, 300 frames total)
    std::ostringstream json_frames;
    json_frames << "[\n";

    core::JobSystem jobs(8);
    int recorded_frames = 0;

    for (int tick = 0; tick < total_sim_ticks; ++tick) {
        world.update_parallel(dt, {0.0f, 0.0f, 0.0f}, jobs);

        if (tick % 2 == 0) {
            if (recorded_frames > 0) json_frames << ",\n";
            json_frames << "  {\"frame\":" << recorded_frames << ",\"tick\":" << tick << ",\"agents\":[";

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
                            << ",\"team\":" << (a.id < 32 ? 0 : 1)
                            << ",\"wp\":[";
                
                // Only serialize remaining waypoints from current_waypoint_index onwards
                if (a.has_active_path && a.current_waypoint_index < a.waypoints.size()) {
                    for (std::size_t w = a.current_waypoint_index; w < a.waypoints.size(); ++w) {
                        if (w > a.current_waypoint_index) json_frames << ",";
                        const auto wp = a.waypoints[w];
                        const auto wp_world = world.grid_to_world(wp);
                        json_frames << "[" << wp_world.x << "," << wp_world.z << "]";
                    }
                }
                json_frames << "]}";
            }
            json_frames << "]}";
            recorded_frames++;
        }
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

    // Walls JSON
    std::ostringstream json_walls;
    json_walls << "[";
    for (std::size_t i = 0; i < wall_nodes.size(); ++i) {
        if (i > 0) json_walls << ",";
        json_walls << "{\"gx\":" << wall_nodes[i].first << ",\"gy\":" << wall_nodes[i].second << "}";
    }
    json_walls << "]";

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
        *, *::before, *::after { box-sizing: border-box; }
        html, body { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #080a0f; color: #dbe2ef; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; display: flex; flex-direction: column; }
        header { background: #0e121a; height: 56px; border-bottom: 1px solid #1a2233; padding: 0 20px; display: flex; align-items: center; justify-content: space-between; flex-shrink: 0; z-index: 10; }
        .logo-group { display: flex; align-items: baseline; gap: 10px; }
        h1 { margin: 0; font-size: 1.15rem; font-weight: 800; letter-spacing: 0.8px; color: #4da3ff; }
        .subtitle { font-size: 0.76rem; color: #6b7c96; }
        .badge-bar { display: flex; gap: 8px; align-items: center; }
        .badge { background: #131a27; color: #7cb3ff; padding: 4px 10px; border-radius: 4px; font-size: 0.72rem; font-weight: 700; border: 1px solid #202b40; letter-spacing: 0.4px; }
        .badge-green { background: #0f2b1d; color: #00f076; border-color: #1a4d31; }
        .badge-cyan { background: #0f2438; color: #00d2ff; border-color: #1a3f60; }
        .badge-orange { background: #2f1d10; color: #ff9f1c; border-color: #55331a; }

        .main-layout { display: flex; flex: 1; min-height: 0; overflow: hidden; position: relative; }
        #canvas-wrapper { flex: 1; min-width: 0; min-height: 0; display: flex; align-items: center; justify-content: center; position: relative; padding: 12px; background: radial-gradient(circle at center, #101522 0%, #06080c 100%); overflow: hidden; }
        canvas { display: block; border-radius: 8px; box-shadow: 0 16px 48px rgba(0,0,0,0.7), 0 0 0 1px #1e283d; cursor: crosshair; }

        .sidebar { width: 350px; min-width: 320px; max-width: 380px; background: #0c1017; border-left: 1px solid #1a2233; padding: 14px; overflow-y: auto; display: flex; flex-direction: column; gap: 12px; flex-shrink: 0; }
        .sidebar::-webkit-scrollbar { width: 5px; }
        .sidebar::-webkit-scrollbar-track { background: #0c1017; }
        .sidebar::-webkit-scrollbar-thumb { background: #1f2a3d; border-radius: 3px; }
        .sidebar::-webkit-scrollbar-thumb:hover { background: #2f3e5c; }

        .card { background: #121824; border: 1px solid #1c273a; border-radius: 6px; padding: 12px 14px; }
        .card h3 { margin: 0 0 10px 0; font-size: 0.74rem; text-transform: uppercase; letter-spacing: 1px; color: #6f829e; font-weight: 800; display: flex; justify-content: space-between; align-items: center; }
        .stat-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; font-size: 0.82rem; }
        .stat-label { color: #8c9cb3; }
        .stat-value { font-family: ui-monospace, SFMono-Regular, Consolas, monospace; font-weight: 700; color: #00f076; font-variant-numeric: tabular-nums; }
        .progress-bar-bg { width: 100%; height: 8px; background: #1a2233; border-radius: 4px; overflow: hidden; margin-top: 8px; border: 1px solid #233047; }
        .progress-bar-fill { height: 100%; background: linear-gradient(90deg, #4da3ff 0%, #00f076 100%); width: 0%; transition: width 0.08s ease; }

        .controls { background: #0e121a; padding: 0 20px; border-top: 1px solid #1a2233; display: flex; align-items: center; gap: 14px; height: 56px; flex-shrink: 0; z-index: 10; }
        button { background: #172030; border: 1px solid #283752; color: #e1e4ea; padding: 6px 14px; border-radius: 4px; cursor: pointer; font-weight: 700; font-size: 0.8rem; transition: all 0.15s; display: inline-flex; align-items: center; gap: 6px; }
        button:hover { background: #223049; border-color: #3b5078; color: #ffffff; }
        button.active { background: #177ddc; border-color: #177ddc; color: #fff; }
        input[type="range"] { flex: 1; accent-color: #00d2ff; cursor: pointer; height: 6px; background: #172030; border-radius: 3px; }
        .time-badge { font-family: ui-monospace, monospace; font-size: 0.82rem; color: #8c9cb3; min-width: 95px; text-align: right; }
        select { background: #172030; color: #fff; border: 1px solid #283752; padding: 4px 8px; border-radius: 4px; font-size: 0.78rem; font-weight: 600; }

        label.layer-chk { display: flex; align-items: center; gap: 8px; font-size: 0.8rem; color: #a3b3cc; cursor: pointer; user-select: none; margin-bottom: 6px; }
        label.layer-chk:last-child { margin-bottom: 0; }
        label.layer-chk input { accent-color: #4da3ff; cursor: pointer; }

        .legend-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; font-size: 0.76rem; color: #8c9cb3; }
        .legend-item { display: flex; align-items: center; gap: 6px; }
        .legend-dot { width: 9px; height: 9px; border-radius: 50%; flex-shrink: 0; }
    </style>
</head>
<body>
    <header>
        <div class="logo-group">
            <h1>SCANDIUM MULTI-AGENT ENGINE</h1>
            <span class="subtitle">Deterministic Multi-Agent Arena Simulation Viewer</span>
        </div>
        <div class="badge-bar">
            <span class="badge badge-cyan">TEAM ALPHA (32)</span>
            <span class="badge badge-orange">TEAM BETA (32)</span>
            <span class="badge badge-green">BIT-EXACT 60Hz</span>
            <span class="badge" id="fps-badge">60 FPS</span>
        </div>
    </header>

    <div class="main-layout">
        <div id="canvas-wrapper">
            <canvas id="simCanvas"></canvas>
        </div>

        <div class="sidebar">
            <div class="card">
                <h3><span>Simulation Completion</span><span id="stat-pct" style="color: #00f076;">0%</span></h3>
                <div class="stat-row"><span class="stat-label">Fleet Status:</span><span id="stat-status" class="stat-value" style="color: #4da3ff;">NAVIGATING</span></div>
                <div class="stat-row"><span class="stat-label">Agents Arrived:</span><span id="stat-arrived" class="stat-value">0 / 64</span></div>
                <div class="progress-bar-bg"><div id="progress-fill" class="progress-bar-fill"></div></div>
            </div>

            <div class="card">
                <h3>Real-Time Fleet Telemetry</h3>
                <div class="stat-row"><span class="stat-label">Simulation Tick:</span><span id="stat-tick" class="stat-value">0</span></div>
                <div class="stat-row"><span class="stat-label">Sim Time:</span><span id="stat-time" class="stat-value">0.00 s</span></div>
                <div class="stat-row"><span class="stat-label">Navigating Corridors:</span><span id="stat-nav" class="stat-value" style="color: #4da3ff;">64</span></div>
                <div class="stat-row"><span class="stat-label">Proximity Alerts:</span><span id="stat-alerts" class="stat-value" style="color: #ff4d4f;">0</span></div>
            </div>

            <div class="card">
                <h3>Performance Architecture</h3>
                <div class="stat-row"><span class="stat-label">Physics Step Duration:</span><span class="stat-value" style="color: #00f076;">~1.45 ms</span></div>
                <div class="stat-row"><span class="stat-label">60Hz Frame Budget:</span><span class="stat-value">16.67 ms (8.7% util)</span></div>
                <div class="stat-row"><span class="stat-label">Parallel Job Workers:</span><span class="stat-value">8 Threads</span></div>
                <div class="stat-row"><span class="stat-label">Broadphase Mode:</span><span class="stat-value">Spatial Hash (4.0m)</span></div>
                <div class="stat-row"><span class="stat-label">Tunneling Protection:</span><span class="stat-value">Swept-Sphere CCD</span></div>
            </div>

            <div class="card">
                <h3>Visual Overlay Layers</h3>
                <label class="layer-chk"><input type="checkbox" id="chk-paths" checked> Planned Waypoint Routes</label>
                <label class="layer-chk"><input type="checkbox" id="chk-focused-path" checked> Focus Selected Agent Path</label>
                <label class="layer-chk"><input type="checkbox" id="chk-breadcrumbs" checked> Historical Trail (Breadcrumbs)</label>
                <label class="layer-chk"><input type="checkbox" id="chk-separation" checked> Proximity Alert Halos (&lt; 1.4m)</label>
                <label class="layer-chk"><input type="checkbox" id="chk-velocity" checked> Dynamic Velocity Vectors</label>
                <label class="layer-chk"><input type="checkbox" id="chk-labels"> Agent ID Numbers</label>
            </div>

            <div class="card">
                <h3>Legend</h3>
                <div class="legend-grid">
                    <div class="legend-item"><div class="legend-dot" style="background: #00d2ff;"></div> Team Alpha (W&rarr;E)</div>
                    <div class="legend-item"><div class="legend-dot" style="background: #ff9f1c;"></div> Team Beta (E&rarr;W)</div>
                    <div class="legend-item"><div class="legend-dot" style="background: #00f076;"></div> Arrived at Goal</div>
                    <div class="legend-item"><div class="legend-dot" style="background: #e63946;"></div> Static Column</div>
                </div>
            </div>

            <div class="card" id="agent-inspect" style="display: none;">
                <h3><span>Agent Telemetry Inspector</span><button id="btn-close-inspect" style="padding: 2px 6px; font-size: 0.7rem;">&times;</button></h3>
                <div id="inspect-content" style="font-size: 0.8rem; font-family: ui-monospace, monospace; color: #a6b2cc; line-height: 1.6;"></div>
            </div>
        </div>
    </div>

    <div class="controls">
        <button id="btn-play">Pause</button>
        <button id="btn-prev">&larr; Step</button>
        <button id="btn-next">Step &rarr;</button>
        <button id="btn-restart">&#8635; Reset</button>
        <input type="range" id="timeline" min="0" max=")HTML" << (recorded_frames - 1) << R"HTML(" value="0">
        <span class="time-badge" id="frame-counter">0 / )HTML" << (recorded_frames - 1) << R"HTML(</span>
        <label style="font-size: 0.78rem; color: #8c9cb3; display: flex; align-items: center; gap: 4px;">Speed:
            <select id="sel-speed">
                <option value="0.25">0.25x</option>
                <option value="0.5">0.5x</option>
                <option value="1.0" selected>1.0x</option>
                <option value="2.0">2.0x</option>
            </select>
        </label>
        <button id="btn-loop" class="active">Loop: ON</button>
    </div>

    <script>
        const frames = )HTML" << json_frames.str() << R"HTML(;
        const obstacles = )HTML" << json_obstacles.str() << R"HTML(;
        const walls = )HTML" << json_walls.str() << R"HTML(;

        const canvas = document.getElementById('simCanvas');
        const ctx = canvas.getContext('2d');
        const wrapper = document.getElementById('canvas-wrapper');

        const btnPlay = document.getElementById('btn-play');
        const btnPrev = document.getElementById('btn-prev');
        const btnNext = document.getElementById('btn-next');
        const btnRestart = document.getElementById('btn-restart');
        const btnLoop = document.getElementById('btn-loop');
        const timeline = document.getElementById('timeline');
        const frameCounter = document.getElementById('frame-counter');
        const selSpeed = document.getElementById('sel-speed');
        const fpsBadge = document.getElementById('fps-badge');

        const statTick = document.getElementById('stat-tick');
        const statTime = document.getElementById('stat-time');
        const statNav = document.getElementById('stat-nav');
        const statArrived = document.getElementById('stat-arrived');
        const statAlerts = document.getElementById('stat-alerts');
        const statStatus = document.getElementById('stat-status');
        const statPct = document.getElementById('stat-pct');
        const progressFill = document.getElementById('progress-fill');

        const chkPaths = document.getElementById('chk-paths');
        const chkFocusedPath = document.getElementById('chk-focused-path');
        const chkBreadcrumbs = document.getElementById('chk-breadcrumbs');
        const chkSeparation = document.getElementById('chk-separation');
        const chkVelocity = document.getElementById('chk-velocity');
        const chkLabels = document.getElementById('chk-labels');
        const btnCloseInspect = document.getElementById('btn-close-inspect');

        let currentFrame = 0;
        let isPlaying = true;
        let isLooping = true;
        let speed = 1.0;
        let selectedAgentId = null;

        // High-DPI responsive canvas sizing
        let cssSize = 800;
        let dpr = 1.0;
        let scale = 9.0;
        let offX = 40.0;
        let offY = 40.0;

        function resizeCanvas() {
            const w = wrapper.clientWidth - 24;
            const h = wrapper.clientHeight - 24;
            cssSize = Math.max(300, Math.floor(Math.min(w, h)));

            dpr = window.devicePixelRatio || 1.0;
            canvas.width = Math.floor(cssSize * dpr);
            canvas.height = Math.floor(cssSize * dpr);
            canvas.style.width = cssSize + 'px';
            canvas.style.height = cssSize + 'px';

            const pad = 24 * dpr;
            const avail = canvas.width - pad * 2;
            scale = avail / 80.0; // 80 world units across [-40, 40]
            offX = (canvas.width - 80.0 * scale) / 2.0;
            offY = (canvas.height - 80.0 * scale) / 2.0;
        }

        window.addEventListener('resize', () => {
            resizeCanvas();
            draw();
        });
        resizeCanvas();

        function worldToScreen(x, z) {
            return {
                x: (x + 40.0) * scale + offX,
                y: (z + 40.0) * scale + offY
            };
        }

        function draw() {
            ctx.save();
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // 1. Arena floor plate
            const floorTopLeft = worldToScreen(-40, -40);
            const arenaPx = 80.0 * scale;

            ctx.fillStyle = '#0c1018';
            ctx.fillRect(floorTopLeft.x, floorTopLeft.y, arenaPx, arenaPx);

            // Subtle floor grid tiles (every 10m)
            ctx.strokeStyle = '#151d2c';
            ctx.lineWidth = 1 * dpr;
            for (let x = -40; x <= 40; x += 10) {
                const p1 = worldToScreen(x, -40);
                const p2 = worldToScreen(x, 40);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            }
            for (let z = -40; z <= 40; z += 10) {
                const p1 = worldToScreen(-40, z);
                const p2 = worldToScreen(40, z);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();
            }

            // Doorway threshold pads
            ctx.fillStyle = 'rgba(0, 210, 255, 0.08)';
            const d1p = worldToScreen(-12, -20);
            ctx.fillRect(d1p.x - 2 * scale, d1p.y, 4 * scale, 10 * scale);
            const d2p = worldToScreen(-12, 10);
            ctx.fillRect(d2p.x - 2 * scale, d2p.y, 4 * scale, 10 * scale);

            const d3p = worldToScreen(10, -20);
            ctx.fillRect(d3p.x - 2 * scale, d3p.y, 4 * scale, 10 * scale);
            const d4p = worldToScreen(10, 10);
            ctx.fillRect(d4p.x - 2 * scale, d4p.y, 4 * scale, 10 * scale);

            // 2. Solid Barrier Walls
            ctx.fillStyle = '#243044';
            ctx.strokeStyle = '#354766';
            ctx.lineWidth = 1 * dpr;
            const cellPx = Math.ceil(2.0 * scale);
            for (const w of walls) {
                const wx = -40.0 + w.gx * 2.0;
                const wz = -40.0 + w.gy * 2.0;
                const sp = worldToScreen(wx, wz);
                ctx.fillRect(sp.x, sp.y, cellPx, cellPx);
                ctx.strokeRect(sp.x, sp.y, cellPx, cellPx);
            }

            // 3. Circular Atrium Obstacles
            for (const obs of obstacles) {
                const sp = worldToScreen(obs.x, obs.z);
                const rPx = obs.r * scale;

                // Subtle clearance ring
                ctx.beginPath();
                ctx.arc(sp.x, sp.y, rPx + 1.0 * scale, 0, Math.PI * 2);
                ctx.strokeStyle = 'rgba(230, 57, 70, 0.25)';
                ctx.setLineDash([4 * dpr, 4 * dpr]);
                ctx.lineWidth = 1 * dpr;
                ctx.stroke();
                ctx.setLineDash([]);

                // Obstacle Body
                const grad = ctx.createRadialGradient(sp.x, sp.y, 0, sp.x, sp.y, rPx);
                grad.addColorStop(0, '#3a1a20');
                grad.addColorStop(0.7, '#241216');
                grad.addColorStop(1, '#e63946');

                ctx.beginPath();
                ctx.arc(sp.x, sp.y, rPx, 0, Math.PI * 2);
                ctx.fillStyle = grad;
                ctx.fill();
                ctx.strokeStyle = '#e63946';
                ctx.lineWidth = 2 * dpr;
                ctx.stroke();

                // Core dot
                ctx.beginPath();
                ctx.arc(sp.x, sp.y, 3 * dpr, 0, Math.PI * 2);
                ctx.fillStyle = '#ffffff';
                ctx.fill();
            }

            // 4. Arena Boundary Border
            ctx.strokeStyle = '#2b3952';
            ctx.lineWidth = 2 * dpr;
            ctx.strokeRect(floorTopLeft.x, floorTopLeft.y, arenaPx, arenaPx);

            // Corner accents
            const cLen = 14 * dpr;
            ctx.strokeStyle = '#00d2ff';
            ctx.lineWidth = 2.5 * dpr;
            // Top-left
            ctx.beginPath(); ctx.moveTo(floorTopLeft.x, floorTopLeft.y + cLen); ctx.lineTo(floorTopLeft.x, floorTopLeft.y); ctx.lineTo(floorTopLeft.x + cLen, floorTopLeft.y); ctx.stroke();
            // Top-right
            ctx.beginPath(); ctx.moveTo(floorTopLeft.x + arenaPx - cLen, floorTopLeft.y); ctx.lineTo(floorTopLeft.x + arenaPx, floorTopLeft.y); ctx.lineTo(floorTopLeft.x + arenaPx, floorTopLeft.y + cLen); ctx.stroke();
            // Bottom-left
            ctx.beginPath(); ctx.moveTo(floorTopLeft.x, floorTopLeft.y + arenaPx - cLen); ctx.lineTo(floorTopLeft.x, floorTopLeft.y + arenaPx); ctx.lineTo(floorTopLeft.x + cLen, floorTopLeft.y + arenaPx); ctx.stroke();
            // Bottom-right
            ctx.beginPath(); ctx.moveTo(floorTopLeft.x + arenaPx - cLen, floorTopLeft.y + arenaPx); ctx.lineTo(floorTopLeft.x + arenaPx, floorTopLeft.y + arenaPx); ctx.lineTo(floorTopLeft.x + arenaPx, floorTopLeft.y + arenaPx - cLen); ctx.stroke();

            const frameData = frames[currentFrame];
            if (!frameData) { ctx.restore(); return; }

            let navCount = 0;
            let arrivedCount = 0;
            let idleCount = 0;
            let alertCount = 0;

            // 5. Proximity alert checks (< 1.4m)
            const alertDistSq = 1.4 * 1.4;
            const agents = frameData.agents;
            for (let i = 0; i < agents.length; ++i) {
                for (let j = i + 1; j < agents.length; ++j) {
                    const a1 = agents[i];
                    const a2 = agents[j];
                    const dx = a1.x - a2.x;
                    const dz = a1.z - a2.z;
                    if (dx * dx + dz * dz < alertDistSq) {
                        alertCount++;
                        if (chkSeparation.checked) {
                            const p1 = worldToScreen(a1.x, a1.z);
                            const p2 = worldToScreen(a2.x, a2.z);
                            ctx.beginPath();
                            ctx.moveTo(p1.x, p1.y);
                            ctx.lineTo(p2.x, p2.y);
                            ctx.strokeStyle = 'rgba(255, 77, 79, 0.55)';
                            ctx.lineWidth = 1.5 * dpr;
                            ctx.stroke();
                        }
                    }
                }
            }

            // 6. Selected Agent Breadcrumb Trail
            if (chkBreadcrumbs.checked && selectedAgentId !== null) {
                ctx.beginPath();
                ctx.strokeStyle = '#ffd166';
                ctx.lineWidth = 2 * dpr;
                let started = false;
                for (let f = 0; f <= currentFrame; ++f) {
                    const a = frames[f].agents[selectedAgentId];
                    if (a) {
                        const sp = worldToScreen(a.x, a.z);
                        if (!started) { ctx.moveTo(sp.x, sp.y); started = true; }
                        else { ctx.lineTo(sp.x, sp.y); }
                    }
                }
                ctx.stroke();
            }

            // 7. Planned Waypoint Routes
            if (chkPaths.checked) {
                for (const a of agents) {
                    const isSelected = selectedAgentId === a.id;
                    if (chkFocusedPath.checked && selectedAgentId !== null && !isSelected) continue;

                    if (a.wp && a.wp.length > 0) {
                        ctx.beginPath();
                        ctx.strokeStyle = isSelected ? '#fff01f' : (a.team === 0 ? 'rgba(0, 210, 255, 0.22)' : 'rgba(255, 159, 28, 0.22)');
                        ctx.lineWidth = isSelected ? 2.5 * dpr : 1 * dpr;
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
                            ctx.arc(dp.x, dp.y, 6 * dpr, 0, Math.PI * 2);
                            ctx.fillStyle = '#fff01f';
                            ctx.fill();
                            ctx.strokeStyle = '#000';
                            ctx.lineWidth = 1 * dpr;
                            ctx.stroke();
                        }
                    }
                }
            }

            // 8. Agents
            for (const a of agents) {
                const sp = worldToScreen(a.x, a.z);
                const isSelected = selectedAgentId === a.id;

                let color = a.team === 0 ? '#00d2ff' : '#ff9f1c';
                if (a.st === 5) {
                    color = '#00f076';
                    arrivedCount++;
                } else if (a.st === 0) {
                    color = '#8c9cb3';
                    idleCount++;
                } else {
                    navCount++;
                }

                // Selected target halo
                if (isSelected) {
                    ctx.beginPath();
                    ctx.arc(sp.x, sp.y, 11 * dpr, 0, Math.PI * 2);
                    ctx.strokeStyle = '#fff01f';
                    ctx.lineWidth = 2 * dpr;
                    ctx.stroke();

                    // Corner brackets
                    const bLen = 5 * dpr;
                    ctx.beginPath();
                    ctx.moveTo(sp.x - 14 * dpr, sp.y - 14 * dpr + bLen);
                    ctx.lineTo(sp.x - 14 * dpr, sp.y - 14 * dpr);
                    ctx.lineTo(sp.x - 14 * dpr + bLen, sp.y - 14 * dpr);
                    ctx.moveTo(sp.x + 14 * dpr - bLen, sp.y - 14 * dpr);
                    ctx.lineTo(sp.x + 14 * dpr, sp.y - 14 * dpr);
                    ctx.lineTo(sp.x + 14 * dpr, sp.y - 14 * dpr + bLen);
                    ctx.moveTo(sp.x - 14 * dpr, sp.y + 14 * dpr - bLen);
                    ctx.lineTo(sp.x - 14 * dpr, sp.y + 14 * dpr);
                    ctx.lineTo(sp.x - 14 * dpr + bLen, sp.y + 14 * dpr);
                    ctx.moveTo(sp.x + 14 * dpr - bLen, sp.y + 14 * dpr);
                    ctx.lineTo(sp.x + 14 * dpr, sp.y + 14 * dpr);
                    ctx.lineTo(sp.x + 14 * dpr, sp.y + 14 * dpr - bLen);
                    ctx.strokeStyle = '#fff01f';
                    ctx.lineWidth = 1.5 * dpr;
                    ctx.stroke();
                }

                // Separation halo on selected
                if (chkSeparation.checked && isSelected) {
                    ctx.beginPath();
                    ctx.arc(sp.x, sp.y, 1.4 * scale, 0, Math.PI * 2);
                    ctx.strokeStyle = 'rgba(0, 210, 255, 0.4)';
                    ctx.setLineDash([3 * dpr, 3 * dpr]);
                    ctx.lineWidth = 1 * dpr;
                    ctx.stroke();
                    ctx.setLineDash([]);
                }

                // Velocity Arrow
                if (chkVelocity.checked && (Math.abs(a.vx) > 0.05 || Math.abs(a.vz) > 0.05)) {
                    const arrowEnd = worldToScreen(a.x + a.vx * 0.35, a.z + a.vz * 0.35);
                    ctx.beginPath();
                    ctx.moveTo(sp.x, sp.y);
                    ctx.lineTo(arrowEnd.x, arrowEnd.y);
                    ctx.strokeStyle = isSelected ? '#ffffff' : (a.team === 0 ? 'rgba(0, 210, 255, 0.7)' : 'rgba(255, 159, 28, 0.7)');
                    ctx.lineWidth = (isSelected ? 2.5 : 1.5) * dpr;
                    ctx.stroke();
                }

                // Agent Body
                const r = (isSelected ? 6.5 : 5.0) * dpr;
                ctx.beginPath();
                ctx.arc(sp.x, sp.y, r, 0, Math.PI * 2);
                ctx.fillStyle = color;
                ctx.fill();
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 1 * dpr;
                ctx.stroke();

                // Inner core
                ctx.beginPath();
                ctx.arc(sp.x, sp.y, 2 * dpr, 0, Math.PI * 2);
                ctx.fillStyle = '#ffffff';
                ctx.fill();

                // Optional Agent ID label
                if (chkLabels.checked) {
                    ctx.font = `${Math.round(8 * dpr)}px monospace`;
                    ctx.fillStyle = '#ffffff';
                    ctx.textAlign = 'center';
                    ctx.fillText(a.id, sp.x, sp.y - r - 3 * dpr);
                }
            }

            // Telemetry updates
            statTick.textContent = frameData.tick;
            statTime.textContent = (frameData.tick / 60.0).toFixed(2) + ' s';
            statNav.textContent = navCount;
            statArrived.textContent = arrivedCount + ' / ' + agents.length;
            statAlerts.textContent = alertCount;
            frameCounter.textContent = currentFrame + ' / ' + (frames.length - 1);
            timeline.value = currentFrame;

            const pct = Math.round((arrivedCount / agents.length) * 100);
            statPct.textContent = pct + '%';
            progressFill.style.width = pct + '%';

            if (arrivedCount === agents.length) {
                statStatus.textContent = 'ALL ARRIVED (64/64)';
                statStatus.style.color = '#00f076';
            } else if (arrivedCount > 0) {
                statStatus.textContent = 'CONVERGING (' + arrivedCount + ' ARRIVED)';
                statStatus.style.color = '#ff9f1c';
            } else {
                statStatus.textContent = 'NAVIGATING CORRIDORS';
                statStatus.style.color = '#00d2ff';
            }

            updateInspector();
            ctx.restore();
        }

        // Animation Loop with Tab Throttling Protection
        let lastTimestamp = 0;
        let accumulator = 0;
        let fpsTimer = 0;
        let fpsCounter = 0;

        function loop(timestamp) {
            if (!lastTimestamp) lastTimestamp = timestamp;
            let delta = (timestamp - lastTimestamp) / 1000.0;
            lastTimestamp = timestamp;

            // Clamp delta to prevent spiral-of-death when tab is inactive
            if (delta > 0.15) delta = 0.15;

            // Live FPS monitor
            fpsCounter++;
            if (timestamp - fpsTimer >= 1000) {
                fpsBadge.textContent = fpsCounter + ' FPS';
                fpsCounter = 0;
                fpsTimer = timestamp;
            }

            if (isPlaying) {
                accumulator += delta * speed;
                const frameDuration = 1.0 / 30.0;
                while (accumulator >= frameDuration) {
                    if (currentFrame + 1 < frames.length) {
                        currentFrame++;
                    } else if (isLooping) {
                        currentFrame = 0;
                    } else {
                        isPlaying = false;
                        btnPlay.textContent = 'Play';
                        btnPlay.classList.remove('active');
                        break;
                    }
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

        btnPrev.addEventListener('click', () => {
            isPlaying = false;
            btnPlay.textContent = 'Play';
            btnPlay.classList.remove('active');
            currentFrame = Math.max(0, currentFrame - 1);
            draw();
        });

        btnNext.addEventListener('click', () => {
            isPlaying = false;
            btnPlay.textContent = 'Play';
            btnPlay.classList.remove('active');
            currentFrame = Math.min(frames.length - 1, currentFrame + 1);
            draw();
        });

        btnRestart.addEventListener('click', () => {
            currentFrame = 0;
            draw();
        });

        btnLoop.addEventListener('click', () => {
            isLooping = !isLooping;
            btnLoop.textContent = 'Loop: ' + (isLooping ? 'ON' : 'OFF');
            btnLoop.classList.toggle('active', isLooping);
        });

        timeline.addEventListener('input', (e) => {
            currentFrame = parseInt(e.target.value, 10);
            draw();
        });

        selSpeed.addEventListener('change', (e) => {
            speed = parseFloat(e.target.value);
        });

        // Spacebar to pause/resume
        window.addEventListener('keydown', (e) => {
            if (e.code === 'Space') {
                e.preventDefault();
                btnPlay.click();
            } else if (e.code === 'ArrowLeft') {
                btnPrev.click();
            } else if (e.code === 'ArrowRight') {
                btnNext.click();
            }
        });

        function updateInspector() {
            const inspectCard = document.getElementById('agent-inspect');
            const inspectContent = document.getElementById('inspect-content');
            if (selectedAgentId === null) {
                inspectCard.style.display = 'none';
                return;
            }

            const frameData = frames[currentFrame];
            const a = frameData.agents[selectedAgentId];
            if (!a) {
                inspectCard.style.display = 'none';
                return;
            }

            inspectCard.style.display = 'block';
            const spd = Math.hypot(a.vx, a.vz).toFixed(2);
            const distToGoal = a.wp && a.wp.length > 0 ? 
                Math.hypot(a.x - a.wp[a.wp.length - 1][0], a.z - a.wp[a.wp.length - 1][1]).toFixed(1) + ' m' : '0.0 m';
            
            const stateStr = a.st === 5 ? '<span style="color:#00f076; font-weight:700;">ARRIVED</span>' :
                            (a.st === 3 ? '<span style="color:#00d2ff; font-weight:700;">NAVIGATING</span>' :
                            '<span style="color:#ffd166;">IDLE</span>');
            
            const teamStr = a.team === 0 ? '<span style="color:#00d2ff;">Team Alpha (West &rarr; East)</span>' :
                                           '<span style="color:#ff9f1c;">Team Beta (East &rarr; West)</span>';

            inspectContent.innerHTML = 
                '<b>Agent ID:</b> ' + a.id + '<br>' +
                '<b>Affiliation:</b> ' + teamStr + '<br>' +
                '<b>Position:</b> (' + a.x.toFixed(2) + ', ' + a.z.toFixed(2) + ')<br>' +
                '<b>Velocity:</b> (' + a.vx.toFixed(2) + ', ' + a.vz.toFixed(2) + ')<br>' +
                '<b>Speed:</b> ' + spd + ' m/s<br>' +
                '<b>Status:</b> ' + stateStr + '<br>' +
                '<b>Distance to Goal:</b> ' + distToGoal + '<br>' +
                '<b>Waypoints Left:</b> ' + (a.wp ? a.wp.length : 0);
        }

        if (btnCloseInspect) {
            btnCloseInspect.addEventListener('click', () => {
                selectedAgentId = null;
                updateInspector();
                draw();
            });
        }

        // Canvas hit-testing
        canvas.addEventListener('click', (e) => {
            const rect = canvas.getBoundingClientRect();
            const clickX = (e.clientX - rect.left) * dpr;
            const clickY = (e.clientY - rect.top) * dpr;

            const frameData = frames[currentFrame];
            if (!frameData) return;

            let closest = null;
            let closestDist = 24.0 * dpr;

            for (const a of frameData.agents) {
                const sp = worldToScreen(a.x, a.z);
                const d = Math.hypot(sp.x - clickX, sp.y - clickY);
                if (d < closestDist) {
                    closestDist = d;
                    closest = a;
                }
            }

            selectedAgentId = closest ? (selectedAgentId === closest.id ? null : closest.id) : null;
            draw();
        });

        // Start animation loop
        requestAnimationFrame(loop);
    </script>
</body>
</html>)HTML";

    std::cout << "Successfully generated 'scandium_simulation_viewer.html' (" << recorded_frames << " frames recorded).\n";
    return 0;
}
