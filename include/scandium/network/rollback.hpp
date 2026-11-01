#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

namespace scandium::network {

/**
 * High-performance circular history buffer for deterministic simulation snapshots.
 * Backed by fixed-capacity compile-time array with O(1) bitwise modular addressing.
 */
template <typename State, std::size_t Capacity = 128>
class StateRingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

public:
    StateRingBuffer() = default;

    void store(std::uint64_t tick, State state) {
        const std::size_t slot = tick & (Capacity - 1);
        entries_[slot] = Entry{tick, std::move(state), true};
        if (empty_ || tick > newest_tick_) {
            newest_tick_ = tick;
        }
        if (empty_ || tick < oldest_tick_) {
            oldest_tick_ = tick;
        }
        empty_ = false;
    }

    [[nodiscard]] const State* get(std::uint64_t tick) const noexcept {
        const std::size_t slot = tick & (Capacity - 1);
        if (!entries_[slot].valid || entries_[slot].tick != tick) {
            return nullptr;
        }
        return &entries_[slot].state;
    }

    [[nodiscard]] State* get_mutable(std::uint64_t tick) noexcept {
        const std::size_t slot = tick & (Capacity - 1);
        if (!entries_[slot].valid || entries_[slot].tick != tick) {
            return nullptr;
        }
        return &entries_[slot].state;
    }

    [[nodiscard]] bool contains(std::uint64_t tick) const noexcept {
        return get(tick) != nullptr;
    }

    [[nodiscard]] bool empty() const noexcept {
        return empty_;
    }

    [[nodiscard]] std::uint64_t newest_tick() const noexcept {
        return newest_tick_;
    }

    [[nodiscard]] std::uint64_t oldest_tick() const noexcept {
        return oldest_tick_;
    }

    [[nodiscard]] constexpr std::size_t capacity() const noexcept {
        return Capacity;
    }

    void clear() noexcept {
        for (auto& entry : entries_) {
            entry.valid = false;
        }
        empty_ = true;
        newest_tick_ = 0;
        oldest_tick_ = 0;
    }

private:
    struct Entry {
        std::uint64_t tick{0};
        State state{};
        bool valid{false};
    };

    std::array<Entry, Capacity> entries_{};
    std::uint64_t newest_tick_{0};
    std::uint64_t oldest_tick_{0};
    bool empty_{true};
};

/**
 * Deterministic Client-Side Prediction and Rollback Netcode Controller.
 * Handles zero-latency local prediction, speculative execution, server reconciliation,
 * and bit-exact state rewind/re-simulation on authoritative divergence.
 */
template <typename World, typename Snapshot, typename Input, std::size_t HistorySize = 128>
class RollbackController {
public:
    using StepFunction = std::function<void(World&, std::uint64_t tick, const std::unordered_map<std::uint32_t, Input>&)>;

    RollbackController(World& world,
                       std::vector<std::uint32_t> player_ids,
                       std::uint32_t local_player_id,
                       StepFunction step_fn)
        : world_(world),
          players_(std::move(player_ids)),
          local_player_id_(local_player_id),
          step_fn_(std::move(step_fn)) {}

    [[nodiscard]] std::uint64_t current_tick() const noexcept {
        return predicted_tick_;
    }

    [[nodiscard]] std::uint64_t authoritative_tick() const noexcept {
        return authoritative_tick_;
    }

    [[nodiscard]] std::uint64_t total_predictions() const noexcept {
        return total_predictions_;
    }

    [[nodiscard]] std::uint64_t total_rollbacks() const noexcept {
        return total_rollbacks_;
    }

    [[nodiscard]] std::uint64_t max_rollback_frames() const noexcept {
        return max_rollback_frames_;
    }

    /**
     * Executes one client-side predicted simulation tick with local input immediately.
     * Speculatively predicts remote player inputs by repeating last confirmed inputs.
     */
    void predict_local_tick(const Input& local_input) {
        // 1. Save pre-step snapshot of the current predicted state
        history_.store(predicted_tick_, world_.capture_snapshot(predicted_tick_));

        // 2. Register local input for this tick
        inputs_[predicted_tick_][local_player_id_] = local_input;

        // 3. Speculatively fill in inputs for remote players if not already known
        for (const auto pid : players_) {
            if (pid != local_player_id_ && inputs_[predicted_tick_].find(pid) == inputs_[predicted_tick_].end()) {
                // Speculate by repeating last known input or default
                inputs_[predicted_tick_][pid] = last_known_inputs_[pid];
            }
        }

        // 4. Step world forward
        step_fn_(world_, predicted_tick_, inputs_[predicted_tick_]);
        ++predicted_tick_;
        ++total_predictions_;
    }

    /**
     * Receives an authoritative input frame from the network for a specific player at a given tick.
     * If this input diverges from our speculative prediction for a past tick,
     * triggers rollback and fast-forwards the simulation back to current predicted_tick.
     */
    bool receive_authoritative_input(std::uint32_t player_id, std::uint64_t tick, const Input& auth_input) {
        if (tick < authoritative_tick_) {
            // Drop stale packet older than verified baseline
            return false;
        }

        last_known_inputs_[player_id] = auth_input;

        // Check if we already predicted this tick
        auto& tick_inputs = inputs_[tick];
        const auto it = tick_inputs.find(player_id);
        const bool had_predicted = (it != tick_inputs.end());
        const bool matched = had_predicted && (it->second == auth_input);

        // Update stored input with authoritative truth
        tick_inputs[player_id] = auth_input;

        if (tick < predicted_tick_) {
            if (!matched) {
                // Divergence detected! Execute rollback and resimulate
                perform_rollback(tick);
                return true;
            }
        }

        // Update authoritative watermark if all players confirmed
        update_authoritative_watermark();
        return false;
    }

private:
    void perform_rollback(std::uint64_t rollback_tick) {
        const auto* snapshot = history_.get(rollback_tick);
        if (!snapshot) {
            // Cannot rollback further than history capacity
            return;
        }

        const std::uint64_t frames_rewound = predicted_tick_ - rollback_tick;
        if (frames_rewound > max_rollback_frames_) {
            max_rollback_frames_ = frames_rewound;
        }
        ++total_rollbacks_;

        // 1. Restore simulation state at rollback point
        world_.restore_snapshot(*snapshot);

        // 2. Resimulate forward from rollback_tick to predicted_tick_
        for (std::uint64_t t = rollback_tick; t < predicted_tick_; ++t) {
            // Update snapshot at this tick along the new trajectory
            history_.store(t, world_.capture_snapshot(t));
            step_fn_(world_, t, inputs_[t]);
        }
    }

    void update_authoritative_watermark() {
        while (authoritative_tick_ < predicted_tick_) {
            const auto it = inputs_.find(authoritative_tick_);
            if (it == inputs_.end()) break;

            bool all_confirmed = true;
            for (const auto pid : players_) {
                if (it->second.find(pid) == it->second.end()) {
                    all_confirmed = false;
                    break;
                }
            }

            if (!all_confirmed) break;
            ++authoritative_tick_;
        }
    }

    World& world_;
    std::vector<std::uint32_t> players_;
    std::uint32_t local_player_id_{0};
    StepFunction step_fn_;

    std::uint64_t predicted_tick_{0};
    std::uint64_t authoritative_tick_{0};

    StateRingBuffer<Snapshot, HistorySize> history_;
    std::unordered_map<std::uint64_t, std::unordered_map<std::uint32_t, Input>> inputs_;
    std::unordered_map<std::uint32_t, Input> last_known_inputs_;

    std::uint64_t total_predictions_{0};
    std::uint64_t total_rollbacks_{0};
    std::uint64_t max_rollback_frames_{0};
};

} // namespace scandium::network
