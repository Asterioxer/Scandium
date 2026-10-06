#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <unordered_map>
#include <vector>

namespace scandium::network {

struct InputFrame {
    std::uint64_t tick{};
    std::int32_t axis_x{};
    std::int32_t axis_y{};

    friend bool operator==(const InputFrame&, const InputFrame&) = default;
};

class LockstepSession {
public:
    explicit LockstepSession(std::vector<std::uint32_t> peers)
        : peers_(std::move(peers)) {}

    void submit(std::uint32_t peer, InputFrame input) {
        if (!is_peer(peer)) {
            return;
        }
        frames_[input.tick][peer] = input;
    }

    [[nodiscard]] bool ready(std::uint64_t tick) const {
        const auto tick_it = frames_.find(tick);
        if (tick_it == frames_.end()) {
            return false;
        }
        for (const auto peer : peers_) {
            if (tick_it->second.find(peer) == tick_it->second.end()) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] std::optional<std::vector<InputFrame>> consume(
        std::uint64_t tick) {

        const auto tick_it = frames_.find(tick);
        if (tick_it == frames_.end()) {
            return std::nullopt;
        }

        for (const auto peer : peers_) {
            if (tick_it->second.find(peer) == tick_it->second.end()) {
                return std::nullopt;
            }
        }

        std::vector<InputFrame> result;
        result.reserve(peers_.size());
        for (const auto peer : peers_) {
            result.push_back(tick_it->second.at(peer));
        }

        frames_.erase(tick_it);
        return result;
    }

    [[nodiscard]] std::size_t peer_count() const noexcept {
        return peers_.size();
    }

private:
    [[nodiscard]] bool is_peer(std::uint32_t peer) const noexcept {
        for (const auto candidate : peers_) {
            if (candidate == peer) return true;
        }
        return false;
    }

    std::vector<std::uint32_t> peers_;
    std::map<std::uint64_t,
             std::unordered_map<std::uint32_t, InputFrame>> frames_;
};

} // namespace scandium::network
