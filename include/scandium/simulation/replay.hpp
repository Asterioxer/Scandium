#pragma once

#include <cstdint>
#include <vector>

namespace scandium::simulation {

struct InputCommand {
    std::uint64_t tick{};
    std::int32_t target_x{};
    std::int32_t target_z{};

    friend bool operator==(const InputCommand&, const InputCommand&) = default;
};

class Replay {
public:
    void record(InputCommand command) {
        commands_.push_back(command);
    }

    void clear() noexcept {
        commands_.clear();
        cursor_ = 0;
    }

    void rewind() noexcept {
        cursor_ = 0;
    }

    [[nodiscard]] bool has_next() const noexcept {
        return cursor_ < commands_.size();
    }

    [[nodiscard]] const InputCommand* next() noexcept {
        if (!has_next()) {
            return nullptr;
        }
        return &commands_[cursor_++];
    }

    [[nodiscard]] const std::vector<InputCommand>& commands() const noexcept {
        return commands_;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return commands_.size();
    }

private:
    std::vector<InputCommand> commands_;
    std::size_t cursor_{};
};

} // namespace scandium::simulation
