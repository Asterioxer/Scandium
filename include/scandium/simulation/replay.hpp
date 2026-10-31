#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
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

    [[nodiscard]] const InputCommand* peek() const noexcept {
        if (!has_next()) {
            return nullptr;
        }
        return &commands_[cursor_];
    }

    [[nodiscard]] const InputCommand* find_command_for_tick(std::uint64_t tick) const noexcept {
        for (const auto& cmd : commands_) {
            if (cmd.tick == tick) {
                return &cmd;
            }
        }
        return nullptr;
    }

    [[nodiscard]] const std::vector<InputCommand>& commands() const noexcept {
        return commands_;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return commands_.size();
    }

    [[nodiscard]] std::size_t cursor() const noexcept {
        return cursor_;
    }

    bool save(std::ostream& out) const {
        out << "SCANDIUM_REPLAY_V1\n";
        out << commands_.size() << "\n";
        for (const auto& cmd : commands_) {
            out << cmd.tick << ' ' << cmd.target_x << ' ' << cmd.target_z << "\n";
        }
        return static_cast<bool>(out);
    }

    bool load(std::istream& in) {
        try {
            std::string header;
            if (!(in >> header) || header != "SCANDIUM_REPLAY_V1") {
                return false;
            }

            std::size_t count = 0;
            if (!(in >> count)) {
                return false;
            }

            // Cap reasonable maximum count to protect against malformed astronomical counts
            constexpr std::size_t max_commands = 10'000'000;
            if (count > max_commands) {
                return false;
            }

            std::vector<InputCommand> loaded;
            loaded.reserve(std::min<std::size_t>(count, 4096));
            for (std::size_t i = 0; i < count; ++i) {
                InputCommand cmd;
                if (!(in >> cmd.tick >> cmd.target_x >> cmd.target_z)) {
                    return false;
                }
                loaded.push_back(cmd);
            }

            commands_ = std::move(loaded);
            cursor_ = 0;
            return true;
        } catch (...) {
            return false;
        }
    }

    bool save_to_file(const std::string& path) const {
        std::ofstream file(path);
        if (!file.is_open()) {
            return false;
        }
        return save(file);
    }

    bool load_from_file(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            return false;
        }
        return load(file);
    }

    [[nodiscard]] std::string to_string() const {
        std::ostringstream ss;
        save(ss);
        return ss.str();
    }

    bool from_string(std::string_view data) {
        std::string str(data);
        std::istringstream ss(str);
        return load(ss);
    }

private:
    std::vector<InputCommand> commands_;
    std::size_t cursor_{0};
};

} // namespace scandium::simulation
