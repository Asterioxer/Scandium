#pragma once

#include <chrono>
#include <cstdint>
#include <string_view>

namespace scandium::profiling {

class ScopeTimer {
public:
    using clock = std::chrono::steady_clock;

    explicit ScopeTimer(std::string_view name)
        : name_(name), start_(clock::now()) {}

    ~ScopeTimer() {
        elapsed_microseconds =
            std::chrono::duration<double, std::micro>(clock::now() - start_).count();
    }

    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;

    [[nodiscard]] double microseconds() const noexcept {
        return elapsed_microseconds;
    }

private:
    std::string_view name_;
    clock::time_point start_;
    double elapsed_microseconds{};
};

struct FrameStats {
    std::uint64_t frame_count{};
    double total_milliseconds{};
    double max_milliseconds{};

    void record(double milliseconds) noexcept {
        ++frame_count;
        total_milliseconds += milliseconds;
        if (milliseconds > max_milliseconds) {
            max_milliseconds = milliseconds;
        }
    }

    [[nodiscard]] double average_milliseconds() const noexcept {
        return frame_count == 0 ? 0.0 : total_milliseconds / frame_count;
    }

    [[nodiscard]] double average_fps() const noexcept {
        const double average = average_milliseconds();
        return average <= 0.0 ? 0.0 : 1000.0 / average;
    }
};

} // namespace scandium::profiling
