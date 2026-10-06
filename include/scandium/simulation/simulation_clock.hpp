#pragma once

#include <cstddef>
#include <functional>
#include <utility>

namespace scandium::simulation {

class SimulationClock {
public:
    struct Config {
        double fixed_delta_seconds{1.0 / 60.0};
        std::size_t max_steps_per_frame{8};
    };

    SimulationClock() noexcept : SimulationClock(Config{}) {}

    explicit SimulationClock(Config config) noexcept
        : config_{
              config.fixed_delta_seconds > 0.0 ? config.fixed_delta_seconds
                                                : (1.0 / 60.0),
              config.max_steps_per_frame > 0 ? config.max_steps_per_frame : 1} {}

    template <typename StepFn>
    std::size_t advance(double frame_delta_seconds, StepFn&& step) {
        if (frame_delta_seconds < 0.0) {
            return 0;
        }

        accumulator_seconds_ += frame_delta_seconds;

        std::size_t steps = 0;
        while (accumulator_seconds_ >= config_.fixed_delta_seconds &&
               steps < config_.max_steps_per_frame) {
            std::invoke(step, config_.fixed_delta_seconds);
            accumulator_seconds_ -= config_.fixed_delta_seconds;
            ++steps;
            ++total_steps_;
        }

        // Prevent a long stall from causing an unbounded catch-up loop.
        if (steps == config_.max_steps_per_frame &&
            accumulator_seconds_ >= config_.fixed_delta_seconds) {
            accumulator_seconds_ = config_.fixed_delta_seconds * 0.999999;
        }

        return steps;
    }

    [[nodiscard]] double interpolation_alpha() const noexcept {
        return accumulator_seconds_ / config_.fixed_delta_seconds;
    }

    [[nodiscard]] double fixed_delta_seconds() const noexcept {
        return config_.fixed_delta_seconds;
    }

    [[nodiscard]] std::size_t total_steps() const noexcept {
        return total_steps_;
    }

    [[nodiscard]] double accumulated_seconds() const noexcept {
        return accumulator_seconds_;
    }

    void reset() noexcept {
        accumulator_seconds_ = 0.0;
        total_steps_ = 0;
    }

private:
    Config config_;
    double accumulator_seconds_{0.0};
    std::size_t total_steps_{0};
};

} // namespace scandium::simulation
