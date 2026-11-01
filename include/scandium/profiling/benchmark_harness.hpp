#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace scandium::bench {

// Assembly / memory sink preventing compiler dead-code elimination
template <typename T>
inline void do_not_optimize(T&& val) {
    volatile auto sink = val;
    (void)sink;
}

struct BenchmarkStats {
    std::size_t sample_count{0};
    double min_ms{0.0};
    double max_ms{0.0};
    double mean_ms{0.0};
    double median_ms{0.0};
    double stddev_ms{0.0};
    double p90_ms{0.0};
    double p99_ms{0.0};

    [[nodiscard]] std::string to_csv() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4)
           << median_ms << ","
           << mean_ms << ","
           << stddev_ms << ","
           << min_ms << ","
           << max_ms << ","
           << p90_ms << ","
           << p99_ms;
        return ss.str();
    }

    [[nodiscard]] std::string to_json() const {
        std::ostringstream ss;
        ss << "{"
           << "\"samples\":" << sample_count << ","
           << std::fixed << std::setprecision(4)
           << "\"median_ms\":" << median_ms << ","
           << "\"mean_ms\":" << mean_ms << ","
           << "\"stddev_ms\":" << stddev_ms << ","
           << "\"min_ms\":" << min_ms << ","
           << "\"max_ms\":" << max_ms << ","
           << "\"p90_ms\":" << p90_ms << ","
           << "\"p99_ms\":" << p99_ms
           << "}";
        return ss.str();
    }
};

inline BenchmarkStats compute_stats(std::vector<double> samples) {
    if (samples.empty()) {
        return {};
    }

    std::sort(samples.begin(), samples.end());
    const std::size_t n = samples.size();

    const double min_val = samples.front();
    const double max_val = samples.back();

    const double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    const double mean = sum / static_cast<double>(n);

    double var_sum = 0.0;
    for (double val : samples) {
        const double diff = val - mean;
        var_sum += diff * diff;
    }
    const double stddev = (n > 1) ? std::sqrt(var_sum / static_cast<double>(n - 1)) : 0.0;

    const auto percentile = [&](double p) -> double {
        if (n == 1) return samples[0];
        const double rank = p * static_cast<double>(n - 1);
        const auto low = static_cast<std::size_t>(std::floor(rank));
        const auto high = static_cast<std::size_t>(std::ceil(rank));
        const double weight = rank - static_cast<double>(low);
        return samples[low] * (1.0 - weight) + samples[high] * weight;
    };

    const double p50 = percentile(0.50);
    const double p90 = percentile(0.90);
    const double p99 = percentile(0.99);

    return BenchmarkStats{
        n, min_val, max_val, mean, p50, stddev, p90, p99
    };
}

template <typename SetupFn, typename ExecFn>
BenchmarkStats measure(int warmup_trials, int measured_trials, SetupFn&& setup, ExecFn&& exec) {
    using Clock = std::chrono::steady_clock;

    // Warm-up trials (discarded)
    for (int w = 0; w < warmup_trials; ++w) {
        auto state = setup();
        exec(state);
    }

    // Measured trials
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(measured_trials));

    for (int t = 0; t < measured_trials; ++t) {
        auto state = setup();
        const auto start = Clock::now();
        exec(state);
        const auto finish = Clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(finish - start).count();
        samples.push_back(elapsed_ms);
    }

    return compute_stats(std::move(samples));
}

template <typename ExecFn>
BenchmarkStats measure_simple(int warmup_trials, int measured_trials, ExecFn&& exec) {
    return measure(warmup_trials, measured_trials, []() { return 0; }, [&](int) { exec(); });
}

} // namespace scandium::bench
