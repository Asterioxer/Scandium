#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <vector>

#include "scandium/core/job_system.hpp"

using Clock = std::chrono::steady_clock;
using namespace scandium::core;

template <typename T>
void do_not_optimize(T&& val) {
    volatile auto sink = val;
    (void)sink;
}

double median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t mid = v.size() / 2;
    return (v.size() % 2 == 0) ? (v[mid - 1] + v[mid]) * 0.5 : v[mid];
}

int main() {
    std::cout << "SCANDIUM JOB SYSTEM RIGOROUS THROUGHPUT BENCHMARK\n";
    std::cout << "=================================================\n";
    std::cout << "workload_type,workload_size,workers,adaptive_threshold,median_ms,speedup_vs_1w\n";

    constexpr int warmup_trials = 2;
    constexpr int measured_trials = 5;

    for (const std::size_t workload : {500ULL, 5000ULL, 100000ULL}) {
        double baseline_1w = 0.0;

        for (const std::size_t workers : {1ULL, 2ULL, 4ULL, 8ULL, 16ULL}) {
            JobSystem jobs(workers);
            std::vector<float> data(workload, 1.0f);

            std::vector<double> trial_times;
            trial_times.reserve(measured_trials);

            for (int t = 0; t < warmup_trials + measured_trials; ++t) {
                constexpr int rounds = 20;
                const auto start = Clock::now();

                for (int r = 0; r < rounds; ++r) {
                    jobs.parallel_for(workload, [&](std::size_t i) {
                        data[i] = std::sqrt(data[i] * 1.5f + 2.0f);
                    });
                }

                const auto end = Clock::now();
                const double elapsed_ms =
                    std::chrono::duration<double, std::milli>(end - start).count() / rounds;

                do_not_optimize(data[0]);

                if (t >= warmup_trials) {
                    trial_times.push_back(elapsed_ms);
                }
            }

            const double med_ms = median(trial_times);
            if (workers == 1) {
                baseline_1w = med_ms;
            }

            const double speedup = baseline_1w > 0.0 ? baseline_1w / med_ms : 1.0;
            const std::string w_type = (workload <= 1000) ? "SmallMicro" : ((workload <= 10000) ? "Medium" : "LargeBulk");

            std::cout << w_type << ","
                      << workload << ","
                      << workers << ","
                      << jobs.min_elements_per_task() << ","
                      << std::fixed << std::setprecision(4)
                      << med_ms << ","
                      << std::setprecision(2) << speedup << "x\n";
        }
    }

    return 0;
}
