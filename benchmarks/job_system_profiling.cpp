#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include "scandium/core/job_system.hpp"
#include "scandium/profiling/benchmark_harness.hpp"

using Clock = std::chrono::steady_clock;
using namespace scandium::core;
using scandium::bench::compute_stats;
using scandium::bench::do_not_optimize;

void profile_job_system_workloads() {
    std::cout << "SCANDIUM JOB SYSTEM DETAILED PROFILING\n";
    std::cout << "======================================\n";
    std::cout << "workload_type,size,workers,min_chunk,seq_ms,parallel_ms,speedup,sync_overhead_us\n";

    constexpr int warmup_trials = 3;
    constexpr int measured_trials = 10;

    for (const std::size_t workload : {500ULL, 5000ULL, 25000ULL, 100000ULL}) {
        // Measure sequential baseline
        std::vector<float> data_seq(workload, 1.0f);
        std::vector<double> seq_samples;
        seq_samples.reserve(measured_trials);

        for (int t = 0; t < warmup_trials + measured_trials; ++t) {
            const auto t0 = Clock::now();
            for (std::size_t i = 0; i < workload; ++i) {
                data_seq[i] = std::sqrt(data_seq[i] * 1.5f + 2.0f);
            }
            const auto t1 = Clock::now();
            const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            do_not_optimize(data_seq[0]);
            if (t >= warmup_trials) {
                seq_samples.push_back(ms);
            }
        }
        const auto seq_stats = compute_stats(seq_samples);

        for (const std::size_t workers : {1ULL, 2ULL, 4ULL, 8ULL, 16ULL}) {
            JobSystem jobs(workers);
            std::vector<float> data(workload, 1.0f);
            std::vector<double> par_samples;
            par_samples.reserve(measured_trials);

            for (int t = 0; t < warmup_trials + measured_trials; ++t) {
                const auto t0 = Clock::now();
                jobs.parallel_for(workload, [&](std::size_t i) {
                    data[i] = std::sqrt(data[i] * 1.5f + 2.0f);
                });
                const auto t1 = Clock::now();
                const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                do_not_optimize(data[0]);
                if (t >= warmup_trials) {
                    par_samples.push_back(ms);
                }
            }

            const auto par_stats = compute_stats(par_samples);
            const double speedup = seq_stats.median_ms > 0.0 ? seq_stats.median_ms / par_stats.median_ms : 1.0;
            const double sync_overhead_us = (par_stats.median_ms - (seq_stats.median_ms / workers)) * 1000.0;
            const std::string wtype = (workload <= 1000) ? "Micro" : ((workload <= 10000) ? "Medium" : "Bulk");

            std::cout << wtype << ","
                      << workload << ","
                      << workers << ","
                      << jobs.min_elements_per_task() << ","
                      << std::fixed << std::setprecision(4)
                      << seq_stats.median_ms << ","
                      << par_stats.median_ms << ","
                      << std::setprecision(2) << speedup << "x,"
                      << std::setprecision(1) << sync_overhead_us << "us\n";
        }
    }
}

int main() {
    profile_job_system_workloads();
    return 0;
}
