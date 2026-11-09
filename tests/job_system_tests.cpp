#include <atomic>
#include <cassert>
#include <iostream>
#include <numeric>
#include <vector>

#include "scandium/core/job_system.hpp"
#include "scandium/core/work_stealing_queue.hpp"

using namespace scandium::core;

void test_job_system_basic() {
    std::cout << "Starting test_job_system_basic..." << std::endl;
    JobSystem jobs(4);
    assert(jobs.worker_count() == 4);

    std::atomic<int> counter{0};
    for (int i = 0; i < 50; ++i) {
        jobs.enqueue([&counter] {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    jobs.wait_idle();
    assert(counter.load() == 50);
    std::cout << "Passed test_job_system_basic." << std::endl;
}

void test_parallel_for_correctness() {
    std::cout << "Starting test_parallel_for_correctness..." << std::endl;
    JobSystem jobs(4);

    for (const std::size_t count : {0ULL, 1ULL, 2ULL, 13ULL, 256ULL, 1000ULL, 50000ULL}) {
        std::vector<std::size_t> data(count, 0);
        jobs.parallel_for(count, [&data](std::size_t idx) {
            data[idx] = idx * 2 + 1;
        });

        for (std::size_t i = 0; i < count; ++i) {
            assert(data[i] == i * 2 + 1);
        }
    }
    std::cout << "Passed test_parallel_for_correctness." << std::endl;
}

void test_nested_parallel_for() {
    std::cout << "Starting test_nested_parallel_for..." << std::endl;
    JobSystem jobs(4);
    constexpr std::size_t outer_count = 10;
    constexpr std::size_t inner_count = 100;

    std::vector<std::vector<int>> matrix(outer_count, std::vector<int>(inner_count, 0));

    jobs.parallel_for(outer_count, [&](std::size_t i) {
        jobs.parallel_for(inner_count, [&](std::size_t j) {
            matrix[i][j] = static_cast<int>(i * 1000 + j);
        });
    });

    for (std::size_t i = 0; i < outer_count; ++i) {
        for (std::size_t j = 0; j < inner_count; ++j) {
            assert(matrix[i][j] == static_cast<int>(i * 1000 + j));
        }
    }
    std::cout << "Passed test_nested_parallel_for." << std::endl;
}

void test_exception_resilience() {
    std::cout << "Starting test_exception_resilience..." << std::endl;
    JobSystem jobs(2);

    jobs.enqueue([] {
        throw std::runtime_error("Simulated worker error");
    });

    jobs.wait_idle();

    std::atomic<bool> executed{false};
    jobs.enqueue([&executed] {
        executed.store(true);
    });

    jobs.wait_idle();
    assert(executed.load());
    std::cout << "Passed test_exception_resilience." << std::endl;
}

void test_concurrency_stress() {
    std::cout << "Starting test_concurrency_stress..." << std::endl;

    // 1. Rapid lifecycle stress (clean startup and shutdown)
    for (int cycle = 0; cycle < 10; ++cycle) {
        JobSystem quick_pool(8);
        std::atomic<int> local_count{0};
        for (int i = 0; i < 50; ++i) {
            quick_pool.enqueue([&local_count] {
                local_count.fetch_add(1, std::memory_order_relaxed);
            });
        }
        quick_pool.wait_idle();
        assert(local_count.load() == 50);
    }

    // 2. Multi-producer contention stress
    {
        JobSystem shared_pool(16);
        std::atomic<std::int64_t> total_sum{0};
        constexpr int producer_count = 8;
        constexpr int jobs_per_producer = 500;

        std::vector<std::thread> producers;
        producers.reserve(producer_count);

        for (int p = 0; p < producer_count; ++p) {
            producers.emplace_back([&shared_pool, &total_sum, p] {
                for (int j = 0; j < jobs_per_producer; ++j) {
                    shared_pool.enqueue([&total_sum, p, j] {
                        total_sum.fetch_add(p * 1000 + j, std::memory_order_relaxed);
                    });
                }
            });
        }

        for (auto& t : producers) {
            t.join();
        }

        shared_pool.wait_idle();

        std::int64_t expected_sum = 0;
        for (int p = 0; p < producer_count; ++p) {
            for (int j = 0; j < jobs_per_producer; ++j) {
                expected_sum += (p * 1000 + j);
            }
        }
        assert(total_sum.load() == expected_sum);
    }

    // 3. High element parallel_for stress
    {
        JobSystem pool(16);
        constexpr std::size_t N = 500'000;
        std::vector<std::uint32_t> buffer(N, 0);

        pool.parallel_for(N, [&buffer](std::size_t idx) {
            buffer[idx] = static_cast<std::uint32_t>(idx ^ 0x5A5A5A5A);
        });

        for (std::size_t i = 0; i < N; ++i) {
            assert(buffer[i] == static_cast<std::uint32_t>(i ^ 0x5A5A5A5A));
        }
    }

    std::cout << "Passed test_concurrency_stress." << std::endl;
}

void test_adaptive_scheduling() {
    std::cout << "Starting test_adaptive_scheduling..." << std::endl;
    JobSystem jobs(4);
    assert(jobs.min_elements_per_task() == 1024);

    jobs.set_min_elements_per_task(500);
    assert(jobs.min_elements_per_task() == 500);

    const auto caller_id = std::this_thread::get_id();

    // Workload below threshold: runs sequentially on caller thread
    constexpr std::size_t small_count = 250;
    std::atomic<bool> executed_on_other_thread{false};
    std::vector<int> small_data(small_count, 0);

    jobs.parallel_for(small_count, [&](std::size_t idx) {
        if (std::this_thread::get_id() != caller_id) {
            executed_on_other_thread.store(true);
        }
        small_data[idx] = static_cast<int>(idx * 3);
    });

    assert(!executed_on_other_thread.load());
    for (std::size_t i = 0; i < small_count; ++i) {
        assert(small_data[i] == static_cast<int>(i * 3));
    }

    // Workload above threshold: partitions across workers
    constexpr std::size_t large_count = 2000;
    std::vector<int> large_data(large_count, 0);
    jobs.parallel_for(large_count, [&](std::size_t idx) {
        large_data[idx] = static_cast<int>(idx * 5);
    });

    for (std::size_t i = 0; i < large_count; ++i) {
        assert(large_data[i] == static_cast<int>(i * 5));
    }

    std::cout << "Passed test_adaptive_scheduling." << std::endl;
}

void test_rapid_fire_parallel_for_stress() {
    std::cout << "Starting test_rapid_fire_parallel_for_stress..." << std::endl;
    for (const std::size_t workers : {1ULL, 2ULL, 4ULL, 8ULL, 16ULL}) {
        JobSystem jobs(workers);
        constexpr std::size_t workload = 3000;
        constexpr int rounds = 30;
        std::vector<int> data(workload, 0);

        for (int r = 0; r < rounds; ++r) {
            jobs.parallel_for(workload, [&](std::size_t idx) {
                data[idx] += 1;
            });
        }

        for (std::size_t i = 0; i < workload; ++i) {
            assert(data[i] == rounds);
        }
    }
    std::cout << "Passed test_rapid_fire_parallel_for_stress." << std::endl;
}

void test_work_stealing_queue() {
    std::cout << "Starting test_work_stealing_queue..." << std::endl;
    // 1. Basic LIFO push / pop
    {
        WorkStealingQueue<int, 64> q;
        assert(q.empty());
        assert(q.size() == 0);

        for (int i = 1; i <= 10; ++i) {
            bool ok = q.push(i);
            assert(ok);
        }
        assert(!q.empty());
        assert(q.size() == 10);

        // LIFO order for owner pop
        for (int i = 10; i >= 1; --i) {
            auto val = q.pop();
            assert(val.has_value());
            assert(*val == i);
        }
        assert(q.empty());
        assert(!q.pop().has_value());
    }

    // 2. Multi-threaded steal contention
    {
        constexpr int num_items = 2000;
        WorkStealingQueue<int, 4096> q;
        for (int i = 0; i < num_items; ++i) {
            q.push(i);
        }

        std::atomic<bool> start{false};
        std::atomic<int> stolen_count{0};
        std::atomic<int> popped_count{0};
        std::vector<int> collected;
        std::mutex collected_mutex;

        // 4 concurrent stealer threads
        std::vector<std::thread> stealers;
        for (int s = 0; s < 4; ++s) {
            stealers.emplace_back([&] {
                while (!start.load(std::memory_order_acquire)) {
                    std::this_thread::yield();
                }
                while (true) {
                    auto item = q.steal();
                    if (item.has_value()) {
                        stolen_count.fetch_add(1, std::memory_order_relaxed);
                        std::lock_guard lock(collected_mutex);
                        collected.push_back(*item);
                    } else if (q.empty()) {
                        break;
                    }
                }
            });
        }

        // Owner thread pops concurrently
        start.store(true, std::memory_order_release);
        while (true) {
            auto item = q.pop();
            if (item.has_value()) {
                popped_count.fetch_add(1, std::memory_order_relaxed);
                std::lock_guard lock(collected_mutex);
                collected.push_back(*item);
            } else if (q.empty()) {
                break;
            }
        }

        for (auto& t : stealers) {
            t.join();
        }

        assert(collected.size() == num_items);
        std::sort(collected.begin(), collected.end());
        for (int i = 0; i < num_items; ++i) {
            assert(collected[i] == i); // No dropped or duplicate items!
        }
        assert(stolen_count.load() + popped_count.load() == num_items);
    }
    std::cout << "Passed test_work_stealing_queue." << std::endl;
}

int main() {
    test_work_stealing_queue();
    test_job_system_basic();
    test_parallel_for_correctness();
    test_nested_parallel_for();
    test_exception_resilience();
    test_concurrency_stress();
    test_adaptive_scheduling();
    test_rapid_fire_parallel_for_stress();
    std::cout << "All job system tests passed successfully.\n";
    return 0;
}
