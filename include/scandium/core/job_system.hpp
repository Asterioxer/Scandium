#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace scandium::core {

// High-performance CPU job system with caller-assisted parallel execution,
// inline functor storage (zero heap allocations for standard job captures),
// and stack-safe task synchronization.
class JobSystem {
public:
    class Job {
    public:
        Job() noexcept = default;

        template <typename Fn, typename = std::enable_if_t<!std::is_same_v<std::decay_t<Fn>, Job>>>
        Job(Fn&& fn) {
            using Decayed = std::decay_t<Fn>;
            static_assert(sizeof(Decayed) <= kInlineStorageSize, "Job functor exceeds inline storage size");
            static_assert(alignof(Decayed) <= alignof(std::max_align_t), "Job alignment not supported");
            ::new (static_cast<void*>(storage_)) Decayed(std::forward<Fn>(fn));
            invoker_ = [](void* ptr) {
                (*static_cast<Decayed*>(ptr))();
            };
            destructor_ = [](void* ptr) {
                static_cast<Decayed*>(ptr)->~Decayed();
            };
        }

        ~Job() {
            reset();
        }

        Job(Job&& other) noexcept {
            move_from(std::move(other));
        }

        Job& operator=(Job&& other) noexcept {
            if (this != &other) {
                reset();
                move_from(std::move(other));
            }
            return *this;
        }

        Job(const Job&) = delete;
        Job& operator=(const Job&) = delete;

        explicit operator bool() const noexcept {
            return invoker_ != nullptr;
        }

        void operator()() {
            if (invoker_) {
                invoker_(storage_);
            }
        }

    private:
        static constexpr std::size_t kInlineStorageSize = 64;
        alignas(std::max_align_t) std::byte storage_[kInlineStorageSize];
        void (*invoker_)(void*){nullptr};
        void (*destructor_)(void*){nullptr};

        void reset() noexcept {
            if (destructor_) {
                destructor_(storage_);
                destructor_ = nullptr;
                invoker_ = nullptr;
            }
        }

        void move_from(Job&& other) noexcept {
            invoker_ = other.invoker_;
            destructor_ = other.destructor_;
            if (other.invoker_) {
                for (std::size_t i = 0; i < kInlineStorageSize; ++i) {
                    storage_[i] = other.storage_[i];
                }
                other.invoker_ = nullptr;
                other.destructor_ = nullptr;
            }
        }
    };

    explicit JobSystem(std::size_t worker_count = 0)
        : stopping_(false) {
        if (worker_count == 0) {
            const auto hw = std::thread::hardware_concurrency();
            worker_count = std::min<std::size_t>(
                16, std::max<std::size_t>(1, hw == 0 ? 4 : hw));
        }
        workers_.reserve(worker_count);
        for (std::size_t i = 0; i < worker_count; ++i) {
            workers_.emplace_back([this] { worker_loop(); });
        }
    }

    ~JobSystem() {
        {
            std::lock_guard lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        for (auto& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    template <typename Fn>
    void enqueue(Fn&& fn) {
        {
            std::lock_guard lock(mutex_);
            jobs_.emplace(std::forward<Fn>(fn));
            ++pending_jobs_;
        }
        condition_.notify_one();
    }

    void set_min_elements_per_task(std::size_t min_elements) noexcept {
        min_elements_per_task_ = min_elements > 0 ? min_elements : 1;
    }

    [[nodiscard]] std::size_t min_elements_per_task() const noexcept {
        return min_elements_per_task_;
    }

    template <typename Fn>
    void parallel_for(std::size_t count, Fn&& fn) {
        if (count == 0) {
            return;
        }

        if (is_worker_thread() || workers_.size() <= 1 || count <= min_elements_per_task_) {
            for (std::size_t i = 0; i < count; ++i) {
                fn(i);
            }
            return;
        }

        const std::size_t workers = workers_.size();
        const std::size_t max_chunks = count / min_elements_per_task_;
        const std::size_t target_chunks = std::clamp<std::size_t>(max_chunks, 1, workers);
        const std::size_t chunk_size = (count + target_chunks - 1) / target_chunks;

        parallel_for_chunked(count, chunk_size, std::forward<Fn>(fn));
    }

    template <typename Fn>
    void parallel_for_chunked(std::size_t count, std::size_t chunk_size, Fn&& fn) {
        if (count == 0) {
            return;
        }

        if (is_worker_thread() || workers_.size() <= 1 || count <= chunk_size) {
            for (std::size_t i = 0; i < count; ++i) {
                fn(i);
            }
            return;
        }

        struct SyncToken {
            std::atomic<std::size_t> next_idx{0};
            std::atomic<std::size_t> active_tasks{0};
            std::mutex wait_mutex;
            std::condition_variable wait_cond;
        };

        const std::size_t total_chunks = (count + chunk_size - 1) / chunk_size;
        const std::size_t tasks_to_enqueue = total_chunks > 1 ? std::min(total_chunks - 1, workers_.size()) : 0;

        SyncToken token;
        token.active_tasks.store(tasks_to_enqueue + 1, std::memory_order_relaxed);

        for (std::size_t c = 0; c < tasks_to_enqueue; ++c) {
            enqueue([&token, count, chunk_size, &fn] {
                while (true) {
                    const std::size_t begin =
                        token.next_idx.fetch_add(chunk_size, std::memory_order_relaxed);
                    if (begin >= count) {
                        break;
                    }
                    const std::size_t end = std::min(count, begin + chunk_size);
                    for (std::size_t i = begin; i < end; ++i) {
                        fn(i);
                    }
                }

                if (token.active_tasks.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                    std::lock_guard lock(token.wait_mutex);
                    token.wait_cond.notify_all();
                }
            });
        }

        // Caller helps execute chunks directly
        while (true) {
            const std::size_t begin =
                token.next_idx.fetch_add(chunk_size, std::memory_order_relaxed);
            if (begin >= count) {
                break;
            }
            const std::size_t end = std::min(count, begin + chunk_size);
            for (std::size_t i = begin; i < end; ++i) {
                fn(i);
            }
        }

        // Wait for workers to finish all active tasks
        if (token.active_tasks.fetch_sub(1, std::memory_order_acq_rel) > 1) {
            while (token.active_tasks.load(std::memory_order_acquire) > 0) {
                Job job;
                {
                    std::unique_lock lock(mutex_);
                    if (!jobs_.empty()) {
                        job = std::move(jobs_.front());
                        jobs_.pop();
                    }
                }
                if (job) {
                    try {
                        job();
                    } catch (...) {
                    }
                    --pending_jobs_;
                } else {
                    std::unique_lock lock(token.wait_mutex);
                    token.wait_cond.wait_for(lock, std::chrono::microseconds(25), [&] {
                        return token.active_tasks.load(std::memory_order_acquire) == 0;
                    });
                }
            }
        }
    }

    void wait_idle() {
        while (pending_jobs_.load(std::memory_order_acquire) > 0) {
            Job job;
            {
                std::unique_lock lock(mutex_);
                if (!jobs_.empty()) {
                    job = std::move(jobs_.front());
                    jobs_.pop();
                }
            }
            if (job) {
                try {
                    job();
                } catch (...) {
                }
                --pending_jobs_;
            } else {
                std::this_thread::yield();
            }
        }
    }

    [[nodiscard]] std::size_t worker_count() const noexcept {
        return workers_.size();
    }

    [[nodiscard]] std::size_t pending_jobs() const noexcept {
        return pending_jobs_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] static bool is_worker_thread() noexcept {
        return is_worker_;
    }

private:
    void worker_loop() {
        is_worker_ = true;
        while (true) {
            Job job;
            {
                std::unique_lock lock(mutex_);
                condition_.wait(lock, [&] {
                    return stopping_ || !jobs_.empty();
                });

                if (stopping_ && jobs_.empty()) {
                    return;
                }

                if (!jobs_.empty()) {
                    job = std::move(jobs_.front());
                    jobs_.pop();
                }
            }

            if (job) {
                try {
                    job();
                } catch (...) {
                }
                --pending_jobs_;
            }
        }
    }

    static inline thread_local bool is_worker_{false};

    std::vector<std::thread> workers_;
    std::queue<Job> jobs_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::atomic<std::size_t> pending_jobs_{0};
    std::size_t min_elements_per_task_{1024};
    bool stopping_{false};
};

} // namespace scandium::core
