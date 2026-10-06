#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace scandium::core {

// Small fixed worker pool for CPU-bound engine systems.
// Jobs are fire-and-forget internally, while parallel_for provides a
// deterministic chunking contract for systems whose work is order-independent.
class JobSystem {
public:
    explicit JobSystem(std::size_t worker_count = 0)
        : stopping_(false) {
        if (worker_count == 0) {
            worker_count = std::min<std::size_t>(
                16, std::max<std::size_t>(1, std::thread::hardware_concurrency()));
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
        }
        condition_.notify_one();
    }

    template <typename Fn>
    void parallel_for(std::size_t count, Fn&& fn) {
        if (count == 0) {
            return;
        }

        const std::size_t chunks = std::min(count, workers_.size() + 1);
        const std::size_t chunk_size = (count + chunks - 1) / chunks;

        std::mutex wait_mutex;
        std::condition_variable wait_condition;
        std::size_t remaining = chunks;

        for (std::size_t chunk = 0; chunk < chunks; ++chunk) {
            const std::size_t begin = chunk * chunk_size;
            const std::size_t end = std::min(count, begin + chunk_size);

            enqueue([&, begin, end] {
                for (std::size_t index = begin; index < end; ++index) {
                    fn(index);
                }

                {
                    std::lock_guard lock(wait_mutex);
                    --remaining;
                }
                wait_condition.notify_one();
            });
        }

        std::unique_lock lock(wait_mutex);
        wait_condition.wait(lock, [&] { return remaining == 0; });
    }

    [[nodiscard]] std::size_t worker_count() const noexcept {
        return workers_.size();
    }

private:
    void worker_loop() {
        while (true) {
            std::function<void()> job;

            {
                std::unique_lock lock(mutex_);
                condition_.wait(lock, [&] {
                    return stopping_ || !jobs_.empty();
                });

                if (stopping_ && jobs_.empty()) {
                    return;
                }

                job = std::move(jobs_.front());
                jobs_.pop();
            }

            job();
        }
    }

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> jobs_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_;
};

} // namespace scandium::core
