#pragma once

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace scandium::core {

/**
 * @brief Lock-free work-stealing double-ended queue based on the Chase-Lev algorithm.
 *
 * Single-producer (owner thread) operations:
 *   - push(): Pushes to bottom of deque (LIFO).
 *   - pop(): Pops from bottom of deque (LIFO, cache-friendly).
 *
 * Multi-consumer (stealer threads) operations:
 *   - steal(): Steals from top of deque (FIFO).
 *
 * Cache-line aligned (64-byte) indices eliminate false sharing between owner and stealers.
 */
template <typename T, std::size_t InitialCapacity = 1024>
class WorkStealingQueue {
    static_assert((InitialCapacity & (InitialCapacity - 1)) == 0, "InitialCapacity must be a power of two");

public:
    explicit WorkStealingQueue(std::size_t capacity = InitialCapacity)
        : capacity_(next_power_of_two(capacity))
        , mask_(capacity_ - 1)
        , buffer_(std::make_unique<Slot[]>(capacity_)) {
        top_.store(0, std::memory_order_relaxed);
        bottom_.store(0, std::memory_order_relaxed);
    }

    ~WorkStealingQueue() = default;

    WorkStealingQueue(const WorkStealingQueue&) = delete;
    WorkStealingQueue& operator=(const WorkStealingQueue&) = delete;

    WorkStealingQueue(WorkStealingQueue&& other) noexcept
        : capacity_(other.capacity_)
        , mask_(other.mask_)
        , buffer_(std::move(other.buffer_)) {
        top_.store(other.top_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        bottom_.store(other.bottom_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }

    WorkStealingQueue& operator=(WorkStealingQueue&& other) noexcept {
        if (this != &other) {
            capacity_ = other.capacity_;
            mask_ = other.mask_;
            buffer_ = std::move(other.buffer_);
            top_.store(other.top_.load(std::memory_order_relaxed), std::memory_order_relaxed);
            bottom_.store(other.bottom_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }
        return *this;
    }

    /**
     * @brief Pushes an element to the bottom of the queue.
     * Must only be called by the owning thread.
     * @param item The item to push.
     * @return true if pushed, false if capacity exceeded.
     */
    bool push(T item) {
        const std::int64_t b = bottom_.load(std::memory_order_relaxed);
        const std::int64_t t = top_.load(std::memory_order_acquire);

        if (b - t >= static_cast<std::int64_t>(capacity_)) {
            return false; // Queue is full
        }

        buffer_[b & mask_].item = std::move(item);
        std::atomic_thread_fence(std::memory_order_release);
        bottom_.store(b + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pops an element from the bottom of the queue (LIFO).
     * Must only be called by the owning thread.
     * @return std::optional containing the item, or std::nullopt if empty.
     */
    std::optional<T> pop() {
        const std::int64_t b = bottom_.load(std::memory_order_relaxed) - 1;
        bottom_.store(b, std::memory_order_seq_cst);
        const std::int64_t t = top_.load(std::memory_order_seq_cst);

        if (t <= b) {
            // Queue is non-empty
            if (t == b) {
                // Last item in queue: compete against concurrent stealers
                T item = std::move(buffer_[b & mask_].item);
                std::int64_t expected_top = t;
                if (!top_.compare_exchange_strong(expected_top, t + 1,
                                                   std::memory_order_seq_cst,
                                                   std::memory_order_relaxed)) {
                    // Lost race to stealer
                    bottom_.store(b + 1, std::memory_order_relaxed);
                    return std::nullopt;
                }
                bottom_.store(b + 1, std::memory_order_relaxed);
                return item;
            } else {
                // More than one item: owner is at bottom, stealers at top, no collision
                T item = std::move(buffer_[b & mask_].item);
                return item;
            }
        } else {
            // Queue was already empty
            bottom_.store(b + 1, std::memory_order_relaxed);
            return std::nullopt;
        }
    }

    /**
     * @brief Steals an element from the top of the queue (FIFO).
     * Safe to call concurrently by any thread.
     * @return std::optional containing the stolen item, or std::nullopt if empty or conflict.
     */
    std::optional<T> steal() {
        std::int64_t t = top_.load(std::memory_order_acquire);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        const std::int64_t b = bottom_.load(std::memory_order_acquire);

        if (t < b) {
            // Queue has at least one element
            T item = buffer_[t & mask_].item;
            if (top_.compare_exchange_strong(t, t + 1,
                                             std::memory_order_seq_cst,
                                             std::memory_order_relaxed)) {
                return item;
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] bool empty() const noexcept {
        const std::int64_t b = bottom_.load(std::memory_order_relaxed);
        const std::int64_t t = top_.load(std::memory_order_relaxed);
        return b <= t;
    }

    [[nodiscard]] std::size_t size() const noexcept {
        const std::int64_t b = bottom_.load(std::memory_order_relaxed);
        const std::int64_t t = top_.load(std::memory_order_relaxed);
        return (b > t) ? static_cast<std::size_t>(b - t) : 0;
    }

    [[nodiscard]] std::size_t capacity() const noexcept {
        return capacity_;
    }

private:
    struct Slot {
        T item{};
    };

    static std::size_t next_power_of_two(std::size_t v) noexcept {
        if (v < 2) return 2;
        --v;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v |= v >> 32;
        return v + 1;
    }

    // Cache-line aligned indices to eliminate false sharing
    alignas(64) std::atomic<std::int64_t> top_{0};
    alignas(64) std::atomic<std::int64_t> bottom_{0};

    std::size_t capacity_{InitialCapacity};
    std::size_t mask_{InitialCapacity - 1};
    std::unique_ptr<Slot[]> buffer_{};
};

} // namespace scandium::core
