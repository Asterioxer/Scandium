#pragma once

#include <atomic>
#include <cstddef>

namespace scandium::profiling {

struct AllocationStats {
    std::atomic<std::size_t> allocations{0};
    std::atomic<std::size_t> deallocations{0};
    std::atomic<std::size_t> bytes_allocated{0};
    std::atomic<std::size_t> bytes_deallocated{0};

    void record_allocation(std::size_t bytes) noexcept {
        allocations.fetch_add(1, std::memory_order_relaxed);
        bytes_allocated.fetch_add(bytes, std::memory_order_relaxed);
    }

    void record_deallocation(std::size_t bytes) noexcept {
        deallocations.fetch_add(1, std::memory_order_relaxed);
        bytes_deallocated.fetch_add(bytes, std::memory_order_relaxed);
    }

    [[nodiscard]] std::size_t live_bytes() const noexcept {
        return bytes_allocated.load(std::memory_order_relaxed) -
               bytes_deallocated.load(std::memory_order_relaxed);
    }
};

} // namespace scandium::profiling
