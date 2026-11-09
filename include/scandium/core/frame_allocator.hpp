#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <utility>
#include <vector>

namespace scandium::core {

/**
 * @brief High-performance linear bump frame allocator.
 *
 * Provides O(1) allocation with arbitrary alignment (16, 32, 64 bytes).
 * Supports chained pages to preserve pointer stability if allocation exceeds
 * the primary page, while retaining buffers across reset() calls for zero-malloc
 * subsequent frames.
 */
class FrameAllocator {
public:
    static constexpr std::size_t DefaultPageSize = 1024 * 1024; // 1 MB default page
    static constexpr std::size_t DefaultAlignment = alignof(std::max_align_t);

    struct Marker {
        std::size_t page_index{0};
        std::size_t offset{0};
    };

    explicit FrameAllocator(std::size_t page_size = DefaultPageSize)
        : page_size_(page_size > 0 ? page_size : DefaultPageSize) {
        allocate_new_page(page_size_);
    }

    ~FrameAllocator() {
        release_all();
    }

    FrameAllocator(const FrameAllocator&) = delete;
    FrameAllocator& operator=(const FrameAllocator&) = delete;

    FrameAllocator(FrameAllocator&& other) noexcept
        : page_size_(other.page_size_)
        , current_page_(other.current_page_)
        , current_offset_(other.current_offset_)
        , total_allocated_(other.total_allocated_)
        , pages_(std::move(other.pages_)) {
        other.current_page_ = 0;
        other.current_offset_ = 0;
        other.total_allocated_ = 0;
    }

    FrameAllocator& operator=(FrameAllocator&& other) noexcept {
        if (this != &other) {
            release_all();
            page_size_ = other.page_size_;
            current_page_ = other.current_page_;
            current_offset_ = other.current_offset_;
            total_allocated_ = other.total_allocated_;
            pages_ = std::move(other.pages_);

            other.current_page_ = 0;
            other.current_offset_ = 0;
            other.total_allocated_ = 0;
        }
        return *this;
    }

    /**
     * @brief Allocates a contiguous memory block with specified alignment.
     * @param bytes Number of bytes to allocate.
     * @param alignment Power-of-two alignment boundary.
     * @return Aligned pointer to allocated memory block.
     */
    [[nodiscard]] void* allocate(std::size_t bytes, std::size_t alignment = DefaultAlignment) {
        if (bytes == 0) {
            return nullptr;
        }

        assert((alignment & (alignment - 1)) == 0 && "Alignment must be a power of two");
        alignment = std::max(alignment, alignof(void*));

        auto& page = pages_[current_page_];
        uintptr_t base = reinterpret_cast<uintptr_t>(page.data);
        uintptr_t current_addr = base + current_offset_;
        uintptr_t aligned_addr = (current_addr + alignment - 1) & ~(static_cast<uintptr_t>(alignment - 1));
        std::size_t new_offset = static_cast<std::size_t>(aligned_addr - base) + bytes;

        if (new_offset <= page.capacity) {
            current_offset_ = new_offset;
            total_allocated_ += bytes;
            return reinterpret_cast<void*>(aligned_addr);
        }

        // Need next page. If allocation is larger than standard page_size_, allocate oversized page
        std::size_t next_page_capacity = std::max(page_size_, bytes + alignment);

        // Check if we already have an allocated dormant page from a previous frame
        if (current_page_ + 1 < pages_.size()) {
            ++current_page_;
            if (pages_[current_page_].capacity < next_page_capacity) {
                // Resize dormant page if it cannot fit this request
                std::free(pages_[current_page_].data);
                pages_[current_page_].data = static_cast<std::byte*>(std::malloc(next_page_capacity));
                pages_[current_page_].capacity = next_page_capacity;
            }
        } else {
            allocate_new_page(next_page_capacity);
            ++current_page_;
        }

        current_offset_ = 0;
        auto& new_page = pages_[current_page_];
        base = reinterpret_cast<uintptr_t>(new_page.data);
        aligned_addr = (base + alignment - 1) & ~(static_cast<uintptr_t>(alignment - 1));
        new_offset = static_cast<std::size_t>(aligned_addr - base) + bytes;

        current_offset_ = new_offset;
        total_allocated_ += bytes;
        return reinterpret_cast<void*>(aligned_addr);
    }

    /**
     * @brief Constructs an object of type T in arena-allocated memory.
     */
    template <typename T, typename... Args>
    [[nodiscard]] T* create(Args&&... args) {
        void* mem = allocate(sizeof(T), alignof(T));
        return ::new (mem) T(std::forward<Args>(args)...);
    }

    /**
     * @brief Allocates an uninitialized array of elements.
     */
    template <typename T>
    [[nodiscard]] T* allocate_array(std::size_t count) {
        return static_cast<T*>(allocate(sizeof(T) * count, alignof(T)));
    }

    /**
     * @brief Captures current allocation marker for scoped scratch rollback.
     */
    [[nodiscard]] Marker mark() const noexcept {
        return Marker{current_page_, current_offset_};
    }

    /**
     * @brief Rolls back allocation state to the captured marker.
     */
    void reset_to(Marker marker) noexcept {
        assert(marker.page_index <= current_page_ && "Invalid marker rollback");
        current_page_ = marker.page_index;
        current_offset_ = marker.offset;
    }

    /**
     * @brief Resets the allocator back to zero offset.
     * Retains all allocated page memory for zero-malloc subsequent ticks.
     */
    void reset() noexcept {
        current_page_ = 0;
        current_offset_ = 0;
        total_allocated_ = 0;
    }

    [[nodiscard]] std::size_t total_allocated() const noexcept {
        return total_allocated_;
    }

    [[nodiscard]] std::size_t page_count() const noexcept {
        return pages_.size();
    }

    [[nodiscard]] std::size_t page_size() const noexcept {
        return page_size_;
    }

private:
    struct Page {
        std::byte* data{nullptr};
        std::size_t capacity{0};
    };

    void allocate_new_page(std::size_t capacity) {
        std::byte* ptr = static_cast<std::byte*>(std::malloc(capacity));
        if (!ptr) {
            throw std::bad_alloc();
        }
        pages_.push_back(Page{ptr, capacity});
    }

    void release_all() noexcept {
        for (auto& page : pages_) {
            if (page.data) {
                std::free(page.data);
                page.data = nullptr;
                page.capacity = 0;
            }
        }
        pages_.clear();
        current_page_ = 0;
        current_offset_ = 0;
        total_allocated_ = 0;
    }

    std::size_t page_size_{DefaultPageSize};
    std::size_t current_page_{0};
    std::size_t current_offset_{0};
    std::size_t total_allocated_{0};
    std::vector<Page> pages_{};
};

/**
 * @brief STL-compatible allocator backed by FrameAllocator.
 */
template <typename T>
class ArenaAllocator {
public:
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template <typename U>
    struct rebind {
        using other = ArenaAllocator<U>;
    };

    explicit ArenaAllocator(FrameAllocator& arena) noexcept
        : arena_(&arena) {}

    template <typename U>
    ArenaAllocator(const ArenaAllocator<U>& other) noexcept
        : arena_(other.arena_) {}

    [[nodiscard]] T* allocate(std::size_t n) {
        if (!arena_) {
            throw std::bad_alloc();
        }
        void* ptr = arena_->allocate(n * sizeof(T), alignof(T));
        return static_cast<T*>(ptr);
    }

    void deallocate(T* /*ptr*/, std::size_t /*n*/) noexcept {
        // Linear bump allocator does not reclaim individual items.
    }

    [[nodiscard]] FrameAllocator* arena() const noexcept {
        return arena_;
    }

    template <typename U>
    bool operator==(const ArenaAllocator<U>& other) const noexcept {
        return arena_ == other.arena_;
    }

    template <typename U>
    bool operator!=(const ArenaAllocator<U>& other) const noexcept {
        return arena_ != other.arena_;
    }

private:
    template <typename U>
    friend class ArenaAllocator;

    FrameAllocator* arena_{nullptr};
};

/**
 * @brief Scoped RAII guard that restores a FrameAllocator to its marker upon destruction.
 */
class ScopedArenaMarker {
public:
    explicit ScopedArenaMarker(FrameAllocator& arena) noexcept
        : arena_(arena)
        , marker_(arena.mark()) {}

    ~ScopedArenaMarker() {
        arena_.reset_to(marker_);
    }

    ScopedArenaMarker(const ScopedArenaMarker&) = delete;
    ScopedArenaMarker& operator=(const ScopedArenaMarker&) = delete;

private:
    FrameAllocator& arena_;
    FrameAllocator::Marker marker_;
};

} // namespace scandium::core
