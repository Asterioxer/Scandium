#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace scandium::core {

template <typename T>
class ObjectPool {
public:
    explicit ObjectPool(std::size_t capacity) {
        items_.resize(capacity);
        active_.resize(capacity, false);
        free_.reserve(capacity);

        // Populate free indices in reverse order so index 0 is acquired first
        for (std::size_t i = capacity; i > 0; --i) {
            free_.push_back(i - 1);
        }
    }

    T* acquire() {
        if (free_.empty()) {
            return nullptr;
        }

        const auto index = free_.back();
        free_.pop_back();
        active_[index] = true;
        return &items_[index];
    }

    void release(T* object) {
        if (object == nullptr || !owns(object)) {
            return;
        }

        const auto index = static_cast<std::size_t>(object - items_.data());
        if (!active_[index]) {
            return;
        }

        active_[index] = false;
        free_.push_back(index);
    }

    [[nodiscard]] std::size_t capacity() const noexcept {
        return items_.size();
    }

    [[nodiscard]] std::size_t available() const noexcept {
        return free_.size();
    }

    [[nodiscard]] bool owns(const T* object) const noexcept {
        if (object == nullptr || items_.empty()) {
            return false;
        }
        return object >= items_.data() && object < (items_.data() + items_.size());
    }

private:
    std::vector<T> items_;
    std::vector<std::size_t> free_;
    std::vector<bool> active_;
};

} // namespace scandium::core
