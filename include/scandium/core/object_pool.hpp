#pragma once

#include <cstddef>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace scandium::core {

template <typename T>
class ObjectPool {
public:
    explicit ObjectPool(std::size_t capacity) {
        active_.resize(capacity, false);
        storage_.reserve(capacity);
        free_.reserve(capacity);
        indices_.reserve(capacity);

        for (std::size_t i = 0; i < capacity; ++i) {
            storage_.push_back(std::make_unique<T>());
            free_.push_back(i);
            indices_.emplace(storage_.back().get(), i);
        }
    }

    T* acquire() {
        if (free_.empty()) {
            return nullptr;
        }

        const auto index = free_.back();
        free_.pop_back();
        active_[index] = true;
        return storage_[index].get();
    }

    void release(T* object) {
        if (object == nullptr) {
            return;
        }

        const auto it = indices_.find(object);
        if (it == indices_.end()) {
            return;
        }

        const auto index = it->second;
        if (!active_[index]) {
            return;
        }

        active_[index] = false;
        free_.push_back(index);
    }

    [[nodiscard]] std::size_t capacity() const noexcept {
        return storage_.size();
    }

    [[nodiscard]] std::size_t available() const noexcept {
        return free_.size();
    }

    [[nodiscard]] bool owns(const T* object) const noexcept {
        return object != nullptr && indices_.find(object) != indices_.end();
    }

private:
    std::vector<std::unique_ptr<T>> storage_;
    std::vector<std::size_t> free_;
    std::vector<bool> active_;
    std::unordered_map<T*, std::size_t> indices_;
};

} // namespace scandium::core
