#pragma once

#include <cstddef>
#include <memory>
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
        for (std::size_t i = 0; i < capacity; ++i) {
            storage_.push_back(std::make_unique<T>());
            free_.push_back(i);
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

        for (std::size_t i = 0; i < storage_.size(); ++i) {
            if (storage_[i].get() == object && active_[i]) {
                active_[i] = false;
                free_.push_back(i);
                return;
            }
        }
    }

    [[nodiscard]] std::size_t capacity() const noexcept {
        return storage_.size();
    }

    [[nodiscard]] std::size_t available() const noexcept {
        return free_.size();
    }

private:
    std::vector<std::unique_ptr<T>> storage_;
    std::vector<std::size_t> free_;
    std::vector<bool> active_;
};

} // namespace scandium::core
