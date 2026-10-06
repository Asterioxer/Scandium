#pragma once

#include <cstddef>
#include <functional>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace scandium::core {

// Lightweight synchronous event bus. Dispatch is deterministic in subscription
// order and intentionally executes on the calling thread.
class EventBus {
public:
    template <typename Event, typename Fn>
    std::size_t subscribe(Fn&& callback) {
        const auto type = std::type_index(typeid(Event));
        auto& bucket = handlers_[type];
        const std::size_t id = next_id_++;
        bucket.emplace_back(Handler{id, [callback = std::forward<Fn>(callback)](const void* value) {
            callback(*static_cast<const Event*>(value));
        }});
        return id;
    }

    template <typename Event>
    void publish(const Event& event) const {
        const auto it = handlers_.find(std::type_index(typeid(Event)));
        if (it == handlers_.end()) {
            return;
        }
        for (const auto& handler : it->second) {
            handler.callback(&event);
        }
    }

    template <typename Event>
    void unsubscribe(std::size_t subscription_id) {
        const auto it = handlers_.find(std::type_index(typeid(Event)));
        if (it == handlers_.end()) {
            return;
        }

        auto& bucket = it->second;
        bucket.erase(
            std::remove_if(bucket.begin(), bucket.end(),
                [subscription_id](const Handler& handler) {
                    return handler.id == subscription_id;
                }),
            bucket.end());
    }

private:
    struct Handler {
        std::size_t id;
        std::function<void(const void*)> callback;
    };

    std::unordered_map<std::type_index, std::vector<Handler>> handlers_;
    std::size_t next_id_{1};
};

} // namespace scandium::core
