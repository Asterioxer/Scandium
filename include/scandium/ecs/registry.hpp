#pragma once

#include <cstdint>
#include <vector>

#include "scandium/ecs/entity.hpp"
#include "scandium/math/vec3.hpp"

namespace scandium::ecs {

struct TransformComponent {
    math::Vec3f position{};
    math::Vec3f velocity{};
};

class Registry {
public:
    EntityId create() {
        if (!free_.empty()) {
            const EntityId id = free_.back();
            free_.pop_back();
            entities_[id].active = true;
            transforms_[id] = {};
            ++active_count_;
            return id;
        }

        const EntityId id = static_cast<EntityId>(entities_.size());
        entities_.push_back({id, true});
        transforms_.push_back({});
        ++active_count_;
        return id;
    }

    void destroy(EntityId id) noexcept {
        if (id < entities_.size() && entities_[id].active) {
            entities_[id].active = false;
            transforms_[id] = {};
            free_.push_back(id);
            --active_count_;
        }
    }

    [[nodiscard]] bool alive(EntityId id) const noexcept {
        return id < entities_.size() && entities_[id].active;
    }

    TransformComponent& transform(EntityId id) {
        return transforms_.at(id);
    }

    [[nodiscard]] const std::vector<Entity>& entities() const noexcept {
        return entities_;
    }

    [[nodiscard]] std::size_t active_count() const noexcept {
        return active_count_;
    }

private:
    std::vector<Entity> entities_;
    std::vector<TransformComponent> transforms_;
    std::vector<EntityId> free_;
    std::size_t active_count_{};
};

} // namespace scandium::ecs
