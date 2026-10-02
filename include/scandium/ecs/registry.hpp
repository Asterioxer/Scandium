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
        const EntityId id = static_cast<EntityId>(entities_.size());
        entities_.push_back({id, true});
        transforms_.push_back({});
        return id;
    }

    void destroy(EntityId id) noexcept {
        if (id < entities_.size()) {
            entities_[id].active = false;
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

private:
    std::vector<Entity> entities_;
    std::vector<TransformComponent> transforms_;
};

} // namespace scandium::ecs
