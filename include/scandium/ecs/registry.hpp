#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "scandium/ecs/entity.hpp"
#include "scandium/math/vec3.hpp"

namespace scandium::ecs {

struct TransformComponent {
    math::Vec3f position{};
    math::Vec3f velocity{};
};

class ISparseSet {
public:
    virtual ~ISparseSet() = default;
    virtual void remove(EntityId id) noexcept = 0;
    virtual bool contains(EntityId id) const noexcept = 0;
    virtual void clear() noexcept = 0;
};

template <typename Component>
class ComponentPool : public ISparseSet {
public:
    static constexpr std::size_t kInvalidIndex = static_cast<std::size_t>(-1);

    Component& emplace(EntityId id, Component component) {
        if (id >= sparse_.size()) {
            sparse_.resize(id + 1, kInvalidIndex);
        }

        if (sparse_[id] != kInvalidIndex) {
            dense_[sparse_[id]] = std::move(component);
            return dense_[sparse_[id]];
        }

        const std::size_t dense_idx = dense_.size();
        sparse_[id] = dense_idx;
        dense_entities_.push_back(id);
        dense_.push_back(std::move(component));
        return dense_.back();
    }

    Component* get(EntityId id) noexcept {
        if (id < sparse_.size() && sparse_[id] != kInvalidIndex) {
            return &dense_[sparse_[id]];
        }
        return nullptr;
    }

    const Component* get(EntityId id) const noexcept {
        if (id < sparse_.size() && sparse_[id] != kInvalidIndex) {
            return &dense_[sparse_[id]];
        }
        return nullptr;
    }

    [[nodiscard]] bool contains(EntityId id) const noexcept override {
        return id < sparse_.size() && sparse_[id] != kInvalidIndex;
    }

    void remove(EntityId id) noexcept override {
        if (!contains(id)) {
            return;
        }

        const std::size_t removed_dense_idx = sparse_[id];
        const std::size_t last_dense_idx = dense_.size() - 1;

        if (removed_dense_idx != last_dense_idx) {
            const EntityId last_entity = dense_entities_[last_dense_idx];
            dense_[removed_dense_idx] = std::move(dense_[last_dense_idx]);
            dense_entities_[removed_dense_idx] = last_entity;
            sparse_[last_entity] = removed_dense_idx;
        }

        dense_.pop_back();
        dense_entities_.pop_back();
        sparse_[id] = kInvalidIndex;
    }

    void clear() noexcept override {
        dense_.clear();
        dense_entities_.clear();
        std::fill(sparse_.begin(), sparse_.end(), kInvalidIndex);
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return dense_.size();
    }

    [[nodiscard]] const std::vector<Component>& dense() const noexcept {
        return dense_;
    }

    [[nodiscard]] std::vector<Component>& dense_mutable() noexcept {
        return dense_;
    }

    [[nodiscard]] const std::vector<EntityId>& entities() const noexcept {
        return dense_entities_;
    }

private:
    std::vector<Component> dense_;
    std::vector<EntityId> dense_entities_;
    std::vector<std::size_t> sparse_;
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
        entities_.push_back({id, 1, true});
        transforms_.push_back({});
        ++active_count_;
        return id;
    }

    Entity create_entity() {
        const EntityId id = create();
        return entities_[id];
    }

    void destroy(EntityId id) noexcept {
        if (id < entities_.size() && entities_[id].active) {
            entities_[id].active = false;
            ++entities_[id].generation;
            transforms_[id] = {};

            for (auto& [type, pool] : component_pools_) {
                pool->remove(id);
            }

            free_.push_back(id);
            --active_count_;
        }
    }

    void destroy(Entity entity) noexcept {
        if (alive(entity)) {
            destroy(entity.id);
        }
    }

    [[nodiscard]] bool alive(EntityId id) const noexcept {
        return id < entities_.size() && entities_[id].active;
    }

    [[nodiscard]] bool alive(Entity entity) const noexcept {
        return entity.id < entities_.size() &&
               entities_[entity.id].active &&
               entities_[entity.id].generation == entity.generation;
    }

    [[nodiscard]] std::uint32_t generation(EntityId id) const noexcept {
        return id < entities_.size() ? entities_[id].generation : 0;
    }

    TransformComponent& transform(EntityId id) {
        return transforms_.at(id);
    }

    const TransformComponent& transform(EntityId id) const {
        return transforms_.at(id);
    }

    [[nodiscard]] const std::vector<Entity>& entities() const noexcept {
        return entities_;
    }

    [[nodiscard]] std::size_t active_count() const noexcept {
        return active_count_;
    }

    template <typename Component>
    Component& emplace(EntityId id, Component component) {
        assert(alive(id) && "Cannot attach component to inactive entity");
        return get_or_create_pool<Component>().emplace(id, std::move(component));
    }

    template <typename Component>
    Component& assign(EntityId id, Component component) {
        return emplace<Component>(id, std::move(component));
    }

    template <typename Component>
    [[nodiscard]] Component* get(EntityId id) noexcept {
        if (!alive(id)) {
            return nullptr;
        }
        auto* pool = get_pool<Component>();
        return pool ? pool->get(id) : nullptr;
    }

    template <typename Component>
    [[nodiscard]] const Component* get(EntityId id) const noexcept {
        if (!alive(id)) {
            return nullptr;
        }
        const auto* pool = get_pool<Component>();
        return pool ? pool->get(id) : nullptr;
    }

    template <typename Component>
    [[nodiscard]] bool has(EntityId id) const noexcept {
        if (!alive(id)) {
            return false;
        }
        const auto* pool = get_pool<Component>();
        return pool && pool->contains(id);
    }

    template <typename Component>
    void remove(EntityId id) noexcept {
        auto* pool = get_pool<Component>();
        if (pool) {
            pool->remove(id);
        }
    }

    template <typename Component, typename Fn>
    void each(Fn&& fn) {
        auto* pool = get_pool<Component>();
        if (!pool) {
            return;
        }
        auto& dense_comps = pool->dense_mutable();
        const auto& entities = pool->entities();
        for (std::size_t i = 0; i < dense_comps.size(); ++i) {
            fn(entities[i], dense_comps[i]);
        }
    }

    template <typename Primary, typename... Secondary, typename Fn>
    void view(Fn&& fn) {
        auto* primary_pool = get_pool<Primary>();
        if (!primary_pool) {
            return;
        }

        auto& dense_comps = primary_pool->dense_mutable();
        const auto& entities = primary_pool->entities();

        for (std::size_t i = 0; i < dense_comps.size(); ++i) {
            const EntityId id = entities[i];
            if ((has<Secondary>(id) && ...)) {
                fn(id, dense_comps[i], (*get<Secondary>(id))...);
            }
        }
    }

    void clear() noexcept {
        for (auto& [type, pool] : component_pools_) {
            pool->clear();
        }
        entities_.clear();
        transforms_.clear();
        free_.clear();
        active_count_ = 0;
    }

private:
    template <typename Component>
    ComponentPool<Component>& get_or_create_pool() {
        const auto type = std::type_index(typeid(Component));
        auto it = component_pools_.find(type);
        if (it == component_pools_.end()) {
            auto pool = std::make_unique<ComponentPool<Component>>();
            auto* raw = pool.get();
            component_pools_.emplace(type, std::move(pool));
            return *raw;
        }
        return *static_cast<ComponentPool<Component>*>(it->second.get());
    }

    template <typename Component>
    ComponentPool<Component>* get_pool() noexcept {
        const auto type = std::type_index(typeid(Component));
        auto it = component_pools_.find(type);
        if (it == component_pools_.end()) {
            return nullptr;
        }
        return static_cast<ComponentPool<Component>*>(it->second.get());
    }

    template <typename Component>
    const ComponentPool<Component>* get_pool() const noexcept {
        const auto type = std::type_index(typeid(Component));
        auto it = component_pools_.find(type);
        if (it == component_pools_.end()) {
            return nullptr;
        }
        return static_cast<const ComponentPool<Component>*>(it->second.get());
    }

    std::vector<Entity> entities_;
    std::vector<TransformComponent> transforms_;
    std::vector<EntityId> free_;
    std::size_t active_count_{0};
    std::unordered_map<std::type_index, std::unique_ptr<ISparseSet>> component_pools_;
};

} // namespace scandium::ecs
