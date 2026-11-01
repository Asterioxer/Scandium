#pragma once

#include <any>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "scandium/math/vec3.hpp"

namespace scandium::ai {

enum class NodeStatus : std::uint8_t {
    Success,
    Failure,
    Running
};

/**
 * High-performance blackboard for agent behavior state sharing.
 */
class Blackboard {
public:
    template <typename T>
    void set(std::string_view key, T value) {
        data_[std::string(key)] = std::move(value);
    }

    template <typename T>
    [[nodiscard]] const T* get(std::string_view key) const noexcept {
        const auto it = data_.find(std::string(key));
        if (it == data_.end()) return nullptr;
        return std::any_cast<T>(&it->second);
    }

    template <typename T>
    [[nodiscard]] T get_or(std::string_view key, T default_value) const noexcept {
        const auto* val = get<T>(key);
        return val ? *val : default_value;
    }

    [[nodiscard]] bool contains(std::string_view key) const noexcept {
        return data_.find(std::string(key)) != data_.end();
    }

    void remove(std::string_view key) {
        data_.erase(std::string(key));
    }

    void clear() noexcept {
        data_.clear();
    }

private:
    std::unordered_map<std::string, std::any> data_;
};

class BehaviorNode {
public:
    virtual ~BehaviorNode() = default;
    virtual NodeStatus tick(Blackboard& bb, float dt) = 0;
    virtual void reset() noexcept {}
};

/**
 * Executes children sequentially until one fails or returns Running.
 * Returns Success only if all children succeed (Logical AND).
 */
class SequenceNode : public BehaviorNode {
public:
    SequenceNode() = default;
    explicit SequenceNode(std::vector<std::unique_ptr<BehaviorNode>> children)
        : children_(std::move(children)) {}

    void add_child(std::unique_ptr<BehaviorNode> child) {
        children_.push_back(std::move(child));
    }

    NodeStatus tick(Blackboard& bb, float dt) override {
        for (; current_child_ < children_.size(); ++current_child_) {
            const auto status = children_[current_child_]->tick(bb, dt);
            if (status != NodeStatus::Success) {
                if (status == NodeStatus::Failure) {
                    reset();
                }
                return status;
            }
        }
        reset();
        return NodeStatus::Success;
    }

    void reset() noexcept override {
        current_child_ = 0;
        for (auto& child : children_) {
            child->reset();
        }
    }

private:
    std::vector<std::unique_ptr<BehaviorNode>> children_;
    std::size_t current_child_{0};
};

/**
 * Executes children sequentially until one succeeds or returns Running.
 * Returns Failure only if all children fail (Logical OR / Priority Selector).
 */
class SelectorNode : public BehaviorNode {
public:
    SelectorNode() = default;
    explicit SelectorNode(std::vector<std::unique_ptr<BehaviorNode>> children)
        : children_(std::move(children)) {}

    void add_child(std::unique_ptr<BehaviorNode> child) {
        children_.push_back(std::move(child));
    }

    NodeStatus tick(Blackboard& bb, float dt) override {
        for (; current_child_ < children_.size(); ++current_child_) {
            const auto status = children_[current_child_]->tick(bb, dt);
            if (status != NodeStatus::Failure) {
                if (status == NodeStatus::Success) {
                    reset();
                }
                return status;
            }
        }
        reset();
        return NodeStatus::Failure;
    }

    void reset() noexcept override {
        current_child_ = 0;
        for (auto& child : children_) {
            child->reset();
        }
    }

private:
    std::vector<std::unique_ptr<BehaviorNode>> children_;
    std::size_t current_child_{0};
};

/**
 * Decorator: inverts child result (Success becomes Failure, Failure becomes Success).
 */
class InverterNode : public BehaviorNode {
public:
    explicit InverterNode(std::unique_ptr<BehaviorNode> child)
        : child_(std::move(child)) {}

    NodeStatus tick(Blackboard& bb, float dt) override {
        if (!child_) return NodeStatus::Failure;
        const auto status = child_->tick(bb, dt);
        if (status == NodeStatus::Success) return NodeStatus::Failure;
        if (status == NodeStatus::Failure) return NodeStatus::Success;
        return NodeStatus::Running;
    }

    void reset() noexcept override {
        if (child_) child_->reset();
    }

private:
    std::unique_ptr<BehaviorNode> child_;
};

/**
 * Decorator: enforces a cooldown interval between successful child executions.
 */
class CooldownNode : public BehaviorNode {
public:
    CooldownNode(std::unique_ptr<BehaviorNode> child, float cooldown_seconds)
        : child_(std::move(child)),
          cooldown_(cooldown_seconds) {}

    NodeStatus tick(Blackboard& bb, float dt) override {
        if (time_since_last_success_ < cooldown_) {
            time_since_last_success_ += dt;
            return NodeStatus::Failure;
        }

        if (!child_) return NodeStatus::Failure;
        const auto status = child_->tick(bb, dt);
        if (status == NodeStatus::Success) {
            time_since_last_success_ = 0.0f;
        }
        return status;
    }

    void reset() noexcept override {
        time_since_last_success_ = cooldown_;
        if (child_) child_->reset();
    }

private:
    std::unique_ptr<BehaviorNode> child_;
    float cooldown_{1.0f};
    float time_since_last_success_{1000.0f};
};

/**
 * Leaf node executing an action functor returning NodeStatus.
 */
class ActionNode : public BehaviorNode {
public:
    using ActionFunc = std::function<NodeStatus(Blackboard&, float)>;

    explicit ActionNode(ActionFunc action)
        : action_(std::move(action)) {}

    NodeStatus tick(Blackboard& bb, float dt) override {
        return action_ ? action_(bb, dt) : NodeStatus::Failure;
    }

private:
    ActionFunc action_;
};

/**
 * Leaf node checking a boolean condition predicate.
 */
class ConditionNode : public BehaviorNode {
public:
    using ConditionFunc = std::function<bool(const Blackboard&)>;

    explicit ConditionNode(ConditionFunc condition)
        : condition_(std::move(condition)) {}

    NodeStatus tick(Blackboard& bb, float) override {
        if (condition_ && condition_(bb)) {
            return NodeStatus::Success;
        }
        return NodeStatus::Failure;
    }

private:
    ConditionFunc condition_;
};

/**
 * Behavior Tree Root Controller.
 */
class BehaviorTree {
public:
    BehaviorTree() = default;
    explicit BehaviorTree(std::unique_ptr<BehaviorNode> root)
        : root_(std::move(root)) {}

    void set_root(std::unique_ptr<BehaviorNode> root) {
        root_ = std::move(root);
    }

    NodeStatus tick(float dt) {
        if (!root_) return NodeStatus::Failure;
        return root_->tick(blackboard_, dt);
    }

    void reset() noexcept {
        if (root_) root_->reset();
    }

    [[nodiscard]] Blackboard& blackboard() noexcept {
        return blackboard_;
    }

    [[nodiscard]] const Blackboard& blackboard() const noexcept {
        return blackboard_;
    }

private:
    std::unique_ptr<BehaviorNode> root_;
    Blackboard blackboard_;
};

} // namespace scandium::ai
