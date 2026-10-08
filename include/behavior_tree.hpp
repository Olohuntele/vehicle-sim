#pragma once

#include <vector>
#include <memory>
#include <string>

namespace vehicle_sim {

enum class NodeStatus {
    SUCCESS,
    FAILURE,
    RUNNING
};

class BTNode {
public:
    virtual ~BTNode() = default;
    virtual NodeStatus execute() = 0;
};

class CompositeNode : public BTNode {
protected:
    std::vector<std::unique_ptr<BTNode>> children_;
public:
    void addChild(std::unique_ptr<BTNode> child) {
        children_.push_back(std::move(child));
    }
};

class Selector : public CompositeNode {
public:
    NodeStatus execute() override {
        for (const auto& child : children_) {
            NodeStatus status = child->execute();
            if (status == NodeStatus::SUCCESS || status == NodeStatus::RUNNING) {
                return status;
            }
        }
        return NodeStatus::FAILURE;
    }
};

class Sequence : public CompositeNode {
public:
    NodeStatus execute() override {
        for (const auto& child : children_) {
            NodeStatus status = child->execute();
            if (status != NodeStatus::SUCCESS) {
                return status;
            }
        }
        return NodeStatus::SUCCESS;
    }
};

} // namespace vehicle_sim
