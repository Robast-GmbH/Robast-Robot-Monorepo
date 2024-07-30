#include "behaviortree_cpp/controls/if_then_else_node.h"

namespace BT
{
  IfThenElseNode::IfThenElseNode(const std::string &name)
      : ControlNode::ControlNode(name, {})
  {
    setRegistrationID("ReactiveIfThenElse");
  }

  void IfThenElseNode::halt()
  {
    ControlNode::halt();
  }

  NodeStatus IfThenElseNode::tick()
  {
    const size_t children_count = children_nodes_.size();

    if (children_count != 2 && children_count != 3)
    {
      throw std::logic_error("IfThenElseNode must have either 2 or 3 children");
    }

    setStatus(NodeStatus::RUNNING);

    uint8_t child_idx = 0;
    NodeStatus condition_status = children_nodes_[0]->executeTick();

    if (condition_status == NodeStatus::RUNNING)
    {
      return condition_status;
    }
    else if (condition_status == NodeStatus::SUCCESS)
    {
      child_idx = 1;
    }
    else if (condition_status == NodeStatus::FAILURE)
    {
      if (children_count == 3)
      {
        child_idx = 2;
      }
      else
      {
        return condition_status;
      }
    }

    // not an else
    if (child_idx > 0)
    {
      NodeStatus status = children_nodes_[child_idx]->executeTick();
      if (status == NodeStatus::RUNNING)
      {
        return NodeStatus::RUNNING;
      }
      else
      {
        resetChildren();
        child_idx = 0;
        return status;
      }
    }

    throw std::logic_error("Something unexpected happened in ReactiveIfThenElseNode");
  }

} // namespace statemachine
