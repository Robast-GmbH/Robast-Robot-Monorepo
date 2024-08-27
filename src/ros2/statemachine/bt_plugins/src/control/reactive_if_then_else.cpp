#include "bt_plugins/control/reactive_if_then_else.hpp"

namespace statemachine
{
  ReactiveIfThenElseNode::ReactiveIfThenElseNode(const std::string &name)
      : ControlNode::ControlNode(name, {})
  {
  }

  void ReactiveIfThenElseNode::halt()
  {
    ControlNode::halt();
  }

  BT::NodeStatus ReactiveIfThenElseNode::tick()
  {
    const size_t children_count = children_nodes_.size();

    if (children_count != 2 && children_count != 3)
    {
      throw std::logic_error("ReactiveIfThenElseNode must have either 2 or 3 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    uint8_t child_idx = 0;
    BT::NodeStatus condition_status = children_nodes_[0]->executeTick();

    if (condition_status == BT::NodeStatus::RUNNING)
    {
      return condition_status;
    }
    else if (condition_status == BT::NodeStatus::SUCCESS)
    {
      child_idx = 1;
    }
    else if (condition_status == BT::NodeStatus::FAILURE)
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
      BT::NodeStatus status = children_nodes_[child_idx]->executeTick();

      if (status == BT::NodeStatus::RUNNING)
      {
        return BT::NodeStatus::RUNNING;
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
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<statemachine::ReactiveIfThenElseNode>("ReactiveIfThenElse");
}