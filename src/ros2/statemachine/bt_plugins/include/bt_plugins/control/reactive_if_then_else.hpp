#ifndef REACTIVE_IF_THEN_ELSE_HPP
#define REACTIVE_IF_THEN_ELSE_HPP

#include "behaviortree_cpp/control_node.h"

namespace statemachine
{
  /**
   * @brief ReactiveIfThenElseNode must have exactly 2 or 3 children. This node is reactive.
   *
   * The first child is the "statement" of the if.
   *
   * If that return SUCCESS, then the second child is executed.
   *
   * Instead, if it returned FAILURE, the third child is executed.
   *
   * If you have only 2 children, this node will return FAILURE whenever the
   * statement returns FAILURE.
   *
   * This is equivalent to add AlwaysFailure as 3rd child.
   *
   */
  class ReactiveIfThenElseNode : public BT::ControlNode
  {
  public:
    ReactiveIfThenElseNode(const std::string &name);

    virtual ~ReactiveIfThenElseNode() override = default;

    virtual void halt() override;

  private:
    virtual BT::NodeStatus tick() override;
  };

} // namespace statemachine
#endif // !REACTIVE_IF_THEN_ELSE_HPP
