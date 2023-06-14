
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace drawer_statemachine
{
  template <typename T>
  class BaseCompareCondition : public BT::ConditionNode
  {

  public:
    BaseCompareCondition(const std::string &name, const BT::NodeConfig &config) : BT::ConditionNode(name, config)
    {
      _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
      _callback_group = _node->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive,
          false);
      _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

      getInput("topic", topic_name_);

      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local().reliable();

      rclcpp::SubscriptionOptions sub_option;
      sub_option.callback_group = _callback_group;
      drawer_status_sub_ = _node->create_subscription<T>(
          topic_name_,
          qos,
          std::bind(&BaseCompareCondition::callbackDrawerFeedback, this, std::placeholders::_1),
          sub_option);

      initialize_target_value();
    }

    virtual BT::NodeStatus tick()
    {
      _callback_group_executor.spin_some();

      if (comparator(last_message_, target_value_))
      {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }

    virtual void initialize_target_value() = 0;

  protected:
    T target_value_;
    T last_message_;

    virtual bool comparator(T last_message_, T target_value_) = 0;
    virtual void callbackDrawerFeedback(const typename T::SharedPtr msg) = 0;
    typename rclcpp::Subscription<T>::SharedPtr drawer_status_sub_;
    std::string topic_name_;

  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
  };
} // namespace drawer_statemachine