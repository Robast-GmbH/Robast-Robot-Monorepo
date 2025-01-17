
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace statemachine
{
  template <typename T, typename V>
  class BaseCompareCondition : public BT::ConditionNode
  {
    public:
      BaseCompareCondition(const std::string &name, const BT::NodeConfig &config, rclcpp::QoS qos)
          : BT::ConditionNode(name, config)
      {
        _node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(_callback_group, _node->get_node_base_interface());

        getInput("topic", topic_name_);

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = _callback_group;
        drawer_status_sub_ = _node->create_subscription<T>(
          topic_name_,
          qos,
          std::bind(&BaseCompareCondition::callbackTopicFeedback, this, std::placeholders::_1),
          sub_option);
        blackboard_ = config.blackboard;
      }
      // virtual static BT::PortsList providedPorts() = 0;

      virtual BT::NodeStatus tick() = 0;
      virtual void initialize_target_value() = 0;

    protected:
      V target_value_;
      T last_message_;
      const std::chrono::seconds timeout_duration_{20};

      virtual bool comparator(T last_message, V target_value) = 0;
      virtual void callbackTopicFeedback(const typename T::SharedPtr msg) = 0;
      typename rclcpp::Subscription<T>::SharedPtr drawer_status_sub_;
      std::string topic_name_;
      BT::Blackboard::Ptr blackboard_;
      rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    private:
      rclcpp::Node::SharedPtr _node;
      rclcpp::CallbackGroup::SharedPtr _callback_group;
  };
}   // namespace statemachine