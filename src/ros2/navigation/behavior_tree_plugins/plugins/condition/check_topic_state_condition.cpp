// Copyright (c) 22021 Jacob Ritterbach
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_tree_plugins/plugins/condition/check_topic_state_condition.hpp"

namespace nav2_behavior_tree
{

  CheckTopicStateCondition::CheckTopicStateCondition(const std::string& condition_name,
                                                     const BT::NodeConfiguration& conf)
      : BT::ConditionNode(condition_name, conf)
  {
    _node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    _callback_group = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _callback_group_executor.add_callback_group(_callback_group, _node->get_node_base_interface());

    _received_state = false;

    getInput("topic", _topic_name);
    getInput("target_state", _target_state);

    if (_topic_name == "")
    {
      _topic_name = "/robot/safety_module/safety_stop";
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = _callback_group;
    _subscriber = _node->create_subscription<std_msgs::msg::Bool>(
      _topic_name, qos, std::bind(&CheckTopicStateCondition::topic_callback, this, std::placeholders::_1), sub_option);
  }

  BT::NodeStatus CheckTopicStateCondition::tick()
  {
    _callback_group_executor.spin_some();

    if (_received_state)
    {
      _received_state = false;
      return (_current_state == _target_state) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  void CheckTopicStateCondition::topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    _current_state = msg->data;
    _received_state = true;
  }

}   // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CheckTopicStateCondition>("TopicStateCheck");
}