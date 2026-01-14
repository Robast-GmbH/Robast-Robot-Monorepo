#ifndef NAV2_BEHAVIOR_TREE__CHECK_PERSON_IN_FRONT_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__CHECK_PERSON_IN_FRONT_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

  class CheckPersonInFrontCondition : public BT::ConditionNode
  {
   public:
    CheckPersonInFrontCondition(const std::string& name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<std::string>("topic"), BT::InputPort<float>("detection_range")};
    }
    BT::NodeStatus tick() override;

   private:
    void detection_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _callback_group;
    rclcpp::executors::SingleThreadedExecutor _callback_group_executor;
    geometry_msgs::msg::PointStamped _detection_data;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _detection_subscriber;
  };

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__CHECK_PERSON_IN_FRONT_CONDITION_HPP_
