#ifndef DRAWER_BRIDGE__QOS_CONFIG_HPP_
#define DRAWER_BRIDGE__QOS_CONFIG_HPP_

#include <rclcpp/qos.hpp>

namespace drawer_bridge
{
  class QoSConfig
  {
   public:
    rclcpp::QoS get_qos_open_drawer()
    {
      rclcpp::QoS qos_open_drawer = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos_open_drawer.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      qos_open_drawer.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      qos_open_drawer.avoid_ros_namespace_conventions(false);

      return qos_open_drawer;
    };

    rclcpp::QoS get_qos_drawer_leds()
    {
      rclcpp::QoS qos_drawer_leds = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
      qos_drawer_leds.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      qos_drawer_leds.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      qos_drawer_leds.avoid_ros_namespace_conventions(false);

      return qos_drawer_leds;
    };

    rclcpp::QoS get_qos_error_msgs()
    {
      rclcpp::QoS qos_error_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
      qos_error_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      qos_error_msgs.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      qos_error_msgs.avoid_ros_namespace_conventions(false);

      return qos_error_msgs;
    };

    rclcpp::QoS get_qos_can_messages()
    {
      return rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
    };
  };
}   // namespace drawer_bridge

#endif   // DRAWER_BRIDGE__QOS_CONFIG_HPP_