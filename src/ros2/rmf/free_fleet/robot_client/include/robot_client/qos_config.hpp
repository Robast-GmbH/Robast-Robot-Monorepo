#ifndef ROBOT_CLIENT__QOS_CONFIG_HPP_
#define ROBOT_CLIENT__QOS_CONFIG_HPP_

#include <rclcpp/qos.hpp>

namespace rmf_robot_client
{
  class QoSConfig
  {
   public:
    static rclcpp::QoS get_robotnik_bms_qos()
    {
      rclcpp::QoS qos_robotnik = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos_robotnik.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      return qos_robotnik;
    }

    static rclcpp::QoS get_fleet_communication_qos()
    {
      rclcpp::QoS qos_fleet_communication =
          rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
      qos_fleet_communication.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      qos_fleet_communication.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      qos_fleet_communication.avoid_ros_namespace_conventions(false);
      return qos_fleet_communication;
    }

    static rclcpp::QoS get_statemaschine_qos()
    {
      rclcpp::QoS qos_statemaschine = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
      qos_statemaschine.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      qos_statemaschine.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      qos_statemaschine.avoid_ros_namespace_conventions(false);
      return qos_statemaschine;
    }
  };
}   // namespace rmf_robot_client

#endif   // ROBOT_CLIENT__QOS_CONFIG_HPP_