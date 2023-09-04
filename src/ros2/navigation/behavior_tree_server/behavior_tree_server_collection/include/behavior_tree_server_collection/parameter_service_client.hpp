#ifndef BEHAVIOR_TREE_SERVER__PARAMETER_SERVICE_CLIENT_HPP_
#define BEHAVIOR_TREE_SERVER__PARAMETER_SERVICE_CLIENT_HPP_

#include <optional>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace behavior_tree_server
{

  class ParameterServiceClient
  {
   public:
    ParameterServiceClient(std::shared_ptr<nav2_util::LifecycleNode> node);
    ~ParameterServiceClient() = default;

    bool set_parameter_for_local_costmap(const rcl_interfaces::msg::Parameter parameter);

    bool set_parameter_for_global_costmap(const rcl_interfaces::msg::Parameter parameter);

    std::optional<rcl_interfaces::msg::ParameterValue> get_parameter_value_for_local_costmap(
      std::string parameter_name);

    std::optional<rcl_interfaces::msg::ParameterValue> get_parameter_value_for_global_costmap(
      std::string parameter_name);

   private:
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr _set_local_costmap_parameters_client;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr _set_global_costmap_parameters_client;

    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr _get_local_costmap_parameters_client;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr _get_global_costmap_parameters_client;

    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr create_set_parameters_client(
      std::shared_ptr<nav2_util::LifecycleNode> node, const std::string service_name);

    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr create_get_parameters_client(
      std::shared_ptr<nav2_util::LifecycleNode> node, const std::string service_name);

    bool send_parameter_set_service_request(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
                                            const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request);

    std::optional<rcl_interfaces::msg::ParameterValue> get_service_parameter_request(
      rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client,
      const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request);
  };

}   // namespace behavior_tree_server

#endif   // BEHAVIOR_TREE_SERVER__PARAMETER_SERVICE_CLIENT_HPP_