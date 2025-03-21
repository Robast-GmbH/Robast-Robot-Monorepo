#include "behavior_tree_server_collection/parameter_service_client.hpp"

namespace behavior_tree_server
{
  ParameterServiceClient::ParameterServiceClient(std::shared_ptr<nav2_util::LifecycleNode> node)
  {
    _set_local_costmap_parameters_client =
      create_set_parameters_client(node, "/local_costmap/local_costmap/set_parameters");

    _set_global_costmap_parameters_client =
      create_set_parameters_client(node, "/global_costmap/global_costmap/set_parameters");

    _get_local_costmap_parameters_client =
      create_get_parameters_client(node, "/local_costmap/local_costmap/get_parameters");

    _get_global_costmap_parameters_client =
      create_get_parameters_client(node, "/global_costmap/global_costmap/get_parameters");
  }

  bool ParameterServiceClient::set_parameter_for_local_costmap(const rcl_interfaces::msg::Parameter parameter)
  {
    std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(parameter);

    return send_parameter_set_service_request(_set_local_costmap_parameters_client, request);
  }

  bool ParameterServiceClient::set_parameter_for_global_costmap(const rcl_interfaces::msg::Parameter parameter)
  {
    std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(parameter);

    return send_parameter_set_service_request(_set_global_costmap_parameters_client, request);
  }

  std::optional<rcl_interfaces::msg::ParameterValue> ParameterServiceClient::get_parameter_value_for_local_costmap(
    std::string parameter_name)
  {
    std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request =
      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(parameter_name);

    return get_service_parameter_request(_get_local_costmap_parameters_client, request);
  }

  std::optional<rcl_interfaces::msg::ParameterValue> ParameterServiceClient::get_parameter_value_for_global_costmap(
    std::string parameter_name)
  {
    std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request =
      std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(parameter_name);

    return get_service_parameter_request(_get_global_costmap_parameters_client, request);
  }

  bool ParameterServiceClient::send_parameter_set_service_request(
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
    const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request)
  {
    auto response = client->async_send_request(request);

    if (response.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // TODO@Jacob: Use generics for this to make one function out of this function and send_parameter_set_service_request
  std::optional<rcl_interfaces::msg::ParameterValue> ParameterServiceClient::get_service_parameter_request(
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client,
    const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request)
  {
    auto response = client->async_send_request(request);

    if (response.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
      return response.get()->values[0];
    }
    else
    {
      return std::nullopt;
    }
  }

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr ParameterServiceClient::create_set_parameters_client(
    std::shared_ptr<nav2_util::LifecycleNode> node, const std::string service_name)
  {
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client =
      node->create_client<rcl_interfaces::srv::SetParameters>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(node->get_logger(), "Service %s not available...", service_name.c_str());
    }
    return client;
  }

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr ParameterServiceClient::create_get_parameters_client(
    std::shared_ptr<nav2_util::LifecycleNode> node, const std::string service_name)
  {
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client =
      node->create_client<rcl_interfaces::srv::GetParameters>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(node->get_logger(), "Service %s not available...", service_name.c_str());
    }
    return client;
  }

}   // namespace behavior_tree_server
