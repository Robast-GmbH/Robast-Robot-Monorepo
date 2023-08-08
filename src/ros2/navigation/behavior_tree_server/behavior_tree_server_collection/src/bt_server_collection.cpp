#include "behavior_tree_server_collection/bt_server_collection.hpp"

namespace behavior_tree_server
{
  BtServerCollection::BtServerCollection(const rclcpp::NodeOptions& options)
      : nav2_util::LifecycleNode("split_path_follower", "", options)
  {
    RCLCPP_INFO(get_logger(), "Creating");
  }

  BtServerCollection::~BtServerCollection()
  {
  }

  nav2_util::CallbackReturn BtServerCollection::on_configure(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());

    action_server_ =
      std::make_unique<ChangeFootprintActionServer>(get_node_base_interface(),
                                                    get_node_clock_interface(),
                                                    get_node_logging_interface(),
                                                    get_node_waitables_interface(),
                                                    "change_footprint",
                                                    std::bind(&BtServerCollection::change_footprint, this));

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_activate(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating");

    action_server_->activate();

    // create bond connection
    createBond();   // TODO@Jacob: What is this? Do we need this?

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    action_server_->deactivate();

    // destroy bond connection
    destroyBond();   // TODO@Jacob: What is this? Do we need this?

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");

    action_server_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr BtServerCollection::create_set_parameters_client(
    std::string service_name)
  {
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client =
      shared_from_this()->create_client<rcl_interfaces::srv::SetParameters>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(get_logger(), "Service %s not available...", service_name.c_str());
    }
    return client;
  }

  bool BtServerCollection::send_parameter_set_service_request(
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
    std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request)
  {
    auto response = client->async_send_request(request);

    if (response.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
    {
      RCLCPP_INFO(get_logger(), "Parameter set successfully");
      return true;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to set parameter");
      return false;
    }
  }

  void BtServerCollection::change_footprint()
  {
    RCLCPP_INFO(get_logger(), "Change of footprint for local and global footprint requested!");

    auto goal = action_server_->get_current_goal();
    auto feedback = std::make_shared<ChangeFootprintAction::Feedback>();
    auto result = std::make_shared<ChangeFootprintAction::Result>();

    // Prepare the SetParameters service request
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "footprint";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter.value.string_value = goal->footprint;

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(parameter);

    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_local_costmap_parameters_client =
      create_set_parameters_client("/local_costmap/local_costmap/set_parameters");

    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_global_costmap_parameters_client =
      create_set_parameters_client("/global_costmap/global_costmap/set_parameters");

    bool set_parameter_succeeded = false;
    set_parameter_succeeded = send_parameter_set_service_request(set_local_costmap_parameters_client, request) &&
                              send_parameter_set_service_request(set_global_costmap_parameters_client, request);

    if (set_parameter_succeeded)
    {
      action_server_->succeeded_current(result);
    }
  }

}   // namespace behavior_tree_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_tree_server::BtServerCollection)