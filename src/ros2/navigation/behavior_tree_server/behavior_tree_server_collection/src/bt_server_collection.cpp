#include "behavior_tree_server_collection/bt_server_collection.hpp"

namespace behavior_tree_server
{
  BtServerCollection::BtServerCollection(const rclcpp::NodeOptions& options)
      : nav2_util::LifecycleNode("split_path_follower", "", options)
  {
    RCLCPP_INFO(get_logger(), "Creating BtServerCollection");
  }

  BtServerCollection::~BtServerCollection()
  {
  }

  nav2_util::CallbackReturn BtServerCollection::on_configure(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring BtServerCollection");

    _action_server_change_footprint =
      std::make_unique<ChangeFootprintActionServer>(get_node_base_interface(),
                                                    get_node_clock_interface(),
                                                    get_node_logging_interface(),
                                                    get_node_waitables_interface(),
                                                    "change_footprint",
                                                    std::bind(&BtServerCollection::change_footprint, this));

    _action_server_change_footprint_padding = std::make_unique<ChangeFootprintPaddingActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "change_footprint_padding",
      std::bind(&BtServerCollection::change_footprint_padding, this));

    _parameter_service_client = std::make_unique<ParameterServiceClient>();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_activate(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating BtServerCollection");

    _action_server_change_footprint->activate();
    _action_server_change_footprint_padding->activate();

    // create bond connection
    createBond();   // TODO@Jacob: What is this? Do we need this?

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating BtServerCollection");

    _action_server_change_footprint->deactivate();
    _action_server_change_footprint_padding->deactivate();

    // destroy bond connection
    destroyBond();   // TODO@Jacob: What is this? Do we need this?

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up BtServerCollection");

    _action_server_change_footprint.reset();
    _action_server_change_footprint_padding.reset();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerCollection::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Shutting down BtServerCollection");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void BtServerCollection::change_footprint()
  {
    auto goal = _action_server_change_footprint->get_current_goal();
    auto feedback = std::make_shared<ChangeFootprintAction::Feedback>();
    auto result = std::make_shared<ChangeFootprintAction::Result>();

    RCLCPP_INFO(get_logger(),
                "Change to target footprint %s for local and global footprint requested!",
                (goal->footprint).c_str());

    // Prepare the SetParameters service request
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "footprint";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter.value.string_value = goal->footprint;

    if (_parameter_service_client->set_parameter_for_local_and_global_costmap(parameter))
    {
      RCLCPP_INFO(get_logger(), "Changing footprint parameter was successfully");
      _action_server_change_footprint->succeeded_current(result);
    }
  }

  void BtServerCollection::change_footprint_padding()
  {
    auto goal = _action_server_change_footprint_padding->get_current_goal();
    auto feedback = std::make_shared<ChangeFootprintPaddingAction::Feedback>();
    auto result = std::make_shared<ChangeFootprintPaddingAction::Result>();

    RCLCPP_INFO(get_logger(),
                "Change of footprint padding to %f for local and global footprint requested!",
                goal->footprint_padding);

    // Prepare the SetParameters service request
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = "footprint_padding";
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter.value.double_value = goal->footprint_padding;

    if (_parameter_service_client->set_parameter_for_local_and_global_costmap(parameter))
    {
      RCLCPP_INFO(get_logger(), "Changing footprint padding parameter was successfully");
      _action_server_change_footprint_padding->succeeded_current(result);
    }
  }

}   // namespace behavior_tree_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_tree_server::BtServerCollection)