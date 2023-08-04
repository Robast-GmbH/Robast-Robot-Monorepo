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

  void BtServerCollection::change_footprint()
  {
    auto goal = action_server_->get_current_goal();
    auto feedback = std::make_shared<ChangeFootprintAction::Feedback>();
    auto result = std::make_shared<ChangeFootprintAction::Result>();

    // TODO@Jacob: Implement change of footprint
  }

}   // namespace behavior_tree_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_tree_server::BtServerCollection)