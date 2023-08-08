#ifndef BEHAVIOR_TREE_SERVER__BT_SERVER_COLLECTION_HPP_
#define BEHAVIOR_TREE_SERVER__BT_SERVER_COLLECTION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "communication_interfaces/action/change_footprint.hpp"
#include "communication_interfaces/action/change_footprint_padding.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace behavior_tree_server
{

  enum class ActionStatus
  {
    UNKNOWN = 0,
    PROCESSING = 1,
    FAILED = 2,
    SUCCEEDED = 3
  };

  /**
   * @class split_path_follower::BtServerCollection
   * @brief An action server that uses behavior tree for navigating a robot to its
   * goal position.
   */
  class BtServerCollection : public nav2_util::LifecycleNode
  {
   public:
    using ChangeFootprintAction = communication_interfaces::action::ChangeFootprint;
    using ChangeFootprintActionServer = nav2_util::SimpleActionServer<ChangeFootprintAction>;
    using ChangeFootprintPaddingAction = communication_interfaces::action::ChangeFootprintPadding;
    using ChangeFootprintPaddingActionServer = nav2_util::SimpleActionServer<ChangeFootprintPaddingAction>;

    /**
     * @brief A constructor for split_path_follower::BtServerCollection class
     * @param options Additional options to control creation of the node.
     */
    explicit BtServerCollection(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /**
     * @brief A destructor for split_path_follower::BtServerCollection class
     */
    ~BtServerCollection();

   protected:
    /**
     * @brief Configures member variables
     *
     * Initializes action server for "follow_waypoints"
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    /**
     * @brief Activates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    /**
     * @brief Deactivates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    /**
     * @brief Resets member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    /**
     * @brief Called when in shutdown state
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

   private:
    std::unique_ptr<ChangeFootprintActionServer> _action_server_change_footprint;
    std::unique_ptr<ChangeFootprintPaddingActionServer> _action_server_change_footprint_padding;

    bool set_parameter_for_local_and_global_costmap(const rcl_interfaces::msg::Parameter parameter);

    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr create_set_parameters_client(
      const std::string service_name);

    bool send_parameter_set_service_request(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
                                            const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request);

    void change_footprint();

    void change_footprint_padding();
  };

}   // namespace behavior_tree_server

#endif   // BEHAVIOR_TREE_SERVER__BT_SERVER_COLLECTION_HPP_