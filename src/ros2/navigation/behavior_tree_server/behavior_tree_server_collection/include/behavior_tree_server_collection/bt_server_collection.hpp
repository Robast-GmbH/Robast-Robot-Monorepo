#ifndef BEHAVIOR_TREE_SERVER__BT_SERVER_COLLECTION_HPP_
#define BEHAVIOR_TREE_SERVER__BT_SERVER_COLLECTION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "communication_interfaces/action/change_footprint.hpp"
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

    // Our action server
    std::unique_ptr<ChangeFootprintActionServer> action_server_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    bool stop_on_failure_;
    ActionStatus current_goal_status_;
    int loop_rate_;
    std::vector<int> failed_ids_;

   private:
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr create_set_parameters_client(
      std::string service_name);

    bool send_parameter_set_service_request(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
                                            std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request);

    /**
     * @brief Action server callbacks
     */
    void change_footprint();
  };

}   // namespace behavior_tree_server

#endif   // BEHAVIOR_TREE_SERVER__BT_SERVER_COLLECTION_HPP_