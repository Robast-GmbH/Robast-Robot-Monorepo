#ifndef BEHAVIOR_TREE_SERVER__SPLIT_PATH_TO_GOAL_HPP_
#define BEHAVIOR_TREE_SERVER__SPLIT_PATH_TO_GOAL_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace split_path_follower
{

  enum class ActionStatus
  {
    UNKNOWN = 0,
    PROCESSING = 1,
    FAILED = 2,
    SUCCEEDED = 3
  };

  /**
   * @class split_path_follower::SplitPathFollower
   * @brief An action server that uses behavior tree for navigating a robot to its
   * goal position.
   */
  class SplitPathFollower : public nav2_util::LifecycleNode
  {
   public:
    using ActionT = nav2_msgs::action::FollowWaypoints;
    using ClientT = nav2_msgs::action::NavigateToPose;
    using ActionServer = nav2_util::SimpleActionServer<ActionT>;
    using ActionClient = rclcpp_action::Client<ClientT>;

    /**
     * @brief A constructor for split_path_follower::SplitPathFollower class
     * @param options Additional options to control creation of the node.
     */
    explicit SplitPathFollower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /**
     * @brief A destructor for split_path_follower::SplitPathFollower class
     */
    ~SplitPathFollower();

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

    /**
     * @brief Action server callbacks
     */
    void followWaypoints();

    /**
     * @brief Action client result callback
     * @param result Result of action server updated asynchronously
     */
    void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult& result);

    /**
     * @brief Action client goal response callback
     * @param goal Response of action server updated asynchronously
     */
    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr& goal);

    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    // Dynamic parameters handler
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    // Our action server
    std::unique_ptr<ActionServer> action_server_;
    ActionClient::SharedPtr nav_to_pose_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
    bool stop_on_failure_;
    ActionStatus current_goal_status_;
    int loop_rate_;
    std::vector<int> failed_ids_;

    // Task Execution At Waypoint Plugin
    pluginlib::ClassLoader<nav2_core::WaypointTaskExecutor> waypoint_task_executor_loader_;
    pluginlib::UniquePtr<nav2_core::WaypointTaskExecutor> waypoint_task_executor_;
    std::string waypoint_task_executor_id_;
    std::string waypoint_task_executor_type_;
  };

}   // namespace split_path_follower

#endif   // BEHAVIOR_TREE_SERVER__SPLIT_PATH_TO_GOAL_HPP_