#ifndef RB_THERON__DRAWER_BRIDGE_SIMULATION_HPP_
#define RB_THERON__DRAWER_BRIDGE_SIMULATION_HPP_

#include <boost/asio.hpp>   // Used for the async timer
#include <boost/bind.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "communication_interfaces/msg/drawer_address.hpp"
#include "communication_interfaces/msg/drawer_status.hpp"

namespace drawer_bridge_simulation
{

  class DrawerSimulation : public rclcpp::Node
  {
   public:
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    using DrawerStatus = communication_interfaces::msg::DrawerStatus;

    DrawerSimulation();
    ~DrawerSimulation(){};

   private:
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr _drawer_joint_trajectory_client;
    rclcpp::Subscription<DrawerAddress>::SharedPtr _open_drawer_subscription;
    rclcpp::Publisher<DrawerStatus>::SharedPtr _drawer_status_publisher;

    const float _target_pose_open_drawer = 0.34;
    const float _target_pose_closed_drawer = 0.0;

    uint32_t _time_until_drawer_closes_automatically;
    const int _DEFAULT_TIME_UNTIL_DRAWER_CLOSES_AUTOMATICALLY = 3000;   // ms

    int _num_of_drawers;
    const int _DEFAULT_NUM_OF_DRAWERS = 5;

    std::shared_ptr<rclcpp::Node> get_shared_pointer_of_node();

    void declare_parameters();

    void create_drawer_joint_trajectory_action_client();

    void common_goal_response(
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr future);

    void common_feedback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
                         const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback);

    void common_result_response(
        const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result,
        const DrawerAddress drawer_address,
        const float target_pose);

    trajectory_msgs::msg::JointTrajectoryPoint create_trajectory_point(const uint8_t module_id,
                                                                       const float target_position,
                                                                       const float time_from_start);

    control_msgs::action::FollowJointTrajectory_Goal create_trajectory_goal(const uint8_t module_id,
                                                                            const float target_position);

    void open_drawer_topic_callback(const DrawerAddress &msg);

    void move_drawer_in_simulation_to_target_pose(const DrawerAddress drawer_address, const float target_pose);

    void async_wait_until_closing_drawer_in_simulation(const int time_until_drawer_closing,
                                                       const DrawerAddress drawer_address,
                                                       const float target_pose);

    void open_drawer_in_simulation(const DrawerAddress drawer_address, const float target_pose);

    void send_drawer_feedback(communication_interfaces::msg::DrawerStatus drawer_status_msg, bool drawer_is_open);
  };
}   // namespace drawer_bridge_simulation
#endif   // RB_THERON__DRAWER_BRIDGE_SIMULATION_HPP_
