#ifndef RB_THERON__JOINT_TRAJECTORY_CONTROLLER_HPP_
#define RB_THERON__JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <unordered_map>

namespace gazebo_controller_manager
{
    class JointTrajectoryController: public rclcpp::Node
    {
        public:
            JointTrajectoryController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            ~JointTrajectoryController() {};

        private:
            void initialize_gz_transport_node(std::vector<std::string> joint_names, std::vector<std::string> gz_joint_topics);
        
            void set_joint_trajectory_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr);
            
            void update_position_timer_cb();

            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
            
            void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

            void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

            bool is_trajectory_motion_finished();

        private:
            std::shared_ptr<gz::transport::Node> gz_transport_node_;
            rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
            
            //gz pub
            std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
            rclcpp::TimerBase::SharedPtr update_position_timer_;
            // joint names and map
            std::vector<std::string> joint_names_;
            size_t joint_num_;
            std::vector<double> target_positions_;
            std::unordered_map<std::string, int> joint_names_map_;
            std::mutex trajectory_mutex_;
            std::mutex cv_mutex_;
            std::condition_variable cv_;
            std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_;
            rclcpp::Time trajectory_start_time_;
            unsigned int trajectory_index_;
            bool has_trajectory_ { false };
    };
}
#endif //RB_THERON__JOINT_TRAJECTORY_CONTROLLER_HPP_