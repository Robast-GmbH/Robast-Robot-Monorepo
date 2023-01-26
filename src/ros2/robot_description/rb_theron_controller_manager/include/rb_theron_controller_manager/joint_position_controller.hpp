#ifndef RB_THERON__JOINT_POSITION_CONTROLLER_HPP_
#define RB_THERON__JOINT_POSITION_CONTROLLER_HPP_

#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>

namespace rb_theron_controller_manager
{

    class JointPositionController
    {
        public:
            JointPositionController(const rclcpp::Node::SharedPtr& nh,
                const std::vector<std::string>& joint_names,
                const std::string& ros_cmd_topic,
                const std::vector<std::string>& gz_cmd_topics);
            ~JointPositionController() {};

        private:
            void set_joint_position_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
                        
            private:
            rclcpp::Node::SharedPtr nh_;
            std::shared_ptr<gz::transport::Node> gz_node_;
            // ros pub and sub
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ros_cmd_joint_state_sub_;
            //gz pub
            std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
            // joint names and map
            std::vector<std::string> joint_names_;
            std::unordered_map<std::string, int> joint_names_map_;
    };
}
#endif //RB_THERON__JOINT_POSITION_CONTROLLER_HPP_