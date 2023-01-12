#ifndef RB_THERON__JOINT_STATE_PUBLISHER_HPP_
#define RB_THERON__JOINT_STATE_PUBLISHER_HPP_

#include <ignition/transport/Node.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>

namespace rb_theron_controller_manager {

class JointStatePublisher {
    public:
        JointStatePublisher(const rclcpp::Node::SharedPtr& nh,
            const std::vector<std::string>& joint_names,
            const std::string& ros_topic, 
            const std::string& ign_topic,
            const unsigned int update_rate);
        ~JointStatePublisher() {};

    private:
        void jointStateTimerCb();
        //callback for Ignition
        void ignJointStateCb(const ignition::msgs::Model& msg);

    private:
        rclcpp::Node::SharedPtr nh_;
        std::shared_ptr<ignition::transport::Node> ign_node_;
        // ros pub and sub
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros_joint_state_pub_;
        rclcpp::TimerBase::SharedPtr joint_state_timer_;
        // joint names and map
        std::vector<std::string> joint_names_;
        std::map<std::string,int> joint_names_map_;
        //joint state info recieved form Ignition
        ignition::msgs::Model current_ign_joint_msg_;
        sensor_msgs::msg::JointState current_joint_msg_;
        std::mutex current_joint_msg_mut_;
    };

}

#endif //RB_THERON__JOINT_STATE_PUBLISHER_HPP_