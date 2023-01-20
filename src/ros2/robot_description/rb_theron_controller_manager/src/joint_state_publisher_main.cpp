#include <rclcpp/rclcpp.hpp>
#include <rb_theron_controller_manager/joint_state_publisher.hpp>

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_state_publisher");
    // variable
    std::vector<std::string> joint_names;
    std::vector<std::string> default_joint_names = {"top_drawer_joint"};
    std::string ign_topic;
    int update_rate;
    // get parameter
    ros_node->declare_parameter("joint_names", default_joint_names);
    ros_node->declare_parameter("ign_topic", "");
    ros_node->declare_parameter("rate", 30);
    joint_names = ros_node->get_parameter("joint_names").get_parameter_value().get<std::vector<std::string>>();
    ign_topic = ros_node->get_parameter("ign_topic").as_string();
    update_rate = ros_node->get_parameter("rate").as_int();

    // create publisher
    auto joint_publisher = std::make_shared<rb_theron_controller_manager::JointStatePublisher>(ros_node,
        joint_names, "joint_states", ign_topic, update_rate);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
