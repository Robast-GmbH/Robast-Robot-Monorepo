#include <rclcpp/rclcpp.hpp>
#include <rb_theron_controller_manager/joint_position_controller.hpp>

int main(int argc, char* argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_position_controller");
    // variable
    std::vector<std::string> joint_names;
    std::vector<std::string> default_joint_names = {"drawer_1_joint", "drawer_2_joint", "drawer_3_joint", "drawer_4_joint", "drawer_5_joint"};
    std::vector<std::string> gz_joint_topics;
    std::vector<std::string> default_gz_joint_topics = {"drawer_1_joint", "drawer_2_joint", "drawer_3_joint", "drawer_4_joint", "drawer_5_joint"};
    // parameters
    ros_node->declare_parameter("joint_names", default_joint_names);
    ros_node->declare_parameter("gz_joint_topics", default_gz_joint_topics);
    ros_node->declare_parameter("rate", 200);
    joint_names = ros_node->get_parameter("joint_names").get_parameter_value().get<std::vector<std::string>>();
    gz_joint_topics = ros_node->get_parameter("gz_joint_topics").get_parameter_value().get<std::vector<std::string>>();
    // create controller 
    auto joint_position_controller = std::make_shared<rb_theron_controller_manager::JointPositionController>(ros_node,
        joint_names, "set_joint_state", gz_joint_topics);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
