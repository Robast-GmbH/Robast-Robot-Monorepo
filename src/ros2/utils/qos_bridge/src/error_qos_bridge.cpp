#include <functional>
#include "qos_bridge/qos_bridge.hpp"
#include "communication_interfaces/msg/error_base_msg.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::QoS qos_error_msgs = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos_error_msgs.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_error_msgs.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_error_msgs.avoid_ros_namespace_conventions(false);

    auto error_qos_bridge = std::make_shared<QosBridge<communication_interfaces::msg::ErrorBaseMsg>>(
        "robast_error",        
        "robast_error_best_effort", 
        qos_error_msgs,             
        10                          
    );

    rclcpp::spin(error_qos_bridge);
    rclcpp::shutdown();

    return 0;
}