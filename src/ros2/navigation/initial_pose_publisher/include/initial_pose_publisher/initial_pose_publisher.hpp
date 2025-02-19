#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>

class InitialPoseNode : public rclcpp::Node {
public:
    InitialPoseNode();

private:
    void set_initial_pose_callback(const geometry_msgs::msg::Point::SharedPtr point);

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _set_initial_pose_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _initial_pose_publisher;
};