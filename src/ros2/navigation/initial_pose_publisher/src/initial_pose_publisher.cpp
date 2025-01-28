#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>

class InitialPoseNode : public rclcpp::Node {
public:
    InitialPoseNode() : Node("initial_pose_node") {
        set_initial_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/set_initial_pose",
            10,
            std::bind(&InitialPoseNode::set_initial_pose_callback, this, std::placeholders::_1)
        );

        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10
        );
    }

private:
    void set_initial_pose_callback(const geometry_msgs::msg::Point::SharedPtr point) {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();
        msg.pose.pose.position.x = point->x;
        msg.pose.pose.position.y = point->y;
        msg.pose.pose.orientation.z = std::sin(point->z / 2.0);
        msg.pose.pose.orientation.w = std::cos(point->z / 2.0);

        initial_pose_publisher_->publish(msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_initial_pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
