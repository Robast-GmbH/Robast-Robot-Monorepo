#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class TestInitialPoseNode : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(TestInitialPoseNode, TestInitialPoseCallback) {
    const int x = 1.0;
    const int y = 2.0;
    const double z = 3.14;

    auto subscription = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10,
        [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            EXPECT_EQ(msg->header.frame_id, "map");
            EXPECT_NEAR(msg->pose.pose.position.x, x, 1e-5);
            EXPECT_NEAR(msg->pose.pose.position.y, y, 1e-5);
            EXPECT_NEAR(msg->pose.pose.orientation.z, std::sin(z / 2.0), 1e-5);
            EXPECT_NEAR(msg->pose.pose.orientation.w, std::cos(z / 2.0), 1e-5);
            rclcpp::shutdown();
        }
    );

    auto publisher = node_->create_publisher<geometry_msgs::msg::Point>("/set_initial_pose", 10);

    rclcpp::WallRate(1).sleep();

    auto test_msg = geometry_msgs::msg::Point();
    test_msg.x = x;
    test_msg.y = y;
    test_msg.z = z;
    publisher->publish(test_msg);

    rclcpp::spin(node_);
}
