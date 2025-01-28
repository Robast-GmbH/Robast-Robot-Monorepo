#include <catch2/catch_all.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class TestInitialPosePublisher {
public:
  TestInitialPosePublisher() {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  ~TestInitialPosePublisher() {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
};

TEST_CASE("TestInitialPosePublisherCallback") {
  TestInitialPosePublisher test_node;

  const double x = 1.0;
  const double y = 2.0;
  const double z = 3.14;

  auto subscription = test_node.node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    10,
    [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      REQUIRE(msg->header.frame_id == "map");
      REQUIRE(std::abs(msg->pose.pose.position.x - x) < 1e-5);
      REQUIRE(std::abs(msg->pose.pose.position.y - y) < 1e-5);
      REQUIRE(std::abs(msg->pose.pose.orientation.z - std::sin(z / 2.0)) < 1e-5);
      REQUIRE(std::abs(msg->pose.pose.orientation.w - std::cos(z / 2.0)) < 1e-5);
      rclcpp::shutdown();
    }
  );

  auto publisher = test_node.node->create_publisher<geometry_msgs::msg::Point>("/set_initial_pose", 10);

  rclcpp::WallRate(1).sleep();

  auto test_msg = geometry_msgs::msg::Point();
  test_msg.x = x;
  test_msg.y = y;
  test_msg.z = z;
  publisher->publish(test_msg);

  rclcpp::spin_some(test_node.node);
}
