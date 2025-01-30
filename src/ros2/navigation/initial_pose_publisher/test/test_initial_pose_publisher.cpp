#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>
#include "initial_pose_publisher.hpp"

class TestInitialPosePublisher
{
  public:
    TestInitialPosePublisher()
    {
      node = std::make_shared<rclcpp::Node>("test_node");
    }

    ~TestInitialPosePublisher()
    {
      rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node;
};

void run_background_node()
{
  auto node = std::make_shared<InitialPoseNode>();
  rclcpp::spin(node);
}

TEST_CASE("TestInitialPosePublisherCallback")
{
  rclcpp::init(0, nullptr);
  TestInitialPosePublisher test_node;

  std::thread background_thread(run_background_node);

  const double x = 1.0;
  const double y = 2.0;
  const double z = 3.14;

  auto subscription = test_node.node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    10,
    [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
      REQUIRE(msg->header.frame_id == "map");
      REQUIRE(std::abs(msg->pose.pose.position.x - x) < 1e-5);
      REQUIRE(std::abs(msg->pose.pose.position.y - y) < 1e-5);
      REQUIRE(std::abs(msg->pose.pose.orientation.z - std::sin(z / 2.0)) < 1e-5);
      REQUIRE(std::abs(msg->pose.pose.orientation.w - std::cos(z / 2.0)) < 1e-5);
      rclcpp::shutdown();
    });

  auto publisher = test_node.node->create_publisher<geometry_msgs::msg::Point>("/set_initial_pose", 10);

  rclcpp::WallRate(1).sleep();

  auto test_msg = geometry_msgs::msg::Point();
  test_msg.x = x;
  test_msg.y = y;
  test_msg.z = z;
  publisher->publish(test_msg);

  // Create main node and let it spin for a while
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node.node);

  // Spin the main node for 5 seconds
  std::thread main_thread(
    [&executor]()
    {
      executor.spin();
    });

  std::this_thread::sleep_for(std::chrono::seconds(5));

  rclcpp::shutdown();
  if (background_thread.joinable())
  {
    background_thread.join();
  }
  if (main_thread.joinable())
  {
    main_thread.join();
  }
}
