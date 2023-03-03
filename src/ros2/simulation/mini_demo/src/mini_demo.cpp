#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "mini_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("mini_demo");
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);
  std::thread(
      [&executor]()
      {
        executor.spin();
      })
      .detach();

  using moveit::planning_interface::MoveGroupInterface;
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "manipulator");

  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.509207546710968;
    msg.position.y = -0.03342067822813988;
    msg.position.z = 0.9415116906166077;
    msg.orientation.x = 0.2441798746585846;
    msg.orientation.y = -0.17545120418071747;
    msg.orientation.z = 0.7745198607444763;
    msg.orientation.w = 0.5565177798271179;
    return msg;
  }();
  move_group_interface.setGoalTolerance(0.000001);
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}