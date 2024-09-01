#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
class SaveMapPose : public rclcpp::Node
{
 public:
  SaveMapPose() : Node("save_map_pose_node")
  {
    _tf = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf->setUsingDedicatedThread(true);
    _transform_listener = std::make_shared<tf2_ros::TransformListener>(*_tf, this, false);
    _timer = this->create_wall_timer(5s, std::bind(&SaveMapPose::timer_callback, this));
  }

 private:
  void timer_callback()
  {
    geometry_msgs::msg::PoseStamped global_pose;
    if (nav2_util::getCurrentPose(global_pose, *_tf, "map", "base_link", 0.2))
    {
      RCLCPP_INFO(this->get_logger(),
                  "Save map pose: %f %f %f",
                  global_pose.pose.position.x,
                  global_pose.pose.position.y,
                  global_pose.pose.position.z);

      // Convert quaternion to yaw
      tf2::Quaternion q(global_pose.pose.orientation.x,
                        global_pose.pose.orientation.y,
                        global_pose.pose.orientation.z,
                        global_pose.pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      YAML::Node config = YAML::LoadFile("/workspace/last_pose.yaml");
      config["map_pose"]["position"]["x"] = global_pose.pose.position.x;
      config["map_pose"]["position"]["y"] = global_pose.pose.position.y;
      config["map_pose"]["position"]["z"] = global_pose.pose.position.z;
      config["map_pose"]["orientation"]["x"] = global_pose.pose.orientation.x;
      config["map_pose"]["orientation"]["y"] = global_pose.pose.orientation.y;
      config["map_pose"]["orientation"]["z"] = global_pose.pose.orientation.z;
      config["map_pose"]["orientation"]["w"] = global_pose.pose.orientation.w;
      config["map_pose"]["orientation"]["yaw"] = yaw;   // Save yaw instead of quaternion

      std::ofstream fout("/workspace/last_pose.yaml");
      fout << config;
    }

    // save_map_pose
  }
  std::shared_ptr<tf2_ros::Buffer> _tf;
  std::shared_ptr<tf2_ros::TransformListener> _transform_listener;
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaveMapPose>());
  rclcpp::shutdown();
  return 0;
}