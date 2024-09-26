#include "save_pose/save_map_pose.hpp"

SaveMapPose::SaveMapPose() : Node("save_map_pose_node")
{
  using namespace std::chrono_literals;
  _tf = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  _tf->setUsingDedicatedThread(true);
  _transform_listener = std::make_shared<tf2_ros::TransformListener>(*_tf, this, false);
  _timer = this->create_wall_timer(1s, std::bind(&SaveMapPose::timer_callback, this));
  _base_frame = this->declare_parameter<std::string>("base_frame", "robot/base_link");
  _file_path = this->declare_parameter<std::string>("file_path", "/workspace/last_pose.yaml");
}

void SaveMapPose::timer_callback()
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (nav2_util::getCurrentPose(global_pose, *_tf, "map", _base_frame, 0.2))
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

    YAML::Node config;

    // Check if the file exists
    if (std::filesystem::exists(_file_path))
    {
      config = YAML::LoadFile(_file_path);
    }
    else
    {
      config = YAML::Node();
    }

    config["map_pose"]["position"]["x"] = global_pose.pose.position.x;
    config["map_pose"]["position"]["y"] = global_pose.pose.position.y;
    config["map_pose"]["position"]["z"] = global_pose.pose.position.z;
    config["map_pose"]["orientation"]["yaw"] = yaw;

    std::ofstream fout(_file_path);
    fout << config;
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaveMapPose>());
  rclcpp::shutdown();
  return 0;
}