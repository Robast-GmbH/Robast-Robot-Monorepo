#include <inttypes.h>
#include <memory>
#include <filesystem>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "robast_nav_interim_goal/yaml_poses_importer.hpp"


namespace robast_nav_poses_importer
{


YamlPosesImporter::YamlPosesImporter()
: nav2_util::LifecycleNode("robast_nav_poses_importer", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

YamlPosesImporter::~YamlPosesImporter()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn YamlPosesImporter::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  
  import_poses_service_ = create_service<robast_msgs::srv::ImportYamlPoses>(
    "robast_import_poses_service",
    std::bind(&YamlPosesImporter::providePosesCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(get_logger(), "End of Configuring");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void YamlPosesImporter::providePosesCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<robast_msgs::srv::ImportYamlPoses::Request> request,
  std::shared_ptr<robast_msgs::srv::ImportYamlPoses::Response> response)
{
  std::string poses_yaml_filename = request->yaml_name;

  // Only import the yaml, if it hasn't been imported before
  if (!yaml_filename_by_poses.contains(poses_yaml_filename))
  {
    load_poses_from_yaml(poses_yaml_filename);
  }  

  response->poses = yaml_filename_by_poses[poses_yaml_filename];
}

void YamlPosesImporter::load_poses_from_yaml(const std::string poses_yaml_filename)
{
  std::filesystem::path yaml_path = std::filesystem::current_path();
  yaml_path.append("config");
  yaml_path.append(poses_yaml_filename);

  YAML::Node doc = YAML::LoadFile(yaml_path.string());

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (std::size_t i=1; i<doc.size()+1; i++) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = doc[i]["x"].as<double>();
    pose_stamped.pose.position.y = doc[i]["y"].as<double>() * (-1.0);

    //transform euler pose orientation to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, doc[i]["yaw"].as<double>());
    pose_stamped.pose.orientation = tf2::toMsg(q);

    poses.push_back(pose_stamped);
  }
  yaml_filename_by_poses[poses_yaml_filename] = poses;
}

} // namespace robast_nav_poses_importer