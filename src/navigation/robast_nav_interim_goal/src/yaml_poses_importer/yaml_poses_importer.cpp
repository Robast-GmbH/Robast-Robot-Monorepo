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

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "yaml_poses_importer", std::bind(&YamlPosesImporter::provide_poses, this), false);

  RCLCPP_INFO(get_logger(), "End of Configuring");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn YamlPosesImporter::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void YamlPosesImporter::provide_poses()
{
  RCLCPP_INFO(get_logger(), "YamlPosesImporter callback!");

  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  if (!is_request_valid(goal, result)) {
    return;
  }

  std::string poses_yaml_filename = goal->yaml_name;

  // Only import the yaml, if it hasn't been imported before
  if (!yaml_filename_by_poses_.contains(poses_yaml_filename))
  {
    load_poses_from_yaml(poses_yaml_filename);
  }  

  RCLCPP_INFO(
    get_logger(), "Provided list of poses with %i poses!",
    yaml_filename_by_poses_[poses_yaml_filename].size());
  result->poses = yaml_filename_by_poses_[poses_yaml_filename];
  action_server_->succeeded_current(result);
}

bool YamlPosesImporter::is_request_valid(
  const std::shared_ptr<const typename ActionT::Goal> goal,
  std::shared_ptr<ActionT::Result> result)
{
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return false;
  } 

  if (goal->yaml_name.empty()) {
    RCLCPP_ERROR(get_logger(), "Invalid yaml filename, string is empty.");
    action_server_->terminate_current(result);
    return false;
  }

  return true;
}

void YamlPosesImporter::load_poses_from_yaml(const std::string poses_yaml_filename)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("robast_nav_interim_goal");
  std::filesystem::path yaml_path = std::filesystem::path(package_share_directory);
  yaml_path.append("config");
  yaml_path.append(poses_yaml_filename);

  RCLCPP_INFO(
    get_logger(), "Importing yaml from path" + yaml_path.string());

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
  yaml_filename_by_poses_[poses_yaml_filename] = poses;

  RCLCPP_INFO(
    get_logger(), "Imported list of poses with %i poses!",
    yaml_filename_by_poses_[poses_yaml_filename].size());
}

} // namespace robast_nav_poses_importer