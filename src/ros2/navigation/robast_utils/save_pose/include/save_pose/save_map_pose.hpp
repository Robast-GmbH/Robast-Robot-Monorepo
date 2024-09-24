#ifndef SAVE_MAP_POSE_HPP
#define SAVE_MAP_POSE_HPP

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

class SaveMapPose : public rclcpp::Node
{
 public:
  SaveMapPose();

 private:
    void timer_callback();

    std::shared_ptr<tf2_ros::Buffer> _tf;
    std::shared_ptr<tf2_ros::TransformListener> _transform_listener;
    rclcpp::TimerBase::SharedPtr _timer;
    std::string _file_path = "/workspace/last_pose.yaml";
    std::string _base_frame = "base_link";
};

#endif // SAVE_MAP_POSE_HPP
