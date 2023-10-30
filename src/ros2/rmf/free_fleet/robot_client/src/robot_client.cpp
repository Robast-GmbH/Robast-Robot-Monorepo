#include "robot_client/robot_client.hpp"

namespace rmf_robot_client
{

  RobotClient::RobotClient() : Node("robot_client")
  {
    init_param();
    start_receive_tasks();
    start_update_robot_state();
    _drawer_list = std::make_shared<std::map<std::string, DrawerState>>();
    _task_sizes = std::map<int, int>();
    _is_new_tf_error = true;

    _reset_simple_tree_publisher =
        this->create_publisher<StdMsgBool>(this->get_parameter("statemaschine_reset_simple_tree_topic").as_string(),
                                           QoSConfig::get_statemaschine_reset_tree_qos());
  }

  void RobotClient::init_param()
  {
    this->declare_parameter("fleet_name", "ROBAST");
    this->declare_parameter("robot_name", "RB0");
    this->declare_parameter("robot_model", "Robast_Theron");
    this->declare_parameter("nav_behavior_tree",
                            "/workspace/src/navigation/nav_bringup/behavior_trees/humble/"
                            "navigate_to_pose_w_replanning_goal_patience_and_recovery.xml");
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("robot_frame_id", "robot_base_footprint");
    this->declare_parameter("robot_info_inteval", 1);     // in seconds
    this->declare_parameter("nfc_timeout_interval", 1);   // in minutes

    this->declare_parameter("statemaschine_open_drawer_topic", "/trigger_drawer_tree");
    this->declare_parameter("statemaschine_open_e_drawer_topic", "/trigger_electric_drawer_tree");
    this->declare_parameter("statemaschine_close_e_drawer_topic", "/close_drawer");
    this->declare_parameter("statemaschine_reset_simple_tree_topic", "/reset_simple_tree");
    this->declare_parameter("drawer_bridge_drawer_status_change_topic", "/drawer_is_open");

    this->declare_parameter("nfc_write_usertag_to_card_action_topic", "/create_user");
    this->declare_parameter("nfc_on_off_switch_topic", "/nfc_switch");
    this->declare_parameter("nfc_authenticated_user_topic", "/authenticated_user");

    this->declare_parameter("fleet_communication_compose_header_topic", "/compose_header");
    this->declare_parameter("fleet_communication_create_nfc_topic", "/new_user_request");
    this->declare_parameter("fleet_communication_destination_topic", "/destination_request");
    this->declare_parameter("fleet_communication_drawer_topic", "/drawer_request");
    this->declare_parameter("fleet_communication_setting_topic", "/setting_request");

    this->declare_parameter("fleet_communication_task_info_topic", "/task_state");
    this->declare_parameter("fleet_communication_robot_info_topic", "/robot_state");

    this->declare_parameter("nav2_navigation_to_pose_action_topic", "/navigate_to_pose");
    this->declare_parameter("robotnik_battery_level_topic", "/robot/battery_estimator/data");

    _robot.fleet_name = this->get_parameter("fleet_name").as_string();
    _robot.robot_name = this->get_parameter("robot_name").as_string();
    _robot_model = this->get_parameter("robot_model").as_string();
    _map_frame_id = this->get_parameter("map_frame_id").as_string();
    _robot_frame_id = this->get_parameter("robot_frame_id").as_string();
    _robot_info_inteval = this->get_parameter("robot_info_inteval").as_int();
  }

  void RobotClient::start_receive_tasks()
  {
    _task_sequence_request_subscriber = this->create_subscription<FleetDataTaskSequenceHeaderRequest>(
        this->get_parameter("fleet_communication_compose_header_topic").as_string(),
        QoSConfig::get_fleet_communication_qos(),
        std::bind(&RobotClient::receive_task_sequence_header, this, std::placeholders::_1));

    _write_nfc_card_request_subscriber = this->create_subscription<FleetDataCreateNfcRequest>(
        this->get_parameter("fleet_communication_create_nfc_topic").as_string(),
        QoSConfig::get_fleet_communication_qos(),
        std::bind(&RobotClient::receive_create_nfc_task, this, std::placeholders::_1));

    _drawer_request_subscriber = this->create_subscription<FleetDataDrawerRequest>(
        this->get_parameter("fleet_communication_drawer_topic").as_string(),
        QoSConfig::get_fleet_communication_qos(),
        std::bind(&RobotClient::receive_drawer_task, this, std::placeholders::_1));

    _navigation_request_subscriber = this->create_subscription<FleetDataDestinationRequest>(
        this->get_parameter("fleet_communication_destination_topic").as_string(),
        QoSConfig::get_fleet_communication_qos(),
        std::bind(&RobotClient::receive_destination_task, this, std::placeholders::_1));

    _setting_subscriber = this->create_subscription<FleetDataSettingRequest>(
        this->get_parameter("fleet_communication_setting_topic").as_string(),
        QoSConfig::get_fleet_communication_qos(),
        std::bind(&RobotClient::receive_settings, this, std::placeholders::_1));

    _drawer_status_subscriber = this->create_subscription<DrawerStatus>(
        this->get_parameter("drawer_bridge_drawer_status_change_topic").as_string(),
        10,
        std::bind(&RobotClient::receive_drawer_status, this, std::placeholders::_1));

    _authentication_subscriber = this->create_subscription<StdMsgInt>(
        this->get_parameter("nfc_authenticated_user_topic").as_string(),
        10,
        std::bind(&RobotClient::receive_authenticated_user, this, std::placeholders::_1));
  }

  void RobotClient::receive_task_sequence_header(const FleetDataTaskSequenceHeaderRequest::ConstPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Task_Sequence_Header_received");
    TaskId task_id = TaskId();
    if (!task_validation(msg->task_id, RobotRef(msg->fleet_name, msg->robot_name), std::make_unique<TaskId>(task_id)))
    {
      return;
    }

    _task_sizes[std::stoi(msg->task_id)] = msg->sequence_length;
  }

  void RobotClient::receive_create_nfc_task(const FleetDataCreateNfcRequest::ConstPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "nfc_received");

    TaskId task_id = TaskId();
    if (!task_validation(msg->task_id, RobotRef(msg->fleet_name, msg->robot_name), std::make_unique<TaskId>(task_id)))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(_receive_task_mutex);
    if (!task_extension_validaton(task_id))
    {
      return;
    }

    if (!_task_sequence
             .insert(
                 std::make_pair(task_id.phase, std::make_unique<NFCTask>(task_id, shared_from_this(), msg->user_id)))
             .second)
    {
      RCLCPP_ERROR(
          this->get_logger(), "Task %i, phase %i could not be added to robot Queue", task_id.id, task_id.phase);
      return;
    }

    start_task();
  }

  void RobotClient::receive_drawer_task(const FleetDataDrawerRequest::ConstPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "drawer_received");
    TaskId task_id = TaskId();
    if (!task_validation(msg->task_id, RobotRef(msg->fleet_name, msg->robot_name), std::make_unique<TaskId>(task_id)))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(_receive_task_mutex);
    if (!task_extension_validaton(task_id))
    {
      return;
    }

    if (!_task_sequence
             .insert(std::make_pair(
                 task_id.phase,
                 std::make_unique<DrawerTask>(DrawerTask(
                     task_id,
                     shared_from_this(),
                     _drawer_list,
                     DrawerState(
                         DrawerRef(msg->module_id, msg->drawer_id), msg->e_drawer, false, msg->authorized_user)))))
             .second)
    {
      RCLCPP_ERROR(
          this->get_logger(), "Task %i, phase %i could not be added to robot Queue", task_id.id, task_id.phase);
      return;
    }

    start_task();
  }

  void RobotClient::receive_destination_task(const FleetDataDestinationRequest::ConstPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "destination_received");
    TaskId task_id;
    if (!task_validation(msg->task_id, RobotRef(msg->fleet_name, msg->robot_name), std::make_unique<TaskId>(task_id)))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(_receive_task_mutex);
    if (!task_extension_validaton(task_id))
    {
      return;
    }

    if (!_task_sequence
             .insert(std::make_pair(task_id.phase,
                                    std::make_unique<NavigationTask>(
                                        task_id,
                                        shared_from_this(),
                                        RobotPose(msg->destination.x, msg->destination.y, msg->destination.yaw))))
             .second)
    {
      RCLCPP_ERROR(
          this->get_logger(), "Task %i, phase %i could not be added to robot Queue", task_id.id, task_id.phase);
      return;
    }

    start_task();
  }

  void RobotClient::receive_settings(const FleetDataSettingRequest::SharedPtr msg)
  {
    //  global settings (reset tree,..)
    if (msg->command == "reset_tree")
    {
      StdMsgBool msg = StdMsgBool();
      msg.data = true;
      _reset_simple_tree_publisher->publish(msg);
    }
    else if (_current_task.phase == 0 || _current_task.phase > _task_sequence.size())
    {
      RCLCPP_INFO(this->get_logger(), "invalid request");
      return;
    }
    else
    {
      _task_sequence[_current_task.phase]->receive_new_settings(msg->command, msg->value);   // task settings
    }
    RCLCPP_INFO(this->get_logger(), "settings done");
  }

  void RobotClient::receive_drawer_status(const DrawerStatus::SharedPtr msg)
  {
    std::string drawer_ref =
        std::to_string(msg->drawer_address.module_id) + '#' + std::to_string(msg->drawer_address.drawer_id);
    if (_current_task.phase == 0 || _current_task.phase > _task_sequence.size())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Drawer %s state received(Drawer_id: %i, Module_id: %i ).",
                   msg->drawer_is_open ? "open" : "close",
                   msg->drawer_address.drawer_id,
                   msg->drawer_address.module_id);
      return;
    }
    else
    {
      _task_sequence[_current_task.phase]->receive_new_settings(
          "DrawerState", {drawer_ref + "#" + (msg->drawer_is_open ? "Opened" : "Closed")});
    }

    if (_drawer_list->count(drawer_ref))
    {
      _drawer_list->at(drawer_ref).is_open = msg->drawer_is_open;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "External triggered Drawer %s state received (Drawer_id: %i, Module_id: %i ).",
                   msg->drawer_is_open ? "open" : "close",
                   msg->drawer_address.drawer_id,
                   msg->drawer_address.module_id);
    }
  }

  void RobotClient::receive_authenticated_user(const StdMsgInt::SharedPtr msg)
  {
    _task_sequence[_current_task.phase]->receive_new_settings("drawer",
                                                              {"Authenticated_user", std::to_string(msg->data)});
  }

  void RobotClient::start_update_robot_state()
  {
    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    update_robot_location();

    // TODO@Torben: currently not funktional since the Bridge is not able to bridge the topic properly.
    // until the BMS is not in ROS2 or the bridge is fixed this is not usable

    // battery_status_sub_ = this->create_subscription<BatteryLevel>(
    //     this->get_parameter("robotnik_battery_level_topic").as_string(),
    //     QoSConfig::get_qos_robotnik_bms_qos(),
    //     std::bind(&RobotClient::update_battery_level, this, std::placeholders::_1));

    _robot_info_publisher = this->create_publisher<FleetDataRobotState>(
        this->get_parameter("fleet_communication_robot_info_topic").as_string(),
        QoSConfig::get_fleet_communication_status_qos());
    _publish_robot_info_timer = this->create_wall_timer(std::chrono::seconds(_robot_info_inteval),
                                                        std::bind(&RobotClient::publish_fleet_state, this));
  }

  void RobotClient::publish_fleet_state()
  {
    update_robot_location();
    FleetDataRobotState robot_state_msg = FleetDataRobotState();

    robot_state_msg.name = _robot.robot_name;
    robot_state_msg.model = _robot_model;
    robot_state_msg.task_id = _current_task.id;

    robot_state_msg.location.level_name = "";
    robot_state_msg.location.nanosec = 0;
    robot_state_msg.location.sec = 0;
    robot_state_msg.location.x = _current_robot_pose.x_pose;
    robot_state_msg.location.y = _current_robot_pose.y_pose;
    robot_state_msg.location.yaw = _current_robot_pose.yaw_pose;

    robot_state_msg.battery_percent = _current_battery_level;
    _robot_info_publisher->publish(robot_state_msg);
  }

  void RobotClient::update_robot_location()
  {
    geometry_msgs::msg::TransformStamped transform_map_to_robot;
    try
    {
      transform_map_to_robot = _tf_buffer->lookupTransform(_map_frame_id, _robot_frame_id, tf2::TimePointZero);
      if (!_is_new_tf_error)
      {
        RCLCPP_INFO(this->get_logger(), "Transformation error resolved.");
        _is_new_tf_error = true;
      }
    }
    catch (const tf2::TransformException& ex)
    {
      if (_is_new_tf_error)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Could not transform %s to %s: %s",
                    _map_frame_id.c_str(),
                    _robot_frame_id.c_str(),
                    ex.what());
        _is_new_tf_error = false;
      }
      return;
    }

    double roll, pitch, yaw;
    tf2::Quaternion quat;
    tf2::fromMsg(transform_map_to_robot.transform.rotation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    _current_robot_pose =
        RobotPose(transform_map_to_robot.transform.translation.x, transform_map_to_robot.transform.translation.y, yaw);

    RCLCPP_DEBUG(this->get_logger(),
                 "x: %f, y: %f, z:%f w: %f",
                 transform_map_to_robot.transform.rotation.x,
                 transform_map_to_robot.transform.rotation.y,
                 transform_map_to_robot.transform.rotation.z,
                 transform_map_to_robot.transform.rotation.w);
    RCLCPP_DEBUG(this->get_logger(), "yaw: %f, degree %f", yaw, yaw * (180 / M_PI) + 180);
  }

  //  ToDo only works after the topic is bridged properly
  //  void RobotClient::receive_battery_status(const BatteryLevel::ConstPtr msg)
  // {
  //   current_battery_level= msg->level;
  // }

  bool RobotClient::start_task()
  {
    if (!(_task_sequence.size() == 1 && _task_sequence.find(0) != _task_sequence.end()))
    {
      _current_task.phase = 0;
    }
    else if (_task_sizes.find(_current_task.id) != _task_sizes.end() &&
             _task_sequence.size() == _task_sizes[_current_task.id])
    {
      if (!link_tasks())
      {
        return false;
      }
    }
    else
    {
      return false;
    }

    _task_executer = std::thread(std::bind(&BaseTask::start, _task_sequence[_current_task.phase]));
    _task_executer.detach();
    return true;
  }

  bool RobotClient::link_tasks()
  {
    for (int i = 1; i < _task_sequence.size(); i++)
    {
      if (_task_sequence.find(i) != _task_sequence.end())
      {
        _task_sequence[i]->assign_next_task(_task_sequence[i + 1]);
      }
      else
      {
        return false;
      }
    }

    if (_task_sequence[_task_sequence.size()])
    {
      _current_task.phase = 1;
    }
    else
    {
      return false;
    }
    return true;
  }

  bool RobotClient::task_validation(std::string task_def, RobotRef robot, std::unique_ptr<TaskId> task_id)
  {
    // check if the task is for this robot
    if (!(robot.fleet_name == _robot.fleet_name && robot.robot_name == _robot.robot_name))
    {
      return false;
    }

    // check that the task ID is valid
    *task_id = TaskId(task_def);
    if (task_id->id == 0)
    {
      return false;
    }
    return true;
  }

  bool RobotClient::task_extension_validaton(TaskId task_id)
  {
    // check if a task is already running
    if (!_task_executer.joinable())
    {
      return false;
    }

    // check if a phase already exists
    if (_task_sequence.find(task_id.phase) != _task_sequence.end())
    {
      return false;
    }

    // resets the task queue if the phase is not of the current task
    if (_current_task.id != task_id.id)
    {
      empty_task_sequence();
      _current_task.id = task_id.id;
    }

    return true;
  }

  void RobotClient::empty_task_sequence()
  {
    for (auto& id_by_task : _task_sequence)
    {
      id_by_task.second.reset();
    }
    _task_sequence.clear();
    _current_task.phase = 0;
  }

}   // namespace rmf_robot_client
