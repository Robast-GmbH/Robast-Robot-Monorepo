#include "robot_client/robot_client.hpp"

namespace rmf_robot_client
{

  RobotClient::RobotClient() : Node("robot_client")
  {
    init_param();
    start_receive_tasks();
  
  }

  void RobotClient::init_param()
  {
    this->declare_parameter<std::string>("fleet_name", "ROBAST");
    this->declare_parameter<std::string>("robot_name", "RB0");
    this->declare_parameter<std::string>("robot_model", "Robast_Theron");

    this->declare_parameter<std::string>("robot_frame_id", "map");
    this->declare_parameter<std::string>("robot_odom", "/odometry/filtered");
    this->declare_parameter<std::string>("behavior_tree", "/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_to_pose_w_replanning_goal_patience_and_recovery.xml");
    this->declare_parameter<std::string>("move_base_server_name", "goal_pose");
    this->declare_parameter<std::string>("tf_goal_frame", "map");
     this->declare_parameter<std::string>("tf_start_frame", "robot_base_footprint");

    this->declare_parameter<int8_t>("update_frequency", 500);
    this->declare_parameter<float>("patrol_break_frequency", 0.0056);//3 minute;
    this->declare_parameter<float>("dds_domain", 42);

    this->declare_parameter<std::string>("statemaschine_open_drawer_topic", "trigger_drawer_tree");
    this->declare_parameter<std::string>("statemaschine_close_e_drawer_topic", "trigger_electric_drawer_tree");
    this->declare_parameter<std::string>("statemaschine_open_e_drawer_topic", "close_drawer");
    this->declare_parameter<std::string>("statemaschine_reset_simple_tree_topic", "reset_simple_tree");
    this->declare_parameter<std::string>("drawer_status_change_topic", "/drawer_is_open");

    this->declare_parameter<std::string>("fleet_communication_setting_topic", "setting_request");
    this->declare_parameter<std::string>("fleet_communication_create_nfc_topic", "new_user_request");
    this->declare_parameter<std::string>("fleet_communication_destination_topic", "destination_request");
    this->declare_parameter<std::string>("fleet_communication_drawer_topic", "drawer_request");

    this->declare_parameter<std::string>("fleet_communication_robot_info_topic", "robot_state");
    this->declare_parameter<std::string>("fleet_communication_task_info_topic", "task_state");

    this->get_parameter_or<std::string>("fleet_name", fleet_name);
    this->get_parameter_or<std::string>("robot_name", robot_name);
    this->get_parameter_or<std::string>("robot_model", robot_model);

    // this->get_parameter_or<std::string>("robot_frame_id", robot_frame_id);
    // this->get_parameter_or<std::string>("robot_odom", robot_odom);
    //this->get_parameter_or<std::string>("behavior_tree", behavior_tree);
    // this->get_parameter_or<std::string>("move_base_server_name", move_base_server_name);
    this->get_parameter_or<std::string>("tf_goal_frame", tf_goal_frame);
    this->get_parameter_or<std::string>("tf_start_frame", tf_start_frame);

    this->get_parameter_or<int8_t>("update_frequency", update_frequency);
    // this->get_parameter_or<float>("patrol_break_frequency", patrol_break_frequency);
    // this->get_parameter_or<float>("dds_domain", dds_domain);

    get_parameter_to_config("statemaschine_open_drawer_topic");
    get_parameter_to_config( "statemaschine_open_e_drawer_topic");
    get_parameter_to_config( "statemaschine_close_e_drawer_topic");
    get_parameter_to_config( "statemaschine_reset_simple_tree_topic");
    get_parameter_to_config("drawer_status_change_topic");
    
    get_parameter_to_config("fleet_communication_setting_topic");
    get_parameter_to_config("fleet_communication_create_nfc_topic");
    get_parameter_to_config("fleet_communication_destination_topic");
    get_parameter_to_config("fleet_communication_drawer_topic");

    get_parameter_to_config("fleet_communication_task_info_topic");
    get_parameter_to_config("fleet_communication_robot_info_topic");

  }

  void RobotClient::get_parameter_to_config(std::string parameter_name )
  {   
    std::string temp;
    this->get_parameter_or<std::string>(parameter_name, temp);
    config.insert( std::make_pair( parameter_name, temp));
  }

  void RobotClient::start_receive_tasks()
  {
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);

    setting_subscriber_ = this->create_subscription<FreeFleetDataSettingRequest>(config["fleet_communication_setting_topic"], qos, std::bind(&RobotClient::receive_settings, this,std::placeholders::_1));
    write_nfc_card_request_subscriber_ = this->create_subscription<FreeFleetDataCreateNfcRequest>(config["fleet_communication_create_nfc_topic"], qos, std::bind(&RobotClient::receive_create_nfc_task, this,std::placeholders::_1));
    drawer_request_subscriber_ = this->create_subscription<FreeFleetDataDrawerRequest>(config["fleet_communication_drawer_topic"],qos, std::bind(&RobotClient::receive_drawer_task, this, std::placeholders::_1));
    navigation_request_subscriber_ = this->create_subscription<FreeFleetDataDestinationRequest>(config["fleet_communication_destination_topic"], qos, std::bind(&RobotClient::receive_destination_task, this, std::placeholders::_1));

  }
  void RobotClient::initialise_task_publisher()
  {
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);

    robot_info_publisher_ = this->create_publisher<FreeFleetDataRobotInfo>(config["fleet_communication_robot_info_topic"], qos);
    // task_info_publisher_ = this->create_publisher<FreeFleetDataTaskInfo>(fleet_communication_task_info_topic, qos);
    publish_robot_info_timer= this->create_wall_timer(std::chrono::milliseconds(update_frequency), std::bind(&RobotClient::publish_fleet_state, this));
  }

  void RobotClient::receive_settings(const FreeFleetDataSettingRequest::SharedPtr msg)
  {
    task_sequence[current_step]->receive_new_settings(msg->command, msg->value);
  }

  void RobotClient::receive_create_nfc_task(const FreeFleetDataCreateNfcRequest::ConstPtr msg)
  {
    std::vector<std::string> task_header= split(msg->task_id, '#');
    int task_id = stoi(task_header[0]);
    int step = stoi(task_header[1]);
    if (!prepare_new_action(msg->fleet_name, msg->robot_name, task_id))
    {
      return;
    }

    if (task_sequence.insert(std::make_pair(step, std::make_unique<NFCAction>(task_id, step, shared_from_this(), config, msg->user_id))).second)
    {
      //action could not be added
      return;
    }
    if(step==1)
    {
      task_sequence[1]->start();
    }

  }

  void RobotClient::receive_drawer_task(const FreeFleetDataDrawerRequest::ConstPtr msg)
  {
    std::vector<std::string> task_header= split(msg->task_id, '#');
    int task_id = stoi(task_header[0]);
    int step = stoi(task_header[1]);
  
    if(!prepare_new_action(msg->fleet_name, msg->robot_name, task_id))
    {
      return;
    }

    if (task_sequence.insert(std::make_pair(step, std::make_unique<DrawerAction>(DrawerAction(task_id, step, shared_from_this(), config, msg->drawer_id, msg->module_id, msg->e_drawer, msg->restricted)))).second)
    {
      //action could not be added
      return;
    }
  
    if(step==1)
    {
      task_sequence[1]->start();
    }
  }

  void RobotClient::receive_destination_task(const FreeFleetDataDestinationRequest::ConstPtr msg)
  {
    std::vector<std::string> task_header= split(msg->task_id, '#');
    int task_id = stoi(task_header[0]);
    int step = stoi(task_header[1]);
    if(!prepare_new_action(msg->fleet_name, msg->robot_name, task_id))
    {
      return;
    }

    if(task_sequence.insert( std::make_pair(step,std::make_unique<NavigationAction>(task_id, step, shared_from_this(), config, msg->destination.x, msg->destination.y, msg->destination.yaw))).second)
    {
      //action could not be added
      return;
    }
    
    if(step==1)
    {
      task_sequence[1]->start();
    }
  }

  bool RobotClient::prepare_new_action(std::string recipient_fleet, std::string recipient_robot, int task_id)
  {
    if(recipient_fleet == fleet_name && recipient_robot == robot_name)
    {
      return false;
    }

    if (task_id != this->task_id)
    {
      // publish_task_state("Task", "Abort", false);
      end_current_task(); 
      task_id = task_id;
    }

    return true;
  }

  void RobotClient::end_current_task()
  {
    
    empty_task_sequence();
    task_id = 0;
    publish_fleet_state();
  }

  void RobotClient::empty_task_sequence()
  {
    for (auto& pair : task_sequence) {
        pair.second.reset(); 
    }
    task_sequence.empty();
    current_step = 0;
  }

  void RobotClient::start_next_action()
  {
    auto it = task_sequence.find(current_step);
    if(it != task_sequence.end()) 
    {
      it++;
      if (it != task_sequence.end())
      { 
          current_step = it->first;
          // it->second->start();
      }
      // publish_task_state("Task", "Completed", true);
      end_current_task();
    }
    // publish_task_state("Task", "NotFound", true);
    end_current_task();
  }

  void RobotClient::publish_fleet_state()
  {
    update_robot_location();
    FreeFleetDataRobotInfo robot_state_msg = FreeFleetDataRobotInfo();

    robot_state_msg.name = robot_name;
    robot_state_msg.model = robot_model;
    robot_state_msg.task_id = task_id;

    robot_state_msg.location.level_name= "";
    robot_state_msg.location.nanosec=0;
    robot_state_msg.location.sec=0;
    robot_state_msg.location.x= current_x;
    robot_state_msg.location.y= current_y;
    robot_state_msg.location.yaw= current_yaw;

    robot_state_msg.battery_percent = -1;
    //robot_state_msg.mode = FreeFleetDataRobotMode();
    //robot_state_msg.path = std::vector<>;
    robot_info_publisher_->publish(robot_state_msg);
  }

  
  void RobotClient::update_robot_location()
  {
    geometry_msgs::msg::TransformStamped t;
    try {
          t = tf_buffer_->lookupTransform(tf_goal_frame, tf_start_frame, tf2::TimePointZero);
        } 
    catch (const tf2::TransformException & ex) 
      {
          RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", tf_goal_frame, tf_start_frame, ex.what());
          return;
      }

    current_yaw= quaternion_to_yaw(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    current_x = t.transform.translation.x;
    current_y = t.transform.translation.y;
  }

  std::vector<std::string> RobotClient::split( std::string input_text, char delimiter)
  {
    std::vector<std::string> result;
    int pos;
    while(pos=std::find(input_text.begin(), input_text.end(), delimiter)!=input_text.end())
    {
      result.push_back(input_text.substr(0, pos));
      input_text = input_text.substr(pos + 1);
    }
    result.push_back(input_text.substr(pos + 1));
  }
  
  double RobotClient::quaternion_to_yaw(double x, double y, double z, double w ) 
  {
    double sinYaw = 2.0 * (w * z + x * y);
    double cosYaw = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(sinYaw, cosYaw);
    return yaw * 180.0 / M_PI;    
  }

}