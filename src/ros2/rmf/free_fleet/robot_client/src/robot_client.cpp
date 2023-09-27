#include "robot_client/robot_client.hpp"

namespace rmf_robot_client
{

  RobotClient::RobotClient() : Node("robot_client")
  {
    init_param();
    start_receive_tasks();
    start_update_robot_state();
    drawer_list = std::make_shared<std::map<std::string, DrawerState>>();
    
    rclcpp::QoS qos_statemaschine = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos_statemaschine.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_statemaschine.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_statemaschine.avoid_ros_namespace_conventions(false);

    reset_simple_tree_publisher_ = this->create_publisher<StdMsgBool>(this->get_parameter("statemaschine_reset_simple_tree_topic").as_string(), qos_statemaschine);
  }

  void RobotClient::init_param()
  {
    this->declare_parameter("fleet_name", "ROBAST");
    this->declare_parameter("robot_name", "RB0");
    this->declare_parameter("robot_model", "Robast_Theron");
    this->declare_parameter("behavior_tree", "/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_to_pose_w_replanning_goal_patience_and_recovery.xml");
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("robot_frame_id", "robot_base_footprint");
    this->declare_parameter("robot_info_inteval", 1);//in seconds
    this->declare_parameter("nfc_timeout_interval", 3);// in minutes

    this->declare_parameter("statemaschine_open_drawer_topic", "/trigger_drawer_tree");
    this->declare_parameter("statemaschine_open_e_drawer_topic", "/trigger_electric_drawer_tree");
    this->declare_parameter("statemaschine_close_e_drawer_topic", "/close_drawer");
    this->declare_parameter("statemaschine_reset_simple_tree_topic", "/reset_simple_tree");
    this->declare_parameter("drawer_status_change_topic","/drawer_is_open" );

    this->declare_parameter("nfc_write_usertag_to_card_action_topic","/create_user");
    this->declare_parameter("nfc_on_off_switch_topic","/nfc_switch");
    this->declare_parameter("nfc_authenticated_user_topic","/authenticated_user");

    this->declare_parameter("fleet_communication_setting_topic", "/setting_request");
    this->declare_parameter("fleet_communication_create_nfc_topic", "/new_user_request");
    this->declare_parameter("fleet_communication_destination_topic", "/destination_request");
    this->declare_parameter("fleet_communication_drawer_topic", "/drawer_request");

    this->declare_parameter("fleet_communication_task_info_topic", "/task_state");
    this->declare_parameter("fleet_communication_robot_info_topic", "/robot_state"); 
    
    this->declare_parameter("nav2_navigation_to_pose_action_topic", "/navigate_to_pose");
    this->declare_parameter("robotnik_battery_level_topic", "/robot/battery_estimator/data");

     fleet_name=this->get_parameter("fleet_name").as_string();
     robot_name= this->get_parameter("robot_name").as_string();
     robot_model= this->get_parameter("robot_model").as_string();
     map_frame_id= this->get_parameter("map_frame_id").as_string();
     robot_frame_id= this->get_parameter("robot_frame_id").as_string();
     robot_info_inteval = this->get_parameter("robot_info_inteval").as_int();
  }

  void RobotClient::start_receive_tasks()
  {
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);

    write_nfc_card_request_subscriber_ = this->create_subscription<FreeFleetDataCreateNfcRequest>(this->get_parameter("fleet_communication_create_nfc_topic").as_string(), qos, std::bind(&RobotClient::receive_create_nfc_task, this,std::placeholders::_1));
    drawer_request_subscriber_ = this->create_subscription<FreeFleetDataDrawerRequest>(this->get_parameter("fleet_communication_drawer_topic").as_string(),qos, std::bind(&RobotClient::receive_drawer_task, this, std::placeholders::_1));
    navigation_request_subscriber_ = this->create_subscription<FreeFleetDataDestinationRequest>(this->get_parameter("fleet_communication_destination_topic").as_string(), qos, std::bind(&RobotClient::receive_destination_task, this, std::placeholders::_1));
    setting_subscriber_ = this->create_subscription<FreeFleetDataSettingRequest>(this->get_parameter("fleet_communication_setting_topic").as_string(), qos, std::bind(&RobotClient::receive_settings, this,std::placeholders::_1));
    drawer_status_subscriber_= this->create_subscription<DrawerStatus>( this->get_parameter("drawer_status_change_topic").as_string(), 10, std::bind(&RobotClient::receive_drawer_status, this, std::placeholders::_1));
    authentication_subscriber_=this->create_subscription<StdMsgInt>(this->get_parameter("nfc_authenticated_user_topic").as_string(), 10, std::bind(&RobotClient::receive_authenticated_user, this, std::placeholders::_1));
  }

  void RobotClient::start_update_robot_state()
  {
    rclcpp::QoS qos_fleet_communication = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos_fleet_communication.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_fleet_communication.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_fleet_communication.avoid_ros_namespace_conventions(false);

    rclcpp::QoS qos_robotnik = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos_robotnik.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    update_robot_location();

    //battery_status_sub_ = this->create_subscription<BatteryLevel>( this->get_parameter("robotnik_battery_level_topic").as_string(), qos_robotnik, std::bind(&RobotClient::update_battery_level, this, std::placeholders::_1));

    robot_info_publisher_ = this->create_publisher<FreeFleetDataRobotInfo>(this->get_parameter("fleet_communication_robot_info_topic").as_string(), qos_fleet_communication);
    publish_robot_info_timer_ = this->create_wall_timer(std::chrono::seconds(robot_info_inteval), std::bind(&RobotClient::publish_fleet_state, this)); 
    
  }

  void RobotClient::receive_create_nfc_task(const FreeFleetDataCreateNfcRequest::ConstPtr msg)
  {
     RCLCPP_INFO(this->get_logger(), "nfc_received");

    int step, task_id;
    if (!prepare_new_action(msg->task_id, msg->fleet_name, msg->robot_name, task_id, step))
    {
      return;
    }

    if (!task_sequence.insert(std::make_pair(step, std::make_unique<NFCAction>(task_id, step, shared_from_this(), msg->user_id))).second)
    {
      RCLCPP_ERROR(this->get_logger(),"Task %i, step %i could not be added to robot Queue", task_id, step);
      return;
    }
    if(step==1)
    {
      task_sequence.at(step)->start([this](bool successful) { end_current_action(successful); });
    }
  }

  void RobotClient::receive_destination_task(const FreeFleetDataDestinationRequest::ConstPtr msg)
  {
      RCLCPP_INFO(this->get_logger(), "destination_received");
    int task_id, step;

    if(!prepare_new_action(msg->task_id, msg->fleet_name, msg->robot_name, task_id, step))
    {
      return;
    }

    if(!task_sequence.insert( std::make_pair(step,std::make_unique<NavigationAction>(task_id, step, shared_from_this(), msg->destination.x, msg->destination.y, msg->destination.yaw))).second)
    {
      RCLCPP_ERROR(this->get_logger(),"Task %i, step %i could not be added to robot Queue", task_id, step);

      return;
    }
    
    if(step==1)
    {
      current_step = step;
      task_sequence.at(step)->start([this](bool successful)
                                    { end_current_action(successful); });
    }
  }

  void RobotClient::receive_drawer_task(const FreeFleetDataDrawerRequest::ConstPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "drawer_received");
    int step, task_id;
    if (!prepare_new_action(msg->task_id, msg->fleet_name, msg->robot_name, task_id, step))
    {
      RCLCPP_ERROR(this->get_logger(), "no valid task");
      return;
    }

    if (!task_sequence.insert(std::make_pair(step, std::make_unique<DrawerAction>(DrawerAction(task_id, step, shared_from_this(), drawer_list, msg->drawer_id, msg->module_id, msg->e_drawer, msg->authorized_user)))).second)
    {
      RCLCPP_ERROR(this->get_logger(),"Task %i, step %i could not be added to robot Queue", task_id, step);
      return;
    }
   
    if(step==1)
    {
      current_step = step;
      task_sequence.at(step)->start([this](int step) { end_current_action(step); });
    }
  }

  void RobotClient::receive_settings(const FreeFleetDataSettingRequest::SharedPtr msg)
  {
    //  global settings (reset tree,..)
    if(msg->command=="reset_tree")
    {
      StdMsgBool msg = StdMsgBool();
      msg.data = true;
      reset_simple_tree_publisher_->publish(msg);
    }
    else if (current_step==0|| current_step>task_sequence.size())
    {
       RCLCPP_INFO( this->get_logger(), "invalid request");
       return;
    }
    else
    {
      task_sequence[current_step]->receive_new_settings(msg->command, msg->value);//task settings
    }
    RCLCPP_INFO( this->get_logger(), "settings done");
  }

  void RobotClient::receive_drawer_status(const DrawerStatus::SharedPtr msg)
  {
    
    std::string drawer_ref = std::to_string(msg->drawer_address.module_id) + '#' + std::to_string(msg->drawer_address.drawer_id);
    if (current_step==0 ||current_step>task_sequence.size())
    {
      RCLCPP_ERROR( this->get_logger(), "External triggered, Drawer %s state received (Drawer_id: %i, Module_id: %i ).",
      msg->drawer_is_open? "open": "close",msg->drawer_address.drawer_id, msg->drawer_address.module_id);
      return;
    }
    else
    { 
      task_sequence[current_step]->receive_new_settings("DrawerState", { drawer_ref + "#"+ (msg->drawer_is_open?"Opened":"Closed") });
    }      
    
    if (drawer_list->count(drawer_ref))
    {
      drawer_list->at(drawer_ref).is_open = msg->drawer_is_open;
    }
    else
    {
      RCLCPP_ERROR( this->get_logger(), "External triggered Drawer %s state received (Drawer_id: %i, Module_id: %i ).",
       msg->drawer_is_open? "open": "close",msg->drawer_address.drawer_id, msg->drawer_address.module_id);
    }                    
    
  } 

  void RobotClient::receive_authenticated_user(const StdMsgInt::SharedPtr msg)
  {
    task_sequence[current_step]->receive_new_settings("drawer", {"Authenticated_user", std::to_string(msg->data)});
  }

  //ToDo only works after the topic is bridged properly
  // void RobotClient::receive_battery_status(const BatteryLevel::ConstPtr msg)
  // {
  //   current_battery_level= msg->level;
  // }

  bool RobotClient::prepare_new_action(std::string task_def, std::string recipient_fleet, std::string recipient_robot,int& new_task_id, int& new_step )
  {  
    if(!(recipient_fleet == fleet_name && recipient_robot == robot_name))
    {
      return false;
    }

    std::vector<std::string> task_header= split(task_def, '#');
    try
    {
      new_task_id = stoi(task_header[0]);
      new_step = stoi(task_header[1]);
    }
    catch (const std::invalid_argument &e)
    {
       RCLCPP_ERROR(this->get_logger(),"Task ID %s, is not in the correct format", task_def.c_str());
       return false;
    }

    if (task_id != new_task_id && task_id!=0)
    {
      // publish_task_state("Task", "Abort", false);
      task_sequence[current_step]->cancel();
      end_current_task(); 
    }
    task_id = new_task_id;
    return true;
  }

  void RobotClient::end_current_action(int comletted_step)
  {
    RCLCPP_INFO(this->get_logger(),"End Action nr %i", current_step);
    if(comletted_step==current_step)
    {
    start_next_action();
    }
    return;
  }

  void RobotClient::start_next_action()
  {
    RCLCPP_INFO(this->get_logger(),"start_next_action %i",task_id);
    auto it = task_sequence.find(current_step);
    if(it == task_sequence.end()) 
    {
      //publish_task_state("Task", "NotFound", true);
      RCLCPP_INFO(this->get_logger(), "Task: %i Step: %i| Not Found in Queue", task_id, current_step);
      end_current_task();
      return;
    }

    it++;
    if (it != task_sequence.end())
    { 
      current_step = it->first;
      it->second->start([this](int step) { end_current_action(step); });
    }
    // publish_task_state("Task", "Completed", true);
    end_current_task();
    return;
  }

  void RobotClient::end_current_task()
  {
    RCLCPP_INFO(this->get_logger(),"End task %i",task_id);
    empty_task_sequence();
    task_id = 0;
    current_step = 0;
   
    publish_fleet_state();
  }

  void RobotClient::empty_task_sequence()
  {
    for (auto& pair : task_sequence) {
        pair.second.reset(); 
    }
    task_sequence.clear();
    current_step = 0;
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

    robot_state_msg.battery_percent = current_battery_level;
    //robot_state_msg.mode = FreeFleetDataRobotMode();
    //robot_state_msg.path = std::vector<>;
    robot_info_publisher_->publish(robot_state_msg);
  }

  void RobotClient::update_robot_location()
  {
    geometry_msgs::msg::TransformStamped t;
    try {
          t = tf_buffer_->lookupTransform(map_frame_id, robot_frame_id, tf2::TimePointZero);
          // rclcpp::Time now = this->get_clock()->now();
          // t = tf_buffer_->lookupTransform( map_frame_id, robot_frame_id, now,rclcpp::Duration::from_seconds(0.05));
        } 
    catch (const tf2::TransformException & ex) 
      {
          //RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", map_frame_id.c_str(), robot_frame_id.c_str(), ex.what());
          return;
      }

      tf2::Quaternion quat_tf;
      geometry_msgs::msg::Quaternion quat_msg = t.transform.rotation;
      tf2::fromMsg(quat_msg, quat_tf);
      double p{}, y{};
      tf2::Matrix3x3 m(quat_tf);
      m.getRPY(current_yaw, p, y);

      current_x = t.transform.translation.x;
      current_y = t.transform.translation.y;
  }

  std::vector<std::string> RobotClient::split( std::string input_text, char delimiter)
  {
    std::vector<std::string> result;
    std::istringstream iss(input_text);
    std::string token;

    while (std::getline(iss, token, delimiter)) 
    {
      result.push_back(token);
    }
    
    return result;
  }
  
  double RobotClient::quaternion_to_yaw(double x, double y, double z, double w ) 
  {
    double sinYaw = 2.0 * (w * z + x * y);
    double cosYaw = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(sinYaw, cosYaw);
    yaw= yaw * 180.0 / M_PI;
    return yaw;
  }

}
