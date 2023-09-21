#include "robot_client/drawer_action.hpp"

namespace rmf_robot_client
{
  DrawerAction::DrawerAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config, std::shared_ptr<std::map<std::string, DrawerState>> drawer_states, int drawer_id, int module_id, bool is_edrawer, std::vector<uint16_t> autorised_user):Action(task_id, step, ros_node, config)
  {
    this->drawer_id = drawer_id;
    this->module_id = module_id;
    this->is_edrawer = is_edrawer;
    this->autorised_user = autorised_user;
    this->drawers = drawer_states;

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      qos.avoid_ros_namespace_conventions(false);
      drawer_status_subscriber_= ros_node->create_subscription<DrawerStatus>( "/drawer_is_open", 10, std::bind(&DrawerAction::receive_drawer_status, this, std::placeholders::_1));
   
     //controll drawer
    if(is_edrawer)
    {
      trigger_open_drawer_publisher_ = ros_node->create_publisher<DrawerAddress>(config["statemaschine_open_e_drawer_topic"], qos);
    }
    else
    {
      trigger_open_drawer_publisher_ = ros_node->create_publisher<DrawerAddress>(config["statemaschine_open_drawer_topic"], qos);
    }
    trigger_close_drawer_publisher_ = ros_node->create_publisher<DrawerAddress>(config["statemaschine_close_e_drawer_topic"], qos);
    nfc_on_off_publisher_ = ros_node->create_publisher<StdMsgBool>(config["nfc_on_off_switch_topic"], qos);
  }

  void DrawerAction::receive_drawer_status(const DrawerStatus::SharedPtr msg)
  {
    RCLCPP_INFO(ros_node->get_logger(), "closed drawer_action");
    if(msg->drawer_is_open==false)
    publish_drawer_closed(msg->drawer_address.module_id, msg->drawer_address.drawer_id);
  }

  bool DrawerAction::start(std::function<void(int)> next_action_callback)
  {
    RCLCPP_INFO(ros_node->get_logger(), "start drawer_action");
    finish_action = next_action_callback;
    open_drawer_action(module_id, drawer_id);
    return true;
  }

  void  DrawerAction::open_drawer_action(int target_module_id, int target_drawer_id)
  {
    std::string drawer_ref = get_drawer_ref(module_id, drawer_id);

    if (drawers->count(drawer_ref) > 0)
    {
      selected_drawer= std::make_unique<DrawerState>(drawers->at(drawer_ref));
    }
    else
    {
      DrawerState NewDrawer= DrawerState(target_module_id, target_drawer_id, false, std::vector<u_int16_t>());  
      drawers->insert(std::pair(drawer_ref, NewDrawer));
      selected_drawer = std::make_unique<DrawerState>(NewDrawer);
      selected_drawer->authorised_users = std::vector<u_int16_t>();
    }

    if(selected_drawer->authorised_users.size()>0 )
    {
      //scan for user
      start_authentication_scan();
    }
    else
    {
      open_drawer(target_module_id, target_drawer_id);
    }
  }

  void DrawerAction::open_drawer(int target_module_id, int target_drawer_id)
  {
    std::string drawer_ref = get_drawer_ref(module_id, drawer_id);
    if (target_module_id == module_id && target_drawer_id == drawer_id)
    {
      //set new lock
      drawers->at(drawer_ref).authorised_users = autorised_user; 
    }

    DrawerAddress drawer_msg = DrawerAddress();
    drawer_msg.drawer_id = target_drawer_id;
    drawer_msg.module_id = target_module_id;
    trigger_open_drawer_publisher_->publish(drawer_msg);
    
    drawers->at(drawer_ref).is_opened==true;
   
    publish_task_state("DrawerState", drawer_ref+"#Opened" , false);
  }

  void DrawerAction::start_authentication_scan()
  {
    authentication_subscriber_=ros_node->create_subscription<StdMsgInt>(config["nfc_authenticated_user_topic"], 10, std::bind(&DrawerAction::check_scant_user, this, std::placeholders::_1));
    StdMsgBool msg;
    msg.data = true;
    nfc_on_off_publisher_->publish(msg);
   // nfc_reading_timer = ros_node->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&DrawerAction::end_authentication_scan, this));
    
  }
  
  void DrawerAction::end_authentication_scan(bool successfull= false)
  {
      StdMsgBool off_msg;
      off_msg.data = false;
      nfc_on_off_publisher_->publish(off_msg);
      nfc_reading_timer->cancel();
  }

  void DrawerAction::check_scant_user(const StdMsgInt& msg) 
  {
        for(int authorisised_user_id : selected_drawer->authorised_users)
        {
          if(authorisised_user_id == msg.data)//fix user msg to ID
          {
            end_authentication_scan(true);
            open_drawer(selected_drawer->module_id, selected_drawer->drawer_id);
            return;
          }
        }
  }

  void  DrawerAction::close_drawer(int module_id, int drawer_id)
  {
    std::string drawer_ref = get_drawer_ref(module_id, drawer_id);
   
      DrawerAddress drawer_msg = DrawerAddress();
      drawer_msg.drawer_id = drawer_id;
      drawer_msg.module_id = module_id;
      trigger_close_drawer_publisher_->publish(drawer_msg);
      publish_drawer_closed(module_id, drawer_id);
    
  }

  void DrawerAction::publish_drawer_closed(int module_id, int drawer_id)
  {
    std::string drawer_ref = get_drawer_ref(module_id, drawer_id);
    publish_task_state("DrawerState", drawer_ref + "#Closed", false);
    drawers->at(drawer_ref).is_opened=false;
    RCLCPP_INFO( ros_node->get_logger(), "drawer(%i, %i, Closed)",module_id, drawer_id);
  }
  std::string DrawerAction::get_drawer_ref(int module_id, int drawer_id)
  {
    return std::to_string(module_id) + "#" + std::to_string(drawer_id);
  }

  bool DrawerAction::cancel()
  {
    if(all_drawers_closed())
    {
      publish_task_state("Canceld", "", true);
      finish_action(false);
      return true;
    }
      return false;
  }
  
  std::string DrawerAction::get_type()
  {
    return "DRAWER_ACTION";
  }

  bool DrawerAction::receive_new_settings(std::string command, std::vector<std::string> value)
  {   
    if(command!= "drawer")
    {
      RCLCPP_ERROR(ros_node->get_logger(),"Command %s/%s is unknown for this action %s", command.c_str(), value[0].c_str(), get_type().c_str());
      return false;
    }

    if(value[0]=="Closed")
    {
       close_drawer(std::stoi(value[1]), std::stoi(value[2]));
      RCLCPP_ERROR(ros_node->get_logger(),"close drawer setting");
      publish_task_state("DrawerState", value[1] + "#" + value[2] + "#Closed", false);
    }
    else if(value[0]=="Opend")
    {
      open_drawer_action(std::stoi(value[1]), std::stoi(value[2]));
    }
    else if(value[0] == "Completed")
    {
      publish_task_state("Action", "Done", true);
      finish_action(step);
    }
    else
    {
        RCLCPP_ERROR(ros_node->get_logger(),"Command %s/%s is unknown for action %s", command.c_str(), value[0].c_str(), get_type().c_str());
    }
    return true;
  }

  bool DrawerAction::all_drawers_closed()
  {
    return  std::all_of(drawers->begin(), drawers->end(), [](const std::pair<std::string, DrawerState> &pair)
                                  { return pair.second.is_opened; });
  }
}