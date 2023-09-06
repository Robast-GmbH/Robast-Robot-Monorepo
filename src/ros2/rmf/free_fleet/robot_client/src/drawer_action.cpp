#include "robot_client/drawer_action.hpp"

namespace rmf_robot_client
{
  DrawerAction::DrawerAction(int task_id, int step, std::shared_ptr<rclcpp::Node> ros_node, std::map<std::string,std::string> config, std::shared_ptr<std::map<std::string, DrawerStatus>> drawer_states, int drawer_id, int module_id, bool is_edrawer, std::vector<std::string> redistricted_user):Action(task_id, step, ros_node, config)
  {
    this->drawer_id = drawer_id;
    this->module_id = module_id;
    this->is_edrawer = is_edrawer;
    this->redistricted_user = redistricted_user;
    this->drawers = drawer_states;

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);

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
  }

  bool DrawerAction::start(std::function<void(bool)> next_action_callback)
  {
    RCLCPP_INFO(ros_node->get_logger(), "start drawer_action");
    finish_action = next_action_callback;
    // TODo handle locking
    // access controll
    // set new lock

    open_drawer(this->module_id, this->drawer_id);
    
    return true;
  }

  bool DrawerAction::open_drawer(int module_id, int drawer_id)
  {
    DrawerAddress drawer_msg = DrawerAddress();
    drawer_msg.drawer_id = drawer_id;
    drawer_msg.module_id = module_id;

    trigger_open_drawer_publisher_->publish(drawer_msg);

    std::string drawer_ref =module_id+"#"+drawer_id;
    if(drawers->count(drawer_ref)>0)
    {
      drawers->at(drawer_ref).is_opened==true;
    }
    else 
    {
      drawers->at(drawer_ref) = DrawerStatus(module_id, drawer_id, true, "");
    }
    publish_task_state("DrawerState", drawer_ref+"#Opened" , false);
    return true;
  }

  bool DrawerAction::close_drawer(int module_id, int drawer_id)
  {
      std::string drawer_ref =module_id+"#"+drawer_id;
      if(is_edrawer && drawers->at(drawer_ref).is_opened)
      {
        
        DrawerAddress drawer_msg = DrawerAddress();
        drawer_msg.drawer_id = this->drawer_id;
        drawer_msg.module_id = this->module_id;
        //handle list of open drawers
        trigger_close_drawer_publisher_->publish(drawer_msg);
        publish_task_state("DrawerState", drawer_ref + "#Closed", false);
        drawers->at(drawer_ref).is_opened==false;
      }
      return false;
  }
      //   def end_drawer_task(self):
      //       self.publish_task_state("DrawerAction", "Finished", True)

      //  def set_drawer_lock(self, module_id:int, drawer_id:int, restriction):
      //       if(len(restriction)>0):
      //           user_dict={}
      //           for user in restriction:
      //               entry= user.split(':')
      //               user_dict.update({entry[0] : entry[2]})
      //           self.locked_drawers.append(drawer(module_id= module_id, drawer_id= drawer_id, locked_for=user_dict))

  bool DrawerAction::cancel()
  {
    publish_task_state("Canceld", "", true);
    finish_action(false);
    return true;
  }
  
  std::string DrawerAction::get_type()
  {
    return "DRAWER_ACTION";
  }

  bool DrawerAction::receive_new_settings(std::string command, std::string value)
  {   
    if(command == "Closed")
    {
        publish_task_state("DrawerState", value + "#Closed", false);
    }
    else if(command == "Completed")
    {
      finish_action(true);
      publish_task_state("Action", "Done", true);
    }

    else
    {
        RCLCPP_ERROR(ros_node->get_logger(),"Command %s is unknown for action %s", command, get_type().c_str());
    }
  }

  bool DrawerAction::all_drawers_closed()
  {
  return  std::all_of(drawers->begin(), drawers->end(), [](const std::pair<std::string, DrawerStatus> &pair)
                                  { return pair.second.is_opened; });
  }
}
