#include "robot_client/drawer_task.hpp"

namespace rmf_robot_client
{
  DrawerTask::DrawerTask(int task_id,
                         int step,
                         std::shared_ptr<rclcpp::Node> ros_node,
                         std::shared_ptr<std::map<std::string, DrawerState>> drawer_states,
                         int drawer_id,
                         int module_id,
                         bool is_edrawer,
                         std::vector<uint16_t> autorised_users)
      : BaseTask(task_id, step, ros_node)
  {
    this->drawer_id_ = drawer_id;
    this->module_id_ = module_id;
    this->is_e_drawer_ = is_edrawer;
    this->authorised_users_ = autorised_users;
    this->drawers_ = drawer_states;

    nfc_timeout_interval_ = ros_node_->get_parameter("nfc_timeout_interval").as_int();

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    //  controll drawer
    trigger_open_e_drawer_publisher_ = ros_node->create_publisher<DrawerAddress>(
        ros_node_->get_parameter("statemaschine_open_e_drawer_topic").as_string(), qos);
    trigger_open_drawer_publisher_ = ros_node->create_publisher<DrawerAddress>(
        ros_node_->get_parameter("statemaschine_open_drawer_topic").as_string(), qos);
    trigger_close_e_drawer_publisher_ = ros_node->create_publisher<DrawerAddress>(
        ros_node_->get_parameter("statemaschine_close_e_drawer_topic").as_string(), qos);
    nfc_on_off_publisher_ =
        ros_node->create_publisher<StdMsgBool>(ros_node_->get_parameter("nfc_on_off_switch_topic").as_string(), qos);
  }

  bool DrawerTask::start(std::function<void(int)> next_task_callback)
  {
    BaseTask::start(next_task_callback);
    RCLCPP_INFO(ros_node_->get_logger(), "start drawer_task");
    open_drawer_task(module_id_, drawer_id_, is_e_drawer_);
    return true;
  }

  void DrawerTask::open_drawer_task(int target_module_id, int target_drawer_id, bool target_is_edrawer)
  {
    std::string drawer_ref = get_drawer_ref(target_module_id, target_drawer_id);

    if (drawers_->count(drawer_ref))
    {
      selected_drawer_ = std::make_unique<DrawerState>(drawers_->at(drawer_ref));
    }
    else
    {
      DrawerState NewDrawer =
          DrawerState(target_module_id, target_drawer_id, target_is_edrawer, false, std::vector<u_int16_t>());
      drawers_->insert(std::pair(drawer_ref, NewDrawer));
      selected_drawer_ = std::make_unique<DrawerState>(NewDrawer);
    }

    if (check_user_permission(active_user_, selected_drawer_->authorised_users))
    {
      open_drawer(selected_drawer_->module_id, selected_drawer_->drawer_id);
    }
    else
    {
      // scan for user
      RCLCPP_INFO(ros_node_->get_logger(), "start authentification");
      start_authentication_scan();
    }
  }

  void DrawerTask::open_drawer(int target_module_id, int target_drawer_id)
  {
    std::string drawer_ref = get_drawer_ref(module_id_, drawer_id_);
    if (target_module_id == module_id_ && target_drawer_id == drawer_id_)
    {
      // set new lock
      drawers_->at(drawer_ref).authorised_users = authorised_users_;
    }

    DrawerAddress drawer_msg = DrawerAddress();
    drawer_msg.drawer_id = target_drawer_id;
    drawer_msg.module_id = target_module_id;
    if (drawers_->at(drawer_ref).is_e_drawer)
    {
      trigger_open_e_drawer_publisher_->publish(drawer_msg);
      publish_task_state("DrawerState", drawer_ref + "#Opened", false);
    }
    else
    {
      trigger_open_drawer_publisher_->publish(drawer_msg);
      publish_task_state("DrawerState", drawer_ref + "#Unlocked", false);
    }
    drawers_->at(drawer_ref).is_open = true;
    selected_drawer_.release();
  }

  bool DrawerTask::receive_new_settings(std::string command, std::vector<std::string> value)
  {
    if (BaseTask::receive_new_settings(command, value))
    {
      return true;
    }

    if (command != "drawer")
    {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "Command %s/%s is unknown for this task %s",
                   command.c_str(),
                   value[0].c_str(),
                   get_type().c_str());
      return false;
    }

    if (value[0] == "Closed")
    {
      close_drawer(std::stoi(value[1]), std::stoi(value[2]));
      publish_task_state("DrawerState", value[1] + "#" + value[2] + "#Closed", false);
    }
    else if (value[0] == "Opend")
    {
      open_drawer_task(std::stoi(value[1]), std::stoi(value[2]), value[3] == "E-drawer");
    }
    else if (value[0] == "Completed")
    {
      publish_task_state("Task", "Done", true);
      task_done(true);
    }
    else if (value[0] == "Authenticated_user")
    {
      if (selected_drawer_ != NULL)
      {
        check_scant_user(std::stoi(value[1]));
      }
    }
    else
    {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "Command %s/%s is unknown for task %s",
                   command.c_str(),
                   value[0].c_str(),
                   get_type().c_str());
    }
    return true;
  }

  void DrawerTask::check_scant_user(int user_id)
  {
    active_user_ = user_id;

    if (check_user_permission(active_user_, selected_drawer_->authorised_users))
    {
      end_authentication_scan();
      open_drawer(selected_drawer_->module_id, selected_drawer_->drawer_id);
      return;
    }
  }

  void DrawerTask::close_drawer(int module_id, int drawer_id)
  {
    DrawerAddress drawer_msg = DrawerAddress();
    drawer_msg.drawer_id = drawer_id;
    drawer_msg.module_id = module_id;
    trigger_close_e_drawer_publisher_->publish(drawer_msg);
    publish_close_drawer_status(module_id, drawer_id);
  }

  void DrawerTask::publish_close_drawer_status(int module_id, int drawer_id)
  {
    std::string drawer_ref = get_drawer_ref(module_id, drawer_id);
    if (drawers_->count(drawer_ref))
    {
      publish_task_state("DrawerState", drawer_ref + "#Closed", false);
      drawers_->at(drawer_ref).is_open = false;
      RCLCPP_INFO(ros_node_->get_logger(), "drawer(%i, %i, Closed)", module_id, drawer_id);
    }
  }

  void DrawerTask::start_authentication_scan()
  {
    StdMsgBool msg;
    msg.data = true;
    nfc_on_off_publisher_->publish(msg);
    nfc_timeout_timer_ = ros_node_->create_wall_timer(std::chrono::minutes(nfc_timeout_interval_),
                                                      std::bind(&DrawerTask::nfc_timeout, this));
  }

  void DrawerTask::nfc_timeout()
  {
    end_authentication_scan();
    selected_drawer_.release();
    RCLCPP_WARN(ros_node_->get_logger(), "NFC reader timeout");
  }

  void DrawerTask::end_authentication_scan()
  {
    StdMsgBool off_msg;
    off_msg.data = false;
    nfc_on_off_publisher_->publish(off_msg);
    nfc_timeout_timer_->cancel();
  }

  bool DrawerTask::all_drawers_closed()
  {
    return std::all_of(drawers_->begin(),
                       drawers_->end(),
                       [](const std::pair<std::string, DrawerState>& pair)
                       {
                         return pair.second.is_open;
                       });
  }

  void DrawerTask::task_done(bool is_completed)
  {
    trigger_open_drawer_publisher_.reset();
    trigger_open_e_drawer_publisher_.reset();
    trigger_close_e_drawer_publisher_.reset();
    nfc_on_off_publisher_.reset();
    finish_task_(step_);
    publish_task_state("DrawerState", "Drawertask_completed", is_completed);
  }

  bool DrawerTask::cancel()
  {
    if (all_drawers_closed())
    {
      publish_task_state("Canceld", "", true);
      task_done(false);
      return true;
    }
    return false;
  }

  std::string DrawerTask::get_type()
  {
    return "DRAWER_TASK";
  }

  std::string DrawerTask::get_drawer_ref(int module_id, int drawer_id)
  {
    return std::to_string(module_id) + "#" + std::to_string(drawer_id);
  }

  bool DrawerTask::check_user_permission(int user, std::vector<u_int16_t> authorised_user_list)
  {
    return authorised_user_list.size() == 0 ||
           std::find(authorised_user_list.begin(), authorised_user_list.end(), user) != authorised_user_list.end();
  }
}   // namespace rmf_robot_client