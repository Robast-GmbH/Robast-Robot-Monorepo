#include "robot_client/drawer_task.hpp"

namespace rmf_robot_client
{
  DrawerTask::DrawerTask(TaskId task_id,
                         std::shared_ptr<rclcpp::Node> ros_node,
                         std::shared_ptr<std::map<std::string, DrawerState>> drawer_states,
                         DrawerState used_drawer)
      : BaseTask(task_id, ros_node)
  {
    this->_selected_drawer = std::make_unique<DrawerState>(used_drawer);
    this->_drawers = drawer_states;

    _nfc_timeout_interval = ros_node->get_parameter("nfc_timeout_interval").as_int();

    /////////////////////////////////////////////////////////////////////

    //  controll drawer
    _trigger_open_e_drawer_publisher = ros_node->create_publisher<DrawerAddress>(
        ros_node->get_parameter("statemaschine_open_e_drawer_topic").as_string(),
        QoSConfig::get_statemaschine_drawer_qos());
    _trigger_open_drawer_publisher = ros_node->create_publisher<DrawerAddress>(
        ros_node->get_parameter("statemaschine_open_drawer_topic").as_string(),
        QoSConfig::get_statemaschine_drawer_qos());
    _trigger_close_e_drawer_publisher = ros_node->create_publisher<DrawerAddress>(
        ros_node->get_parameter("statemaschine_close_e_drawer_topic").as_string(),
        QoSConfig::get_statemaschine_drawer_qos());
    _nfc_on_off_publisher = ros_node->create_publisher<StdMsgBool>(
        ros_node->get_parameter("nfc_on_off_switch_topic").as_string(), QoSConfig::get_nfc_controll_qos());
  }

  bool DrawerTask::start(std::function<void(int)> next_task_callback)
  {
    BaseTask::start(next_task_callback);
    RCLCPP_INFO(ros_node_->get_logger(), "start drawer_task");
    open_drawer_task(*_selected_drawer);
    return true;
  }

  void DrawerTask::open_drawer_task(DrawerState selected_drawer)
  {
    std::string drawer_ref = selected_drawer.drawer_ref.get_ref_string();

    if (_drawers->count(drawer_ref))
    {
      _selected_drawer = std::make_unique<DrawerState>(_drawers->at(drawer_ref));
    }
    else
    {
      DrawerState NewDrawer = DrawerState(selected_drawer);
      _drawers->insert(std::pair(drawer_ref, NewDrawer));
      _selected_drawer = std::make_unique<DrawerState>(NewDrawer);
    }

    if (check_user_permission(_active_user, _selected_drawer->authorised_users))
    {
      open_drawer(*_selected_drawer);
    }
    else
    {
      // scan for user
      RCLCPP_INFO(ros_node_->get_logger(), "start authentification");
      start_authentication_scan();
    }
  }

  void DrawerTask::open_drawer(DrawerState selected_drawer)
  {
    std::string drawer_ref = selected_drawer.drawer_ref.get_ref_string();
    if (selected_drawer.drawer_ref == _selected_drawer->drawer_ref)
    {
      // set new lock
      _drawers->at(drawer_ref).authorised_users = selected_drawer.authorised_users;
    }

    DrawerAddress drawer_msg = DrawerAddress();
    drawer_msg.drawer_id = selected_drawer.drawer_ref.drawer_id;
    drawer_msg.module_id = selected_drawer.drawer_ref.module_id;
    if (_drawers->at(drawer_ref).is_e_drawer)
    {
      _trigger_open_e_drawer_publisher->publish(drawer_msg);
      publish_task_state("DrawerState", drawer_ref + "#Opened", false);
    }
    else
    {
      _trigger_open_drawer_publisher->publish(drawer_msg);
      publish_task_state("DrawerState", drawer_ref + "#Unlocked", false);
    }
    _drawers->at(drawer_ref).is_open = true;
    _selected_drawer.release();
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
      DrawerRef drawer = DrawerRef(std::stoi(value[1]), std::stoi(value[2]));
      close_drawer(drawer);
      publish_task_state("DrawerState", drawer.get_ref_string() + "#Closed", false);
    }
    else if (value[0] == "Opend")
    {
      open_drawer_task(DrawerState(DrawerRef(std::stoi(value[1]), std::stoi(value[2])), value[3] == "E-drawer"));
    }
    else if (value[0] == "Completed")
    {
      publish_task_state("Task", "Done", true);
      task_done(true);
    }
    else if (value[0] == "Authenticated_user")
    {
      if (_selected_drawer != NULL)
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
    _active_user = user_id;

    if (check_user_permission(_active_user, _selected_drawer->authorised_users))
    {
      end_authentication_scan();
      open_drawer(*_selected_drawer);
      return;
    }
  }

  void DrawerTask::close_drawer(DrawerRef selected_drawer)
  {
    DrawerAddress drawer_msg = DrawerAddress();
    drawer_msg.drawer_id = selected_drawer.drawer_id;
    drawer_msg.module_id = selected_drawer.module_id;
    _trigger_close_e_drawer_publisher->publish(drawer_msg);
    publish_close_drawer_status(selected_drawer);
  }

  void DrawerTask::publish_close_drawer_status(DrawerRef selected_drawer)
  {
    std::string drawer_ref = selected_drawer.get_ref_string();
    if (_drawers->count(drawer_ref))
    {
      publish_task_state("DrawerState", drawer_ref + "#Closed", false);
      _drawers->at(drawer_ref).is_open = false;
      RCLCPP_INFO(
          ros_node_->get_logger(), "drawer(%i, %i, Closed)", selected_drawer.module_id, selected_drawer.drawer_id);
    }
  }

  void DrawerTask::start_authentication_scan()
  {
    StdMsgBool msg;
    msg.data = true;
    _nfc_on_off_publisher->publish(msg);
    _nfc_timeout_timer = ros_node_->create_wall_timer(std::chrono::minutes(_nfc_timeout_interval),
                                                      std::bind(&DrawerTask::nfc_timeout, this));
  }

  void DrawerTask::nfc_timeout()
  {
    end_authentication_scan();
    _selected_drawer.release();
    RCLCPP_WARN(ros_node_->get_logger(), "NFC reader timeout");
  }

  void DrawerTask::end_authentication_scan()
  {
    StdMsgBool off_msg;
    off_msg.data = false;
    _nfc_on_off_publisher->publish(off_msg);
    _nfc_timeout_timer->cancel();
  }

  bool DrawerTask::all_drawers_closed()
  {
    return std::all_of(_drawers->begin(),
                       _drawers->end(),
                       [](const std::pair<std::string, DrawerState>& pair)
                       {
                         return pair.second.is_open;
                       });
  }

  void DrawerTask::task_done(bool is_completed)
  {
    _trigger_open_drawer_publisher.reset();
    _trigger_open_e_drawer_publisher.reset();
    _trigger_close_e_drawer_publisher.reset();
    _nfc_on_off_publisher.reset();
    finish_task_(task_id_.step);
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

  bool DrawerTask::check_user_permission(int user, std::vector<u_int16_t> authorised_user_list)
  {
    return authorised_user_list.size() == 0 ||
           std::find(authorised_user_list.begin(), authorised_user_list.end(), user) != authorised_user_list.end();
  }
}   // namespace rmf_robot_client