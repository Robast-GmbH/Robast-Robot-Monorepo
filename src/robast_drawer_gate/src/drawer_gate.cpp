#include "robast_drawer_gate/drawer_gate.hpp"


// For DEBUGGING purposes this is the action send_goal command:
// ros2 action send_goal /control_drawer robast_ros2_msgs/action/DrawerUserAccess "{drawer: {drawer_controller_id: 1, drawer_id: 1}, state: 1}"

namespace robast_drawer_gate
{
  DrawerGate::DrawerGate() : Node("robast_drawer_gate")
  {
    this->drawer_gate_server = rclcpp_action::create_server<DrawerUserAccess>(
      this,
      "control_drawer",
      std::bind(&DrawerGate::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrawerGate::cancel_callback, this, std::placeholders::_1),
      std::bind(&DrawerGate::accepted_callback, this, std::placeholders::_1));

    this->shelf_setup_info_service = this->create_service<ShelfSetupInfo>(
      "shelf_setup_info",
      std::bind(&DrawerGate::provide_shelf_setup_info_callback, this, std::placeholders::_1, std::placeholders::_2));

    this->setup_serial_can_ubs_converter();
    // When the USB-CAN Adapter isn't sending CAN messages, the default state should be the open can channel to enable receiving CAN messages
    this->open_can_channel();
    this->serial_helper.close_serial();
  }

  //TODO: Dekonstruktor

  rclcpp_action::GoalResponse DrawerGate::goal_callback(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DrawerUserAccess::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    // (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerGate::cancel_callback(
    const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerGate::accepted_callback(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DrawerGate::handle_drawer_user_access, this, std::placeholders::_1), goal_handle}.detach();
  }

  void DrawerGate::update_drawer_status(std::vector<robast_can_msgs::CanMessage> drawer_feedback_can_msgs)
  {
    std::lock_guard<std::mutex> scoped_lock(drawer_status_mutex); // the lock will be released after the scope of this function
    for (uint8_t i = 0; i < drawer_feedback_can_msgs.size(); i++)
    {
      if (drawer_feedback_can_msgs.at(i).id == CAN_ID_DRAWER_FEEDBACK)
      {
        uint32_t drawer_controller_id = drawer_feedback_can_msgs.at(i).can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data;
        drawer_status drawer_status = {};
        drawer_status.is_endstop_switch_1_pushed = drawer_feedback_can_msgs.at(i).can_signals.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).data == 1;
        drawer_status.is_lock_switch_1_pushed = drawer_feedback_can_msgs.at(i).can_signals.at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).data == 1;
        drawer_status.is_endstop_switch_2_pushed = drawer_feedback_can_msgs.at(i).can_signals.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).data == 1;
        drawer_status.is_lock_switch_2_pushed = drawer_feedback_can_msgs.at(i).can_signals.at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).data == 1;
        drawer_status.received_initial_drawer_status = true;
        this->drawer_status_by_drawer_controller_id[drawer_controller_id] = drawer_status;
      }
    }
    cv.notify_one(); // Notify the waiting thread in the open_drawer function to check if condition is now satisfied 
  }

  void DrawerGate::timer_callback(void)
  {
    this->update_drawer_status_from_can();
  }

  void DrawerGate::update_drawer_status_from_can(void)
  {
    this->setup_serial_can_ubs_converter();
    this->open_can_channel(); 
    std::string serial_read_ascii_command;
    uint16_t num_of_received_bytes = this->serial_helper.read_serial(&serial_read_ascii_command, 200);

    std::vector<robast_can_msgs::CanMessage> received_can_msgs = robast_can_msgs::decode_multiple_ascii_commands_into_can_messages(serial_read_ascii_command, CAN_ID_DRAWER_FEEDBACK, CAN_DLC_DRAWER_FEEDBACK, can_db.can_messages);

    if (cleared_serial_buffer_from_old_can_msgs) 
    {
      this->update_drawer_status(received_can_msgs);
    }
    else
    {
      // Dump only the very first received can msgs because they might contain old data in the buffer
      cleared_serial_buffer_from_old_can_msgs = true;
    }

    this->serial_helper.close_serial();
  }

  void DrawerGate::provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DrawerGate::provide_shelf_setup_info, this, std::placeholders::_1, std::placeholders::_2), request, response}.detach();
  }

  void DrawerGate::provide_shelf_setup_info(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    //TODO: This should actually be done automatically by polling all drawer_controller on the CAN bus

    robast_ros2_msgs::msg::Box box_10x40x1;
    box_10x40x1.x = DRAWER_INSIDE_WIDTH_10x40x1;
    box_10x40x1.y = DRAWER_INSIDE_DEPTH_10x40x1;
    box_10x40x1.z = DRAWER_INSIDE_HEIGHT_10x40x1;

    robast_ros2_msgs::msg::Box box_20x40x1;
    box_20x40x1.x = DRAWER_INSIDE_WIDTH_20x40x1;
    box_20x40x1.y = DRAWER_INSIDE_DEPTH_20x40x1;
    box_20x40x1.z = DRAWER_INSIDE_HEIGHT_20x40x1;

    robast_ros2_msgs::msg::Box box_30x40x1;
    box_30x40x1.x = DRAWER_INSIDE_WIDTH_30x40x1;
    box_30x40x1.y = DRAWER_INSIDE_DEPTH_30x40x1;
    box_30x40x1.z = DRAWER_INSIDE_HEIGHT_30x40x1;

    robast_ros2_msgs::msg::Drawer drawer_1;
    drawer_1.drawer_address.drawer_controller_id = 1;
    drawer_1.drawer_address.drawer_id = 1;
    drawer_1.number_of_drawers = 1;
    drawer_1.drawer_size = box_10x40x1;

    robast_ros2_msgs::msg::Drawer drawer_2;
    drawer_2.drawer_address.drawer_controller_id = 2;
    drawer_2.drawer_address.drawer_id = 1;
    drawer_2.number_of_drawers = 1;
    drawer_2.drawer_size = box_10x40x1;

    robast_ros2_msgs::msg::Drawer drawer_3;
    drawer_3.drawer_address.drawer_controller_id = 3;
    drawer_3.drawer_address.drawer_id = 1;
    drawer_3.number_of_drawers = 1;
    drawer_3.drawer_size = box_10x40x1;

    robast_ros2_msgs::msg::Drawer drawer_4;
    drawer_4.drawer_address.drawer_controller_id = 4;
    drawer_4.drawer_address.drawer_id = 1;
    drawer_4.number_of_drawers = 1;
    drawer_4.drawer_size = box_20x40x1;

    robast_ros2_msgs::msg::Drawer drawer_5;
    drawer_5.drawer_address.drawer_controller_id = 5;
    drawer_5.drawer_address.drawer_id = 1;
    drawer_5.number_of_drawers = 1;
    drawer_5.drawer_size = box_30x40x1;

    response->drawers = {drawer_1, drawer_2, drawer_3, drawer_4, drawer_5};
  }

  void DrawerGate::setup_serial_can_ubs_converter(void)
  {
    std::string setup_serial_port_result = this->serial_helper.open_serial();
    if (setup_serial_port_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from opening serial Port: %s", setup_serial_port_result.c_str());
    }
    this->set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps);
  }

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_user_access(uint32_t drawer_controller_id, uint8_t drawer_id, led_parameters led_parameters, uint8_t can_data_open_lock)
  {
    robast_can_msgs::CanMessage can_msg_drawer_user_access = can_db.can_messages.at(CAN_MSG_DRAWER_USER_ACCESS);

    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data = drawer_controller_id;

    // Default state is lock close
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data = CAN_DATA_CLOSE_LOCK;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data = CAN_DATA_CLOSE_LOCK;

    if (drawer_id == 1) 
    {
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data = can_data_open_lock;
    }
    if (drawer_id == 2)
    {
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data = can_data_open_lock;
    }

    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_RED).data = led_parameters.led_red;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_GREEN).data = led_parameters.led_green;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_BLUE).data = led_parameters.led_blue;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_BRIGHTNESS).data = led_parameters.brightness;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_MODE).data = led_parameters.mode;

    return can_msg_drawer_user_access;
  }

  void DrawerGate::set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface can_baudrate)
  {
    std::string msg;

    switch (can_baudrate)
    {
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_10kbps:
        msg = "S0";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_20kbps:
        msg = "S1";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_50kbps:
        msg = "S2";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_100kbps:
        msg = "S3";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_125kbps:
        msg = "S4";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps:
        msg = "S5";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_500kbps:
        msg = "S6";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_800kbps:
        msg = "S7";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_1000kbps:
        msg = "S8";
        break;
      default:
        msg = "S5";
        break;
    }

    std::string send_ascii_cmd_result = this->serial_helper.send_ascii_cmd(msg);
    if (send_ascii_cmd_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
    }
  }

  void DrawerGate::open_can_channel(void)
  {
    std::string send_ascii_cmd_result = this->serial_helper.send_ascii_cmd("O");
    if (send_ascii_cmd_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
    }
  }

  void DrawerGate::open_can_channel_listen_only_mode(void)
  {
    std::string send_ascii_cmd_result = this->serial_helper.send_ascii_cmd("L");
    if (send_ascii_cmd_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
    }
  }

  void DrawerGate::close_can_channel(void)
  {
    std::string send_ascii_cmd_result = this->serial_helper.send_ascii_cmd("C");
    if (send_ascii_cmd_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
    }
  }
  
  void DrawerGate::send_can_msg(robast_can_msgs::CanMessage can_message, led_parameters led_parameters)
  {
    this->setup_serial_can_ubs_converter();

    this->open_can_channel();

    std::optional<std::string> ascii_cmd = robast_can_msgs::encode_can_message_into_ascii_command(can_message, can_db.can_messages);
    
    if (ascii_cmd.has_value())
    {
      std::string send_ascii_cmd_result = this->serial_helper.send_ascii_cmd(ascii_cmd.value());
      if (send_ascii_cmd_result.size() > 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
      }
    }
    this->serial_helper.close_serial();
  }

  void DrawerGate::open_drawer(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    led_parameters led_parameters = {};
    led_parameters.led_red = 0;
    led_parameters.led_blue = 0;
    led_parameters.led_green = 255;
    led_parameters.brightness = 150;
    led_parameters.mode = 1;

    robast_can_msgs::CanMessage can_msg_open_drawer = DrawerGate::create_can_msg_drawer_user_access(drawer_controller_id, drawer_id, led_parameters, CAN_DATA_OPEN_LOCK);

    this->send_can_msg(can_msg_open_drawer, led_parameters);
  }

  void DrawerGate::wait_until_initial_drawer_status_received(uint32_t drawer_controller_id)
  {
    std::unique_lock<std::mutex> lock_guard_initial_drawer_status_received(this->drawer_status_mutex); // the lock will be released after the wait check
    cv.wait(
      lock_guard_initial_drawer_status_received, [this, drawer_controller_id]
      {
        return this->is_initial_drawer_status_received(drawer_controller_id);
      });
  }

  void DrawerGate::wait_until_drawer_is_opened(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    std::unique_lock<std::mutex> lock_guard_is_drawer_opend(this->drawer_status_mutex); // the lock will be released after the wait check
    cv.wait(
      lock_guard_is_drawer_opend, [this, drawer_controller_id, drawer_id]
      {
        return this->is_drawer_open(drawer_controller_id, drawer_id);
      });
  }

  bool DrawerGate::is_initial_drawer_status_received(uint32_t drawer_controller_id)
  {
    if (drawer_status_by_drawer_controller_id.find(drawer_controller_id) == drawer_status_by_drawer_controller_id.end())
    {
      // key does not exists in the map
      return false; 
    } 
    else
    {
      drawer_status drawer_status = this->drawer_status_by_drawer_controller_id[drawer_controller_id];
      return drawer_status.received_initial_drawer_status;
    }
  }

  bool DrawerGate::is_drawer_open(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    if (drawer_status_by_drawer_controller_id.find(drawer_controller_id) == drawer_status_by_drawer_controller_id.end())
    {
      // key does not yet exist in the map
      return false;
    } 
    else
    {
      drawer_status drawer_status = this->drawer_status_by_drawer_controller_id[drawer_controller_id];

      if (drawer_id == 1)
      {
        return !drawer_status.is_endstop_switch_1_pushed;
      }
      if (drawer_id == 2)
      {
        return !drawer_status.is_endstop_switch_2_pushed;
      }  
    }

    return false;
  }

  void DrawerGate::handle_open_drawer(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    led_parameters led_parameters = {};
    led_parameters.led_red = 255;
    led_parameters.led_blue = 255;
    led_parameters.led_green = 255;
    led_parameters.brightness = 150;
    led_parameters.mode = 0; // mode 0 = instantly light up LEDs, mode 1 = fade up led light

    robast_can_msgs::CanMessage can_msg_close_drawer = DrawerGate::create_can_msg_drawer_user_access(drawer_controller_id, drawer_id, led_parameters, CAN_DATA_CLOSE_LOCK);

    this->send_can_msg(can_msg_close_drawer, led_parameters);
  }

  void DrawerGate::wait_until_drawer_is_closed(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    std::unique_lock<std::mutex> lock_guard(this->drawer_status_mutex); // the lock will be released after the wait check
    cv.wait(
      lock_guard, [this, drawer_controller_id, drawer_id]
      {
        return this->is_drawer_closed(drawer_controller_id, drawer_id);
      });
  }

  bool DrawerGate::is_drawer_closed(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    drawer_status drawer_status = this->drawer_status_by_drawer_controller_id[drawer_controller_id];

    if (drawer_id == 1)
    {
      if (drawer_status.is_endstop_switch_1_pushed && !drawer_status.is_lock_switch_1_pushed)
      {
        return true;
      }
    }
    if (drawer_id == 2)
    {
      if (drawer_status.is_endstop_switch_2_pushed && !drawer_status.is_lock_switch_2_pushed)
      {
        return true;
      }
    }  

    return false;
  }

  void DrawerGate::handle_closed_drawer(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    led_parameters led_parameters = {};
    led_parameters.led_red = 255;
    led_parameters.led_blue = 0;
    led_parameters.led_green = 128;
    led_parameters.brightness = 150;
    led_parameters.mode = 2; // mode 0 = instantly light up LEDs, mode 1 = fade up led light, mode 2 = closing drawer led mode

    robast_can_msgs::CanMessage can_msg_closed_drawer = DrawerGate::create_can_msg_drawer_user_access(drawer_controller_id, drawer_id, led_parameters, CAN_DATA_CLOSE_LOCK);

    this->send_can_msg(can_msg_closed_drawer, led_parameters);

     // Cancel the timer that is handling the feedback of the drawer_controller
    this->timer_ptr_->cancel();

    // Reset flag that is responsible for indicating that a up-to-date drawer_status was received
    this->drawer_status_by_drawer_controller_id[drawer_controller_id].received_initial_drawer_status = false; 
    
    // Reset the flag that is responsible for clearing the serial buffer from old CAN messages
    cleared_serial_buffer_from_old_can_msgs = false;
  }
  
  void DrawerGate::handle_drawer_user_access(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle) 
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal"); // DEBUGGING

    // Starting timer to receive feedback from drawer controller until the process of opening a drawer is finished
    this->timer_cb_group_ = nullptr; //This might be replaced in the future to better use callback groups. With the default setting above (nullptr / None), the timer will use the node’s default Mutually Exclusive Callback Group.
    this->timer_ptr_ = this->create_wall_timer(50ms, std::bind(&DrawerGate::timer_callback, this), timer_cb_group_);

    // Although the feedback is received periodical via the timer,
    // we update the drawer_status once manually to make sure
    // the drawer_status is up to date at the start
    this->update_drawer_status_from_can();

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DrawerUserAccess::Feedback>();
    auto result = std::make_shared<DrawerUserAccess::Result>();
    uint32_t drawer_controller_id = goal->drawer.drawer_controller_id;
    uint8_t drawer_id = goal->drawer.drawer_id;
    uint8_t state = goal -> state; // variable to control which step of the drawer user access should be performed
 
    switch (state)
    {
      case 1:
        // 1. step: Open lock of the drawer and light up LEDs to signalize which drawer should be opened
        this->open_drawer(drawer_controller_id, drawer_id);
      
      case 2:
        // 2. step: Wait until at least one drawer_status feedback message from the Drawer Controller is received  
        this->wait_until_initial_drawer_status_received(drawer_controller_id);

      case 3:
        // 3. step: Wait until drawer is opened
        this->wait_until_drawer_is_opened(drawer_controller_id, drawer_id);

      case 4:
        // 4. step: After drawer was opened, close lock and change light color
        this->handle_open_drawer(drawer_controller_id, drawer_id);

      case 5:
        // 5. step: Wait until drawer is closed again
        this->wait_until_drawer_is_closed(drawer_controller_id, drawer_id);

      case 6:
        // 6. step: LED feedback
        this->handle_closed_drawer(drawer_controller_id, drawer_id);
      
      default:
        break;
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Finished executing goal"); // DEBUGGING
  }
}  // namespace robast_drawer_gate
