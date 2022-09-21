#include "drawer_gate/drawer_gate.hpp"


// For DEBUGGING purposes this is the action send_goal command:
// ros2 action send_goal /control_drawer communication_interfaces/action/DrawerUserAccess "{drawer_address: {drawer_controller_id: 1, drawer_id: 1}, state: 1}"

namespace drawer_gate
{
  DrawerGate::DrawerGate() : Node("drawer_gate")
  {
    this->drawer_gate_server_ = rclcpp_action::create_server<DrawerUserAccess>(
      this,
      "control_drawer",
      std::bind(&DrawerGate::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrawerGate::cancel_callback, this, std::placeholders::_1),
      std::bind(&DrawerGate::accepted_callback, this, std::placeholders::_1));

    this->shelf_setup_info_service_ = this->create_service<ShelfSetupInfo>(
      "shelf_setup_info",
      std::bind(&DrawerGate::provide_shelf_setup_info_callback, this, std::placeholders::_1, std::placeholders::_2));

    this->drawer_refill_subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "drawer_refill_status", 10, std::bind(&DrawerGate::drawer_refill_topic_callback, this, std::placeholders::_1));

    // Some tests show that a timer period of 3 ms is to fast and asci cmds get lost at this speed, with 4 ms it worked, so use 5ms with safety margin
    this->send_ascii_cmds_timer_ = this->create_wall_timer(5ms, std::bind(&DrawerGate::send_ascii_cmds_timer_callback, this));

    this->setup_serial_can_ubs_converter();
    // this->serial_helper_.close_serial();

    this->drawer_is_beeing_accessed_ = false;
  }

  //TODO: Dekonstruktor with this->serial_helper_.close_serial();

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
    std::lock_guard<std::mutex> scoped_lock(drawer_status_mutex_); // the lock will be released after the scope of this function
    for (uint8_t i = 0; i < drawer_feedback_can_msgs.size(); i++)
    {
      if (drawer_feedback_can_msgs.at(i).id == CAN_ID_DRAWER_FEEDBACK)
      {
        uint32_t drawer_controller_id = drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data();
        drawer_status drawer_status = {};
        drawer_status.is_endstop_switch_1_pushed = drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).get_data() == 1;
        drawer_status.is_lock_switch_1_pushed = drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).get_data() == 1;
        drawer_status.is_endstop_switch_2_pushed = drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).get_data() == 1;
        drawer_status.is_lock_switch_2_pushed = drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).get_data() == 1;
        drawer_status.received_initial_drawer_status = true;
        this->drawer_status_by_drawer_controller_id_[drawer_controller_id] = drawer_status;
      }
    }
    this->cv_.notify_one(); // Notify the waiting thread in the open_drawer function to check if condition is now satisfied 
  }


  // This timer callback makes sure there is enough time between each ascii command, otherwise some commands might get lost
  void DrawerGate::send_ascii_cmds_timer_callback(void)
  {
    if (this->ascii_cmd_queue_.empty())
    {
      this->send_ascii_cmds_timer_->cancel(); // cancel the time when queue is empty
      return;
    }
    else
    {
      std::string send_ascii_cmd_result = this->serial_helper_.send_ascii_cmd(this->ascii_cmd_queue_.front());
      if (send_ascii_cmd_result.size() > 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
      }
      this->ascii_cmd_queue_.pop(); //remove first element of the queue
    }
  }

  void DrawerGate::timer_callback(void)
  {
    this->update_drawer_status_from_can();
  }

  void DrawerGate::update_drawer_status_from_can(void)
  {
    std::string serial_read_ascii_command;
    uint16_t num_of_received_bytes = this->serial_helper_.read_serial(&serial_read_ascii_command, 200);

    //RCLCPP_INFO(this->get_logger(), "Serial read: %s", serial_read_ascii_command.c_str()); //DEBUGGING

    std::vector<robast_can_msgs::CanMessage> received_can_msgs = robast_can_msgs::decode_multiple_ascii_commands_into_can_messages(serial_read_ascii_command, CAN_ID_DRAWER_FEEDBACK, CAN_DLC_DRAWER_FEEDBACK, this->can_db_.can_messages);

    if (this->cleared_serial_buffer_from_old_can_msgs_) 
    {
      this->update_drawer_status(received_can_msgs);
    }
    else
    {
      // Dump only the very first received can msgs because they might contain old data in the buffer
      this->cleared_serial_buffer_from_old_can_msgs_ = true;
    }
  }

  void DrawerGate::drawer_refill_topic_callback(const std_msgs::msg::UInt8MultiArray & msg)
  {
    // How to publish test msg: ros2 topic pub /drawer_refill_status std_msgs/msg/UInt8MultiArray "{layout: {dim: [], data_offset: 0}, data: [1, 2, 3]}"
    if (drawer_is_beeing_accessed_)
    {
      RCLCPP_INFO(this->get_logger(), "There is currently a drawer beeing accessed so drawer refill status will be changed after drawer access is finished!"); // DEBUGGING
      return;
    }

    for (uint8_t i = 0; i < std::size(msg.data); i++)
    {
      uint32_t drawer_controller_id = msg.data[i];

      if (this->drawer_to_be_refilled_by_drawer_controller_id_.find(drawer_controller_id) == this->drawer_to_be_refilled_by_drawer_controller_id_.end())
      {
        // key does not exists in the map
        this->drawer_to_be_refilled_by_drawer_controller_id_[drawer_controller_id] = true;

        this->send_drawer_refill_status(drawer_controller_id, true);
      }
      else
      {
        if (this->drawer_to_be_refilled_by_drawer_controller_id_[drawer_controller_id] == false)
        {
          this->drawer_to_be_refilled_by_drawer_controller_id_[drawer_controller_id] = true;
          this->send_drawer_refill_status(drawer_controller_id, true);
        }
      }
    }

    // Check if there is one drawer_controller_id that isn't sent anymore, which means it has been refilled

    // Create a map iterator and point to beginning of map
    std::map<uint32_t, bool>::iterator it = this->drawer_to_be_refilled_by_drawer_controller_id_.begin();
    // Iterate over the map using c++11 range based for loop
    for (std::pair<uint32_t, bool> element : this->drawer_to_be_refilled_by_drawer_controller_id_)
    {   
      uint32_t drawer_controller_id = element.first; // Accessing KEY from element
      bool drawer_needs_to_be_refilled = element.second; // Accessing VALUE from element

      if (std::find(std::begin(msg.data), std::end(msg.data), drawer_controller_id) == std::end(msg.data))
      {
        // drawer_controller_id not found in data from topic, which means that this drawer has been refilled
        if (drawer_needs_to_be_refilled == true)
        {
          this->send_drawer_refill_status(drawer_controller_id, false);
          this->drawer_to_be_refilled_by_drawer_controller_id_[drawer_controller_id] = false;
        }
      }
    }
  }

  void DrawerGate::send_drawer_refill_status(uint32_t drawer_controller_id, bool refill_drawer)
  {
    led_parameters led_parameters = {};

    if (refill_drawer)
    {
      led_parameters.led_red = 0;
      led_parameters.led_blue = 255;
      led_parameters.led_green = 50;
      led_parameters.brightness = 150;
      led_parameters.mode = 3;
      RCLCPP_INFO(this->get_logger(), "Drawer with drawer controller id %i needs to be refilled! Sending CAN message!", drawer_controller_id);
    }
    else
    {
      led_parameters.led_red = 0;
      led_parameters.led_blue = 255;
      led_parameters.led_green = 50;
      led_parameters.brightness = 150;
      led_parameters.mode = 2;
      RCLCPP_INFO(this->get_logger(), "Drawer with drawer controller id %i was refilled! Sending CAN message!", drawer_controller_id);
    }  

    robast_can_msgs::CanMessage can_msg_refill_drawer = DrawerGate::create_can_msg_drawer_user_access(drawer_controller_id, 1, led_parameters, CAN_DATA_CLOSE_LOCK);

    this->send_can_msg(can_msg_refill_drawer);
  }

  void DrawerGate::provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    //TODO: This should actually be done automatically by polling all drawer_controller on the CAN bus

    communication_interfaces::msg::Box box_10x40x1;
    box_10x40x1.x = DRAWER_INSIDE_WIDTH_10x40x1;
    box_10x40x1.y = DRAWER_INSIDE_DEPTH_10x40x1;
    box_10x40x1.z = DRAWER_INSIDE_HEIGHT_10x40x1;

    communication_interfaces::msg::Box box_20x40x1;
    box_20x40x1.x = DRAWER_INSIDE_WIDTH_20x40x1;
    box_20x40x1.y = DRAWER_INSIDE_DEPTH_20x40x1;
    box_20x40x1.z = DRAWER_INSIDE_HEIGHT_20x40x1;

    communication_interfaces::msg::Box box_30x40x1;
    box_30x40x1.x = DRAWER_INSIDE_WIDTH_30x40x1;
    box_30x40x1.y = DRAWER_INSIDE_DEPTH_30x40x1;
    box_30x40x1.z = DRAWER_INSIDE_HEIGHT_30x40x1;

    communication_interfaces::msg::Drawer drawer_1;
    drawer_1.drawer_address.drawer_controller_id = 1;
    drawer_1.drawer_address.drawer_id = 1;
    drawer_1.number_of_drawers = 1;
    drawer_1.drawer_size = box_10x40x1;

    communication_interfaces::msg::Drawer drawer_2;
    drawer_2.drawer_address.drawer_controller_id = 2;
    drawer_2.drawer_address.drawer_id = 1;
    drawer_2.number_of_drawers = 1;
    drawer_2.drawer_size = box_10x40x1;

    communication_interfaces::msg::Drawer drawer_3;
    drawer_3.drawer_address.drawer_controller_id = 3;
    drawer_3.drawer_address.drawer_id = 1;
    drawer_3.number_of_drawers = 1;
    drawer_3.drawer_size = box_10x40x1;

    communication_interfaces::msg::Drawer drawer_4;
    drawer_4.drawer_address.drawer_controller_id = 4;
    drawer_4.drawer_address.drawer_id = 1;
    drawer_4.number_of_drawers = 1;
    drawer_4.drawer_size = box_20x40x1;

    communication_interfaces::msg::Drawer drawer_5;
    drawer_5.drawer_address.drawer_controller_id = 5;
    drawer_5.drawer_address.drawer_id = 1;
    drawer_5.number_of_drawers = 1;
    drawer_5.drawer_size = box_30x40x1;

    response->drawers = {drawer_1, drawer_2, drawer_3, drawer_4, drawer_5};
  }

  void DrawerGate::setup_serial_can_ubs_converter(void)
  {
    std::string setup_serial_port_result = this->serial_helper_.open_serial();
    if (setup_serial_port_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from opening serial Port: %s", setup_serial_port_result.c_str());
    }
    this->close_can_channel();
    this->set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps);
    this->open_can_channel(); // the default state should be the open can channel to enable receiving CAN messages
  }

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_user_access(uint32_t drawer_controller_id, uint8_t drawer_id, led_parameters led_parameters, uint8_t can_data_open_lock)
  {
    robast_can_msgs::CanMessage can_msg_drawer_user_access = this->can_db_.can_messages.at(CAN_MSG_DRAWER_USER_ACCESS);

    std::vector can_signals_drawer_user_access = can_msg_drawer_user_access.get_can_signals();

    can_signals_drawer_user_access.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(drawer_controller_id);

    // Default state is lock close
    can_signals_drawer_user_access.at(CAN_SIGNAL_OPEN_LOCK_1).set_data(CAN_DATA_CLOSE_LOCK);
    can_signals_drawer_user_access.at(CAN_SIGNAL_OPEN_LOCK_2).set_data(CAN_DATA_CLOSE_LOCK);

    if (drawer_id == 1) 
    {
      can_signals_drawer_user_access.at(CAN_SIGNAL_OPEN_LOCK_1).set_data(can_data_open_lock);
    }
    if (drawer_id == 2)
    {
      can_signals_drawer_user_access.at(CAN_SIGNAL_OPEN_LOCK_2).set_data(can_data_open_lock);
    }

    can_signals_drawer_user_access.at(CAN_SIGNAL_LED_RED).set_data(led_parameters.led_red);
    can_signals_drawer_user_access.at(CAN_SIGNAL_LED_GREEN).set_data(led_parameters.led_green);
    can_signals_drawer_user_access.at(CAN_SIGNAL_LED_BLUE).set_data(led_parameters.led_blue);
    can_signals_drawer_user_access.at(CAN_SIGNAL_LED_BRIGHTNESS).set_data(led_parameters.brightness);
    can_signals_drawer_user_access.at(CAN_SIGNAL_LED_MODE).set_data(led_parameters.mode);

    can_msg_drawer_user_access.set_can_signals(can_signals_drawer_user_access);

    return can_msg_drawer_user_access;
  }

  void DrawerGate::add_ascii_cmd_to_queue(std::string ascii_cmd)
  {
    // if queue was empty, the timer has been canceled before, so start timer for timer callbacks which trigger sending the ascii cmds
    if (this->ascii_cmd_queue_.empty())
    {
      this->ascii_cmd_queue_.push(ascii_cmd);
      this->send_ascii_cmds_timer_ = this->create_wall_timer(5ms, std::bind(&DrawerGate::send_ascii_cmds_timer_callback, this));
    }
    else
    {
      this->ascii_cmd_queue_.push(ascii_cmd);
    }
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

    this->add_ascii_cmd_to_queue(msg);
  }

  void DrawerGate::open_can_channel(void)
  {
    this->add_ascii_cmd_to_queue("O");
  }

  void DrawerGate::open_can_channel_listen_only_mode(void)
  {
    this->add_ascii_cmd_to_queue("L");
  }

  void DrawerGate::close_can_channel(void)
  {
    this->add_ascii_cmd_to_queue("C");
  }
  
  void DrawerGate::send_can_msg(robast_can_msgs::CanMessage can_message)
  {
    std::optional<std::string> ascii_cmd = robast_can_msgs::encode_can_message_into_ascii_command(can_message, this->can_db_.can_messages);
    
    if (ascii_cmd.has_value())
    {
      this->add_ascii_cmd_to_queue(ascii_cmd.value());
    }
    else
    {
      //TODO: Error handling
    }
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

    this->send_can_msg(can_msg_open_drawer);
  }

  void DrawerGate::wait_until_initial_drawer_status_received(uint32_t drawer_controller_id)
  {
    std::unique_lock<std::mutex> lock_guard_initial_drawer_status_received(this->drawer_status_mutex_); // the lock will be released after the wait check
    this->cv_.wait(
      lock_guard_initial_drawer_status_received, [this, drawer_controller_id]
      {
        return this->is_initial_drawer_status_received(drawer_controller_id);
      });
  }

  void DrawerGate::wait_until_drawer_is_opened(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    std::unique_lock<std::mutex> lock_guard_is_drawer_opend(this->drawer_status_mutex_); // the lock will be released after the wait check
    this->cv_.wait(
      lock_guard_is_drawer_opend, [this, drawer_controller_id, drawer_id]
      {
        return this->is_drawer_open(drawer_controller_id, drawer_id);
      });
  }

  bool DrawerGate::is_initial_drawer_status_received(uint32_t drawer_controller_id)
  {
    if (drawer_status_by_drawer_controller_id_.find(drawer_controller_id) == drawer_status_by_drawer_controller_id_.end())
    {
      // key does not exists in the map
      return false; 
    } 
    else
    {
      drawer_status drawer_status = this->drawer_status_by_drawer_controller_id_[drawer_controller_id];
      return drawer_status.received_initial_drawer_status;
    }
  }

  bool DrawerGate::is_drawer_open(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    if (drawer_status_by_drawer_controller_id_.find(drawer_controller_id) == drawer_status_by_drawer_controller_id_.end())
    {
      // key does not yet exist in the map
      return false;
    } 
    else
    {
      drawer_status drawer_status = this->drawer_status_by_drawer_controller_id_[drawer_controller_id];

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

    this->send_can_msg(can_msg_close_drawer);
  }

  void DrawerGate::wait_until_drawer_is_closed(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    std::unique_lock<std::mutex> lock_guard(this->drawer_status_mutex_); // the lock will be released after the wait check
    this->cv_.wait(
      lock_guard, [this, drawer_controller_id, drawer_id]
      {
        return this->is_drawer_closed(drawer_controller_id, drawer_id);
      });
  }

  bool DrawerGate::is_drawer_closed(uint32_t drawer_controller_id, uint8_t drawer_id)
  {
    drawer_status drawer_status = this->drawer_status_by_drawer_controller_id_[drawer_controller_id];

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

    this->send_can_msg(can_msg_closed_drawer);

     // Cancel the timer that is handling the feedback of the drawer_controller
    this->timer_ptr_->cancel();

    // Reset flag that is responsible for indicating that a up-to-date drawer_status was received
    this->drawer_status_by_drawer_controller_id_[drawer_controller_id].received_initial_drawer_status = false; 
    
    // Reset the flag that is responsible for clearing the serial buffer from old CAN messages
    this->cleared_serial_buffer_from_old_can_msgs_ = false;
  }


  void DrawerGate::state_machine_drawer_gate(uint32_t drawer_controller_id, uint8_t drawer_id, uint8_t state)
  {
    switch (state)
    {
      case 1:
        // 1. step: Open lock of the drawer and light up LEDs to signalize which drawer should be opened
        RCLCPP_INFO(this->get_logger(), "Step 1: Open lock of the drawer and light up LEDs to signalize which drawer should be opened"); // DEBUGGING
        this->open_drawer(drawer_controller_id, drawer_id);
      
      case 2:
        // 2. step: Wait until at least one drawer_status feedback message from the Drawer Controller is received  
        RCLCPP_INFO(this->get_logger(), "Step 2: Wait until at least one drawer_status feedback message from the Drawer Controller is received "); // DEBUGGING
        this->wait_until_initial_drawer_status_received(drawer_controller_id);

      case 3:
        // 3. step: Wait until drawer is opened
        RCLCPP_INFO(this->get_logger(), "Step 3: Wait until drawer is opened"); // DEBUGGING
        this->wait_until_drawer_is_opened(drawer_controller_id, drawer_id);

      case 4:
        // 4. step: After drawer was opened, close lock and change light color
        RCLCPP_INFO(this->get_logger(), "Step 4: After drawer was opened, close lock and change light color"); // DEBUGGING
        this->handle_open_drawer(drawer_controller_id, drawer_id);

      case 5:
        // 5. step: Wait until drawer is closed again
        RCLCPP_INFO(this->get_logger(), "Step 5: Wait until drawer is closed again"); // DEBUGGING
        this->wait_until_drawer_is_closed(drawer_controller_id, drawer_id);

      case 6:
        // 6. step: LED closed feedback
        RCLCPP_INFO(this->get_logger(), "Step 6: LED closed feedback"); // DEBUGGING
        this->handle_closed_drawer(drawer_controller_id, drawer_id);
      
      default:
        break;
    }
  }

  
  void DrawerGate::handle_drawer_user_access(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle) 
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal"); // DEBUGGING

    if (this->drawer_is_beeing_accessed_)
    {
      RCLCPP_INFO(this->get_logger(), "There is currently another drawer beeing accessed so accessing another drawer is denied!"); // DEBUGGING
      return;
    }

    this->drawer_is_beeing_accessed_ = true;

    this->setup_serial_can_ubs_converter();

    // Starting timer to receive feedback from drawer controller until the process of opening a drawer is finished
    this->timer_cb_group_ = nullptr; //This might be replaced in the future to better use callback groups. With the default setting above (nullptr / None), the timer will use the node’s default Mutually Exclusive Callback Group.
    this->timer_ptr_ = this->create_wall_timer(50ms, std::bind(&DrawerGate::timer_callback, this), timer_cb_group_);

    // Although the feedback is received periodical via the timer, we update the drawer_status once manually to make sure
    // the drawer_status is up to date at the start
    this->update_drawer_status_from_can();

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DrawerUserAccess::Feedback>();
    auto result = std::make_shared<DrawerUserAccess::Result>();
    uint32_t drawer_controller_id = goal->drawer_address.drawer_controller_id;
    uint8_t drawer_id = goal->drawer_address.drawer_id;
    uint8_t state = goal -> state; // variable to control which step of the drawer user access should be performed
 
    this->state_machine_drawer_gate(drawer_controller_id, drawer_id, state);

    goal_handle->succeed(result);

    this->drawer_is_beeing_accessed_ = false;
    //TODO: Wait a little bit to clear drawer_to_be_refilled_by_drawer_controller_id_ to give LEDs enough time to make suitable lightshow
    this->drawer_to_be_refilled_by_drawer_controller_id_.clear(); // clear this mapping so it is again checked, if drawer still needs to be refilled

    RCLCPP_INFO(this->get_logger(), "Finished executing goal"); // DEBUGGING
  }
}  // namespace drawer_gate
