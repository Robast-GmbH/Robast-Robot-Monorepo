#include "drawer_gate/drawer_gate.hpp"

// For DEBUGGING purposes this is the action send_goal command:
// ros2 action send_goal /control_drawer communication_interfaces/action/DrawerUserAccess "{drawer_address: {drawer_controller_id: 1, drawer_id: 1}, state: 1}"

namespace drawer_gate
{
  DrawerGate::DrawerGate(const string serial_path) : Node("drawer_gate")
  {
    this->serial_helper_ = new serial_helper::SerialHelper(serial_path);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    this->open_drawer_subscription_ = this->create_subscription<DrawerAddress>(
      "open_drawer", qos, std::bind(&DrawerGate::open_drawer_topic_callback, this, std::placeholders::_1));

    this->drawer_leds_subscription_ = this->create_subscription<DrawerLeds>(
      "drawer_leds", qos, std::bind(&DrawerGate::drawer_leds_topic_callback, this, std::placeholders::_1));

    this->drawer_status_publisher_ = this->create_publisher<DrawerStatus>("drawer_is_open", qos);

    this->shelf_setup_info_service_ = this->create_service<ShelfSetupInfo>(
      "shelf_setup_info",
      std::bind(&DrawerGate::provide_shelf_setup_info_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Some tests show that a timer period of 3 ms is to fast and asci cmds get lost at this speed, with 4 ms it worked, so use 5ms with safety margin
    this->send_ascii_cmds_timer_ = this->create_wall_timer(5ms, std::bind(&DrawerGate::send_ascii_cmds_timer_callback, this));

    this->setup_serial_can_ubs_converter();
    // this->serial_helper_->close_serial();
  }

  DrawerGate::~DrawerGate()
  {
    this->serial_helper_->close_serial();
    delete this->serial_helper_;
  }

  void DrawerGate::open_drawer_topic_callback(const DrawerAddress& msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%i'", msg.drawer_controller_id); // Debugging

    uint32_t drawer_controller_id = msg.drawer_controller_id;
    uint8_t drawer_id = msg.drawer_id;

    robast_can_msgs::CanMessage can_msg_open_lock = this->create_can_msg_drawer_lock(drawer_controller_id, drawer_id, CAN_DATA_OPEN_LOCK);

    this->send_can_msg(can_msg_open_lock);
  }

  void DrawerGate::drawer_leds_topic_callback(const DrawerLeds& msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%i'", msg.red); // Debugging

    led_parameters led_parameters = {};
    led_parameters.led_red = msg.red;
    led_parameters.led_blue = msg.blue;
    led_parameters.led_green = msg.green;
    led_parameters.brightness = msg.brightness;
    led_parameters.mode = msg.mode;

    robast_can_msgs::CanMessage can_msg_drawer_led = this->create_can_msg_drawer_led(msg.drawer_address.drawer_controller_id, led_parameters);

    this->send_can_msg(can_msg_drawer_led);
  }

  void DrawerGate::update_drawer_status(std::vector<robast_can_msgs::CanMessage> drawer_feedback_can_msgs)
  {
    std::lock_guard<std::mutex> scoped_lock(drawer_status_mutex_); // the lock will be released after the scope of this function
    for (uint8_t i = 0; i < drawer_feedback_can_msgs.size(); i++)
    {
      if (drawer_feedback_can_msgs.at(i).get_id() == CAN_ID_DRAWER_FEEDBACK)
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
      std::string send_ascii_cmd_result = this->serial_helper_->send_ascii_cmd(this->ascii_cmd_queue_.front());
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
    uint16_t num_of_received_bytes = this->serial_helper_->read_serial(&serial_read_ascii_command, 200);

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

  std::vector<communication_interfaces::msg::Drawer> DrawerGate::get_all_mounted_drawers()
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

    return { drawer_1, drawer_2, drawer_3, drawer_4, drawer_5 };
  }

  void DrawerGate::provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    response->drawers = this->get_all_mounted_drawers();
  }

  void DrawerGate::setup_serial_can_ubs_converter(void)
  {
    std::string setup_serial_port_result = this->serial_helper_->open_serial();
    if (setup_serial_port_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from opening serial Port: %s", setup_serial_port_result.c_str());
    }
    this->close_can_channel();
    this->set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps);
    this->open_can_channel(); // the default state should be the open can channel to enable receiving CAN messages
  }

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_lock(uint32_t drawer_controller_id, uint8_t drawer_id, uint8_t can_data_open_lock) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_lock = this->can_db_.can_messages.at(CAN_MSG_DRAWER_LOCK);

    std::vector can_signals_drawer_lock = can_msg_drawer_lock.get_can_signals();

    can_signals_drawer_lock.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(drawer_controller_id);

    // Default state is lock close
    can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_1).set_data(CAN_DATA_CLOSE_LOCK);
    can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_2).set_data(CAN_DATA_CLOSE_LOCK);

    if (drawer_id == 1)
    {
      can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_1).set_data(can_data_open_lock);
    }
    if (drawer_id == 2)
    {
      can_signals_drawer_lock.at(CAN_SIGNAL_OPEN_LOCK_2).set_data(can_data_open_lock);
    }

    can_msg_drawer_lock.set_can_signals(can_signals_drawer_lock);

    return can_msg_drawer_lock;
  }

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_led(uint32_t drawer_controller_id, led_parameters led_parameters) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_led = this->can_db_.can_messages.at(CAN_MSG_DRAWER_LED);

    std::vector can_signals_drawer_led = can_msg_drawer_led.get_can_signals();

    can_signals_drawer_led.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(drawer_controller_id);

    can_signals_drawer_led.at(CAN_SIGNAL_LED_RED).set_data(led_parameters.led_red);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_GREEN).set_data(led_parameters.led_green);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_BLUE).set_data(led_parameters.led_blue);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_BRIGHTNESS).set_data(led_parameters.brightness);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_MODE).set_data(led_parameters.mode);

    can_msg_drawer_led.set_can_signals(can_signals_drawer_led);

    return can_msg_drawer_led;
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


  void DrawerGate::send_default_led_status_to_drawer(uint32_t drawer_controller_id)
  {
    led_parameters led_parameters = {};
    led_parameters.led_red = 0;
    led_parameters.led_blue = 255;
    led_parameters.led_green = 50;
    led_parameters.brightness = 100;
    led_parameters.mode = LedMode::steady_light;

    robast_can_msgs::CanMessage can_msg_default_led_status = this->create_can_msg_drawer_led(drawer_controller_id, led_parameters);

    this->send_can_msg(can_msg_default_led_status);

    RCLCPP_INFO(this->get_logger(), "Setting default LED status for drawer controller id %i", drawer_controller_id); // DEBUGGING
  }

  void DrawerGate::handle_default_led_status_for_all_drawers()
  {
    for (communication_interfaces::msg::Drawer drawer : this->get_all_mounted_drawers())
    {
      if (this->drawer_to_be_refilled_by_drawer_controller_id_.find(drawer.drawer_address.drawer_controller_id) == this->drawer_to_be_refilled_by_drawer_controller_id_.end())
      {
        // not found, meaning that drawer should just light up in default state
        this->send_default_led_status_to_drawer(drawer.drawer_address.drawer_controller_id);
      }
      else
      {
        // found, meaning that drawer needs to be refilled. This is handled in the drawer_refill_topic_callback
      }
    }
  }

  void DrawerGate::handle_default_drawer_status(uint32_t drawer_controller_id)
  {
    // Wait a little bit to clear drawer_to_be_refilled_by_drawer_controller_id_ to give LEDs enough time to make suitable lightshow when closing
    boost::asio::io_context io;
    boost::asio::steady_timer t(io, boost::asio::chrono::milliseconds(800));
    t.async_wait(boost::bind(&DrawerGate::handle_default_led_status_for_all_drawers, this));
    io.run();

    this->drawer_to_be_refilled_by_drawer_controller_id_.clear(); // clear this mapping so it is again checked and resynchronized, if drawer still needs to be refilled
  }

}  // namespace drawer_gate
