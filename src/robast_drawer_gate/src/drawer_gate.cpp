#include "robast_drawer_gate/drawer_gate.hpp"


// For DEBUGGING purposes this is the action send_goal command:
// ros2 action send_goal /control_drawer robast_ros2_msgs/action/DrawerUserAccess "{drawer: {drawer_controller_id: 1, drawer_id: 1}}"

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
      std::bind(&DrawerGate::provide_shelf_setup_info, this, std::placeholders::_1, std::placeholders::_2));

    this->setup_serial_can_ubs_converter();
    // When the USB-CAN Adapter isn't sending CAN messages, the default state should be the open can channel to enable receiving CAN messages
    this->open_can_channel();
    this->serial_helper.close_serial();

    //TODO: Timer callback, der regelmäßig aufgerufen wird und CAN Messages EINLIEST.
    //TODO: Das sollte aber niemals gleichzeitig zum Abschicken von CAN Messages passieren, daher beide in eine Mutually Exclusive Callback Group packen!
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
    std::thread{std::bind(&DrawerGate::open_drawer, this, std::placeholders::_1), goal_handle}.detach();
  }

  void DrawerGate::timer_callback(void)
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggert!");
    this->setup_serial_can_ubs_converter();
    this->open_can_channel(); 
    std::string serial_read_ascii_command;
    uint16_t num_of_received_bytes = this->serial_helper.read_serial(&serial_read_ascii_command, 200);

    std::vector<robast_can_msgs::CanMessage> received_can_msgs = robast_can_msgs::decode_multiple_ascii_commands_into_can_messages(serial_read_ascii_command, CAN_ID_DRAWER_FEEDBACK, CAN_DLC_DRAWER_FEEDBACK, can_db.can_messages);

    for (uint8_t i = 0; i < received_can_msgs.size(); i++)
    {
      RCLCPP_INFO(this->get_logger(), "received_can_msgs.at(i).can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data: %u", received_can_msgs.at(i).can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data);
    }

    this->serial_helper.close_serial();
  }

  void DrawerGate::provide_shelf_setup_info(const std::shared_ptr<ShelfSetupInfo::Request> request, std::shared_ptr<ShelfSetupInfo::Response> response)
  {
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

    robast_ros2_msgs::msg::Drawer drawer;
    drawer.drawer_address.drawer_controller_id = 1;
    drawer.drawer_address.drawer_id = 1;
    drawer.number_of_drawers = 1;
    drawer.drawer_size = box_10x40x1;

    // robast_ros2_msgs::msg::Drawer
    // response->drawers = 

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

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_user_access(std::shared_ptr<const DrawerUserAccess::Goal> goal, led_parameters led_parameters)
  {
    robast_can_msgs::CanMessage can_msg_drawer_user_access = can_db.can_messages.at(CAN_MSG_DRAWER_USER_ACCESS);

    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data = goal->drawer.drawer_controller_id;

    // DEBUGGING
    if (goal->drawer.drawer_id == 0) 
    {
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data = CAN_DATA_CLOSE_LOCK;
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data = CAN_DATA_CLOSE_LOCK;
    }
    if (goal->drawer.drawer_id == 1) 
    {
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data = CAN_DATA_OPEN_LOCK;
    }
    if (goal->drawer.drawer_id == 2)
    {
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data = CAN_DATA_OPEN_LOCK;
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
  
  void DrawerGate::open_drawer(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle) 
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal"); // DEBUGGING

    // Starting timer to receive feedback from drawer controller until the process of opening a drawer is finished
    this->timer_cb_group_ = nullptr; //This might be replaced in the future to better use callback groups. With the default setting above (nullptr / None), the timer will use the node’s default Mutually Exclusive Callback Group.
    this->timer_ptr_ = this->create_wall_timer(1000ms, std::bind(&DrawerGate::timer_callback, this), timer_cb_group_);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DrawerUserAccess::Feedback>();
    auto result = std::make_shared<DrawerUserAccess::Result>();

    led_parameters led_parameters = {};
    // Debugging
    if (goal->drawer.drawer_id == 1)
    {
      led_parameters.led_red = 00;
      led_parameters.led_blue = 0;
      led_parameters.led_green = 255;
      led_parameters.brightness = 255;
      led_parameters.mode = 1;
    }
    else if (goal->drawer.drawer_id == 0)
    {
      led_parameters.led_red = 100;
      led_parameters.led_blue = 0;
      led_parameters.led_green = 50;
      led_parameters.brightness = 0;
      led_parameters.mode = 1;
    }  

    robast_can_msgs::CanMessage can_msg_drawer_user_access = DrawerGate::create_can_msg_drawer_user_access(goal, led_parameters);

    this->setup_serial_can_ubs_converter();

    this->open_can_channel();

    std::optional<std::string> ascii_cmd_drawer_user_access = robast_can_msgs::encode_can_message_into_ascii_command(can_msg_drawer_user_access, can_db.can_messages);
    
    if (ascii_cmd_drawer_user_access.has_value())
    {
      std::string send_ascii_cmd_result = this->serial_helper.send_ascii_cmd(ascii_cmd_drawer_user_access.value());
      if (send_ascii_cmd_result.size() > 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
      }
    }

    this->serial_helper.close_serial();

    this->timer_ptr_->cancel(); // Cancel the timer that is handling the feedback of the 

    RCLCPP_INFO(this->get_logger(), "Finished executing goal"); // DEBUGGING
  }
}  // namespace robast_drawer_gate
