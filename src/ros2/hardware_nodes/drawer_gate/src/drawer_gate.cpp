#include "drawer_gate/drawer_gate.hpp"

// For DEBUGGING purposes this is the action send_goal command:
// ros2 action send_goal /control_drawer communication_interfaces/action/DrawerUserAccess "{drawer_address:
// {drawer_controller_id: 1, drawer_id: 1}, state: 1}"

namespace drawer_gate
{
  DrawerGate::DrawerGate() : Node("drawer_gate")
  { 
    setup_subscriptions();
    setup_publishers();
    setup_services();
  }

  void DrawerGate::setup_subscriptions()
  {
    auto qos_subscriptions = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));

    this->open_drawer_subscription_ = this->create_subscription<DrawerAddress>(
        "open_drawer",
        qos_subscriptions,
        std::bind(&DrawerGate::open_drawer_topic_callback, this, std::placeholders::_1));

    this->drawer_leds_subscription_ = this->create_subscription<DrawerLeds>(
        "drawer_leds",
        qos_subscriptions,
      std::bind(&DrawerGate::drawer_leds_topic_callback, this, std::placeholders::_1));

    this->can_message_subscription= this->create_subscription<CanMessage>(
        "/from_can_bus",
        qos_subscriptions,
        std::bind(&DrawerGate::receive_can_msg_callback, this, std::placeholders::_1));
  }

  void DrawerGate::setup_publishers()
  {
    auto qos_publishers = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));

    this->drawer_status_publisher_ = this->create_publisher<DrawerStatus>("drawer_is_open", qos_publishers);

    this->can_message_publisher_ = this->create_publisher<CanMessage>("/to_can_bus", qos_publishers);
  }

  void DrawerGate::setup_services()
  {
    this->shelf_setup_info_service_ = this->create_service<ShelfSetupInfo>(
    "shelf_setup_info",
    std::bind(&DrawerGate::provide_shelf_setup_info_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void DrawerGate::open_drawer_topic_callback(const DrawerAddress& msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "I heard from open_drawer topic the drawer_controller_id: '%i'",
                msg.drawer_controller_id);   // Debugging

    uint32_t drawer_controller_id = msg.drawer_controller_id;
    uint8_t drawer_id = msg.drawer_id;

    if (drawer_controller_id == 0 && drawer_id == 0)
    {
      return;
    }
    robast_can_msgs::CanMessage can_msg_open_lock =
        this->create_can_msg_drawer_lock(drawer_controller_id, drawer_id, CAN_DATA_OPEN_LOCK);


    this->send_can_msg(can_encoder_decoder_.encode_msg(can_msg_open_lock));
  }

  void DrawerGate::drawer_leds_topic_callback(const DrawerLeds& msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard from drawer_leds topic the led mode: '%i'", msg.mode);   // Debugging

    led_parameters led_parameters = {};
    led_parameters.led_red = msg.red;
    led_parameters.led_blue = msg.blue;
    led_parameters.led_green = msg.green;
    led_parameters.brightness = msg.brightness;
    led_parameters.mode = msg.mode;

    robast_can_msgs::CanMessage can_msg_drawer_led =
        this->create_can_msg_drawer_led(msg.drawer_address.drawer_controller_id, led_parameters);

    this->send_can_msg(can_encoder_decoder_.encode_msg(can_msg_drawer_led));
  }

  void DrawerGate::update_drawer_status(std::vector<robast_can_msgs::CanMessage> drawer_feedback_can_msgs)
  {
    for (uint8_t i = 0; i < drawer_feedback_can_msgs.size(); i++)
    {
      if (drawer_feedback_can_msgs.at(i).get_id() == CAN_ID_DRAWER_FEEDBACK)
      {
        uint32_t drawer_controller_id =
            drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).get_data();
        drawer_status drawer_status = {};
        drawer_status.is_endstop_switch_1_pushed =
            drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED).get_data() == 1;
        drawer_status.is_lock_switch_1_pushed =
            drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED).get_data() == 1;
        drawer_status.is_endstop_switch_2_pushed =
            drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED).get_data() == 1;
        drawer_status.is_lock_switch_2_pushed =
            drawer_feedback_can_msgs.at(i).get_can_signals().at(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED).get_data() == 1;
        this->drawer_status_by_drawer_controller_id_[drawer_controller_id] = drawer_status;

        communication_interfaces::msg::DrawerStatus drawer_status_msg = DrawerStatus();
        drawer_status_msg.drawer_address.drawer_controller_id = drawer_controller_id;
        if (!drawer_status.is_endstop_switch_1_pushed)
        {
          this->send_drawer_is_open_feedback(drawer_status_msg, 1);
        }
        if (!drawer_status.is_lock_switch_1_pushed && drawer_status.is_endstop_switch_1_pushed)
        {
          this->send_drawer_is_closed_feedback(drawer_status_msg, 1);
        }

        // TODO: If lock 2 is used as well, add the if (drawer_status.is_endstop_switch_2_pushed) block
      }
    }
  }

  void DrawerGate::send_drawer_is_open_feedback(communication_interfaces::msg::DrawerStatus drawer_status_msg,
                                                uint8_t drawer_id)
  {
    RCLCPP_INFO(this->get_logger(),
                "Sending send_drawer_is_open_feedback with drawer_controller_id: '%i'",
                drawer_status_msg.drawer_address.drawer_controller_id);   // Debugging
    drawer_status_msg.drawer_address.drawer_id = drawer_id;
    drawer_status_msg.drawer_is_open = true;
    this->drawer_status_publisher_->publish(drawer_status_msg);
  }

  void DrawerGate::send_drawer_is_closed_feedback(communication_interfaces::msg::DrawerStatus drawer_status_msg,
                                                  uint8_t drawer_id)
  {
    RCLCPP_INFO(this->get_logger(),
                "Sending send_drawer_is_closed_feedback with drawer_controller_id: '%i'",
                drawer_status_msg.drawer_address.drawer_controller_id);   // Debugging
    drawer_status_msg.drawer_address.drawer_id = drawer_id;
    drawer_status_msg.drawer_is_open = false;
    this->drawer_status_publisher_->publish(drawer_status_msg);
    this->receive_can_msgs_timer_->cancel();
  }





  std::vector<communication_interfaces::msg::Drawer> DrawerGate::get_all_mounted_drawers()
  {
    // TODO: This should actually be done automatically by polling all drawer_controller on the CAN bus
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

    return {drawer_1, drawer_2, drawer_3, drawer_4, drawer_5};
  }

  void DrawerGate::provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request,
                                                     std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    response->drawers = this->get_all_mounted_drawers();
  }

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_lock(uint32_t drawer_controller_id,
                                                                     uint8_t drawer_id,
                                                                     uint8_t can_data_open_lock) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_lock = this->can_db_.can_messages.at(CAN_MSG_DRAWER_LOCK);

    std::vector<robast_can_msgs::CanSignal> can_signals_drawer_lock = can_msg_drawer_lock.get_can_signals();

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

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_led(uint32_t drawer_controller_id,
                                                                    led_parameters led_parameters) const
  {
    robast_can_msgs::CanMessage can_msg_drawer_led = this->can_db_.can_messages.at(CAN_MSG_DRAWER_LED);

    std::vector<robast_can_msgs::CanSignal> can_signals_drawer_led = can_msg_drawer_led.get_can_signals();

    can_signals_drawer_led.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(drawer_controller_id);

    can_signals_drawer_led.at(CAN_SIGNAL_LED_RED).set_data(led_parameters.led_red);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_GREEN).set_data(led_parameters.led_green);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_BLUE).set_data(led_parameters.led_blue);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_BRIGHTNESS).set_data(led_parameters.brightness);
    can_signals_drawer_led.at(CAN_SIGNAL_LED_MODE).set_data(led_parameters.mode);

    can_msg_drawer_led.set_can_signals(can_signals_drawer_led);

    return can_msg_drawer_led;
  }



  
  void DrawerGate::receive_can_msg_callback(CanMessage can_message)
  {
       RCLCPP_INFO(this->get_logger(), "Received: id:'%d' dlc:'%d' \n ", can_message.id,can_message.dlc);
  }

  void DrawerGate::send_can_msg(CanMessage can_message)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'\n ", can_message.id);
    
    can_message_publisher_->publish(can_message);
  }

}   // namespace drawer_gate
