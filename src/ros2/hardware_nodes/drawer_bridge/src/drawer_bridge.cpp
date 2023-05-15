#include "drawer_bridge/drawer_bridge.hpp"

// For DEBUGGING purposes this is the action send_goal command:
// ros2 action send_goal /control_drawer communication_interfaces/action/DrawerUserAccess "{drawer_address:
// {drawer_controller_id: 1, drawer_id: 1}, state: 1}"

namespace drawer_bridge
{
  DrawerBridge::DrawerBridge() : Node("drawer_bridge")
  {
    setup_subscriptions();
    setup_publishers();
    setup_services();
  }

  void DrawerBridge::setup_subscriptions()
  {
    open_drawer_subscription_ = this->create_subscription<DrawerAddress>(
        "open_drawer",
        qos_config.get_qos_open_drawer(),
        std::bind(&DrawerBridge::open_drawer_topic_callback, this, std::placeholders::_1));

    drawer_task_subscription_ = this->create_subscription<DrawerTask>(
        "electrical_drawer_task",
        qos_config.get_qos_open_drawer(),
        std::bind(&DrawerBridge::electrical_drawer_task_topic_callback, this, std::placeholders::_1));

    drawer_leds_subscription_ = this->create_subscription<DrawerLeds>(
        "drawer_leds",
        qos_config.get_qos_drawer_leds(),
        std::bind(&DrawerBridge::drawer_leds_topic_callback, this, std::placeholders::_1));

    can_messages_subscription_ = this->create_subscription<CanMessage>(
        "/from_can_bus",
        qos_config.get_qos_can_messages(),
        std::bind(&DrawerBridge::receive_can_msg_callback, this, std::placeholders::_1));
  }

  void DrawerBridge::setup_publishers()
  {
    can_messages_publisher_ = create_publisher<CanMessage>("/to_can_bus", qos_config.get_qos_can_messages());

    drawer_status_publisher_ = create_publisher<DrawerStatus>("drawer_is_open", qos_config.get_qos_open_drawer());

    electrical_drawer_status_publisher_ =
        create_publisher<ElectricalDrawerStatus>("electrical_drawer_status", qos_config.get_qos_open_drawer());
  }

  void DrawerBridge::setup_services()
  {
    shelf_setup_info_service_ = create_service<ShelfSetupInfo>(
        "shelf_setup_info",
        std::bind(
            &DrawerBridge::provide_shelf_setup_info_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void DrawerBridge::open_drawer_topic_callback(const DrawerAddress& msg)
  {
    uint32_t module_id = msg.module_id;
    uint8_t drawer_id = msg.drawer_id;

    RCLCPP_INFO(get_logger(),
                "I heard from open_drawer topic the module_id: '%i' drawer_id: '%d ",
                module_id,
                drawer_id);   // Info

    if (module_id != 0)
    {
      const CanMessage can_msg = can_message_creator_.create_can_msg_drawer_unlock(msg);
      send_can_msg(can_msg);
    }
  }

  void DrawerBridge::electrical_drawer_task_topic_callback(const DrawerTask& msg)
  {
    RCLCPP_INFO(get_logger(), "I heard from drawer task topic");

    const CanMessage can_msg = can_message_creator_.create_can_msg_drawer_task(msg);
    send_can_msg(can_msg);
  }

  void DrawerBridge::drawer_leds_topic_callback(const DrawerLeds& msg)
  {
    RCLCPP_INFO(get_logger(), "I heard from drawer_leds topic the led mode: '%i'", msg.mode);   // Debugging

    const CanMessage can_msg = can_message_creator_.create_can_msg_drawer_led(msg);
    send_can_msg(can_msg);
  }

  void DrawerBridge::publish_drawer_status(robast_can_msgs::CanMessage drawer_feedback_can_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = drawer_feedback_can_msg.get_can_signals();

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.module_id = can_signals.at(CAN_SIGNAL_MODULE_ID).get_data();
    drawer_address.drawer_id = can_signals.at(CAN_SIGNAL_DRAWER_ID).get_data();

    const bool is_endstop_switch_pushed =
        can_signals.at(CAN_SIGNAL_IS_ENDSTOP_SWITCH_PUSHED).get_data() == CAN_DATA_SWITCH_IS_PUSHED;
    const bool is_lock_switch_pushed =
        can_signals.at(CAN_SIGNAL_IS_LOCK_SWITCH_PUSHED).get_data() == CAN_DATA_SWITCH_IS_PUSHED;

    DrawerStatus drawer_status_msg = DrawerStatus();
    drawer_status_msg.drawer_address = drawer_address;

    if (!is_endstop_switch_pushed)
    {
      drawer_status_msg.drawer_is_open = true;
      this->drawer_status_publisher_->publish(drawer_status_msg);
      // Debugging
      RCLCPP_INFO(this->get_logger(),
                  "Sending send_drawer_is_open_feedback with module_id: '%i'",
                  drawer_status_msg.drawer_address.module_id);
    }

    if (!is_lock_switch_pushed && is_endstop_switch_pushed)
    {
      drawer_status_msg.drawer_is_open = false;
      this->drawer_status_publisher_->publish(drawer_status_msg);
      // Debugging
      RCLCPP_INFO(this->get_logger(),
                  "Sending send_drawer_is_closed_feedback with module_id: '%i'",
                  drawer_status_msg.drawer_address.module_id);
    }
  }

  void DrawerBridge::publish_electrical_drawer_status(robast_can_msgs::CanMessage electrical_drawer_feedback_can_msg)
  {
    (void) electrical_drawer_feedback_can_msg;
    // ElectricalDrawerStatus status = ElectricalDrawerStatus();
    // DrawerAddress address = DrawerAddress();

    // electrical_drawer_status_publisher_->publish(status);
  }

  void DrawerBridge::provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request,
                                                       std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    (void) request;
    response->modules = ShelfSetup::get_all_mounted_drawers();
  }

  void DrawerBridge::receive_can_msg_callback(CanMessage can_message)
  {
    RCLCPP_INFO(this->get_logger(), "Received: id:'%d' dlc:'%d' \n ", can_message.id, can_message.dlc);

    switch (can_message.id)
    {
      case CAN_ID_DRAWER_FEEDBACK:
      {
        const std::optional<robast_can_msgs::CanMessage> decoded_msg = can_encoder_decoder_.decode_msg(can_message);
        if (decoded_msg.has_value())
        {
          publish_drawer_status(decoded_msg.value());
        }
      }
      break;
      case CAN_ID_ELECTRICAL_DRAWER_FEEDBACK:
      {
        const std::optional<robast_can_msgs::CanMessage> decoded_msg = can_encoder_decoder_.decode_msg(can_message);
        if (decoded_msg.has_value())
        {
          publish_electrical_drawer_status(decoded_msg.value());
        }
      }
      break;
    }
  }

  void DrawerBridge::send_can_msg(CanMessage can_message)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'\n ", can_message.id);

    can_messages_publisher_->publish(can_message);
  }

}   // namespace drawer_bridge
