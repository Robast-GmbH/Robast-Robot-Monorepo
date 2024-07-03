#include "drawer_bridge/drawer_bridge.hpp"

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
    _open_drawer_subscription = this->create_subscription<DrawerAddress>(
      "open_drawer",
      _qos_config.get_qos_open_drawer(),
      std::bind(&DrawerBridge::open_drawer_topic_callback, this, std::placeholders::_1));

    _drawer_task_subscription = this->create_subscription<DrawerTask>(
      "electrical_drawer_task",
      _qos_config.get_qos_open_drawer(),
      std::bind(&DrawerBridge::electrical_drawer_task_topic_callback, this, std::placeholders::_1));

    _led_cmd_subscription =
      this->create_subscription<LedCmd>("led_cmd",
                                        _qos_config.get_qos_led_cmd(),
                                        std::bind(&DrawerBridge::led_cmd_topic_callback, this, std::placeholders::_1));

    _tray_task_subscription = this->create_subscription<TrayTask>(
      "tray_task",
      _qos_config.get_qos_open_drawer(),
      std::bind(&DrawerBridge::tray_task_topic_callback, this, std::placeholders::_1));

    _can_messages_subscription = this->create_subscription<CanMessage>(
      "from_can_bus",
      _qos_config.get_qos_can_messages(),
      std::bind(&DrawerBridge::receive_can_msg_callback, this, std::placeholders::_1));
  }

  void DrawerBridge::setup_publishers()
  {
    _can_msg_publisher = create_publisher<can_msgs::msg::Frame>("to_can_bus", 10);

    _drawer_status_publisher = create_publisher<DrawerStatus>("drawer_is_open", _qos_config.get_qos_open_drawer());

    _electrical_drawer_status_publisher =
      create_publisher<ElectricalDrawerStatus>("electrical_drawer_status", _qos_config.get_qos_open_drawer());

    _error_msg_publisher = create_publisher<ErrorBaseMsg>("robast_error", _qos_config.get_qos_error_msgs());
  }

  void DrawerBridge::setup_services()
  {
    _shelf_setup_info_service = create_service<ShelfSetupInfo>(
      "shelf_setup_info",
      std::bind(&DrawerBridge::provide_shelf_setup_info_callback, this, std::placeholders::_1, std::placeholders::_2));
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
      const CanMessage can_msg = _can_message_creator.create_can_msg_drawer_unlock(msg);
      send_can_msg(can_msg);
    }
  }

  void DrawerBridge::electrical_drawer_task_topic_callback(const DrawerTask& msg)
  {
    uint32_t module_id = msg.drawer_address.module_id;
    uint8_t drawer_id = msg.drawer_address.drawer_id;
    uint8_t target_pos = msg.target_position;

    RCLCPP_INFO(get_logger(),
                "I heard from electrical_drawer_task topic the module_id: '%i' drawer_id: '%d with target position: %d",
                module_id,
                drawer_id,
                target_pos);

    const CanMessage can_msg = _can_message_creator.create_can_msg_drawer_task(msg);
    send_can_msg(can_msg);
  }

  void DrawerBridge::led_cmd_topic_callback(const LedCmd& msg)
  {
    uint16_t num_of_leds = msg.leds.size();

    RCLCPP_INFO(get_logger(),
                "I heard from the /led_cmd topic the module id %i and the number of led states = %i. The states for "
                "the first led are red = %i, green = %i, blue = %i, brightness = %i",
                msg.drawer_address.module_id,
                num_of_leds,
                msg.leds[0].red,
                msg.leds[0].green,
                msg.leds[0].blue,
                msg.leds[0].brightness);

    const CanMessage can_msg = _can_message_creator.create_can_msg_led_header(msg);
    send_can_msg(can_msg);

    for (uint16_t i = 0; i < num_of_leds; i++)
    {
      const CanMessage can_msg =
        _can_message_creator.create_can_msg_set_single_led_state(msg.leds[i], msg.drawer_address);
      send_can_msg(can_msg);
    }
  }

  void DrawerBridge::tray_task_topic_callback(const TrayTask& msg)
  {
    uint8_t num_of_leds = msg.led_brightness.size();

    for (uint8_t i = 1; i <= num_of_leds; i++)
    {
      const CanMessage can_msg =
        _can_message_creator.create_can_msg_tray_led_brightness(msg.drawer_address, i, msg.led_brightness[i]);
      send_can_msg(can_msg);
    }
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
      this->_drawer_status_publisher->publish(drawer_status_msg);
      // Debugging
      RCLCPP_INFO(this->get_logger(),
                  "Sending send_drawer_is_open_feedback with module_id: '%i'",
                  drawer_status_msg.drawer_address.module_id);
    }

    if (!is_lock_switch_pushed && is_endstop_switch_pushed)
    {
      drawer_status_msg.drawer_is_open = false;
      this->_drawer_status_publisher->publish(drawer_status_msg);
      // Debugging
      RCLCPP_INFO(this->get_logger(),
                  "Sending send_drawer_is_closed_feedback with module_id: '%i'",
                  drawer_status_msg.drawer_address.module_id);
    }
  }

  void DrawerBridge::publish_electrical_drawer_status(robast_can_msgs::CanMessage electrical_drawer_feedback_can_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = electrical_drawer_feedback_can_msg.get_can_signals();

    ElectricalDrawerStatus status = ElectricalDrawerStatus();

    status.drawer_address.module_id = can_signals.at(CAN_SIGNAL_MODULE_ID).get_data();
    status.drawer_address.drawer_id = can_signals.at(CAN_SIGNAL_DRAWER_ID).get_data();

    status.position = can_signals.at(CAN_SIGNAL_DRAWER_POSITION).get_data();

    _electrical_drawer_status_publisher->publish(status);
  }

  void DrawerBridge::publish_drawer_error_msg(robast_can_msgs::CanMessage drawer_error_feedback_can_msg)
  {
    std::vector<robast_can_msgs::CanSignal> can_signals = drawer_error_feedback_can_msg.get_can_signals();

    ErrorBaseMsg error_msg = ErrorBaseMsg();

    auto message_converter = MessageConverter<ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED_INTERFACE>();

    DrawerAddress drawer_address = DrawerAddress();
    drawer_address.module_id = can_signals.at(CAN_SIGNAL_MODULE_ID).get_data();
    drawer_address.drawer_id = can_signals.at(CAN_SIGNAL_DRAWER_ID).get_data();

    switch (can_signals.at(CAN_SIGNAL_ERROR_CODE).get_data())
    {
      case CAN_DATA_ERROR_CODE_TIMEOUT_DRAWER_NOT_OPENED:
        error_msg.error_code = ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED;
        error_msg.error_description =
          "The drawer was not opened and therefore a timeout occurred. Drawer is now locked again.";
        error_msg.error_data = message_converter.messageToString(drawer_address);
        _error_msg_publisher->publish(error_msg);
        break;

      default:
        break;
    }
  }

  void DrawerBridge::provide_shelf_setup_info_callback(const std::shared_ptr<ShelfSetupInfo::Request> request,
                                                       std::shared_ptr<ShelfSetupInfo::Response> response)
  {
    (void) request;
    response->modules = ShelfSetup::get_all_mounted_drawers();
  }

  void DrawerBridge::receive_can_msg_callback(CanMessage can_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received CAN message: id:'%d' dlc:'%d' \n ", can_msg.id, can_msg.dlc);

    const std::optional<robast_can_msgs::CanMessage> decoded_msg = _can_encoder_decoder.decode_msg(can_msg);

    if (decoded_msg.has_value())
    {
      switch (can_msg.id)
      {
        case CAN_ID_DRAWER_FEEDBACK:
          publish_drawer_status(decoded_msg.value());
          break;
        case CAN_ID_ELECTRICAL_DRAWER_FEEDBACK:
          publish_electrical_drawer_status(decoded_msg.value());
          break;
        case CAN_ID_ERROR_FEEDBACK:
          publish_drawer_error_msg(decoded_msg.value());
          break;
      }
    }
  }

  void DrawerBridge::send_can_msg(CanMessage can_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Sending can message with id '%d'!\n ", can_msg.id);

    _can_msg_publisher->publish(can_msg);
  }

}   // namespace drawer_bridge
