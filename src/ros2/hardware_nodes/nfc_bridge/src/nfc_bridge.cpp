#include "nfc_bridge/nfc_bridge.hpp"

namespace nfc_bridge
{

  NFCBridge::NFCBridge() : Node("nfc_bridge")
  {
    this->declare_parameter("serial_port_path", "/dev/robast/robast_nfc");
    std::string serial_port_path = this->get_parameter("serial_port_path").as_string();

    this->_serial_connector =
        std::make_unique<serial_helper::SerialHelper>(serial_helper::SerialHelper(serial_port_path));
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    _nfc_key_publisher = this->create_publisher<std_msgs::msg::String>("/nfc_key", qos);
    _timer_subscriber = this->create_subscription<std_msgs::msg::Bool>(
        "/nfc_switch", qos, std::bind(&NFCBridge::toggle_NFC_Reader_State, this, std::placeholders::_1));

    _write_nfc_tag_service = this->create_service<communication_interfaces::srv::WriteNFCTag>(
        "/write_nfc_tag", std::bind(&NFCBridge::write_nfc_tag, this, std::placeholders::_1, std::placeholders::_2));
  }

  rclcpp_action::GoalResponse NFCBridge::handle_goal(const rclcpp_action::GoalUUID&,
                                                     std::shared_ptr<const CreateUser::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse NFCBridge::handle_cancel(const std::shared_ptr<GoalHandleCreateUser> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void NFCBridge::handle_accepted(const std::shared_ptr<GoalHandleCreateUser> goal_handle)
  {
    // std::thread{std::bind(&NFCBridge::createUser, this, std::placeholders::_1), goal_handle}.detach();
  }

  void NFCBridge::toggle_NFC_Reader_State(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      this->timer_start();
    }
    else
    {
      this->timer_stop();
      // write_tag(10);
      send_nfc_cmd(NFC_READ_MC("02"));
    }
  }

  void NFCBridge::timer_start()
  {
    _timer = this->create_wall_timer(std::chrono::milliseconds(READER_INTEVALL),
                                     std::bind(&NFCBridge::reading_procedure, this));
  }

  void NFCBridge::timer_stop()
  {
    if (this->_timer)
    {
      this->_timer->cancel();
    }
  }

  NFCBridge::~NFCBridge()
  {
    shutdown_scanner();
  }

  void NFCBridge::start_up_scanner()
  {
    this->_serial_connector->open_serial();
  }

  void NFCBridge::shutdown_scanner()
  {
    this->_serial_connector->close_serial();
  }

  bool NFCBridge::read_nfc_code(std::shared_ptr<std::string> scanned_key)
  {
    int size_of_received_data = _serial_connector->read_serial(scanned_key.get(), 100);
    return size_of_received_data > 0;
  }

  void NFCBridge::reading_procedure()
  {
    start_up_scanner();
    std::shared_ptr<std::string> scanned_key = std::make_shared<std::string>();
    if (read_nfc_code(scanned_key))
    {
      std_msgs::msg::String nfc_msg;
      nfc_msg.data = *scanned_key;
      _nfc_key_publisher->publish(nfc_msg);
    }
    shutdown_scanner();
  }

  void NFCBridge::write_nfc_tag(const std::shared_ptr<communication_interfaces::srv::WriteNFCTag::Request> request,
                                std::shared_ptr<communication_interfaces::srv::WriteNFCTag::Response> response)
  {
    start_up_scanner();
    _serial_connector->write_serial(request->nfc_tag_id);
    shutdown_scanner();
    response->success = true;
  }

  bool NFCBridge::wait_for_tag()
  {
    start_up_scanner();
    std::string tag;
    // wait for the Tag and read TAG ID
    do   // search for a tag with the length of 10
    {
      this->_serial_connector->send_ascii_cmd(SEARCH_TAG);
      if (this->_serial_connector->read_serial(&tag, 100) <= 0)
      {
        continue;
      }
      // RCLCPP_INFO(this->get_logger(),"Received message: %s ", tag.c_str() );
    } while (tag.length() < 10);
    shutdown_scanner();
    return true;
  }

  void NFCBridge::send_nfc_cmd(std::string cmd)
  {
    start_up_scanner();
    this->_serial_connector->send_ascii_cmd(cmd);
    for (size_t i = 0; i < 100; i++)
    {
      read_nfc_code(std::make_shared<std::string>());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    shutdown_scanner();
  }

  bool NFCBridge::write_tag(int card_data)
  {
    start_up_scanner();

    std::string tag;
    wait_for_tag();
    this->_serial_connector->send_ascii_cmd(NFC_LOGIN_MC_STANDARD("00"));
    this->_serial_connector->send_ascii_cmd(NFC_WRITE_MC("02", std::bitset<BLOCK_SIZE>(card_data).to_string()));
    if (this->_serial_connector->read_serial(&tag, 100) <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "writing probably failed");
      shutdown_scanner();
      return false;
    }

    shutdown_scanner();
    return true;
  }

}   // namespace nfc_bridge