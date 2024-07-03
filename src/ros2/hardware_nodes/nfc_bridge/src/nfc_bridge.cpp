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

    _nfc_key_publisher = this->create_publisher<std_msgs::msg::String>("/nfc_key", qos);
    _timer_subscriber = this->create_subscription<std_msgs::msg::Bool>(
        "/nfc_switch", qos, std::bind(&NFCBridge::toggle_NFC_Reader_State, this, std::placeholders::_1));
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
}   // namespace nfc_bridge