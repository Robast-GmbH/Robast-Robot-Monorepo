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

    _nfc_key_publisher = create_publisher<std_msgs::msg::String>("/nfc_key", qos);
    setup_subscriber();
  }

  NFCBridge::~NFCBridge()
  {
    shutdown_scanner();
  }

  void NFCBridge::trigger_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    read_nfc_code();
  }

  void NFCBridge::setup_subscriber()
  {
    _trigger_subscriber = this->create_subscription<std_msgs::msg::Empty>(
        "trigger_topic", 10, std::bind(&NFCBridge::trigger_callback, this, std::placeholders::_1));
    start_up_scanner();
    _serial_connector->send_ascii_cmd(Twn4Elatec::BeepReq(0x10, 0x6009, 0xF401, 0xF401));
    // _serial_connector->send_ascii_cmd("0407646009F401F401");
    shutdown_scanner();
  }

  void NFCBridge::start_up_scanner()
  {
    _serial_connector->open_serial();
  }

  void NFCBridge::shutdown_scanner()
  {
    _serial_connector->close_serial();
  }

  bool NFCBridge::wait_for_tag()
  {
    std::string response = "";
    int length = 0;
    while (length < 6)
    {
      //"00 01 80 20 04 DEAB9B03"
      //"00 01 80 38 07 04D03501700703"
      length = _serial_connector->ascii_interaction(Twn4Elatec::SearchTagReq(0x10), response);
      if (length < 6)
      {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        response = "";
        length = _serial_connector->read_serial(response, 100);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }
    return true;
  }

  bool NFCBridge::read_nfc_code()
  {
    start_up_scanner();
    if (wait_for_tag())
    {
      std::array<uint8_t, 16> data;
      std::string response = "";
      int length = 0;
      while (length < 6)
      {
        length = _serial_connector->ascii_interaction(Twn4Elatec::NTAGReadReq(0x04), response);
        if (length < 6)
        {
          rclcpp::sleep_for(std::chrono::milliseconds(100));
          response = "";
          length = _serial_connector->read_serial(response, 100);
        }
      }
      if (length > 5)
      {
        //"00010102030401020304", '0' <repeats 16 times>, "\r"
        uint8_t result;
        Twn4Elatec::NTAGReadResp(response, result, data);
        if (result == 1)
        {
          std::string nfc_key = "";
          for (auto byte : data)
          {
            nfc_key += byte;
          }
          std_msgs::msg::String msg;
          msg.data = nfc_key;
          _nfc_key_publisher->publish(msg);
          shutdown_scanner();
          return true;
        }
      }
    }
    shutdown_scanner();
    return false;
  }

}   // namespace nfc_bridge