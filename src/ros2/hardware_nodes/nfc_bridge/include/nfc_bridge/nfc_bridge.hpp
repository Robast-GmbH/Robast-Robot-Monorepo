#ifndef HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_
#define HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL              33
#define CHECK_ON_DB                  false

#include <string.h>

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "serial_helper/serial_helper.hpp"
#include "twn4.hpp"
// #include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace nfc_bridge
{
  class NFCBridge : public rclcpp::Node
  {
   public:
    NFCBridge();
    ~NFCBridge();

    friend class TestNFCBridge;   // this class has full access to all private and protected parts of this class

   private:
    std::unique_ptr<serial_helper::ISerialHelper> _serial_connector;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _nfc_key_publisher;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _trigger_subscriber;

    static const Twn4Elatec _twn4_msg_converter;

    void trigger_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void setup_subscriber();
    void start_up_scanner();
    void shutdown_scanner();
    bool wait_for_tag();
    bool read_nfc_code();

    // void publish_nfc_key(std::string nfc_key);
    // void send_cmd(std::string cmd);
  };

}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_