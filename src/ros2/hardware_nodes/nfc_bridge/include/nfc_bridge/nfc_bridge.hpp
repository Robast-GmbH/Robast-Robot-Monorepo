#ifndef HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_
#define HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL              33
#define CHECK_ON_DB                  false

#include <string.h>

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "serial_helper/serial_helper.h"
#include "std_msgs/msg/bool.hpp"
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
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _nfc_key_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _timer_subscriber;

    void start_up_scanner();
    void shutdown_scanner();

    void reading_procedure();
    bool read_nfc_code(std::shared_ptr<std::string> scanned_key);
    // void createUser(const std::shared_ptr<GoalHandleCreateUser> goal_handle);

    void toggle_NFC_Reader_State(const std_msgs::msg::Bool::SharedPtr msg);
    void timer_start();
    void timer_stop();
  };

}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_