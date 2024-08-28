#ifndef HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_
#define HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL              33
#define CHECK_ON_DB                  false

#include <string.h>

#include <memory>
#include <thread>

#include "communication_interfaces/srv/read_nfc_ntag_payload.hpp"
#include "communication_interfaces/srv/read_nfc_tag.hpp"
#include "communication_interfaces/srv/write_nfc_ntag_payload.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_helper/serial_helper.hpp"
#include "std_msgs/msg/string.hpp"
#include "twn4.hpp"

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
    rclcpp::Service<communication_interfaces::srv::WriteNfcNtagPayload>::SharedPtr _write_payload_service;
    rclcpp::Service<communication_interfaces::srv::ReadNfcNtagPayload>::SharedPtr _read_payload_service;
    rclcpp::Service<communication_interfaces::srv::ReadNfcTag>::SharedPtr _read_service;

    static const Twn4Elatec _twn4_msg_converter;
    const uint8_t _nfc_buffer_read_freq = 2;

    void boot_beep();
    void start_up_scanner();
    void shutdown_scanner();
    bool wait_for_tag(std::string& tag_id, uint32_t max_iterations = 100);
    bool read_nfc_code(std::string& nfc_code, const uint32_t max_iterations = 100);
    bool read_nfc_payload(std::string& nfc_code, const uint32_t max_iterations = 100);
    bool write_nfc_payload(const std::string nfc_key,
                           const std::string nfc_tag_type = "",
                           const uint32_t max_iterations = 20);

    void write_nfc_payload_callback(
        const std::shared_ptr<communication_interfaces::srv::WriteNfcNtagPayload::Request> request,
        std::shared_ptr<communication_interfaces::srv::WriteNfcNtagPayload::Response> response);

    void read_nfc_payload_callback(
        const std::shared_ptr<communication_interfaces::srv::ReadNfcNtagPayload::Request> request,
        std::shared_ptr<communication_interfaces::srv::ReadNfcNtagPayload::Response> response);

    void read_nfc_callback(const std::shared_ptr<communication_interfaces::srv::ReadNfcTag::Request> request,
                           std::shared_ptr<communication_interfaces::srv::ReadNfcTag::Response> response);
    uint32_t calculate_max_iterations(uint16_t timeout_in_s);
  };

}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__NFC_BRIDGE__NFC_BRIDGE_HPP_