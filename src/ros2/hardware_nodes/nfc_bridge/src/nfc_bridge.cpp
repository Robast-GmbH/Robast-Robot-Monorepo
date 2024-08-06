#include "nfc_bridge/nfc_bridge.hpp"

namespace nfc_bridge
{

  NFCBridge::NFCBridge() : Node("nfc_bridge")
  {
    declare_parameter("serial_port_path", "/dev/robast/robast_nfc");
    std::string serial_port_path = get_parameter("serial_port_path").as_string();

    _serial_connector = std::make_unique<serial_helper::SerialHelper>(serial_helper::SerialHelper(serial_port_path));

    boot_beep();
    _write_service = this->create_service<communication_interfaces::srv::WriteNfcTag>(
        "write_nfc", std::bind(&NFCBridge::write_nfc_callback, this, std::placeholders::_1, std::placeholders::_2));
    _read_service = this->create_service<communication_interfaces::srv::ReadNfcTag>(
        "read_nfc", std::bind(&NFCBridge::read_nfc_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  NFCBridge::~NFCBridge()
  {
    shutdown_scanner();
  }

  void NFCBridge::boot_beep()
  {
    start_up_scanner();
    std::string response = "";
    _serial_connector->ascii_interaction(Twn4Elatec::beep_req(0x10, 0x6009, 0xF401, 0xF401), response);
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
    u_int8_t result = 0;
    int max_iterations = 100;
    while (result != 1 && max_iterations-- >= 1)
    {
      std::string tmp = "";
      _serial_connector->ascii_interaction(Twn4Elatec::search_tag_req(0x10), response);
      Twn4Elatec::seatch_tag_resp(response, result, tmp);
      rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
    return result == Twn4Elatec::ResultOK;
  }

  bool NFCBridge::read_nfc_code(std::string& nfc_key, int max_iterations)
  {
    if(max_iterations < 0)
    {
      return false;
    }
    start_up_scanner();
    if (!wait_for_tag())
    {
      shutdown_scanner();
      return false;
    }

    std::array<uint8_t, 16> data;
    std::string response = "";
    uint8_t result = 0;

    while (result != 1 && max_iterations-- >= 1)
    {
      _serial_connector->ascii_interaction(Twn4Elatec::ntag_read_req(0x04), response);
      Twn4Elatec::ntag_read_resp(response, result, data, nfc_key);
    }
    shutdown_scanner();
    return result == Twn4Elatec::ResultOK;
  }

  bool NFCBridge::write_nfc_code(const std::string nfc_key, const std::string nfc_tag_type, int max_iterations)
  {
    if(max_iterations < 0)
    {
      return false;
    }
    start_up_scanner();
    if (!wait_for_tag())
    {
      shutdown_scanner();
      return false;
    }
    for (int j = 0; j < 4 && j < nfc_key.length() / 8; ++j)
    {
      std::array<uint8_t, 4> data;
      for (int i = 0; i < 4; i++)
      {
        std::string byte_str = nfc_key.substr((i + j * 4) * 2, 2);
        data[i] = std::stoi(byte_str, nullptr, 16);
      }
      std::string response = "";
      u_int8_t result = 0;
      // TODO(@TAlscher): Use result for feedback.
      while (result != 1)
      {
        _serial_connector->ascii_interaction(Twn4Elatec::ntag_write_req(0x04 + j, data), response);
        Twn4Elatec::ntag_write_resp(response, result);
        if (max_iterations-- <= 1)
        {
          shutdown_scanner();
          return false;
        }
      }
    }

    shutdown_scanner();
    return true;
  }

  void NFCBridge::write_nfc_callback(const std::shared_ptr<communication_interfaces::srv::WriteNfcTag::Request> request,
                                     std::shared_ptr<communication_interfaces::srv::WriteNfcTag::Response> response)
  {
    // Implement your logic to write data to an NFC tag
    response->success = write_nfc_code(request->nfc_tag_id, "", request->timout_in_s*2);
  }

  void NFCBridge::read_nfc_callback(const std::shared_ptr<communication_interfaces::srv::ReadNfcTag::Request> request,
                                    std::shared_ptr<communication_interfaces::srv::ReadNfcTag::Response> response)
  {
    std::string nfc_key = "";
    if (read_nfc_code(nfc_key, request->timout_in_s*2))
    {
      response->nfc_tag_id = nfc_key;
    }
    else
    {
      response->nfc_tag_id = "";
    }
  }

}   // namespace nfc_bridge