#include "nfc_bridge/nfc_bridge.hpp"

namespace nfc_bridge
{

  NFCBridge::NFCBridge() : Node("nfc_bridge")
  {
    declare_parameter("serial_port_path", "/dev/robast/robast_nfc");
    std::string serial_port_path = get_parameter("serial_port_path").as_string();

    _serial_connector = std::make_unique<serial_helper::SerialHelper>(serial_helper::SerialHelper(serial_port_path));

    liveliness_ckeck();
    _write_service = this->create_service<communication_interfaces::srv::WriteNfcTag>(
        "write_nfc", std::bind(&NFCBridge::write_nfc_callback, this, std::placeholders::_1, std::placeholders::_2));
    _read_service = this->create_service<communication_interfaces::srv::ReadNfcTag>(
        "read_nfc", std::bind(&NFCBridge::read_nfc_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  NFCBridge::~NFCBridge()
  {
    shutdown_scanner();
  }

  void NFCBridge::liveliness_ckeck()
  {
    start_up_scanner();
    std::string response = "";
    _serial_connector->ascii_interaction(Twn4Elatec::BeepReq(0x10, 0x6009, 0xF401, 0xF401), response);
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
    while (result != 1)
    {
      std::string tmp = "";
      _serial_connector->ascii_interaction(Twn4Elatec::SearchTagReq(0x10), response);
      Twn4Elatec::SearchTagResp(response, result, tmp);
      rclcpp::sleep_for(std::chrono::milliseconds(300));
      if (max_iterations-- <= 1)
      {
        return false;
      }
    }
    return true;
  }

  bool NFCBridge::read_nfc_code(std::string& nfc_key)
  {
    start_up_scanner();
    if (!wait_for_tag())
    {
      shutdown_scanner();
      return false;
    }

    std::array<uint8_t, 16> data;
    std::string response = "";
    u_int8_t result = 0;
    int max_iterations = 100;

    while (result != 1)
    {
      _serial_connector->ascii_interaction(Twn4Elatec::NTAGReadReq(0x04), response);
      Twn4Elatec::NTAGReadResp(response, result, data, nfc_key);
      if (max_iterations-- <= 1)
      {
        shutdown_scanner();
        return false;
      }
    }
    shutdown_scanner();
    return true;
  }

  bool NFCBridge::write_nfc_code(const std::string nfc_key, const std::string nfc_tag_type)
  {
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
      int max_iterations = 20;

      while (result != 1)
      {
        _serial_connector->ascii_interaction(Twn4Elatec::NTAGWriteReq(0x04 + j, data), response);
        Twn4Elatec::NTAGWriteResp(response, result);
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
    response->success = write_nfc_code(request->nfc_tag_id, "");
  }

  void NFCBridge::read_nfc_callback(
      const std::shared_ptr<communication_interfaces::srv::ReadNfcTag::Request> request_header,
      std::shared_ptr<communication_interfaces::srv::ReadNfcTag::Response> response)
  {
    std::string nfc_key = "";
    if (read_nfc_code(nfc_key))
    {
      response->nfc_tag_id = nfc_key;
    }
    else
    {
      response->nfc_tag_id = "";
    }
  }

}   // namespace nfc_bridge