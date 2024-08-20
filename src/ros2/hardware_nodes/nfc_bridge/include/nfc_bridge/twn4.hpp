#ifndef NFC_BRIDGE_TWN4_HPP
#define NFC_BRIDGE_TWN4_HPP

/// TWN4 Elatec class for NFC communication with the TWN4 RFID reader
/// This class contains static functions for generating commands and parsing responses
/// You can find more details regarding the commands in the TWN4 Simple Protocol DocRev25.pdf

#include <array>
#include <iomanip>   // Include for std::setfill and std::setw
#include <iostream>
#include <sstream>     // Include for std::ostringstream
#include <stdexcept>   // Include for std::runtime_error
#include <string>

namespace nfc_bridge
{
#define TAG_TYPE_NTAG213       "04"
#define TAG_TYPE_NTAG215       "05"
#define TAG_TYPE_MIFARE_CLASIC "80"

  class Twn4Elatec
  {
   public:
    // Constants
    static const std::string FlagLEDOn;
    static const std::string SearchTagCmd;

    static const uint8_t ResultOK = 1;

    // Static request functions
    static std::string beep_req(uint8_t volume, uint16_t frequency, uint16_t onTime, uint16_t offTime);
    static std::string search_tag_req(uint8_t MaxIDBytes);

    static std::string ntag_read_payload_req(uint8_t page);
    static std::string ntag_write_payload_req(uint8_t page, const std::array<uint8_t, 4>& data);

    // Static response functions
    static void ntag_read_payload_resp(const std::string& response,
                                       uint8_t& result,
                                       std::array<uint8_t, 16>& data,
                                       std::string& nfc_key);
    static void ntag_write_payload_resp(const std::string& response, uint8_t& result);
    static void search_tag_resp(const std::string& response, uint8_t& result, std::string& tagType, std::string& tagID);
    static bool TagTypeValidation(const std::string& tagType);
  };

  // Initialize constants
  const std::string Twn4Elatec::FlagLEDOn = "0408";
  const std::string Twn4Elatec::SearchTagCmd = "";

  // Implement static function using std::ostringstream for formatting
  std::string Twn4Elatec::beep_req(uint8_t volume, uint16_t frequency, uint16_t onTime, uint16_t offTime)
  {
    std::ostringstream command;
    command << "0407" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(volume)
            << std::setw(4) << frequency << std::setw(4) << onTime << std::setw(4) << offTime;
    return command.str();
  }

  std::string Twn4Elatec::search_tag_req(uint8_t MaxIDBytes)
  {
    std::ostringstream command;
    command << "0500" << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
            << static_cast<int>(MaxIDBytes);
    return command.str();
  }

  std::string Twn4Elatec::ntag_read_payload_req(uint8_t page)
  {
    std::ostringstream command;
    command << "2000" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(page);
    return command.str();
  }

  std::string Twn4Elatec::ntag_write_payload_req(uint8_t page, const std::array<uint8_t, 4>& data)
  {
    std::ostringstream command;
    command << "2001" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(page);
    for (auto byte : data)
    {
      command << std::setw(2) << static_cast<int>(byte);
    }
    return command.str();
  }

  void Twn4Elatec::ntag_read_payload_resp(const std::string& response,
                                          uint8_t& result,
                                          std::array<uint8_t, 16>& data,
                                          std::string& nfc_key)
  {
    if (response.size() != 36)
    {
      std::cout << "error, length of the response is not 36. The response was:" << response << std::endl;
      result = 0;
      return;
    }
    result = std::stoi(response.substr(2, 2), nullptr, 16);
    for (int i = 0; i < 16; i++)
    {
      data[i] = std::stoi(response.substr(4 + i * 2, 2), nullptr, 16);
    }
    nfc_key = response.substr(4, 32);
  }

  void Twn4Elatec::ntag_write_payload_resp(const std::string& response, uint8_t& result)
  {
    if (response.size() != 4)
    {
      throw std::runtime_error("Response must have a length of 4");
    }
    result = std::stoi(response.substr(2, 2), nullptr, 16);
  }

  void Twn4Elatec::search_tag_resp(const std::string& response,
                                   uint8_t& result,
                                   std::string& tagType,
                                   std::string& tagID)
  {
    if (response.size() < 10)
    {
      std::cout << "Response is too short to be a valid SearchTag response" << response << std::endl;
      result = 0;
      return;
    }
    result = std::stoi(response.substr(2, 2), nullptr, 16);
    tagType = response.substr(4, 2);
    tagID = response.substr(10, response.size() - 10);
  }

  bool Twn4Elatec::TagTypeValidation(const std::string& tagType)
  {
    return tagType == TAG_TYPE_NTAG213 || tagType == TAG_TYPE_NTAG215 || tagType == TAG_TYPE_MIFARE_CLASIC;
  }

}   // namespace nfc_bridge

#endif   // NFC_BRIDGE_TWN4_HPP