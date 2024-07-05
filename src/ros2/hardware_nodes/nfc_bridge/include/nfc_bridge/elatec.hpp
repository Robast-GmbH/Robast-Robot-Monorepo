#include <string>

class Elatec
{
 public:
  // Constants
  static const std::string DlagLEDOn;
  static const std::string SearchTagCmd;

  // Static request functions
  static std::string BeepReq(uint8_t volume, uint16_t frequency, uint16_t onTime, uint16_t offTime);
  static std::string SearchTagReq(u_int8_t MaxIDBytes);

  static std::string NTAGReadReq(uint8_t page);
  static std::string NTAGWriteReq(uint8_t page, std::array<uint8_t, 4> &data);

  // Static response functions
  static void NTAGReadReq(std::string response, u_int8_t &result, std::array<uint8_t, 16> &data);
  static void NTAGWriteReq(std::string response, u_int8_t &result);
};

// Initialize constants
const std::string Elatec::DlagLEDOn = "0408";

// Implement static function
std::string Elatec::BeepReq(uint8_t volume, uint16_t frequency, uint16_t onTime, uint16_t offTime)
{
  std::string command = "0407";
  return command + std::format("{:02X}", volume) + std::format("{:04X}", frequency) + std::format("{:04X}", onTime) +
         std::format("{:04X}", offTime);
}

std::string Elatec::SearchTagReq(u_int8_t MaxIDBytes)
{
  std::string command = "0500";
  return command + std::format("{:02X}", MaxIDBytes);
}

std::string Elatec::NTAGReadReq(uint8_t page)
{
  std::string command = "0501";
  return command + std::format("{:02X}", page);
}

std::string Elatec::NTAGWriteReq(uint8_t page, std::array<uint8_t, 4> &data)
{
  std::string command = "0502";
  std::string dataStr = "";
  if (data.size() != 4)
  {
    // Throw an exception or handle the error appropriately
    throw std::runtime_error("Data array must have a length of 4");
  }
  for (int i = 0; i < data.size(); i++)
  {
    dataStr += std::format("{:02X}", data[i]);
  }
  return command + std::format("{:02X}", page) + dataStr;
}

void Elatec::NTAGReadReq(std::string response, u_int8_t &result, std::array<uint8_t, 16> &data)
{
  if (response.size() != 34)
  {
    // Throw an exception or handle the error appropriately
    throw std::runtime_error("Response must have a length of 34");
  }
  result = std::stoi(response.substr(4, 2), nullptr, 16);
  for (int i = 0; i < 16; i++)
  {
    data[i] = std::stoi(response.substr(6 + i * 2, 2), nullptr, 16);
  }
}

void Elatec::NTAGWriteReq(std::string response, u_int8_t &result)
{
  if (response.size() != 6)
  {
    // Throw an exception or handle the error appropriately
    throw std::runtime_error("Response must have a length of 6");
  }
  result = std::stoi(response.substr(4, 2), nullptr, 16);
}