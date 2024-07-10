#include "nfc_bridge/twn4.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("BeepReq generates correct command string", "[Twn4Elatec]")
{
  REQUIRE(nfc_bridge::Twn4Elatec::BeepReq(0x64, 0x6009, 0xF401, 0xF401) == "0407646009F401F401");
}

TEST_CASE("SearchTagReq generates correct command string", "[Twn4Elatec]")
{
  REQUIRE(nfc_bridge::Twn4Elatec::SearchTagReq(0x10) == "050010");
}

TEST_CASE("NTAGReadReq generates correct command string", "[Twn4Elatec]")
{
  REQUIRE(nfc_bridge::Twn4Elatec::NTAGReadReq(0x04) == "200004");
}

TEST_CASE("NTAGWriteReq generates correct command string", "[Twn4Elatec]")
{
  REQUIRE(nfc_bridge::Twn4Elatec::NTAGWriteReq(0x04, {0x01, 0x02, 0x03, 0x04}) == "20010401020304");
}

TEST_CASE("NTAGReadResp - Valid response")
{
  std::string response = "000103B691028C537091016855016E78702E";
  uint8_t result;
  std::array<uint8_t, 16> data;
  std::string nfc_key;

  REQUIRE_NOTHROW(nfc_bridge::Twn4Elatec::NTAGReadResp(response, result, data, nfc_key));

  SECTION("Result should be parsed correctly")
  {
    REQUIRE(result == 0x01);
  }

  SECTION("Data should be parsed correctly")
  {
    std::array<uint8_t, 16> expectedData = {
        0x03, 0xB6, 0x91, 0x02, 0x8C, 0x53, 0x70, 0x91, 0x01, 0x68, 0x55, 0x01, 0x6E, 0x78, 0x70, 0x2E};
    REQUIRE(data == expectedData);
  }

  SECTION("NFC_key should be parsed correctly")
  {
    REQUIRE(nfc_key == "03B691028C537091016855016E78702E");
  }
}

TEST_CASE("NTAGReadResp - Invalid response format")
{
  std::string response = "20ZZ";
  uint8_t result = 0;

  REQUIRE_THROWS_AS(nfc_bridge::Twn4Elatec::NTAGWriteResp(response, result), std::invalid_argument);
}

TEST_CASE("NTAGWriteResp Test", "[Twn4Elatec]")
{
  uint8_t result;
  std::string response = "0001";

  SECTION("Valid response")
  {
    REQUIRE_NOTHROW(nfc_bridge::Twn4Elatec::NTAGWriteResp(response, result));
    REQUIRE(result == 1);
  }

  SECTION("Invalid response length")
  {
    response = "2001FF00";
    REQUIRE_THROWS_AS(nfc_bridge::Twn4Elatec::NTAGWriteResp(response, result), std::runtime_error);
  }
}

TEST_CASE("NTAGWriteResp - Valid response")
{
  std::string response = "0001";
  uint8_t result = 0;

  REQUIRE_NOTHROW(nfc_bridge::Twn4Elatec::NTAGWriteResp(response, result));
  REQUIRE(result == 0x01);
}

TEST_CASE("NTAGWriteResp - Invalid response length")
{
  std::string response = "01";
  uint8_t result = 0;

  REQUIRE_THROWS_AS(nfc_bridge::Twn4Elatec::NTAGWriteResp(response, result), std::runtime_error);
}

TEST_CASE("NTAGWriteResp - Invalid response format")
{
  std::string response = "20ZZ";
  uint8_t result = 0;

  REQUIRE_THROWS_AS(nfc_bridge::Twn4Elatec::NTAGWriteResp(response, result), std::invalid_argument);
}

TEST_CASE("SearchTagResp - Valid response")
{
  std::string response = "0001FF123456789";
  uint8_t result;
  std::string tagType;

  REQUIRE_NOTHROW(nfc_bridge::Twn4Elatec::SearchTagResp(response, result, tagType));

  SECTION("Result should be parsed correctly")
  {
    REQUIRE(result == 0x01);
  }

  SECTION("Tag type should be parsed correctly")
  {
    REQUIRE(tagType == "FF");
  }
}
