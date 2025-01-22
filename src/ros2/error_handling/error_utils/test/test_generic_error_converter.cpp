#include <catch2/catch_all.hpp>

#include "../include/error_utils/error_definitions.hpp"
#include "../include/error_utils/generic_error_converter.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

TEST_CASE("Generic Error Converter Tests", "[generic_error_converter]")
{
  communication_interfaces::msg::DrawerAddress msg_to_be_serialized;
  msg_to_be_serialized.module_id = 0x010203;
  msg_to_be_serialized.drawer_id = 0x01;

  communication_interfaces::msg::DrawerAddress wrong_msg;
  wrong_msg.module_id = 12;
  wrong_msg.drawer_id = 4;

  std::string serialized_msg =
    error_utils::message_to_string<communication_interfaces::msg::DrawerAddress>(msg_to_be_serialized);

  communication_interfaces::msg::DrawerAddress deserialized_msg =
    error_utils::string_to_message<communication_interfaces::msg::DrawerAddress>(serialized_msg);

  SECTION("Serialization Test")
  {
    REQUIRE_FALSE(serialized_msg.empty());
  }

  SECTION("Deserialization Test")
  {
    REQUIRE(deserialized_msg.module_id == msg_to_be_serialized.module_id);
    REQUIRE(deserialized_msg.drawer_id == msg_to_be_serialized.drawer_id);
  }

  SECTION("Test Message Equality")
  {
    REQUIRE(msg_to_be_serialized == deserialized_msg);
    REQUIRE_FALSE(wrong_msg == deserialized_msg);
  }
}