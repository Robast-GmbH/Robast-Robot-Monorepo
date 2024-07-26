#include <catch2/catch_all.hpp>

#include "../include/can/can_db.hpp"
#include "../include/can/can_frame.h"
#include "../include/can/can_helper.h"
/*
 * HOW TO RUN THIS TEST ON WINDOWS:
 * - Go to directory libs/can/tests
 * - Run the following commands:
 * g++ -std=c++17 -c .\tests_main.cpp
 * g++ -std=c++17 tests_main.o tests_robast_can_msgs.cpp ..\src\* -o test_executable -I .. -Wall -Wextra -Wuseless-cast
 * -Wdouble-promotion -Wnull-dereference -Wpedantic -Wshadow -Wnon-virtual-dtor -Wlogical-op
 * .\test_executable.exe
 */
/*
 * HOW TO RUN THIS TEST ON LINUX:
 * - Go to directory libs/can/test
 * - Run the following commands:
 * g++ -std=c++17 -c tests_main.cpp
 * g++ -std=c++17 tests_main.o tests_robast_can_msgs.cpp ../src/* -o test_executable -I .. -Wall -Wextra -Wuseless-cast
 * -Wdouble-promotion -Wnull-dereference -Wpedantic -Wshadow -Wnon-virtual-dtor -Wlogical-op
 * ./test_executable
 */
SCENARIO("Test class creation of CanSignal, CanMessage, CanDb and CanFrame", "[robast_can_msgs]")
{
  GIVEN("A CAN msg_id and dlc as well as data for a drawer_id, LED_red, LED_green, LED_blue, LED_brightness, LED_mode")
  {
    uint32_t msg_id = robast_can_msgs::can_id::SINGLE_LED_STATE;
    uint8_t dlc = robast_can_msgs::can_dlc::SINGLE_LED_STATE;
    uint64_t data_module_id = 0x010203;
    uint8_t data_drawer_id = 0;
    uint64_t data_LED_red = 1;
    uint64_t data_LED_green = 2;
    uint64_t data_LED_blue = 3;
    uint64_t data_LED_brightness = 7;
    uint8_t u8_can_data[8] = {0x01,
                              0x02,
                              0x03,
                              (uint8_t) data_LED_red,
                              (uint8_t) data_LED_green,
                              (uint8_t) data_LED_blue,
                              (uint8_t) data_LED_brightness,
                              0};
    WHEN("Creating the CanSignal classes")
    {
      uint8_t bit_start_module_id = CAN_SIGNAL_MODULE_ID_BIT_START;
      uint8_t bit_length_module_id = CAN_SIGNAL_MODULE_ID_BIT_LENGTH;
      robast_can_msgs::CanSignal can_signal_module_id =
        robast_can_msgs::CanSignal(bit_start_module_id, bit_length_module_id, data_module_id);
      uint8_t bit_start_LED_red = CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_START;
      uint8_t bit_length_LED_red = CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_LENGTH;
      robast_can_msgs::CanSignal can_signal_LED_red =
        robast_can_msgs::CanSignal(bit_start_LED_red, bit_length_LED_red, data_LED_red);
      uint8_t bit_start_LED_green = CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_START;
      uint8_t bit_length_LED_green = CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_LENGTH;
      robast_can_msgs::CanSignal can_signal_LED_green =
        robast_can_msgs::CanSignal(bit_start_LED_green, bit_length_LED_green, data_LED_green);
      uint8_t bit_start_LED_blue = CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_START;
      uint8_t bit_length_LED_blue = CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_LENGTH;
      robast_can_msgs::CanSignal can_signal_LED_blue =
        robast_can_msgs::CanSignal(bit_start_LED_blue, bit_length_LED_blue, data_LED_blue);
      uint8_t bit_start_LED_brightness = CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_START;
      uint8_t bit_length_LED_brightness = CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_LENGTH;
      robast_can_msgs::CanSignal can_signal_LED_brightness =
        robast_can_msgs::CanSignal(bit_start_LED_brightness, bit_length_LED_brightness, data_LED_brightness);
      THEN("The created CanSignal classes should encapsulate the data correctly")
      {
        REQUIRE(can_signal_module_id.get_bit_start() == bit_start_module_id);
        REQUIRE(can_signal_module_id.get_bit_length() == bit_length_module_id);
        REQUIRE(can_signal_module_id.get_data() == data_module_id);
        REQUIRE(can_signal_LED_red.get_bit_start() == bit_start_LED_red);
        REQUIRE(can_signal_LED_red.get_bit_length() == bit_length_LED_red);
        REQUIRE(can_signal_LED_red.get_data() == data_LED_red);
        REQUIRE(can_signal_LED_green.get_bit_start() == bit_start_LED_green);
        REQUIRE(can_signal_LED_green.get_bit_length() == bit_length_LED_green);
        REQUIRE(can_signal_LED_green.get_data() == data_LED_green);
        REQUIRE(can_signal_LED_blue.get_bit_start() == bit_start_LED_blue);
        REQUIRE(can_signal_LED_blue.get_bit_length() == bit_length_LED_blue);
        REQUIRE(can_signal_LED_blue.get_data() == data_LED_blue);
        REQUIRE(can_signal_LED_brightness.get_bit_start() == bit_start_LED_brightness);
        REQUIRE(can_signal_LED_brightness.get_bit_length() == bit_length_LED_brightness);
        REQUIRE(can_signal_LED_brightness.get_data() == data_LED_brightness);
      }
      WHEN("Creating the CanMessage class")
      {
        robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(msg_id,
                                                                              dlc,
                                                                              {can_signal_module_id,
                                                                               can_signal_LED_red,
                                                                               can_signal_LED_green,
                                                                               can_signal_LED_blue,
                                                                               can_signal_LED_brightness});
        THEN("The created CanMessage class should encapsulate the data correctly")
        {
          REQUIRE(can_message.get_id() == msg_id);
          REQUIRE(can_message.get_dlc() == dlc);
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_MODULE_ID].get_bit_start() ==
                  can_signal_module_id.get_bit_start());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_MODULE_ID].get_bit_length() ==
                  can_signal_module_id.get_bit_length());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_MODULE_ID].get_data() == can_signal_module_id.get_data());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_RED].get_bit_start() ==
                  can_signal_LED_red.get_bit_start());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_RED].get_bit_length() ==
                  can_signal_LED_red.get_bit_length());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_RED].get_data() ==
                  can_signal_LED_red.get_data());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_GREEN].get_bit_start() ==
                  can_signal_LED_green.get_bit_start());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_GREEN].get_bit_length() ==
                  can_signal_LED_green.get_bit_length());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_GREEN].get_data() ==
                  can_signal_LED_green.get_data());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BLUE].get_bit_start() ==
                  can_signal_LED_blue.get_bit_start());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BLUE].get_bit_length() ==
                  can_signal_LED_blue.get_bit_length());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BLUE].get_data() ==
                  can_signal_LED_blue.get_data());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS].get_bit_start() ==
                  can_signal_LED_brightness.get_bit_start());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS].get_bit_length() ==
                  can_signal_LED_brightness.get_bit_length());
          REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS].get_data() ==
                  can_signal_LED_brightness.get_data());
        }
        WHEN("Creating the CanDb class")
        {
          robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
          THEN(
            "The created CanDb class should contain the correct id, dlc and CanSignals with the correct bit_start and "
            "bit_length and data should be default 0.")
          {
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE].get_id() ==
                    robast_can_msgs::can_id::SINGLE_LED_STATE);
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE].get_dlc() ==
                    robast_can_msgs::can_dlc::SINGLE_LED_STATE);
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_MODULE_ID]
                      .get_bit_start() == can_signal_module_id.get_bit_start());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_MODULE_ID]
                      .get_bit_length() == can_signal_module_id.get_bit_length());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_MODULE_ID]
                      .get_data() == 0);
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_RED]
                      .get_bit_start() == can_signal_LED_red.get_bit_start());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_RED]
                      .get_bit_length() == can_signal_LED_red.get_bit_length());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_RED]
                      .get_data() == 0);
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_GREEN]
                      .get_bit_start() == can_signal_LED_green.get_bit_start());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_GREEN]
                      .get_bit_length() == can_signal_LED_green.get_bit_length());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_GREEN]
                      .get_data() == 0);
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BLUE]
                      .get_bit_start() == can_signal_LED_blue.get_bit_start());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BLUE]
                      .get_bit_length() == can_signal_LED_blue.get_bit_length());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BLUE]
                      .get_data() == 0);
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS]
                      .get_bit_start() == can_signal_LED_brightness.get_bit_start());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS]
                      .get_bit_length() == can_signal_LED_brightness.get_bit_length());
            REQUIRE(can_db.can_messages[robast_can_msgs::can_msg::SINGLE_LED_STATE]
                      .get_can_signals()[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS]
                      .get_data() == 0);
          }
        }
      }
    }
    WHEN("Creating the CanFrame class")
    {
      robast_can_msgs::CanFrame can_frame = robast_can_msgs::CanFrame(msg_id, dlc, u8_can_data);
      THEN("The created CanFrame class should encapsulate the data correctly")
      {
        REQUIRE(can_frame.get_id() == msg_id);
        REQUIRE(can_frame.get_dlc() == dlc);
        for (uint8_t i = 0; i < dlc; i++)
        {
          REQUIRE(can_frame.get_data()[i] == u8_can_data[i]);
        }
      }
    }
  }
}
SCENARIO("Test CAN helper functions", "[robast_can_msgs]")
{
  GIVEN("A CAN msg_id and dlc as well as data for a drawer_id, LED_red, LED_green, LED_blue")
  {
    uint32_t msg_id_unlock = robast_can_msgs::can_id::DRAWER_UNLOCK;
    uint8_t dlc_unlock = robast_can_msgs::can_dlc::DRAWER_UNLOCK;
    uint8_t u8_can_data_unlock[8] = {0x01, 0x02, 0x03, 0b00000000};
    uint64_t u64_can_data_expected_unlock = 0x0102030000000000;
    uint32_t msg_id = robast_can_msgs::can_id::SINGLE_LED_STATE;
    uint8_t dlc = robast_can_msgs::can_dlc::SINGLE_LED_STATE;
    uint64_t data_module_id = 0x010203;
    uint8_t data_drawer_id = 0;
    uint64_t data_LED_red = 1;
    uint64_t data_LED_green = 2;
    uint64_t data_LED_blue = 3;
    uint64_t data_LED_brightness = 7;
    uint8_t u8_can_data[8] = {0x01, 0x02, 0x03, 0x01, 0x02, 0x03, 0x07, 0};
    uint64_t u64_can_data_expected = 0x102030102030700;
    uint64_t u64_can_data_not_expected = 0x0907060509030201;
    uint8_t bit_start_module_id = CAN_SIGNAL_MODULE_ID_BIT_START;
    uint8_t bit_length_module_id = CAN_SIGNAL_MODULE_ID_BIT_LENGTH;
    robast_can_msgs::CanSignal can_signal_module_id =
      robast_can_msgs::CanSignal(bit_start_module_id, bit_length_module_id, data_module_id);
    uint8_t bit_start_LED_red = CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_START;
    uint8_t bit_length_LED_red = CAN_SIGNAL_SINGLE_LED_STATE_RED_BIT_LENGTH;
    robast_can_msgs::CanSignal can_signal_LED_red =
      robast_can_msgs::CanSignal(bit_start_LED_red, bit_length_LED_red, data_LED_red);
    uint8_t bit_start_LED_green = CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_START;
    uint8_t bit_length_LED_green = CAN_SIGNAL_SINGLE_LED_STATE_GREEN_BIT_LENGTH;
    robast_can_msgs::CanSignal can_signal_LED_green =
      robast_can_msgs::CanSignal(bit_start_LED_green, bit_length_LED_green, data_LED_green);
    uint8_t bit_start_LED_blue = CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_START;
    uint8_t bit_length_LED_blue = CAN_SIGNAL_SINGLE_LED_STATE_BLUE_BIT_LENGTH;
    robast_can_msgs::CanSignal can_signal_LED_blue =
      robast_can_msgs::CanSignal(bit_start_LED_blue, bit_length_LED_blue, data_LED_blue);
    uint8_t bit_start_LED_brightness = CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_START;
    uint8_t bit_length_LED_brightness = CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS_BIT_LENGTH;
    robast_can_msgs::CanSignal can_signal_LED_brightness =
      robast_can_msgs::CanSignal(bit_start_LED_brightness, bit_length_LED_brightness, data_LED_brightness);
    robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
      msg_id,
      dlc,
      {can_signal_module_id, can_signal_LED_red, can_signal_LED_green, can_signal_LED_blue, can_signal_LED_brightness});
    robast_can_msgs::CanMessage can_message_invalid_id = robast_can_msgs::CanMessage(
      0x456,
      dlc,
      {can_signal_module_id, can_signal_LED_red, can_signal_LED_green, can_signal_LED_blue, can_signal_LED_brightness});
    robast_can_msgs::CanMessage can_message_drawer_unlock = robast_can_msgs::CanMessage(
      robast_can_msgs::can_id::DRAWER_UNLOCK,
      robast_can_msgs::can_dlc::DRAWER_UNLOCK,
      {
        robast_can_msgs::CanSignal(CAN_SIGNAL_MODULE_ID_BIT_START, CAN_SIGNAL_MODULE_ID_BIT_LENGTH, data_module_id),
        robast_can_msgs::CanSignal(CAN_SIGNAL_DRAWER_ID_BIT_START, CAN_SIGNAL_DRAWER_ID_BIT_LENGTH, data_drawer_id),
      });
    uint64_t u64_can_data_expected_drawer_feedback = 0x0102038000000000;
    robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
    robast_can_msgs::CanFrame can_frame = robast_can_msgs::CanFrame(msg_id, dlc, u8_can_data);
    WHEN("Joining together CAN data bytes from data array")
    {
      uint64_t u64_can_data_unlock =
        robast_can_msgs::join_together_CAN_data_bytes_from_array(u8_can_data_unlock, dlc_unlock);
      uint64_t u64_can_data = robast_can_msgs::join_together_CAN_data_bytes_from_array(u8_can_data, dlc);
      THEN("The result should correspond to the expeted uint64")
      {
        REQUIRE(u64_can_data_unlock == u64_can_data_expected_unlock);
        REQUIRE(u64_can_data == u64_can_data_expected);
        REQUIRE(u64_can_data != u64_can_data_not_expected);
      }
    }
    WHEN("Joining together CAN data from CAN messages")
    {
      uint64_t u64_can_data = robast_can_msgs::join_together_CAN_data_from_CAN_message(can_message);
      THEN("The result should correspond to the expected uint64")
      {
        REQUIRE(u64_can_data == u64_can_data_expected);
        REQUIRE(u64_can_data != u64_can_data_not_expected);
      }
    }
    WHEN("Swapping endian")
    {
      uint64_t input_to_be_swapped = 0x1034567891234560;
      uint64_t input_unswapped = input_to_be_swapped;
      robast_can_msgs::SwapEndian<uint64_t>(input_to_be_swapped);
      THEN("The input_to_be_swapped should have swapped Endian")
      {
        REQUIRE(input_to_be_swapped == 0x6045239178563410);
        REQUIRE(input_to_be_swapped != input_unswapped);
      }
      WHEN("Swapping endian twice")
      {
        uint64_t input_to_be_swapped_twice = input_to_be_swapped;
        robast_can_msgs::SwapEndian<uint64_t>(input_to_be_swapped_twice);
        THEN("The input_to_be_swapped_twice should have the original Endian again")
        {
          REQUIRE(input_to_be_swapped_twice == input_unswapped);
        }
      }
    }
    WHEN("Splitting uint64 into Bytes")
    {
      uint8_t u8_result[8];
      robast_can_msgs::u64_to_eight_bytes(u64_can_data_expected, u8_result);
      THEN("The resulting array should match the expected array")
      {
        for (uint8_t i = 0; i < dlc; i++)
        {
          REQUIRE(u8_result[i] == u8_can_data[i]);
        }
      }
    }
    WHEN("Encoding a CAN message with an id, that exists in the can_db messages")
    {
      robast_can_msgs::CanFrame encoded_can_frame =
        robast_can_msgs::encode_can_message_into_can_frame(can_message, can_db.can_messages);
      THEN("The resulting CanFrame Class should contain all the data that was contained in the CanMessage class")
      {
        REQUIRE(encoded_can_frame.get_id() == can_message.get_id());
        REQUIRE(encoded_can_frame.get_dlc() == can_message.get_dlc());
        for (uint8_t i = 0; i < dlc; i++)
        {
          REQUIRE(encoded_can_frame.get_data()[i] == u8_can_data[i]);
        }
      }
    }
    WHEN("Encoding a CAN message with an id, that does not exist in the can_db messages")
    {
      THEN("The resulting CanFrame should throw a invalid_argument exception")
      {
        REQUIRE_THROWS_AS(
          robast_can_msgs::encode_can_message_into_can_frame(can_message_invalid_id, can_db.can_messages),
          std::invalid_argument);
      }
    }
    WHEN("Decoding a CAN message with an id, that exists in the can_db messages")
    {
      std::optional<robast_can_msgs::CanMessage> decoded_can_message_drawer_unlock =
        robast_can_msgs::decode_can_message(msg_id_unlock, u8_can_data_unlock, dlc_unlock, can_db.can_messages);
      std::optional<robast_can_msgs::CanMessage> decoded_can_message_drawer_led =
        robast_can_msgs::decode_can_message(msg_id, u8_can_data, dlc, can_db.can_messages);
      std::vector<robast_can_msgs::CanSignal> can_signals = decoded_can_message_drawer_led.value().get_can_signals();
      THEN("The resulting CanMessage class should contain all the data that was contained in the CanMessage class")
      {
        REQUIRE(decoded_can_message_drawer_unlock.has_value());
        REQUIRE(decoded_can_message_drawer_unlock.value().get_id() == robast_can_msgs::can_id::DRAWER_UNLOCK);
        REQUIRE(decoded_can_message_drawer_unlock.value().get_dlc() == robast_can_msgs::can_dlc::DRAWER_UNLOCK);
        REQUIRE(can_signals[CAN_SIGNAL_MODULE_ID].get_bit_start() == CAN_SIGNAL_MODULE_ID_BIT_START);
        REQUIRE(can_signals[CAN_SIGNAL_MODULE_ID].get_bit_length() == CAN_SIGNAL_MODULE_ID_BIT_LENGTH);
        REQUIRE(decoded_can_message_drawer_led.has_value());
        REQUIRE(decoded_can_message_drawer_led.value().get_id() == can_message.get_id());
        REQUIRE(decoded_can_message_drawer_led.value().get_dlc() == can_message.get_dlc());
        REQUIRE(can_signals[CAN_SIGNAL_MODULE_ID].get_bit_start() == bit_start_module_id);
        REQUIRE(can_signals[CAN_SIGNAL_MODULE_ID].get_bit_length() == bit_length_module_id);
        REQUIRE(can_signals[CAN_SIGNAL_MODULE_ID].get_data() == data_module_id);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_RED].get_bit_start() == bit_start_LED_red);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_RED].get_bit_length() == bit_length_LED_red);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_RED].get_data() == data_LED_red);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_GREEN].get_bit_start() == bit_start_LED_green);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_GREEN].get_bit_length() == bit_length_LED_green);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_GREEN].get_data() == data_LED_green);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_BLUE].get_bit_start() == bit_start_LED_blue);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_BLUE].get_bit_length() == bit_length_LED_blue);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_BLUE].get_data() == data_LED_blue);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS].get_bit_start() == bit_start_LED_brightness);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS].get_bit_length() == bit_length_LED_brightness);
        REQUIRE(can_signals[CAN_SIGNAL_SINGLE_LED_STATE_BRIGHTNESS].get_data() == data_LED_brightness);
      }
    }
    WHEN("Decoding a CAN Message with an id, that does not exist in the can_db messages")
    {
      uint32_t invalid_msg_id = 0x456;
      std::optional<robast_can_msgs::CanMessage> decoded_can_message =
        robast_can_msgs::decode_can_message(invalid_msg_id, u8_can_data, dlc, can_db.can_messages);
      THEN("The resulting CanFrame should not contain a value")
      {
        REQUIRE_FALSE(decoded_can_message.has_value());
      }
    }
  }
}
