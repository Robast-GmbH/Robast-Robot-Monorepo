#include <catch2/catch_all.hpp>

#include "../include/can/can_db.hpp"
#include "../include/can/can_frame.h"
#include "../include/can/can_helper.h"

/*
* HOW TO RUN THIS TEST ON WINDOWS:
* - Go to directory libs/can/tests
* - Run the following commands:
* g++ -std=c++17 -c .\tests_main.cpp
* g++ -std=c++17 tests_main.o tests_robast_can_msgs.cpp ..\src\* -o test_executable -I .. -Wall -Wextra -Wuseless-cast -Wdouble-promotion -Wnull-dereference -Wpedantic -Wshadow -Wnon-virtual-dtor -Wlogical-op
* .\test_executable.exe
*/

/*
* HOW TO RUN THIS TEST ON LINUX:
* - Go to directory libs/can/tests
* - Run the following commands:
* g++ -std=c++17 -c tests_main.cpp
* g++ -std=c++17 tests_main.o tests_robast_can_msgs.cpp ../src/* -o test_executable -I .. -Wall -Wextra -Wuseless-cast -Wdouble-promotion -Wnull-dereference -Wpedantic -Wshadow -Wnon-virtual-dtor -Wlogical-op
* ./test_executable
*/

SCENARIO("Test class creation of CanSignal, CanMessage, CanDb and CanFrame", "[robast_can_msgs]")
{
    GIVEN("A CAN msg_id and dlc as well as data for a drawer_id, LED_red, LED_green, LED_blue, LED_brightness, LED_mode")
    {
        uint32_t msg_id = CAN_ID_DRAWER_LED;
        uint8_t dlc = CAN_DLC_DRAWER_LED;
        uint64_t data_drawer_controller_id = 0x010203;
        uint64_t data_LED_red = 1;
        uint64_t data_LED_green = 2;
        uint64_t data_LED_blue = 3;
        uint64_t data_LED_brightness = 7;
        uint64_t data_LED_mode = 1;
        uint8_t u8_can_data[8] = { 0x01, 0x02, 0x03, (uint8_t)data_LED_red, (uint8_t)data_LED_green, (uint8_t)data_LED_blue, (uint8_t)data_LED_brightness, 0b00100000 };

        WHEN("Creating the CanSignal classes")
        {
            uint8_t bit_start_drawer_id = CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START;
            uint8_t bit_length_drawer_id = CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH;
            robast_can_msgs::CanSignal can_signal_drawer_id = robast_can_msgs::CanSignal(bit_start_drawer_id, bit_length_drawer_id, data_drawer_controller_id);

            uint8_t bit_start_LED_red = CAN_SIGNAL_LED_RED_BIT_START;
            uint8_t bit_length_LED_red = CAN_SIGNAL_LED_RED_BIT_LENGTH;
            robast_can_msgs::CanSignal can_signal_LED_red = robast_can_msgs::CanSignal(bit_start_LED_red, bit_length_LED_red, data_LED_red);

            uint8_t bit_start_LED_green = CAN_SIGNAL_LED_GREEN_BIT_START;
            uint8_t bit_length_LED_green = CAN_SIGNAL_LED_GREEN_BIT_LENGTH;
            robast_can_msgs::CanSignal can_signal_LED_green = robast_can_msgs::CanSignal(bit_start_LED_green, bit_length_LED_green, data_LED_green);

            uint8_t bit_start_LED_blue = CAN_SIGNAL_LED_BLUE_BIT_START;
            uint8_t bit_length_LED_blue = CAN_SIGNAL_LED_BLUE_BIT_LENGTH;
            robast_can_msgs::CanSignal can_signal_LED_blue = robast_can_msgs::CanSignal(bit_start_LED_blue, bit_length_LED_blue, data_LED_blue);

            uint8_t bit_start_LED_brightness = CAN_SIGNAL_LED_BRIGHTNESS_BIT_START;
            uint8_t bit_length_LED_brightness = CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH;
            robast_can_msgs::CanSignal can_signal_LED_brightness = robast_can_msgs::CanSignal(bit_start_LED_brightness, bit_length_LED_brightness, data_LED_brightness);

            uint8_t bit_start_LED_mode = CAN_SIGNAL_LED_MODE_BIT_START;
            uint8_t bit_length_LED_mode = CAN_SIGNAL_LED_MODE_BIT_LENGTH;
            robast_can_msgs::CanSignal can_signal_LED_mode = robast_can_msgs::CanSignal(bit_start_LED_mode, bit_length_LED_mode, data_LED_mode);

            THEN("The created CanSignal classes should encapsulate the data correctly")
            {
                REQUIRE(can_signal_drawer_id.get_bit_start() == bit_start_drawer_id);
                REQUIRE(can_signal_drawer_id.get_bit_length() == bit_length_drawer_id);
                REQUIRE(can_signal_drawer_id.get_data() == data_drawer_controller_id);

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

                REQUIRE(can_signal_LED_mode.get_bit_start() == bit_start_LED_mode);
                REQUIRE(can_signal_LED_mode.get_bit_length() == bit_length_LED_mode);
                REQUIRE(can_signal_LED_mode.get_data() == data_LED_mode);
            }

            WHEN("Creating the CanMessage class")
            {
                robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
                    msg_id,
                    dlc,
                    {
                        can_signal_drawer_id,
                        can_signal_LED_red,
                        can_signal_LED_green,
                        can_signal_LED_blue,
                        can_signal_LED_brightness,
                        can_signal_LED_mode
                    });

                THEN("The created CanMessage class should encapsulate the data correctly")
                {
                    REQUIRE(can_message.get_id() == msg_id);
                    REQUIRE(can_message.get_dlc() == dlc);

                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_start() == can_signal_drawer_id.get_bit_start());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_length() == can_signal_drawer_id.get_bit_length());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_data() == can_signal_drawer_id.get_data());

                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_start() == can_signal_LED_red.get_bit_start());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_length() == can_signal_LED_red.get_bit_length());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_RED].get_data() == can_signal_LED_red.get_data());

                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_start() == can_signal_LED_green.get_bit_start());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_length() == can_signal_LED_green.get_bit_length());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_GREEN].get_data() == can_signal_LED_green.get_data());

                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_start() == can_signal_LED_blue.get_bit_start());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_length() == can_signal_LED_blue.get_bit_length());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_BLUE].get_data() == can_signal_LED_blue.get_data());

                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_start() == can_signal_LED_brightness.get_bit_start());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_length() == can_signal_LED_brightness.get_bit_length());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_data() == can_signal_LED_brightness.get_data());

                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_start() == can_signal_LED_mode.get_bit_start());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_length() == can_signal_LED_mode.get_bit_length());
                    REQUIRE(can_message.get_can_signals()[CAN_SIGNAL_LED_MODE].get_data() == can_signal_LED_mode.get_data());
                }

                WHEN("Creating the CanDb class")
                {
                    robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

                    THEN("The created CanDb class should contain the correct id, dlc and CanSignals with the correct bit_start and bit_length and data should be default 0.")
                    {
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_id() == CAN_ID_DRAWER_LED);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_dlc() == CAN_DLC_DRAWER_LED);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_start() == can_signal_drawer_id.get_bit_start());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_length() == can_signal_drawer_id.get_bit_length());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_data() == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_start() == can_signal_LED_red.get_bit_start());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_length() == can_signal_LED_red.get_bit_length());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_RED].get_data() == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_start() == can_signal_LED_green.get_bit_start());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_length() == can_signal_LED_green.get_bit_length());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_GREEN].get_data() == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_start() == can_signal_LED_blue.get_bit_start());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_length() == can_signal_LED_blue.get_bit_length());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_BLUE].get_data() == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_start() == can_signal_LED_brightness.get_bit_start());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_length() == can_signal_LED_brightness.get_bit_length());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_data() == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_start() == can_signal_LED_mode.get_bit_start());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_length() == can_signal_LED_mode.get_bit_length());
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_LED].get_can_signals()[CAN_SIGNAL_LED_MODE].get_data() == 0);
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
        uint32_t msg_id_lock = CAN_ID_DRAWER_LOCK;
        uint8_t dlc_lock = CAN_DLC_DRAWER_LOCK;
        uint64_t data_open_lock_1 = 1;
        uint64_t data_open_lock_2 = 0;
        uint8_t u8_can_data_lock[8] = { 0x01,0x02,0x03,0b10000000,0,0,0,0 };
        uint64_t u64_can_data_expected_lock = 0x0102038000000000;

        uint32_t msg_id = CAN_ID_DRAWER_LED;
        uint8_t dlc = CAN_DLC_DRAWER_LED;
        uint64_t data_drawer_controller_id = 0x010203;
        uint64_t data_LED_red = 1;
        uint64_t data_LED_green = 2;
        uint64_t data_LED_blue = 3;
        uint64_t data_LED_brightness = 7;
        uint64_t data_LED_mode = 1;
        uint8_t u8_can_data[8] = { 0x01,0x02,0x03,(uint8_t)data_LED_red,(uint8_t)data_LED_green,(uint8_t)data_LED_blue,(uint8_t)data_LED_brightness,0b00100000 };

        uint64_t u64_can_data_expected = 0x0102030102030720;
        uint64_t u64_can_data_not_expected = 0x0907060509030201;

        std::string ascii_command_expected = "t00";
        ascii_command_expected.append(std::to_string(msg_id));
        ascii_command_expected.append(std::to_string(dlc));
        ascii_command_expected.append("0102030102030720");

        uint8_t bit_start_drawer_id = CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START;
        uint8_t bit_length_drawer_id = CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH;
        robast_can_msgs::CanSignal can_signal_drawer_id = robast_can_msgs::CanSignal(bit_start_drawer_id, bit_length_drawer_id, data_drawer_controller_id);

        uint8_t bit_start_LED_red = CAN_SIGNAL_LED_RED_BIT_START;
        uint8_t bit_length_LED_red = CAN_SIGNAL_LED_RED_BIT_LENGTH;
        robast_can_msgs::CanSignal can_signal_LED_red = robast_can_msgs::CanSignal(bit_start_LED_red, bit_length_LED_red, data_LED_red);

        uint8_t bit_start_LED_green = CAN_SIGNAL_LED_GREEN_BIT_START;
        uint8_t bit_length_LED_green = CAN_SIGNAL_LED_GREEN_BIT_LENGTH;
        robast_can_msgs::CanSignal can_signal_LED_green = robast_can_msgs::CanSignal(bit_start_LED_green, bit_length_LED_green, data_LED_green);

        uint8_t bit_start_LED_blue = CAN_SIGNAL_LED_BLUE_BIT_START;
        uint8_t bit_length_LED_blue = CAN_SIGNAL_LED_BLUE_BIT_LENGTH;
        robast_can_msgs::CanSignal can_signal_LED_blue = robast_can_msgs::CanSignal(bit_start_LED_blue, bit_length_LED_blue, data_LED_blue);

        uint8_t bit_start_LED_brightness = CAN_SIGNAL_LED_BRIGHTNESS_BIT_START;
        uint8_t bit_length_LED_brightness = CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH;
        robast_can_msgs::CanSignal can_signal_LED_brightness = robast_can_msgs::CanSignal(bit_start_LED_brightness, bit_length_LED_brightness, data_LED_brightness);

        uint8_t bit_start_LED_mode = CAN_SIGNAL_LED_MODE_BIT_START;
        uint8_t bit_length_LED_mode = CAN_SIGNAL_LED_MODE_BIT_LENGTH;
        robast_can_msgs::CanSignal can_signal_LED_mode = robast_can_msgs::CanSignal(bit_start_LED_mode, bit_length_LED_mode, data_LED_mode);

        robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
            msg_id,
            dlc,
            {
                can_signal_drawer_id,
                can_signal_LED_red,
                can_signal_LED_green,
                can_signal_LED_blue,
                can_signal_LED_brightness,
                can_signal_LED_mode
            });

        robast_can_msgs::CanMessage can_message_invalid_id = robast_can_msgs::CanMessage(
            0x456,
            dlc,
            {
                can_signal_drawer_id,
                can_signal_LED_red,
                can_signal_LED_green,
                can_signal_LED_blue,
                can_signal_LED_brightness,
                can_signal_LED_mode
            });

        robast_can_msgs::CanMessage can_message_drawer_feedback = robast_can_msgs::CanMessage(
            CAN_ID_DRAWER_FEEDBACK,
            CAN_DLC_DRAWER_FEEDBACK,
            {
                robast_can_msgs::CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, 0x010203),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED_BIT_LENGTH, 1),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED_BIT_LENGTH, 0),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_START, CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED_BIT_LENGTH, 0),
                robast_can_msgs::CanSignal(CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_START, CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED_BIT_LENGTH, 0),
            });
        std::string ascii_command_expected_drawer_feedback = "t00";
        ascii_command_expected_drawer_feedback.append(std::to_string(CAN_ID_DRAWER_FEEDBACK));
        ascii_command_expected_drawer_feedback.append(std::to_string(CAN_DLC_DRAWER_FEEDBACK));
        ascii_command_expected_drawer_feedback.append("01020380");
        uint64_t u64_can_data_expected_drawer_feedback = 0x0102038000000000;

        robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

        robast_can_msgs::CanFrame can_frame = robast_can_msgs::CanFrame(msg_id, dlc, u8_can_data);

        WHEN("Joining together CAN data bytes from data array")
        {
            uint64_t u64_can_data_lock = robast_can_msgs::join_together_CAN_data_bytes_from_array(u8_can_data_lock, dlc_lock);
            uint64_t u64_can_data = robast_can_msgs::join_together_CAN_data_bytes_from_array(u8_can_data, dlc);

            THEN("The result should correspond to the expeted uint64")
            {
                REQUIRE(u64_can_data_lock == u64_can_data_expected_lock);

                REQUIRE(u64_can_data == u64_can_data_expected);
                REQUIRE(u64_can_data != u64_can_data_not_expected);
            }
        }

        WHEN("Joining together CAN data from CAN messages")
        {
            uint64_t u64_can_data = robast_can_msgs::join_together_CAN_data_from_CAN_message(can_message);
            uint64_t u64_can_data_drawer_feedback = robast_can_msgs::join_together_CAN_data_from_CAN_message(can_message_drawer_feedback);

            THEN("The result should correspond to the expected uint64")
            {
                REQUIRE(u64_can_data == u64_can_data_expected);
                REQUIRE(u64_can_data != u64_can_data_not_expected);
                REQUIRE(u64_can_data_drawer_feedback == u64_can_data_expected_drawer_feedback);
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

        WHEN("Converting a uint32 or uint64 into a string where the numbers are represented in hex format")
        {
            uint32_t u32_input = 0x01C40;
            uint64_t u64_input = 0x01240123FF;
            std::string u32_expected_string = "01C40";
            std::string u32_unexpected_string = "001C40";
            std::string u64_expected_string = "01240123FF";
            std::string u64_unexpected_string = "1240123ff";
            std::string u32_result = robast_can_msgs::uint_to_hex_string(u32_input, 5);
            std::string u64_result = robast_can_msgs::uint_to_hex_string(u64_input, 10);

            THEN("The converted string should represent the uint32 and uint64 in the hex format.")
            {
                REQUIRE(u32_result == u32_expected_string);
                REQUIRE_FALSE(u32_result == u32_unexpected_string);
                REQUIRE(u64_result == u64_expected_string);
                REQUIRE_FALSE(u64_result == u64_unexpected_string);
            }
        }

        WHEN("Encoding a CAN message with an id, that exists in the can_db messages")
        {
            robast_can_msgs::CanFrame encoded_can_frame = robast_can_msgs::encode_can_message_into_can_frame(can_message, can_db.can_messages);

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
                REQUIRE_THROWS_AS(robast_can_msgs::encode_can_message_into_can_frame(can_message_invalid_id, can_db.can_messages), std::invalid_argument);
            }
        }

        WHEN("Encoding a CAN message into an ASCII command with an id, that exists in the can_db messages")
        {
            std::optional<std::string> ascii_command = robast_can_msgs::encode_can_message_into_ascii_command(can_message, can_db.can_messages);
            std::optional<std::string> ascii_command_drawer_feedback = robast_can_msgs::encode_can_message_into_ascii_command(can_message_drawer_feedback, can_db.can_messages);

            THEN("The resulting ASCII command should contain the id, the dlc and the data that was contained in the CanMessage class")
            {
                REQUIRE(ascii_command.has_value());
                REQUIRE(ascii_command.value() == ascii_command_expected);
                REQUIRE(ascii_command_drawer_feedback.has_value());
                REQUIRE(ascii_command_drawer_feedback.value() == ascii_command_expected_drawer_feedback);
            }
        }

        WHEN("Decoding a CAN message with an id, that exists in the can_db messages")
        {
            std::optional<robast_can_msgs::CanMessage> decoded_can_message_drawer_lock = robast_can_msgs::decode_can_message(msg_id_lock, u8_can_data_lock, dlc_lock, can_db.can_messages);
            std::optional<robast_can_msgs::CanMessage> decoded_can_message_drawer_led = robast_can_msgs::decode_can_message(msg_id, u8_can_data, dlc, can_db.can_messages);

            THEN("The resulting CanMessage class should contain all the data that was contained in the CanMessage class")
            {
                REQUIRE(decoded_can_message_drawer_lock.has_value());
                REQUIRE(decoded_can_message_drawer_lock.value().get_id() == CAN_ID_DRAWER_LOCK);
                REQUIRE(decoded_can_message_drawer_lock.value().get_dlc() == CAN_DLC_DRAWER_LOCK);
                REQUIRE(decoded_can_message_drawer_lock.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_start() == CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START);
                REQUIRE(decoded_can_message_drawer_lock.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_length() == CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH);
                REQUIRE(decoded_can_message_drawer_lock.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_data() == data_drawer_controller_id);

                REQUIRE(decoded_can_message_drawer_led.has_value());
                REQUIRE(decoded_can_message_drawer_led.value().get_id() == can_message.get_id());
                REQUIRE(decoded_can_message_drawer_led.value().get_dlc() == can_message.get_dlc());

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_start() == bit_start_drawer_id);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_length() == bit_length_drawer_id);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_data() == data_drawer_controller_id);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_start() == bit_start_LED_red);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_length() == bit_length_LED_red);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_RED].get_data() == data_LED_red);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_start() == bit_start_LED_green);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_length() == bit_length_LED_green);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_GREEN].get_data() == data_LED_green);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_start() == bit_start_LED_blue);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_length() == bit_length_LED_blue);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BLUE].get_data() == data_LED_blue);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_start() == bit_start_LED_brightness);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_length() == bit_length_LED_brightness);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_data() == data_LED_brightness);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_start() == bit_start_LED_mode);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_length() == bit_length_LED_mode);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_MODE].get_data() == data_LED_mode);
            }
        }

        WHEN("Decoding a CAN Message with an id, that does not exist in the can_db messages")
        {
            uint32_t invalid_msg_id = 0x456;
            std::optional<robast_can_msgs::CanMessage> decoded_can_message = robast_can_msgs::decode_can_message(invalid_msg_id, u8_can_data, dlc, can_db.can_messages);

            THEN("The resulting CanFrame should not contain a value")
            {
                REQUIRE_FALSE(decoded_can_message.has_value());
            }
        }

        // TODO(anyone): Somehow this tests results in an undefined reference error although it worked at some point.
        // TODO(anyone): Nevertheless the function is used in the "decode_ascii_command_into_can_message" function which is tested too.
        // WHEN("Converting a string containing hex numbers to an unsigned int") {
        //     std::string hex_string_1 = "00F";
        //     std::string hex_string_2 = "010";
        //     uint16_t u16_result_expected_1 = 15;
        //     uint16_t u16_result_expected_2 = 16;
        //     uint16_t u16_result_unexpected = 1;
        //     uint16_t u16_result_1 = robast_can_msgs::hex_string_to_unsigned_int<uint16_t>(hex_string_1);
        //     uint16_t u16_result_2 = robast_can_msgs::hex_string_to_unsigned_int<uint16_t>(hex_string_2);

        //     uint64_t u64_result_expected_1 = 15;
        //     uint64_t u64_result_expected_2 = 16;
        //     uint16_t u64_result_unexpected = 1;
        //     uint64_t u64_result_1 = robast_can_msgs::hex_string_to_unsigned_int<uint64_t>(hex_string_1);
        //     uint64_t u64_result_2 = robast_can_msgs::hex_string_to_unsigned_int<uint64_t>(hex_string_2);

        //     THEN("The resulting unsigned integer should represent the same number that was contained in the hex string") {
        //         REQUIRE(u16_result_1 == u16_result_expected_1);
        //         REQUIRE(u16_result_1 != u16_result_unexpected);
        //         REQUIRE(u16_result_2 == u16_result_expected_2);
        //         REQUIRE(u64_result_1 == u64_result_expected_1);
        //         REQUIRE(u64_result_1 != u64_result_unexpected);
        //         REQUIRE(u64_result_2 == u64_result_expected_2);
        //     }
        // }

        WHEN("Decoding an ASCII command into a CAN message")
        {
            std::string ascii_command_drawer_lock = "t001400000180";
            std::optional<robast_can_msgs::CanMessage> decoded_can_message_drawer_lock = robast_can_msgs::decode_single_ascii_command_into_can_message(ascii_command_drawer_lock, ascii_command_drawer_lock.length(), can_db.can_messages);

            std::string ascii_command_drawer_led = "t00280102030102030720";
            std::optional<robast_can_msgs::CanMessage> decoded_can_message_drawer_led = robast_can_msgs::decode_single_ascii_command_into_can_message(ascii_command_drawer_led, ascii_command_drawer_led.length(), can_db.can_messages);

            std::string ascii_command_drawer_feedback = "t003400000170";
            std::optional<robast_can_msgs::CanMessage> decoded_can_message_drawer_feedback = robast_can_msgs::decode_single_ascii_command_into_can_message(ascii_command_drawer_feedback, ascii_command_drawer_feedback.length(), can_db.can_messages);

            THEN("The resulting CAN message should contain the correct data that was contained in the ASCII command")
            {
                REQUIRE(decoded_can_message_drawer_lock.has_value());
                REQUIRE(decoded_can_message_drawer_lock.value().get_id() == CAN_ID_DRAWER_LOCK);
                REQUIRE(decoded_can_message_drawer_lock.value().get_dlc() == CAN_DLC_DRAWER_LOCK);
                REQUIRE(decoded_can_message_drawer_lock.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_data() == 1);
                REQUIRE(decoded_can_message_drawer_lock.value().get_can_signals()[CAN_SIGNAL_OPEN_LOCK_1].get_data() == 1);
                REQUIRE(decoded_can_message_drawer_lock.value().get_can_signals()[CAN_SIGNAL_OPEN_LOCK_2].get_data() == 0);

                REQUIRE(decoded_can_message_drawer_led.has_value());
                REQUIRE(decoded_can_message_drawer_led.value().get_id() == can_message.get_id());
                REQUIRE(decoded_can_message_drawer_led.value().get_dlc() == can_message.get_dlc());

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_start() == bit_start_drawer_id);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_bit_length() == bit_length_drawer_id);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_data() == data_drawer_controller_id);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_start() == bit_start_LED_red);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_RED].get_bit_length() == bit_length_LED_red);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_RED].get_data() == data_LED_red);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_start() == bit_start_LED_green);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_GREEN].get_bit_length() == bit_length_LED_green);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_GREEN].get_data() == data_LED_green);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_start() == bit_start_LED_blue);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BLUE].get_bit_length() == bit_length_LED_blue);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BLUE].get_data() == data_LED_blue);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_start() == bit_start_LED_brightness);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_bit_length() == bit_length_LED_brightness);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_BRIGHTNESS].get_data() == data_LED_brightness);

                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_start() == bit_start_LED_mode);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_MODE].get_bit_length() == bit_length_LED_mode);
                REQUIRE(decoded_can_message_drawer_led.value().get_can_signals()[CAN_SIGNAL_LED_MODE].get_data() == data_LED_mode);

                REQUIRE(decoded_can_message_drawer_feedback.has_value());
                REQUIRE(decoded_can_message_drawer_feedback.value().get_id() == CAN_ID_DRAWER_FEEDBACK);
                REQUIRE(decoded_can_message_drawer_feedback.value().get_dlc() == CAN_DLC_DRAWER_FEEDBACK);

                REQUIRE(decoded_can_message_drawer_feedback.value().get_can_signals()[CAN_SIGNAL_DRAWER_CONTROLLER_ID].get_data() == 1);
                REQUIRE(decoded_can_message_drawer_feedback.value().get_can_signals()[CAN_SIGNAL_IS_ENDSTOP_SWITCH_1_PUSHED].get_data() == 0);
                REQUIRE(decoded_can_message_drawer_feedback.value().get_can_signals()[CAN_SIGNAL_IS_LOCK_SWITCH_1_PUSHED].get_data() == 1);
                REQUIRE(decoded_can_message_drawer_feedback.value().get_can_signals()[CAN_SIGNAL_IS_ENDSTOP_SWITCH_2_PUSHED].get_data() == 1);
                REQUIRE(decoded_can_message_drawer_feedback.value().get_can_signals()[CAN_SIGNAL_IS_LOCK_SWITCH_2_PUSHED].get_data() == 1);
            }
        }
    }
}
