#include "catch.hpp"

#include "../can_db.h"
#include "../can_frame.h"
#include "../can_helper.h"

/*
* HOW TO RUN THIS TEST ON WINDOWS:
* g++ -std=c++17 -c .\tests_main.cpp
* g++ -std=c++17 tests_main.o tests_can_db.cpp ..\can_helper.cpp -o test_executable -I ..\
* .\test_executable.exe
*/

SCENARIO("Test class creation of CanSignal, CanMessage, CanDb and CanFrame", "[robast_can_msgs]") {

    GIVEN("A CAN msg_id and dlc as well as data for a drawer_id, open_drawer, LED_red, LED_green, LED_blue") {
        uint32_t msg_id = CAN_ID_DRAWER_USER_ACCESS;;
        uint8_t dlc = 7;
        uint8_t u8_can_data[8] = {0x01,0x02,0x03,0b11000000,0b01000000,0b10000000,0b11000000,0x00};
        uint64_t data_drawer_id = 0x010203;
        uint64_t data_open_drawer_1 = 1;
        uint64_t data_open_drawer_2 = 1;
        uint64_t data_LED_red = 1;
        uint64_t data_LED_green = 2;
        uint64_t data_LED_blue = 3;    

        WHEN("Creating the CanSignal classes") {
            uint8_t bit_start_drawer_id = 0;
            uint8_t bit_length_drawer_id = 24;
            robast_can_msgs::CanSignal can_signal_drawer_id = robast_can_msgs::CanSignal(bit_start_drawer_id, bit_length_drawer_id, data_drawer_id);

            uint8_t bit_start_open_drawer_1 = 24;
            uint8_t bit_length_open_drawer_1 = 1;
            robast_can_msgs::CanSignal can_signal_open_drawer_1 = robast_can_msgs::CanSignal(bit_start_open_drawer_1, bit_length_open_drawer_1, data_open_drawer_1);

            uint8_t bit_start_open_drawer_2 = 25;
            uint8_t bit_length_open_drawer_2 = 1;
            robast_can_msgs::CanSignal can_signal_open_drawer_2 = robast_can_msgs::CanSignal(bit_start_open_drawer_2, bit_length_open_drawer_2, data_open_drawer_2);

            uint8_t bit_start_LED_red = 26;
            uint8_t bit_length_LED_red = 8;
            robast_can_msgs::CanSignal can_signal_LED_red = robast_can_msgs::CanSignal(bit_start_LED_red, bit_length_LED_red, data_LED_red);

            uint8_t bit_start_LED_green = 34;
            uint8_t bit_length_LED_green = 8;
            robast_can_msgs::CanSignal can_signal_LED_green = robast_can_msgs::CanSignal(bit_start_LED_green, bit_length_LED_green, data_LED_green);

            uint8_t bit_start_LED_blue = 42;
            uint8_t bit_length_LED_blue = 8;
            robast_can_msgs::CanSignal can_signal_LED_blue = robast_can_msgs::CanSignal(bit_start_LED_blue, bit_length_LED_blue, data_LED_blue);

            THEN("The created CanSignal classes should encapsulate the data correctly") {
                REQUIRE(can_signal_drawer_id.bit_start == bit_start_drawer_id);
                REQUIRE(can_signal_drawer_id.bit_length == bit_length_drawer_id);
                REQUIRE(can_signal_drawer_id.data == data_drawer_id);

                REQUIRE(can_signal_open_drawer_1.bit_start == bit_start_open_drawer_1);
                REQUIRE(can_signal_open_drawer_1.bit_length == bit_length_open_drawer_1);
                REQUIRE(can_signal_open_drawer_1.data == data_open_drawer_1);

                REQUIRE(can_signal_open_drawer_2.bit_start == bit_start_open_drawer_2);
                REQUIRE(can_signal_open_drawer_2.bit_length == bit_length_open_drawer_2);
                REQUIRE(can_signal_open_drawer_2.data == data_open_drawer_2);

                REQUIRE(can_signal_LED_red.bit_start == bit_start_LED_red);
                REQUIRE(can_signal_LED_red.bit_length == bit_length_LED_red);
                REQUIRE(can_signal_LED_red.data == data_LED_red);

                REQUIRE(can_signal_LED_green.bit_start == bit_start_LED_green);
                REQUIRE(can_signal_LED_green.bit_length == bit_length_LED_green);
                REQUIRE(can_signal_LED_green.data == data_LED_green);

                REQUIRE(can_signal_LED_blue.bit_start == bit_start_LED_blue);
                REQUIRE(can_signal_LED_blue.bit_length == bit_length_LED_blue);
                REQUIRE(can_signal_LED_blue.data == data_LED_blue);

            }

            WHEN("Creating the CanMessage class") {
                robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
                        msg_id,
                        dlc,
                        {
                            can_signal_drawer_id,
                            can_signal_open_drawer_1,
                            can_signal_open_drawer_2,
                            can_signal_LED_red,
                            can_signal_LED_green,
                            can_signal_LED_blue,
                        });

                THEN("The created CanMessage class should encapsulate the data correctly") {
                    REQUIRE(can_message.id == msg_id);
                    REQUIRE(can_message.dlc == dlc);

                    REQUIRE(can_message.can_signals[CAN_SIGNAL_DRAWER_ID].bit_start == can_signal_drawer_id.bit_start);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_DRAWER_ID].bit_length == can_signal_drawer_id.bit_length);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_DRAWER_ID].data == can_signal_drawer_id.data);

                    REQUIRE(can_message.can_signals[CAN_SIGNAL_OPEN_DRAWER_1].bit_start == can_signal_open_drawer_1.bit_start);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_OPEN_DRAWER_1].bit_length == can_signal_open_drawer_1.bit_length);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_OPEN_DRAWER_1].data == can_signal_open_drawer_1.data);

                    REQUIRE(can_message.can_signals[CAN_SIGNAL_OPEN_DRAWER_2].bit_start == can_signal_open_drawer_2.bit_start);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_OPEN_DRAWER_2].bit_length == can_signal_open_drawer_2.bit_length);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_OPEN_DRAWER_2].data == can_signal_open_drawer_2.data);

                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_RED].bit_start == can_signal_LED_red.bit_start);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_RED].bit_length == can_signal_LED_red.bit_length);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_RED].data == can_signal_LED_red.data);

                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_GREEN].bit_start == can_signal_LED_green.bit_start);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_GREEN].bit_length == can_signal_LED_green.bit_length);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_GREEN].data == can_signal_LED_green.data);

                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_BLUE].bit_start == can_signal_LED_blue.bit_start);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_BLUE].bit_length == can_signal_LED_blue.bit_length);
                    REQUIRE(can_message.can_signals[CAN_SIGNAL_LED_BLUE].data == can_signal_LED_blue.data);
                }

                WHEN("Creating the CanDb class") {
                    robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

                    THEN("The created CanDb class should contain the correct id, dlc and CanSignals with the correct bit_start and bit_length and data should be default 0.") {
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].id == CAN_ID_DRAWER_USER_ACCESS);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].dlc == CAN_DLC_DRAWER_USER_ACCESS);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_DRAWER_ID].bit_start == can_signal_drawer_id.bit_start);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_DRAWER_ID].bit_length == can_signal_drawer_id.bit_length);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_DRAWER_ID].data == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_OPEN_DRAWER_1].bit_start == can_signal_open_drawer_1.bit_start);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_OPEN_DRAWER_1].bit_length == can_signal_open_drawer_1.bit_length);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_OPEN_DRAWER_1].data == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_OPEN_DRAWER_2].bit_start == can_signal_open_drawer_2.bit_start);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_OPEN_DRAWER_2].bit_length == can_signal_open_drawer_2.bit_length);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_OPEN_DRAWER_2].data == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_RED].bit_start == can_signal_LED_red.bit_start);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_RED].bit_length == can_signal_LED_red.bit_length);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_RED].data == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_GREEN].bit_start == can_signal_LED_green.bit_start);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_GREEN].bit_length == can_signal_LED_green.bit_length);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_GREEN].data == 0);

                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_BLUE].bit_start == can_signal_LED_blue.bit_start);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_BLUE].bit_length == can_signal_LED_blue.bit_length);
                        REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].can_signals[CAN_SIGNAL_LED_BLUE].data == 0);
                    }
                }
            }        
        }

        WHEN("Creating the CanFrame class") {
            robast_can_msgs::CanFrame can_frame = robast_can_msgs::CanFrame(msg_id, dlc, u8_can_data); 

            THEN("The created CanFrame class should encapsulate the data correctly") {
                REQUIRE(can_frame.id == msg_id);
                REQUIRE(can_frame.dlc == dlc);
                for (uint8_t i=0; i<dlc; i++) {
                    REQUIRE(can_frame.data[i] == u8_can_data[i]);
                }            
            }    
        }
    }
}

SCENARIO("Test CAN helper functions", "[robast_can_msgs]") {

    GIVEN("A CAN msg_id and dlc as well as data for a drawer_id, open_drawer, LED_red, LED_green, LED_blue") {
        uint32_t msg_id = CAN_ID_DRAWER_USER_ACCESS;;
        uint8_t dlc = 7;
        uint8_t u8_can_data[8] = {0x01,0x02,0x03,0b11000000,0b01000000,0b10000000,0b11000000,0x00};
        uint64_t u64_can_data_expected = 0x010203C04080C000;
        uint64_t u64_can_data_not_expected = 0x0907060509030201;
        uint64_t data_drawer_id = 0x010203;
        uint64_t data_open_drawer_1 = 1;
        uint64_t data_open_drawer_2 = 1;
        uint64_t data_LED_red = 1;
        uint64_t data_LED_green = 2;
        uint64_t data_LED_blue = 3;

        uint8_t bit_start_drawer_id = 0;
        uint8_t bit_length_drawer_id = 24;
        robast_can_msgs::CanSignal can_signal_drawer_id = robast_can_msgs::CanSignal(bit_start_drawer_id, bit_length_drawer_id, data_drawer_id);
        uint8_t bit_start_open_drawer_1 = 24;
        uint8_t bit_length_open_drawer_1 = 1;
        robast_can_msgs::CanSignal can_signal_open_drawer_1 = robast_can_msgs::CanSignal(bit_start_open_drawer_1, bit_length_open_drawer_1, data_open_drawer_1);
        uint8_t bit_start_open_drawer_2 = 25;
        uint8_t bit_length_open_drawer_2 = 1;
        robast_can_msgs::CanSignal can_signal_open_drawer_2 = robast_can_msgs::CanSignal(bit_start_open_drawer_2, bit_length_open_drawer_2, data_open_drawer_2);
        uint8_t bit_start_LED_red = 26;
        uint8_t bit_length_LED_red = 8;
        robast_can_msgs::CanSignal can_signal_LED_red = robast_can_msgs::CanSignal(bit_start_LED_red, bit_length_LED_red, data_LED_red);
        uint8_t bit_start_LED_green = 34;
        uint8_t bit_length_LED_green = 8;
        robast_can_msgs::CanSignal can_signal_LED_green = robast_can_msgs::CanSignal(bit_start_LED_green, bit_length_LED_green, data_LED_green);
        uint8_t bit_start_LED_blue = 42;
        uint8_t bit_length_LED_blue = 8;
        robast_can_msgs::CanSignal can_signal_LED_blue = robast_can_msgs::CanSignal(bit_start_LED_blue, bit_length_LED_blue, data_LED_blue);

        robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
                        msg_id,
                        dlc,
                        {
                            can_signal_drawer_id,
                            can_signal_open_drawer_1,
                            can_signal_open_drawer_2,
                            can_signal_LED_red,
                            can_signal_LED_green,
                            can_signal_LED_blue,
                        });

        robast_can_msgs::CanMessage can_message_invalid_id = robast_can_msgs::CanMessage(
                        0x456,
                        dlc,
                        {
                            can_signal_drawer_id,
                            can_signal_open_drawer_1,
                            can_signal_open_drawer_2,
                            can_signal_LED_red,
                            can_signal_LED_green,
                            can_signal_LED_blue,
                        });

        robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

        robast_can_msgs::CanFrame can_frame = robast_can_msgs::CanFrame(msg_id, dlc, u8_can_data); 

        WHEN("Joining together CAN data bytes from data array") {
            uint64_t u64_can_data = robast_can_msgs::join_together_CAN_data_bytes_from_array(u8_can_data, dlc);

            THEN("The result should correspond to the expeted uint64") {
                REQUIRE(u64_can_data == u64_can_data_expected);
                REQUIRE(u64_can_data != u64_can_data_not_expected);
            }
        }

        WHEN("Joining together CAN data from CAN messages") {
            uint64_t u64_can_data = robast_can_msgs::join_together_CAN_data_from_CAN_message(can_message);

            THEN("The result should correspond to the expeted uint64") {
               REQUIRE(u64_can_data == u64_can_data_expected);
               REQUIRE(u64_can_data != u64_can_data_not_expected);
            }
        }

        WHEN("Swapping Endian") {
            uint64_t input_to_be_swapped = 0x1034567891234560;
            uint64_t input_unswapped = input_to_be_swapped;
            robast_can_msgs::SwapEndian<uint64_t>(input_to_be_swapped);
            THEN("The input_to_be_swapped should have swapped Endian") {
                REQUIRE(input_to_be_swapped == 0x6045239178563410);
                REQUIRE(input_to_be_swapped != input_unswapped);
            }
            WHEN("Swapping Endian twice") {
                uint64_t input_to_be_swapped_twice = input_to_be_swapped;
                robast_can_msgs::SwapEndian<uint64_t>(input_to_be_swapped_twice);
                THEN("The input_to_be_swapped_twice should have the original Endian again") {
                    REQUIRE(input_to_be_swapped_twice == input_unswapped);
                }
            }
        }

        WHEN("Splitting uint64 into Bytes") {
            uint8_t u8_result[8];
            robast_can_msgs::u64_to_eight_bytes(u64_can_data_expected, u8_result);

            THEN("The resulting array should match the expected array") {
                for(uint8_t i=0; i<dlc; i++) {
                    REQUIRE(u8_result[i] == u8_can_data[i]);
                }
            }
        }

        WHEN("Encoding a CAN Message with an id, that exists in the can_db messages") {
            std::optional<robast_can_msgs::CanFrame> encoded_can_frame = robast_can_msgs::encode_can_message(can_message, can_db.can_messages);

            THEN("The resulting CanFrame Class should contain all the data that was contained in the CanMessage class") {
                REQUIRE(encoded_can_frame.has_value());
                REQUIRE(encoded_can_frame.value().id == can_message.id);
                REQUIRE(encoded_can_frame.value().dlc == can_message.dlc);
                for(uint8_t i=0; i<dlc; i++) {
                    REQUIRE(encoded_can_frame.value().data[i] == u8_can_data[i]);
                }
            }
        }

        WHEN("Encoding a CAN Message with an id, that does not exist in the can_db messages") {
            std::optional<robast_can_msgs::CanFrame> encoded_can_frame = robast_can_msgs::encode_can_message(can_message_invalid_id, can_db.can_messages);

            THEN("The resulting CanFrame should not contain a value") {
                REQUIRE_FALSE(encoded_can_frame.has_value());
            }
        }

        WHEN("Decoding a CAN Message with an id, that exists in the can_db messages") {
            std::optional<robast_can_msgs::CanMessage> decoded_can_message = robast_can_msgs::decode_can_message(msg_id, u8_can_data, dlc, can_db.can_messages);

            THEN("The resulting CanMessage Class should contain all the data that was contained in the CanMessage class") {
                REQUIRE(decoded_can_message.has_value());
                REQUIRE(decoded_can_message.value().id == can_message.id);
                REQUIRE(decoded_can_message.value().dlc == can_message.dlc);

                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_DRAWER_ID].bit_start == bit_start_drawer_id);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_DRAWER_ID].bit_length == bit_length_drawer_id);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_DRAWER_ID].data == data_drawer_id);

                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_OPEN_DRAWER_1].bit_start == bit_start_open_drawer_1);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_OPEN_DRAWER_1].bit_length == bit_length_open_drawer_1);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_OPEN_DRAWER_1].data == data_open_drawer_1);

                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_OPEN_DRAWER_2].bit_start == bit_start_open_drawer_2);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_OPEN_DRAWER_2].bit_length == bit_length_open_drawer_2);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_OPEN_DRAWER_2].data == data_open_drawer_2);

                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_RED].bit_start == bit_start_LED_red);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_RED].bit_length == bit_length_LED_red);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_RED].data == data_LED_red);

                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_GREEN].bit_start == bit_start_LED_green);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_GREEN].bit_length == bit_length_LED_green);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_GREEN].data == data_LED_green);

                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_BLUE].bit_start == bit_start_LED_blue);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_BLUE].bit_length == bit_length_LED_blue);
                REQUIRE(decoded_can_message.value().can_signals[CAN_SIGNAL_LED_BLUE].data == data_LED_blue);
            }
        }

        WHEN("Decoding a CAN Message with an id, that does not exist in the can_db messages") {
            uint32_t invalid_msg_id = 0x456;
            std::optional<robast_can_msgs::CanMessage> decoded_can_message = robast_can_msgs::decode_can_message(invalid_msg_id, u8_can_data, dlc, can_db.can_messages);

            THEN("The resulting CanFrame should not contain a value") {
                REQUIRE_FALSE(decoded_can_message.has_value());
            }
        }
    }
}