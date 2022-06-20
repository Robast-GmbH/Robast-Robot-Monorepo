#include "catch.hpp"

#include "../can_db/can_db.h"

/*
* HOW TO RUN THIS TEST:
* g++ -c .\tests_main.cpp
* g++ tests_main.o tests_can_db.cpp -o test_executable -I ..\..\..\robast_msgs\
* .\test_executable.exe
*/

SCENARIO("Test can_db and the underlying classes CanMessage, CanSignal and CanFrame", "[can_db]") {

    GIVEN("A CanMessage with a msg_id and dlc containing data for a drawer_id, open_drawer, LED_red, LED_green, LED_blue") {
        uint32_t msg_id = CAN_ID_DRAWER_USER_ACCESS;;
        uint8_t dlc = 7;
        uint8_t u8_can_data[8] = {0x01,0x02,0x03,0b10000000,0b10000001,0b00000001,0b10000000,0x00};
        uint64_t can_data_expected = 0x0102038081018000;
        uint64_t can_data_not_expected = 0x0907060509030201;
        uint64_t data_drawer_id = 0x010203;
        uint64_t data_open_drawer = 1;
        uint64_t data_LED_red = 1;
        uint64_t data_LED_green = 2;
        uint64_t data_LED_blue = 3;        

        robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

        robast_can_msgs::CanFrame can_frame = robast_can_msgs::CanFrame(msg_id, dlc, u8_can_data); 


        WHEN("Creating the CanSignal classes") {
            uint8_t bit_start_drawer_id = 0;
            uint8_t bit_length_drawer_id = 24;
            robast_can_msgs::CanSignal can_signal_drawer_id = robast_can_msgs::CanSignal(bit_start_drawer_id, bit_length_drawer_id, data_drawer_id);

            uint8_t bit_start_open_drawer = 24;
            uint8_t bit_length_open_drawer = 1;
            robast_can_msgs::CanSignal can_signal_open_drawer = robast_can_msgs::CanSignal(bit_start_open_drawer, bit_length_open_drawer, data_open_drawer);

            uint8_t bit_start_LED_red = 25;
            uint8_t bit_length_LED_red = 8;
            robast_can_msgs::CanSignal can_signal_LED_red = robast_can_msgs::CanSignal(bit_start_LED_red, bit_length_LED_red, data_LED_red);

            uint8_t bit_start_LED_green = 33;
            uint8_t bit_length_LED_green = 8;
            robast_can_msgs::CanSignal can_signal_LED_green = robast_can_msgs::CanSignal(bit_start_LED_green, bit_length_LED_green, data_LED_green);

            uint8_t bit_start_LED_blue = 41;
            uint8_t bit_length_LED_blue = 8;
            robast_can_msgs::CanSignal can_signal_LED_blue = robast_can_msgs::CanSignal(bit_start_LED_blue, bit_length_LED_blue, data_LED_blue);

            THEN("The created CanSignal classes should encapsulate the data correctly") {
                REQUIRE(can_signal_drawer_id.bit_start == bit_start_drawer_id);
                REQUIRE(can_signal_drawer_id.bit_length == bit_length_drawer_id);
                REQUIRE(can_signal_drawer_id.data == data_drawer_id);

                REQUIRE(can_signal_open_drawer.bit_start == bit_start_open_drawer);
                REQUIRE(can_signal_open_drawer.bit_length == bit_length_open_drawer);
                REQUIRE(can_signal_open_drawer.data == data_open_drawer);

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
        }

        // WHEN("Creating the CanMessage class") {


        //     robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
        //             msg_id,
        //             dlc,
        //             {
        //                 robast_can_msgs::CanSignal(0, 24, drawer_id),
        //                 robast_can_msgs::CanSignal(24, 1, open_drawer),
        //                 robast_can_msgs::CanSignal(25, 8, LED_red),
        //                 robast_can_msgs::CanSignal(33, 8, LED_green),
        //                 robast_can_msgs::CanSignal(41, 8, LED_blue),
        //             });

        //     THEN("The created classes should encapsulate the data correctly") {
        //         REQUIRE(can_message.id == msg_id);
        //         REQUIRE(can_message.dlc == dlc);
        //         REQUIRE(can_message.can_signals[CAN_SIGNAL_DRAWER_ID].bit_start == 0);
        //     }
        // }

        // WHEN("Checking that creating of the classes encapsulate the data correctly") {

        //     THEN("The created classes should encapsulate the data correctly") {
        //         REQUIRE(can_db.can_messages[CAN_MSG_DRAWER_USER_ACCESS].id == msg_id);
        //     }
        // }

    }
}