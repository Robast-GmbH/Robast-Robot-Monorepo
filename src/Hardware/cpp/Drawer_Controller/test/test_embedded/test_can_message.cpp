#include <Arduino.h>
#include <unity.h>
#include <iostream>

#include <CAN.h>
#include "robast_can_msgs/can_message/can_message.h"
#include "robast_can_msgs/can_db/can_db.h"


CAN_frame_t rx_frame;
uint8_t u8_can_data[8];
uint64_t drawer_id;
uint64_t open_drawer;
uint64_t LED_red;
uint64_t LED_green;
uint64_t LED_blue;

void setUp(void) {
    rx_frame.MsgID = 0x01;
    rx_frame.FIR.B.DLC = 8;
    drawer_id = 0x010203;
    open_drawer = 1;
    LED_red = 1;
    LED_green = 1;
    LED_blue = 1;

    uint8_t u8_can_data[8] = {0x01,0x02,0x03,0b10000000,0b10000000,0b10000000,0b10000000,0x00};
    
    std::copy(std::begin(u8_can_data), std::end(u8_can_data), std::begin(rx_frame.data.u8)); // copy data array to rx_frame
}

void tearDown(void) {
  // clean stuff up here
}

void test_testing_data(void) {
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

  robast_can_msgs::CanMessage can_message = robast_can_msgs::CanMessage(
                    0x01,
                    "drawer_user_access",
                    {
                        {"drawer_id", 0, 24, 0},
                        {"open_drawer", 24, 1, 0},
                        {"LED_red", 25, 8, 0},
                        {"LED_green", 33, 8, 0},
                        {"LED_blue", 41, 8, 0},
                    });

  TEST_ASSERT_EQUAL_UINT32(2, can_db.can_messages.size());

  TEST_ASSERT_EQUAL_UINT32_MESSAGE(rx_frame.MsgID, can_db.can_messages[0].id, std::to_string(can_db.can_messages[0].id).c_str());
  TEST_ASSERT_EQUAL_UINT32_MESSAGE(rx_frame.MsgID, can_message.id, std::to_string(can_message.id).c_str());
  TEST_ASSERT_EQUAL_STRING("drawer_user_access", can_db.can_messages[0].name.c_str());
}

// test_function_should_doBlahAndBlah
void test_join_together_CAN_data_bytes_from_array(void) {
  // Mind that for the tests unity doesn't support 64-bit, although it should be possible via the unity_config.h
  // Error message is: "FAIL: Unity 64-bit Support Disabled"
  // At the moment this can't be fixed despite great effort, therefore 64 Bits are separated into 2 x 32 Bits
  uint64_t can_data_64;
  uint32_t can_data_32_1;
  uint32_t can_data_32_2;

  uint64_t can_data_expected_64 = 0x0102038080808000;
  uint32_t can_data_expected_32_1 = (uint32_t) ((can_data_expected_64 & 0xFFFFFFFF00000000) >> 32);
  uint32_t can_data_expected_32_2 = (uint32_t) (can_data_expected_64 & 0xFFFFFFFF);

  uint64_t can_data_not_expected_64 = 0x0907060509030201;
  uint32_t can_data_not_expected_32_1 = (uint32_t) ((can_data_not_expected_64 & 0xFFFFFFFF00000000) >> 32);
  uint32_t can_data_not_expected_32_2 = (uint32_t) (can_data_not_expected_64 & 0xFFFFFFFF);

  can_data_64 = robast_can_msgs::join_together_CAN_data_bytes_from_array(rx_frame);

  can_data_32_1 = (uint32_t) ((can_data_64 & 0xFFFFFFFF00000000) >> 32);
  can_data_32_2 = (uint32_t) (can_data_64 & 0xFFFFFFFF);
  TEST_ASSERT_EQUAL_UINT32(can_data_expected_32_1, can_data_32_1);
  TEST_ASSERT_EQUAL_UINT32(can_data_expected_32_2, can_data_32_2);
  TEST_ASSERT_NOT_EQUAL(can_data_not_expected_32_1, can_data_32_1);
  TEST_ASSERT_NOT_EQUAL(can_data_not_expected_32_2, can_data_32_2);
}

void test_decode_can_message(void)
{
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
  uint64_t can_data_expected_64[5] = {drawer_id, open_drawer, LED_red, LED_green, LED_blue};

  std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(rx_frame, can_db.can_messages);

  if (can_message.has_value()) {
    for (int j = 0; j < can_db.can_messages.size(); j++)
    {
      if (rx_frame.MsgID == can_db.can_messages[j].id)
      {
        for (int i = 0; i < can_db.can_messages[j].can_signals.size(); i++) {
          uint32_t can_data_32_1 = (uint32_t) ((can_message.value().can_signals[i].data & 0xFFFFFFFF00000000) >> 32);
          uint32_t can_data_32_2 = (uint32_t) (can_message.value().can_signals[i].data & 0xFFFFFFFF);

          uint32_t can_data_expected_32_1 = (uint32_t) ((can_data_expected_64[i] & 0xFFFFFFFF00000000) >> 32);
          uint32_t can_data_expected_32_2 = (uint32_t) (can_data_expected_64[i] & 0xFFFFFFFF);

          TEST_ASSERT_EQUAL_UINT32(can_data_expected_32_1, can_data_32_1);
          TEST_ASSERT_EQUAL_UINT32(can_data_expected_32_2, can_data_32_2);
        }
      }
    }
  } else {
    TEST_FAIL();
  }  
}

int runUnityTests(void) {
  UNITY_BEGIN();

  RUN_TEST(test_testing_data);
  RUN_TEST(test_join_together_CAN_data_bytes_from_array);
  RUN_TEST(test_decode_can_message);
  
  return UNITY_END();
}

/**
  * For native dev-platform or for some embedded frameworks
  */
int main(void) {
  return runUnityTests();
}

/**
  * For Arduino framework
  */
void setup() {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);

  runUnityTests();
}
void loop() {}