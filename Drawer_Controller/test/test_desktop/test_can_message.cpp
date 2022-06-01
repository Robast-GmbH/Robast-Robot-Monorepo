#include <unity.h>
#include <string>
#include <iostream>

#include "robast_can_msgs/can_message/can_message.h"
#include "robast_can_msgs/can_db/can_db.h"

CAN_frame_t rx_frame;
uint8_t u8_can_data[8];
uint64_t can_data_expected;
uint64_t can_data_not_expected;
uint64_t drawer_id;
uint64_t open_drawer;
uint64_t LED_red;
uint64_t LED_green;
uint64_t LED_blue;

void setUp(void)
{
    rx_frame.MsgID = 0x01;
    rx_frame.FIR.B.DLC = 8;
    drawer_id = 0x010203;
    open_drawer = 1;
    LED_red = 1;
    LED_green = 1;
    LED_blue = 0;

    uint8_t u8_can_data[8] = {0x01,0x02,0x03,0b10000000,0b10000000,0b10000000,0b10000000,0x00};
    can_data_expected = 0x0102038080808000;
    can_data_not_expected = 0x0907060509030201;
    
    std::copy(std::begin(u8_can_data), std::end(u8_can_data), std::begin(rx_frame.data.u8)); // copy data array to rx_frame
}

void tearDown(void)
{
  // clean stuff up here
}

void test_testing_data(void)
{
  robast_can_msgs::CanMessage can_message(
                    0x01,
                    "drawer_user_access",
                    {
                        {"drawer_id", 0, 24, 0},
                        {"open_drawer", 24, 1, 0},
                        {"LED_red", 25, 8, 0},
                        {"LED_green", 33, 8, 0},
                        {"LED_blue", 41, 8, 0},
                    });
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

  TEST_ASSERT_EQUAL_UINT32(rx_frame.MsgID, can_db.can_messages[0].id);
  TEST_ASSERT_EQUAL_UINT32(rx_frame.MsgID, can_message.id);
  TEST_ASSERT_EQUAL_STRING("drawer_user_access", can_db.can_messages[0].name.c_str());
  TEST_ASSERT_EQUAL_STRING("drawer_id", can_db.can_messages[0].can_signals[0].name.c_str());
}

// test_function_should_doBlahAndBlah
void test_join_together_CAN_data_bytes_from_array(void)
{
  uint64_t can_data = robast_can_msgs::join_together_CAN_data_bytes_from_array(rx_frame);

  TEST_ASSERT_FALSE(can_data_not_expected == can_data);
  TEST_ASSERT_TRUE(can_data_expected == can_data);
  TEST_ASSERT_EQUAL_UINT64(can_data_expected, can_data);
}

void test_join_together_CAN_data_from_CAN_message(void)
{
  robast_can_msgs::CanMessage can_message(
                    0x01,
                    "drawer_user_access",
                    {
                        {"drawer_id", 0, 24, drawer_id},
                        {"open_drawer", 24, 1, open_drawer},
                        {"LED_red", 25, 8, LED_red},
                        {"LED_green", 33, 8, LED_green},
                        {"LED_blue", 41, 8, LED_blue},
                    });

  uint64_t can_data = robast_can_msgs::join_together_CAN_data_from_CAN_message(can_message);

  TEST_ASSERT_EQUAL_UINT64(can_data_expected ,can_data);
}

void test_decode_can_message(void)
{
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
  uint64_t can_data_expected_drawer_user_access[5] = {drawer_id, open_drawer, LED_red, LED_green, LED_blue};

  std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(rx_frame, can_db.can_messages);

  if (can_message.has_value()) {
    for (int j = 0; j < can_db.can_messages.size(); j++)
    {
      if (rx_frame.MsgID == can_db.can_messages[j].id)
      {
        for (int i = 0; i < can_db.can_messages[j].can_signals.size(); i++)
        {
          TEST_ASSERT_EQUAL_UINT64(can_data_expected_drawer_user_access[i], can_message.value().can_signals[i].data);
        }
      }
    }
  }
  else
  {
    TEST_FAIL();
  }  
}

void test_encode_can_message(void)
{
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
  robast_can_msgs::CanMessage can_message(
                    0x01,
                    "drawer_user_access",
                    {
                        {"drawer_id", 0, 24, drawer_id},
                        {"open_drawer", 24, 1, open_drawer},
                        {"LED_red", 25, 8, LED_red},
                        {"LED_green", 33, 8, LED_green},
                        {"LED_blue", 41, 8, LED_blue},
                    });

  std::optional<CAN_frame_t> can_frame = robast_can_msgs::encode_can_message(can_message, can_db.can_messages);
  CAN_frame_t can_frame_value = can_frame.value();

  if (can_frame.has_value()) {
    for (int j = 0; j < can_db.can_messages.size(); j++)
    {
      if (can_frame.value().MsgID == can_db.can_messages[j].id)
      {
        for (int i = 0; i < can_frame.value().FIR.B.DLC; i++)
        {
          TEST_ASSERT_EQUAL_UINT8(rx_frame.data.u8[i], can_frame.value().data.u8[i]);
        }
      }
    }
  }
  else
  {
    TEST_FAIL();
  }  
}

void test_get_dlc_of_can_message(void)
{
  robast_can_msgs::CanMessage can_message(
                    0x01,
                    "drawer_user_access",
                    {
                        {"drawer_id", 0, 24, drawer_id},
                        {"open_drawer", 24, 1, open_drawer},
                        {"LED_red", 25, 8, LED_red},
                        {"LED_green", 33, 8, LED_green},
                        {"LED_blue", 41, 8, LED_blue},
                    });

  uint8_t dlc = robast_can_msgs::get_dlc_of_can_message(can_message);

  TEST_ASSERT_EQUAL_UINT8(7, dlc);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_testing_data);
    RUN_TEST(test_join_together_CAN_data_bytes_from_array);
    RUN_TEST(test_join_together_CAN_data_from_CAN_message);
    RUN_TEST(test_get_dlc_of_can_message);
    RUN_TEST(test_decode_can_message);
    RUN_TEST(test_encode_can_message);

    UNITY_END();

    return 0;
}