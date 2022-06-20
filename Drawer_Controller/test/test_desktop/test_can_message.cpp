#include <unity.h>
#include <string>
#include <iostream>
#include <stdio.h>

#include "robast_can_msgs/can_db/can_db.h"

uint32_t msg_id;
uint8_t dlc;
uint8_t u8_can_data[8] = {0x01,0x02,0x03,0b10000000,0b10000001,0b00000001,0b10000000,0x00};
uint64_t can_data_expected;
uint64_t can_data_not_expected;
uint64_t drawer_id;
uint64_t open_drawer;
uint64_t LED_red;
uint64_t LED_green;
uint64_t LED_blue;

void setUp(void)
{
    msg_id = 0x01;
    dlc = 8;
    drawer_id = 0x010203;
    open_drawer = 1;
    LED_red = 1;
    LED_green = 2;
    LED_blue = 3;

    can_data_expected = 0x0102038081018000;
    can_data_not_expected = 0x0907060509030201;
}

void tearDown(void)
{
  // clean stuff up here
}

void test_testing_data(void)
{
  printf("Testing Data!");

  robast_can_msgs::CanMessage can_message(
                    CAN_ID_DRAWER_USER_ACCESS,
                    7,
                    {
                        robast_can_msgs::CanSignal(0, 24, 0),
                        robast_can_msgs::CanSignal(24, 1, 0),
                        robast_can_msgs::CanSignal(25, 8, LED_red),
                        robast_can_msgs::CanSignal(33, 8, LED_green),
                        robast_can_msgs::CanSignal(41, 8, LED_blue),
                    });
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

  robast_can_msgs::CanFrame can_frame(msg_id, dlc, u8_can_data);

  TEST_ASSERT_EQUAL_UINT32(msg_id, can_db.can_messages[0].id);
  TEST_ASSERT_EQUAL_UINT32(msg_id, can_message.id);

  TEST_ASSERT_EQUAL_UINT64(LED_red, can_message.can_signals[CAN_SIGNAL_LED_RED].data);
  TEST_ASSERT_EQUAL_UINT64(LED_green, can_message.can_signals[CAN_SIGNAL_LED_GREEN].data);
  TEST_ASSERT_EQUAL_UINT64(LED_blue, can_message.can_signals[CAN_SIGNAL_LED_BLUE].data);

  TEST_ASSERT_EQUAL_UINT8(u8_can_data[0], can_frame.data[0]);
  TEST_ASSERT_EQUAL_UINT8(u8_can_data[1], can_frame.data[1]);
}

// test_function_should_doBlahAndBlah
void test_join_together_CAN_data_bytes_from_array(void)
{
  uint64_t can_data = robast_can_msgs::join_together_CAN_data_bytes_from_array(u8_can_data, dlc);

  TEST_ASSERT_FALSE(can_data_not_expected == can_data);
  TEST_ASSERT_TRUE(can_data_expected == can_data);
  TEST_ASSERT_EQUAL_UINT64(can_data_expected, can_data);
}

void test_join_together_CAN_data_from_CAN_message(void)
{
  robast_can_msgs::CanMessage can_message(
                    CAN_ID_DRAWER_USER_ACCESS,
                    7,
                    {
                        robast_can_msgs::CanSignal(0, 24, drawer_id),
                        robast_can_msgs::CanSignal(24, 1, open_drawer),
                        robast_can_msgs::CanSignal(25, 8, LED_red),
                        robast_can_msgs::CanSignal(33, 8, LED_green),
                        robast_can_msgs::CanSignal(41, 8, LED_blue),
                    });

  uint64_t can_data = robast_can_msgs::join_together_CAN_data_from_CAN_message(can_message);

  TEST_ASSERT_EQUAL_UINT64(can_data_expected, can_data);
}

void test_decode_can_message(void)
{
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
  uint64_t can_data_expected_drawer_user_access[5] = {drawer_id, open_drawer, LED_red, LED_green, LED_blue};

  std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(msg_id, u8_can_data, dlc, can_db.can_messages);

  if (can_message.has_value()) {
    for (int j = 0; j < can_db.can_messages.size(); j++)
    {
      if (msg_id == can_db.can_messages[j].id)
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
                    CAN_ID_DRAWER_USER_ACCESS,
                    7,
                    {
                        robast_can_msgs::CanSignal(0, 24, drawer_id),
                        robast_can_msgs::CanSignal(24, 1, open_drawer),
                        robast_can_msgs::CanSignal(25, 8, LED_red),
                        robast_can_msgs::CanSignal(33, 8, LED_green),
                        robast_can_msgs::CanSignal(41, 8, LED_blue),
                    });

  std::optional<robast_can_msgs::CanFrame> can_frame = robast_can_msgs::encode_can_message(can_message, can_db.can_messages);

  // printf(" can_data: %llu", can_data);

  if (can_frame.has_value()) {
    for (int j = 0; j < can_db.can_messages.size(); j++)
    {
      if (can_frame.value().id == can_db.can_messages[j].id)
      {
        for (int i = 0; i < can_frame.value().dlc; i++)
        {
          printf(" u8_can_data[%d]: %d", i, u8_can_data[i]);
          printf(" can_frame.value().data[%d]: %d", i, can_frame.value().data[i]);
          TEST_ASSERT_EQUAL_UINT8(u8_can_data[i], can_frame.value().data[i]);
        }
      }
    }
  }
  else
  {
    TEST_FAIL();
  }  
}


int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_testing_data);
    RUN_TEST(test_join_together_CAN_data_bytes_from_array);
    RUN_TEST(test_join_together_CAN_data_from_CAN_message);
    RUN_TEST(test_decode_can_message);
    RUN_TEST(test_encode_can_message);

    UNITY_END();

    return 0;
}