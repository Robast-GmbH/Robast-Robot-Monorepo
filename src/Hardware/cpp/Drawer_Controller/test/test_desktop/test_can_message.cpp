#include <unity.h>
#include <string>
#include <iostream>
#include <stdio.h>

#include "can_db.hpp"
#include "can_frame.h"
#include "can_helper.h"

uint32_t msg_id;
uint8_t dlc;
uint8_t u8_can_data[8] = {0x01,0x02,0x03,0b11000000,0b01000000,0b10000000,0b11000001,0b11001000};
uint64_t can_data_expected;
uint64_t can_data_not_expected;
uint64_t drawer_id;
uint64_t data_open_drawer_1;
uint64_t data_open_drawer_2;
uint64_t data_LED_red;
uint64_t data_LED_green;
uint64_t data_LED_blue;    
uint64_t data_LED_brightness;
uint64_t data_LED_mode;

void setUp(void)
{
    msg_id = 0x01;
    dlc = 8;
    drawer_id = 0x010203;
    data_open_drawer_1 = 1;
    data_open_drawer_2 = 1;
    data_LED_red = 1;
    data_LED_green = 2;
    data_LED_blue = 3;
    data_LED_brightness = 7;
    data_LED_mode = 1; 

    can_data_expected = 0x010203C04080C1C8;
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
                    CAN_DLC_DRAWER_USER_ACCESS,
                    {
                        robast_can_msgs::CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, 0),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_OPEN_LOCK_1_BIT_START, CAN_SIGNAL_OPEN_LOCK_1_BIT_LENGTH, 0),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_OPEN_LOCK_2_BIT_START, CAN_SIGNAL_OPEN_LOCK_2_BIT_LENGTH, 0),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_RED_BIT_START, CAN_SIGNAL_LED_RED_BIT_LENGTH, data_LED_red),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_GREEN_BIT_START, CAN_SIGNAL_LED_GREEN_BIT_LENGTH, data_LED_green),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_BLUE_BIT_START, CAN_SIGNAL_LED_BLUE_BIT_LENGTH, data_LED_blue),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_BRIGHTNESS_BIT_START, CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH, 0),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_MODE_BIT_START, CAN_SIGNAL_LED_MODE_BIT_LENGTH, 0),
                    });
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();

  robast_can_msgs::CanFrame can_frame(msg_id, dlc, u8_can_data);

  TEST_ASSERT_EQUAL_UINT32(msg_id, can_db.can_messages[0].get_id());
  TEST_ASSERT_EQUAL_UINT32(msg_id, can_message.get_id());

  TEST_ASSERT_EQUAL_UINT64(data_LED_red, can_message.get_can_signals()[CAN_SIGNAL_LED_RED].get_data());
  TEST_ASSERT_EQUAL_UINT64(data_LED_green, can_message.get_can_signals()[CAN_SIGNAL_LED_GREEN].get_data());
  TEST_ASSERT_EQUAL_UINT64(data_LED_blue, can_message.get_can_signals()[CAN_SIGNAL_LED_BLUE].get_data());

  TEST_ASSERT_EQUAL_UINT8(u8_can_data[0], can_frame.get_data()[0]);
  TEST_ASSERT_EQUAL_UINT8(u8_can_data[1], can_frame.get_data()[1]);
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
                    CAN_DLC_DRAWER_USER_ACCESS,
                    {
                        robast_can_msgs::CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, drawer_id),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_OPEN_LOCK_1_BIT_START, CAN_SIGNAL_OPEN_LOCK_1_BIT_LENGTH, data_open_drawer_1),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_OPEN_LOCK_2_BIT_START, CAN_SIGNAL_OPEN_LOCK_2_BIT_LENGTH, data_open_drawer_2),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_RED_BIT_START, CAN_SIGNAL_LED_RED_BIT_LENGTH, data_LED_red),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_GREEN_BIT_START, CAN_SIGNAL_LED_GREEN_BIT_LENGTH, data_LED_green),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_BLUE_BIT_START, CAN_SIGNAL_LED_BLUE_BIT_LENGTH, data_LED_blue),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_BRIGHTNESS_BIT_START, CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH, data_LED_brightness),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_MODE_BIT_START, CAN_SIGNAL_LED_MODE_BIT_LENGTH, data_LED_mode),
                    });

  uint64_t can_data = robast_can_msgs::join_together_CAN_data_from_CAN_message(can_message);

  TEST_ASSERT_EQUAL_UINT64(can_data_expected, can_data);
}

void test_decode_can_message(void)
{
  robast_can_msgs::CanDb can_db = robast_can_msgs::CanDb();
  uint64_t can_data_expected_drawer_user_access[8] = {drawer_id, data_open_drawer_1, data_open_drawer_2, data_LED_red, data_LED_green, data_LED_blue, data_LED_brightness, data_LED_mode};

  std::optional<robast_can_msgs::CanMessage> can_message = robast_can_msgs::decode_can_message(msg_id, u8_can_data, dlc, can_db.can_messages);

  if (can_message.has_value()) {
    for (int j = 0; j < can_db.can_messages.size(); j++)
    {
      if (msg_id == can_db.can_messages[j].get_id())
      {
        for (int i = 0; i < can_db.can_messages[j].get_can_signals().size(); i++)
        {
          TEST_ASSERT_EQUAL_UINT64(can_data_expected_drawer_user_access[i], can_message.value().get_can_signals()[i].get_data());
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
                    CAN_DLC_DRAWER_USER_ACCESS,
                    {
                        robast_can_msgs::CanSignal(CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_START, CAN_SIGNAL_DRAWER_CONTROLLER_ID_BIT_LENGTH, drawer_id),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_OPEN_LOCK_1_BIT_START, CAN_SIGNAL_OPEN_LOCK_1_BIT_LENGTH, data_open_drawer_1),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_OPEN_LOCK_2_BIT_START, CAN_SIGNAL_OPEN_LOCK_2_BIT_LENGTH, data_open_drawer_2),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_RED_BIT_START, CAN_SIGNAL_LED_RED_BIT_LENGTH, data_LED_red),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_GREEN_BIT_START, CAN_SIGNAL_LED_GREEN_BIT_LENGTH, data_LED_green),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_BLUE_BIT_START, CAN_SIGNAL_LED_BLUE_BIT_LENGTH, data_LED_blue),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_BRIGHTNESS_BIT_START, CAN_SIGNAL_LED_BRIGHTNESS_BIT_LENGTH, data_LED_brightness),
                        robast_can_msgs::CanSignal(CAN_SIGNAL_LED_MODE_BIT_START, CAN_SIGNAL_LED_MODE_BIT_LENGTH, data_LED_mode),
                    });

  robast_can_msgs::CanFrame can_frame = robast_can_msgs::encode_can_message_into_can_frame(can_message, can_db.can_messages);

  // printf(" can_data: %llu", can_data);

  for (int j = 0; j < can_db.can_messages.size(); j++)
  {
    if (can_frame.get_id() == can_db.can_messages[j].get_id())
    {
      for (int i = 0; i < can_frame.get_dlc(); i++)
      {
        printf(" u8_can_data[%d]: %d", i, u8_can_data[i]);
        printf(" can_frame.value().get_data()[%d]: %d", i, can_frame.get_data()[i]);
        TEST_ASSERT_EQUAL_UINT8(u8_can_data[i], can_frame.get_data()[i]);
      }
    }
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