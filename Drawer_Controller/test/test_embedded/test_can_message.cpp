#include <Arduino.h>
#include <unity.h>

#include <CAN.h>
#include "robast_can_msgs/can_message.hpp"

CAN_frame_t rx_frame;
uint8_t u8_can_data[8];

void setUp(void) {
    rx_frame.MsgID = 0x01;
    rx_frame.FIR.B.DLC = 8;
    uint8_t u8_can_data[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
    
    std::copy(std::begin(u8_can_data), std::end(u8_can_data), std::begin(rx_frame.data.u8)); // copy data array to rx_frame
}

void tearDown(void) {
  // clean stuff up here
}

// test_function_should_doBlahAndBlah
void test_join_together_CAN_data_bytes(void) {
  // Mind that for the tests unity doesn't support 64-bit, although it should be possible via the unity_config.h
  // Error message is: "FAIL: Unity 64-bit Support Disabled"
  // At the moment this can't be fixed despite great effort, therefore 64 Bits are separated into 2 x 32 Bits
  uint64_t can_data_64;
  uint32_t can_data_32_1;
  uint32_t can_data_32_2;

  uint64_t can_data_expected_64 = 0x0807060504030201;
  uint32_t can_data_expected_32_1 = (uint32_t) ((can_data_expected_64 & 0xFFFFFFFF00000000) >> 32);
  uint32_t can_data_expected_32_2 = (uint32_t) (can_data_expected_64 & 0xFFFFFFFF);

  uint64_t can_data_not_expected_64 = 0x0907060509030201;
  uint32_t can_data_not_expected_32_1 = (uint32_t) ((can_data_not_expected_64 & 0xFFFFFFFF00000000) >> 32);
  uint32_t can_data_not_expected_32_2 = (uint32_t) (can_data_not_expected_64 & 0xFFFFFFFF);

  can_data_64 = robast_can_msgs::join_together_CAN_data_bytes(rx_frame);

  can_data_32_1 = (uint32_t) ((can_data_64 & 0xFFFFFFFF00000000) >> 32);
  can_data_32_2 = (uint32_t) (can_data_64 & 0xFFFFFFFF);
  TEST_ASSERT_EQUAL_UINT32(can_data_expected_32_1, can_data_32_1);
  TEST_ASSERT_EQUAL_UINT32(can_data_expected_32_2, can_data_32_2);
  TEST_ASSERT_NOT_EQUAL(can_data_not_expected_32_1, can_data_32_1);
  TEST_ASSERT_NOT_EQUAL(can_data_not_expected_32_2, can_data_32_2);
}

int runUnityTests(void) {
  UNITY_BEGIN();

  RUN_TEST(test_join_together_CAN_data_bytes);
  
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