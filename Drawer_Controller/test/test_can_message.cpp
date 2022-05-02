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
  uint64_t can_data;
  uint64_t can_data_expected = 0x0807060504030201;

  can_data = robast_can_msgs::join_together_CAN_data_bytes(rx_frame);

  TEST_ASSERT_EQUAL_UINT64(can_data_expected, can_data);
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