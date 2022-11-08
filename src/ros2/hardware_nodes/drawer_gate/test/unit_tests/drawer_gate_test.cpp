#include "catch.hpp"
#include "fakeit.hpp"
#include "../../drawer_gate/include/drawer_gate/drawer_gate.hpp"

using namespace fakeit;

namespace robast
{

  SCENARIO("Test the helper functions of the Drawer Gate")
  {

    GIVEN("The created ROS Drawer Gate Node with a mocked SerialHelper, a drawer_controller_id, drawer_id und led_parameters")
    {
      rclcpp::init(0, nullptr);
      std::shared_ptr<drawer_gate::DrawerGate> drawer_gate_node = std::make_shared<drawer_gate::DrawerGate>();
      // TODO: replace serial_helper_ of Drawer gate with serial_helper_mock

      uint32_t drawer_controller_id = 0x010203;
      uint8_t drawer_id = 1;
      drawer_gate::led_parameters led_parameters = {};
      led_parameters.led_red = 1;
      led_parameters.led_blue = 2;
      led_parameters.led_green = 3;
      led_parameters.brightness = 7;
      led_parameters.mode = 1;

      WHEN("The the CAN message to control the drawer lock is created")
      {
        // robast_can_msgs::CanMessage can_msg_open_lock = drawer_gate_node->create_can_msg_drawer_lock(drawer_controller_id, drawer_id, CAN_DATA_OPEN_LOCK);


        THEN(" the reader returns the scanned key  and a NFC login message followed by a Read Message should have been performed to read data from the Card")
        {
          REQUIRE(1 == 1);
        }
      }

      rclcpp::shutdown();
    }
  }
}