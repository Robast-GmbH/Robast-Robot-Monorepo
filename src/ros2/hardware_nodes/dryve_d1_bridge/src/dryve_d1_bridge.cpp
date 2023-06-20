#include "dryve_d1_bridge/dryve_d1_bridge.hpp"

namespace dryve_d1_bridge
{
  DryveD1Gate::DryveD1Gate() : Node("dryve_d1_bridge")
  {
    this->execute();

    rclcpp::shutdown();
  }

  void DryveD1Gate::execute()
  {
    // Declare the first object of the class D1 with the name "xAxis" (IP Address of the D1 as String, Port Number as
    // Int)
    D1 xAxis("10.10.13.6", 502, std::make_unique<SocketWrapper>());

    // Set the Debug Mode to ON or OFF; Debug Mode displays all received telegrams from the D1 in the console
    // xAxis.set_debug_mode_on();
    xAxis.set_debug_mode_off();

    RCLCPP_INFO(this->get_logger(), "I am going to run through D1 state machine now ...");   // Debugging

    // Run through State Machine --> Current is applied to the motor
    xAxis.run_dryve_state_machine();

    RCLCPP_INFO(this->get_logger(), "Reading object value 0x2014");   // Debugging

    // Read the value of the Object 2014.1 "Status Flags" and asigned it to a variable (First Objectindex in
    // hexadecimal, Second Objectindex in hexadecimal, Subindex)
    int ref = xAxis.read_object_value(0x20, 0x14, 0);

    // Start Homing only if not yet referenced
    // if (ref == 0)
    //{
    // xAxis.start_dryve_homing(
    //     10, 1, 100);   // Homing (Switch search speed[°/s], Zero search speed[°/s], Acceleration for Homing[°/s²])
    // //}

    // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
    xAxis.move_profile_to_relative_position(90, 500, 500, 500);
    sleep(2);
    xAxis.move_profile_to_relative_position(90, 500, 500, 500);
    sleep(2);
    xAxis.move_profile_to_relative_position(90, 500, 500, 500);
    sleep(2);

    // Move the motor with a set target velocity
    // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/s²],
    xAxis.set_profile_velocity(90, 100, 100);
    sleep(1);   // Wait for 2 seconds before a new movement is started

    // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/ ²], Deceleration[°/s])
    xAxis.set_profile_velocity(-90, 100, 100);
    sleep(1);   // Wait for 2 seconds before a new movement is started

    xAxis.set_profile_velocity(90, 100, 100);
    sleep(1);   // Wait for 2 seconds before a new movement is started

    // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/ ²], Deceleration[°/s])
    xAxis.set_profile_velocity(-90, 100, 100);
    sleep(1);

    xAxis.set_profile_velocity(0, 100, 100);   // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/s²],
                                               // Deceleration[°/s]) Stops the movement

    // Shutdown the motor when the dryve in the state "Ready" --> no current is applied anymore to the motor
    xAxis.wait_for_dryve_ready_state();
    xAxis.set_dryve_shutdown_state();

    // Gracefully close everything down
    close(xAxis.sock);
  }

  DryveD1Gate::~DryveD1Gate()
  {
  }
}   // namespace dryve_d1_bridge
