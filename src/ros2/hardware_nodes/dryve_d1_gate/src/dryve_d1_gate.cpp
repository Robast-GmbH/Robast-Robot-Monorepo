#include "dryve_d1_gate/dryve_d1_gate.hpp"

namespace dryve_d1_gate
{
  DryveD1Gate::DryveD1Gate() : Node("dryve_d1_gate")
  {
    this->execute();
  }

  void DryveD1Gate::execute()
  {
    // Declare the first object of the class D1 with the name "xAxis" (IP Address of the D1 as String, Port Number as
    // Int)
    D1 xAxis("10.10.13.6", 502);

    // Set the Debug Mode to ON or OFF; Debug Mode displays all received telegrams from the D1 in the console
    // xAxis.setDebugModeON();
    xAxis.setDebugModeOFF();

    RCLCPP_INFO(this->get_logger(), "I am going to run through D1 state machine now ...");   // Debugging

    // Run through State Machine --> Current is applied to the motor
    xAxis.runStateMachine();

    RCLCPP_INFO(this->get_logger(), "Reading object value 0x2014");   // Debugging

    // Read the value of the Object 2014.1 "Status Flags" and asigned it to a variable (First Objectindex in
    // hexadecimal, Second Objectindex in hexadecimal, Subindex)
    int ref = xAxis.readObjectValue(0x20, 0x14, 0);

    // Start Homing only if not yet referenced
    // if (ref == 0)
    //{
    // xAxis.homing(
    //     10, 1, 100);   // Homing (Switch search speed[°/s], Zero search speed[°/s], Acceleration for Homing[°/s²])
    // //}

    // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
    xAxis.profilePositionRel(90, 500, 500, 500);
    sleep(5);
    xAxis.profilePositionRel(90, 500, 500, 500);
    sleep(5);
    xAxis.profilePositionRel(90, 500, 500, 500);
    sleep(5);

    // Move the motor with a set target velocity
    // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/s²],
    xAxis.profileVelocity(90, 100, 100);
    sleep(2);   // Wait for 2 seconds before a new movement is started

    // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/ ²], Deceleration[°/s])
    xAxis.profileVelocity(-90, 100, 100);
    sleep(2);   // Wait for 2 seconds before a new movement is started

    xAxis.profileVelocity(90, 100, 100);
    sleep(2);   // Wait for 2 seconds before a new movement is started

    // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/ ²], Deceleration[°/s])
    xAxis.profileVelocity(-90, 100, 100);
    sleep(2);

    xAxis.profileVelocity(0, 100, 100);   // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/s²],
                                          // Deceleration[°/s]) Stops the movement

    // Shutdown the motor when the dryve in the state "Ready" --> no current is applied anymore to the motor
    xAxis.waitForReady();
    xAxis.setShutdown();

    // Gracefully close everything down
    close(xAxis.sock);
  }

  DryveD1Gate::~DryveD1Gate()
  {
  }
}   // namespace dryve_d1_gate
