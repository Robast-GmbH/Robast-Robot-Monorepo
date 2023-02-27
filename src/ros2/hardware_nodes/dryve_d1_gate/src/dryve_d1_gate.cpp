#include "dryve_d1_gate/dryve_d1_gate.hpp"

namespace dryve_d1_gate
{
  DryveD1Gate::DryveD1Gate() : Node("dryve_d1_gate")
  {
    // Declare the first object of the class D1 with the name "xAxis" (IP Address of the D1 as String, Port Number as
    // Int)
    D1 xAxis("10.10.13.6", 502);

    // Set the Debug Mode to ON or OFF; Debug Mode displays all received telegrams from the D1 in the console
    xAxis.setDebugModeON();
    // xAxis.setDebugModeOFF();

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
    xAxis.homing(
        10, 1, 100);   // Homing (Switch search speed[°/s], Zero search speed[°/s], Acceleration for Homing[°/s²])
    //}

    // Move to the Position 90° with an acceleration of 300 °/s² and a velocity of 30 °/s
    xAxis.profilePositionAbs(
        90, 60, 600, 600);   // Profile Position Mode (Position[°], Velocity[°/s], Acceleration[°/s²],
                             // Deceleration[°/s]) Moves the X axis to absolute position 90°

    // Read the value of the Object 6064.0 "Position Actual Value" and display it on the console
    std::cout << "Position Actual Value: " << xAxis.readObjectValue(0x60, 0x64, 0) << std::endl;

    // Several absolute and relative Movements
    xAxis.profilePositionAbs(
        0, 60, 600, 600);   // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²], Deceleration[°/s])
                            // Moves the X axis to absolute position 0°
    xAxis.profilePositionAbs(
        180, 240, 2400, 2400);   // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
                                 // Deceleration[°/s]) Moves the X axis to absolute position 180°
    xAxis.profilePositionAbs(
        0, 240, 2400, 2400);   // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
                               // Deceleration[°/s]) Moves the X axis to absolute position 0°
    xAxis.profilePositionAbs(
        270, 480, 4800, 4800);   // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
                                 // Deceleration[°/s]) Moves the X axis to absolute position 270°
    xAxis.profilePositionAbs(
        0, 480, 4800, 4800);   // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
                               // Deceleration[°/s]) Moves the X axis to absolute position 0°
    xAxis.profilePositionAbs(
        360, 960, 9600, 9600);   // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
                                 // Deceleration[°/s]) Moves the X axis to absolute position 360°
    xAxis.profilePositionAbs(
        0, 960, 9600, 9600);   // Profile Position Mode(Position[°], Velocity[°/s], Acceleration[°/s²],
                               // Deceleration[°/s]) Moves the X axis to absolute position 0°
    sleep(1000);               // Wait 1 second
    xAxis.profilePositionRel(
        -720, 600, 3000, 3000);   // Profile Position Mode (Position[°], Velocity[°/s], Acceleration[°/s²],
                                  // Deceleration[°/s]) Moves the X axis relative 720° counter clockwise
    xAxis.profilePositionRel(
        1440, 600, 3000, 3000);   // Profile Position Mode (Position[°], Velocity[°/s], Acceleration[°/s²],
                                  // Deceleration[°/s]) Moves the X axis relative 1440° clockwise
    sleep(1000);                  // Wait 1 second

    // Move the motor with a set target velocity
    xAxis.profileVelocity(2000,
                          20000);   // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/s²],
                                    // Deceleration[°/s]) Moves the X axis with a target velocity of 9000 °/s clockwise
    sleep(2000);   // Wait for 2 seconds before a new movement is started
    xAxis.profileVelocity(
        -600, 3000, 20000);   // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/ ²], Deceleration[°/s])
                              // Moves the X axis with a target velocity of 600 °/s counter-clockwise
    sleep(2000);              // Wait for 2 seconds before a new movement is started
    xAxis.profileVelocity(0, 6000, 3000);   // Profile Velocity Mode(Target Velocity[°/s], Acceleration[°/s²],
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
