#include "dryve_d1_bridge/dryve_d1_bridge.hpp"

namespace dryve_d1_bridge
{
  DryveD1Gate::DryveD1Gate() : Node("dryve_d1_bridge")
  {
    // Note: This project is only used for testing purposes with the dryve d1

    try
    {
      execute();
    }
    catch (const std::exception& e)
    {
      _dryve_d1->close_connection();
    }
  }

  void DryveD1Gate::execute()
  {
    // To run this on the robot use the individual IP address of the d1 within the robot network
    // To run this from another pc in the robast network in the office use: 10.10.23.7
    // As this file is only for debugging, we use the 10.10.23.7
    std::unique_ptr<D1> _dryve_d1 =
        std::make_unique<D1>(RB_THERON_IP_ADDRESS, RB_THERON_PORT_Y_AXIS, std::make_unique<SocketWrapper>());

    const double SI_UNIT_FACTOR = 100.0;

    RCLCPP_INFO(this->get_logger(), "Setting debug mode on!");   // Debugging
    _dryve_d1->set_debug_mode_on();

    RCLCPP_INFO(this->get_logger(), "I am going to run through D1 state machine now ...");   // Debugging

    // Run through State Machine --> Current is applied to the motor
    _dryve_d1->run_dryve_state_machine();

    RCLCPP_INFO(this->get_logger(), "Start homing now ...");   // Debugging
    _dryve_d1->start_dryve_homing(3, 1, 10);

    double current_position =
        (static_cast<double>(_dryve_d1->read_object_value(_dryve_d1->OBJECT_INDEX_1_READ_POSITION_ACTUAL_VALUE,
                                                          _dryve_d1->OBJECT_INDEX_2_READ_POSITION_ACTUAL_VALUE)) /
         SI_UNIT_FACTOR) *
        MM_TO_M;

    RCLCPP_INFO(this->get_logger(), "Current position: %f", current_position);

    RCLCPP_INFO(this->get_logger(), "Move to absolute position now!");   // Debugging
    // Please mind: Move to absolute position does only work if the axis was homed before!
    _dryve_d1->move_profile_to_absolute_position(0.20 * 1000, 3, 3);

    _dryve_d1->close_connection();
  }

  DryveD1Gate::~DryveD1Gate()
  {
  }
}   // namespace dryve_d1_bridge
