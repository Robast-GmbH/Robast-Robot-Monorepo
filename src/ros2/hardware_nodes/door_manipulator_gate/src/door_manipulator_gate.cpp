#include "door_manipulator_gate/door_manipulator_gate.hpp"

namespace door_manipulator_gate
{
  DoorManipulatorGate::DoorManipulatorGate(const std::string serial_path) : Node("door_manipulator_gate")
  {
    this->serial_helper_ = new serial_helper::SerialHelper(serial_path);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 2));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    this->norelem_stepper_control_subscription_ = this->create_subscription<NorelemStepperControl>(
        "control_norelem_stepper", qos,
        std::bind(&DoorManipulatorGate::control_norelem_stepper_topic_callback, this, std::placeholders::_1));

    this->send_ascii_cmds_timer_ = this->create_wall_timer(
        TIMER_PERIOD_SEND_ASCII_CMDS, std::bind(&DoorManipulatorGate::send_ascii_cmds_timer_callback, this));

    this->serial_can_usb_converter_is_set_up_ = false;
    this->cleared_serial_buffer_from_old_can_msgs_ = false;
  }

  DoorManipulatorGate::~DoorManipulatorGate()
  {
    this->serial_helper_->close_serial();
    delete this->serial_helper_;
  }

  void DoorManipulatorGate::control_norelem_stepper_topic_callback(const NorelemStepperControl& msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard from control_norelem_stepper topic the drawer_controller_id: '%i'",
                msg.drawer_controller_id);   // Debugging
    RCLCPP_INFO(this->get_logger(), "I heard from control_norelem_stepper topic the msg.motor_id: '%i'",
                msg.motor_id);   // Debugging

    uint32_t drawer_controller_id = msg.drawer_controller_id;

    norelem_stepper_parameter norelem_stepper_parameter = {};
    norelem_stepper_parameter.motor_id = msg.motor_id;
    norelem_stepper_parameter.motor_e1 = msg.motor_e1;
    norelem_stepper_parameter.motor_e2 = msg.motor_e2;
    norelem_stepper_parameter.motor_e3 = msg.motor_e3;
    norelem_stepper_parameter.motor_e4 = msg.motor_e4;

    robast_can_msgs::CanMessage can_msg_door_manipulator =
        this->create_can_msg_norelem_stepper(drawer_controller_id, norelem_stepper_parameter);

    this->send_can_msg(can_msg_door_manipulator);
  }

  robast_can_msgs::CanMessage DoorManipulatorGate::create_can_msg_norelem_stepper(
      uint32_t drawer_controller_id, norelem_stepper_parameter norelem_stepper_parameter) const
  {
    robast_can_msgs::CanMessage can_msg_door_manipulator = this->can_db_.can_messages.at(CAN_MSG_DOOR_MANIPULATOR);

    std::vector can_signals_door_manipulator = can_msg_door_manipulator.get_can_signals();

    can_signals_door_manipulator.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).set_data(drawer_controller_id);
    can_signals_door_manipulator.at(CAN_SIGNAL_MOTOR_ID).set_data(norelem_stepper_parameter.motor_id);
    can_signals_door_manipulator.at(CAN_SIGNAL_MOTOR_E1).set_data(norelem_stepper_parameter.motor_e1);
    can_signals_door_manipulator.at(CAN_SIGNAL_MOTOR_E2).set_data(norelem_stepper_parameter.motor_e2);
    can_signals_door_manipulator.at(CAN_SIGNAL_MOTOR_E3).set_data(norelem_stepper_parameter.motor_e3);
    can_signals_door_manipulator.at(CAN_SIGNAL_MOTOR_E4).set_data(norelem_stepper_parameter.motor_e4);

    can_msg_door_manipulator.set_can_signals(can_signals_door_manipulator);

    return can_msg_door_manipulator;
  }

  void DoorManipulatorGate::send_can_msg(robast_can_msgs::CanMessage can_message)
  {
    if (!this->serial_can_usb_converter_is_set_up_)
    {
      this->setup_serial_can_ubs_converter();   // mind that this should actually be done in the constructor, but due to
                                                // testability we need to execute this here
    }

    std::optional<std::string> ascii_cmd =
        robast_can_msgs::encode_can_message_into_ascii_command(can_message, this->can_db_.can_messages);

    if (ascii_cmd.has_value())
    {
      this->add_ascii_cmd_to_queue(ascii_cmd.value());
    }
    else
    {
      // TODO: Error handling
    }
  }

  // This timer callback makes sure there is enough time between each ascii command, otherwise some commands might get
  // lost
  void DoorManipulatorGate::send_ascii_cmds_timer_callback(void)
  {
    if (this->ascii_cmd_queue_.empty())
    {
      this->send_ascii_cmds_timer_->cancel();   // cancel the timer when queue is empty
      return;
    }
    else
    {
      std::string send_ascii_cmd_result = this->serial_helper_->send_ascii_cmd(this->ascii_cmd_queue_.front());
      if (send_ascii_cmd_result.size() > 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Error sending serial ascii cmd: %s", send_ascii_cmd_result.c_str());
      }
      this->ascii_cmd_queue_.pop();   // remove first element of the queue
    }
  }

  void DoorManipulatorGate::setup_serial_can_ubs_converter(void)
  {
    std::string setup_serial_port_result = this->serial_helper_->open_serial();
    if (setup_serial_port_result.size() > 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from opening serial Port: %s", setup_serial_port_result.c_str());
    }
    this->close_can_channel();
    this->set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps);
    this->open_can_channel();   // the default state should be the open can channel to enable receiving CAN messages

    this->serial_can_usb_converter_is_set_up_ = true;
  }

  void DoorManipulatorGate::add_ascii_cmd_to_queue(std::string ascii_cmd)
  {
    // if queue was empty, the timer has been canceled before, so start timer for timer callbacks which trigger sending
    // the ascii cmds
    if (this->ascii_cmd_queue_.empty())
    {
      this->ascii_cmd_queue_.push(ascii_cmd);
      this->send_ascii_cmds_timer_ = this->create_wall_timer(
          TIMER_PERIOD_SEND_ASCII_CMDS, std::bind(&DoorManipulatorGate::send_ascii_cmds_timer_callback, this));
    }
    else
    {
      this->ascii_cmd_queue_.push(ascii_cmd);
    }
  }

  void DoorManipulatorGate::close_can_channel(void)
  {
    this->add_ascii_cmd_to_queue("C");
  }

  void DoorManipulatorGate::open_can_channel(void)
  {
    this->add_ascii_cmd_to_queue("O");
  }

  void DoorManipulatorGate::set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface can_baudrate)
  {
    std::string msg;

    switch (can_baudrate)
    {
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_10kbps:
        msg = "S0";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_20kbps:
        msg = "S1";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_50kbps:
        msg = "S2";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_100kbps:
        msg = "S3";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_125kbps:
        msg = "S4";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps:
        msg = "S5";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_500kbps:
        msg = "S6";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_800kbps:
        msg = "S7";
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_1000kbps:
        msg = "S8";
        break;
      default:
        msg = "S5";
        break;
    }

    this->add_ascii_cmd_to_queue(msg);
  }

}   // namespace door_manipulator_gate
