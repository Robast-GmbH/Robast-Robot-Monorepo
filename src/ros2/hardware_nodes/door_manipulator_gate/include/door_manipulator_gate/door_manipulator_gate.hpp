#ifndef DOOR_MANIPULATOR_GATE__DOOR_MANIPULATOR_GATE_HPP_
#define DOOR_MANIPULATOR_GATE__DOOR_MANIPULATOR_GATE_HPP_

#include <queue>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

// Linux headers for serial communication
#include <errno.h>     // Error integer and strerror() function
#include <fcntl.h>     // Contains file controls like O_RDWR
#include <termios.h>   // Contains POSIX terminal control definitions
#include <unistd.h>    // write(), read(), close()

#include "can/can_db.hpp"
#include "can/can_helper.h"
#include "communication_interfaces/msg/norelem_stepper_control.hpp"
#include "serial_helper/serial_helper.h"

// Some tests show that a timer period of 3 ms is to fast and asci cmds get lost at this speed, with 4 ms it worked, so
// use 5ms with safety margin
#define TIMER_PERIOD_SEND_ASCII_CMDS std::chrono::milliseconds(5)

namespace door_manipulator_gate
{
  struct norelem_stepper_parameter
  {
    uint8_t motor_id;
    bool motor_e1;
    bool motor_e2;
    bool motor_e3;
    bool motor_e4;
  };

  class DoorManipulatorGate : public rclcpp::Node
  {
   public:
    using NorelemStepperControl = communication_interfaces::msg::NorelemStepperControl;

    /**
     * @brief A constructor for door_manipulator_gate::DoorManipulatorGate class
     */
    DoorManipulatorGate(const std::string serial_path = "/dev/robast/robast_can");
    /**
     * @brief A destructor for door_manipulator_gate::DoorManipulatorGate class
     */
    ~DoorManipulatorGate();

   private:
    rclcpp::Subscription<NorelemStepperControl>::SharedPtr norelem_stepper_control_subscription_;

    rclcpp::TimerBase::SharedPtr send_ascii_cmds_timer_;

    serial_helper::ISerialHelper* serial_helper_;

    robast_can_msgs::CanDb can_db_ = robast_can_msgs::CanDb();

    std::queue<std::string> ascii_cmd_queue_;   // queue that contains all ascii commands to be sent to the usb serial
                                                // can adapter to make sure there is enough time between each ascii
                                                // command, otherwise some commands might get lost

    bool cleared_serial_buffer_from_old_can_msgs_;   // flag, that is responsible for clearing the serial buffer from
                                                     // old CAN messages

    bool serial_can_usb_converter_is_set_up_;   // flag to prevent that setup routine for serial can usb converter is
                                                // executed more than once and we need this for testability

    void control_norelem_stepper_topic_callback(const NorelemStepperControl& msg);

    robast_can_msgs::CanMessage create_can_msg_norelem_stepper(
        uint32_t drawer_controller_id, norelem_stepper_parameter norelem_stepper_parameter) const;

    void add_ascii_cmd_to_queue(std::string ascii_cmd);

    void send_ascii_cmds_timer_callback(void);

    void setup_serial_can_ubs_converter(void);

    void set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface can_baudrate);

    void open_can_channel(void);

    void close_can_channel(void);

    void send_can_msg(robast_can_msgs::CanMessage can_message);
  };

}   // namespace door_manipulator_gate

#endif   // DOOR_MANIPULATOR_GATE__DOOR_MANIPULATOR_GATE_HPP_