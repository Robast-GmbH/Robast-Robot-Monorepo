#ifndef HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_
#define HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL              10
#define CHECK_ON_DB                  false

#include <string.h>
#include <memory>
#include <thread>

#include "communication_interfaces/action/create_user_nfc_tag.hpp"
#include "db_helper/postgresql_connector.hpp"
#include "nfc_bridge/elatec_api.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "serial_helper/serial_helper.h"
#include "std_msgs/msg/string.hpp"

namespace nfc_bridge
{
  class NFCBridge : public rclcpp::Node
  {
   public:
    using CreateUser = communication_interfaces::action::CreateUserNfcTag;
    using GoalHandleCreateUser = rclcpp_action::ServerGoalHandle<CreateUser>;

    NFCBridge(std::string serial_port_path = "/dev/robast/robast_nfc");
    ~NFCBridge();

    friend class TestNFCBridge;   // this class has full access to all private and protected parts of this class

   private:
    rclcpp_action::Server<CreateUser>::SharedPtr action_server_;
    int numReadings_;

    serial_helper::ISerialHelper* serial_connector_;
    db_helper::IDBHelper* db_connector_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp_action::Server<CreateUser>::SharedPtr create_user_server_;
    const std::map<std::string, std::string> nfc_code_to_drawer_ =
        std::map<std::string, std::string>{{"000100000000000000000000000000000001", "1"},
                                           {"000100000000000000000000000000000100", "2"},
                                           {"000100000000000000000000000000010000", "3"},
                                           {"000100000000000000000000000001000000", "4"},
                                           {"000100000000000000000000000100000000", "5"}};
    bool execute_scan(std::shared_ptr<std::string> received_raw_data);
    bool scan_tag(std::shared_ptr<std::string> tag_data);

    // void write_tag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response>
    // response);
    void start_up_scanner();
    void reader_procedure();
    void turn_off_scanner();
    void prepare_scanning();

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const CreateUser::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
    void createUser(const std::shared_ptr<GoalHandleCreateUser> goal_handle);
    bool write_tag(int card_data);
  };

}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_