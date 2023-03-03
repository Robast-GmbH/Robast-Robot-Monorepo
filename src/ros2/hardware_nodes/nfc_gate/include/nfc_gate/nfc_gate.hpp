#ifndef HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_
#define HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_

#define STANDART_REPLAY_MESSAGE_SIZE 500
#define READER_INTEVALL              10
#define CHECK_ON_DB                  false

#include <string.h>

#include "communication_interfaces/srv/create_user_nfc_tag.hpp"
#include "db_helper/postgresql_connector.hpp"
#include "nfc_gate/elatec_api.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "serial_helper/serial_helper.h"
#include "std_msgs/msg/string.hpp"

namespace nfc_gate
{
  class NFCGate : public rclcpp::Node
  {
   public:
    using CreateUser = communication_interfaces::srv::CreateUserNfcTag;

    NFCGate(std::string serial_port_path = "/dev/robast/robast_nfc");
    ~NFCGate();

    friend class TestNFCGate;   // this class has full access to all private and protected parts of this class

   private:
    int numReadings_;

    serial_helper::ISerialHelper* serial_connector_;
    db_helper::IDBHelper* db_connector_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<CreateUser>::SharedPtr create_user_server_;
    const std::map<std::string, std::string> nfc_code_to_drawer_= std::map<std::string,std::string>{
                                                   {"000100000000000000000000000000000001","1"},
                                                   {"000100000000000000000000000000000100","2"},
                                                   {"000100000000000000000000000000010000","3"},
                                                   {"000100000000000000000000000001000000","4"},
                                                   {"000100000000000000000000000100000000","5"} }; 
    bool execute_scan(std::shared_ptr<std::string> received_raw_data);
    bool scan_tag(std::shared_ptr<std::string> tag_data);

    void write_tag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response);
    void start_up_scanner();
    void reader_procedure();
    void turn_off_scanner();
    bool checkUserTag(std::string scanned_key, std::shared_ptr<std::string> related_username);
    void prepare_scanning();
    
  };

}   // namespace robast
#endif   // HARDWARE_NODES__NFC_GATE__NFC_GATE_HPP_