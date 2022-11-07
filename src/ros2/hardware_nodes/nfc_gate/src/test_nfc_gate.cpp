#include "nfc_gate/test_nfc_gate.hpp"

namespace robast
{
    TestNFCGate::TestNFCGate(serial_helper::ISerialHelper *serial_connector):NFCGate("")
    {
     
    }
}

 //  declare_parameter("debug", false);
    //  declare_parameter("key", "");
    //  debug = get_parameter("debug").as_bool();
   
    //   if(debug )
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Debug mode: on");
    //     string key =get_parameter("key").as_string();
    //     this->mock_connector(key);
    //   }else 
    //   {