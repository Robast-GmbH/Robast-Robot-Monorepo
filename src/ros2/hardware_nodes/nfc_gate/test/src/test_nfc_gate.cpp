#include "test/test_nfc_gate.hpp"

namespace robast
{
    TestNFCGate::TestNFCGate(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector ):NFCGate()
    {
        NFCGate::serial_connector_ = serial_connector;
        NFCGate::db_connector_ = db_connector;
    }

    TestNFCGate::TestNFCGate() :NFCGate()
    {
        declare_parameter("key", "Test1");
        declare_parameter("User1_name", "1");
        declare_parameter("User1_key", "Test1");
        declare_parameter("User2_name", "");
        declare_parameter("User2_key", "");
        declare_parameter("User3_name", "");
        declare_parameter("User3_key", "");
            
        std::string key = get_parameter("key").as_string();
        
        std::map< std::string, std::string> validUserList;
        validUserList.insert(std::pair<std::string, std::string>( get_parameter("User1_key").as_string(), get_parameter("User1_name").as_string()));
        validUserList.insert(std::pair<std::string, std::string>(get_parameter("User2_key").as_string(), get_parameter("User2_name").as_string()));
        validUserList.insert(std::pair<std::string, std::string>( get_parameter("User3_key").as_string(), get_parameter("User3_name").as_string()));

        NFCGate::serial_connector_ = new serial_helper::MockSerialHelper(serial_helper::MockSerialHelper(key));
      
        NFCGate::db_connector_ = new db_helper::MockPostgreSqlHelper(validUserList);
    }
}