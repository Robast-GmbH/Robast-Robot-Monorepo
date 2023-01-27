#include "test/test_nfc_gate.hpp"

namespace robast
{
    TestNFCGate::TestNFCGate(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector ):NFCGate()
    {
        NFCGate::serial_connector_ = serial_connector;
        NFCGate::db_conncetor_ = db_connector;
    }

    TestNFCGate::TestNFCGate() :NFCGate()
    {
        declare_parameter("key", "");
        declare_parameter("User1_name", "");
        declare_parameter("User1_key", "");
        declare_parameter("User2_name", "");
        declare_parameter("User2_key", "");
        declare_parameter("User3_name", "");
        declare_parameter("User3_key", "");
            
        std::string key = get_parameter("key").as_string();
        
        std::map< std::string, std::string> validUserList;
        validUserList.insert(std::pair<std::string, std::string>(get_parameter("User1_name").as_string(), get_parameter("User1_key").as_string()));
        validUserList.insert(std::pair<std::string, std::string>(get_parameter("User2_name").as_string(), get_parameter("User2_key").as_string()));
        validUserList.insert(std::pair<std::string, std::string>(get_parameter("User3_name").as_string(), get_parameter("User3_key").as_string()));
        
        NFCGate::serial_connector_ = new serial_helper::MockSerialHelper(key);
        NFCGate::db_conncetor_ = new db_helper::MockPostgreSqlHelper(validUserList);
    }

    std::string TestNFCGate::execute_scan(std::vector<std::string> permission_keys, std::unique_ptr<bool>found)
    {
        return  NFCGate::execute_scan(permission_keys, std::move(found));
    }

    std::string TestNFCGate::validate_key(std::string scanned_key, std::vector<std::string> allValidKeys, std::unique_ptr<bool>found)
    {
        return  NFCGate::validate_key(scanned_key, allValidKeys, std::move(found));
    }

    std::string TestNFCGate::scan_tag(std::unique_ptr<bool> found)
    {
        return  NFCGate::scan_tag(std::move(found));
    }
}