#include "test/test_nfc_bridge.hpp"

namespace nfc_bridge
{
  TestNFCBridge::TestNFCBridge(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector)
      : NFCBridge()
  {
    NFCBridge::serial_connector_ = serial_connector;
    NFCBridge::db_connector_ = db_connector;
  }

  TestNFCBridge::TestNFCBridge() : NFCBridge()
  {
    declare_parameter("key", "000001000000100");
    declare_parameter("User1_name", "1");
    declare_parameter("User1_key", "000001000000100");
    declare_parameter("User2_name", "");
    declare_parameter("User2_key", "");
    declare_parameter("User3_name", "");
    declare_parameter("User3_key", "");

    std::string key = get_parameter("key").as_string();

    std::map<std::string, std::string> valid_user_list;
    valid_user_list.insert(std::pair<std::string, std::string>(get_parameter("User1_key").as_string(),
                                                               get_parameter("User1_name").as_string()));
    valid_user_list.insert(std::pair<std::string, std::string>(get_parameter("User2_key").as_string(),
                                                               get_parameter("User2_name").as_string()));
    valid_user_list.insert(std::pair<std::string, std::string>(get_parameter("User3_key").as_string(),
                                                               get_parameter("User3_name").as_string()));                                                           

    NFCBridge::serial_connector_ = new serial_helper::MockSerialHelper(serial_helper::MockSerialHelper(key));

    NFCBridge::db_connector_ = new db_helper::MockPostgreSqlHelper(valid_user_list);
  }
}   // namespace nfc_bridge