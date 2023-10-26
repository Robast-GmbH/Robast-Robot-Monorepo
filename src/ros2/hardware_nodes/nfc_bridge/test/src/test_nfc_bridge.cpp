#include "test/test_nfc_bridge.hpp"

namespace nfc_bridge
{
  TestNFCBridge::TestNFCBridge(serial_helper::MockSerialHelper* serial_connector, db_helper::MockPostgreSqlHelper* db_connector)
      : NFCBridge()
  {
    NFCBridge::serial_connector_ = std::make_unique<serial_helper::MockSerialHelper>( *serial_connector);
    NFCBridge::db_connector_ = std::make_unique<db_helper::MockPostgreSqlHelper>( *db_connector);
  }

  TestNFCBridge::TestNFCBridge() : NFCBridge()
  {
    declare_parameter("key", "000001000000100");
    declare_parameter("User1_name","1");
    declare_parameter("User1_id", 1);
    declare_parameter("User1_key", "000001000000100");
    declare_parameter("User2_name", "2");
    declare_parameter("User2_id", 2);
    declare_parameter("User2_key", "");
    declare_parameter("User3_name", "");
    declare_parameter("User3_id", 3);
    declare_parameter("User3_key", "");

    std::string key = get_parameter("key").as_string();

    std::map<std::string, std::pair<int, std::string>> valid_user_list;
    valid_user_list.insert(std::pair<std::string, std::pair<int, std::string>>(
        get_parameter("User1_key").as_string(),
        make_pair(get_parameter("User1_id").as_int(), get_parameter("User1_name").as_string())));
    valid_user_list.insert(std::pair<std::string, std::pair<int, std::string>>(
        get_parameter("User2_key").as_string(),
        make_pair(get_parameter("User2_id").as_int(), get_parameter("User2_name").as_string())));
    valid_user_list.insert(std::pair<std::string, std::pair<int, std::string>>(
        get_parameter("User3_key").as_string(),
        make_pair(get_parameter("User3_id").as_int(), get_parameter("User3_name").as_string())));

    NFCBridge::serial_connector_ = std::make_unique<serial_helper::MockSerialHelper>(
        serial_helper::MockSerialHelper(serial_helper::MockSerialHelper(key)));

    NFCBridge::db_connector_ =
        std::make_unique<db_helper::MockPostgreSqlHelper>(db_helper::MockPostgreSqlHelper(valid_user_list));
  }

  bool TestNFCBridge::lookup_user_tag(std::string scanned_key,
                                      std::shared_ptr<std::string> related_username,
                                      std::shared_ptr<int> related_id)
  {
    return db_connector_->checkUserTag(
        scanned_key, related_username, related_id, std::shared_ptr<std::string>());
  }

  bool TestNFCBridge::read_nfc_code(std::shared_ptr<std::string> scanned_key)
  {
    return serial_connector_->read_serial(scanned_key.get(), 50);
  }

}   // namespace nfc_bridge