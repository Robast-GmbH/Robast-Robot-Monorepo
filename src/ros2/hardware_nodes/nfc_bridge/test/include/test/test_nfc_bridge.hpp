#ifndef HARDWARE_NODES__TEST_NFC_BRIDGE_HPP_
#define HARDWARE_NODES__TEST_NFC_BRIDGE_HPP_

#include "mock_postgresql_helper.hpp"
#include "mock_serial_helper.hpp"
#include "nfc_bridge/nfc_bridge.hpp"

namespace nfc_bridge
{
  class TestNFCBridge : public NFCBridge
  {
   public:
    TestNFCBridge(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector);
    TestNFCBridge();

    bool execute_scan(std::shared_ptr<std::string> received_raw_data);
    bool scan_tag(std::shared_ptr<std::string> tag_data);
    bool lookup_user_tag(std::string scanned_key,
                         std::shared_ptr<std::string> related_username,
                         std::shared_ptr<int> related_id);
  };
}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__TEST_NFC_BRIDGE_HPP_