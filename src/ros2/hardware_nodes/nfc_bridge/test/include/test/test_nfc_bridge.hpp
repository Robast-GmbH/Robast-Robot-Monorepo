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
    TestNFCBridge(serial_helper::MockSerialHelper* serial_connector, db_helper::MockPostgreSqlHelper* db_connector);
    TestNFCBridge();


    bool read_nfc_code(std::shared_ptr<std::string> scanned_key);
    bool lookup_user_tag(std::string scanned_key,
                         std::shared_ptr<std::string> related_username,
                         std::shared_ptr<int> related_id);
  };
}   // namespace nfc_bridge
#endif   // HARDWARE_NODES__TEST_NFC_BRIDGE_HPP_