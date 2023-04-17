#ifndef HARDWARE_NODES__TEST_NFC_BRIDGE_HPP_
#define HARDWARE_NODES__TEST_NFC_BRIDGE_HPP_

#include "nfc_bridge/nfc_bridge.hpp"
#include "mock_serial_helper.hpp"
#include "mock_postgresql_helper.hpp"


namespace nfc_bridge
{
    class TestNFCBridge : public NFCBridge
    {
    public:
        TestNFCBridge(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector);
        TestNFCBridge();

        bool execute_scan(std::vector<std::string> permission_keys, std::shared_ptr< std::string>validated_User );
        bool validate_key(std::string scanned_key, std::vector<std::string> allValidKeys, std::shared_ptr<std::string> validated_user );
        bool scan_tag(std::shared_ptr<std::string> tag_data);
    };
}
#endif //HARDWARE_NODES__TEST_NFC_BRIDGE_HPP_