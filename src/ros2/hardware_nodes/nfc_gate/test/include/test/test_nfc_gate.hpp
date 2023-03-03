#ifndef HARDWARE_NODES__TEST_NFC_GATE_HPP_
#define HARDWARE_NODES__TEST_NFC_GATE_HPP_

#include "nfc_gate/nfc_gate.hpp"
#include "mock_serial_helper.hpp"
#include "mock_postgresql_helper.hpp"


namespace nfc_gate
{
    class TestNFCGate : public NFCGate
    {
    public:
        TestNFCGate(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector);
        TestNFCGate();

        bool execute_scan(std::vector<std::string> permission_keys, std::shared_ptr< std::string>validated_User );
        bool validate_key(std::string scanned_key, std::vector<std::string> allValidKeys, std::shared_ptr<std::string> validated_user );
        bool scan_tag(std::shared_ptr<std::string> tag_data);
    };
}
#endif //HARDWARE_NODES__TEST_NFC_GATE_HPP_