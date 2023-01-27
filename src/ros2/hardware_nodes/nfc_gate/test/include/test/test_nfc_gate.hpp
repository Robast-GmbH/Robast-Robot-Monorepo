#include "nfc_gate/nfc_gate.hpp"
#include "mock_serial_helper.hpp"
#include "mock_postgresql_helper.hpp"

namespace robast
{
    class TestNFCGate : public NFCGate
    {
    public:
        TestNFCGate(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector);
        TestNFCGate();

        std::string execute_scan(std::vector<std::string> permission_keys, std::unique_ptr<bool> found);
        std::string validate_key(std::string scanned_key, std::vector<std::string> allValidKeys, std::unique_ptr<bool> found);
        std::string scan_tag(std::unique_ptr<bool> found);
    };
}