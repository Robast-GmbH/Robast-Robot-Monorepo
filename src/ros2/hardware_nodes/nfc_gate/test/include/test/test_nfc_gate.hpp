#include "nfc_gate/nfc_gate.hpp"
#include "mock_serial_helper.hpp"
#include "mock_postgresql_helper.hpp"

using namespace std;
namespace robast
{
    class TestNFCGate : public NFCGate
    {
    public:
        TestNFCGate(serial_helper::ISerialHelper* serial_connector, db_helper::IDBHelper* db_connector);
        TestNFCGate();

        bool execute_scan(std::vector<std::string> permission_keys, std::string* validated_User );
        bool validate_key(string scanned_key, std::vector<std::string> allValidKeys, std::string* validated_user );
        bool scan_tag(std::string* tag_data);
    };
}