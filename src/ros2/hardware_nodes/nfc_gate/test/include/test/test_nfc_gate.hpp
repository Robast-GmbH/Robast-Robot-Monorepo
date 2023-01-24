#include "nfc_gate/nfc_gate.hpp"
#include "mock_serial_helper.hpp"
#include "mock_postgresql_helper.hpp"

using namespace std;
namespace robast
{
    class TestNFCGate : public NFCGate
    {
    public:
        TestNFCGate(serial_helper::ISerialHelper* serial_connector, db::IDBHelper* db_connector);
        TestNFCGate();

        string execute_scan(std::vector<std::string> permission_keys, bool* found);
        string validate_key(string scanned_key, std::vector<std::string> allValidKeys, bool* found);
        string scan_tag(bool* found);
    };
}