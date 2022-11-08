#include "test/test_nfc_gate.hpp"

namespace robast
{
    TestNFCGate::TestNFCGate(serial_helper::ISerialHelper* serial_connector) :NFCGate()
    {
        NFCGate::serial_connector_ = serial_connector;
    }

    TestNFCGate::TestNFCGate() :NFCGate()
    {
        declare_parameter("key", "");
        string key = get_parameter("key").as_string();
        NFCGate::serial_connector_ = new serial_helper::MockSerialHelper(serial_helper::MockSerialHelper(key));
    }

    string  TestNFCGate::execute_scan(std::vector<std::string> permission_keys, bool* found)
    {
        return  NFCGate::execute_scan(permission_keys, found);
    }

    string  TestNFCGate::validate_key(string scanned_key, std::vector<std::string> allValidKeys, bool* found)
    {
        return  NFCGate::validate_key(scanned_key, allValidKeys, found);
    }

    string  TestNFCGate::scan_tag(bool* found)
    {
        return  NFCGate::scan_tag(found);
    }
}
