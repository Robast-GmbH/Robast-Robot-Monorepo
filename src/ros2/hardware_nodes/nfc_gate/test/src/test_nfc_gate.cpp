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

}
