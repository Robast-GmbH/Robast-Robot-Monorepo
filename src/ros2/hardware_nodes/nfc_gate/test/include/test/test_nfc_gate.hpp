
#include "nfc_gate/nfc_gate.hpp"
#include "mock_serial_helper.hpp"

using namespace std;
namespace robast
{
    class TestNFCGate : public NFCGate
    {
    public:
        TestNFCGate(serial_helper::ISerialHelper* serial_connector);
        TestNFCGate();
    };
}