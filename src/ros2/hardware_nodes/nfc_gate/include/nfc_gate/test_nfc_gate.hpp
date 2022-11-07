
#include "nfc_gate/nfc_gate.hpp"

using namespace std;
namespace robast
{ 
    class TestNFCGate : public NFCGate
    {
        public: 
        TestNFCGate(serial_helper::ISerialHelper *serial_connector);

        private:
        string keycode;
    };
}