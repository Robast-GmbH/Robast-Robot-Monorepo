#include "drawer_gate/drawer_gate.hpp"
#include "mock_serial_helper.hpp"

using namespace std;
namespace drawer_gate
{
    class TestDrawerGate : public DrawerGate
    {
    public:
        TestDrawerGate(serial_helper::ISerialHelper* serial_helper);
        TestDrawerGate();
    };
}