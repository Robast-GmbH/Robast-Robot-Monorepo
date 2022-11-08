#include "test/test_drawer_gate.hpp"

namespace drawer_gate
{
    TestDrawerGate::TestDrawerGate(serial_helper::ISerialHelper* serial_helper) : DrawerGate()
    {
        DrawerGate::serial_helper_ = serial_helper;
    }

    TestDrawerGate::TestDrawerGate() : DrawerGate()
    {
        DrawerGate::serial_helper_ = new serial_helper::MockSerialHelper();
    }
}
