#ifndef HARDWARE_NODES__TEST_DRAWER_BRIDGE_HPP_
#define HARDWARE_NODES__TEST_DRAWER_BRIDGE_HPP_

#include "drawer_bridge/drawer_bridge.hpp"

using namespace std;
namespace drawer_bridge
{
    class TestDrawerBridge: public DrawerBridge
    {
    public:
        TestDrawerBridge();
    };
}

#endif //HARDWARE_NODES__TEST_DRAWER_BRIDGE_HPP_