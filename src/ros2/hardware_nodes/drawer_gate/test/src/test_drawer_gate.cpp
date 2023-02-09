#include "test/test_drawer_gate.hpp"

#include <iostream>

namespace drawer_gate
{
  TestDrawerGate::TestDrawerGate(serial_helper::ISerialHelper* serial_helper) : DrawerGate()
  {
    DrawerGate::serial_helper_ = serial_helper;
    DrawerGate::serial_can_usb_converter_is_set_up_ = true;
  }

  TestDrawerGate::TestDrawerGate() : DrawerGate()
  {
    DrawerGate::serial_helper_ = new serial_helper::MockSerialHelper();
    DrawerGate::serial_can_usb_converter_is_set_up_ = true;
  }
}   // namespace drawer_gate