#pragma once
#include <smacc2/client_behaviors/cb_subscription_callback_base.hpp>
#include <sm_drawer_gate/clients/cl_drawer_control/cl_drawer_control.hpp>

#include "communication_interfaces/msg/drawer_address.hpp"

namespace sm_drawer_gate
{
namespace cl_drawer_control
{
struct EvCbReceivedMsg : sc::event<EvCbReceivedMsg>
{
};

//--------------------------------------------------------------------------------------
class CbOpenDrawerSubscriber
: public smacc2::client_behaviors::CbSubscriptionCallbackBase<communication_interfaces::msg::DrawerAddress>
{
public:
  CbOpenDrawerSubscriber() {}
  void onEntry() override
  {
    RCLCPP_INFO(getLogger(), "[CbOpenDrawerSubscriber] onEntry");
    smacc2::client_behaviors::CbSubscriptionCallbackBase<communication_interfaces::msg::DrawerAddress>::onEntry();
  }

  void onMessageReceived(const communication_interfaces::msg::DrawerAddress & msg) override
  {
    //TODO: drawer address in class varible schreiben
    auto ev = new EvCbReceivedMsg();
    this->postEvent(ev);
  }
};
}  // namespace cl_drawer_control
}  // namespace sm_drawer_gate
