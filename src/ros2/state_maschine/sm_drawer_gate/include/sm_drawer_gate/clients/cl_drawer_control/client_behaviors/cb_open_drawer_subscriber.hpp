#pragma once
#include <smacc2/client_behaviors/cb_subscription_callback_base.hpp>
#include <sm_drawer_gate/clients/cl_drawer_control/cl_drawer_control.hpp>

#include "communication_interfaces/msg/drawer_address.hpp"

namespace sm_drawer_gate
{
namespace cl_drawer_control
{
template <typename TSource, typename TOrthogonal>
struct EvCbReceivedMsg : sc::event<EvCbReceivedMsg<TSource, TOrthogonal>>
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

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postMyEvent_ = [=] { this->postEvent<EvCbReceivedMsg<TSourceObject, TOrthogonal>>(); };
    RCLCPP_INFO(getLogger(), "onOrthogonalAllocation !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"); //DEBUGGING
  }

  void onMessageReceived(const communication_interfaces::msg::DrawerAddress & msg) override
  {
    last_msg = msg;
    // drawer_controller_id = msg.drawer_controller_id;
    this->drawer_controller_id = 7;

    //TODO: drawer address in class varible schreiben
    RCLCPP_INFO(getLogger(), "Received Drawer Address: %i", msg.drawer_controller_id);
    this->postMyEvent_();
  }

  int getLastMsg()
  {
    return 1;
  }

  std::function<void()> postMyEvent_;

  uint8_t drawer_controller_id;

protected:
  communication_interfaces::msg::DrawerAddress last_msg;
  
};
}  // namespace cl_drawer_control
}  // namespace sm_drawer_gate
