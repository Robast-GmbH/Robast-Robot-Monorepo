#pragma once

#include <smacc2/smacc.hpp>

namespace sm_drawer_gate
{
// STATE DECLARATION
struct StOpenDrawer : smacc2::SmaccState<StOpenDrawer, MsDrawerControlRunMode>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct ReceivedMsg : SUCCESS{}; //TODO: This should actually be named transition?

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbReceivedMsg<CbOpenDrawerSubscriber, OrDrawerControl>, StLoopDrawerAccess, ReceivedMsg>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrDrawerControl, CbOpenDrawerSubscriber>();
  }

  void runtimeConfigure() {}

  void onEntry() 
  {
    cl_drawer_control::ClDrawerControl * drawerclient;
    this->requiresClient(drawerclient);
    RCLCPP_INFO(getLogger(), "On Entry-------------------------On reading!");
    auto data = drawerclient->getComponent<CpDrawerAddress>()->getLastMsg();
    RCLCPP_INFO(getLogger(), "On Entry-------------------------On writing!%i", data.drawer_controller_id);
    // data->setData(msging);
    RCLCPP_INFO(getLogger(), "On Entry!");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }

};
}  // namespace sm_drawer_gate
