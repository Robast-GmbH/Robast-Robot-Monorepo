#pragma once

#include <smacc2/smacc.hpp>

namespace sm_drawer_gate
{
// STATE DECLARATION
struct StLoopDrawerAccess : smacc2::SmaccState<StLoopDrawerAccess, MsDrawerControlRunMode>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct ReceivedMsg : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbReceivedMsg<CbOpenDrawerSubscriber, OrDrawerControl>, StOpenDrawer, ReceivedMsg>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrDrawerControl, CbOpenDrawerSubscriber>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit()
  {
    cl_drawer_control::ClDrawerControl * drawerclient;
    this->requiresClient(drawerclient);
    RCLCPP_INFO(getLogger(), "-------------------------On reading!");
    auto data = drawerclient->getComponent<CpDrawerAddress>()->getLastMsg();
    auto old_msg = this->getOrthogonal<OrDrawerControl>()->getClientBehavior<CbOpenDrawerSubscriber>()->getLastMsg();
    RCLCPP_INFO(getLogger(), "-------------------------On writing!%i", data.drawer_controller_id);
    // data->setData(msging);
    RCLCPP_INFO(getLogger(), "On Exit!");
  }
};
}  // namespace sm_drawer_gate
