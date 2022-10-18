#pragma once

#include <smacc2/smacc.hpp>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.hpp>
#include <sm_drawer_gate/clients/cl_drawer_control/cl_drawer_control.hpp>

// ORTHOGONALS
#include <sm_drawer_gate/orthogonals/or_timer.hpp>
#include <sm_drawer_gate/orthogonals/or_drawer_control.hpp>

using namespace cl_ros_timer;
using namespace sm_drawer_gate::cl_drawer_control;

// CLIENT BEHAVIORS
#include <sm_drawer_gate/clients/cl_drawer_control/client_behaviors/cb_open_drawer_subscriber.hpp>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>


using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;

namespace sm_drawer_gate
{
// SUPERSTATES
// namespace SSDrawerAccess
// {
// class SsDrawerAccess;
// }  // namespace SSDrawerAccess


// STATES
class StLoopDrawerAccess;  // first state specially needs a forward declaration

class MsDrawerControlRunMode;
// class MsRecover;

// struct EvToDeep : sc::event<EvToDeep>
// {
// };

// struct EvFail : sc::event<EvFail>
// {
// };

// STATE MACHINE
struct SmDrawerGate : public smacc2::SmaccStateMachineBase<SmDrawerGate, MsDrawerControlRunMode>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrDrawerControl>();
  }
};
}  // namespace sm_three_some

// MODE STATES
#include <sm_drawer_gate/mode_states/ms_drawer_control_run_mode.hpp>

// #include <sm_drawer_gate/mode_states/ms_recover.hpp>

// STATES
#include <sm_drawer_gate/states/st_loop_drawer_access.hpp>
// #include <sm_drawer_gate/superstates/ss_drawer_access.hpp>
