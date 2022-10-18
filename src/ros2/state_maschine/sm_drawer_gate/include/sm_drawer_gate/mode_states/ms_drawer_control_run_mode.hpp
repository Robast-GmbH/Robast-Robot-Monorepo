#include <smacc2/smacc.hpp>
namespace sm_drawer_gate
{
// STATE DECLARATION
class MsDrawerGateRunMode : public smacc2::SmaccState<MsDrawerGateRunMode, SmDrawerGate, StLoopDrawerAccess>
{
public:
  using SmaccState::SmaccState;
};
}  // namespace sm_drawer_gate
