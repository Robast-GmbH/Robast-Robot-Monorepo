#include <smacc2/smacc.hpp>
namespace sm_drawer_gate
{
// STATE DECLARATION
class MsDrawerControlRunMode : public smacc2::SmaccState<MsDrawerControlRunMode, SmDrawerGate, StLoopDrawerAccess>
{
public:
  using SmaccState::SmaccState;
};
}  // namespace sm_drawer_gate
