#include <smacc2/smacc.hpp>
namespace sm_drawer_gate
{
// STATE DECLARATION
class MsRun : public smacc2::SmaccState<MsRun, SmDrawerGate, StLoopDrawerAccess>
{
public:
  using SmaccState::SmaccState;
};
}  // namespace sm_drawer_gate
