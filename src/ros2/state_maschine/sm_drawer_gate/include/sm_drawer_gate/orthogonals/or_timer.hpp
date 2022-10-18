#pragma once

#include <ros_timer_client/cl_ros_timer.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace sm_drawer_gate
{
using namespace std::chrono_literals;
class OrTimer : public smacc2::Orthogonal<OrTimer>
{
public:
  void onInitialize() override
  {
    auto actionclient = this->createClient<cl_ros_timer::ClRosTimer>(5000ms);
  }
};
}  // namespace sm_drawer_gate
