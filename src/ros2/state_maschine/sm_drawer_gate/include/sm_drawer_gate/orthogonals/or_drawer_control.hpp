#pragma once

#include <smacc2/smacc_orthogonal.hpp>
#include <sm_drawer_gate/clients/cl_drawer_control/cl_drawer_control.hpp>

using namespace std::chrono_literals;

namespace sm_drawer_gate
{
class OrDrawerControl : public smacc2::Orthogonal<OrDrawerControl>
{
public:
  void onInitialize() override
  {
    auto clDrawerControl = this->createClient<cl_drawer_control::ClDrawerControl>("/open_drawer");
  }
};
}  // namespace sm_drawer_gate