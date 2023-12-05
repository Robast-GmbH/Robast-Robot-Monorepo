#ifndef DRYVE_D1_BRIDGE__DRYVE_D1_BRIDGE_HPP_
#define DRYVE_D1_BRIDGE__DRYVE_D1_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "d1.hpp"
#include "dryve_d1_bridge/socket_wrapper.hpp"

namespace dryve_d1_bridge
{
  class DryveD1Gate : public rclcpp::Node
  {
   public:
    /**
     * @brief A constructor for dryve_d1_bridge::DryveD1Gate class
     */
    DryveD1Gate();
    /**
     * @brief A destructor for dryve_d1_bridge::DryveD1Gate class
     */
    ~DryveD1Gate();

   private:
    std::unique_ptr<D1> _dryve_d1;

    void execute();
  };

}   // namespace dryve_d1_bridge

#endif   // DRYVE_D1_BRIDGE__DRYVE_D1_BRIDGE_HPP_