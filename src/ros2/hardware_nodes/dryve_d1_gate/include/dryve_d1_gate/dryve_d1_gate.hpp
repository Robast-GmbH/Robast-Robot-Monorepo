#ifndef DRYVE_D1_GATE__DRYVE_D1_GATE_HPP_
#define DRYVE_D1_GATE__DRYVE_D1_GATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "D1.hpp"

namespace dryve_d1_gate
{
  class DryveD1Gate : public rclcpp::Node
  {
   public:
    /**
     * @brief A constructor for dryve_d1_gate::DryveD1Gate class
     */
    DryveD1Gate();
    /**
     * @brief A destructor for dryve_d1_gate::DryveD1Gate class
     */
    ~DryveD1Gate();

   private:
    void execute();
  };

}   // namespace dryve_d1_gate

#endif   // DRYVE_D1_GATE__DRYVE_D1_GATE_HPP_