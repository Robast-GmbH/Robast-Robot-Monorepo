#ifndef DRAWER_STATEMACHINE_ELECTRIC_DRAWER_STATUS_CONDITION_HPP_
#define DRAWER_STATEMACHINE_ELECTRIC_DRAWER_STATUS_CONDITION_HPP_

#include "bt_plugins/condition/base_compare_condition.hpp" // Base class include
#include "communication_interfaces/msg/drawer_address.hpp" // Include für communication_interfaces::msg::DrawerAddress
#include "communication_interfaces/msg/drawer_status.hpp"  // Include für communication_interfaces::msg::DrawerStatus

namespace drawer_statemachine
{
    class ElectricDrawerStatusCondition : public BaseCompareCondition<communication_interfaces::msg::DrawerStatus>
    {
    public:
        ElectricDrawerStatusCondition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("target_value", "false"),
                BT::InputPort<std::string>("topic", "/drawer_feedback_status")};
        }

    protected:
        bool new_value_received_ = false;
        bool comparator(communication_interfaces::msg::DrawerStatus last_message_, communication_interfaces::msg::DrawerStatus target_value_) override;
        void callbackDrawerFeedback(const communication_interfaces::msg::DrawerStatus::SharedPtr msg) override;
        void initialize_target_value() override;
    };
} // namespace drawer_statemachine

#endif // DRAWER_STATEMACHINE_ELECTRIC_DRAWER_STATUS_CONDITION_HPP_
