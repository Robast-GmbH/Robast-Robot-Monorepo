#ifndef DRAWER_STATEMACHINE_ELECTRIC_DRAWER_STATUS_CONDITION_HPP_
#define DRAWER_STATEMACHINE_ELECTRIC_DRAWER_STATUS_CONDITION_HPP_

#include "bt_plugins/condition/base_compare_condition.hpp"           // Base class include
#include "communication_interfaces/msg/drawer_address.hpp"           // Include für communication_interfaces::msg::DrawerAddress
#include "communication_interfaces/msg/electrical_drawer_status.hpp" // Include für communication_interfaces::msg::DrawerStatus

namespace statemachine
{
    class ElectricDrawerStatusCondition : public BaseCompareCondition<communication_interfaces::msg::ElectricalDrawerStatus, uint8_t>
    {
    public:
        ElectricDrawerStatusCondition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<uint8_t>("target_value", "0"),
                BT::InputPort<std::string>("topic", "/drawer_feedback_status"),
                BT::InputPort<bool>("use_stallguard", "false")};
        }
        BT::NodeStatus tick() override;

    protected:
        bool new_value_received_ = false;
        bool comparator(communication_interfaces::msg::ElectricalDrawerStatus last_message_, uint8_t target_value_) override;
        void callbackTopicFeedback(const communication_interfaces::msg::ElectricalDrawerStatus::SharedPtr msg) override;
        void initialize_target_value() override;

    private:
        bool _use_stallguard;
    };
} // namespace statemachine

#endif // DRAWER_STATEMACHINE_ELECTRIC_DRAWER_STATUS_CONDITION_HPP_
