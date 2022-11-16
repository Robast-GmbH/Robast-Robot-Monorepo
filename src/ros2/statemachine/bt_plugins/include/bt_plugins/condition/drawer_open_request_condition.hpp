#include <string>
#include <vector>
#include <memory>


#include "behaviortree_cpp_v3/condition_node.h"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace drawer_statemachine
{
    /**
     * @brief A BT::ConditionNode that returns SUCCESS when goal is
     * updated on the blackboard and FAILURE otherwise
     */
    class DrawerOpenReq : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::GoalUpdatedCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        DrawerOpenReq(
            const std::string& action_name,
            const BT::NodeConfiguration& conf);

        DrawerOpenReq() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return
            {
              BT::InputPort<std::string>("topic_name", "topic", ""),
              BT::OutputPort<communication_interfaces::msg::DrawerAddress>("drawer_address", "topic")
            };
        }

        

    protected:
        std::string topic_name_;
        void DrawerOpenReq::callbackDrawerOpenReq(const communication_interfaces::msg::DrawerAddress::SharedPtr msg);

    private:
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drawer_open_sub;

    };
}