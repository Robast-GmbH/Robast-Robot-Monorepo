#include <vector>
#include <string>
#include <memory>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"
#include "ros_bt_basics/bt_actor/drawer_actor.hpp"

namespace bt_basics
{
    class DrawerAccessActor: public bt_basics::Actor<communication_interfaces::msg::DrawerAddress>
    {
    public:
        using ActionT = communication_interfaces::msg::DrawerAddress;

        DrawerAccessActor():Actor()
        {}
        bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) override;
        bool cleanup() override;
        std::string getName() { return std::string("drawer_access_actor"); }
        std::string getDefaultBTFilepath(rclcpp_lifecycleNode::LifecycleNode::WeakPtr node) override;

    protected:
        // bool requestRecieved(ActionT::ConstSharedPtr request) override;
        void onLoop() override;
        // void onActionRecieved(ActionT::SharedPtr drawerAddress) override;
        void actionCompleated(ActionT::SharedPtr result,
            //TODO nav BT
            ) override;

        rclcpp::Time start_time_;
        //TODO muss das ne action sein?
        rclcpp::Subscription<ActionT>::SharedPtr drawer_sub_;
        rclcpp_action::Client<ActionT>::SharedPtr self_client_;

        std::string drawer_blackboard_id_;
        // std::string path_blackboard_id_;
    };
}
