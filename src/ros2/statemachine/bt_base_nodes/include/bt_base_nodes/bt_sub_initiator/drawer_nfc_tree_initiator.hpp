#ifndef DRAWER_NFC_TREE_INITIATOR__BT_BASE_NODES_HPP
#define DRAWER_NFC_TREE_INITIATOR__BT_BASE_NODES_HPP

#include <functional>
#include <chrono>
#include <string>
#include <map>
#include <functional>


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "std_msgs/msg/string.hpp"
#include "bt_base_nodes/bt_sub_base.hpp"
#include "communication_interfaces/msg/drawer_address.hpp"

namespace bt_base_nodes
{
    class DrawerNFCTreeInitiator: public bt_base_nodes::BTSubBase<std_msgs::msg::String> {
        
    public:
        DrawerNFCTreeInitiator(
            std::map<std::string, communication_interfaces::msg::DrawerAddress> nfc_key_to_DrawerAddress,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        ): BTSubBase(options) 
        {
            nfc_key_to_DrawerAddress_ = nfc_key_to_DrawerAddress;
        }

    protected:
        void callbackRunBT(const std_msgs::msg::String::SharedPtr msg) override
        {
            _blackboard->set<std::map<std::string, communication_interfaces::msg::DrawerAddress>>(
                "nfc_keys", nfc_key_to_DrawerAddress_
                );
            _blackboard->set<std::string>("user_access_name", msg->data);
            _bt.tickWhileRunning(std::chrono::milliseconds(10));
        }
        
    private:
        std::map<std::string, communication_interfaces::msg::DrawerAddress> nfc_key_to_DrawerAddress_;
    };
}

#endif