#include "bt_plugins/action/nfc_to_drawer_action.hpp"

namespace drawer_statemachine
{  
  NFCToDrawer::NFCToDrawer(const std::string& name,
          const BT::NodeConfig& config)
          : BT::StatefulActionNode(name, config) //TODO @tobi evtl noch abstract base class?
  {
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;
    
    blackboard_ = config.blackboard;
    nfc_key_to_DrawerAddress_ =
      blackboard_->get<std::map<std::string, DrawerAddress>>("nfc_keys");
  }

  BT::NodeStatus NFCToDrawer::onStart()
  {
    std::string nfc_user = blackboard_->get<std::string>("user_access_name");
    if (nfc_user != "" && nfc_key_to_DrawerAddress_.find(nfc_user) != nfc_key_to_DrawerAddress_.end())
    {
      communication_interfaces::msg::DrawerAddress drawer_address = nfc_key_to_DrawerAddress_[nfc_user];
      blackboard_->set<communication_interfaces::msg::DrawerAddress>("drawer_address", drawer_address);
      
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  
  BT::NodeStatus NFCToDrawer::onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }
  
  void NFCToDrawer::onHalted()
  {
    blackboard_->set<std::string>("user_access_name", "");
  }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<drawer_statemachine::NFCToDrawer>("NFCToDrawer");
}
