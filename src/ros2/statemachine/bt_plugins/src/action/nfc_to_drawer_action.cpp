#include "bt_plugins/action/nfc_to_drawer_action.hpp"

namespace statemachine
{
  NFCToDrawer::NFCToDrawer(const std::string &name,
                           const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config) // TODO @tobi evtl noch abstract base class?
  {

    blackboard_ = config.blackboard;
  }

  BT::NodeStatus NFCToDrawer::onStart()
  {
    using DrawerAddress = communication_interfaces::msg::DrawerAddress;

    _nfc_key_to_DrawerAddress =
        blackboard_->get<std::map<std::string, DrawerAddress>>("nfc_keys");

    std::string nfc_user = blackboard_->get<std::string>("user_access_name");
    if (nfc_user != "" && _nfc_key_to_DrawerAddress.find(nfc_user) != _nfc_key_to_DrawerAddress.end())
    {
      DrawerAddress drawer_address = _nfc_key_to_DrawerAddress[nfc_user];
      blackboard_->set<DrawerAddress>("drawer_address", drawer_address);
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
  factory.registerNodeType<statemachine::NFCToDrawer>("NFCToDrawer");
}
