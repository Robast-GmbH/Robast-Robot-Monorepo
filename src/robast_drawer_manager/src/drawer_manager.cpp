#include <memory>

#include "robast_drawer_manager/drawer_manager.hpp"

namespace robast_drawer_manager
{
  

  DrawerManager::DrawerManager(): Node("drawer_manager_Node")
  {
    RCLCPP_INFO(get_logger(), "Creating");
    this->get_all_drawers_server = this->create_service<robast_msgs::srv::GetAllAttachedDrawers>("Get_all_attached_drawer_drawer_management", &get_all_drawer);
    this->handle_drawer_server = this->create_service<robast_msgs::srv::HandleDrawer>("open_drawer_drawer_management", &open_drawer);
    this->open_drawers_client = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");
  }

  DrawerManager::~DrawerManager()
  {
    RCLCPP_INFO(get_logger(), "Destroying");
  }

  void get_all_drawer(const std::shared_ptr<robast_msgs::srv::GetAllAttachedDrawers::Request> request,
                      std::shared_ptr<robast_msgs::srv::GetAllAttachedDrawers::Response>      response)
{
    rclcpp::Client<robast_msgs::srv::GetAllAttachedDrawers>::SharedPtr stateMaschineRequest =  node->create_client<robast_msgs::srv::GetAllAttachedDrawers>("Get_all_attached_drawer");
    response->drawers;
}

void open_drawer(const std::shared_ptr<robast_msgs::srv::HandleDrawer::Request> request,
                      std::shared_ptr<robast_msgs::srv::HandleDrawer::Response>      response)
{
    //start NFC Reader 
      //im Frontend anzeigen das der Reader  aktive ist
       RCLCPP_INFO(get_logger(), "The reader is active please authinticate yourselfe at the reader.");
    
    // open drawer
      // Schublade offen im Frontend anzeigen
      RCLCPP_INFO(get_logger(), "The drawer is open please close the drawer after you are done.");
      
      // nach zeit intervall //bitte schließen imfrontend anzeigen

    //  wenn die Schublade  geschlossen ist action beenden 
}

} // namespace robast_drawer_manager