#ifndef HARDWARE_NODES__DRAWER_SYM_HPP_
#define HARDWARE_NODES__DRAWER_SYM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/srv/shelf_setup_info.hpp"
#include "communication_interfaces/action/drawer_interaction.hpp"
#include <string.h>
namespace robast
{ 
  
  class DrawerSym : public rclcpp::Node
  {
    private:
    
      using ShelfSetupInfo = communication_interfaces::srv::ShelfSetupInfo; 
      using DrawerInteraction =communication_interfaces::action::DrawerInteraction; 
      using DrawerInteractionGoal = DrawerInteraction::Goal;
      
      using GoalHandleDrawerInteraction = rclcpp_action::ClientGoalHandle<DrawerInteraction>;
      
      bool debug;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
      
      rclcpp::Client<ShelfSetupInfo>::SharedPtr shelfInfoClient_;
      rclcpp_action::Client<DrawerInteraction>::SharedPtr drawerInteractionClients_;

      
      void startTask(const std_msgs::msg::String::SharedPtr msg);
      void split(std::string input, char deliminator,  std::vector<std::string> & output); 
      
      std::vector<communication_interfaces::msg::Drawer, std::allocator<communication_interfaces::msg::Drawer>> drawerList;

      void drawer_goal_response_callback(const GoalHandleDrawerInteraction::SharedPtr & goal_handle);
      void drawer_feedback_callback( GoalHandleDrawerInteraction::SharedPtr, const std::shared_ptr<const DrawerInteraction::Feedback> feedback);
      void drawer_result_callback(const GoalHandleDrawerInteraction::WrappedResult & result);


      void drawer_info();
      void open_drawer(std::vector<std::string> msg_split);
      DrawerInteractionGoal create_dummy_drawer_interaction_msg();
      void open_drawer_test(std::vector<std::string> msg_split);
      void send_drawer_interaction(communication_interfaces::action::DrawerInteraction::Goal goal_msg);
    public:
    DrawerSym();
  };
}  // namespace robast
#endif  //HARDWARE_NODES__DRAWER_SYM_HPP_