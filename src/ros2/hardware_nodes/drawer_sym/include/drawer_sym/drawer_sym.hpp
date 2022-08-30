#ifndef DRAWER_SYM_HPP_
#define DRAWER_SYM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "communication_interfaces/srv/shelf_setup_info.hpp"
#include "communication_interfaces/action/drawer_interaction.hpp"
#include <string.h>
using namespace std;
   
namespace robast
{ 
  
  class DrawerSym : public rclcpp::Node
  {
    private:
      using ShelfSetupInfo = communication_interfaces::srv::ShelfSetupInfo; 
      using DrawerInteraction =communication_interfaces::action::DrawerInteraction; 
      using DrawerInteractionGoal = DrawerInteraction::Goal;
      
      using GoalHandleDrawerInteraction = rclcpp_action::ClientGoalHandle<DrawerInteraction>;
      
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
      
      rclcpp::Client<ShelfSetupInfo>::SharedPtr shelfInfoClient;
      rclcpp_action::Client<DrawerInteraction>::SharedPtr drawerInteractionClients;

      void startTask(const std_msgs::msg::String::SharedPtr msg);
      void split(string input, char deliminator,  vector<string> & output); 
      
      vector<communication_interfaces::msg::Drawer, allocator<communication_interfaces::msg::Drawer>> drawerList;

      void drawer_goal_response_callback(const GoalHandleDrawerInteraction::SharedPtr & goal_handle);
      void drawer_feedback_callback( GoalHandleDrawerInteraction::SharedPtr, const shared_ptr<const DrawerInteraction::Feedback> feedback);
      void drawer_result_callback(const GoalHandleDrawerInteraction::WrappedResult & result);


      void drawer_info();
      void open_drawer(std::vector<std::string> msg_split);
      DrawerInteractionGoal create_dummy_drawer_interaction_msg();

    public:
    DrawerSym();
  };
}  // namespace robast
#endif  // DRAWER_SYM_HPP_