#ifndef ROBAST_DRAWER_SYM_HPP_
#define ROBAST_DRAWER_SYM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "robast_ros2_msgs/srv/shelf_setup_info.hpp"
#include "robast_ros2_msgs/action/drawer_interaction.hpp"
#include <string.h>
using namespace std;
   
namespace robast
{ 
  
  class DrawerSym : public rclcpp::Node
  {
    private:
      using ShelfSetupInfo = robast_ros2_msgs::srv::ShelfSetupInfo; 
      using DrawerInteraction =robast_ros2_msgs::action::DrawerInteraction; 
      
      using GoalHandleDrawerInteraction = rclcpp_action::ClientGoalHandle<DrawerInteraction>;
      
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
      
      rclcpp::Client<ShelfSetupInfo>::SharedPtr shelfInfoClient;
      rclcpp_action::Client<DrawerInteraction>::SharedPtr drawerActionsClients;

      void startTask(const std_msgs::msg::String::SharedPtr msg);
      void split(string input, char deliminator,  vector<string> output); 
      
      vector<robast_ros2_msgs::msg::Drawer, allocator<robast_ros2_msgs::msg::Drawer>> drawerList;

      void drawer_goal_response_callback(const GoalHandleDrawerInteraction::SharedPtr & goal_handle);
      void drawer_feedback_callback( GoalHandleDrawerInteraction::SharedPtr, const shared_ptr<const DrawerInteraction::Feedback> feedback);
      void drawer_result_callback(const GoalHandleDrawerInteraction::WrappedResult & result);


    public:
    DrawerSym();
  };
}  // namespace robast
#endif  // ROBAST_DRAWER_SYM_HPP_