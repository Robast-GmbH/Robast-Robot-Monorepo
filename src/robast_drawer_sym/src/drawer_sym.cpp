
#include "robast_drawer_sym/drawer_sym.hpp"

namespace robast
{
    DrawerSym::DrawerSym(): Node("robast_drawer_sym")
    {
        this->publisher = this->create_publisher<std_msgs::msg::String>("drawerInfo", 10);
        this->subscription = this->create_subscription<std_msgs::msg::String>( "drawerCommand", 10, bind(&DrawerSym::startTask, this, placeholders::_1));
    }

    void DrawerSym::startTask(const std_msgs::msg::String::SharedPtr msg)
    {   
        vector<string> msg_split;
        this->split(msg->data.c_str(),' ', msg_split);

         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request Command: %s",msg_split.at(0).c_str());

        if(msg_split.at(0)=="INFO")
        {
            drawer_info();
        }
        else if(msg_split.at(0)=="OPEN")
        {
            open_drawer(msg_split);
        }
        else if(msg_split.at(0)=="HELP")
        {
             auto responce = std_msgs::msg::String();
                    responce.data= "Shelf info: INFO \n open Drawer V1: DRAWER <LOADING/NOTLOADING> <controller_id> <drawer_id>\n Open Drawer V2: DRAWER <LOADING/NOTLOADING> <shelf info index> \n";
                    this->publisher->publish(responce);
        } 
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The recived Commad \"%s\" in unknown.", msg_split.at(0).c_str());
            auto responce = std_msgs::msg::String();
            responce.data = "Command unknown!!" ;
            this->publisher->publish(responce);
        }        
    }

    void DrawerSym::drawer_info()
    {
        shelfInfoClient = this->create_client<ShelfSetupInfo>("/Get_module_setup");
            auto request = std::make_shared<ShelfSetupInfo::Request>();
            
            while (!shelfInfoClient->wait_for_service(1s)) 
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto response_received_load_callback = [this]( rclcpp::Client<ShelfSetupInfo>::SharedFuture future) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result recived");
                this->drawerList =future.get()->drawers;
                for ( size_t i = 0; i != this->drawerList.size(); )
                {
                    auto responce = std_msgs::msg::String();
                    auto drawer = this->drawerList [i];
                    responce.data= drawer.drawer_address.drawer_controller_id + " : " + to_string(drawer.number_of_drawers) + " ( " + to_string(drawer.drawer_size.x) +" * "+ to_string(drawer.drawer_size.y)+" * "+ to_string(drawer.drawer_size.z);
                    this->publisher->publish(responce);
                }
            };

            auto result = shelfInfoClient->async_send_request(request, response_received_load_callback );

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request send");
    }

    void DrawerSym::open_drawer(std::vector<std::string> msg_split)
    {
        if (!drawerActionsClients->wait_for_action_server()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                return;
            }
            auto goal_msg =create_dummy_drawer_interaction_msg();
            goal_msg.loading= msg_split[1] == "LOADING";
            
            
            if(msg_split.size() == 4)
            {
                goal_msg.task.drawer.drawer_controller_id= stoi(msg_split[2]);
                goal_msg.task.drawer.drawer_id= stoi(msg_split[3]);
            }
            else
            {
                goal_msg.task.drawer.drawer_controller_id= this->drawerList[ stoi( msg_split[1])].drawer_address.drawer_controller_id;
                goal_msg.task.drawer.drawer_id= this->drawerList[ stoi( msg_split[2])].drawer_address.drawer_id;
            }

            auto send_goal_options = rclcpp_action::Client<DrawerInteraction>::SendGoalOptions();
            send_goal_options.goal_response_callback = bind(&DrawerSym::drawer_goal_response_callback, this, placeholders::_1);
            send_goal_options.feedback_callback = bind(&DrawerSym::drawer_feedback_callback, this, placeholders::_1, placeholders::_2);
            send_goal_options.result_callback = bind(&DrawerSym::drawer_result_callback, this, placeholders::_1);
            this->drawerActionsClients->async_send_goal(goal_msg, send_goal_options);
    } 

    robast_ros2_msgs::action::DrawerInteraction::Goal DrawerSym::create_dummy_drawer_interaction_msg()
    {
        auto pose =geometry_msgs::msg::PoseStamped();
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
            
        auto goal_msg = DrawerInteractionGoal();
        goal_msg.task= robast_ros2_msgs::msg::Task();
        goal_msg.task.ticket= robast_ros2_msgs::msg::Ticket();
            
            
        goal_msg.task.ticket.start_pose = pose;
        goal_msg.task.ticket.goal_pose = pose;

        goal_msg.task.ticket.item_size = robast_ros2_msgs::msg::Box();
        goal_msg.task.ticket.item_size.x = 0;
        goal_msg.task.ticket.item_size.y = 0;
        goal_msg.task.ticket.item_size.z = 0;
        goal_msg.task.ticket.load_key =     vector<string> {"0000000000000000"};
        goal_msg.task.ticket.drop_of_key =  vector<string> {"0000000000000000"};
        return goal_msg;
    }

    void DrawerSym::drawer_goal_response_callback(const GoalHandleDrawerInteraction::SharedPtr & goal_handle)
    {
        if (!goal_handle) 
        {
            auto responce = std_msgs::msg::String();
            responce.data = "request was Rejected" ;
            this->publisher->publish(responce);
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(), "drawer opening process started ");
        }
    }

    void DrawerSym::drawer_feedback_callback( GoalHandleDrawerInteraction::SharedPtr, const shared_ptr<const DrawerInteraction::Feedback> feedback)
    {
        auto responce = std_msgs::msg::String();
        responce.data = "feedback:" ;
        this->publisher->publish(responce);
    }

    void DrawerSym::drawer_result_callback(const GoalHandleDrawerInteraction::WrappedResult & result)
    {
        auto responce = std_msgs::msg::String();
        responce.data = "Door operation is done." ;
        this->publisher->publish(responce);
    }

    void DrawerSym::split(string input, char deliminator,  vector<string> &output) 
    {
       int start = 0;
       int end = input.find(deliminator);
        while (end != -1) {
            output.push_back( input.substr(start, end - start));
            start = end + 1;
            end = input.find(deliminator, start);
        }
        output.push_back( input.substr(start, end - start));
    }


}  // namespace robast