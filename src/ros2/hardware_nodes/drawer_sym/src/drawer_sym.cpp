
#include "drawer_sym/drawer_sym.hpp"

namespace robast
{
    DrawerSym::DrawerSym(): Node("drawer_sym")
    {
        this->publisher = this->create_publisher<std_msgs::msg::String>("drawer_info", 10);
        this->subscription = this->create_subscription<std_msgs::msg::String>( "drawer_command", 10, bind(&DrawerSym::startTask, this, placeholders::_1));
        this->drawerInteractionClients = rclcpp_action::create_client<DrawerInteraction>(this, "drawer_interaction");
    }

    void DrawerSym::startTask(const std_msgs::msg::String::SharedPtr msg)
    {   
        vector<string> msg_split;
        this->split(msg->data.c_str(),' ', msg_split);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request Command: |%s|",msg_split.at(0).c_str());

        if(msg_split.at(0)=="INFO")
        {
            drawer_info();
        }
        else if(msg_split.at(0)=="OPEN")
        {
            DrawerSym::open_drawer(msg_split);
        }
        else if(msg_split.at(0)=="TEST")
        {
            DrawerSym::open_drawer_test(msg_split);
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
        shelfInfoClient = this->create_client<ShelfSetupInfo>("get_module_setup");
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

        auto response_received_load_callback = [this](rclcpp::Client<ShelfSetupInfo>::SharedFuture future)
        {
            RCLCPP_INFO(this->get_logger(), "result received");
            this->drawerList =future.get()->drawers;
            for ( size_t i = 0; i != this->drawerList.size(); i++)
            {
                auto response = std_msgs::msg::String();
                auto drawer = this->drawerList [i];
                response.data= drawer.drawer_address.drawer_controller_id + " : " + to_string(drawer.number_of_drawers) + " ( " + to_string(drawer.drawer_size.x) +" * "+ to_string(drawer.drawer_size.y)+" * "+ to_string(drawer.drawer_size.z);
                this->publisher->publish(response);
            }
        };

        auto result = shelfInfoClient->async_send_request(request, response_received_load_callback );

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_module_setup request sent!");
    }

    void DrawerSym::open_drawer(std::vector<std::string> msg_split)
    {
            communication_interfaces::action::DrawerInteraction::Goal goal_msg =create_dummy_drawer_interaction_msg();
            goal_msg.loading= msg_split[1] == "LOADING";
            goal_msg.state=0;
            
            if(msg_split.size() == 4)
            {
                goal_msg.task.drawer_address.drawer_controller_id= stoi(msg_split[2]);
                goal_msg.task.drawer_address.drawer_id= stoi(msg_split[3]);
            }
            else
            {
                goal_msg.task.drawer_address.drawer_controller_id= this->drawerList[ stoi( msg_split[1])].drawer_address.drawer_controller_id;
                goal_msg.task.drawer_address.drawer_id= this->drawerList[ stoi( msg_split[2])].drawer_address.drawer_id;
            }

            send_drawer_interaction( goal_msg);         
    } 

    void DrawerSym::send_drawer_interaction(communication_interfaces::action::DrawerInteraction::Goal goal_msg)
    { 
            auto send_goal_options = rclcpp_action::Client<DrawerInteraction>::SendGoalOptions();
            send_goal_options.goal_response_callback = bind(&DrawerSym::drawer_goal_response_callback, this, placeholders::_1);
            send_goal_options.feedback_callback = bind(&DrawerSym::drawer_feedback_callback, this, placeholders::_1, placeholders::_2);
            send_goal_options.result_callback = bind(&DrawerSym::drawer_result_callback, this, placeholders::_1);

            this->drawerInteractionClients->async_send_goal(goal_msg, send_goal_options);
    }

    void DrawerSym::open_drawer_test(std::vector<std::string> msg_split)
    {
        
        if (!drawerInteractionClients->wait_for_action_server()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                return;
            }
            communication_interfaces::action::DrawerInteraction::Goal goal_msg =create_dummy_drawer_interaction_msg();
             goal_msg.state= atoi( msg_split[1].c_str());
             goal_msg.loading= msg_split[2] == "LOADING";
           
            
            if(msg_split.size() == 4)
            {
                goal_msg.task.drawer_address.drawer_controller_id= stoi(msg_split[3]);
                goal_msg.task.drawer_address.drawer_id= stoi(msg_split[4]);
            }
            else
            {
                goal_msg.task.drawer_address.drawer_controller_id= this->drawerList[ stoi( msg_split[2])].drawer_address.drawer_controller_id;
                goal_msg.task.drawer_address.drawer_id= this->drawerList[ stoi( msg_split[3])].drawer_address.drawer_id;
            }
            this->send_drawer_interaction( goal_msg);    
    } 

    communication_interfaces::action::DrawerInteraction::Goal DrawerSym::create_dummy_drawer_interaction_msg()
    {
        auto pose =geometry_msgs::msg::PoseStamped();
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
            
        auto goal_msg = DrawerInteractionGoal();
        goal_msg.task= communication_interfaces::msg::Task();
        goal_msg.task.ticket= communication_interfaces::msg::Ticket();
            
            
        goal_msg.task.ticket.start_pose = pose;
        goal_msg.task.ticket.goal_pose = pose;

        goal_msg.task.ticket.item_size = communication_interfaces::msg::Box();
        goal_msg.task.ticket.item_size.x = 0;
        goal_msg.task.ticket.item_size.y = 0;
        goal_msg.task.ticket.item_size.z = 0;
        goal_msg.task.ticket.load_keys =     vector<string> {"000101000000000000000000000000010001"};
        goal_msg.task.ticket.drop_of_keys =  vector<string> {"000101000000000000000000000000010001"};
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
        (void) feedback;
        auto responce = std_msgs::msg::String();
        responce.data = "feedback:" ;
        this->publisher->publish(responce);
    }

    void DrawerSym::drawer_result_callback(const GoalHandleDrawerInteraction::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "DrawerInteraction succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "DrawerInteraction Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "DrawerInteraction Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code for DrawerInteraction");
                return;
        }

        auto response = std_msgs::msg::String();
        response.data = "Door operation is done." ;
        this->publisher->publish(response);
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
