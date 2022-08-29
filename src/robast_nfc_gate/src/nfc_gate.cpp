
#include "robast_nfc_gate/nfc_gate.hpp"

namespace robast
{

  NFCGate::NFCGate( ):NFCGate( "/dev/robast/robast_nfc" ) { }

  NFCGate::NFCGate( string serial_port_path ) : Node("robast_nfc_gate"), serial_connector(serial_port_path)
  {
    //this->serial_connector= new robast_serial::SerialHelper(); 
    RCLCPP_INFO(this->get_logger(), "constructor start: %s",serial_port_path.c_str()); 
    this->serial_connector=robast_serial::SerialHelper(serial_port_path);
    this->user_authenticate_server = rclcpp_action::create_server<AuthenticateUser>(
      this,
      "authenticate_user",
      bind(&NFCGate::auth_goal_callback, this, placeholders::_1, placeholders::_2),
      bind(&NFCGate::auth_cancel_callback, this, placeholders::_1),
      bind(&NFCGate::auth_accepted_callback, this, placeholders::_1)
      );
    this->create_user_server =this->create_service<CreateUser>("create_user_tag",bind(&NFCGate::writeTag, this, placeholders::_1, placeholders::_2));
    
    //RCLCPP_INFO(this->get_logger(), "constructor done");
  }


  rclcpp_action::GoalResponse NFCGate::auth_goal_callback( const rclcpp_action::GoalUUID & uuid, shared_ptr<const AuthenticateUser::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse NFCGate::auth_cancel_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void NFCGate::auth_accepted_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "scan task");
    numReadings = 0;
    this->timer_handle = goal_handle;
    timer = this->create_wall_timer( 500ms, bind(&NFCGate::scanTag, this));
    //std::thread{std::bind(&NFCGate::scanTag, this, placeholders::_1), goal_handle}.detach();
    
  }


  void NFCGate::scanTag() 
  {
    auto reader_status = std::make_shared<robast_ros2_msgs::msg::NFCStatus>();
    reader_status->is_preparing = false;
    reader_status->is_reading = true;
    reader_status->is_completed = false;
    string scanned_key, tag, response, request;  
    bool is_tag_valid = false;
    auto result = std::make_shared<AuthenticateUser::Result>();
    const auto goal = timer_handle->get_goal();

    this->serial_connector.open_serial();
    request = this->serial_connector.send_ascii_cmd( SET_SERIAL_TO_ASCII);  
    this->serial_connector.read_serial(&response, 5010);

    request = this->serial_connector.send_ascii_cmd(BOTTOM_LED_ON);
    this->serial_connector.read_serial(&response, 500);
    
    request = this->serial_connector.send_ascii_cmd((TOP_LEDS_INIT(LED_RED)));
    this->serial_connector.read_serial(&response, 50);
    
    request = this->serial_connector.send_ascii_cmd(TOP_LEDS_ON(LED_RED));
    this->serial_connector.read_serial(&response, 500);

    request = this->serial_connector.send_ascii_cmd(SEARCH_TAG);
    if(this->serial_connector.read_serial(&tag, 500)<=0)
    {
      return;
    } 

   
    request=this->serial_connector.send_ascii_cmd(NFC_LOGIN_MC_STANDART("00"));
    this->serial_connector.read_serial(&response, 500);
    RCLCPP_INFO(this->get_logger(),"login response %s ", response.c_str());
    
    request= this->serial_connector.send_ascii_cmd(NFC_READ_MC("02"));
    if(this->serial_connector.read_serial(&scanned_key, 500)<=0)
    {
      return;
    }
    RCLCPP_INFO(this->get_logger(),"data on the Tag %s ", scanned_key.c_str());
    scanned_key.pop_back(); // remove '/r'
    for( int i=0; i < goal->permission_keys.size(); i++)
    {
      if(goal->permission_keys[i] == scanned_key)
      {
        is_tag_valid = true;
        RCLCPP_INFO(this->get_logger(),"found");
        break;
      }
    } 
    
    auto feedback = std::make_shared<AuthenticateUser::Feedback>();
    feedback->reader_status.reading_attempts = ++numReadings;
    if(!is_tag_valid)
    {
      this->timer_handle->publish_feedback(feedback);
      return;
    }
    else
    {
      this->timer->cancel();
      feedback->reader_status.is_completed = true;
      this->timer_handle->publish_feedback(feedback);
      
      //this->serial_connector.send_ascii_cmd(BEEP_STANDART);
      this->serial_connector.send_ascii_cmd((TOP_LED_OFF(LED_RED)));
      this->serial_connector.read_serial(&response, 50);
      this->serial_connector.send_ascii_cmd(BOTTOM_LED_OFF); 
      this->serial_connector.read_serial(&response, 50);

      this->serial_connector.close_serial();

      // Check if goal is done
      if (rclcpp::ok()) 
      {
          result->permission_key_used = scanned_key;
          result->error_message = ""; 
          this->timer_handle->succeed(result);
      }
    }    
  }

  void NFCGate::writeTag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response)
  {
      this->serial_connector.open_serial();
      this->serial_connector.send_ascii_cmd(SET_SERIAL_TO_ASCII);
  
      this->serial_connector.send_ascii_cmd(BOTTOM_LED_ON);
      this->serial_connector.send_ascii_cmd((TOP_LEDS_INIT(LED_RED)));
      this->serial_connector.send_ascii_cmd(TOP_LEDS_ON(LED_RED));
  
      string tag;
      //wait for the Tag and read TAG ID 
      do{ 
    
      this->serial_connector.send_ascii_cmd(SEARCH_TAG);//search for a tag with the length of 10
      if(this->serial_connector.read_serial(&tag, 50)<=0)
      {
          return;
      } 

      RCLCPP_INFO(this->get_logger(),"Received message: %s ", tag.c_str() );
      } while(tag.length() <10);
  
      RCLCPP_INFO(this->get_logger(),"READ");
      this->serial_connector.send_ascii_cmd(NFC_LOGIN_MC_STANDART("00"));
    
      this->serial_connector.send_ascii_cmd(NFC_WRITE_MC("02",/*request->card_key*/"000001000000100"));//ToDo dynamic key defination 
   
      if( this->serial_connector.read_serial(&tag, 50)<=0)
      {
        return;
      } 
      if(tag!=BOOL_SUCESS)
      {
        return;
      }
    

    //show that aktion is done
      this->serial_connector.send_ascii_cmd(BEEP_STANDART);   
      this->serial_connector.send_ascii_cmd((TOP_LED_OFF(LED_RED)));
      this->serial_connector.send_ascii_cmd(BOTTOM_LED_OFF); 
      this->serial_connector.close_serial();

    response-> sucessful= true;
    response-> card_id= tag;
    response-> error_message;
  }

}  // namespace robast_drawer_gate