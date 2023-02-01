#include "nfc_gate/nfc_gate.hpp"


namespace robast
{

  NFCGate::NFCGate(std::string serial_port_path) :Node("nfc_gate")
  {
    this->serial_connector_ = new serial_helper::SerialHelper(serial_port_path);
    this->db_connector_= new db_helper::PostgreSqlHelper("robot", "123456789", "10.10.23.9", "robast");

    this->user_authenticate_server_ = rclcpp_action::create_server<AuthenticateUser>(
      this,
      "authenticate_user",
      std::bind(&NFCGate::auth_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NFCGate::auth_cancel_callback, this, std::placeholders::_1),
      std::bind(&NFCGate::auth_accepted_callback, this, std::placeholders::_1)
      );

    this->create_user_server_ = this->create_service<CreateUser>("create_user_tag", bind(&NFCGate::write_tag, this, std::placeholders::_1, std::placeholders::_2));
  }

  NFCGate::~NFCGate()
  {
   
  }

  rclcpp_action::GoalResponse NFCGate::auth_goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const AuthenticateUser::Goal> goal)
  {
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse NFCGate::auth_cancel_callback(const std::shared_ptr<GoalHandleAuthenticateUser> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void NFCGate::auth_accepted_callback(const std::shared_ptr<GoalHandleAuthenticateUser> goal_handle)
  {

    numReadings_ = 0;
    this->timer_handle_ = goal_handle;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(READER_INTEVALL), std::bind(&NFCGate::reader_procedure, this));
    //std::thread{std::bind(&NFCGate::scanTag, this, placeholders::_1), goal_handle}.detach();
  }

  void NFCGate::start_up_scanner()
  {
    this->serial_connector_->open_serial();
    std::string response;
    this->serial_connector_->ascii_interaction(DEVICE_STATE, &response, STANDART_REPLAY_MESSAGE_SIZE);
    this->serial_connector_->ascii_interaction(DEVICE_STATE, &response, STANDART_REPLAY_MESSAGE_SIZE);

    if (response != RESPONCE_DEVICE_STATE_CONFIGURED)
    {

      RCLCPP_ERROR(this->get_logger(), "NFC Device is not setup properly. nfc Node shutting down(%s)", response.c_str());
      rclcpp::shutdown();
    }

    this->serial_connector_->ascii_interaction(SET_SERIAL_TO_ASCII, &response, STANDART_REPLAY_MESSAGE_SIZE);
    this->serial_connector_->ascii_interaction(BOTTOM_LED_ON, &response, STANDART_REPLAY_MESSAGE_SIZE);
    this->serial_connector_->ascii_interaction(TOP_LEDS_INIT(LED_RED), &response, STANDART_REPLAY_MESSAGE_SIZE);
    this->serial_connector_->ascii_interaction(TOP_LEDS_ON(LED_RED), &response, STANDART_REPLAY_MESSAGE_SIZE);
  }

  bool NFCGate::scan_tag(std::shared_ptr< std::string> scanned_key )
  {
    std::string response;
    std::string replay = this->serial_connector_->ascii_interaction(SEARCH_TAG, &response, STANDART_REPLAY_MESSAGE_SIZE);
    if (replay == RESPONCE_ERROR)
    {
      *scanned_key = "";
      return false;
    }

    this->serial_connector_->ascii_interaction(NFC_LOGIN_MC_STANDART("00"), &response, STANDART_REPLAY_MESSAGE_SIZE);
    replay = this->serial_connector_->ascii_interaction(NFC_READ_MC("02"), scanned_key.get(), STANDART_REPLAY_MESSAGE_SIZE);

    if (replay == RESPONCE_ERROR)
    {
      return false;
    }
    return true;
  }

  bool NFCGate::validate_key(std::string scanned_key, std::vector<std::string> allValidUser, std::shared_ptr<std::string> validated_user )
  {
    std::string name;
     return db_connector_->checkUserTag(scanned_key, allValidUser, validated_user);
  }
  
  void NFCGate::turn_off_scanner()
  {
    //this->serial_connector.send_ascii_cmd(BEEP_STANDART);
    this->serial_connector_->send_ascii_cmd(TOP_LED_OFF(LED_RED));
    this->serial_connector_->send_ascii_cmd(BOTTOM_LED_OFF);
    this->serial_connector_->close_serial();
  }

  bool NFCGate::execute_scan(std::vector<std::string> permission_users, std::shared_ptr<std::string> validated_user)
  {
    start_up_scanner();
    std::shared_ptr<std::string> scanned_key = std::make_shared<std::string>();

    // abort this scan attempt if the reader could not detect a compatible card. 
    if (scan_tag(scanned_key))
    {
        return this->validate_key(*scanned_key, permission_users, validated_user);
    }
    else
    {
      *validated_user = "";
      return false;
    }
  }

  void NFCGate::reader_procedure()
  {
    auto reader_status = std::make_shared<communication_interfaces::msg::NFCStatus>();
    reader_status->is_preparing = false;
    reader_status->is_reading = true;
    reader_status->is_completed = false;
    std::shared_ptr<std::string> scanned_key = std::make_shared<std::string>();
    bool found = false;

    auto result = std::make_shared<AuthenticateUser::Result>();
    const auto goal = timer_handle_->get_goal();

    found = execute_scan(goal->permission_keys, scanned_key );

    auto feedback = std::make_shared<AuthenticateUser::Feedback>();
    feedback->reader_status.reading_attempts = ++numReadings_;

    if (!found)
    {
      this->timer_handle_->publish_feedback(feedback);
      return;
    }
    else
    {
      this->timer_->cancel();
      feedback->reader_status.is_completed = true;
      this->timer_handle_->publish_feedback(feedback);

      if (rclcpp::ok())
      {

        turn_off_scanner();
        result->permission_key_used = *scanned_key;
        result->error_message = "";
        this->timer_handle_->succeed(result);
      }
    }
  }

  void NFCGate::write_tag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response)
  {
    (void)request;
    start_up_scanner();

    std::string tag;
    //wait for the Tag and read TAG ID 
    do
    {
      this->serial_connector_->send_ascii_cmd(SEARCH_TAG);//search for a tag with the length of 10
      if (this->serial_connector_->read_serial(&tag, 50) <= 0)
      {
        return;
      }

      //RCLCPP_INFO(this->get_logger(),"Received message: %s ", tag.c_str() );
    } while (tag.length() < 10);
    this->serial_connector_->send_ascii_cmd(NFC_LOGIN_MC_STANDART("00"));
    this->serial_connector_->send_ascii_cmd(NFC_WRITE_MC("02", "000001000000100"));//ToDo dynamic key defination 

    if (this->serial_connector_->read_serial(&tag, 50) <= 0)
    {
      return;
    }

    if (tag != RESPONCE_SUCCESS)
    {
      return;
    }

    turn_off_scanner();

    response->sucessful = true;
    response->card_id = tag;
    response->error_message;
  }

}  // namespace drawer_gate