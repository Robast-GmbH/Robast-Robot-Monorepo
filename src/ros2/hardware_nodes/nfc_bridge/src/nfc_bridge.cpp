#include "nfc_bridge/nfc_bridge.hpp"


namespace nfc_bridge
{

  NFCBridge::NFCBridge(std::string serial_port_path) : Node("nfc_bridge")
  {
     using namespace std::placeholders;
    this->serial_connector_ = new serial_helper::SerialHelper(serial_port_path);
    this->db_connector_ = new db_helper::PostgreSqlHelper("robot", "123456789", "10.10.23.9", "robast");
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    publisher_ = this->create_publisher<std_msgs::msg::String>("/authenticated_user", qos);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(READER_INTEVALL),
                                     std::bind(&NFCBridge::reader_procedure, this));

    this->action_server_ = rclcpp_action::create_server<CreateUser>(this,
                                                                   "create_user",
                                                                   std::bind(&NFCBridge::handle_goal, this, _1, _2),
                                                                   std::bind(&NFCBridge::handle_cancel, this, _1),
                                                                   std::bind(&NFCBridge::handle_accepted, this, _1)
                                                                   );
  }

  rclcpp_action::GoalResponse NFCBridge::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CreateUser::Goal> goal)
  {
    // first and last name longer then 1 char 
    if(goal->first_name.size()>=2 && goal->last_name.size()>=2)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
    
  }

  rclcpp_action::CancelResponse NFCBridge::handle_cancel(
    const std::shared_ptr<GoalHandleCreateUser> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // TODO delete user from DB

    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void NFCBridge::handle_accepted(const std::shared_ptr<GoalHandleCreateUser> goal_handle)
  {
    std::thread{std::bind(&NFCBridge::createUser, this, std::placeholders::_1), 11}.detach();
  }

NFCBridge::~NFCBridge()
{
  turn_off_scanner();
}

void NFCBridge::start_up_scanner()
{
  this->serial_connector_->open_serial();
  std::string response;

  this->serial_connector_->ascii_interaction(DEVICE_STATE, &response, STANDART_REPLAY_MESSAGE_SIZE);
  if (response != RESPONCE_DEVICE_STATE_CONFIGURED)
  {
    return;
  }

  this->serial_connector_->ascii_interaction(SET_SERIAL_TO_ASCII, &response, STANDART_REPLAY_MESSAGE_SIZE);
}

void NFCBridge::prepare_scanning()
{
  std::string response;
  this->serial_connector_->ascii_interaction(BOTTOM_LED_ON, &response, STANDART_REPLAY_MESSAGE_SIZE);
  this->serial_connector_->ascii_interaction(TOP_LEDS_INIT(LED_RED), &response, STANDART_REPLAY_MESSAGE_SIZE);
  this->serial_connector_->ascii_interaction(TOP_LEDS_ON(LED_RED), &response, STANDART_REPLAY_MESSAGE_SIZE);
}

bool NFCBridge::scan_tag(std::shared_ptr<std::string> scanned_key)
{
  std::string response;
  std::string replay = this->serial_connector_->ascii_interaction(SEARCH_TAG, &response, STANDART_REPLAY_MESSAGE_SIZE);

  if (replay == RESPONCE_ERROR)
  {
    *scanned_key = "";
    return false;
  }

  this->serial_connector_->ascii_interaction(NFC_LOGIN_MC_STANDART("00"), &response, STANDART_REPLAY_MESSAGE_SIZE);
  replay =
      this->serial_connector_->ascii_interaction(NFC_READ_MC("02"), scanned_key.get(), STANDART_REPLAY_MESSAGE_SIZE);

  if (replay == RESPONCE_ERROR)
  {
    return false;
  }
  // this->serial_connector_.send_ascii_cmd(BEEP_STANDART);
  this->serial_connector_->send_ascii_cmd(TOP_LED_OFF(LED_GREEN));
  return true;
}

void NFCBridge::turn_off_scanner()
{
  this->serial_connector_->send_ascii_cmd(TOP_LED_OFF(LED_RED));
  this->serial_connector_->send_ascii_cmd(BOTTOM_LED_OFF);
  this->serial_connector_->close_serial();
}

bool NFCBridge::execute_scan(std::shared_ptr<std::string> received_raw_data)
{
  bool found_tag;
  prepare_scanning();
  found_tag = scan_tag(received_raw_data);

  return found_tag;
}

void NFCBridge::reader_procedure()
{
  std::shared_ptr<std::string> scanned_key = std::make_shared<std::string>();
  std::shared_ptr<std::string> found_user = std::make_shared<std::string>();
  bool found = false;
  start_up_scanner();
  found = execute_scan(scanned_key);
  if (found)
  {
    // RCLCPP_INFO(this->get_logger(), "tag located %s", (*scanned_key).c_str());
    if (CHECK_ON_DB || db_connector_->test_connection() == "Dummy")
    {
      if (this->db_connector_->checkUserTag(*scanned_key, std::vector<std::string>(), found_user))
      {
        RCLCPP_INFO(this->get_logger(), "Found tag");
        std_msgs::msg::String message = std_msgs::msg::String();
        message.data = *found_user;
        RCLCPP_INFO(this->get_logger(), "Publishing authenticated user %s", (*found_user).c_str());
        publisher_->publish(message);
      }
    }
    else
    {
      std_msgs::msg::String message = std_msgs::msg::String();
      if (nfc_code_to_drawer_.count(*scanned_key) == 1)
      {
        *found_user = nfc_code_to_drawer_.at(*scanned_key);
        message.data = *found_user;
        RCLCPP_INFO(this->get_logger(), "Publishing Authenticated user %s", (*found_user).c_str());
        publisher_->publish(message);
      }
    }
  }
}
 
void NFCBridge::createUser(const std::shared_ptr<GoalHandleCreateUser> goal_handle)
{
  
  add_user_to_db();
  write_tag(int card_data);
}

void NFCBridge::write_tag(int card_data )
{
 
  start_up_scanner();

  std::string tag;
  // wait for the Tag and read TAG ID
  do
  {
    this->serial_connector_->send_ascii_cmd(SEARCH_TAG);   // search for a tag with the length of 10
    if (this->serial_connector_->read_serial(&tag, 50) <= 0)
    {
      return;
    }

    // RCLCPP_INFO(this->get_logger(),"Received message: %s ", tag.c_str() );
  } while (tag.length() < 10);
  this->serial_connector_->send_ascii_cmd(NFC_LOGIN_MC_STANDART("00"));
  this->serial_connector_->send_ascii_cmd(NFC_WRITE_MC("02", "000001000000100"));   // ToDo dynamic key defination

  if (this->serial_connector_->read_serial(&tag, 50) <= 0)
  {
    return;
  }

  // if (tag != RESPONCE_SUCCESS)
  // {
  //   return;
  // }

  turn_off_scanner();

  // response->sucessful = true;
  // response->card_id = tag;
  // response->error_message;
}

}   // namespace nfc_bridge