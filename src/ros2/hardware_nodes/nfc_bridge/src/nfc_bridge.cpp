#include "nfc_bridge/nfc_bridge.hpp"

namespace nfc_bridge
{

  NFCBridge::NFCBridge() : Node("nfc_bridge")
  {
    this->declare_parameter("serial_port_path", "/dev/robast/robast_nfc");
    this->declare_parameter("db_username", "postgres");
    this->declare_parameter("db_password", "postgres");
    this->declare_parameter("db_host_address", "localhost");
    this->declare_parameter("db_port", 5432);
    this->declare_parameter("db_name", "robast");

    std::string serial_port_path = this->get_parameter("serial_port_path").as_string();
    std::string db_username = this->get_parameter("db_username").as_string();
    std::string db_password = this->get_parameter("db_password").as_string();
    std::string db_host = this->get_parameter("db_host_address").as_string();
    int db_port = this->get_parameter("db_port").as_int();
    std::string db_name = this->get_parameter("db_name").as_string();

    this->serial_connector_ =
        std::make_unique<serial_helper::SerialHelper>(serial_helper::SerialHelper(serial_port_path));
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.avoid_ros_namespace_conventions(false);

    authentication_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/authenticated_user", qos);
    timer_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/nfc_switch", qos, std::bind(&NFCBridge::control_timer, this, std::placeholders::_1));
    this->db_connector_ = std::make_unique<db_helper::PostgreSqlHelper>(
        db_helper::PostgreSqlHelper(db_username, db_password, db_host, db_port, db_name));
    if (this->db_connector_->test_connection() != "successfull")
    {
      RCLCPP_WARN(this->get_logger(), "DB connection test: %s", this->db_connector_->test_connection().c_str());
    }

    this->action_server_ = rclcpp_action::create_server<CreateUser>(
        this,
        "/create_user",
        std::bind(&NFCBridge::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NFCBridge::handle_cancel, this, std::placeholders::_1),
        std::bind(&NFCBridge::handle_accepted, this, std::placeholders::_1));
  }

  rclcpp_action::GoalResponse NFCBridge::handle_goal(const rclcpp_action::GoalUUID&,
                                                     std::shared_ptr<const CreateUser::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse NFCBridge::handle_cancel(const std::shared_ptr<GoalHandleCreateUser> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void NFCBridge::handle_accepted(const std::shared_ptr<GoalHandleCreateUser> goal_handle)
  {
    std::thread{std::bind(&NFCBridge::createUser, this, std::placeholders::_1), goal_handle}.detach();
  }

  void NFCBridge::control_timer(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      this->timer_start();
    }
    else
    {
      this->timer_stop();
    }
  }

  void NFCBridge::timer_start()
  {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(READER_INTEVALL),
                                     std::bind(&NFCBridge::reading_procedure, this));
  }

  void NFCBridge::timer_stop()
  {
    this->timer_->cancel();
  }

  NFCBridge::~NFCBridge()
  {
    shutdown_scanner();
  }

  void NFCBridge::start_up_scanner()
  {
    this->serial_connector_->open_serial();
  }

  void NFCBridge::shutdown_scanner()
  {
    this->serial_connector_->close_serial();
  }

  bool NFCBridge::read_nfc_code(std::shared_ptr<std::string> scanned_key)
  {
    int size_of_received_data = serial_connector_->read_serial(scanned_key.get(), 100);
    return size_of_received_data > 0;
  }

  void NFCBridge::reading_procedure()
  {
    start_up_scanner();
    std::shared_ptr<std::string> scanned_key = std::make_shared<std::string>();
    std::shared_ptr<std::string> found_user = std::make_shared<std::string>();
    std::shared_ptr<std::string> error_msg = std::make_shared<std::string>();
    std::shared_ptr<int> found_user_id = std::make_shared<int>();

    if (read_nfc_code(scanned_key))
    {
      if (db_connector_->checkUserTag(*scanned_key, found_user, found_user_id, error_msg))
      {
        std_msgs::msg::Int64 message = std_msgs::msg::Int64();
        message.data = *found_user_id;
        RCLCPP_INFO(this->get_logger(), "Publishing authenticated user %s %i", (*found_user).c_str(), *found_user_id);
        authentication_publisher_->publish(message);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Card was not recognised.");
        std_msgs::msg::Int64 message = std_msgs::msg::Int64();
        message.data = -1;
        authentication_publisher_->publish(message);
      }
    }
    shutdown_scanner();
  }

  void NFCBridge::createUser(const std::shared_ptr<GoalHandleCreateUser> goal_handle)
  {
    std::string user_id;
    std::shared_ptr<const communication_interfaces::action::CreateUserNfcTag_Goal> goal = goal_handle->get_goal();
    std::shared_ptr<CreateUser::Result> result = std::make_shared<CreateUser::Result>();
    std::shared_ptr<CreateUser::Feedback> feedback = std::make_shared<CreateUser::Feedback>();
    std::shared_ptr<std::string> error_msg = std::make_shared<std::string>();

    feedback->task_status.is_db_user_created = false;
    feedback->task_status.is_reader_ready_to_write = false;
    feedback->task_status.is_reader_completed = false;
    goal_handle->publish_feedback(feedback);

    if (db_connector_->checkUser(goal->user_id, error_msg))
    {
      feedback->task_status.user_id = user_id;
    }
    else if (*error_msg != "")
    {
      feedback->task_status.is_reader_error = true;
      result->task_status = feedback->task_status;
      result->successful = false;
      result->error_message = *error_msg;
      goal_handle->canceled(result);
      return;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "user_id not found");
      result->task_status = feedback->task_status;
      result->successful = false;
      result->error_message = "Der Benutzer konnte nicht gefunden werden.";
      goal_handle->canceled(result);
      return;
    }

    user_id = goal->user_id;
    feedback->task_status.is_db_user_created = true;
    goal_handle->publish_feedback(feedback);
    std::shared_ptr<std::string> scanned_key = std::make_shared<std::string>();
    while (!read_nfc_code(scanned_key))
    {
      if (goal_handle->is_canceling())
      {
        result->nfc_code = "";
        result->error_message = "create new nfc token was canceled.";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "%s", result->error_message.c_str());
        return;
      }
    }

    if (!db_connector_->createNfcCode(user_id, *scanned_key))
    {
      RCLCPP_ERROR(this->get_logger(), "card Id could not be stored.");
      result->task_status = feedback->task_status;
      result->successful = false;
      result->error_message = "Die Karten daten konnten nicht gespeichert werden.";
      goal_handle->canceled(result);
      return;
    }
    feedback->task_status.is_reader_ready_to_write = false;
    goal_handle->publish_feedback(feedback);

    feedback->task_status.is_reader_completed = true;
    goal_handle->publish_feedback(feedback);
    result->nfc_code = *scanned_key;
    result->successful = true;
    result->task_status = feedback->task_status;
    goal_handle->succeed(result);
  }

}   // namespace nfc_bridge