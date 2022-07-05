
#include "robast_nfc_gate/nfc_gate.hpp"

namespace robast_nfc_gate
{

NFCGate::NFCGate( ):NFCGate( "/dev/ttyACM0" ) { }

NFCGate::NFCGate( string serial_port_path ) : Node("robast_nfc_gate")
{
  RCLCPP_INFO(this->get_logger(), "constructor start: %s",serial_path.c_str()); 
  serial_path = serial_port_path;
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
  // (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NFCGate::auth_accepted_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "scan task");
  std::thread{std::bind(&NFCGate::scanTag, this, placeholders::_1), goal_handle}.detach();
}


void NFCGate::open_serial()
{
    this->serial_port = open(serial_path.c_str() , O_RDWR);
    // Check for errors
    if (this->serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
      return;
    }

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(this->serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port, TCSANOW, &tty) != 0) {
     RCLCPP_INFO(this->get_logger(),"Error %i from tcsetattr: %s\n", errno, strerror(errno)); 

      return;
    }
  }  

  void NFCGate::close_serial()
  {
     close(this->serial_port);
     this->serial_port = 0;
  }


void NFCGate::write_serial( string msg)
{
  if (this->serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
    }
  write(this->serial_port, &msg[0], msg.length());
}

string NFCGate::read_serial()
{
  char read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));
  int num_bytes; 

  do
  {
    num_bytes = read(this->serial_port, &read_buf, sizeof(read_buf));
    if( num_bytes >20)
    {
      RCLCPP_ERROR(this->get_logger(),"Error reading: %s", strerror(errno));
      return "false";
    }
    //RCLCPP_ERROR(this->get_logger(),"%i",num_bytes);
  }while(num_bytes<1);
  
  return string(read_buf, num_bytes);
}

string NFCGate::send_command(string command )
{
  this->write_serial(command+"\r");
  return this->read_serial();
}

void NFCGate::scanTag(const std::shared_ptr<GoalHandleAuthenticateUser> goal_handle) 
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<AuthenticateUser::Result>();
  auto reader_feedback = std::make_shared<robast_ros2_msgs::msg::NFCStatus>();
  string scaned_key; 


  this->open_serial();
 
  this->send_command(SET_SERIAL_TO_ASCII);
  bool validTagNotfound= true;
  int numReadings=0;
  while(validTagNotfound)
  {
    this->send_command(BOTTOM_LED_ON);
    this->send_command((TOP_LEDS_INIT(LED_RED)));
    this->send_command(TOP_LEDS_ON(LED_RED));
  
    string tag;
    //wait for the Tag and read TAG ID 
    do{ 
    tag= this->send_command(SEARCH_TAG);//search for a tag with the length of 10
    RCLCPP_INFO(this->get_logger(),"Received message: %s ", tag.c_str() );
    } while(tag.length() <10);
  
    RCLCPP_INFO(this->get_logger(),"READ");
    this->send_command(NFC_LOGIN_MC_STANDART("00"));
    scaned_key =this->send_command(NFC_READ_MC("02"));
    RCLCPP_INFO(this->get_logger(),"data on the Tag %s ", scaned_key.c_str());
    numReadings++;
    //show that aktion is done
    this->send_command(BEEP_STANDART); 

    validTagNotfound= std::find(std::begin(goal->permission_keys), std::end(goal->permission_keys), scaned_key) != std::end(goal->permission_keys);
    
    if(!validTagNotfound)
    {
      //feedback
      auto feedback = std::make_shared<AuthenticateUser::Feedback>();
      feedback->reader_status.is_completted=true;
      feedback->reader_status.unidentified_readings = numReadings;
      goal_handle->publish_feedback(feedback);
    }
  }
  this->send_command((TOP_LED_OFF(LED_RED)));
  this->send_command(BOTTOM_LED_OFF); 
  close(serial_port);

  // Check if goal is done
    if (rclcpp::ok()) 
    {
      result-> permission_key_used = scaned_key;
      result-> sucessful = true;
      result-> error_message = ""; 
      goal_handle->succeed(result);
    }
}

void NFCGate::writeTag(const std::shared_ptr<CreateUser::Request> request, std::shared_ptr<CreateUser::Response> response)
{
  this->open_serial();
 
  this->send_command(SET_SERIAL_TO_ASCII);
  int numReadings=0;
  
    this->send_command(BOTTOM_LED_ON);
    this->send_command((TOP_LEDS_INIT(LED_RED)));
    this->send_command(TOP_LEDS_ON(LED_RED));
  
    string tag;
    //wait for the Tag and read TAG ID 
    do{ 
    tag= this->send_command(SEARCH_TAG);//search for a tag with the length of 10
    RCLCPP_INFO(this->get_logger(),"Received message: %s ", tag.c_str() );
    } while(tag.length() <10);
  
    RCLCPP_INFO(this->get_logger(),"READ");
    this->send_command(NFC_LOGIN_MC_STANDART("00"));
    string bitsChanged =this->send_command(NFC_WRITE_MC("02",/*request->card_key*/"00000100000010000"));//ToDo dynamic key defination 
    RCLCPP_INFO(this->get_logger(),"data on the Tag %s ", bitsChanged.c_str());
    

  //show that aktion is done
  this->send_command(BEEP_STANDART);   
  this->send_command((TOP_LED_OFF(LED_RED)));
  this->send_command(BOTTOM_LED_OFF); 
  close(serial_port);

  response->sucessful= true;
  response-> card_id= tag;
  response->error_message;
}

}  // namespace robast_drawer_gate
