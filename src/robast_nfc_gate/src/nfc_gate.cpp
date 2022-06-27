
#include "robast_nfc_gate/nfc_gate.hpp"

namespace robast_nfc_gate
{

NFCGate::NFCGate( ):NFCGate( "/dev/ttyACM0" ) { }

NFCGate::NFCGate( string serial_port_path ) : Node("robast_nfc_gate")
{
  RCLCPP_INFO(this->get_logger(), "constructor start"); 
  serial_path = serial_port_path;
  this->user_authenticate_server = rclcpp_action::create_server<AuthenticateUser>(
    this,
    "authenticate_user",
    bind(&NFCGate::goal_callback, this, placeholders::_1, placeholders::_2),
    bind(&NFCGate::cancel_callback, this, placeholders::_1),
    bind(&NFCGate::accepted_callback, this, placeholders::_1));

  this->scan();
  RCLCPP_INFO(this->get_logger(), "constructor done");
}


rclcpp_action::GoalResponse NFCGate::goal_callback( const rclcpp_action::GoalUUID & uuid, shared_ptr<const AuthenticateUser::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NFCGate::cancel_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  // (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NFCGate::accepted_callback(const shared_ptr<GoalHandleAuthenticateUser> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "scan task");
 //thread(&NFCGate::scan, goal_handle).detach();
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
  //unsigned char msg[] = { 'S', '4', '\r', 'O', '\r', 't', '0', '1', '4', '1', '1', '2', '2', '3', '3', '4', '4', '\r'};
  const char* msg_array =msg.c_str();
  write(this->serial_port, msg_array, sizeof(msg_array));
}

string NFCGate::read_serial()
{
  char read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));
  int num_bytes; 
  do{
     num_bytes = read(this->serial_port, &read_buf, sizeof(read_buf));
      if( num_bytes >20)
      {
        RCLCPP_ERROR(this->get_logger(),"Error reading: %s", strerror(errno));
        return "false";
      }
      RCLCPP_INFO(this->get_logger(),"Waiting");
  }while(num_bytes<1);

  string msg= string(read_buf, num_bytes);
   RCLCPP_INFO(this->get_logger(),"Received message: %s", msg.c_str());
  return msg;
}

string NFCGate::send_command(string command )
{

 
  RCLCPP_INFO(this->get_logger(),"set comunication to ASCII"); 
  this->write_serial(command);

  RCLCPP_INFO(this->get_logger(),"sent");
  string message=this->read_serial();
  RCLCPP_INFO(this->get_logger(),"Received message: %s", message.c_str());
  return message;
}

void NFCGate::scan() 
{ 
  RCLCPP_INFO(this->get_logger(),"start scan");
  this->open_serial();
  this->send_command("0408\r" );

  this->close_serial();
}




}  // namespace robast_drawer_gate
