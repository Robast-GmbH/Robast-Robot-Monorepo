#include "robast_drawer_gate/drawer_gate.hpp"

namespace robast_drawer_gate
{
  DrawerGate::DrawerGate() : Node("robast_drawer_gate")
  {
    this->drawer_gate_server = rclcpp_action::create_server<DrawerUserAccess>(
      this,
      "control_drawer",
      std::bind(&DrawerGate::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrawerGate::cancel_callback, this, std::placeholders::_1),
      std::bind(&DrawerGate::accepted_callback, this, std::placeholders::_1));

      this->timer_cb_group_ = nullptr; //This might be replaced in the future to better use callback groups. With the default setting above (nullptr / None), the timer will use the node’s default Mutually Exclusive Callback Group.
      this->timer_ptr_ = this->create_wall_timer(1s, std::bind(&DrawerGate::timer_callback, this), timer_cb_group_);

      this->setup_serial_port();
      set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps);
      // When the USB-CAN Adapter isn't sending CAN messages, the default state should be the listen only mode to enable receiving CAN messages
      open_can_channel_listen_only_mode(); 
      close(serial_port);

    //TODO: Timer callback, der regelmäßig aufgerufen wird und CAN Messages EINLIEST.
    //TODO: Das sollte aber niemals gleichzeitig zum Abschicken von CAN Messages passieren, daher beide in eine Mutually Exclusive Callback Group packen!
  }

  //TODO: Dekonstruktor

  rclcpp_action::GoalResponse DrawerGate::goal_callback(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DrawerUserAccess::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    // (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DrawerGate::cancel_callback(
    const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DrawerGate::accepted_callback(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DrawerGate::open_drawer, this, std::placeholders::_1), goal_handle}.detach();
  }

  void DrawerGate::timer_callback(void)
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggert!");
    this->setup_serial_port();
    set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps);
    open_can_channel_listen_only_mode(); 
    std::string serial_read = this->read_serial();
    RCLCPP_INFO(this->get_logger(), "Read from serial: %s", serial_read);
    close(serial_port);
  }

  void DrawerGate::setup_serial_port(void)
  {
    RCLCPP_INFO(this->get_logger(), "Setting up serial port!");

    serial_port = open("/dev/ttyACM0", O_RDWR);

    // Check for errors
    if (serial_port < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from opening serial Port!");
      printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0)
    {
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

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, 115200);
    cfsetospeed(&tty, 115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return;
    }
  }
  
  std::string DrawerGate::read_serial()
  {
    RCLCPP_INFO(this->get_logger(), "Reading serial now!");
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
      RCLCPP_INFO(this->get_logger(),"Number of bytes read from serial: %i",num_bytes);
    }while(num_bytes<1);
    
    return std::string(read_buf, num_bytes);
  }

  robast_can_msgs::CanMessage DrawerGate::create_can_msg_drawer_user_access(std::shared_ptr<const DrawerUserAccess::Goal> goal, led_parameters led_parameters)
  {
    robast_can_msgs::CanMessage can_msg_drawer_user_access = can_db.can_messages.at(CAN_MSG_DRAWER_USER_ACCESS);

    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_DRAWER_CONTROLLER_ID).data = goal->drawer_controller_id;

    if (goal->drawer_id == 1) 
    {
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_1).data = CAN_DATA_OPEN_LOCK;
    }
    if (goal->drawer_id == 2)
    {
      can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_OPEN_LOCK_2).data = CAN_DATA_OPEN_LOCK;
    }

    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_RED).data = led_parameters.led_red;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_GREEN).data = led_parameters.led_green;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_BLUE).data = led_parameters.led_blue;
    can_msg_drawer_user_access.can_signals.at(CAN_SIGNAL_LED_BRIGHTNESS).data = led_parameters.brightness;

    return can_msg_drawer_user_access;
  }

  void DrawerGate::set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface can_baudrate)
  {
    unsigned char msg[3];

    switch (can_baudrate)
    {
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_10kbps:
        msg[0] = 'S';
        msg[1] = '0';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_20kbps:
        msg[0] = 'S';
        msg[1] = '1';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_50kbps:
        msg[0] = 'S';
        msg[1] = '2';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_100kbps:
        msg[0] = 'S';
        msg[1] = '3';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_125kbps:
        msg[0] = 'S';
        msg[1] = '4';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps:
        msg[0] = 'S';
        msg[1] = '5';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_500kbps:
        msg[0] = 'S';
        msg[1] = '6';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_800kbps:
        msg[0] = 'S';
        msg[1] = '7';
        break;
      case robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_1000kbps:
        msg[0] = 'S';
        msg[1] = '8';
        break;
      default:
        msg[0] = 'S';
        msg[1] = '6';
        break;
    }

    msg[2] = '\r';

    write(serial_port, msg, sizeof(msg)); // Write to serial port
  }

  void DrawerGate::open_can_channel(void)
  {
    unsigned char msg[2] = {'L', '\r'};

    write(serial_port, msg, sizeof(msg));
  }

  void DrawerGate::open_can_channel_listen_only_mode(void)
  {
    unsigned char msg[2] = {'O', '\r'};

    write(serial_port, msg, sizeof(msg));
  }

  void DrawerGate::close_can_channel(void)
  {
    unsigned char msg[2] = {'C', '\r'};

    write(serial_port, msg, sizeof(msg));
  }
  
  void DrawerGate::open_drawer(const std::shared_ptr<GoalHandleDrawerUserAccess> goal_handle) 
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal"); // DEBUGGING

    DrawerGate::setup_serial_port();

    // For DEBUGGING purposes this is the action send_goal command:
    // ros2 action send_goal /control_drawer robast_ros2_msgs/action/DrawerUserAccess "{drawer_controller_id: 1, drawer_id: 1}"

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DrawerUserAccess::Feedback>();
    auto result = std::make_shared<DrawerUserAccess::Result>();

    led_parameters led_parameters = {};
    led_parameters.led_red = 100;
    led_parameters.led_blue = 0;
    led_parameters.led_green = 50;
    led_parameters.brightness = 20;
    led_parameters.mode = 0;

    robast_can_msgs::CanMessage can_msg_drawer_user_access = DrawerGate::create_can_msg_drawer_user_access(goal, led_parameters);
    
    set_can_baudrate(robast_can_msgs::can_baudrate_usb_to_can_interface::can_baud_250kbps);

    open_can_channel();

    std::optional<std::string> ascii_cmd_drawer_user_access = robast_can_msgs::encode_can_message_into_ascii_command(can_msg_drawer_user_access, can_db.can_messages);
    
    if (ascii_cmd_drawer_user_access.has_value())
    {
      write(serial_port, &ascii_cmd_drawer_user_access.value()[0], ascii_cmd_drawer_user_access.value().length());
    }

    // When the USB-CAN Adapter isn't sending CAN messages, the default state should be the listen only mode to enable receiving CAN messages
    open_can_channel_listen_only_mode(); 

    close(serial_port);

    RCLCPP_INFO(this->get_logger(), "Finished executing goal"); // DEBUGGING
  }
}  // namespace robast_drawer_gate
