#include "robast_drawer_controller/drawer_controller.hpp"

namespace robast_drawer_controller
{
DrawerController::DrawerController() : Node("robast_drawer_controller")
{
  this->drawer_controller_server = rclcpp_action::create_server<ControlDrawer>(
    this,
    "control_drawer",
    std::bind(&DrawerController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DrawerController::cancel_callback, this, std::placeholders::_1),
    std::bind(&DrawerController::accepted_callback, this, std::placeholders::_1));
}

//TODO: Dekonstruktor

rclcpp_action::GoalResponse DrawerController::goal_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlDrawer::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  // (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DrawerController::cancel_callback(
  const std::shared_ptr<GoalHandleControlDrawer> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  // (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DrawerController::accepted_callback(const std::shared_ptr<GoalHandleControlDrawer> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DrawerController::open_drawer, this, std::placeholders::_1), goal_handle}.detach();
}

void DrawerController::open_drawer(const std::shared_ptr<GoalHandleControlDrawer> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  int serial_port = open("/dev/ttyACM0", O_RDWR);

  // Check for errors
  if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
  }

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
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
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return;
  }

  // Write to serial port
  unsigned char msg[] = { 'O', '\r' };
  write(serial_port, msg, sizeof(msg));

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [256];

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return;
  }

  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  printf("Read %i bytes. Received message: %s", num_bytes, read_buf);

  close(serial_port);
}




}  // namespace robast_drawer_controller
