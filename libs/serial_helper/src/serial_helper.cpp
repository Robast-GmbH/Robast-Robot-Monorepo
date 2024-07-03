#include "serial_helper/serial_helper.h"
#define DEBUG_LOG false
namespace serial_helper
{

  SerialHelper::SerialHelper(std::string serial_path)
  {
    this->serial_path_ = serial_path;
  }

  SerialHelper::~SerialHelper()
  {
  }

  std::string SerialHelper::open_serial()
  {
    this->serial_port_ = open(this->serial_path_.c_str(), O_RDWR);
    // Check for errors
    if (this->serial_port_ < 0)
    {
      std::string result = "Error from open: ";
      result.append(strerror(errno));
      result.append("\n");
      return result;
    }

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(this->serial_port_, &tty) != 0)
    {
      std::string result = "Error from tcgetattr: ";
      result.append(strerror(errno));
      result.append("\n");
      return result;
    }

    tty.c_cflag &= ~PARENB;          // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;          // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;           // Clear all bits that set the data size
    tty.c_cflag |= CS8;              // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;         // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;   // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                     // Disable echo
    tty.c_lflag &= ~ECHOE;                    // Disable erasure
    tty.c_lflag &= ~ECHONL;                   // Disable new-line echo
    tty.c_lflag &= ~ISIG;                     // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL);   // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;   // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;   // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 0.5;   // Wait for up to 0.05s (0.5 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port_, TCSANOW, &tty) != 0)
    {
      std::string result = "Error from tcgetattr: ";
      result.append(strerror(errno));
      result.append("\n");
      return result;
    }
    return "";
  }

  void SerialHelper::close_serial()
  {
    close(this->serial_port_);
    this->serial_port_ = -1;
  }

  uint16_t SerialHelper::read_serial(std::string* result, uint16_t max_num_bytes)
  {
    char read_buf[256];
    memset(&read_buf, '\0', sizeof(read_buf));
    uint16_t num_bytes;
    num_bytes = read(this->serial_port_, &read_buf, sizeof(read_buf));
    if (num_bytes > max_num_bytes)
    {
      *result = "Error reading: ";
      result->append(strerror(errno));
      result->append("/n");
      return 0;
    }
    *result = std::string(read_buf, num_bytes);
    if (DEBUG_LOG)
    {
      std::ofstream logfile;
      logfile.open("Communication.log", std::ios_base::app);
      logfile << "recived: " + *result << std::endl;
      logfile.close();
    }
    return num_bytes;
  }

  std::string SerialHelper::write_serial(std::string msg)
  {
    if (this->serial_port_ < 0)
    {
      std::string errno_string(errno, sizeof(errno));
      return "Error " + errno_string + " from open: " + strerror(errno) + "\n";
    }

    if (DEBUG_LOG)
    {
      std::ofstream logfile;
      logfile.open("Communication.log", std::ios_base::app);
      logfile << "send: " + msg << std::endl;
      logfile.close();
    }

    write(this->serial_port_, &msg[0], msg.length());
    return "";
  }

  std::string SerialHelper::send_ascii_cmd(std::string cmd)
  {
    return this->write_serial(cmd + "\r");
  }

  std::string SerialHelper::ascii_interaction(std::string cmd, std::string* response, uint16_t response_size)
  {
    this->read_serial(response, response_size);
    std::string request_result = this->send_ascii_cmd(cmd);
    if (request_result != "")
    {
      return request_result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    this->read_serial(response, response_size);
    *response = std::regex_replace(*response, std::regex("\r"), "");
    return *response;
  }
}   // namespace serial_helper