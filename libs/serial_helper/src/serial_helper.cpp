#include "../include/serial_helper/serial_helper.h"

namespace serial_helper
{

    SerialHelper::SerialHelper(string serial_path)
    {
        this->serial_path_ = serial_path;
    }
    
    SerialHelper::~SerialHelper()
    {
    }

    string SerialHelper::open_serial()
    {
        this->serial_port_ = open(this->serial_path_.c_str() , O_RDWR);
        // Check for errors
        if (this->serial_port_ < 0) {
            string result = "Error from open: ";
            result.append(strerror(errno));
            result.append("\n");
            return result;
        }

        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;

        // Read in existing settings, and handle any error
        if(tcgetattr(this->serial_port_, &tty) != 0) {
            string result = "Error from tcgetattr: ";
            result.append(strerror(errno));
            result.append("\n");
            return result;
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
        if (tcsetattr(this->serial_port_, TCSANOW, &tty) != 0) {
            string result = "Error from tcgetattr: ";
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

    uint16_t SerialHelper::read_serial(string* result, uint16_t max_num_bytes)
    {
        char read_buf [256];
        memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes;
        num_bytes = read(this->serial_port_, &read_buf, sizeof(read_buf));
        if( num_bytes > max_num_bytes)
        {
            *result = "Error reading: ";
            result->append(strerror(errno));
            result->append("/n");
            return 0;
        }
        *result = string(read_buf, num_bytes);
        //ofstream logfile;
        // logfile.open ("Read.log", std::ios_base::app);
        // logfile << *result << endl;
        // logfile.close();
        return num_bytes;
    }

    string SerialHelper::write_serial(string msg)
    { 
       
       
        
        if (this->serial_port_ < 0)
        {
            string errno_string(errno, sizeof(errno));
            return "Error " + errno_string + " from open: " + strerror(errno) + "\n";

        }   
        //ofstream logfile;
        // logfile.open ("Write.log", std::ios_base::app);
        // logfile << msg << endl;
        // logfile.close();
        write(this->serial_port_, &msg[0], msg.length());

        return "";
    }

    string SerialHelper::send_ascii_cmd(string cmd)
    {
       
        return this->write_serial(cmd + "\r");
    }

    string SerialHelper::ascii_interaction(string cmd, string* responce, uint16_t responce_size )
    {
        string request_result = this->send_ascii_cmd(cmd);
        if(request_result!="")
        {
            return request_result;
        }

        this->read_serial(responce, responce_size); 
        (*responce).pop_back(); // remove '/r'
        return *responce;
    }
}