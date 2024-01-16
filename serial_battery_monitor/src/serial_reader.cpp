#include <stdio.h>
#include <string.h>
#include <signal.h>

#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "control_msg/msg/battery_msg.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

int serial_port;

class BatterySerial : public rclcpp::Node
{
  public:
    BatterySerial()
    : Node("battery_node"), count_(0)
    {
      publisher_ = this->create_publisher<control_msg::msg::BatteryMsg>("battery_percentage", 10);

      int init_result = initialize_serial();
      if (init_result != 0){
        RCLCPP_INFO(this->get_logger(), "Error occured when initializing serial port");
      }

      timer_ = this->create_wall_timer(
      100ms, std::bind(&BatterySerial::timer_callback, this));
    }

  //TODO clear memory
  private:
    int initialize_serial()
    {
      serial_port = open("/dev/ttyACM0", O_RDWR);
      struct termios tty;

      if(tcgetattr(serial_port, &tty) != 0){
        RCLCPP_INFO(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
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

        tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            RCLCPP_INFO(this->get_logger(),"Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return 1;
        }

        RCLCPP_INFO(this->get_logger(),"Flushing Buffer");
        tcflush(serial_port, TCIFLUSH);
        
        std::this_thread::sleep_for(1s);


        return 0;

        
    }

    // Callback
    void timer_callback()
    {
      auto message = control_msg::msg::BatteryMsg();

      char temp_buffer[8];
      int num_of_bytes = read(serial_port, &temp_buffer, sizeof(temp_buffer));
      if (num_of_bytes == 0){;
      }
      else if (num_of_bytes > 0){
        strcpy(serial_buffer, temp_buffer);
      }
      else{
        RCLCPP_INFO(this->get_logger(), "ERROR: Error occured in reading from serial buffer.");
      }

      if (serial_buffer[0] == 'E'){
        RCLCPP_INFO(this->get_logger(), "ERROR: Error code %c.", serial_buffer[1]);
        message.soc = 999;
        message.soh = serial_buffer[1] - '0';
      }
      else{
        std::string _buffer = serial_buffer;
        std::string SOC = _buffer.substr(0, 3);
        std::string SOH = _buffer.substr(3, 3);

        message.soc = std::stoi(SOC);
        message.soh = std::stoi(SOH);

        RCLCPP_INFO(this->get_logger(), "New data read: '%i', '%i'", std::stoi(SOH), std::stoi(SOH));
      }
      publisher_->publish(message);
    }

    char serial_buffer[8];
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<control_msg::msg::BatteryMsg>::SharedPtr publisher_;
    size_t count_;
};

void sig_handler(int signum){
  std::cout << "Handling signum: " << signum << std::endl;
  close(serial_port);
  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  signal(SIGINT, sig_handler);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatterySerial>());
  rclcpp::shutdown();
  return 0;
}