#include <rclcpp/rclcpp.hpp>
#include "pavbot_msgs/msg/wheel_cmd.hpp"
#include "std_msgs/msg/string.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <string>
#include <algorithm>

class TeensySerialTransport : public rclcpp::Node {
public:
  TeensySerialTransport() : Node("teensy_serial_transport") {
    //PARAMETERIZE THESE
    port_     = this->declare_parameter<std::string>("port", "/dev/ttyACM1");
    baudrate_ = this->declare_parameter<int>("baudrate", 20);
    pwm_max_  = this->declare_parameter<int>("pwm_max", 255);

    open_serial();

    sub_ = this->create_subscription<pavbot_msgs::msg::WheelCmd>(
      "/teensy/wheels", 10,
      std::bind(&TeensySerialTransport::on_wheels, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/teensy/transport_state", 10);

    state_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TeensySerialTransport::publish_state, this));

    RCLCPP_INFO(this->get_logger(),
      "Teensy serial transport started on %s @ %d baud",
      port_.c_str(), baudrate_);
  }

  ~TeensySerialTransport() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void open_serial() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open serial port %s: %s",
                   port_.c_str(), strerror(errno));
      serial_ok_ = false;
      return;
    }

    termios tty{};
    tcgetattr(fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to configure serial port: %s",
                   strerror(errno));
      serial_ok_ = false;
      return;
    }

    serial_ok_ = true;
  }

  void on_wheels(const pavbot_msgs::msg::WheelCmd::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(),
                 "WheelCmd received: left=%.3f right=%.3f",
                 msg->left, msg->right);

    send_wheel(msg->left, msg->right);
  }

  void send_wheel(float left_norm, float right_norm) {
    int l_dir = (left_norm >= 0.0f) ? 1 : 0;
    int r_dir = (right_norm >= 0.0f) ? 1 : 0;

    int l_pwm = static_cast<int>(std::round(std::abs(left_norm) * pwm_max_));
    int r_pwm = static_cast<int>(std::round(std::abs(right_norm) * pwm_max_));

    l_pwm = std::clamp(l_pwm, 0, pwm_max_);
    r_pwm = std::clamp(r_pwm, 0, pwm_max_);

    char buf[64];
    int len = snprintf(buf, sizeof(buf),
                       "%d,%d,%d,%d\n",
                       l_dir, l_pwm, r_dir, r_pwm);

    RCLCPP_DEBUG(this->get_logger(),
                 "Sending serial: L(%d,%d) R(%d,%d)",
                 l_dir, l_pwm, r_dir, r_pwm);

    ssize_t written = write(fd_, buf, len);

    if (written == len) {
      serial_ok_ = true;
      RCLCPP_DEBUG(this->get_logger(),
                   "Serial write OK (%ld bytes)", written);
    } else {
      serial_ok_ = false;

      if (written < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Serial write failed: %s", strerror(errno));
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Partial serial write (%ld/%d bytes)",
                    written, len);
      }
    }
  }

  void publish_state() {
    std_msgs::msg::String msg;
    msg.data = serial_ok_ ? "SERIAL_OK" : "SERIAL_ERROR";
    state_pub_->publish(msg);
  }

  rclcpp::Subscription<pavbot_msgs::msg::WheelCmd>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::string port_;
  int baudrate_;
  int pwm_max_;
  int fd_{-1};
  bool serial_ok_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensySerialTransport>());
  rclcpp::shutdown();
  return 0;
}
