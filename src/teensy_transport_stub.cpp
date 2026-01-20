/*
teensy_transport_stub =============================
Simulates transport layer between Jetson and Teensy.

INPUT:
  /teensy/wheels (pavbot_msgs/msg/WheelCmd)

OUTPUT:
  /teensy/transport_state (std_msgs/msg/String)
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "pavbot_msgs/msg/wheel_cmd.hpp"

#include <chrono>
#include <mutex>

class TeensyTransportStubNode : public rclcpp::Node {
private:
  rclcpp::Subscription<pavbot_msgs::msg::WheelCmd>::SharedPtr wheels_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int transport_timeout_ms_{250};
  double log_rate_hz_{10.0};

  std::mutex mtx_;
  rclcpp::Time last_rx_time_{};
  float last_left_{0.0f};
  float last_right_{0.0f};

  void on_wheels(const pavbot_msgs::msg::WheelCmd::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_left_ = msg->left;
    last_right_ = msg->right;
    last_rx_time_ = this->now();
  }

  void tick() {
    rclcpp::Time last;
    float left = 0.0f, right = 0.0f;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      last = last_rx_time_;
      left = last_left_;
      right = last_right_;
    }

    const bool never = (last.nanoseconds() == 0);

    double age_ms;
    if (never) age_ms = 1e9;
    else age_ms = (this->now() - last).nanoseconds() / 1e6;

    const bool timeout = (age_ms > transport_timeout_ms_);

    std_msgs::msg::String state;
    if (timeout || never) {
      state.data = "TIMEOUT_NEUTRAL (would send left=0 right=0 to Teensy)";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No recent /teensy/wheels age_ms=%.1f -> TIMEOUT_NEUTRAL", age_ms);
    } else {
      // In a real transport, you'd serialize left/right and write to serial here.
      state.data = "ACTIVE (would send wheels to Teensy): left=" + std::to_string(left) +
                   " right=" + std::to_string(right);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Transport ACTIVE age_ms=%.1f left=%.3f right=%.3f", age_ms, left, right);
    }

    state_pub_->publish(state);
  }

public:
  TeensyTransportStubNode() : Node("teensy_transport_stub") {
    transport_timeout_ms_ = this->declare_parameter<int>("transport_timeout_ms", 250);
    log_rate_hz_ = this->declare_parameter<double>("log_rate_hz", 10.0);

    state_pub_ = this->create_publisher<std_msgs::msg::String>("/teensy/transport_state", 10);

    wheels_sub_ = this->create_subscription<pavbot_msgs::msg::WheelCmd>(
      "/teensy/wheels", 10,
      std::bind(&TeensyTransportStubNode::on_wheels, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, log_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TeensyTransportStubNode::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "teensy_transport_stub started. timeout=%d ms log_rate=%.1f Hz (listening to /teensy/wheels)",
      transport_timeout_ms_, log_rate_hz_);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyTransportStubNode>());
  rclcpp::shutdown();
  return 0;
}
