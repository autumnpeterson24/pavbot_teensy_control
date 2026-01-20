/*
teensy_transport_stub =============================
Simulates transport layer between Jetson and Teensy.
Will likely need an actual teensy_transport_serial.cpp node this is just a stub :)

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
  rclcpp::Subscription<pavbot_msgs::msg::WheelCmd>::SharedPtr wheels_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
  rclcpp::TimerBase::SharedPtr timer;

  int transport_timeout_ms{250};
  double log_rate_hz{10.0};

  std::mutex mtx;
  rclcpp::Time last_rx_time{};
  float last_left{0.0f};
  float last_right{0.0f};

  void on_wheels(const pavbot_msgs::msg::WheelCmd::SharedPtr msg) {
    /*
    Receives the latest left/right wheel command and records it (with a timestamp) so the transport can decide whether to forward it to the Teensy or force neutral if commands go stale
    */
    std::lock_guard<std::mutex> lk(mtx);
    last_left = msg->left;
    last_right = msg->right;
    last_rx_time = this->now();
  }

  void tick() {
    /*
    Runs at a fixed rate to decide whether the latest wheel command is still valid, and either forwards it to the Teensy or forces a neutral (stop) command.
    */
    rclcpp::Time last;
    float left = 0.0f, right = 0.0f;

    {
      std::lock_guard<std::mutex> lk(mtx);
      last = last_rx_time;
      left = last_left;
      right = last_right;
    }

    const bool never = (last.nanoseconds() == 0);

    double age_ms;
    if (never) age_ms = 1e9;
    else age_ms = (this->now() - last).nanoseconds() / 1e6;

    const bool timeout = (age_ms > transport_timeout_ms);

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

    state_pub->publish(state);
  }

public:
  TeensyTransportStubNode() : Node("teensy_transport_stub") {
    /* 
    Sets up the transport node so it can receive wheel commands, monitor their freshness, and periodically decide whether to forward them or force neutral
    */
    transport_timeout_ms = this->declare_parameter<int>("transport_timeout_ms", 250);
    log_rate_hz = this->declare_parameter<double>("log_rate_hz", 10.0);

    state_pub = this->create_publisher<std_msgs::msg::String>("/teensy/transport_state", 10);

    wheels_sub = this->create_subscription<pavbot_msgs::msg::WheelCmd>(
      "/teensy/wheels", 10,
      std::bind(&TeensyTransportStubNode::on_wheels, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, log_rate_hz));
    timer = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TeensyTransportStubNode::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "teensy_transport_stub started. timeout=%d ms log_rate=%.1f Hz (listening to /teensy/wheels)",
      transport_timeout_ms, log_rate_hz);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyTransportStubNode>());
  rclcpp::shutdown();
  return 0;
}
