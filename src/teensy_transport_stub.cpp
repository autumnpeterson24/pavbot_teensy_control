/*
teensy_transport stub simulates what the trasnport layer looks like when the real teensy is added trhough a serial link
We need it to 1. receive axis commands, 2. detect command loss, 3. enforce a safe neutral output on timeout

INPUTS:
    Subscribed Topic:
        /teensy/axes (std_msgs/msg/String)

OUTPUTS:
    Published topic: /teensy/transport_state (std_msgs/msg/String)

    Two main states:
        ACTIVE: receiving recent axes, would forward them to Teensy
        TIMEOUT_NEUTRAL: commands are stale/missing, would send “neutral” (stop)

How it works rn:
---------------------------
1. Declare parameters:

    - transport_timeout_ms: maximum allowed time since last /teensy/axes

    - log_rate_hz: how often it checks/publishes state

2. Subscribe to /teensy/axes
    - When a message arrives, it stores:

        - last_axes (the string)

        - last_rx_time (time received)

3. Run a timer at fixed rate
    - On each tick it computes how old the last axes message is:

        * If no axes were ever received (last_rx_time_ == 0), treat as “never”
        * else: age_ms = (now - last_rx_time) in milliseconds

4. Timeout logic (deadman)

    - if age_ms > transport_timeout_ms (e.g., 250 ms), it declares timeout:

        - publishes TIMEOUT_NEUTRAL

    - logs a warning (throttled so it does not spam)

    *** Conceptually this is where a real node would send “neutral” commands to Teensy so the chair stops myabe even enable e-stop?

    - If not timed out:

        -publishes ACTIVE and includes the last axes string

        - logs an info message (throttled)

*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

class TeensyTransportStubNode : public rclcpp::Node
{
public:
  TeensyTransportStubNode()
  : Node("teensy_transport_stub")
  {
    transport_timeout_ms = this->declare_parameter<int>("transport_timeout_ms", 250);
    log_rate_hz = this->declare_parameter<double>("log_rate_hz", 10.0);

    state_pub = this->create_publisher<std_msgs::msg::String>("/teensy/transport_state", 10);

    axes_sub = this->create_subscription<std_msgs::msg::String>(
      "/teensy/axes", 10,
      std::bind(&TeensyTransportStubNode::on_axes, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, log_rate_hz));
    timer = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TeensyTransportStubNode::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "teensy_transport_stub started. timeout=%d ms log_rate=%.1f Hz",
      transport_timeout_ms , log_rate_hz);
  }

private:
  void on_axes(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx);
    last_axes = msg->data;
    last_rx_time = this->now();
  }

  void tick()
  {
    rclcpp::Time last;
    std::string axes;

    {
      std::lock_guard<std::mutex> lk(mtx);
      last = last_rx_time;
      axes = last_axes;
    }

    const bool never = (last.nanoseconds() == 0);

    double age_ms;
    if(never) age_ms = 1e9;
    else age_ms = (this->now() - last).nanoseconds() / 1e6;

    const bool timeout = (age_ms > transport_timeout_ms);

    std_msgs::msg::String state;
    if (timeout || never) {
      // What the real transport would do:
      // send "neutral" to Teensy (center stick) so the chair stops to emulate the joystick
      state.data = "TIMEOUT_NEUTRAL (would send neutral axes to Teensy)";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No recent /teensy/axes  age_ms=%.1f -> TIMEOUT_NEUTRAL", age_ms);
    } else {
      state.data = "ACTIVE (would send axes to Teensy): " + axes;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Transport ACTIVE age_ms=%.1f", age_ms);
    }

    state_pub->publish(state);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr axes_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
  rclcpp::TimerBase::SharedPtr timer;

  int transport_timeout_ms{250};
  double log_rate_hz{10.0};

  std::mutex mtx;
  rclcpp::Time last_rx_time{};
  std::string last_axes{"forward=0 turn=0"};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyTransportStubNode>());
  rclcpp::shutdown();
  return 0;
}
