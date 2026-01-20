/*
cmd_vel_to_wheels =============================
Nav2-friendly mixer: converts /cmd_vel into differential wheel commands,
and publishes normalized left/right commands for Teensy transport.

INPUT:
  /cmd_vel (geometry_msgs/msg/Twist)
    linear.x  (m/s)
    angular.z (rad/s)

OUTPUT:
  /teensy/wheels (pavbot_msgs/msg/WheelCmd)
    left,right normalized in [-1,1]
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include "pavbot_teensy_control/axis_math.hpp"
#include "pavbot_msgs/msg/wheel_cmd.hpp"

#include <chrono>
#include <mutex>
#include <sstream>

class CmdVelToWheelsNode : public rclcpp::Node {
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<pavbot_msgs::msg::WheelCmd>::SharedPtr wheels_pub_;

  // Optional debug publisher (keep it if you like)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wheels_debug_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Params
  double track_width_m_{0.6};        // wheel-to-wheel distance (m)
  double max_wheel_speed_mps_{1.0};  // wheel speed corresponding to normalized 1.0
  double deadzone_{0.05};
  double publish_rate_hz_{20.0};
  int cmd_timeout_ms_{250};

  std::mutex mtx_;
  rclcpp::Time last_cmd_time_{};
  double last_v_{0.0};
  double last_w_{0.0};

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_cmd_time_ = this->now();
    last_v_ = msg->linear.x;
    last_w_ = msg->angular.z;
  }

  void publish_wheels() {
    double v = 0.0, w = 0.0;
    rclcpp::Time t{};

    {
      std::lock_guard<std::mutex> lk(mtx_);
      v = last_v_;
      w = last_w_;
      t = last_cmd_time_;
    }

    // Staleness handling
    const bool never = (t.nanoseconds() == 0);

    double age_ms;
    if (never) age_ms = 1e9;
    else age_ms = (this->now() - t).nanoseconds() / 1e6;

    const bool cmd_timed_out = (age_ms > static_cast<double>(cmd_timeout_ms_));
    if (cmd_timed_out) {
      v = 0.0;
      w = 0.0;
    }

    // Differential drive mixing (wheel linear speeds in m/s)
    const double half_T = 0.5 * track_width_m_;
    const double v_left_mps  = v - w * half_T;
    const double v_right_mps = v + w * half_T;

    // Normalize to [-1, 1]
    double left = 0.0, right = 0.0;
    if (max_wheel_speed_mps_ > 1e-6) {
      left  = clamp(v_left_mps  / max_wheel_speed_mps_, -1.0, 1.0);
      right = clamp(v_right_mps / max_wheel_speed_mps_, -1.0, 1.0);
    }

    // Deadzone (if it is close to 0 i.e. 0.023 whatever then you just clamp to 0)
    left = apply_deadzone(left, deadzone_);
    right = apply_deadzone(right, deadzone_);

    // Publish typed command
    pavbot_msgs::msg::WheelCmd cmd;
    cmd.stamp = this->now();
    cmd.left = static_cast<float>(left);
    cmd.right = static_cast<float>(right);
    wheels_pub_->publish(cmd);

    // Optional debug string (helpful while teammate is wiring things)
    if (wheels_debug_pub_) {
      std_msgs::msg::String out;
      std::ostringstream ss;
      ss << "left=" << left
         << " right=" << right
         << " (src: v=" << v << " w=" << w << ")"
         << " cmd_age_ms=" << age_ms
         << " cmd_timeout=" << (cmd_timed_out ? "true" : "false");
      out.data = ss.str();
      wheels_debug_pub_->publish(out);
    }
  }

public:
  CmdVelToWheelsNode() : Node("cmd_vel_to_wheels") {
    track_width_m_ = this->declare_parameter<double>("track_width_m", 0.6);
    max_wheel_speed_mps_ = this->declare_parameter<double>("max_wheel_speed_mps", 1.0);
    deadzone_ = this->declare_parameter<double>("deadzone", 0.05);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);
    cmd_timeout_ms_ = this->declare_parameter<int>("cmd_timeout_ms", 250);

    wheels_pub_ = this->create_publisher<pavbot_msgs::msg::WheelCmd>("/teensy/wheels", 10);

    // Optional debug topic
    wheels_debug_pub_ = this->create_publisher<std_msgs::msg::String>("/teensy/wheels_debug", 10);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelToWheelsNode::on_cmd, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelToWheelsNode::publish_wheels, this));

    RCLCPP_INFO(this->get_logger(),
      "cmd_vel_to_wheels started. track_width=%.3f m max_wheel=%.2f m/s deadzone=%.3f rate=%.1f Hz timeout=%d ms",
      track_width_m_, max_wheel_speed_mps_, deadzone_, publish_rate_hz_, cmd_timeout_ms_);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToWheelsNode>());
  rclcpp::shutdown();
  return 0;
}
