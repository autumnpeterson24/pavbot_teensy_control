/*
Takes standard robot velocity commands (cmd_vel) and converts them into normalized joystick style commands
* forward is a value between -1, and 1 to represent it as a joystick value
* turn is also a value between -1 and 1 to represent it as a joysitck value

INPUTS:
    /cmd_vel is a usual topic used in robotics to represent velocity commands for a robot in ROS (message type is geometry_msgs/msg/Twist)
        Uses:
            linear.x (forward/back velocity, m/s)
            angular.z (yaw rate for turning, rad/s)
OUTPUTS:
    Published topic:
            /teensy/axes (std_msgs/msg/String)
            ** should look something like this: forward=0.30 turn=-0.15 (src: linear.x=0.3 angular.z=-0.2) last_cmd_time_ns=...
                although you can change it based on how you want to format your logger :)

How it works rn:
--------------------------------
1. Declare parameters
    - max_v_mps: what you consider “full forward” for normalization

    - max_w_radps: what you consider “full turning” for normalization

    - deadzone: small values near zero get snapped to 0

    - axis_rate_hz: how often it publishes /teensy/axes

2. Subscribe to /cmd_vel
- Every time a Twist arrives, it stores:

    - the most recent linear.x and angular.z

    - the time it was received (last_cmd_time_)
        * A mutex protects these variables because a timer will read them concurrently! YAY REALTIME!!!! THANK YOU PROF. KING!

3. Use a timer to publish at a fixed rate
    - Even if /cmd_vel messages arrive irregularly it publishes axes at a steady rate (e.g., 20 Hz)

4. Normalize the values
    -  if linear.x = v, then:
        forward = clamp(v / max_v_mps, -1, +1)

    - if angular.z = w, then:
        turn = clamp(w / max_w_radps, -1, +1)

    - So, if max_v_mps=1.0 and you command linear.x=0.25, you get forward=0.25

5. Applies a deadzone
    - if |forward| < deadzone, set it to 0.
    * Same for turn
        * This mimics joystick behavior and removes jitter! :D No joycon drift here!

6. Publish a debug string
    - this is just to make learning easy. Later, you would likely switch /teensy/axes to a real message type (custom msg or Float32MultiArray) if you want


*/
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include "pavbot_teensy_control/axis_math.hpp"

#include <chrono>
#include <mutex>
#include <sstream>

using namespace std::chrono_literals;

class CmdVelToAxesNode : public rclcpp::Node
{
public:
  CmdVelToAxesNode()
  : Node("cmd_vel_to_axes")
  {
    max_v_mps = this->declare_parameter<double>("max_v_mps", 1.0); // max velocity for forward
    max_w_radps = this->declare_parameter<double>("max_w_radps", 1.5); // max velocity for turning
    deadzone = this->declare_parameter<double>("deadzone", 0.05); //deadzone so that really small values don't get drift and are just snapped to 0
    axis_rate_hz = this->declare_parameter<double>("axis_rate_hz", 20.0); // rate being published ~20Hz
    cmd_timeout_ms = this->declare_parameter<int>("cmd_timeout_ms", 250); // timeout for velocity commands (if you dont receive for more than 250 ms then take nerutral position)



    axes_pub = this->create_publisher<std_msgs::msg::String>("/teensy/axes", 10);

    cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelToAxesNode::on_cmd, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, axis_rate_hz));
    timer = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelToAxesNode::publish_axes, this));

    RCLCPP_INFO(this->get_logger(),
      "cmd_vel_to_axes started. max_v=%.2f m/s max_w=%.2f rad/s deadzone=%.3f rate=%.1f Hz",
      max_v_mps, max_w_radps, deadzone, axis_rate_hz);
  }

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx);
    last_cmd_time = this->now();
    last_v = msg->linear.x;
    last_w = msg->angular.z;
  }

  void publish_axes()
  {
    double v = 0.0, w = 0.0;
    rclcpp::Time t{};

    {
      std::lock_guard<std::mutex> lk(mtx);
      v = last_v;
      w = last_w;
      t = last_cmd_time;
    }

    // If /cmd_vel is stale or not received force neutral
        bool never = (t.nanoseconds() == 0);

        double age_ms;
        if(never) age_ms = 1e9;
        else age_ms = (this->now() - t).nanoseconds() / 1e6;

        bool cmd_timed_out = (age_ms > static_cast<double>(cmd_timeout_ms));

        if (cmd_timed_out) {
        v = 0.0;
        w = 0.0;
        }


    // normalize to [-1, 1] throug clamp() function
    //forward
    double forward, turn;
    if(max_v_mps > 1e-6) forward = clamp(v / max_v_mps, -1.0, 1.0); 
    else forward = 0.0;

    //turn
    if(max_w_radps > 1e-6) turn = clamp(w / max_w_radps, -1.0, 1.0); 
    else turn = 0.0;


    // Deadzone
    forward = apply_deadzone(forward, deadzone);
    turn= apply_deadzone(turn, deadzone);

    // keeping track of the messages we publish here for what they are and when they were last published
    std_msgs::msg::String out;
    std::ostringstream ss;
    ss << "forward=" << forward
        << " turn=" << turn
        << " (src: linear.x=" << v << " angular.z=" << w << ")"
        << " cmd_age_ms=" << age_ms
        << " cmd_timeout=" << (cmd_timed_out ? "true" : "false")
        << " last_cmd_time_ns=" << t.nanoseconds();
    out.data = ss.str();

    axes_pub->publish(out);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr axes_pub;
  rclcpp::TimerBase::SharedPtr timer;

  double max_v_mps{1.0};
  double max_w_radps{1.5};
  double deadzone{0.05};
  double axis_rate_hz{20.0};
  int cmd_timeout_ms{250};


  std::mutex mtx;
  rclcpp::Time last_cmd_time{};
  double last_v{0.0};
  double last_w{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToAxesNode>());
  rclcpp::shutdown();
  return 0;
}
