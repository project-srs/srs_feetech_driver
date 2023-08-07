#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders; // NOLINT

class JoyToCmdRate : public rclcpp::Node
{
public:
  JoyToCmdRate() : Node("pan_tilt_node")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_rate", 1);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
        std::bind(&JoyToCmdRate::onJoyReceived, this, std::placeholders::_1));
  }

protected:
  void onJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (2 <= msg->axes.size()) {
      geometry_msgs::msg::TwistStamped twist;
      twist.header = msg->header;
      twist.twist.angular.y = -0.5f * msg->axes[0];
      twist.twist.angular.z = 0.5f * msg->axes[1];
      twist_pub_->publish(twist);
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto joy_to_cmd_rate = std::make_shared<JoyToCmdRate>();
  rclcpp::spin(joy_to_cmd_rate);
  rclcpp::shutdown();
  return 0;
}
