#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
using namespace std::chrono_literals;
using namespace std::placeholders; // NOLINT

struct AxisParameters{
  int id{};
  ServoConfig config{};
  std::string joint_name{};
  bool reverse{};
};

struct PanTiltParameters{
  // common
  std::string frame_id{};
  std::string child_frame_id{};
  bool publish_tf{};
  bool publish_joint_state{};
  // pan
  AxisParameters pan;
  // tilt
  AxisParameters tilt;
};

class PanTiltRosIf : public rclcpp::Node
{
public:
  PanTiltRosIf() : Node("pan_tilt_node")
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_rate", 10,
        std::bind(&PanTiltRosIf::onTwistReceived, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(50ms, std::bind(&PanTiltRosIf::onTimer, this));

    // common parameters
    this->declare_parameter<std::string>("frame_id", "turret_base_link");
    this->declare_parameter<std::string>("child_frame_id", "turret_tip_link");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("publish_joint_state", false);
    // pan parameters
    this->declare_parameter<int>("pan.id", 1);
    this->declare_parameter<int>("pan.min_tick", 1024);
    this->declare_parameter<int>("pan.max_tick", 3072);
    this->declare_parameter<std::string>("pan.joint_name", "turret_pan_joint");
    this->declare_parameter<bool>("pan.reverse", false);
    // tilt parameters
    this->declare_parameter<int>("tilt.id", 2);
    this->declare_parameter<int>("tilt.min_tick", 1024);
    this->declare_parameter<int>("tilt.max_tick", 3072);
    this->declare_parameter<std::string>("tilt.joint_name", "turret_tilt_joint");
    this->declare_parameter<bool>("tilt.reverse", false);
  }

  PanTiltParameters getParameters(void) {
    PanTiltParameters output{};
    // common
    this->get_parameter("frame_id", output.frame_id);
    this->get_parameter("child_frame_id", output.child_frame_id);
    this->get_parameter("publish_tf", output.publish_tf);
    this->get_parameter("publish_joint_state", output.publish_joint_state);
    // pan
    this->get_parameter("pan.id", output.pan.id);
    this->get_parameter("pan.min_tick", output.pan.config.min_tick);
    this->get_parameter("pan.max_tick", output.pan.config.max_tick);
    this->get_parameter("pan.joint_name", output.pan.joint_name);
    this->get_parameter("pan.reverse", output.pan.reverse);
    // tilt
    this->get_parameter("tilt.id", output.tilt.id);
    this->get_parameter("tilt.min_tick", output.tilt.config.min_tick);
    this->get_parameter("tilt.max_tick", output.tilt.config.max_tick);
    this->get_parameter("tilt.joint_name", output.tilt.joint_name);
    this->get_parameter("tilt.reverse", output.tilt.reverse);
    return output;
  }

protected:
  virtual void onTimer() = 0;
  virtual void onTwistReceived(const geometry_msgs::msg::TwistStamped::SharedPtr msg) = 0;

  void publishOdometry(const nav_msgs::msg::Odometry &msg)
  {
    odom_pub_->publish(msg);
  }

  void publishJointState(const sensor_msgs::msg::JointState &msg)
  {
    joint_state_pub_->publish(msg);
  }

  void broadcastTF(const geometry_msgs::msg::TransformStamped& msg)
  {
    tf_broadcaster_->sendTransform(msg);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
};
