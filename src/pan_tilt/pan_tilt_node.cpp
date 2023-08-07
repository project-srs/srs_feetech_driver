#include <feetech_ros/feetech_handler.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "pan_tilt_ros_if.hpp"

class PanTiltNode : public PanTiltRosIf
{
public:
  PanTiltNode(void) : PanTiltRosIf{}
  {
    printf("start feetech\n");
    parameters_ = getParameters();
    std::map<int, ServoConfig> config_list;
    config_list[parameters_.pan.id] = parameters_.pan.config;
    config_list[parameters_.tilt.id] = parameters_.tilt.config;
    bool open_success = feetech_handler_.Initialize(config_list);
    if (!open_success) {
      printf("fail to open serial\n");
      throw;
    }
  }

private:
  void onTimer() override
  {
    feetech_handler_.RequestStatus();
    auto pan_opt = feetech_handler_.GetStatus(parameters_.pan.id);
    auto tilt_opt = feetech_handler_.GetStatus(parameters_.tilt.id);

    if (pan_opt && tilt_opt) {
      SingleJointState pan_joint_state = getSingleJointState(pan_opt.value(), parameters_.pan);
      SingleJointState tilt_joint_state = getSingleJointState(tilt_opt.value(), parameters_.tilt);

      // output odometry
      publishOdometry(GenerateOdometry(pan_joint_state, tilt_joint_state));
      // output joint_state
      if (parameters_.publish_joint_state) {
        publishJointState(GenerateJointstate(pan_joint_state, tilt_joint_state));
      }
      // output TF
      if (parameters_.publish_tf) {
        broadcastTF(generateTransform(pan_joint_state, tilt_joint_state));
      }
    } else {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 1000, "request status fail");
    }
  }

  void onTwistReceived(const geometry_msgs::msg::TwistStamped::SharedPtr msg) override
  {
    setCommand(msg->twist.angular.z, parameters_.pan);
    setCommand(msg->twist.angular.y, parameters_.tilt);
  }

  void setCommand(const float value, const AxisParameters & param)
  {
    float tick_per_s = param.reverse ? -value * tick_per_rad_ : value * tick_per_rad_;
    if (std::abs(tick_per_s) < 5) {  // hold
      auto status_opt = feetech_handler_.GetStatus(param.id);
      if (status_opt) {
        feetech_handler_.SetCommand(param.id, status_opt.value().position, 1);
      } else {
        printf("id[%d] no status for hold\n", param.id);
      }
    } else {
      int temporary_position = 0 < tick_per_s ? 4096 : 0;
      feetech_handler_.SetCommand(param.id, temporary_position, std::abs(tick_per_s));
    }
    usleep(1000);  // guard wait
  }

  struct SingleJointState
  {
    std::string joint_name;
    float position{0.0f};
    float velocity{0.0f};
  };

  nav_msgs::msg::Odometry GenerateOdometry(
    const SingleJointState & pan_joint_status, const SingleJointState & tilt_joint_status) const
  {
    nav_msgs::msg::Odometry output;
    output.header.frame_id = parameters_.frame_id;
    output.header.stamp = now();
    // rpy -> quat
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0f, tilt_joint_status.position, pan_joint_status.position);
    output.pose.pose.orientation = tf2::toMsg(quat_tf);
    // rate
    output.twist.twist.linear.y = tilt_joint_status.velocity;
    output.twist.twist.linear.z = pan_joint_status.velocity;
    return output;
  }

  sensor_msgs::msg::JointState GenerateJointstate(
    const SingleJointState & pan_joint_status, const SingleJointState & tilt_joint_status) const
  {
    sensor_msgs::msg::JointState output;
    output.header.stamp = now();
    output.name.push_back(pan_joint_status.joint_name);
    output.position.push_back(pan_joint_status.position);
    output.velocity.push_back(pan_joint_status.velocity);
    output.name.push_back(tilt_joint_status.joint_name);
    output.position.push_back(tilt_joint_status.position);
    output.velocity.push_back(tilt_joint_status.velocity);
    return output;
  }

  geometry_msgs::msg::TransformStamped generateTransform(
    const SingleJointState & pan_joint_status, const SingleJointState & tilt_joint_status) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parameters_.frame_id;
    ;
    transform.child_frame_id = parameters_.child_frame_id;
    ;
    // rpy -> quat
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0f, tilt_joint_status.position, pan_joint_status.position);
    transform.transform.rotation = tf2::toMsg(quat_tf);
    return transform;
  }

  SingleJointState getSingleJointState(
    const ServoStatus & status, const AxisParameters & param) const
  {
    float scale = param.reverse ? -1.0f : 1.0f;
    SingleJointState output;
    output.joint_name = param.joint_name;
    output.position = scale * (status.position - center_tick_) / tick_per_rad_;
    output.velocity = scale * status.velocity / tick_per_rad_;
    return output;
  }

  FeetechHandler feetech_handler_;
  PanTiltParameters parameters_;
  static constexpr int center_tick_ = 2048;
  static constexpr int tick_per_rad_ = 651.9f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pan_tilt_node = std::make_shared<PanTiltNode>();
  rclcpp::spin(pan_tilt_node);
  rclcpp::shutdown();
  return 0;
}
