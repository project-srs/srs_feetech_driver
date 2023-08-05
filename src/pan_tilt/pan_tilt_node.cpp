#include <feetech_ros/feetech_handler.hpp>
#include "pan_tilt_ros_if.hpp"

class PanTiltNode : public PanTiltRosIf
{
public:
  PanTiltNode(void) : PanTiltRosIf{}
  {
    printf("start feetech\n");
    std::map<int,ServoConfig> config_list;
    config_list[1] = {1024, 3072};
    config_list[2] = {1024, 2400};
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
    
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();

    auto s1_opt = feetech_handler_.GetStatus(1);
    if (s1_opt) {
      auto& status = s1_opt.value();
      joint_state.name.push_back("turret_pan_joint");
      joint_state.position.push_back(((float)(status.position - center_tick_)) / tick_per_rad_);
      joint_state.velocity.push_back((float)status.velocity / tick_per_rad_);
    }
    auto s2_opt = feetech_handler_.GetStatus(2);
    if (s2_opt) {
      auto& status = s2_opt.value();
      joint_state.name.push_back("turret_tilt_joint");
      joint_state.position.push_back(((float)(status.position - center_tick_)) / tick_per_rad_);
      joint_state.velocity.push_back((float)status.velocity / tick_per_rad_);
    }
    publishJointState(joint_state);
  }

  void onTwistReceived(const geometry_msgs::msg::TwistStamped::SharedPtr msg) override {
    setCommand(1, msg->twist.angular.z);
    setCommand(2, msg->twist.angular.y);
  }

  void setCommand(const int id, const float value) {
    float tick_per_s = value * tick_per_rad_;
    if (std::abs(tick_per_s) < 5) { // hold
      auto status_opt = feetech_handler_.GetStatus(id);
      if (status_opt) {
        feetech_handler_.SetCommand(id, status_opt.value().position, 1);
      } else {
        printf("id[%d] no status for hold\n", id);
      }
    } else {
      int temporary_position = 0 < tick_per_s ? 4096 : 0;
      feetech_handler_.SetCommand(id, temporary_position, std::abs(tick_per_s));
    }
    usleep(1000); // guard wait
  }

private:
  FeetechHandler feetech_handler_;
  static constexpr int center_tick_ = 2048;
  static constexpr int tick_per_rad_ = 651.9f;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto pan_tilt_node = std::make_shared<PanTiltNode>();
  rclcpp::spin(pan_tilt_node);
  rclcpp::shutdown();
  return 0;
}
