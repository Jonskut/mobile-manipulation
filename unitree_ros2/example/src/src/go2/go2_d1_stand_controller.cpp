#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_arm/msg/arm_string.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace {
// The Go2 LowCmd contains 12 leg motors. The D1 arm is controlled separately
// through ArmString commands and is not written into LowCmd here.
constexpr int kLegMotorCount = 12;
constexpr int kArmJointCount = 7;
constexpr double kControlPeriod = 0.002;
constexpr double kStandDuration = 3.0;
constexpr double kArmMoveDuration = 2.0;

// Leg joint targets are in radians and follow the Go2 motor order.
constexpr std::array<float, kLegMotorCount> kStandPose = {
    0.0F, 0.67F, -1.3F, 0.0F, 0.67F, -1.3F,
    0.0F, 0.67F, -1.3F, 0.0F, 0.67F, -1.3F};

// D1 ArmString angle fields use degrees. angle6 represents the gripper.
constexpr std::array<double, kArmJointCount> kArmHomeAngles = {
    0.0, -60.0, 60.0, 0.0, 30.0, 0.0, 0.0};

}  // namespace

class Go2D1StandController final : public rclcpp::Node {
 public:
  Go2D1StandController() : Node("go2_d1_stand_controller") {
    // ROS 2 topics are bridged to the corresponding Unitree DDS topics by the
    // active Unitree ROS 2 middleware configuration.
    arm_command_pub_ = create_publisher<unitree_arm::msg::ArmString>(
      "arm_Command", 10);
    // Enable the D1 arm before sending position commands.
    publish_arm_enable(true);

    low_cmd_pub_ = create_publisher<unitree_go::msg::LowCmd>("lowcmd", 10);
    low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
      "lowstate", 10,
        [this](const unitree_go::msg::LowState::SharedPtr msg) {
          // Do not command until the initial motor state is available.
          low_state_ = *msg;
          state_received_ = true;
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(2),
                               [this]() { write_command(); });
    initialize_command();
  }

 private:
  void initialize_command() {
    // Initialize unused motors to a neutral command. The control loop updates
    // only the 12 Go2 leg motors; the D1 arm uses ArmString below.
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = 0xFF;
    low_cmd_.gpio = 0;
    for (auto &motor : low_cmd_.motor_cmd) {
      motor.mode = 0x01;
      motor.q = PosStopF;
      motor.kp = 0.0F;
      motor.dq = VelStopF;
      motor.kd = 0.0F;
      motor.tau = 0.0F;
    }
  }

  static float interpolate(float start, float target, double progress) {
    return static_cast<float>((1.0 - progress) * start + progress * target);
  }

  void write_command() {
    if (!state_received_) {
      return;
    }

    if (!targets_initialized_) {
      // Capture the current pose so startup moves smoothly to the target pose.
      for (int index = 0; index < kLegMotorCount; ++index) {
        start_pose_[index] = low_state_.motor_state[index].q;
      }
      for (int index = 0; index < kArmJointCount - 1; ++index) {
        arm_start_angles_[index] =
            static_cast<double>(low_state_.motor_state[kLegMotorCount + index].q) *
            180.0 / M_PI;
      }
      arm_start_angles_[kArmJointCount - 1] = 0.0;
      targets_initialized_ = true;
    }

    const double elapsed = motion_time_;
    const double stand_progress = std::min(elapsed / kStandDuration, 1.0);
    const double arm_progress = std::min(elapsed / kArmMoveDuration, 1.0);

    for (int index = 0; index < kLegMotorCount; ++index) {
      auto &motor = low_cmd_.motor_cmd[index];
      motor.q = interpolate(start_pose_[index], kStandPose[index],
                            stand_progress);
      motor.dq = 0.0F;
      motor.kp = 60.0F;
      motor.kd = 5.0F;
      motor.tau = 0.0F;
    }

    // LowCmd requires a valid CRC before it is published.
    get_crc(low_cmd_);
    low_cmd_pub_->publish(low_cmd_);
    // The arm follows the same timed interpolation, but through the D1 SDK
    // command protocol rather than LowCmd.
    publish_arm_pose(arm_progress);
    motion_time_ += kControlPeriod;
  }

  void publish_arm_enable(bool enabled) {
    unitree_arm::msg::ArmString command;
    // funcode 5 is the D1 servo enable/disable command.
    command.data = std::string("{\"seq\":") + std::to_string(sequence_++) +
             ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":" +
             (enabled ? "1" : "0") + "}}";
    arm_command_pub_->publish(command);
  }

  void publish_arm_pose(double progress) {
    // funcode 2 commands all seven D1 angle fields in one message.
    std::ostringstream command_json;
    command_json << std::fixed << std::setprecision(3)
                 << "{\"seq\":" << sequence_++
                 << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":1";
    for (int index = 0; index < kArmJointCount; ++index) {
      const double angle = interpolate(
          static_cast<float>(arm_start_angles_[index]),
          static_cast<float>(kArmHomeAngles[index]), progress);
      command_json << ",\"angle" << index << "\":" << angle;
    }
    command_json << "}}";

    unitree_arm::msg::ArmString command;
    command.data = command_json.str();
    arm_command_pub_->publish(command);
  }

  unitree_go::msg::LowCmd low_cmd_;
  unitree_go::msg::LowState low_state_;
  std::array<float, kLegMotorCount> start_pose_{};
  std::array<double, kArmJointCount> arm_start_angles_{};
  rclcpp::Publisher<unitree_arm::msg::ArmString>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double motion_time_ = 0.0;
  bool state_received_ = false;
  bool targets_initialized_ = false;
  int sequence_ = 1;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2D1StandController>());
  rclcpp::shutdown();
  return 0;
}