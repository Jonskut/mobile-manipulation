#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace {
constexpr int kLegMotorCount = 12;
constexpr int kMotorCount = 20;
constexpr double kControlPeriod = 0.002;
constexpr double kStandDuration = 3.0;
constexpr double kArmMoveDuration = 2.0;

constexpr std::array<float, kLegMotorCount> kStandPose = {
    0.0F, 0.67F, -1.3F, 0.0F, 0.67F, -1.3F,
    0.0F, 0.67F, -1.3F, 0.0F, 0.67F, -1.3F};

constexpr std::array<float, 8> kArmHomePose = {
    0.0F, -1.0471976F, 1.0471976F, 0.0F,
    0.5235988F, 0.0F, 0.0F, 0.0F};
}  // namespace

class Go2D1StandController final : public rclcpp::Node {
 public:
  Go2D1StandController() : Node("go2_d1_stand_controller") {
    low_cmd_pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        [this](const unitree_go::msg::LowState::SharedPtr msg) {
          low_state_ = *msg;
          state_received_ = true;
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(2),
                               [this]() { write_command(); });
    initialize_command();
  }

 private:
  void initialize_command() {
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
      for (int index = 0; index < kMotorCount; ++index) {
        start_pose_[index] = low_state_.motor_state[index].q;
      }
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

    for (int index = 0; index < 8; ++index) {
      auto &motor = low_cmd_.motor_cmd[kLegMotorCount + index];
      motor.q = interpolate(start_pose_[kLegMotorCount + index],
                            kArmHomePose[index], arm_progress);
      motor.dq = 0.0F;
      motor.kp = index < 6 ? 20.0F : 5.0F;
      motor.kd = index < 6 ? 1.5F : 0.5F;
      motor.tau = 0.0F;
    }

    get_crc(low_cmd_);
    low_cmd_pub_->publish(low_cmd_);
    motion_time_ += kControlPeriod;
  }

  unitree_go::msg::LowCmd low_cmd_;
  unitree_go::msg::LowState low_state_;
  std::array<float, kMotorCount> start_pose_{};
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double motion_time_ = 0.0;
  bool state_received_ = false;
  bool targets_initialized_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2D1StandController>());
  rclcpp::shutdown();
  return 0;
}