#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"

namespace {
constexpr int kLegMotorCount = 12;
constexpr int kArmJointCount = 7;
constexpr double kControlPeriod = 0.002;
constexpr double kStandDuration = 3.0;
constexpr double kArmMoveDuration = 2.0;
constexpr double kRadiansToDegrees = 57.29577951308232;
constexpr float kPosStop = 2.146E+9F;
constexpr float kVelStop = 16000.0F;

constexpr std::array<float, kLegMotorCount> kStandPose = {
    0.0F, 0.67F, -1.3F, 0.0F, 0.67F, -1.3F,
    0.0F, 0.67F, -1.3F, 0.0F, 0.67F, -1.3F};
constexpr std::array<double, kArmJointCount> kArmHomeAngles = {
    0.0, -60.0, 60.0, 0.0, 30.0, 0.0, 0.0};

uint32_t crc32_core(uint32_t *ptr, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;
  for (uint32_t index = 0; index < length; ++index) {
    uint32_t bit = 1U << 31;
    uint32_t data = ptr[index];
    for (uint32_t count = 0; count < 32; ++count) {
      crc = (crc & 0x80000000U) ? (crc << 1) ^ polynomial : crc << 1;
      if (data & bit) crc ^= polynomial;
      bit >>= 1;
    }
  }
  return crc;
}

float interpolate(float start, float target, double progress) {
  return static_cast<float>((1.0 - progress) * start + progress * target);
}
}  // namespace

class Go2D1StandController final {
 public:
  Go2D1StandController(int domain_id, const std::string &interface_name) {
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, interface_name);

    low_cmd_pub_ = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
        "rt/lowcmd");
    low_cmd_pub_->InitChannel();
    low_state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
        "rt/lowstate");
    low_state_sub_->InitChannel(
        [this](const void *message) { handleLowState(message); }, 1);

    arm_command_pub_ = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(
        "rt/arm_Command");
    arm_command_pub_->InitChannel();
    arm_state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(
        "current_servo_angle");
    arm_state_sub_->InitChannel(
        [this](const void *message) { handleArmState(message); }, 1);

    initialize_command();
  }

  void run() {
    while (!low_state_received_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    initialize_targets();
    publish_arm_enable(true);
    while (running_) {
      write_command();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

 private:
  void initialize_command() {
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;
    for (auto &motor : low_cmd_.motor_cmd()) {
      motor.mode() = 0x01;
      motor.q() = kPosStop;
      motor.kp() = 0.0F;
      motor.dq() = kVelStop;
      motor.kd() = 0.0F;
      motor.tau() = 0.0F;
    }
  }

  void handleLowState(const void *message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
    low_state_received_ = true;
  }

  void handleArmState(const void *message) {
    const auto &state =
        *static_cast<const unitree_arm::msg::dds_::PubServoInfo_ *>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    arm_start_angles_ = {state.servo0_data_(), state.servo1_data_(),
                         state.servo2_data_(), state.servo3_data_(),
                         state.servo4_data_(), state.servo5_data_(),
                         state.servo6_data_()};
    arm_state_received_ = true;
  }

  void initialize_targets() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int index = 0; index < kLegMotorCount; ++index) {
      start_pose_[index] = low_state_.motor_state()[index].q();
    }
    if (!arm_state_received_) {
      for (int index = 0; index < kArmJointCount - 1; ++index) {
        arm_start_angles_[index] =
            low_state_.motor_state()[kLegMotorCount + index].q() *
            kRadiansToDegrees;
      }
      arm_start_angles_[kArmJointCount - 1] = 0.0;
    }
  }

  void write_command() {
    const double elapsed = motion_time_;
    const double stand_progress = std::min(elapsed / kStandDuration, 1.0);
    const double arm_progress = std::min(elapsed / kArmMoveDuration, 1.0);
    for (int index = 0; index < kLegMotorCount; ++index) {
      auto &motor = low_cmd_.motor_cmd()[index];
      motor.q() = interpolate(start_pose_[index], kStandPose[index], stand_progress);
      motor.dq() = 0.0F;
      motor.kp() = 60.0F;
      motor.kd() = 5.0F;
      motor.tau() = 0.0F;
    }
    low_cmd_.crc() = crc32_core(
        reinterpret_cast<uint32_t *>(&low_cmd_),
        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    low_cmd_pub_->Write(low_cmd_);
    publish_arm_pose(arm_progress);
    motion_time_ += kControlPeriod;
  }

  void publish_arm_enable(bool enabled) {
    unitree_arm::msg::dds_::ArmString_ command;
    command.data_(std::string("{\"seq\":") + std::to_string(sequence_++) +
                  ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":" +
                  (enabled ? "1" : "0") + "}}");
    arm_command_pub_->Write(command);
  }

  void publish_arm_pose(double progress) {
    std::ostringstream json;
    json << std::fixed << std::setprecision(3)
         << "{\"seq\":" << sequence_++
         << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":1";
    for (int index = 0; index < kArmJointCount; ++index) {
      json << ",\"angle" << index << "\":"
           << interpolate(static_cast<float>(arm_start_angles_[index]),
                          static_cast<float>(kArmHomeAngles[index]), progress);
    }
    json << "}}";
    unitree_arm::msg::dds_::ArmString_ command;
    command.data_(json.str());
    arm_command_pub_->Write(command);
  }

  unitree_go::msg::dds_::LowCmd_ low_cmd_;
  unitree_go::msg::dds_::LowState_ low_state_;
  std::array<float, kLegMotorCount> start_pose_{};
  std::array<double, kArmJointCount> arm_start_angles_{};
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_state_sub_;
  unitree::robot::ChannelPublisherPtr<unitree_arm::msg::dds_::ArmString_> arm_command_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_arm::msg::dds_::PubServoInfo_> arm_state_sub_;
  std::mutex state_mutex_;
  std::atomic_bool running_{true};
  std::atomic_bool low_state_received_{false};
  bool arm_state_received_ = false;
  double motion_time_ = 0.0;
  int sequence_ = 1;
};

int main(int argc, char **argv) {
  const int domain_id = argc < 2 ? 1 : 0;
  const std::string interface_name = argc < 2 ? "lo" : argv[1];
  Go2D1StandController controller(domain_id, interface_name);
  controller.run();
  return 0;
}
