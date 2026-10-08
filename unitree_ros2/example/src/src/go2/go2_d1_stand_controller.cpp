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

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"

namespace {
constexpr int kArmJointCount = 7;
constexpr double kArmMoveDuration = 2.0;
// NOTE: leg motors 0-11 (rt/lowstate motor_state) are owned by the Go2
// controller/bridge. This node is arm-only and must not subscribe to
// rt/lowstate: that topic is reserved for the Go2. Arm startup feedback comes
// from the D1 SDK topics (see /workspace/d1_sdk/src/get_arm_joint_angle.cpp):
//   - "current_servo_angle" (PubServoInfo_, servo0..6 in degrees)
//   - "arm_Feedback" (ArmString_, status JSON)
constexpr double kControlPeriodMs = 20.0;
constexpr double kArmWaitTimeoutSec = 3.0;
constexpr char kArmCommandTopic[] = "rt/arm_Command";
constexpr char kArmServoTopic[] = "current_servo_angle";
constexpr char kArmFeedbackTopic[] = "arm_Feedback";
constexpr std::array<double, kArmJointCount> kArmHomeAngles = {
    0.0, -60.0, 60.0, 0.0, 30.0, 0.0, 0.0};

// (crc32 helper + rt/lowcmd path removed; leg motors are owned by
// go2_champ_walk_controller. History preserved in git.)

float interpolate_arm(float start, float target, double progress) {
  return static_cast<float>((1.0 - progress) * start + progress * target);
}

}  // namespace

class Go2D1StandController final {
 public:
  Go2D1StandController(int domain_id, const std::string &interface_name) {
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, interface_name);

    arm_command_pub_ = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(
        kArmCommandTopic);
    arm_command_pub_->InitChannel();
    arm_state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(
        kArmServoTopic);
    arm_state_sub_->InitChannel(
        [this](const void *message) { handleArmState(message); }, 1);
    arm_feedback_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(
        kArmFeedbackTopic);
    arm_feedback_sub_->InitChannel(
        [this](const void *message) { handleArmFeedback(message); }, 1);

    // Arm-only node: rt/lowstate + rt/lowcmd are reserved for the Go2
    // controller/bridge; this node only uses the D1 SDK topics above.
  }

  void run() {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(kArmWaitTimeoutSec);
    while (!arm_state_received_.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!arm_state_received_.load()) {
      std::cerr << "[go2_d1_stand_controller] no " << kArmServoTopic
                << " feedback after " << kArmWaitTimeoutSec
                << "s; starting from home pose." << std::endl;
    }
    initialize_targets();
    publish_arm_enable(true);
    while (running_) {
      write_command();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)));
    }
  }

 private:
  void handleArmState(const void *message) {
    const auto &state =
        *static_cast<const unitree_arm::msg::dds_::PubServoInfo_ *>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    arm_current_angles_ = {state.servo0_data_(), state.servo1_data_(),
                           state.servo2_data_(), state.servo3_data_(),
                           state.servo4_data_(), state.servo5_data_(),
                           state.servo6_data_()};
    arm_state_received_ = true;
  }

  void handleArmFeedback(const void *message) {
    const auto &feedback =
        *static_cast<const unitree_arm::msg::dds_::ArmString_ *>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_arm_feedback_ = feedback.data_();
  }

  void initialize_targets() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (arm_state_received_.load()) {
      arm_start_angles_ = arm_current_angles_;
    } else {
      arm_start_angles_ = kArmHomeAngles;
    }
  }

  void write_command() {
    // Arm-only: no rt/lowcmd writes. Only the arm pose stream remains.
    const double arm_progress = std::min(motion_time_ / kArmMoveDuration, 1.0);
    publish_arm_pose(arm_progress);
    motion_time_ += kControlPeriodMs / 1000.0;
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
           << interpolate_arm(static_cast<float>(arm_start_angles_[index]),
                          static_cast<float>(kArmHomeAngles[index]), progress);
    }
    json << "}}";
    unitree_arm::msg::dds_::ArmString_ command;
    command.data_(json.str());
    arm_command_pub_->Write(command);
  }

  std::array<double, kArmJointCount> arm_start_angles_{};
  std::array<double, kArmJointCount> arm_current_angles_{};
  std::string last_arm_feedback_;
  unitree::robot::ChannelPublisherPtr<unitree_arm::msg::dds_::ArmString_> arm_command_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_arm::msg::dds_::PubServoInfo_> arm_state_sub_;
  unitree::robot::ChannelSubscriberPtr<unitree_arm::msg::dds_::ArmString_> arm_feedback_sub_;
  std::mutex state_mutex_;
  std::atomic_bool running_{true};
  std::atomic_bool arm_state_received_{false};
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
