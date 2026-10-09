// Go2 + D1 arm pose example (DDS, arm-only).
// Clone of go2_d1_stand_controller.cpp with a pose table for full
// left / right / forward / backward extensions + home, reusing the same
// D1 SDK DDS commands (no new control code).
//
// DDS protocol (see /workspace/d1_sdk/src/*.cpp):
//   pub "rt/arm_Command" (ArmString_ JSON):
//     funcode 5 mode 1/0 = enable/disable
//     funcode 2 mode 1 angle0..6 = multi-joint absolute
//   sub "current_servo_angle" (PubServoInfo_ servo0..6, degrees)
//   sub "arm_Feedback" (ArmString_ status JSON)
// Arm-only: never touches rt/lowstate / rt/lowcmd (Go2 bridge owns those).
//
// Usage:
//   go2_d1_arm_poses [pose] [interface]
//     pose: home|forward|backward|left|right|demo (default: home)
//     interface: network interface (default: lo, domain 1;
//                if given, domain 0, e.g. eth0 on real robot)
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
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"

namespace {
constexpr int kArmJointCount = 7;
constexpr double kArmMoveDuration = 2.0;
constexpr double kDemoHoldSec = 3.0;
constexpr double kControlPeriodMs = 20.0;
constexpr double kArmWaitTimeoutSec = 3.0;
constexpr char kArmCommandTopic[] = "rt/arm_Command";
constexpr char kArmServoTopic[] = "current_servo_angle";
constexpr char kArmFeedbackTopic[] = "arm_Feedback";

// All angles in degrees (angle0..angle6). angle0 = base yaw, angle6 = gripper.
// URDF revolute limits are +/-2.35 rad (~+/-134.6 deg), so +/-90 deg yaw and
// the 135 deg backward yaw below stay inside limits. Validate on hardware for
// Go2-body self-collision before running at full speed.
constexpr std::array<double, kArmJointCount> kArmHomeAngles = {
    0.0, -60.0, 60.0, 0.0, 30.0, 0.0, 0.0};
constexpr std::array<double, kArmJointCount> kArmForwardAngles = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr std::array<double, kArmJointCount> kArmLeftAngles = {
    90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
constexpr std::array<double, kArmJointCount> kArmRightAngles = {
    -90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// True yaw-180 rear is outside the +/-134.6 deg URDF limit, so use max-safe
// yaw with a slight elbow bend to reach backward over the Go2 body.
constexpr std::array<double, kArmJointCount> kArmBackwardAngles = {
    135.0, 0.0, 30.0, 0.0, 30.0, 0.0, 0.0};

struct NamedPose {
  const char* name;
  std::array<double, kArmJointCount> angles;
};

constexpr NamedPose kPoses[] = {
    {"home", kArmHomeAngles},
    {"forward", kArmForwardAngles},
    {"backward", kArmBackwardAngles},
    {"left", kArmLeftAngles},
    {"right", kArmRightAngles},
};

const NamedPose* FindPose(const std::string& name) {
  for (const auto& pose : kPoses) {
    if (name == pose.name) return &pose;
  }
  return nullptr;
}

float interpolate_arm(float start, float target, double progress) {
  return static_cast<float>((1.0 - progress) * start + progress * target);
}

}  // namespace
class Go2D1ArmPoses final {
 public:
  Go2D1ArmPoses(int domain_id, const std::string& interface_name) {
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, interface_name);
    arm_command_pub_ = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(
        kArmCommandTopic);
    arm_command_pub_->InitChannel();
    arm_state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(
        kArmServoTopic);
    arm_state_sub_->InitChannel(
        [this](const void* message) { handleArmState(message); }, 1);
    arm_feedback_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(
        kArmFeedbackTopic);
    arm_feedback_sub_->InitChannel(
        [this](const void* message) { handleArmFeedback(message); }, 1);
  }

  void runSingle(const std::array<double, kArmJointCount>& target,
                 const std::string& name) {
    waitForState();
    setTarget(target);
    publish_arm_enable(true);
    std::cout << "[go2_d1_arm_poses] holding pose '" << name << "' (";
    printAngles(target);
    std::cout << ")" << std::endl;
    while (running_) {
      write_command();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)));
    }
  }

  void runDemo() {
    waitForState();
    publish_arm_enable(true);
    const std::vector<std::string> order = {"home", "forward", "left", "right",
                                            "backward", "home"};
    size_t index = 0;
    setTarget(FindPose(order[index])->angles);
    std::cout << "[go2_d1_arm_poses] demo -> " << order[index] << std::endl;
    double segment_time = 0.0;
    while (running_) {
      write_command();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(kControlPeriodMs)));
      segment_time += kControlPeriodMs / 1000.0;
      if (segment_time >= kArmMoveDuration + kDemoHoldSec) {
        index = (index + 1) % order.size();
        setTarget(FindPose(order[index])->angles);
        std::cout << "[go2_d1_arm_poses] demo -> " << order[index] << std::endl;
        segment_time = 0.0;
      }
    }
  }

 private:
  void waitForState() {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(kArmWaitTimeoutSec);
    while (!arm_state_received_.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!arm_state_received_.load()) {
      std::cerr << "[go2_d1_arm_poses] no " << kArmServoTopic
                << " feedback after " << kArmWaitTimeoutSec
                << "s; starting from home pose." << std::endl;
    }
  }
  void handleArmState(const void* message) {
    const auto& state =
        *static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    arm_current_angles_ = {state.servo0_data_(), state.servo1_data_(),
                           state.servo2_data_(), state.servo3_data_(),
                           state.servo4_data_(), state.servo5_data_(),
                           state.servo6_data_()};
    arm_state_received_ = true;
  }
  void handleArmFeedback(const void* message) {
    const auto& feedback =
        *static_cast<const unitree_arm::msg::dds_::ArmString_*>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_arm_feedback_ = feedback.data_();
  }
  void setTarget(const std::array<double, kArmJointCount>& target) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (arm_state_received_.load()) {
      arm_start_angles_ = arm_current_angles_;
    } else {
      arm_start_angles_ = kArmHomeAngles;
    }
    arm_target_angles_ = target;
    motion_time_ = 0.0;
  }
  void write_command() {
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
    std::array<double, kArmJointCount> start;
    std::array<double, kArmJointCount> target;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start = arm_start_angles_;
      target = arm_target_angles_;
    }
    std::ostringstream json;
    json << std::fixed << std::setprecision(3)
         << "{\"seq\":" << sequence_++
         << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":1";
    for (int index = 0; index < kArmJointCount; ++index) {
      json << ",\"angle" << index << "\":"
           << interpolate_arm(static_cast<float>(start[index]),
                              static_cast<float>(target[index]), progress);
    }
    json << "}}";
    unitree_arm::msg::dds_::ArmString_ command;
    command.data_(json.str());
    arm_command_pub_->Write(command);
  }
  static void printAngles(const std::array<double, kArmJointCount>& angles) {
    for (int i = 0; i < kArmJointCount; ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << "a" << i << "=" << angles[i];
    }
  }
  std::array<double, kArmJointCount> arm_start_angles_{};
  std::array<double, kArmJointCount> arm_target_angles_ = kArmHomeAngles;
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

int main(int argc, char** argv) {
  std::string pose_name = "home";
  std::string interface_name = "lo";
  int domain_id = 1;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: go2_d1_arm_poses [pose] [interface]\n"
                << "  pose: home|forward|backward|left|right|demo (default home)\n"
                << "  interface: network interface (default lo, domain 1; if "
                   "given, domain 0)\n";
      return 0;
    }
    if (arg == "demo" || FindPose(arg) != nullptr) {
      pose_name = arg;
    } else {
      interface_name = arg;
      domain_id = 0;
    }
  }
  Go2D1ArmPoses controller(domain_id, interface_name);
  if (pose_name == "demo") {
    controller.runDemo();
  } else {
    const NamedPose* pose = FindPose(pose_name);
    if (pose == nullptr) {
      std::cerr << "Unknown pose '" << pose_name << "'" << std::endl;
      return 1;
    }
    controller.runSingle(pose->angles, pose->name);
  }
  return 0;
}
