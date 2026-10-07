/* CHAMP DDS bridge: pure-DDS half of the Go2 CHAMP split (no rclcpp).
 * Subscribes joint_targets (ROS) -> writes rt/lowcmd (Unitree DDS) at 500Hz,
 * forwards rt/lowstate + IMU to joint_states_measured + imu_measured (ROS).
 * Must run as its own process: Unitree ChannelFactory cannot share a process
 * with rclcpp on the same DDS domain.
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "rcl/rcl.h"
#include "rcl/node.h"
#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "rcl/wait.h"
#include "rcutils/allocator.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "sensor_msgs/msg/joint_state.h"
#include "sensor_msgs/msg/imu.h"
namespace {
constexpr int kNumLegMotors = 12;
constexpr double kControlPeriod = 0.002;
constexpr float kPosStop = 2.146E+9F;
constexpr float kVelStop = 16000.0F;
uint32_t crc32_core(uint32_t *ptr, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < length; ++i) {
    uint32_t bit = 1U << 31;
    uint32_t data = ptr[i];
    for (uint32_t c = 0; c < 32; ++c) {
      crc = (crc & 0x80000000U) ? (crc << 1) ^ polynomial : crc << 1;
      if (data & bit) crc ^= polynomial;
      bit >>= 1;
    }
  }
  return crc;
}
}  // namespace
struct BridgeState {
  std::mutex mutex;
  std::array<float, kNumLegMotors> targets{};
  std::array<double, kNumLegMotors> kp{};
  std::array<double, kNumLegMotors> kd{};
  std::atomic_bool have_targets{false};
  std::atomic_bool running{true};
  std::array<float, kNumLegMotors> measured_q{};
  std::array<float, 4> imu_quat{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> imu_gyro{};
  std::array<float, 3> imu_acc{};
  std::atomic_bool have_low_state{false};
  unitree_go::msg::dds_::LowCmd_ low_cmd{};
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> pub;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub;
};
static void handleLowState(BridgeState *s, const void *message) {
  const auto &state =
      *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
  std::lock_guard<std::mutex> lock(s->mutex);
  for (int i = 0; i < kNumLegMotors; ++i)
    s->measured_q[i] = state.motor_state()[i].q();
  s->imu_quat[0] = state.imu_state().quaternion()[0];
  s->imu_quat[1] = state.imu_state().quaternion()[1];
  s->imu_quat[2] = state.imu_state().quaternion()[2];
  s->imu_quat[3] = state.imu_state().quaternion()[3];
  s->imu_gyro[0] = state.imu_state().gyroscope()[0];
  s->imu_gyro[1] = state.imu_state().gyroscope()[1];
  s->imu_gyro[2] = state.imu_state().gyroscope()[2];
  s->imu_acc[0] = state.imu_state().accelerometer()[0];
  s->imu_acc[1] = state.imu_state().accelerometer()[1];
  s->imu_acc[2] = state.imu_state().accelerometer()[2];
  s->have_low_state.store(true, std::memory_order_release);
}
static void initLowCmd(unitree_go::msg::dds_::LowCmd_ &cmd) {
  cmd.head()[0] = 0xFE;
  cmd.head()[1] = 0xEF;
  cmd.level_flag() = 0xFF;
  cmd.gpio() = 0;
  for (auto &motor : cmd.motor_cmd()) {
    motor.mode() = 0x01;
    motor.q() = kPosStop;
    motor.kp() = 0.0F;
    motor.dq() = kVelStop;
    motor.kd() = 0.0F;
    motor.tau() = 0.0F;
  }
}
int main(int argc, char **argv) {
  int dds_domain_id = 1;
  std::string dds_interface = "lo";
  double kp = 60.0, kd = 5.0;
  int pos = 0;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--ros-args" || arg == "-r") break;
    if (arg.rfind("--", 0) == 0) continue;
    if (pos == 0) { try { dds_domain_id = std::stoi(arg); } catch (...) {} }
    else if (pos == 1) { dds_interface = arg; }
    else if (pos == 2) { try { kp = std::stod(arg); } catch (...) {} }
    else if (pos == 3) { try { kd = std::stod(arg); } catch (...) {} }
    ++pos;
  }
  BridgeState state;
  state.kp.fill(kp);
  state.kd.fill(kd);
  initLowCmd(state.low_cmd);
  try {
    unitree::robot::ChannelFactory::Instance()->Init(dds_domain_id, dds_interface);
  } catch (const std::exception &e) {
    std::cerr << "[champ_bridge] ChannelFactory::Init failed: " << e.what() << std::endl;
    return 1;
  }
  state.pub = std::make_shared<
      unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>("rt/lowcmd");
  state.pub->InitChannel();
  state.sub = std::make_shared<
      unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
  state.sub->InitChannel([&state](const void *msg) { handleLowState(&state, msg); }, 1);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_context_t context = rcl_get_zero_initialized_context();
  if (rcl_init(argc, argv, &init_options, &context) != RCL_RET_OK) {
    std::cerr << "[champ_bridge] rcl_init failed" << std::endl;
    return 1;
  }
  rcl_node_options_t node_options = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  if (rcl_node_init(&node, "go2_champ_dds_bridge", "", &context, &node_options) != RCL_RET_OK) {
    std::cerr << "[champ_bridge] rcl_node_init failed" << std::endl;
    return 1;
  }
  const rosidl_message_type_support_t *joint_state_ts =
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
  const rosidl_message_type_support_t *imu_ts =
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
  rcl_publisher_t measured_pub = rcl_get_zero_initialized_publisher();
  rcl_publisher_t imu_pub = rcl_get_zero_initialized_publisher();
  rcl_subscription_options_t sub_options = rcl_subscription_get_default_options();
  rcl_subscription_t targets_sub = rcl_get_zero_initialized_subscription();
  rcl_publisher_init(&measured_pub, &node, joint_state_ts, "joint_states_measured", &pub_options);
  rcl_publisher_init(&imu_pub, &node, imu_ts, "imu_measured", &pub_options);
  rcl_subscription_init(&targets_sub, &node, joint_state_ts, "joint_targets", &sub_options);
  std::cout << "[champ_bridge] ready" << std::endl;

  auto next_tick = std::chrono::steady_clock::now();
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, allocator);
  int pub_counter = 0;
  while (state.running.load() && rcl_context_is_valid(&context)) {
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(kControlPeriod));
    rcl_wait_set_clear(&wait_set);
    size_t idx = 0;
    rcl_wait_set_add_subscription(&wait_set, &targets_sub, &idx);
    if (rcl_wait(&wait_set, RCL_MS_TO_NS(1)) == RCL_RET_OK && wait_set.subscriptions[0]) {
      sensor_msgs__msg__JointState incoming;
      sensor_msgs__msg__JointState__init(&incoming);
      rmw_message_info_t info;
      if (rcl_take(&targets_sub, &incoming, &info, nullptr) == RCL_RET_OK) {
        std::lock_guard<std::mutex> lock(state.mutex);
        int n = std::min<int>(kNumLegMotors, incoming.position.size);
        for (int i = 0; i < n; ++i) state.targets[i] = static_cast<float>(incoming.position.data[i]);
        if (incoming.effort.size >= kNumLegMotors) {
          for (int i = 0; i < kNumLegMotors; ++i) state.kp[i] = incoming.effort.data[i];
          state.kd.fill(kd);
        }
        state.have_targets.store(true, std::memory_order_release);
      }
      sensor_msgs__msg__JointState__fini(&incoming);
    }
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      for (int motor = 0; motor < kNumLegMotors; ++motor) {
        auto &mc = state.low_cmd.motor_cmd()[motor];
        mc.mode() = 0x01;
        mc.q() = state.have_targets.load() ? state.targets[motor] : kPosStop;
        mc.kp() = state.have_targets.load() ? static_cast<float>(state.kp[motor]) : 0.0F;
        mc.dq() = 0.0F;
        mc.kd() = state.have_targets.load() ? static_cast<float>(state.kd[motor]) : 0.0F;
        mc.tau() = 0.0F;
      }
    }
    state.low_cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&state.low_cmd),
                                     (sizeof(state.low_cmd) >> 2) - 1);
    state.pub->Write(state.low_cmd);
    if (++pub_counter % 10 == 0 && state.have_low_state.load()) {
      sensor_msgs__msg__JointState measured;
      sensor_msgs__msg__JointState__init(&measured);
      rosidl_runtime_c__String__Sequence__init(&measured.name, kNumLegMotors);
      rosidl_runtime_c__double__Sequence__init(&measured.position, kNumLegMotors);
      static const char *names[12] = {"fr_hip_joint", "fr_thigh_joint", "fr_calf_joint",
                                      "fl_hip_joint", "fl_thigh_joint", "fl_calf_joint",
                                      "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
                                      "rl_hip_joint", "rl_thigh_joint", "rl_calf_joint"};
      std::lock_guard<std::mutex> lock(state.mutex);
      for (int i = 0; i < kNumLegMotors; ++i) {
        rosidl_runtime_c__String__assign(&measured.name.data[i], names[i]);
        measured.position.data[i] = state.measured_q[i];
      }
      rcl_publish(&measured_pub, &measured, nullptr);
      sensor_msgs__msg__JointState__fini(&measured);
      sensor_msgs__msg__Imu imu_msg;
      sensor_msgs__msg__Imu__init(&imu_msg);
      imu_msg.orientation.w = state.imu_quat[0];
      imu_msg.orientation.x = state.imu_quat[1];
      imu_msg.orientation.y = state.imu_quat[2];
      imu_msg.orientation.z = state.imu_quat[3];
      imu_msg.angular_velocity.x = state.imu_gyro[0];
      imu_msg.angular_velocity.y = state.imu_gyro[1];
      imu_msg.angular_velocity.z = state.imu_gyro[2];
      imu_msg.linear_acceleration.x = state.imu_acc[0];
      imu_msg.linear_acceleration.y = state.imu_acc[1];
      imu_msg.linear_acceleration.z = state.imu_acc[2];
      rcl_publish(&imu_pub, &imu_msg, nullptr);
      sensor_msgs__msg__Imu__fini(&imu_msg);
    }
    std::this_thread::sleep_until(next_tick);
  }
  rcl_subscription_fini(&targets_sub, &node);
  rcl_publisher_fini(&measured_pub, &node);
  rcl_publisher_fini(&imu_pub, &node);
  rcl_node_fini(&node);
  rcl_shutdown(&context);
  return 0;
}

