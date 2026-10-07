/* CHAMP DDS bridge: pure-DDS half of the Go2 CHAMP split (no ROS at all).
 * Receives joint targets over loopback UDP from go2_champ_gait_planner,
 * writes rt/lowcmd (Unitree DDS) at 500Hz, and forwards rt/lowstate + IMU
 * back over loopback UDP.
 * Must stay ROS-free: this process loads Unitree's bundled CycloneDDS
 * (/opt/unitree_robotics), while ROS's rmw_cyclonedds_cpp loads Humble's
 * CycloneDDS -- mixing both in one process breaks DDS domain creation.
 *
 * UDP protocol (all little-endian floats, host order on x86_64):
 *   planner -> bridge (port 17610): 12 x float q targets, then 12 x float kp,
 *                                    then 12 x float kd  (36 floats total).
 *   bridge -> planner (port 17611): 12 x float measured q, then 4 x float
 *                                    imu quat (w,x,y,z), 3 x float gyro,
 *                                    3 x float accel (22 floats total).
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

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
namespace {
constexpr int kNumLegMotors = 12;
constexpr double kControlPeriod = 0.002;
constexpr float kPosStop = 2.146E+9F;
constexpr float kVelStop = 16000.0F;
constexpr uint16_t kTargetsPort = 17610;
constexpr uint16_t kFeedbackPort = 17611;
constexpr size_t kTargetsFloats = 36;   // 12 q + 12 kp + 12 kd
constexpr size_t kFeedbackFloats = 28;  // 12 leg q + 4 quat + 3 gyro + 3 accel
                                        // + 6 arm q (rt/lowstate motors 12..17)
constexpr size_t kFeedbackLegacyFloats = 22;  // pre-arm packet, still accepted
constexpr int kNumArmMotors = 6;
constexpr int kFirstArmMotor = 12;  // legs 0..11, D1 arm 12..17 in LowState
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
  std::array<float, kNumArmMotors> arm_q{};
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
  // D1 arm joints live at LowState motors 12..17 on both sim (MuJoCo
  // d1_joint*_pos sensors) and the real Go2+D1 — 1:1 sim-to-real.
  for (int i = 0; i < kNumArmMotors; ++i)
    s->arm_q[i] = state.motor_state()[kFirstArmMotor + i].q();
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

  // Loopback UDP sockets (no ROS in this process by design).
  const int rx_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  const int tx_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (rx_fd < 0 || tx_fd < 0) {
    std::cerr << "[champ_bridge] socket() failed" << std::endl;
    return 1;
  }
  sockaddr_in rx_addr{};
  rx_addr.sin_family = AF_INET;
  rx_addr.sin_port = htons(kTargetsPort);
  rx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(rx_fd, reinterpret_cast<sockaddr *>(&rx_addr), sizeof(rx_addr)) != 0) {
    std::cerr << "[champ_bridge] bind(17610) failed" << std::endl;
    return 1;
  }
  ::fcntl(rx_fd, F_SETFL, O_NONBLOCK);
  sockaddr_in tx_addr{};
  tx_addr.sin_family = AF_INET;
  tx_addr.sin_port = htons(kFeedbackPort);
  tx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  std::cout << "[champ_bridge] ready (udp 17610/17611)" << std::endl;

  auto next_tick = std::chrono::steady_clock::now();
  int pub_counter = 0;
  std::array<float, kTargetsFloats> rx_buf{};
  std::array<float, kFeedbackFloats> tx_buf{};
  while (state.running.load()) {
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(kControlPeriod));
    // Drain target datagrams (newest wins).
    for (;;) {
      const ssize_t n = ::recv(rx_fd, rx_buf.data(), rx_buf.size() * sizeof(float), 0);
      if (n != static_cast<ssize_t>(rx_buf.size() * sizeof(float))) break;
      std::lock_guard<std::mutex> lock(state.mutex);
      for (int i = 0; i < kNumLegMotors; ++i) {
        state.targets[i] = rx_buf[i];
        state.kp[i] = rx_buf[12 + i];
        state.kd[i] = rx_buf[24 + i];
      }
      state.have_targets.store(true, std::memory_order_release);
    }
    {
      const bool have = state.have_targets.load(std::memory_order_acquire);
      for (int motor = 0; motor < kNumLegMotors; ++motor) {
        auto &mc = state.low_cmd.motor_cmd()[motor];
        mc.mode() = 0x01;
        mc.q() = have ? state.targets[motor] : kPosStop;
        mc.kp() = have ? static_cast<float>(state.kp[motor]) : 0.0F;
        mc.dq() = 0.0F;
        mc.kd() = have ? static_cast<float>(state.kd[motor]) : 0.0F;
        mc.tau() = 0.0F;
      }
    }
    state.low_cmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&state.low_cmd),
                                     (sizeof(state.low_cmd) >> 2) - 1);
    state.pub->Write(state.low_cmd);
    if (++pub_counter % 10 == 0 && state.have_low_state.load(std::memory_order_acquire)) {
      {
        std::lock_guard<std::mutex> lock(state.mutex);
        for (int i = 0; i < kNumLegMotors; ++i) tx_buf[i] = state.measured_q[i];
        tx_buf[12] = state.imu_quat[0];
        tx_buf[13] = state.imu_quat[1];
        tx_buf[14] = state.imu_quat[2];
        tx_buf[15] = state.imu_quat[3];
        tx_buf[16] = state.imu_gyro[0];
        tx_buf[17] = state.imu_gyro[1];
        tx_buf[18] = state.imu_gyro[2];
        tx_buf[19] = state.imu_acc[0];
        tx_buf[20] = state.imu_acc[1];
        tx_buf[21] = state.imu_acc[2];
        for (int i = 0; i < kNumArmMotors; ++i)
          tx_buf[22 + i] = state.arm_q[i];
      }
      ::sendto(tx_fd, tx_buf.data(), tx_buf.size() * sizeof(float), 0,
               reinterpret_cast<sockaddr *>(&tx_addr), sizeof(tx_addr));
    }
    std::this_thread::sleep_until(next_tick);
  }
  ::close(rx_fd);
  ::close(tx_fd);
  return 0;
}

