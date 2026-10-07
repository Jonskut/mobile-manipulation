/* CHAMP gait planner: ROS-only half of the Go2 CHAMP split.
 * Sends joint targets to go2_champ_dds_bridge over loopback UDP (port 17610),
 * receives rt/lowstate feedback over loopback UDP (port 17611), and republishes
 * it as joint_states_measured / imu_measured for introspection.
 * No Unitree DDS here (that lives in the bridge process, which must stay
 * ROS-free: Unitree's bundled CycloneDDS clashes with ROS's rmw_cyclonedds_cpp
 * inside one process).
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <champ/body_controller/body_controller.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>

namespace {
constexpr int kNumLegMotors = 12;
constexpr uint16_t kTargetsPort = 17610;
constexpr uint16_t kFeedbackPort = 17611;
constexpr double kControlPeriod = 0.002;
constexpr std::array<int, kNumLegMotors> kChampToUnitreeMotor = {
    3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8,
};
constexpr float kHipOffsetX = 0.1934F;
constexpr float kHipOffsetY = 0.0465F;
constexpr float kThighOffsetY = 0.0955F;
constexpr float kThighLength = 0.213F;
constexpr float kCalfLength = 0.213F;
float interpolate(float start, float target, double progress) {
  const double c = std::min(std::max(progress, 0.0), 1.0);
  return static_cast<float>((1.0 - c) * start + c * target);
}
void quaternionToRpy(double x, double y, double z, double w, double &roll,
                     double &pitch, double &yaw) {
  roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double sinp = 2.0 * (w * y - z * x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(1.5707963267948966, sinp)
                                : std::asin(sinp);
  yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
inline unsigned long nowChampTimeUs() {
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch());
  return static_cast<unsigned long>(us.count());
}
}  // namespace
class Go2ChampWalkController : public rclcpp::Node {
 public:
  Go2ChampWalkController()
      : Node("go2_champ_walk_controller"),
        body_controller_(base_),
        leg_controller_(base_, nowChampTimeUs()),
        kinematics_(base_) {
    declare_parameter("gait.knee_orientation", std::string(">>"));
    declare_parameter("gait.pantograph_leg", false);
    declare_parameter("gait.odom_scaler", 0.9);
    declare_parameter("gait.max_linear_velocity_x", 0.3);
    declare_parameter("gait.max_linear_velocity_y", 0.25);
    declare_parameter("gait.max_angular_velocity_z", 0.5);
    declare_parameter("gait.com_x_translation", 0.0);
    declare_parameter("gait.swing_height", 0.04);
    declare_parameter("gait.stance_depth", 0.01);
    declare_parameter("gait.stance_duration", 0.25);
    declare_parameter("gait.nominal_height", 0.225);
    declare_parameter("kp", 60.0);
    declare_parameter("kd", 5.0);
    declare_parameter("stand_duration", 3.0);
    declare_parameter("cmd_vel_timeout", 0.5);
    declare_parameter("publish_joint_states", true);
    declare_parameter("publish_imu", true);
    declare_parameter("publish_joint_control", true);
    declare_parameter("motor_signs", std::vector<double>(12, 1.0));
    loadParams();
    gait_config_.knee_orientation = knee_orientation_.c_str();
    setLegGeometry();
    base_.setGaitConfig(gait_config_);
    req_pose_.position.z = gait_config_.nominal_height;
    // Loopback UDP to the DDS bridge (separate process).
    udp_tx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    udp_rx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_tx_fd_ >= 0 && udp_rx_fd_ >= 0) {
      sockaddr_in rx_addr{};
      rx_addr.sin_family = AF_INET;
      rx_addr.sin_port = htons(kFeedbackPort);
      rx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      if (::bind(udp_rx_fd_, reinterpret_cast<sockaddr *>(&rx_addr), sizeof(rx_addr)) == 0) {
        ::fcntl(udp_rx_fd_, F_SETFL, O_NONBLOCK);
        udp_targets_addr_.sin_family = AF_INET;
        udp_targets_addr_.sin_port = htons(kTargetsPort);
        udp_targets_addr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        udp_ok_ = true;
      } else {
        RCLCPP_WARN(get_logger(), "UDP bind(17611) failed; bridge feedback unavailable");
      }
    }
    joint_targets_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_targets", 10);
    measured_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states_measured", 10);
    imu_measured_pub_ =
        create_publisher<sensor_msgs::msg::Imu>("imu_measured", 10);
    measured_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_measured", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          // Debug/introspection republish path (see pollBridgeFeedback):
          // external joint_states_measured still honoured when UDP is down.
          if (!udp_ok_) {
            if (msg->position.size() < kNumLegMotors) return;
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (int i = 0; i < kNumLegMotors; ++i)
              measured_q_[i] = static_cast<float>(msg->position[i]);
            have_state_.store(true, std::memory_order_release);
          }
        });
    imu_measured_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu_measured", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          if (udp_ok_) return;
          std::lock_guard<std::mutex> lock(state_mutex_);
          imu_quat_[0] = static_cast<float>(msg->orientation.w);
          imu_quat_[1] = static_cast<float>(msg->orientation.x);
          imu_quat_[2] = static_cast<float>(msg->orientation.y);
          imu_quat_[3] = static_cast<float>(msg->orientation.z);
          imu_gyro_[0] = static_cast<float>(msg->angular_velocity.x);
          imu_gyro_[1] = static_cast<float>(msg->angular_velocity.y);
          imu_gyro_[2] = static_cast<float>(msg->angular_velocity.z);
          imu_acc_[0] = static_cast<float>(msg->linear_acceleration.x);
          imu_acc_[1] = static_cast<float>(msg->linear_acceleration.y);
          imu_acc_[2] = static_cast<float>(msg->linear_acceleration.z);
        });
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          req_vel_.linear.x = static_cast<float>(msg->linear.x);
          req_vel_.linear.y = static_cast<float>(msg->linear.y);
          req_vel_.angular.z = static_cast<float>(msg->angular.z);
          last_cmd_vel_steady_ = std::chrono::steady_clock::now();
        });


    cmd_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "cmd_pose", 10,
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
          double roll, pitch, yaw;
          quaternionToRpy(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w, roll, pitch,
                          yaw);
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          req_pose_.orientation.roll = static_cast<float>(roll);
          req_pose_.orientation.pitch = static_cast<float>(pitch);
          req_pose_.orientation.yaw = static_cast<float>(yaw);
          req_pose_.position.x = static_cast<float>(msg->position.x);
          req_pose_.position.y = static_cast<float>(msg->position.y);
          req_pose_.position.z = static_cast<float>(msg->position.z) +
                                 gait_config_.nominal_height;
        });
    joint_states_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    joint_commands_pub_ =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_commands", 10);
    RCLCPP_INFO(get_logger(), "CHAMP gait planner ready (ROS-only)");
    control_thread_ = std::thread([this]() { controlLoop(); });
  }
  ~Go2ChampWalkController() override {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
    if (udp_tx_fd_ >= 0) ::close(udp_tx_fd_);
    if (udp_rx_fd_ >= 0) ::close(udp_rx_fd_);
  }

 private:
  void loadParams() {
    get_parameter("gait.knee_orientation", knee_orientation_);
    get_parameter("gait.pantograph_leg", gait_config_.pantograph_leg);
    get_parameter("gait.odom_scaler", gait_config_.odom_scaler);
    get_parameter("gait.max_linear_velocity_x", gait_config_.max_linear_velocity_x);
    get_parameter("gait.max_linear_velocity_y", gait_config_.max_linear_velocity_y);
    get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    get_parameter("gait.com_x_translation", gait_config_.com_x_translation);
    get_parameter("gait.swing_height", gait_config_.swing_height);
    get_parameter("gait.stance_depth", gait_config_.stance_depth);
    get_parameter("gait.stance_duration", gait_config_.stance_duration);
    get_parameter("gait.nominal_height", gait_config_.nominal_height);
    get_parameter("kp", kp_);
    get_parameter("kd", kd_);
    get_parameter("stand_duration", stand_duration_);
    get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
    get_parameter("publish_joint_states", publish_joint_states_);
    get_parameter("publish_imu", publish_imu_);
    get_parameter("publish_joint_control", publish_joint_control_);
    std::vector<double> motor_signs(12, 1.0);
    get_parameter("motor_signs", motor_signs);
    if (motor_signs.size() == 12) {
      for (int i = 0; i < 12; ++i) motor_signs_[i] = motor_signs[i];
    }
  }
  void setLegGeometry() {
    struct LegOrigin { float hip_x, hip_y, thigh_y; };
    const LegOrigin origins[4] = {
        {+kHipOffsetX, +kHipOffsetY, +kThighOffsetY},
        {+kHipOffsetX, -kHipOffsetY, -kThighOffsetY},
        {-kHipOffsetX, +kHipOffsetY, +kThighOffsetY},
        {-kHipOffsetX, -kHipOffsetY, -kThighOffsetY},
    };
    for (int i = 0; i < 4; ++i) {
      champ::QuadrupedLeg *leg = base_.legs[i];
      leg->hip.setOrigin(origins[i].hip_x, origins[i].hip_y, 0.0F, 0.0F, 0.0F, 0.0F);
      leg->upper_leg.setOrigin(0.0F, origins[i].thigh_y, 0.0F, 0.0F, 0.0F, 0.0F);
      leg->lower_leg.setOrigin(0.0F, 0.0F, -kThighLength, 0.0F, 0.0F, 0.0F);
      leg->foot.setOrigin(0.0F, 0.0F, -kCalfLength, 0.0F, 0.0F, 0.0F);
    }
  }
  void controlLoop() {
    RCLCPP_INFO(get_logger(), "Waiting for bridge feedback (UDP 17611) ...");
    while (running_ && !have_state_.load(std::memory_order_acquire)) {
      pollBridgeFeedback();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!running_) return;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (int i = 0; i < kNumLegMotors; ++i) start_pose_[i] = measured_q_[i];
    }
    last_cmd_vel_steady_ = std::chrono::steady_clock::now();
    last_joint_states_steady_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    last_imu_steady_ = last_joint_states_steady_;
    last_joint_commands_steady_ = last_joint_states_steady_;
    RCLCPP_INFO(get_logger(), "Feedback received, ramping to stance ...");
    auto next_tick = std::chrono::steady_clock::now();
    while (running_ && rclcpp::ok()) {
      next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(kControlPeriod));
      pollBridgeFeedback();
      stepOnce();
      std::this_thread::sleep_until(next_tick);
    }
  }
  // Drain bridge feedback datagrams: 12 q + 4 quat + 3 gyro + 3 accel.
  // Also republishes joint_states_measured / imu_measured for introspection.
  void pollBridgeFeedback() {
    if (!udp_ok_) return;
    std::array<float, 22> buf{};
    bool got = false;
    for (;;) {
      const ssize_t n =
          ::recv(udp_rx_fd_, buf.data(), buf.size() * sizeof(float), 0);
      if (n != static_cast<ssize_t>(buf.size() * sizeof(float))) break;
      got = true;
    }
    if (!got) return;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (int i = 0; i < kNumLegMotors; ++i) measured_q_[i] = buf[i];
      imu_quat_[0] = buf[12];
      imu_quat_[1] = buf[13];
      imu_quat_[2] = buf[14];
      imu_quat_[3] = buf[15];
      imu_gyro_[0] = buf[16];
      imu_gyro_[1] = buf[17];
      imu_gyro_[2] = buf[18];
      imu_acc_[0] = buf[19];
      imu_acc_[1] = buf[20];
      imu_acc_[2] = buf[21];
    }
    have_state_.store(true, std::memory_order_release);
    const rclcpp::Time stamp = now();
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = kUnitreeJointNames;
    js.position.assign(buf.begin(), buf.begin() + kNumLegMotors);
    measured_pub_->publish(js);
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.w = buf[12];
    imu.orientation.x = buf[13];
    imu.orientation.y = buf[14];
    imu.orientation.z = buf[15];
    imu.angular_velocity.x = buf[16];
    imu.angular_velocity.y = buf[17];
    imu.angular_velocity.z = buf[18];
    imu.linear_acceleration.x = buf[19];
    imu.linear_acceleration.y = buf[20];
    imu.linear_acceleration.z = buf[21];
    imu_measured_pub_->publish(imu);
  }
  void stepOnce() {
    float target_joints[kNumLegMotors];
    geometry::Transformation target_feet[4];
    if (!have_state_.load(std::memory_order_acquire)) return;
    champ::Velocities vel;
    champ::Pose pose;
    bool vel_fresh;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vel = req_vel_;
      pose = req_pose_;
      const auto steady_now = std::chrono::steady_clock::now();
      vel_fresh = std::chrono::duration<double>(steady_now - last_cmd_vel_steady_).count() <=
                  cmd_vel_timeout_;
    }
    if (!vel_fresh) {
      vel.linear.x = 0.0F;
      vel.linear.y = 0.0F;
      vel.angular.z = 0.0F;
    }
    body_controller_.poseCommand(target_feet, pose);
    leg_controller_.velocityCommand(target_feet, vel, nowChampTimeUs());
    kinematics_.inverse(target_joints, target_feet);
    std::array<float, kNumLegMotors> unitree_targets{};
    const double stand_progress =
        stand_duration_ > 0.0 ? std::min(motion_time_ / stand_duration_, 1.0) : 1.0;
    for (int champ_idx = 0; champ_idx < kNumLegMotors; ++champ_idx) {
      const int motor = kChampToUnitreeMotor[champ_idx];
      float target = target_joints[champ_idx];
      if (std::isnan(target)) {
        target = last_command_[champ_idx];
      } else {
        target = static_cast<float>(motor_signs_[champ_idx] * target);
        last_command_[champ_idx] = target;
      }
      unitree_targets[motor] = interpolate(start_pose_[motor], target, stand_progress);
    }
    motion_time_ += kControlPeriod;
    sendTargetsToBridge(unitree_targets);
    sensor_msgs::msg::JointState targets_msg;
    targets_msg.header.stamp = now();
    targets_msg.name = kUnitreeJointNames;
    targets_msg.position.assign(unitree_targets.begin(), unitree_targets.end());
    joint_targets_pub_->publish(targets_msg);
    publishFeedback(target_joints);
  }

  // 12 q + 12 kp + 12 kd floats to the bridge over loopback UDP.
  void sendTargetsToBridge(const std::array<float, kNumLegMotors> &unitree_targets) {
    if (!udp_ok_) return;
    std::array<float, 36> buf{};
    for (int i = 0; i < kNumLegMotors; ++i) {
      buf[i] = unitree_targets[i];
      buf[12 + i] = static_cast<float>(kp_);
      buf[24 + i] = static_cast<float>(kd_);
    }
    ::sendto(udp_tx_fd_, buf.data(), buf.size() * sizeof(float), 0,
             reinterpret_cast<sockaddr *>(&udp_targets_addr_), sizeof(udp_targets_addr_));
  }

  void publishFeedback(const float target_joints[kNumLegMotors]) {
    const rclcpp::Time stamp = now();
    const auto steady_now = std::chrono::steady_clock::now();
    if (publish_joint_states_ &&
        std::chrono::duration<double>(steady_now - last_joint_states_steady_).count() >= 0.02) {
      last_joint_states_steady_ = steady_now;
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = stamp;
      msg.name = kChampJointNames;
      msg.position.resize(kNumLegMotors);
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (int champ_idx = 0; champ_idx < kNumLegMotors; ++champ_idx) {
        const int motor = kChampToUnitreeMotor[champ_idx];
        msg.position[champ_idx] = measured_q_[motor];
      }
      joint_states_pub_->publish(msg);
    }
    if (publish_imu_ &&
        std::chrono::duration<double>(steady_now - last_imu_steady_).count() >= 0.02) {
      last_imu_steady_ = steady_now;
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "imu_link";
      std::lock_guard<std::mutex> lock(state_mutex_);
      msg.orientation.w = imu_quat_[0];
      msg.orientation.x = imu_quat_[1];
      msg.orientation.y = imu_quat_[2];
      msg.orientation.z = imu_quat_[3];
      msg.angular_velocity.x = imu_gyro_[0];
      msg.angular_velocity.y = imu_gyro_[1];
      msg.angular_velocity.z = imu_gyro_[2];
      msg.linear_acceleration.x = imu_acc_[0];
      msg.linear_acceleration.y = imu_acc_[1];
      msg.linear_acceleration.z = imu_acc_[2];
      imu_pub_->publish(msg);
    }
    if (publish_joint_control_ &&
        std::chrono::duration<double>(steady_now - last_joint_commands_steady_).count() >= 0.02) {
      last_joint_commands_steady_ = steady_now;
      trajectory_msgs::msg::JointTrajectory msg;
      msg.header.stamp = stamp;
      msg.joint_names = kChampJointNames;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.assign(target_joints, target_joints + kNumLegMotors);
      point.time_from_start = rclcpp::Duration::from_seconds(1.0 / 60.0);
      msg.points.push_back(point);
      joint_commands_pub_->publish(msg);
    }
  }
  static const std::vector<std::string> kChampJointNames;
  static const std::vector<std::string> kUnitreeJointNames;
  champ::GaitConfig gait_config_;
  champ::QuadrupedBase base_;
  champ::BodyController body_controller_;
  champ::LegController leg_controller_;
  champ::Kinematics kinematics_;
  champ::Velocities req_vel_;
  champ::Pose req_pose_;
  std::string knee_orientation_;
  double kp_ = 60.0;
  double kd_ = 5.0;
  double stand_duration_ = 3.0;
  double cmd_vel_timeout_ = 0.5;
  bool publish_joint_states_ = true;
  bool publish_imu_ = true;
  bool publish_joint_control_ = true;
  std::array<double, kNumLegMotors> motor_signs_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<float, kNumLegMotors> start_pose_{};
  std::array<float, kNumLegMotors> last_command_{};
  std::array<float, kNumLegMotors> measured_q_{};
  std::array<float, 4> imu_quat_{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> imu_gyro_{};
  std::array<float, 3> imu_acc_{};
  std::atomic_bool have_state_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_targets_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr measured_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_measured_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_measured_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_pub_;
  std::mutex state_mutex_;
  std::mutex cmd_mutex_;
  std::atomic_bool running_{true};
  std::thread control_thread_;
  int udp_tx_fd_{-1};
  int udp_rx_fd_{-1};
  sockaddr_in udp_targets_addr_{};
  bool udp_ok_{false};
  double motion_time_ = 0.0;
  std::chrono::steady_clock::time_point last_cmd_vel_steady_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_joint_states_steady_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_imu_steady_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_joint_commands_steady_{
      std::chrono::steady_clock::now()};
};
const std::vector<std::string> Go2ChampWalkController::kChampJointNames = {
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"};
const std::vector<std::string> Go2ChampWalkController::kUnitreeJointNames = {
    "fr_hip_joint", "fr_thigh_joint", "fr_calf_joint",
    "fl_hip_joint", "fl_thigh_joint", "fl_calf_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
    "rl_hip_joint", "rl_thigh_joint", "rl_calf_joint"};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2ChampWalkController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

