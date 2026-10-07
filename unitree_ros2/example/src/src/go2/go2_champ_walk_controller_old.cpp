/* CHAMP-based quadruped locomotion controller for the Unitree Go2.
 *
 * Port of champ_base's QuadrupedController (unitree_go2_ros2) into the
 * unitree_ros2_example package. Runs the CHAMP gait pipeline
 * (BodyController -> LegController -> Kinematics) at 500 Hz and streams
 * the resulting joint targets to the `rt/lowcmd` DDS topic consumed by
 * the MuJoCo DDS bridge (or the physical robot).
 *
 * Subscribes : `cmd_vel`  (geometry_msgs/Twist)
 *              `cmd_pose` (geometry_msgs/Pose, body pose offset)
 * Publishes : `rt/lowcmd` (DDS, Unitree Go2 LowCmd)
 *             `joint_states` (measured leg joints, sensor_msgs/JointState)
 *             `imu/data` (measured IMU, sensor_msgs/Imu)
 *             `joint_commands` (target joints, debug, trajectory_msgs)
 *
 * CHAMP leg order is LF, RF, LH, RH x (hip, upper, lower), while Unitree
 * motor order is FR, FL, RR, RL x (hip, thigh, calf); the remap below
 * translates between the two.
 *
 * Go2 leg geometry is hardcoded from unitree_go2_description
 * (urdf/const.xacro + urdf/leg.xacro). This is exactly what CHAMP's
 * URDF loader reduces to (it only sums joint origins along each chain),
 * so no URDF/xacro runtime dependency is needed.
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

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <champ/body_controller/body_controller.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>

namespace {

constexpr int kNumLegMotors = 12;
constexpr double kControlPeriod = 0.002;  // 500 Hz, matches the DDS bridge

// CHAMP joint index -> Unitree motor index.
// CHAMP order: LF(0-2), RF(3-5), LH(6-8), RH(9-11).
// Unitree order: FR(0-2), FL(3-5), RR(6-8), RL(9-11).
constexpr std::array<int, kNumLegMotors> kChampToUnitreeMotor = {
    3, 4, 5,    // LF -> FL motors
    0, 1, 2,    // RF -> FR motors
    9, 10, 11,  // LH -> RL motors
    6, 7, 8,    // RH -> RR motors
};

// Go2 leg geometry (unitree_go2_description, urdf/const.xacro):
// leg_offset_x/y = 0.1934/0.0465, thigh_offset = 0.0955,
// thigh_length = calf_length = 0.213.
constexpr float kHipOffsetX = 0.1934F;
constexpr float kHipOffsetY = 0.0465F;
constexpr float kThighOffsetY = 0.0955F;
constexpr float kThighLength = 0.213F;
constexpr float kCalfLength = 0.213F;

constexpr float kPosStop = 2.146E+9F;
constexpr float kVelStop = 16000.0F;

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

// Quaternion (x, y, z, w) -> roll/pitch/yaw, so cmd_pose needs no tf2.
void quaternionToRpy(double x, double y, double z, double w, double &roll,
                     double &pitch, double &yaw) {
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = 2.0 * (w * y - z * x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(1.5707963267948966, sinp)
                                : std::asin(sinp);
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

// CHAMP gait time base: microseconds on a monotonic clock (PhaseGenerator
// drives stance/swing purely off elapsed time, so steady_clock is correct).
inline unsigned long nowChampTimeUs() {
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch());
  return static_cast<unsigned long>(us.count());
}

}  // namespace

class Go2ChampWalkController : public rclcpp::Node {
 public:
  Go2ChampWalkController(int dds_domain_id, const std::string &dds_interface)
      : Node("go2_champ_walk_controller"),
        body_controller_(base_),
        leg_controller_(base_, nowChampTimeUs()),
        kinematics_(base_) {
    // Unitree DDS was already initialized in main() BEFORE rclcpp::init(),
    // so its domain participant exists before the ROS participant is
    // created. Use the pre-init values as param defaults; warn if the
    // parameter file disagrees (the pre-init values win).
    dds_domain_id_ = dds_domain_id;
    dds_interface_ = dds_interface;
    declareParams();
    loadParams();
    if (dds_domain_id_ != dds_domain_id || dds_interface_ != dds_interface) {
      RCLCPP_WARN(get_logger(),
                  "dds_domain_id/dds_interface params (%d/%s) differ from the "
                  "pre-initialized Unitree DDS (%d/%s); using pre-init values",
                  dds_domain_id_, dds_interface_.c_str(), dds_domain_id,
                  dds_interface.c_str());
      dds_domain_id_ = dds_domain_id;
      dds_interface_ = dds_interface;
    }

    // Keep the std::string alive: GaitConfig only stores the pointer.
    gait_config_.knee_orientation = knee_orientation_.c_str();
    setLegGeometry();
    base_.setGaitConfig(gait_config_);

    // Body pose defaults to nominal standing height (matches champ_base:
    // cmd_pose carries offsets, z is absolute = offset + nominal).
    req_pose_.position.z = gait_config_.nominal_height;

    // NOTE: ChannelFactory::Init() already ran in main() before rclcpp::init().
    low_cmd_pub_ = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
        "rt/lowcmd");
    low_cmd_pub_->InitChannel();
    low_state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
        "rt/lowstate");
    low_state_sub_->InitChannel(
        [this](const void *message) { handleLowState(message); }, 1);
    initializeCommand();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          req_vel_.linear.x = msg->linear.x;
          req_vel_.linear.y = msg->linear.y;
          req_vel_.angular.z = msg->angular.z;
          last_cmd_vel_time_ = now();
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

    RCLCPP_INFO(get_logger(),
                "Go2 CHAMP walk controller ready (nominal height %.3f m, "
                "kp %.1f kd %.1f, DDS %d/%s)",
                gait_config_.nominal_height, kp_, kd_, dds_domain_id_,
                dds_interface_.c_str());

    control_thread_ = std::thread([this]() { controlLoop(); });
  }

  ~Go2ChampWalkController() override {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
  }

 private:
  void declareParams() {
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
    declare_parameter("dds_domain_id", 1);
    declare_parameter("dds_interface", std::string("lo"));
    declare_parameter("publish_joint_states", true);
    declare_parameter("publish_imu", true);
    declare_parameter("publish_joint_control", true);
    declare_parameter("motor_signs", std::vector<double>(12, 1.0));
  }

  void loadParams() {
    get_parameter("gait.knee_orientation", knee_orientation_);
    get_parameter("gait.pantograph_leg", gait_config_.pantograph_leg);
    get_parameter("gait.odom_scaler", gait_config_.odom_scaler);
    get_parameter("gait.max_linear_velocity_x",
                  gait_config_.max_linear_velocity_x);
    get_parameter("gait.max_linear_velocity_y",
                  gait_config_.max_linear_velocity_y);
    get_parameter("gait.max_angular_velocity_z",
                  gait_config_.max_angular_velocity_z);
    get_parameter("gait.com_x_translation", gait_config_.com_x_translation);
    get_parameter("gait.swing_height", gait_config_.swing_height);
    get_parameter("gait.stance_depth", gait_config_.stance_depth);
    get_parameter("gait.stance_duration", gait_config_.stance_duration);
    get_parameter("gait.nominal_height", gait_config_.nominal_height);
    get_parameter("kp", kp_);
    get_parameter("kd", kd_);
    get_parameter("stand_duration", stand_duration_);
    get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
    get_parameter("dds_domain_id", dds_domain_id_);
    get_parameter("dds_interface", dds_interface_);
    get_parameter("publish_joint_states", publish_joint_states_);
    get_parameter("publish_imu", publish_imu_);
    get_parameter("publish_joint_control", publish_joint_control_);
    std::vector<double> motor_signs(12, 1.0);
    get_parameter("motor_signs", motor_signs);
    if (motor_signs.size() == 12) {
      for (int i = 0; i < 12; ++i) motor_signs_[i] = motor_signs[i];
    } else {
      RCLCPP_WARN(get_logger(),
                  "motor_signs must have 12 entries, using all +1.0");
    }
  }

  // Fill the CHAMP leg chains with Go2 joint origins. CHAMP's URDF
  // loader only sums joint origins along each chain, so this hardcoding
  // is exactly equivalent for the Go2 (all leg joint RPYs are zero).
  void setLegGeometry() {
    struct LegOrigin {
      float hip_x, hip_y;
      float thigh_y;
    };
    // LF, RF, LH, RH (matches base_.legs[] order).
    const LegOrigin origins[4] = {
        {+kHipOffsetX, +kHipOffsetY, +kThighOffsetY},  // LF
        {+kHipOffsetX, -kHipOffsetY, -kThighOffsetY},  // RF
        {-kHipOffsetX, +kHipOffsetY, +kThighOffsetY},  // LH
        {-kHipOffsetX, -kHipOffsetY, -kThighOffsetY},  // RH
    };
    for (int i = 0; i < 4; ++i) {
      champ::QuadrupedLeg *leg = base_.legs[i];
      // Chain: hip (trunk->hip_link), upper (hip->thigh), lower
      // (thigh->calf), foot (calf->foot), relative offsets.
      leg->hip.setOrigin(origins[i].hip_x, origins[i].hip_y, 0.0F, 0.0F, 0.0F,
                         0.0F);
      leg->upper_leg.setOrigin(0.0F, origins[i].thigh_y, 0.0F, 0.0F, 0.0F,
                               0.0F);
      leg->lower_leg.setOrigin(0.0F, 0.0F, -kThighLength, 0.0F, 0.0F, 0.0F);
      leg->foot.setOrigin(0.0F, 0.0F, -kCalfLength, 0.0F, 0.0F, 0.0F);
    }
  }

  void handleLowState(const void *message) {
    const auto &state =
        *static_cast<const unitree_go::msg::dds_::LowState_ *>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < kNumLegMotors; ++i) {
      measured_q_[i] = state.motor_state()[i].q();
    }
    imu_quat_[0] = state.imu_state().quaternion()[0];  // w
    imu_quat_[1] = state.imu_state().quaternion()[1];  // x
    imu_quat_[2] = state.imu_state().quaternion()[2];  // y
    imu_quat_[3] = state.imu_state().quaternion()[3];  // z
    imu_gyro_[0] = state.imu_state().gyroscope()[0];
    imu_gyro_[1] = state.imu_state().gyroscope()[1];
    imu_gyro_[2] = state.imu_state().gyroscope()[2];
    imu_acc_[0] = state.imu_state().accelerometer()[0];
    imu_acc_[1] = state.imu_state().accelerometer()[1];
    imu_acc_[2] = state.imu_state().accelerometer()[2];
    have_low_state_.store(true, std::memory_order_release);
  }

  void initializeCommand() {
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;
    for (auto &motor : low_cmd_.motor_cmd()) {
      motor.mode() = 0x01;  // servo mode
      motor.q() = kPosStop;
      motor.kp() = 0.0F;
      motor.dq() = kVelStop;
      motor.kd() = 0.0F;
      motor.tau() = 0.0F;
    }
  }

  void controlLoop() {
    RCLCPP_INFO(get_logger(), "Waiting for rt/lowstate ...");
    while (running_ && !have_low_state_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!running_) return;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (int i = 0; i < kNumLegMotors; ++i) start_pose_[i] = measured_q_[i];
    }
    last_cmd_vel_time_ = now();
    RCLCPP_INFO(get_logger(), "Lowstate received, ramping to stance ...");

    auto next_tick = std::chrono::steady_clock::now();
    while (running_ && rclcpp::ok()) {
      next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(kControlPeriod));
      stepOnce();
      std::this_thread::sleep_until(next_tick);
    }
  }

  void stepOnce() {
    float target_joints[kNumLegMotors];
    geometry::Transformation target_feet[4];
    if (!have_low_state_.load(std::memory_order_acquire)) return;

    champ::Velocities vel;
    champ::Pose pose;
    bool vel_fresh;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vel = req_vel_;
      pose = req_pose_;
      vel_fresh =
          (now() - last_cmd_vel_time_).seconds() <= cmd_vel_timeout_;
    }
    if (!vel_fresh) {
      vel.linear.x = 0.0F;
      vel.linear.y = 0.0F;
      vel.angular.z = 0.0F;
    }

    body_controller_.poseCommand(target_feet, pose);
    leg_controller_.velocityCommand(target_feet, vel, nowChampTimeUs());
    kinematics_.inverse(target_joints, target_feet);

    const double stand_progress =
        stand_duration_ > 0.0
            ? std::min(motion_time_ / stand_duration_, 1.0)
            : 1.0;
    for (int champ_idx = 0; champ_idx < kNumLegMotors; ++champ_idx) {
      const int motor = kChampToUnitreeMotor[champ_idx];
      float target = target_joints[champ_idx];
      if (std::isnan(target)) {
        // Kinematics leaves the buffer untouched on singular IK; hold the
        // last command instead of sending garbage.
        target = last_command_[champ_idx];
      } else {
        target = static_cast<float>(motor_signs_[champ_idx] * target);
        last_command_[champ_idx] = target;
      }
      const float from = start_pose_[motor];
      target = interpolate(from, target, stand_progress);
      auto &motor_cmd = low_cmd_.motor_cmd()[motor];
      motor_cmd.mode() = 0x01;
      motor_cmd.q() = target;
      motor_cmd.dq() = 0.0F;
      motor_cmd.kp() = static_cast<float>(kp_);
      motor_cmd.kd() = static_cast<float>(kd_);
      motor_cmd.tau() = 0.0F;
    }
    low_cmd_.crc() = crc32_core(
        reinterpret_cast<uint32_t *>(&low_cmd_),
        (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    low_cmd_pub_->Write(low_cmd_);
    motion_time_ += kControlPeriod;

    publishFeedback(target_joints);
  }

  void publishFeedback(const float target_joints[kNumLegMotors]) {
    const rclcpp::Time stamp = now();
    if (publish_joint_states_ &&
        (stamp - last_joint_states_time_).seconds() >= 0.02) {
      last_joint_states_time_ = stamp;
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
    if (publish_imu_ && (stamp - last_imu_time_).seconds() >= 0.02) {
      last_imu_time_ = stamp;
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
        (stamp - last_joint_commands_time_).seconds() >= 0.02) {
      last_joint_commands_time_ = stamp;
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

  // CHAMP joint names (unitree_go2_sim config/joints/joints.yaml),
  // in CHAMP order: LF, RF, LH, RH x (hip, upper, lower).
  static const std::vector<std::string> kChampJointNames;

  // CHAMP gait stack.
  champ::GaitConfig gait_config_;
  champ::QuadrupedBase base_;
  champ::BodyController body_controller_;
  champ::LegController leg_controller_;
  champ::Kinematics kinematics_;
  champ::Velocities req_vel_;
  champ::Pose req_pose_;

  std::string knee_orientation_;
  std::string dds_interface_ = "lo";
  int dds_domain_id_ = 1;
  double kp_ = 60.0;
  double kd_ = 5.0;
  double stand_duration_ = 3.0;
  double cmd_vel_timeout_ = 0.5;
  bool publish_joint_states_ = true;
  bool publish_imu_ = true;
  bool publish_joint_control_ = true;
  std::array<double, kNumLegMotors> motor_signs_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  unitree_go::msg::dds_::LowCmd_ low_cmd_{};
  std::array<float, kNumLegMotors> start_pose_{};
  std::array<float, kNumLegMotors> last_command_{};
  std::array<float, kNumLegMotors> measured_q_{};
  std::array<float, 4> imu_quat_{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> imu_gyro_{};
  std::array<float, 3> imu_acc_{};
  std::atomic_bool have_low_state_{false};
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>
      low_cmd_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      low_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      joint_commands_pub_;

  std::mutex state_mutex_;
  std::mutex cmd_mutex_;
  std::atomic_bool running_{true};
  std::thread control_thread_;
  double motion_time_ = 0.0;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_joint_states_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_joint_commands_time_;
};

const std::vector<std::string> Go2ChampWalkController::kChampJointNames = {
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"};

int main(int argc, char **argv) {
  // Initialize Unitree's bundled CycloneDDS BEFORE rclcpp::init() creates the
  // ROS DDS participant. Both stacks target the same domain id; creating the
  // Unitree participant first avoids the "Failed to create domain explicitly"
  // race seen when the ROS participant owns the domain first.
  // Defaults match go2_champ_gait.yaml (domain 1, lo). Optional CLI override:
  //   go2_champ_walk_controller [domain_id] [interface]
  // NOTE: ros2 CLI remap args (--ros-args ...) are filtered out here so they
  // are not mistaken for domain/interface overrides.
  int dds_domain_id = 1;
  std::string dds_interface = "lo";
  {
    std::vector<std::string> plain_args;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--ros-args" || arg == "-r") break;
      plain_args.push_back(arg);
    }
    if (plain_args.size() >= 1) {
      try {
        dds_domain_id = std::stoi(plain_args[0]);
      } catch (const std::exception &) {
        dds_domain_id = 1;
      }
    }
    if (plain_args.size() >= 2) dds_interface = plain_args[1];
  }
  try {
    unitree::robot::ChannelFactory::Instance()->Init(dds_domain_id,
                                                     dds_interface);
  } catch (const std::exception &e) {
    fprintf(stderr, "[go2_champ_walk] Unitree ChannelFactory::Init(%d, %s) "
                    "failed: %s\n",
            dds_domain_id, dds_interface.c_str(), e.what());
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<Go2ChampWalkController>(dds_domain_id, dds_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

