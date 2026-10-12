/* CHAMP gait controller: ROS-2-only Go2 locomotion (no DDS bridge, no UDP).
 * Clone of go2_champ_gait_planner.cpp with the loopback-UDP transport to
 * go2_champ_dds_bridge replaced by native ROS 2 topics, exactly like
 * go2_stand_example.cpp / unitree_mujoco/example/ros2/src/stand_go2.cpp:
 *   PUB /lowcmd  (unitree_go::msg::LowCmd, 500 Hz, crc via get_crc())
 *   SUB /lowstate (unitree_go::msg::LowState -> measured q + IMU + arm q)
 * Works against both the real Go2 and unitree_mujoco (rmw_cyclonedds_cpp maps
 * ROS /lowcmd <-> DDS rt/lowcmd on the same domain/interface).
 * Requires: RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, ROS_DOMAIN_ID matching the
 * sim/robot, CYCLONEDDS_URI pointing at the right interface (lo in sim).
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "motor_crc.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "unitree_arm/msg/pub_servo_info.hpp"

#include <champ/body_controller/body_controller.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>
#include <champ/payload/arm_com_estimator.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace {
constexpr int kNumLegMotors = 12;
constexpr int kNumLowCmdMotors = 20;
constexpr int kFirstArmMotor = 12;  // legs 0..11, D1 arm 12..17 in LowState
constexpr int kNumArmJoints = 6;
constexpr double kControlPeriod = 0.002;  // 500 Hz, matches DDS bridge rate
// Arm q is fused from /lowstate motor_state[12..17] (sim) and
// current_servo_angle / d1_joint_states (real robot + sim).
constexpr double kDeg2Rad = 0.017453292519943295;
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
    // Payload-aware CoM compensation (D1 arm, tucked-carry default).
    declare_parameter("payload.enable", true);
    declare_parameter("payload.com_alpha", 0.08);
    declare_parameter("payload.com_max", 0.05);
    declare_parameter("payload.com_rate_limit", 0.02);
    declare_parameter("payload.arm_timeout", 1.0);
    declare_parameter("payload.tucked_q_deg",
                      std::vector<double>({0.0, -60.0, 60.0, 0.0, 30.0, 0.0}));
    declare_parameter("payload.base_com_x", 0.021112);
    declare_parameter("payload.pitch_gain", 0.5);
    declare_parameter("payload.lateral_gain", 1.0);
    declare_parameter("imu_pitch_feedback.enable", false);
    declare_parameter("imu_pitch_feedback.kp", 0.3);
    declare_parameter("imu_pitch_feedback.kd", 0.05);
    declare_parameter("imu_pitch_feedback.max", 0.08);
    loadParams();
    gait_config_.knee_orientation = knee_orientation_.c_str();
    setLegGeometry();
    base_.setGaitConfig(gait_config_);
    req_pose_.position.z = gait_config_.nominal_height;
    initLowCmd();
    // Native ROS 2 transport (same pattern as go2_stand_example.cpp):
    //   PUB /lowcmd at 500 Hz from the control thread,
    //   SUB /lowstate fills measured_q_/IMU/arm q (replaces UDP 17611).
    // rmw_cyclonedds_cpp maps ROS /lowcmd <-> DDS rt/lowcmd on the same
    // domain/interface, so this reaches both unitree_mujoco and the real Go2.
    low_cmd_pub_ =
        create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    low_state_sub_ =
        create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10,
            [this](const unitree_go::msg::LowState::SharedPtr msg) {
              if (msg->motor_state.size() <
                  static_cast<size_t>(kFirstArmMotor + kNumArmJoints)) {
                return;
              }
              std::lock_guard<std::mutex> lock(state_mutex_);
              for (int i = 0; i < kNumLegMotors; ++i) {
                measured_q_[i] = msg->motor_state[i].q;
              }
              imu_quat_[0] = msg->imu_state.quaternion[0];
              imu_quat_[1] = msg->imu_state.quaternion[1];
              imu_quat_[2] = msg->imu_state.quaternion[2];
              imu_quat_[3] = msg->imu_state.quaternion[3];
              imu_gyro_[0] = msg->imu_state.gyroscope[0];
              imu_gyro_[1] = msg->imu_state.gyroscope[1];
              imu_gyro_[2] = msg->imu_state.gyroscope[2];
              imu_acc_[0] = msg->imu_state.accelerometer[0];
              imu_acc_[1] = msg->imu_state.accelerometer[1];
              imu_acc_[2] = msg->imu_state.accelerometer[2];
              bool arm_nonzero = false;
              for (int i = 0; i < kNumArmJoints; ++i) {
                arm_q_lowstate_[i] =
                    msg->motor_state[kFirstArmMotor + i].q;
                if (std::fabs(arm_q_lowstate_[i]) > 1.0e-6F) {
                  arm_nonzero = true;
                }
              }
              if (arm_nonzero) {
                arm_lowstate_received_ = true;
                last_arm_lowstate_ = std::chrono::steady_clock::now();
              }
              have_state_.store(true, std::memory_order_release);
              const rclcpp::Time stamp = now();
              sensor_msgs::msg::JointState js;
              js.header.stamp = stamp;
              js.name = kUnitreeJointNames;
              js.position.resize(kNumLegMotors);
              for (int i = 0; i < kNumLegMotors; ++i) {
                js.position[i] = msg->motor_state[i].q;
              }
              measured_pub_->publish(js);
              sensor_msgs::msg::Imu imu;
              imu.header.stamp = stamp;
              imu.header.frame_id = "imu_link";
              imu.orientation.w = msg->imu_state.quaternion[0];
              imu.orientation.x = msg->imu_state.quaternion[1];
              imu.orientation.y = msg->imu_state.quaternion[2];
              imu.orientation.z = msg->imu_state.quaternion[3];
              imu.angular_velocity.x = msg->imu_state.gyroscope[0];
              imu.angular_velocity.y = msg->imu_state.gyroscope[1];
              imu.angular_velocity.z = msg->imu_state.gyroscope[2];
              imu.linear_acceleration.x = msg->imu_state.accelerometer[0];
              imu.linear_acceleration.y = msg->imu_state.accelerometer[1];
              imu.linear_acceleration.z = msg->imu_state.accelerometer[2];
              imu_measured_pub_->publish(imu);
            });
    joint_targets_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_targets", 10);
    measured_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states_measured", 10);
    imu_measured_pub_ =
        create_publisher<sensor_msgs::msg::Imu>("imu_measured", 10);
    measured_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_measured", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          // Debug/introspection republish path (see /lowstate handler above):
          // external joint_states_measured still honoured when /lowstate is
          // absent (e.g. rosbag replay without the robot/sim).
          if (have_state_.load(std::memory_order_acquire)) return;
          if (msg->position.size() < kNumLegMotors) return;
          std::lock_guard<std::mutex> lock(state_mutex_);
          for (int i = 0; i < kNumLegMotors; ++i)
            measured_q_[i] = static_cast<float>(msg->position[i]);
          have_state_.store(true, std::memory_order_release);
        });
    imu_measured_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu_measured", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          if (have_state_.load(std::memory_order_acquire)) return;
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
    payload_com_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "payload_com", 10);
    // Applied trims for tuning: [com_x_translation, trim_y, trim_z,
    // trim_pitch_ff, trim_pitch_imu, pitch_measured] at the same 50 Hz.
    payload_trims_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "payload_trims", 10);
    arm_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "d1_joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          // Redundant ROS path A: external /d1_joint_states, radians.
          if (msg->position.size() < static_cast<size_t>(kNumArmJoints)) return;
          std::lock_guard<std::mutex> lock(state_mutex_);
          for (int i = 0; i < kNumArmJoints; ++i)
            arm_q_ros_[i] = static_cast<float>(msg->position[i]);
          arm_ros_received_ = true;
          last_arm_ros_ = std::chrono::steady_clock::now();
        });
    // Redundant ROS path B: D1 arm feedback topic (degrees). Published by
    // unitree_mujoco's bridge AND by the real D1 controller — this is the
    // only reliable arm source on the real robot (Go2 LowState arm slots
    // are only filled by the simulator). ROS msg type name matches the
    // Unitree DDS type (unitree_arm::msg::dds_::PubServoInfo_), so
    // rmw_cyclonedds_cpp sees the raw Unitree topic on the same domain.
    // NOTE: subscription lives here (ROS-only process), NOT in
    // go2_d1_stand_controller: that binary loads Unitree's libddsc and
    // adding rclcpp to it would hit the CycloneDDS clash (CMakeLists).
    current_servo_angle_sub_ =
        create_subscription<unitree_arm::msg::PubServoInfo>(
            "current_servo_angle", 10,
            [this](const unitree_arm::msg::PubServoInfo::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(state_mutex_);
              arm_q_ros_[0] = msg->servo0_data * kDeg2Rad;
              arm_q_ros_[1] = msg->servo1_data * kDeg2Rad;
              arm_q_ros_[2] = msg->servo2_data * kDeg2Rad;
              arm_q_ros_[3] = msg->servo3_data * kDeg2Rad;
              arm_q_ros_[4] = msg->servo4_data * kDeg2Rad;
              arm_q_ros_[5] = msg->servo5_data * kDeg2Rad;
              arm_ros_received_ = true;
              last_arm_ros_ = std::chrono::steady_clock::now();
            });
    RCLCPP_INFO(get_logger(), "CHAMP ROS2 controller ready (PUB /lowcmd, SUB /lowstate)");
    control_thread_ = std::thread([this]() { controlLoop(); });
  }
  ~Go2ChampWalkController() override {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
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
    get_parameter("payload.enable", payload_enable_);
    get_parameter("payload.com_alpha", payload_alpha_);
    get_parameter("payload.com_max", payload_com_max_);
    get_parameter("payload.com_rate_limit", payload_rate_limit_);
    get_parameter("payload.arm_timeout", payload_arm_timeout_);
    get_parameter("payload.base_com_x", payload_base_com_x_);
    get_parameter("payload.pitch_gain", payload_pitch_gain_);
    get_parameter("payload.lateral_gain", payload_lateral_gain_);
    std::vector<double> tucked(6, 0.0);
    get_parameter("payload.tucked_q_deg", tucked);
    if (tucked.size() == 6) {
      for (int i = 0; i < 6; ++i)
        tucked_q_rad_[i] = static_cast<float>(tucked[i] * M_PI / 180.0);
    } else {
      tucked_q_rad_ = champ::payload::tuckedQRad();
    }
    // Seed the filter at zero delta so startup is bumpless: the live knob only
    // reacts to motion AWAY from the tucked pose. The tucked arm mass sits
    // ~8mm behind the bare-torso CoM, so the tucked pose itself must read as
    // zero delta — otherwise a constant rearward bias leaks in even on the
    // no-arm model, dragging every step rearward.
    {
      const Eigen::Vector3f c0 =
          champ::payload::computeTotalCom(tucked_q_rad_);
      filt_com_x_ = 0.0;
      filt_com_y_ = 0.0;
      filt_com_z_ = static_cast<double>(c0.z());
      tucked_com_x_ = static_cast<double>(c0.x());
      tucked_com_z_ = static_cast<double>(c0.z());
      // yaml static baseline is preserved; live knob = base + filtered delta.
      com_x_base_ = gait_config_.com_x_translation;
      nominal_height_base_ = gait_config_.nominal_height;
    }
    get_parameter("imu_pitch_feedback.enable", imu_fb_enable_);
    get_parameter("imu_pitch_feedback.kp", imu_fb_kp_);
    get_parameter("imu_pitch_feedback.kd", imu_fb_kd_);
    get_parameter("imu_pitch_feedback.max", imu_fb_max_);
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
  void initLowCmd() {
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = 0xFF;
    low_cmd_.gpio = 0;
    for (int i = 0; i < kNumLowCmdMotors; i++) {
      low_cmd_.motor_cmd[i].mode = 0x01;  // servo (PMSM) mode
      low_cmd_.motor_cmd[i].q = static_cast<float>(PosStopF);
      low_cmd_.motor_cmd[i].kp = 0.0F;
      low_cmd_.motor_cmd[i].dq = static_cast<float>(VelStopF);
      low_cmd_.motor_cmd[i].kd = 0.0F;
      low_cmd_.motor_cmd[i].tau = 0.0F;
    }
  }
  void controlLoop() {
    RCLCPP_INFO(get_logger(), "Waiting for /lowstate ...");
    while (running_ && !have_state_.load(std::memory_order_acquire)) {
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
      stepOnce();
      std::this_thread::sleep_until(next_tick);
    }
  }
  void stepOnce() {
    float target_joints[kNumLegMotors];
    geometry::Transformation target_feet[4];
    if (!have_state_.load(std::memory_order_acquire)) return;
    applyPayloadCompensation();
    champ::Velocities vel;
    champ::Pose pose;
    bool vel_fresh;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vel = req_vel_;
      pose = req_pose_;
      // Payload trims (computed in applyPayloadCompensation from live arm q):
      // Δx shifts the support polygon via com_x_translation (live knob),
      // Δy shifts the body laterally, Δz/pitch trims height and lean.
      pose.position.y += static_cast<float>(payload_trim_y_);
      pose.position.z += static_cast<float>(payload_trim_z_);
      pose.orientation.pitch +=
          static_cast<float>(payload_trim_pitch_ + imu_trim_pitch_);
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
    publishLowCmd(unitree_targets);
    sensor_msgs::msg::JointState targets_msg;
    targets_msg.header.stamp = now();
    targets_msg.name = kUnitreeJointNames;
    targets_msg.position.assign(unitree_targets.begin(), unitree_targets.end());
    joint_targets_pub_->publish(targets_msg);
    publishFeedback(target_joints);
  }

  // Payload-aware CoM compensation @500Hz (quasi-static, tucked-optimized).
  // FK over live arm q -> total CoM in base frame -> low-pass -> clamped,
  // rate-limited trims. gait_config_ is pointer-shared to all 4 legs, so
  // com_x_translation takes effect on the very next poseCommand().
  // Fusion: freshest valid of (/lowstate arm q, ROS arm feedback) else
  // tucked hold.
  void applyPayloadCompensation() {
    if (!payload_enable_) {
      payload_trim_y_ = payload_trim_z_ = payload_trim_pitch_ = 0.0;
      imu_trim_pitch_ = 0.0;
      return;
    }
    // Source fusion: /lowstate arm q (sim fills motor_state[12..17]) vs ROS
    // arm feedback (real robot; sim publishes it too). Freshness decides; an
    // all-zero lowstate arm packet is treated as "slots not filled" (real Go2
    // leaves motor_state[12..17] at 0, filtered at the subscriber).
    // Tucked hold when both are stale/absent.
    std::array<float, kNumArmJoints> q{};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const double age_lowstate =
          std::chrono::duration<double>(now - last_arm_lowstate_).count();
      const double age_ros =
          std::chrono::duration<double>(now - last_arm_ros_).count();
      const bool lowstate_valid =
          arm_lowstate_received_ && age_lowstate <= payload_arm_timeout_;
      const bool ros_valid =
          arm_ros_received_ && age_ros <= payload_arm_timeout_;
      if (lowstate_valid && (!ros_valid || age_lowstate <= age_ros)) {
        q = arm_q_lowstate_;
      } else if (ros_valid) {
        q = arm_q_ros_;
      } else {
        q = tucked_q_rad_;
      }
    }
    const Eigen::Vector3f com = champ::payload::computeTotalCom(q);
    // Delta is measured from the TUCKED pose, not the bare torso: the tucked
    // arm mass is the normal operating point, so holding tucked must read as
    // zero trim. (Bare-torso reference would inject a constant ~8mm rearward
    // bias)
    const double raw_dx = static_cast<double>(com.x()) - tucked_com_x_;
    const double raw_dy = static_cast<double>(com.y());
    const double raw_dz = static_cast<double>(com.z());
    const double a = std::min(std::max(payload_alpha_, 0.0), 1.0);
    filt_com_x_ += a * (raw_dx - filt_com_x_);
    filt_com_y_ += a * (raw_dy - filt_com_y_);
    filt_com_z_ += a * (raw_dz - filt_com_z_);
    const double max_step = payload_rate_limit_ * kControlPeriod;
    double dx = filt_com_x_;
    dx = std::min(std::max(dx, -payload_com_max_), payload_com_max_);
    double step = dx - prev_com_x_applied_;
    step = std::min(std::max(step, -max_step), max_step);
    dx = prev_com_x_applied_ + step;
    prev_com_x_applied_ = dx;
    gait_config_.com_x_translation =
        static_cast<float>(com_x_base_ + dx);
    payload_trim_y_ =
        -std::min(std::max(filt_com_y_ * payload_lateral_gain_,
                           -payload_com_max_),
                  payload_com_max_);
    payload_trim_z_ =
        std::min(std::max((filt_com_z_ - tucked_com_z_) * 0.5,
                          -payload_com_max_ * 0.5),
                 payload_com_max_ * 0.5);
    payload_trim_pitch_ =
        std::min(std::max(-dx * payload_pitch_gain_, -0.08), 0.08);
    // Optional IMU pitch regulator (uses already-available /lowstate IMU).
    if (imu_fb_enable_) {
      double roll, pitch, yaw;
      double gyro_y;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        quaternionToRpy(imu_quat_[1], imu_quat_[2], imu_quat_[3],
                        imu_quat_[0], roll, pitch, yaw);
        gyro_y = static_cast<double>(imu_gyro_[1]);
      }
      double cmd = -(imu_fb_kp_ * pitch + imu_fb_kd_ * gyro_y);
      imu_trim_pitch_ = std::min(std::max(cmd, -imu_fb_max_), imu_fb_max_);
    } else {
      imu_trim_pitch_ = 0.0;
    }
    // Debug CoM + applied-trim topics, throttled to ~50 Hz (loop runs 500 Hz).
    if (++payload_dbg_div_ >= 10) {
      payload_dbg_div_ = 0;
      geometry_msgs::msg::PointStamped dbg;
      dbg.header.stamp = now();
      dbg.header.frame_id = "base_link";
      dbg.point.x = com.x();
      dbg.point.y = com.y();
      dbg.point.z = com.z();
      payload_com_pub_->publish(dbg);
      double roll = 0.0, pitch = 0.0, yaw = 0.0;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        quaternionToRpy(imu_quat_[1], imu_quat_[2], imu_quat_[3],
                        imu_quat_[0], roll, pitch, yaw);
      }
      std_msgs::msg::Float64MultiArray trims;
      trims.data = {static_cast<double>(gait_config_.com_x_translation),
                    payload_trim_y_, payload_trim_z_, payload_trim_pitch_,
                    imu_trim_pitch_, pitch};
      payload_trims_pub_->publish(trims);
    }
  }

  // Publish 12 CHAMP targets as ROS /lowcmd (Unitree motor order FR, FL, RR,
  // RL). Mirrors go2_champ_dds_bridge: mode 0x01 servo, dq 0, tau 0, crc.
  void publishLowCmd(const std::array<float, kNumLegMotors> &unitree_targets) {
    if (!low_cmd_pub_) return;
    unitree_go::msg::LowCmd msg = low_cmd_;
    for (int i = 0; i < kNumLegMotors; ++i) {
      msg.motor_cmd[i].mode = 0x01;
      msg.motor_cmd[i].q = unitree_targets[i];
      msg.motor_cmd[i].dq = 0.0F;
      msg.motor_cmd[i].kp = static_cast<float>(kp_);
      msg.motor_cmd[i].kd = static_cast<float>(kd_);
      msg.motor_cmd[i].tau = 0.0F;
    }
    get_crc(msg);
    low_cmd_pub_->publish(msg);
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
  // --- Payload-aware CoM compensation state (D1 arm) ---
  // Two live sources, each with its own stamp; fused in
  // applyPayloadCompensation() (freshest-valid wins, else tucked hold).
  std::array<float, kNumArmJoints> arm_q_lowstate_{};  // /lowstate[12..17] (rad)
  std::array<float, kNumArmJoints> arm_q_ros_{};  // from ROS topics (radians)
  bool arm_lowstate_received_ = false;
  bool arm_ros_received_ = false;
  std::chrono::steady_clock::time_point last_arm_lowstate_{};
  std::chrono::steady_clock::time_point last_arm_ros_{};
  bool payload_enable_ = true;
  double payload_alpha_ = 0.08;
  double payload_com_max_ = 0.05;
  double payload_rate_limit_ = 0.02;
  double payload_arm_timeout_ = 1.0;
  double payload_base_com_x_ = 0.021112;
  double payload_pitch_gain_ = 0.5;
  double payload_lateral_gain_ = 1.0;
  std::array<float, kNumArmJoints> tucked_q_rad_{};
  // Baseline captured after loadParams(): yaml static value + tucked CoM.
  double com_x_base_ = 0.0;
  double tucked_com_x_ = 0.021112;
  double tucked_com_z_ = 0.0;
  double nominal_height_base_ = 0.225;
  // Low-pass filtered CoM deltas from the tucked pose (normal operating point).
  double filt_com_x_ = 0.0;
  double filt_com_y_ = 0.0;
  double filt_com_z_ = 0.0;
  double prev_com_x_applied_ = 0.0;
  // Trims consumed by stepOnce() under cmd_mutex_.
  double payload_trim_y_ = 0.0;
  double payload_trim_z_ = 0.0;
  double payload_trim_pitch_ = 0.0;
  double imu_trim_pitch_ = 0.0;
  // IMU pitch regulator (Phase 3, enabled by default: the forward-walk
  // pitch-back is a dynamic gait effect the feed-forward cannot see).
  bool imu_fb_enable_ = false;
  double imu_fb_kp_ = 0.3;
  double imu_fb_kd_ = 0.05;
  double imu_fb_max_ = 0.08;
  int payload_dbg_div_ = 0;
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
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr payload_com_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      payload_trims_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub_;
  rclcpp::Subscription<unitree_arm::msg::PubServoInfo>::SharedPtr
      current_servo_angle_sub_;
  std::mutex state_mutex_;
  std::mutex cmd_mutex_;
  std::atomic_bool running_{true};
  std::thread control_thread_;
  unitree_go::msg::LowCmd low_cmd_{};  // template (header/level/gpio + stops)
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
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

