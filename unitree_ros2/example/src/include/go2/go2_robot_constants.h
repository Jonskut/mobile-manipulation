// Shared constants, index mappings and small math helpers for the CHAMP Go2
// ROS 2 controller. Extracted from the anonymous namespace that used to sit
// at the top of go2_champ_ros2_controller.cpp so the payload compensator and
// unit tests can reuse them without dragging in the whole node.
#ifndef GO2_ROBOT_CONSTANTS_H_
#define GO2_ROBOT_CONSTANTS_H_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace go2 {

inline constexpr int kNumLegMotors = 12;
inline constexpr int kNumLowCmdMotors = 20;
inline constexpr int kFirstArmMotor = 12;  // legs 0..11, D1 arm 12..17 in LowState
inline constexpr int kNumArmJoints = 6;
inline constexpr double kControlPeriod = 0.002;  // 500 Hz, matches DDS bridge rate
// Arm q is fused from /lowstate motor_state[12..17] (sim) and
// current_servo_angle / d1_joint_states (real robot + sim).
inline constexpr double kDeg2Rad = 0.017453292519943295;

// CHAMP leg index -> Unitree LowCmd/LowState motor index.
inline constexpr std::array<int, kNumLegMotors> kChampToUnitreeMotor = {
    3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8,
};

// Go2 leg geometry (hip / thigh / calf), metres. Same values as the Go2 URDF.
inline constexpr float kHipOffsetX = 0.1934F;
inline constexpr float kHipOffsetY = 0.0465F;
inline constexpr float kThighOffsetY = 0.0955F;
inline constexpr float kThighLength = 0.213F;
inline constexpr float kCalfLength = 0.213F;

// Joint-name tables: CHAMP ordering and Unitree ordering respectively.
inline const std::vector<std::string> kChampJointNames = {
    "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
    "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
    "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
    "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"};
inline const std::vector<std::string> kUnitreeJointNames = {
    "fr_hip_joint", "fr_thigh_joint", "fr_calf_joint",
    "fl_hip_joint", "fl_thigh_joint", "fl_calf_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
    "rl_hip_joint", "rl_thigh_joint", "rl_calf_joint"};

inline float interpolate(float start, float target, double progress) {
  const double c = std::min(std::max(progress, 0.0), 1.0);
  return static_cast<float>((1.0 - c) * start + c * target);
}

inline void quaternionToRpy(double x, double y, double z, double w, double &roll,
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

}  // namespace go2

#endif  // GO2_ROBOT_CONSTANTS_H_
