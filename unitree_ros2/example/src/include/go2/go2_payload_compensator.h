// Payload-aware CoM compensation for the D1 arm on Go2.
//
// Extracted from Go2ChampWalkController::applyPayloadCompensation(). The
// controller keeps the ROS-owned pieces (arm-q fusion under its state mutex,
// IMU read, debug publishers) and feeds this class:
//   - fused arm q (radians, D1 Joint1..6),
//   - measured body pitch (rad) and pitch rate gyro_y (rad/s),
//   - the control-loop period.
// and gets back the four trims that stepOnce() applies. All low-pass filter
// and rate-limit state lives here, so the estimator is unit-testable in
// isolation (no rclcpp dependency).
#ifndef GO2_PAYLOAD_COMPENSATOR_H_
#define GO2_PAYLOAD_COMPENSATOR_H_

#include <array>

namespace go2 {

constexpr int kNumArmJointsPayload = 6;

struct PayloadCompResult {
  double trim_y = 0.0;        // lateral body shift (m)
  double trim_z = 0.0;        // body height trim (m)
  double trim_pitch_ff = 0.0; // feed-forward pitch trim (rad)
  double trim_pitch_imu = 0.0;  // IMU regulator pitch trim (rad)
  double com_x_translation = 0.0;  // live gait_config_ knob (m)
};

struct PayloadCompInput {
  std::array<float, kNumArmJointsPayload> q{};  // fused arm q (rad)
  double pitch = 0.0;                            // measured body pitch (rad)
  double gyro_y = 0.0;                           // IMU pitch rate (rad/s)
  double control_period = 0.002;                 // loop dt (s)
};

class PayloadCompensator {
 public:
  PayloadCompensator() = default;

  // Seed the low-pass filter at zero delta so startup is bumpless: the live
  // knob only reacts to motion AWAY from the tucked pose. The tucked arm mass
  // sits ~8mm behind the bare-torso CoM, so the tucked pose itself must read
  // as zero delta — otherwise a constant rearward bias leaks in even on the
  // no-arm model, dragging every step rearward.
  void seed(double base_com_x, double nominal_height);

  // Configure gains/limits (called once from loadParams()).
  void configure(bool enable, double com_alpha, double com_max,
                 double com_rate_limit, double pitch_gain,
                 double lateral_gain, bool imu_fb_enable, double imu_fb_kp,
                 double imu_fb_kd, double imu_fb_max);

  bool enabled() const { return enable_; }

  // One 500 Hz iteration. Returns the trims to apply and the live
  // com_x_translation to write back into gait_config_.
  PayloadCompResult compute(const PayloadCompInput &in);

 private:
  bool enable_ = true;
  double com_alpha_ = 0.08;
  double com_max_ = 0.05;
  double com_rate_limit_ = 0.02;
  double pitch_gain_ = 0.5;
  double lateral_gain_ = 1.0;

  bool imu_fb_enable_ = false;
  double imu_fb_kp_ = 0.3;
  double imu_fb_kd_ = 0.05;
  double imu_fb_max_ = 0.08;

  // Baseline captured after seed(): yaml static value + tucked CoM.
  double com_x_base_ = 0.0;
  double tucked_com_x_ = 0.021112;
  double tucked_com_z_ = 0.0;
  double nominal_height_base_ = 0.225;
  // Low-pass filtered CoM deltas from the tucked pose (normal operating point).
  double filt_com_x_ = 0.0;
  double filt_com_y_ = 0.0;
  double filt_com_z_ = 0.0;
  double prev_com_x_applied_ = 0.0;
};

}  // namespace go2

#endif  // GO2_PAYLOAD_COMPENSATOR_H_
