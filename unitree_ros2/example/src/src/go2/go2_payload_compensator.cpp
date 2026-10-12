#include "go2/go2_payload_compensator.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>

#include <champ/payload/arm_com_estimator.h>

namespace go2 {

void PayloadCompensator::seed(double base_com_x, double nominal_height) {
  const std::array<float, kNumArmJointsPayload> tucked =
      champ::payload::tuckedQRad();
  const Eigen::Vector3f c0 = champ::payload::computeTotalCom(tucked);
  filt_com_x_ = 0.0;
  filt_com_y_ = 0.0;
  filt_com_z_ = static_cast<double>(c0.z());
  tucked_com_x_ = static_cast<double>(c0.x());
  tucked_com_z_ = static_cast<double>(c0.z());
  // yaml static baseline is preserved; live knob = base + filtered delta.
  com_x_base_ = base_com_x;
  nominal_height_base_ = nominal_height;
}

void PayloadCompensator::configure(bool enable, double com_alpha, double com_max,
                                   double com_rate_limit, double pitch_gain,
                                   double lateral_gain, bool imu_fb_enable,
                                   double imu_fb_kp, double imu_fb_kd,
                                   double imu_fb_max) {
  enable_ = enable;
  com_alpha_ = com_alpha;
  com_max_ = com_max;
  com_rate_limit_ = com_rate_limit;
  pitch_gain_ = pitch_gain;
  lateral_gain_ = lateral_gain;
  imu_fb_enable_ = imu_fb_enable;
  imu_fb_kp_ = imu_fb_kp;
  imu_fb_kd_ = imu_fb_kd;
  imu_fb_max_ = imu_fb_max;
}

PayloadCompResult PayloadCompensator::compute(const PayloadCompInput &in) {
  PayloadCompResult out;
  if (!enable_) {
    return out;  // all trims zero
  }
  // FK over the (already fused) arm q -> total CoM in base frame.
  const Eigen::Vector3f com = champ::payload::computeTotalCom(in.q);
  // Delta is measured from the TUCKED pose, not the bare torso: the tucked
  // arm mass is the normal operating point, so holding tucked must read as
  // zero trim. (Bare-torso reference would inject a constant ~8mm rearward
  // bias.)
  const double raw_dx = static_cast<double>(com.x()) - tucked_com_x_;
  const double raw_dy = static_cast<double>(com.y());
  const double raw_dz = static_cast<double>(com.z());
  const double a = std::min(std::max(com_alpha_, 0.0), 1.0);
  filt_com_x_ += a * (raw_dx - filt_com_x_);
  filt_com_y_ += a * (raw_dy - filt_com_y_);
  filt_com_z_ += a * (raw_dz - filt_com_z_);
  const double max_step = com_rate_limit_ * in.control_period;
  double dx = filt_com_x_;
  dx = std::min(std::max(dx, -com_max_), com_max_);
  double step = dx - prev_com_x_applied_;
  step = std::min(std::max(step, -max_step), max_step);
  dx = prev_com_x_applied_ + step;
  prev_com_x_applied_ = dx;
  out.com_x_translation = com_x_base_ + dx;
  out.trim_y =
      -std::min(std::max(filt_com_y_ * lateral_gain_, -com_max_), com_max_);
  out.trim_z =
      std::min(std::max((filt_com_z_ - tucked_com_z_) * 0.5,
                        -com_max_ * 0.5),
               com_max_ * 0.5);
  out.trim_pitch_ff = std::min(std::max(-dx * pitch_gain_, -0.08), 0.08);
  // Optional IMU pitch regulator (uses already-available /lowstate IMU).
  if (imu_fb_enable_) {
    const double cmd = -(imu_fb_kp_ * in.pitch + imu_fb_kd_ * in.gyro_y);
    out.trim_pitch_imu = std::min(std::max(cmd, -imu_fb_max_), imu_fb_max_);
  }
  return out;
}

}  // namespace go2
