// Sport-mode state reader for the Go2 (mirrors read_motion_state.cpp).
//
// Subscribes "lf/sportmodestate" (low frequency) or "sportmodestate"
// (500 Hz high frequency), caches the latest odometry/gait/foot feedback
// under its own mutex, and throttled-logs it. Read-only: it never feeds the
// CHAMP control loop, so it is safe to keep enabled alongside /lowstate.
#ifndef GO2_SPORT_STATE_READER_H_
#define GO2_SPORT_STATE_READER_H_

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace go2 {

struct SportStateConfig {
  bool enable = true;         // master switch for the subscription
  bool high_freq = false;     // true -> "sportmodestate" (500 Hz),
                              // false -> "lf/sportmodestate" (low freq)
  bool log_foot = true;       // per-foot detail in the throttled log
  double log_period = 1.0;    // seconds between log bursts
};

class SportStateReader {
 public:
  // Subscribes only when cfg.enable. topic / log behaviour come from cfg.
  SportStateReader(rclcpp::Node *node, const SportStateConfig &cfg);

  // Latest cached values (safe to call from the control thread).
  bool received() const;
  uint8_t gaitType() const;
  float footRaiseHeight() const;
  std::array<float, 3> position() const;
  float bodyHeight() const;
  std::array<float, 3> velocity() const;
  float yawSpeed() const;
  std::array<float, 12> footPositionBody() const;
  std::array<float, 12> footSpeedBody() const;

 private:
  void onSportModeState(const unitree_go::msg::SportModeState::SharedPtr &data);

  bool log_foot_ = true;
  double log_period_ = 1.0;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_;
  rclcpp::Logger logger_;
  mutable std::mutex mutex_;
  bool received_ = false;
  uint8_t gait_type_ = 0;
  float foot_raise_height_ = 0.0F;
  std::array<float, 3> position_{};
  float body_height_ = 0.0F;
  std::array<float, 3> velocity_{};
  float yaw_speed_ = 0.0F;
  std::array<float, 12> foot_position_body_{};
  std::array<float, 12> foot_speed_body_{};
  std::chrono::steady_clock::time_point last_log_steady_{};
};

}  // namespace go2

#endif  // GO2_SPORT_STATE_READER_H_
