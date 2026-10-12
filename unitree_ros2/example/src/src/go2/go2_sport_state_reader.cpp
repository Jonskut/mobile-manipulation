#include "go2/go2_sport_state_reader.h"

namespace go2 {

SportStateReader::SportStateReader(rclcpp::Node *node,
                                   const SportStateConfig &cfg)
    : log_foot_(cfg.log_foot),
      log_period_(cfg.log_period),
      logger_(node->get_logger()) {
  if (!cfg.enable) {
    return;
  }
  const char *topic = cfg.high_freq ? "sportmodestate" : "lf/sportmodestate";
  sub_ = node->create_subscription<unitree_go::msg::SportModeState>(
      topic, 10,
      [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
        onSportModeState(msg);
      });
  RCLCPP_INFO(logger_, "SportModeState reader: SUB %s", topic);
}

void SportStateReader::onSportModeState(
    const unitree_go::msg::SportModeState::SharedPtr &data) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gait_type_ = data->gait_type;
    foot_raise_height_ = data->foot_raise_height;
    for (int i = 0; i < 3; ++i) {
      position_[i] = data->position[i];
      velocity_[i] = data->velocity[i];
    }
    body_height_ = data->body_height;
    yaw_speed_ = data->yaw_speed;
    for (int i = 0; i < 12; ++i) {
      foot_position_body_[i] = data->foot_position_body[i];
      foot_speed_body_[i] = data->foot_speed_body[i];
    }
    received_ = true;
  }
  // Throttled INFO log, same content as read_motion_state.cpp. The
  // high-frequency topic runs at 500 Hz, so never log every callback.
  const auto now_steady = std::chrono::steady_clock::now();
  if (last_log_steady_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now_steady - last_log_steady_).count() <
          log_period_) {
    return;
  }
  last_log_steady_ = now_steady;
  RCLCPP_INFO(logger_, "SportModeState -- gait type: %d; raise height: %f",
              data->gait_type, data->foot_raise_height);
  RCLCPP_INFO(logger_,
              "SportModeState position -- x: %f; y: %f; z: %f; body height: %f",
              data->position[0], data->position[1], data->position[2],
              data->body_height);
  RCLCPP_INFO(logger_,
              "SportModeState velocity -- vx: %f; vy: %f; vz: %f; yaw: %f",
              data->velocity[0], data->velocity[1], data->velocity[2],
              data->yaw_speed);
  if (log_foot_) {
    for (int leg = 0; leg < 4; ++leg) {
      RCLCPP_INFO(logger_,
                  "SportModeState foot rel body -- num: %d; x: %f; y: %f; "
                  "z: %f, vx: %f; vy: %f; vz: %f",
                  leg, data->foot_position_body[leg * 3],
                  data->foot_position_body[leg * 3 + 1],
                  data->foot_position_body[leg * 3 + 2],
                  data->foot_speed_body[leg * 3],
                  data->foot_speed_body[leg * 3 + 1],
                  data->foot_speed_body[leg * 3 + 2]);
    }
  }
}

bool SportStateReader::received() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return received_;
}

uint8_t SportStateReader::gaitType() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return gait_type_;
}

float SportStateReader::footRaiseHeight() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return foot_raise_height_;
}

std::array<float, 3> SportStateReader::position() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return position_;
}

float SportStateReader::bodyHeight() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return body_height_;
}

std::array<float, 3> SportStateReader::velocity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return velocity_;
}

float SportStateReader::yawSpeed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return yaw_speed_;
}

std::array<float, 12> SportStateReader::footPositionBody() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return foot_position_body_;
}

std::array<float, 12> SportStateReader::footSpeedBody() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return foot_speed_body_;
}

}  // namespace go2
