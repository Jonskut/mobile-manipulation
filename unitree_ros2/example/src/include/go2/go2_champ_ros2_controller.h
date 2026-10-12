// CHAMP gait controller: ROS-2-only Go2 locomotion (no DDS bridge, no UDP).
//   PUB /lowcmd  (unitree_go::msg::LowCmd, 500 Hz, crc via get_crc())
//   SUB /lowstate (unitree_go::msg::LowState -> measured q + IMU + arm q)
//
// The node is an orchestrator: it wires ROS I/O, fuses arm-q / IMU inputs
// under state_mutex_, delegates payload CoM compensation to
// go2::PayloadCompensator and sport-state reading to go2::SportStateReader,
// then runs the CHAMP gait/IK core and publishes. Shared constants and small
// math helpers live in go2_robot_constants.h.
#ifndef GO2_CHAMP_ROS2_CONTROLLER_H_
#define GO2_CHAMP_ROS2_CONTROLLER_H_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "unitree_arm/msg/pub_servo_info.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include <champ/body_controller/body_controller.h>
#include <champ/kinematics/kinematics.h>
#include <champ/leg_controller/leg_controller.h>

#include "go2/go2_payload_compensator.h"
#include "go2/go2_robot_constants.h"
#include "go2/go2_sport_state_reader.h"

class Go2ChampWalkController : public rclcpp::Node {
 public:
  Go2ChampWalkController();
  ~Go2ChampWalkController() override;

 private:
  void loadParams();
  void setLegGeometry();
  void initLowCmd();
  void controlLoop();
  void stepOnce();
  void applyPayloadCompensation();
  void publishLowCmd(const std::array<float, go2::kNumLegMotors> &unitree_targets);
  void publishFeedback(const float target_joints[go2::kNumLegMotors]);

  // --- CHAMP core ---
  champ::GaitConfig gait_config_;
  champ::QuadrupedBase base_;
  champ::BodyController body_controller_;
  champ::LegController leg_controller_;
  champ::Kinematics kinematics_;
  champ::Velocities req_vel_;
  champ::Pose req_pose_;

  // --- Gait / control params ---
  std::string knee_orientation_;
  double kp_ = 60.0;
  double kd_ = 5.0;
  double stand_duration_ = 3.0;
  double cmd_vel_timeout_ = 0.5;
  bool publish_joint_states_ = true;
  bool publish_imu_ = true;
  bool publish_joint_control_ = true;
  std::array<double, go2::kNumLegMotors> motor_signs_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                                     1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<float, go2::kNumLegMotors> start_pose_{};
  std::array<float, go2::kNumLegMotors> last_command_{};

  // --- Measured robot state (guarded by state_mutex_) ---
  std::array<float, go2::kNumLegMotors> measured_q_{};
  std::array<float, 4> imu_quat_{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> imu_gyro_{};
  std::array<float, 3> imu_acc_{};
  std::atomic_bool have_state_{false};

  // --- Arm-q sources for payload compensation (guarded by state_mutex_) ---
  std::array<float, go2::kNumArmJoints> arm_q_lowstate_{};  // /lowstate[12..17]
  std::array<float, go2::kNumArmJoints> arm_q_ros_{};       // ROS topics (rad)
  bool arm_lowstate_received_ = false;
  bool arm_ros_received_ = false;
  std::chrono::steady_clock::time_point last_arm_lowstate_{};
  std::chrono::steady_clock::time_point last_arm_ros_{};

  // --- Payload compensation ---
  go2::PayloadCompensator payload_comp_;
  bool payload_enable_ = true;
  double payload_arm_timeout_ = 1.0;
  std::array<float, go2::kNumArmJoints> tucked_q_rad_{};
  // Trims produced by applyPayloadCompensation(), consumed by stepOnce().
  double payload_trim_y_ = 0.0;
  double payload_trim_z_ = 0.0;
  double payload_trim_pitch_ = 0.0;
  double imu_trim_pitch_ = 0.0;
  int payload_dbg_div_ = 0;

  // --- Sport-mode state reader (constructed after loadParams, see ctor) ---
  std::unique_ptr<go2::SportStateReader> sport_state_;

  // --- Sport-mode state reader params (see ctor) ---
  bool sport_state_enable_ = true;
  bool sport_state_high_freq_ = false;
  bool sport_state_log_foot_ = true;
  double sport_state_log_period_ = 1.0;

  // --- Publishers / subscriptions ---
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_targets_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr measured_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_measured_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_measured_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      joint_commands_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr payload_com_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      payload_trims_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub_;
  rclcpp::Subscription<unitree_arm::msg::PubServoInfo>::SharedPtr
      current_servo_angle_sub_;

  // --- Threading / LowCmd template ---
  std::mutex state_mutex_;
  std::mutex cmd_mutex_;
  std::atomic_bool running_{true};
  std::thread control_thread_;
  unitree_go::msg::LowCmd low_cmd_{};  // template (header/level/gpio + stops)
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  double motion_time_ = 0.0;
  std::chrono::steady_clock::time_point last_cmd_vel_steady_{
      std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_joint_states_steady_{
      std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_imu_steady_{
      std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_joint_commands_steady_{
      std::chrono::steady_clock::now()};
};

#endif  // GO2_CHAMP_ROS2_CONTROLLER_H_
