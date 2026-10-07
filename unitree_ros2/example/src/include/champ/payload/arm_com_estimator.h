// Payload-aware CoM estimator for the D1 arm on Go2.
// Best-practice choice: hardcoded FK table (fixed 6R chain, inertials from
// unitree_mujoco/unitree_robots/go2/assets/d1/d1_arm.xml + d1_description.urdf).
// No Pinocchio/KDL dep, deterministic, identical in sim and on the real robot.
// Tucked-carry default: [0,-60,60,0,30,0] deg (matches go2_d1_stand_controller).
#ifndef CHAMP_PAYLOAD_ARM_COM_ESTIMATOR_H_
#define CHAMP_PAYLOAD_ARM_COM_ESTIMATOR_H_

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace champ {
namespace payload {

constexpr int kArmJoints = 6;
constexpr float kDeg2Rad = 0.017453292519943295F;

// Tucked-carry home (deg) — must match Go2D1StandController::kArmHomeAngles.
constexpr std::array<float, kArmJoints> kTuckedQDeg = {0.0F, -60.0F, 60.0F,
                                                       0.0F, 30.0F, 0.0F};

// Base (Go2 torso) inertial, in base_link frame (go2.xml).
constexpr float kBaseMass = 6.921F;
constexpr std::array<float, 3> kBaseCom = {0.021112F, 0.0F, -0.005366F};
// Arm mount offset on base_link (go2.xml: d1_arm pos="0 0 0.055").
constexpr std::array<float, 3> kArmMount = {0.0F, 0.0F, 0.055F};

// Per-link data from assets/d1/d1_arm.xml (MuJoCo quat order w,x,y,z).
struct ArmLinkInfo {
  float mass;
  // Joint origin relative to parent link frame.
  Eigen::Vector3f origin_pos;
  Eigen::Quaternionf origin_quat;  // parent -> link frame rotation
  Eigen::Vector3f joint_axis;      // in link frame, already signed
  Eigen::Vector3f com_local;       // inertial pos in link frame
};

inline std::array<ArmLinkInfo, kArmJoints> armLinkTable() {
  std::array<ArmLinkInfo, kArmJoints> t{};
  // Link1: pos 0 0 0.0533, quat 0 0 0 -1, axis 0 0 1
  t[0].mass = 0.13174F;
  t[0].origin_pos = Eigen::Vector3f(0.0F, 0.0F, 0.0533F);
  t[0].origin_quat = Eigen::Quaternionf(0.0F, 0.0F, 0.0F, -1.0F).normalized();
  t[0].joint_axis = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
  t[0].com_local = Eigen::Vector3f(0.0024649F, 0.00010517F, 0.032696F);
  // Link2: pos 0 0.028 0.0563, quat 0 0 -0.7071 -0.7071, axis 0 0 -1
  t[1].mass = 0.20213F;
  t[1].origin_pos = Eigen::Vector3f(0.0F, 0.028F, 0.0563F);
  t[1].origin_quat =
      Eigen::Quaternionf(0.0F, 0.0F, -0.7071F, -0.7071F).normalized();
  t[1].joint_axis = Eigen::Vector3f(0.0F, 0.0F, -1.0F);
  t[1].com_local = Eigen::Vector3f(0.0002018F, 0.19201F, -0.027007F);
  // Link3: pos 0 0.2693 0.0009, identity rotation, axis 0 0 -1
  t[2].mass = 0.0629F;
  t[2].origin_pos = Eigen::Vector3f(0.0F, 0.2693F, 0.0009F);
  t[2].origin_quat = Eigen::Quaternionf::Identity();
  t[2].joint_axis = Eigen::Vector3f(0.0F, 0.0F, -1.0F);
  t[2].com_local = Eigen::Vector3f(0.015164F, 0.044482F, -0.027461F);
  // Link4: pos 0.0577 0.042 -0.0275, quat 0.5 -0.5 0.5 -0.5, axis 0 0 1
  t[3].mass = 0.083332F;
  t[3].origin_pos = Eigen::Vector3f(0.0577F, 0.042F, -0.0275F);
  t[3].origin_quat =
      Eigen::Quaternionf(0.5F, -0.5F, 0.5F, -0.5F).normalized();
  t[3].joint_axis = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
  t[3].com_local = Eigen::Vector3f(-0.00029556F, -0.00016104F, 0.091339F);
  // Link5: pos -0.0001 -0.0237 0.14018, quat 0.5 0.5 -0.5 0.5, axis 0 0 -1
  t[4].mass = 0.053817F;
  t[4].origin_pos = Eigen::Vector3f(-0.0001F, -0.0237F, 0.14018F);
  t[4].origin_quat =
      Eigen::Quaternionf(0.5F, 0.5F, -0.5F, 0.5F).normalized();
  t[4].joint_axis = Eigen::Vector3f(0.0F, 0.0F, -1.0F);
  t[4].com_local = Eigen::Vector3f(0.040573F, 0.0062891F, -0.023838F);
  // Link6: pos 0.0825 -0.0010782 -0.023822, quat 0.5 -0.5 0.5 -0.5, axis 0 0 -1
  // (gripper 2x0.015kg lumped into Link6 at its origin for CoM purposes)
  t[5].mass = 0.077892F + 2.0F * 0.015046F;
  t[5].origin_pos = Eigen::Vector3f(0.0825F, -0.0010782F, -0.023822F);
  t[5].origin_quat =
      Eigen::Quaternionf(0.5F, -0.5F, 0.5F, -0.5F).normalized();
  t[5].joint_axis = Eigen::Vector3f(0.0F, 0.0F, -1.0F);
  t[5].com_local = Eigen::Vector3f(-0.0068528F, 0.0F, 0.039705F);
  return t;
}

inline float armTotalMass() {
  float m = 0.0F;
  for (const auto &l : armLinkTable()) m += l.mass;
  return m;
}

// q_rad: 6 arm joint angles in radians (D1 Joint1..6 order).
// Returns total-system CoM in base_link frame (x fwd, y left, z up).
inline Eigen::Vector3f computeTotalCom(
    const std::array<float, kArmJoints> &q_rad) {
  static const std::array<ArmLinkInfo, kArmJoints> kTable = armLinkTable();
  Eigen::Affine3f parent = Eigen::Affine3f::Identity();
  parent.translation() =
      Eigen::Vector3f(kArmMount[0], kArmMount[1], kArmMount[2]);

  Eigen::Vector3f acc = Eigen::Vector3f(
      kBaseMass * kBaseCom[0], kBaseMass * kBaseCom[1],
      kBaseMass * kBaseCom[2]);
  float total_m = kBaseMass;

  for (std::size_t i = 0; i < kArmJoints; ++i) {
    const ArmLinkInfo &li = kTable[i];
    Eigen::Affine3f link = Eigen::Affine3f::Identity();
    link.translation() = li.origin_pos;
    link.linear() = li.origin_quat.toRotationMatrix();
    Eigen::Affine3f joint = Eigen::Affine3f::Identity();
    const float half = 0.5F * q_rad[i];
    Eigen::Vector3f ax = li.joint_axis.normalized();
    Eigen::Quaternionf jq(std::cos(half), ax.x() * std::sin(half),
                          ax.y() * std::sin(half), ax.z() * std::sin(half));
    joint.linear() = jq.toRotationMatrix();
    parent = parent * link * joint;
    const Eigen::Vector3f com_world = parent * li.com_local;
    acc += li.mass * com_world;
    total_m += li.mass;
  }
  return acc / total_m;
}

inline std::array<float, kArmJoints> tuckedQRad() {
  std::array<float, kArmJoints> q{};
  for (int i = 0; i < kArmJoints; ++i) q[i] = kTuckedQDeg[i] * kDeg2Rad;
  return q;
}

}  // namespace payload
}  // namespace champ

#endif  // CHAMP_PAYLOAD_ARM_COM_ESTIMATOR_H_
