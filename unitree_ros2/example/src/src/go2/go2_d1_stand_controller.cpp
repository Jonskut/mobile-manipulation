#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "motor_crc.h"

#include "rclcpp/rclcpp.hpp"

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

using namespace unitree::robot;

namespace
{

constexpr int kLegMotorCount = 12;
constexpr int kArmJointCount = 7;

constexpr double kControlPeriod = 0.002;
constexpr double kStandDuration = 3.0;
constexpr double kArmMoveDuration = 2.0;

constexpr std::array<float, kLegMotorCount> kStandPose =
{
    0.0F, 0.67F, -1.3F,
    0.0F, 0.67F, -1.3F,
    0.0F, 0.67F, -1.3F,
    0.0F, 0.67F, -1.3F
};

constexpr std::array<double, kArmJointCount> kArmHomeAngles =
{
    0.0,
    -60.0,
    60.0,
    0.0,
    30.0,
    0.0,
    0.0
};

}

class Go2D1StandController : public rclcpp::Node
{
public:

    Go2D1StandController()
        : Node("go2_d1_stand_controller")
    {
        //------------------------------------------
        // D1 DDS
        //------------------------------------------

        ChannelFactory::Instance()->Init(0);

        arm_pub_ =
            std::make_shared<
                ChannelPublisher<
                    unitree_arm::msg::dds_::ArmString_
                >
            >("rt/arm_Command");

        arm_pub_->InitChannel();

        arm_state_sub_ =
            std::make_shared<
                ChannelSubscriber<
                    unitree_arm::msg::dds_::PubServoInfo_
                >
            >("current_servo_angle");

        arm_state_sub_->InitChannel(
            std::bind(
                &Go2D1StandController::arm_state_callback,
                this,
                std::placeholders::_1),
            1);

        //------------------------------------------
        // Go2 ROS
        //------------------------------------------

        low_cmd_pub_ =
            create_publisher<
                unitree_go::msg::LowCmd>(
                    "lowcmd",
                    10);

        low_state_sub_ =
            create_subscription<
                unitree_go::msg::LowState>(
                    "lowstate",
                    10,
                    std::bind(
                        &Go2D1StandController::low_state_callback,
                        this,
                        std::placeholders::_1));

        initialize_lowcmd();

        publish_arm_enable(true);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(2),
                std::bind(
                    &Go2D1StandController::control_loop,
                    this));
    }

private:

    //------------------------------------------
    // Callbacks
    //------------------------------------------

    void low_state_callback(
        const unitree_go::msg::LowState::SharedPtr msg)
    {
        low_state_ = *msg;
        low_state_received_ = true;
    }

    void arm_state_callback(
        const void* message)
    {
        auto state =
            static_cast<
                const unitree_arm::msg::dds_::PubServoInfo_*
            >(message);

        arm_current_angles_[0] = state->servo0_data_();
        arm_current_angles_[1] = state->servo1_data_();
        arm_current_angles_[2] = state->servo2_data_();
        arm_current_angles_[3] = state->servo3_data_();
        arm_current_angles_[4] = state->servo4_data_();
        arm_current_angles_[5] = state->servo5_data_();
        arm_current_angles_[6] = state->servo6_data_();

        arm_state_received_ = true;
    }

    //------------------------------------------
    // Init
    //------------------------------------------

    void initialize_lowcmd()
    {
        low_cmd_.head[0] = 0xFE;
        low_cmd_.head[1] = 0xEF;

        low_cmd_.level_flag = 0xFF;
        low_cmd_.gpio = 0;

        for (auto& motor : low_cmd_.motor_cmd)
        {
            motor.mode = 0x01;

            motor.q = PosStopF;
            motor.dq = VelStopF;

            motor.kp = 0.0F;
            motor.kd = 0.0F;

            motor.tau = 0.0F;
        }
    }

    //------------------------------------------
    // Helper
    //------------------------------------------

    static float interpolate(
        float start,
        float target,
        double progress)
    {
        return static_cast<float>(
            start * (1.0 - progress)
            + target * progress);
    }

    //------------------------------------------
    // Main control loop
    //------------------------------------------

    void control_loop()
    {
        if (!low_state_received_)
        {
            return;
        }

        if (!arm_state_received_)
        {
            return;
        }

        if (!targets_initialized_)
        {
            for (int i = 0; i < kLegMotorCount; i++)
            {
                start_pose_[i] =
                    low_state_.motor_state[i].q;
            }

            for (int i = 0; i < kArmJointCount; i++)
            {
                arm_start_angles_[i] =
                    arm_current_angles_[i];
            }

            targets_initialized_ = true;
        }

        double stand_progress =
            std::min(
                motion_time_ / kStandDuration,
                1.0);

        double arm_progress =
            std::min(
                motion_time_ / kArmMoveDuration,
                1.0);

        //----------------------------------
        // Legs
        //----------------------------------

        for (int i = 0; i < kLegMotorCount; i++)
        {
            auto& motor =
                low_cmd_.motor_cmd[i];

            motor.q =
                interpolate(
                    start_pose_[i],
                    kStandPose[i],
                    stand_progress);

            motor.dq = 0.0F;

            motor.kp = 60.0F;
            motor.kd = 5.0F;

            motor.tau = 0.0F;
        }

        get_crc(low_cmd_);

        low_cmd_pub_->publish(low_cmd_);

        //----------------------------------
        // Arm
        //----------------------------------

        publish_arm_pose(arm_progress);

        motion_time_ += kControlPeriod;
    }

    //------------------------------------------
    // D1 commands
    //------------------------------------------

    void publish_arm_enable(bool enable)
    {
        unitree_arm::msg::dds_::ArmString_ msg;

        msg.data_() =
            std::string("{\"seq\":")
            + std::to_string(sequence_++)
            + ",\"address\":1,"
              "\"funcode\":5,"
              "\"data\":{\"mode\":"
            + (enable ? "1" : "0")
            + "}}";

        arm_pub_->Write(msg);
    }

    void publish_arm_pose(double progress)
    {
        std::ostringstream json;

        json
            << std::fixed
            << std::setprecision(3)
            << "{\"seq\":"
            << sequence_++
            << ",\"address\":1,"
            << "\"funcode\":2,"
            << "\"data\":{\"mode\":1";

        for (int i = 0; i < kArmJointCount; i++)
        {
            double angle =
                interpolate(
                    static_cast<float>(arm_start_angles_[i]),
                    static_cast<float>(kArmHomeAngles[i]),
                    progress);

      