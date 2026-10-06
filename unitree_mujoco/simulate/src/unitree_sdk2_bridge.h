#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/common/json/json.hpp>

#include "ArmString_.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <mutex>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3
#define LOW_CMD_TOPIC "rt/lowcmd"
#define LOW_CMD_ALIAS_TOPIC "lowcmd"
#define D1_ARM_COMMAND_TOPIC "rt/arm_Command"
#define D1_ARM_COMMAND_ALIAS_TOPIC "rt/arm_command"
#define D1_ARM_COMMAND_RAW_TOPIC "arm_Command"

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>(LOW_CMD_TOPIC);
        lowcmd_alias = std::make_shared<LowCmd_t>(LOW_CMD_ALIAS_TOPIC);
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        lowstate_alias = std::make_unique<LowState_t>("lowstate");
        lowstate_alias->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;

        arm_command = std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(D1_ARM_COMMAND_TOPIC);
        arm_command->InitChannel([this](const void *message) { this->handleArmCommand(message); }, 10);
        arm_command_alias = std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(D1_ARM_COMMAND_ALIAS_TOPIC);
        arm_command_alias->InitChannel([this](const void *message) { this->handleArmCommand(message); }, 10);
        arm_command_raw = std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(D1_ARM_COMMAND_RAW_TOPIC);
        arm_command_raw->InitChannel([this](const void *message) { this->handleArmCommand(message); }, 10);
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        auto applyLowCmd = [this](const auto &command) {
            std::lock_guard<std::mutex> lock(command->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = command->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        };
        if (!lowcmd->isTimeout()) {
            applyLowCmd(lowcmd);
        } else if (!lowcmd_alias->isTimeout()) {
            applyLowCmd(lowcmd_alias);
        }

        applyArmCommand();

        // lowstate
        if(lowstate->trylock()) {
            lowstate_alias->lock();
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate_alias->msg_ = lowstate->msg_;
            lowstate->unlockAndPublish();
            lowstate_alias->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::shared_ptr<LowCmd_t> lowcmd_alias;
    std::unique_ptr<LowState_t> lowstate;
    std::unique_ptr<LowState_t> lowstate_alias;
    
private:
    static constexpr int kD1FirstMotor = 12;
    static constexpr int kD1JointCount = 6;
    static constexpr int kD1GripperLeft = 18;
    static constexpr int kD1GripperRight = 19;

    static double number(const unitree::common::JsonMap &object, const std::string &key, double fallback)
    {
        const auto item = object.find(key);
        if (item == object.end() || !unitree::common::IsNumber(item->second)) return fallback;
        if (unitree::common::IsDouble(item->second)) return unitree::common::AnyCast<double>(&item->second);
        if (unitree::common::IsFloat(item->second)) return unitree::common::AnyCast<float>(&item->second);
        if (unitree::common::IsInt(item->second)) return unitree::common::AnyCast<int32_t>(&item->second);
        if (unitree::common::IsUint(item->second)) return unitree::common::AnyCast<uint32_t>(&item->second);
        if (unitree::common::IsInt64(item->second)) return unitree::common::AnyCast<int64_t>(&item->second);
        if (unitree::common::IsUint64(item->second)) return unitree::common::AnyCast<uint64_t>(&item->second);
        if (unitree::common::IsInt16(item->second)) return unitree::common::AnyCast<int16_t>(&item->second);
        if (unitree::common::IsUint16(item->second)) return unitree::common::AnyCast<uint16_t>(&item->second);
        if (unitree::common::IsInt8(item->second)) return unitree::common::AnyCast<int8_t>(&item->second);
        if (unitree::common::IsUint8(item->second)) return unitree::common::AnyCast<uint8_t>(&item->second);
        return fallback;
    }

    void handleArmCommand(const void *message)
    {
        if (num_motor_ < param::IDL_GO_MOTOR_LIMIT) return;

        const auto *command = static_cast<const unitree_arm::msg::dds_::ArmString_ *>(message);
        try
        {
            const auto root = unitree::common::FromJsonString(command->data_());
            const auto &object = unitree::common::AnyCast<unitree::common::JsonMap>(&root);
            const int funcode = static_cast<int>(number(object, "funcode", -1));
            const auto data_item = object.find("data");
            if (data_item == object.end() || !unitree::common::IsJsonMap(data_item->second)) return;
            const auto &data = unitree::common::AnyCast<unitree::common::JsonMap>(&data_item->second);

            std::lock_guard<std::mutex> lock(arm_mutex);
            if (funcode == 5)
            {
                arm_enabled = number(data, "mode", 0) != 0;
                return;
            }
            if (!arm_enabled) return;
            if (funcode == 2)
            {
                for (int index = 0; index < 7; ++index)
                {
                    const std::string key = "angle" + std::to_string(index);
                    arm_target[index] = number(data, key, arm_target[index]);
                }
                arm_target_valid = true;
            }
            else if (funcode == 1)
            {
                const int index = static_cast<int>(number(data, "id", -1));
                if (index >= 0 && index < 7)
                {
                    arm_target[index] = number(data, "angle", arm_target[index]);
                    arm_target_valid = true;
                }
            }
        }
        catch (const std::exception &error)
        {
            std::cerr << "Invalid D1 arm command: " << error.what() << std::endl;
        }
    }

    void applyArmCommand()
    {
        if (num_motor_ < param::IDL_GO_MOTOR_LIMIT) return;

        std::lock_guard<std::mutex> lock(arm_mutex);
        if (!arm_enabled || !arm_target_valid) return;

        constexpr double kDegreesToRadians = 0.017453292519943295;
        for (int index = 0; index < kD1JointCount; ++index)
        {
            const int motor = kD1FirstMotor + index;
            const double target = arm_target[index] * kDegreesToRadians;
            mj_data_->ctrl[motor] = 40.0 * (target - mj_data_->sensordata[motor])
                                   - 2.0 * mj_data_->sensordata[motor + num_motor_];
        }

        const double opening = std::clamp(arm_target[6] / 180.0 * 0.03, 0.0, 0.03);
        mj_data_->ctrl[kD1GripperLeft] = 20.0 * (opening - mj_data_->sensordata[kD1GripperLeft]);
        mj_data_->ctrl[kD1GripperRight] = 20.0 * (-opening - mj_data_->sensordata[kD1GripperRight]);
    }

    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>> arm_command;
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>> arm_command_alias;
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>> arm_command_raw;
    std::array<double, 7> arm_target{};
    std::mutex arm_mutex;
    bool arm_enabled = false;
    bool arm_target_valid = false;
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
