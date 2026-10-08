#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

using namespace unitree::robot;

class D1Controller
{
public:
  D1Controller()
  {
    ChannelFactory::Instance()->Init(0);

    arm_pub_ = std::make_shared<ChannelPublisher<unitree_arm::msg::dds_::ArmString_> >("rt/arm_Command");

    arm_pub_->InitChannel();

    arm_sub_ = std::make_shared<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> >("current_servo_angle");

    arm_sub_->InitChannel(std::bind(&D1Controller::armCallback, this, std::placeholders::_1), 1);
  }

  bool WaitForState()
  {
    std::cout << "Waiting for arm state..." << std::endl;

    int timeout = 5000;

    while (!state_received_ && timeout > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      timeout -= 10;
    }

    return state_received_;
  }

  void Enable()
  {
    unitree_arm::msg::dds_::ArmString_ msg;

    msg.data_() =
        "{\"seq\":1,"
        "\"address\":1,"
        "\"funcode\":5,"
        "\"data\":{\"mode\":1}}";

    arm_pub_->Write(msg);

    std::cout << "Arm enabled" << std::endl;
  }

  void Disable()
  {
    unitree_arm::msg::dds_::ArmString_ msg;

    msg.data_() =
        "{\"seq\":9999,"
        "\"address\":1,"
        "\"funcode\":5,"
        "\"data\":{\"mode\":0}}";

    arm_pub_->Write(msg);

    std::cout << "Arm disabled" << std::endl;
  }

  void MoveTo(const std::array<double, 7>& target_deg, double duration_sec)
  {
    std::array<double, 7> start;

    for (int i = 0; i < 7; i++)
      start[i] = current_angles_[i];

    const int steps = static_cast<int>(duration_sec / 0.02);

    for (int step = 0; step <= steps; step++)
    {
      double alpha = static_cast<double>(step) / static_cast<double>(steps);

      std::array<double, 7> cmd;

      for (int i = 0; i < 7; i++)
      {
        cmd[i] = start[i] + (target_deg[i] - start[i]) * alpha;
      }

      SendAngles(cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void PrintState()
  {
    std::cout << "J0=" << current_angles_[0] << " J1=" << current_angles_[1] << " J2=" << current_angles_[2]
              << " J3=" << current_angles_[3] << " J4=" << current_angles_[4] << " J5=" << current_angles_[5]
              << " J6=" << current_angles_[6] << std::endl;
  }

private:
  void SendAngles(const std::array<double, 7>& angles)
  {
    std::ostringstream ss;

    ss << std::fixed << std::setprecision(3);

    ss << "{";
    ss << "\"seq\":" << sequence_++ << ",";
    ss << "\"address\":1,";
    ss << "\"funcode\":2,";
    ss << "\"data\":{";
    ss << "\"mode\":1";

    for (int i = 0; i < 7; i++)
    {
      ss << ",\"angle" << i << "\":" << angles[i];
    }

    ss << "}}";

    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = ss.str();

    arm_pub_->Write(msg);
  }

  void armCallback(const void* msg)
  {
    auto state = static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(msg);

    current_angles_[0] = state->servo0_data_();
    current_angles_[1] = state->servo1_data_();
    current_angles_[2] = state->servo2_data_();
    current_angles_[3] = state->servo3_data_();
    current_angles_[4] = state->servo4_data_();
    current_angles_[5] = state->servo5_data_();
    current_angles_[6] = state->servo6_data_();

    state_received_ = true;
  }

private:
  std::shared_ptr<ChannelPublisher<unitree_arm::msg::dds_::ArmString_> > arm_pub_;

  std::shared_ptr<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> > arm_sub_;

  std::array<double, 7> current_angles_{};

  bool state_received_ = false;

  int sequence_ = 1;
};

int main()
{
  D1Controller arm;

  if (!arm.WaitForState())
  {
    std::cerr << "Failed to receive arm state" << std::endl;

    return -1;
  }

  arm.PrintState();

  arm.Enable();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::array<double, 7> home = { 0.0, -60.0, 60.0, 0.0, 30.0, 0.0, 0.0 };

  arm.MoveTo(home, 3.0);

  std::cout << "Move complete" << std::endl;

  while (true)
  {
    arm.PrintState();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return 0;
}