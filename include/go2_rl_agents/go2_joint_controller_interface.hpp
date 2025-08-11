#pragma once

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include <array>

const int32_t ROBOT_STATE_API_ID_SERVICE_SWITCH = 1001;
const int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;

class Go2JointControllerInterface : public rclcpp::Node
{
public:
  Go2JointControllerInterface();
  ~Go2JointControllerInterface();

  void DisableMotionControl();
  void StandUp();

private:
  void publishCmdCallback();

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_req_publisher_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr state_req_publisher_;

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr low_cmd_timer_;

  unitree_go::msg::LowCmd low_cmd_;

  std::array<float, 12> init_state_q_;
  std::array<float, 12> stand_cmd_q_;
  float stand_ratio_ = 0.0;
};
