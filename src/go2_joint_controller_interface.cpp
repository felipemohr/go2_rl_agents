#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "go2_rl_agents/go2_joint_controller_interface.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/motor_crc.h"

Go2JointControllerInterface::Go2JointControllerInterface() : Node("go2_joint_controller_interface")
{
  RCLCPP_INFO(this->get_logger(), "Go2 CPG node initialized.");

  sport_req_publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
  state_req_publisher_ = this->create_publisher<unitree_api::msg::Request>("/api/robot_state/request", 10);
  low_cmd_publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

  stand_cmd_q_ = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  DisableMotionControl();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  StandUp();

  low_cmd_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
    std::bind(&Go2JointControllerInterface::publishCmdCallback, this));
}

Go2JointControllerInterface::~Go2JointControllerInterface()
{}

void Go2JointControllerInterface::DisableMotionControl()
{
  unitree_api::msg::Request req;

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Stand down
  RCLCPP_INFO(this->get_logger(), "Standing down.");
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
  sport_req_publisher_->publish(req);

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // Shut down motion control
  RCLCPP_INFO(this->get_logger(), "Shuting down motion control.");
  req.header.identity.api_id = ROBOT_STATE_API_ID_SERVICE_SWITCH;
  req.parameter = req.parameter = "{\"name\":\"sport_mode\",\"switch\":0}";
  state_req_publisher_->publish(req);
}

void Go2JointControllerInterface::StandUp()
{
  RCLCPP_INFO(this->get_logger(), "Standing Up.");

  unitree_go::msg::LowState init_low_state;

  auto context = rclcpp::contexts::get_global_default_context();

  auto low_state_sub = this->create_subscription<unitree_go::msg::LowState>(
    "lf/lowstate",
    rclcpp::QoS(10),
    [](const unitree_go::msg::LowState::SharedPtr){}
  );

  if (rclcpp::wait_for_message(init_low_state, low_state_sub, context->shared_from_this(), std::chrono::seconds(2)))
  {
    for (int j = 0; j < 12; j++)
      init_state_q_[j] = init_low_state.motor_state[j].q;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Low state message not received. Using default values.");
    init_state_q_ = {0.0,  1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
  }

  low_cmd_.head[0] = 0xFE;
  low_cmd_.head[1] = 0xEF;
  low_cmd_.level_flag = 0xFF;
  low_cmd_.gpio = 0;
}

void Go2JointControllerInterface::publishCmdCallback()
{
  stand_ratio_ += 0.005;
  if (stand_ratio_ > 1.0) stand_ratio_ = 1.0;

  for (int j = 0; j < 12; j++)
  {
    low_cmd_.motor_cmd[j].mode = 0x01;
    low_cmd_.motor_cmd[j].q = init_state_q_[j] + stand_ratio_ * (stand_cmd_q_[j] - init_state_q_[j]);
    low_cmd_.motor_cmd[j].kp = 60;
    low_cmd_.motor_cmd[j].dq = 0;
    low_cmd_.motor_cmd[j].kd = 5;
    low_cmd_.motor_cmd[j].tau = 0;
  }

  get_crc(low_cmd_);
  low_cmd_publisher_->publish(low_cmd_);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2JointControllerInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
