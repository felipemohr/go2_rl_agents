#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

const int32_t ROBOT_STATE_API_ID_SERVICE_SWITCH = 1001;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("go2_enable_motion_control");
  auto state_req_publisher = node->create_publisher<unitree_api::msg::Request>("/api/robot_state/request", 10);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  unitree_api::msg::Request req;
  req.header.identity.api_id = ROBOT_STATE_API_ID_SERVICE_SWITCH;
  req.parameter = req.parameter = "{\"name\":\"sport_mode\",\"switch\":1}";
  state_req_publisher->publish(req);

  rclcpp::shutdown();
  return 0;
}
