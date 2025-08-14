#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState, LowState, MotorCmd, MotorCmds

import torch

class Go2AgentBase(Node):
    def __init__(self):
        super().__init__("go2_agent")

        # TODO: use ros2 parameter
        self.policy_path = "/home/felipe/unitree_ros2/go2_rl_ws/src/go2_rl_agents/models/go2_cpg_blind/policy.pt"
        self.policy = torch.jit.load(self.policy_path, map_location="cpu")

        self.sport_mode_state = SportModeState()
        self.low_state = LowState()

        self.sport_mode_state_sub = self.create_subscription(
            SportModeState,
            "/sportmodestate",
            self.sportModeStateCallback,
            10)

        self.low_state_sub = self.create_subscription(
            LowState,
            "/lowstate",
            self.lowStateCallback,
            10)
        
        self.motor_cmds_pub = self.create_publisher(
            MotorCmds, 
            "/agent_cmds", 
            10)

        self.publish_timer = self.create_timer(0.02, self.publishCallback)

        self.get_logger().info("go2_agent node initialized")

    def getObservation(self):
        pass

    def computeAction(self, obs):
        with torch.inference_mode():
            return self.policy(obs)

    def sportModeStateCallback(self, msg):
        self.sport_mode_state = msg

    def lowStateCallback(self, msg):
        self.low_state = msg
    
    def publishCallback(self):
        motor_cmds = self.computeMotorCmds()
        self.motor_cmds_pub.publish(motor_cmds)

    def computeMotorCmds(self):
        motor_cmds = MotorCmds()
        stand_pos = torch.tensor([0.0,  0.9362, -1.5434,  0.0,  0.9362, -1.5434,  0.0,  0.9362, -1.5434,  0.0,  0.9362, -1.5434], device="cpu")
        # stand_pos = torch.tensor([0.0, 0.67, -1.0, 0.0, 0.67, -1.0, 0.0, 0.67, -1.0, 0.0, 0.67, -1.0], device="cpu")
        for pos in stand_pos.tolist():
            cmd = MotorCmd()
            cmd.mode = 0x01
            cmd.q = pos
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.kp = 100.0
            cmd.kd = 2.0
            motor_cmds.cmds.append(cmd)
        return motor_cmds


def main(args=None):
    rclpy.init(args=args)
    go2_agent = Go2AgentBase()
    rclpy.spin(go2_agent)
    go2_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
