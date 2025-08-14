#!/usr/bin/env python3
import rclpy
from go2_agent_base import Go2AgentBase
from unitree_go.msg import MotorCmd, MotorCmds
from geometry_msgs.msg import Twist
from go2_rl_agents.go2_inverse_kinematics import Go2IK
import torch

class Go2AgentCPG(Go2AgentBase):
    def __init__(self):
        super().__init__()
        self.go2_ik = Go2IK()

        self.cmd_vel = torch.zeros(3, device="cpu")
        # self.cmd_vel[0] = 0.1
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmdVelCallback,
            10)

        self.feet_order = [1, 0, 3, 2]
        self.joints_order = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
        self.default_joint_pos = torch.tensor([0, torch.pi/4, -torch.pi/2,
                                               0, torch.pi/4, -torch.pi/2,
                                               0, torch.pi/4, -torch.pi/2,
                                               0, torch.pi/4, -torch.pi/2,], device="cpu")

        self.last_action = torch.zeros(15, device="cpu")
        self.estimated_lin_vel = torch.zeros(3, device="cpu")

        self.swing_frequency_limit = 6.0
        self.stance_frequency_limit = 6.0
        self.oscilator_limit = (0.5, 2.0)
        self.joints_offset_scale = 0.1
        self.convergence_factor_a = 50.0
        self.coupling_weight = 1.0
        self.ground_clearance = 0.1
        self.ground_penetration = 0.01
        self.body_height_offset = 0.0
        self.step_size = 0.05

        # self.amplitude_mu = 1.0
        # self.swing_frequency = 2.0
        # self.stance_frequency = 1.5

        self.amplitude_r = torch.randn(4, device="cpu")
        self.phase_theta = torch.randn(4, device="cpu")
        self.amplitude_dr = torch.zeros(4, device="cpu")
        self.amplitude_d2r = torch.zeros(4, device="cpu")
        self.phase_dtheta = torch.zeros(4, device="cpu")
        self.frequency_omega = torch.zeros(4, device="cpu")

        self.walk_matrix = torch.Tensor(
            [
                [0, torch.pi, torch.pi / 2, 3 * torch.pi / 2],
                [-torch.pi, 0, -torch.pi / 2, -3 * torch.pi / 2],
                [-torch.pi / 2, torch.pi / 2, 0, -torch.pi],
                [-3 * torch.pi / 2, 3 * torch.pi / 2, torch.pi, 0],
            ], device="cpu"
        )
        self.trot_matrix = torch.Tensor(
            [
                [0, torch.pi, torch.pi, 0],
                [-torch.pi, 0, 0, -torch.pi],
                [-torch.pi, 0, 0, -torch.pi],
                [0, torch.pi, torch.pi, 0],
            ], device="cpu"
        )
        self.pace_matrix = torch.Tensor(
            [
                [0, torch.pi, 0, torch.pi],
                [-torch.pi, 0, -torch.pi, 0],
                [0, torch.pi, 0, torch.pi],
                [-torch.pi, 0, -torch.pi, 0],
            ], device="cpu"
        )
        self.gallop_matrix = torch.Tensor(
            [
                [0, 0, -torch.pi, -torch.pi],
                [0, 0, -torch.pi, -torch.pi],
                [torch.pi, torch.pi, 0, 0],
                [torch.pi, torch.pi, 0, 0],
            ], device="cpu"
        )

        self.feet_distance_x = 0.3868
        self.feet_distance_y = 0.284
        self.feet_location = torch.tensor([
            [ self.feet_distance_x, -self.feet_distance_y],  # Front Right
            [ self.feet_distance_x,  self.feet_distance_y],  # Front Left
            [-self.feet_distance_x, -self.feet_distance_y],  # Rear Right
            [-self.feet_distance_x,  self.feet_distance_y],  # Rear Left
        ], device="cpu")

    def cmdVelCallback(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z

    def getObservation(self):
        # vel command
        cmd_vel = self.cmd_vel

        # robot base
        # base_lin_vel = torch.tensor(self.sport_mode_state.velocity, device="cpu")
        base_lin_vel = torch.zeros(3, device="cpu")
        base_lin_vel[0] = 0.9 * self.estimated_lin_vel[0] + 0.1 * cmd_vel[0]
        base_lin_vel[1] = 0.9 * self.estimated_lin_vel[1] + 0.1 * cmd_vel[1]
        self.estimated_lin_vel = base_lin_vel
        base_ang_vel = torch.tensor(self.low_state.imu_state.gyroscope, device="cpu")
        proj_gravity = torch.tensor(self.low_state.imu_state.accelerometer, device="cpu") / 9.8

        # robot joints
        joint_qs = [self.low_state.motor_state[j].q for j in range(12)]
        joint_dqs = [self.low_state.motor_state[j].dq for j in range(12)]
        joint_pos = torch.tensor(joint_qs, device="cpu")[self.joints_order] - self.default_joint_pos[self.joints_order]
        joint_vel = torch.tensor(joint_dqs, device="cpu")[self.joints_order]

        # feet_contact
        feet_bools = [1 if self.low_state.foot_force[i] >= 20.0 else 0 for i in range(4)]
        feet_contact = torch.tensor(feet_bools, device="cpu")[self.feet_order]

        # cpg_state
        cpg_state = torch.cat((
            self.amplitude_r[self.feet_order],
            self.amplitude_dr[self.feet_order],
            self.phase_theta[self.feet_order],
            self.phase_dtheta[self.feet_order],
        ))

        # last_action
        last_action = self.last_action

        return torch.cat((cmd_vel, base_lin_vel, base_ang_vel, proj_gravity, joint_pos, joint_vel, feet_contact, cpg_state, last_action))

    def computeAction(self, obs):
        with torch.inference_mode():
            action = self.policy(obs)
        self.last_action = action
        return action

    def computeMotorCmds(self):
        obs = self.getObservation()
        action = self.computeAction(obs)

        print(f"Observation: {obs}")
        # print(action)

        processed_action = torch.zeros(15, device="cpu")

        cmd_vel = obs[:3]
        cmd_vel_norm = torch.norm(cmd_vel)

        oscilator_min, oscilator_max = self.oscilator_limit
        oscilator_difference = oscilator_max - oscilator_min
        processed_action[0] = oscilator_min + oscilator_difference * torch.sigmoid(action[0] / oscilator_difference) if cmd_vel_norm.item() > 0.01 else 0.0
        processed_action[1] = self.swing_frequency_limit * torch.sigmoid(action[1] / self.swing_frequency_limit)
        processed_action[2] = self.stance_frequency_limit * torch.sigmoid(action[2] / self.stance_frequency_limit)
        processed_action[3:] = self.joints_offset_scale * action[3:]

        print(f"Processed Action: {processed_action}")

        # action = torch.zeros(15, device="cpu")
        # action[0] = self.amplitude_mu if cmd_vel_norm.item() > 0.01 else 0.0
        # action[1] = self.swing_frequency
        # action[2] = self.stance_frequency

        # feet_pos = self.stepCPG(cmd_vel, action[0], action[1], action[2])

        feet_pos = self.stepCPG(cmd_vel, processed_action[0], processed_action[1], processed_action[2])
        joint_pos = self.go2_ik.computeQuadrupedJoints(feet_pos)

        joint_pos_offset = torch.zeros(12, device="cpu")
        # joint_pos_offset = processed_action[3:]
        joint_pos += joint_pos_offset[self.joints_order]

        motor_cmds = MotorCmds()
        for pos in joint_pos.tolist():
            cmd = MotorCmd()
            cmd.mode = 0x01
            cmd.q = pos
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.kp = 100.0
            cmd.kd = 2.0
            # cmd.kp = 60.0
            # cmd.kd = 5.0
            motor_cmds.cmds.append(cmd)
        return motor_cmds

    def stepCPG(self, cmd_vel, amplitude_mu, swing_frequency, stance_frequency):
        dt = 1 / 50.0

        coupling_matrix = self.trot_matrix

        self.amplitude_d2r = self.convergence_factor_a * ((self.convergence_factor_a / 4.0) * (amplitude_mu - self.amplitude_r) - self.amplitude_dr)
        self.amplitude_dr += self.amplitude_d2r * dt

        for i in range(4):
            self.frequency_omega[i] = (2 * torch.pi) * (swing_frequency if self.phase_theta[i] < torch.pi else stance_frequency)
            self.phase_dtheta[i] = self.frequency_omega[i]
            for j in range(4):
                self.phase_dtheta[i] += self.amplitude_r[j] * self.coupling_weight * torch.sin(self.phase_theta[j] - self.phase_theta[i] - coupling_matrix[i, j])

        self.amplitude_r += self.amplitude_dr * dt
        self.phase_theta += self.phase_dtheta * dt

        self.phase_theta %= (2 * torch.pi)

        ground_multiplier = torch.where(torch.sin(self.phase_theta) > 0, self.ground_clearance, self.ground_penetration)
        if amplitude_mu == 0.0:
            ground_multiplier = torch.zeros(4)

        self.omnidirectional_offset = self.computeOmnidirectionalOffset(cmd_vel)

        foot_x = (
            -self.step_size
            * self.amplitude_r
            * torch.cos(self.phase_theta)
            * torch.cos(self.omnidirectional_offset)
        )
        foot_y = (
            -self.step_size
            * self.amplitude_r
            * torch.cos(self.phase_theta)
            * torch.sin(self.omnidirectional_offset)
        )
        foot_z = ground_multiplier * torch.sin(self.phase_theta) - self.body_height_offset

        return torch.tensor([foot_x[0], foot_y[0], foot_z[0], foot_x[1], foot_y[1], foot_z[1], foot_x[2], foot_y[2], foot_z[2], foot_x[3], foot_y[3], foot_z[3]])

    def computeOmnidirectionalOffset(self, cmd_vel):
        vx, vy, wz = cmd_vel
        lin_vel = torch.tensor([vx, vy], device="cpu")
        rot_vel = wz * torch.stack([-self.feet_location[:, 1], self.feet_location[:, 0]], dim=-1)
        total_vel = lin_vel + rot_vel
        return torch.atan2(total_vel[:, 1], total_vel[:, 0])


def main(args=None):
    rclpy.init(args=args)
    go2_agent = Go2AgentCPG()
    rclpy.spin(go2_agent)
    go2_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
