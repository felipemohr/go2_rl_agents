#!/usr/bin/env python3
import torch

class Go2IK:
    def __init__(self):
        self.FRONT_RIGHT = 0
        self.FRONT_LEFT = 1
        self.REAR_RIGHT = 2
        self.REAR_LEFT = 3
        self.L1 = 0.0955
        self.L2 = 0.2130
        self.L3 = 0.2130
        self.foot_offset_x = -0.05
        self.foot_offset_y = 0.0955
        self.foot_offset_z = -0.3012
        # self.body_length = 0.3868
        # self.body_width = 0.0930
        # self.body_height = 0.3012
        # self.min_hip = -1.0472
        # self.max_hip = 1.0472
        # self.min_thigh = -1.5708
        # self.max_thigh = 3.4907
        # self.min_calf = -2.7227
        # self.max_calf = -0.83776

    def computeQuadrupedJoints(self, feet_pos: torch.Tensor):
        front_right = self.computLegJoints(feet_pos[0:3], self.FRONT_RIGHT)
        front_left = self.computLegJoints(feet_pos[3:6], self.FRONT_LEFT)
        rear_right = self.computLegJoints(feet_pos[6:9], self.REAR_RIGHT)
        rear_left = self.computLegJoints(feet_pos[9:12], self.REAR_LEFT)
        return torch.concat([front_right, front_left, rear_right, rear_left])

    def computLegJoints(self, foot_pos: torch.Tensor, leg):

        reflect = 1 if leg in (self.FRONT_LEFT, self.REAR_LEFT) else -1

        foot_x = foot_pos[0] + self.foot_offset_x
        foot_y = foot_pos[1] + reflect * self.foot_offset_y
        foot_z = foot_pos[2] + self.foot_offset_z

        a = torch.sqrt(foot_y**2 + foot_z**2 - self.L1**2)
        A = (a**2 + foot_x**2 + self.L2**2 - self.L3**2) / (2 * self.L2 * torch.sqrt(a**2 + foot_x**2))
        B = (a**2 + foot_x**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)

        theta1 = torch.atan2(foot_y, -foot_z) - torch.atan2(torch.tensor([reflect * self.L1]), a)
        theta2 = torch.pi / 2 - torch.atan2(a, foot_x) - torch.atan2(torch.sqrt(1 - A**2), A)
        theta3 = torch.atan2(torch.sqrt(1 - B**2), B)

        return torch.tensor([theta1, -theta2, -theta3])

    def checkJointValues(self):
        pass
