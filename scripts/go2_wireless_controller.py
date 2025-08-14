#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import WirelessController
from geometry_msgs.msg import Twist

class Go2WirelessController(Node):
    def __init__(self):
        super().__init__("go2_wireless_controller")

        self.cmd_vel = Twist()

        self.publish_vel_timer = self.create_timer(0.02, self.publishCmdVelCallback)
        self.reset_vel_timer = self.create_timer(0.1, self.resetVelCallback)

        self.wireless_controller_sub = self.create_subscription(
            WirelessController,
            "/wirelesscontroller",
            self.wirelessControllerCallback,
            10)

        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            "/cmd_vel", 
            10)

        self.get_logger().info("go2_wireless_controller node initialized")

    def wirelessControllerCallback(self, msg):
        self.reset_vel_timer.reset()
        self.cmd_vel.linear.x = 0.9 * self.cmd_vel.linear.x + 0.1 * (1.0 * msg.ly)
        self.cmd_vel.linear.y = 0.9 * self.cmd_vel.linear.y + 0.1 * (-1.0 * msg.lx)
        self.cmd_vel.angular.z = 0.9 * self.cmd_vel.angular.z + 0.1 * (1.0 * msg.rx)
        if msg.keys != 0:
            self.cmd_vel = Twist()

    def publishCmdVelCallback(self):
        self.cmd_vel_pub.publish(self.cmd_vel)

    def resetVelCallback(self):
        print("Reseting velocity")
        self.cmd_vel = Twist()


def main(args=None):
    rclpy.init(args=args)
    node = Go2WirelessController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
