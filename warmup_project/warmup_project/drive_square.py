import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np


class square_driver(Node):
    def __init__(self):
        super().__init__("square_driver")

        self.create_subscription(Odometry, "odom", self.process_odom, 10)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.05, self.run_loop)

        # determine whether to be turning
        self.turn = False

        # robot position
        self.xpos = 0.0
        self.ypos = 0.0
        # self.angular = 0.0

        # position of last turn
        self.xpos_b = 0.0
        self.ypos_b = 0.0
        self.orientation_bench = Quaternion()
        self.turns = 0.0
        self.orientation = Quaternion()
        self.wait = False
        self.w_count = 0

    def turn_90(self):
        msg = Twist()

        distance, angdist = self.pythag()

        # stop the turn
        if angdist >= 90.0:
            msg.angular.z = 0.0
            self.turn = False
            self.wait = True
            self.xpos_b = self.xpos
            self.ypos_b = self.ypos
            self.orientation_bench = self.orientation
            msg.linear.x = 0.0
            self.turns += 1.0
        else:
            if np.isnan(angdist):
                msg.angular.z = -0.2
            else:
                msg.angular.z = -0.05 - 0.5 * abs(90 - angdist) / 90

            # print(-1.0 * abs(91 - angdist)/270)
            msg.linear.x = 0.0
        self.vel_pub.publish(msg)

    def forward(self):
        msg = Twist()

        distance, angdist = self.pythag()

        # stop the turn
        if distance >= 1.0:
            msg.linear.x = 0.0
            self.turn = True
            self.wait = True
            self.xpos_b = self.xpos
            self.xpos_b = self.ypos
            self.orientation_bench = self.orientation
            msg.angular.z = 0.0
        else:
            # msg.angular.z = 0.0
            msg.linear.x = 1.1 - distance
            print(distance)
        self.vel_pub.publish(msg)

    def pythag(self):
        distance = math.sqrt(
            (self.xpos - self.xpos_b) ** 2 + (self.ypos - self.ypos_b) ** 2
        )

        # angular_distance = abs(self.ang_bench - self.angular)

        angular_distance = quat_angle(self.orientation, self.orientation_bench)

        print(angular_distance)

        return distance, angular_distance

    def process_odom(self, msg):
        # PoseWithCovariance.pose.
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y

        # robot_q = msg.pose.pose.orientation
        # w,x,y,z
        # my_quaternion = pyq.Quaternion(robot_q.w,robot_q.x,robot_q.y,robot_q.z)
        # self.orientation = my_quaternion
        self.orientation = msg.pose.pose.orientation

    def wait_robot(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)

        self.w_count += 1

        print("waiting")

        if self.w_count > 15:
            self.wait = False
            self.w_count = 0

    def run_loop(self):
        # stop after turn 4
        if self.turns >= 5:
            self.wait_robot()
        elif self.wait:
            self.wait_robot()
        elif self.turn:
            self.turn_90()
        else:
            self.forward()


def main(args=None):
    rclpy.init(args=args)
    node = square_driver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def quat_angle(q1, q2):
    # angle in degrees between two quaternion objects
    inner_prod = q1.w * q2.w + q1.x * q2.x + q1.y + q2.y + q1.z * q2.z

    return 180 / np.pi * np.arccos(2 * (inner_prod**2) - 1)
