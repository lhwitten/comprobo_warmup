import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np
import tty
import select
import sys
import termios

moveBindings = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

speedBindings = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1),
    "x": (0.9, 1),
    "e": (1, 1.1),
    "c": (1, 0.9),
}


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.teleop_mode = True
        self.distance = 0.0
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize teleop settings
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None

        # Initialize person following
        self.person_follower = None

        self.quit = False

    def process_teleop(self):
        try:
            print("Teleop mode active.")
            while self.key != "\x03":
                self.key = self.get_key()
                if (
                    self.key == "\x1b"
                ):  # Check for the escape key (27 is the ASCII code for Esc)
                    self.start_person_following()
                    break  # Exit teleoperation and start person following mode
                elif self.key == "p":
                    self.quit = True
                    break
                elif self.key in moveBindings.keys():
                    self.x, self.y, self.z, self.th = moveBindings[self.key]
                elif self.key in speedBindings.keys():
                    self.speed *= speedBindings[self.key][0]
                    self.turn *= speedBindings[self.key][1]
                    print(f"Current telemetry: speed {self.speed}, turn {self.turn}")
                else:
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn
                self.cmd_vel_publisher.publish(twist)

        except Exception as e:
            print(e)

        finally:
            self.reset_teleop_settings()

    def reset_teleop_settings(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def start_person_following(self):
        if self.person_follower is None:
            self.teleop_mode = False
            self.person_follower = PersonFollower(self)
            self.person_follower.start()

    def stop_person_following(self):
        if self.person_follower is not None:
            self.teleop_mode = True
            self.person_follower.destroy_node()
            self.person_follower = None

    def switch_to_teleop_mode(self):
        self.stop_person_following()  # Stop person following mode
        self.teleop_mode = True
        self.distance = 0.0  # Reset the distance variable
        print("Switched back to Teleop mode.")
        self.process_teleop()  # Restart teleoperation mode


class PersonFollower(Node):
    def __init__(self, controller):
        super().__init__("PersonFollower")
        self.controller = controller
        self.create_timer(0.05, self.run_loop)
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.xpos = 0
        self.ypos = 0
        self.person_coord = [1, 0, 0, 1]
        self.viz_pub = self.create_publisher(Marker, "Sphere", 10)
        self.distance_threshold = (
            1.0  # Distance threshold to switch back to teleoperation
        )

    def start(self):
        print("Person following mode active.")
        self.get_logger().info("Person following mode active.")

    def process_odom(self, msg):
        # PoseWithCovariance.pose.
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y

    def run_loop(self):
        print("person")
        msg = Twist()
        print(self.person_coord)
        if abs(self.person_coord[2]) > np.pi / 2:
            msg.angular.z = -1.0
        else:
            msg.angular.z = self.person_coord[2] / 1.5
            msg.linear.x = 0.5 * (
                self.person_coord[3] - 0.3
            )  # stop around .5 meters away
        self.vel_pub.publish(msg)
        my_mark = self.get_mark()
        self.viz_pub.publish(my_mark)

        # Update the distance variable
        self.controller.distance = self.pythag(
            self.person_coord[0], self.person_coord[1]
        )

        # Check if distance is greater than 1 and switch to teleop mode
        if self.controller.distance > 1.0:
            self.controller.switch_to_teleop_mode()

    def get_mark(self):
        my_mark = Marker()
        my_mark.type = Marker.SPHERE
        my_mark.action = Marker.ADD
        my_mark.color.r = 254.0
        my_mark.color.g = 0.0
        my_mark.color.b = 0.0
        my_mark.color.a = 1.0
        my_mark.pose.position.x = self.xpos + float(self.person_coord[0])
        my_mark.pose.position.y = self.ypos + float(self.person_coord[1])
        my_mark.pose.position.z = 0.0
        my_mark.pose.orientation.x = 0.0
        my_mark.pose.orientation.y = 0.0
        my_mark.pose.orientation.z = 0.0
        my_mark.pose.orientation.w = 1.0
        my_mark.scale.x = 0.3
        my_mark.scale.y = 0.3
        my_mark.scale.z = 0.3
        my_mark.header.frame_id = "odom"
        my_mark.header.stamp = self.get_clock().now().to_msg()
        my_mark.ns = "my_namespace"
        my_mark.id = 0
        return my_mark

    def process_scan(self, msg):
        cutoff_dist = 1.5  # meters
        # short_range = [[i,msg.ranges[i]] for i in range(360) if msg.ranges[i] > cutoff_dist if (i > 30 or 329 < i)]

        short_range = [
            [i, msg.ranges[i]] for i in range(360) if msg.ranges[i] < cutoff_dist
        ]
        x_list = []
        y_list = []

        for pair in short_range:
            i = pair[0]
            dist = pair[1]

            if (
                np.isinf(dist)
                or np.isinf(i)
                or np.isnan(dist)
                or np.isnan(i)
                or dist < 0.01
            ):
                continue
            in_rads = i * np.pi / 180
            x_list.append(dist * np.cos(in_rads))
            y_list.append(dist * np.sin(in_rads))

        if len(x_list) == 0:
            print("fallback coordinates")
            self.person_coord = [1, 0, 0, 1]
        else:
            x_mean = np.mean(x_list)
            y_mean = np.mean(y_list)
            self.person_coord = [
                x_mean,
                y_mean,
                np.arctan2(y_mean, x_mean),
                self.pythag(x_mean, y_mean),
            ]

    def pythag(self, x, y):
        distance = np.sqrt((x) ** 2 + (y) ** 2)
        return distance


def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()
    while not robot_controller.quit:
        if robot_controller.teleop_mode:
            robot_controller.process_teleop()
        else:
            rclpy.spin(robot_controller.person_follower)
            continue

    # Stop person following when teleop exits
    robot_controller.person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
