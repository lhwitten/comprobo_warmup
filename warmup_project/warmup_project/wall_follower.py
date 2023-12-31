# Import necessary libraries and modules
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data
import random
import math

# Constants
DISTANCE_TOLERANCE = 0.5
SPEED = 0.2


# Define the WallFollowerNode class that wraps the basic functionality of the node
class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.create_timer(0.1, self.run_loop)

        # Create a subscription to LaserScan messages
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )

        # Create publishers for Twist and Marker messages
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.viz_pub = self.create_publisher(Marker, "Sphere", 10)

        self.yint = None
        self.slope = None
        self.dist = None
        self.xscan = None
        self.yscan = None
        self.current_msg = Twist()
        self.current_msg.linear.x = SPEED

    # Callback function for processing LaserScan messages
    def process_scan(self, msg):
        x_coords = []
        y_coords = []

        for i in range(360):
            in_rads = i * np.pi / 180
            x = msg.ranges[i] * np.cos(in_rads)
            y = msg.ranges[i] * np.sin(in_rads)

            # Filter out invalid and distant points
            if not np.isinf(x) and not np.isnan(x) and msg.ranges[i] <= 1.0:
                x_coords.append(x)
                y_coords.append(y)

        self.xscan = x_coords
        self.yscan = y_coords

    # Function to calculate angular velocity based on error
    def find_ang_vel(self):
        self.current_msg.angular.z = -self.error / 1.3 + (0.3 - self.dist) / 1.7

    # Random sample consensus (RANSAC) algorithm to estimate a line from points
    def ransac(self):
        iterations = 40
        best_r2 = 0
        num_points = 30
        slope = 1
        yint = 1
        dist = 1

        for _ in range(iterations):
            # Randomly select a subset of points
            indices = random.choices(range(len(self.xscan)), weights=None, k=num_points)
            x_list = np.array([self.xscan[i] for i in indices])
            y_list = np.array([self.yscan[i] for i in indices])

            # Fit a line to the selected points using least squares
            A = np.vstack([x_list, np.ones(len(x_list))]).T
            model, resid = np.linalg.lstsq(A, y_list, rcond=None)[:2]
            m, b = model
            r2 = 1 - resid / (len(y_list) * np.var(y_list))
            d = abs(b) / np.sqrt(m**2 + 1)

            # Update the best line parameters if the fit is better
            if r2 > best_r2:
                best_r2 = r2
                slope = m
                yint = b
                dist = d

        return slope, yint, dist

    # Calculate the turn angle based on the estimated line
    def calculate_turn_angle(self):
        slope, yint, dist = self.ransac()
        self.slope = slope
        self.yint = yint
        self.dist = dist
        opp = yint
        adj = -yint / slope
        self.adj = adj
        self.add_line()
        self.error = math.atan2(adj, opp)

    # Main loop for the wall-following behavior
    def run_loop(self):
        if self.xscan is None or self.yscan is None:
            return
        self.calculate_turn_angle()
        self.find_ang_vel()
        self.vel_pub.publish(self.current_msg)

    # Add a line to visualization for the detected wall
    def add_line(self):
        found_points = 0
        while found_points < 10:
            if self.adj > 0:
                xpoint = random.uniform(0, self.adj)
            else:
                xpoint = random.uniform(self.adj, 0)
            ypoint = self.slope * xpoint + self.yint

            # Ensure points are within a reasonable range for visualization
            if np.sqrt(xpoint**2 + ypoint**2) < 10:
                found_points += 1
                self.add_marker(xpoint, ypoint, 1.0)

    # Add a marker to visualization
    def add_marker(self, x, y, z):
        my_mark = Marker()
        my_mark.type = Marker.SPHERE
        my_mark.action = Marker.ADD
        my_mark.color.r = 255.0
        my_mark.color.g = 0.0
        my_mark.color.b = 0.0
        my_mark.color.a = 1.0
        my_mark.pose.position.x = x
        my_mark.pose.position.y = y
        my_mark.pose.position.z = z
        my_mark.pose.orientation.x = 0.0
        my_mark.pose.orientation.y = 0.0
        my_mark.pose.orientation.z = 0.0
        my_mark.pose.orientation.w = 1.0
        my_mark.scale.x = 1.0
        my_mark.scale.y = 1.0
        my_mark.scale.z = 1.0
        my_mark.header.frame_id = "odom"
        my_mark.header.stamp = self.get_clock().now().to_msg()
        my_mark.ns = "my_namespace"

        self.viz_pub.publish(my_mark)


# Main function to initialize the node and spin it
def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


# Entry point of the script
if __name__ == "__main__":
    main()
