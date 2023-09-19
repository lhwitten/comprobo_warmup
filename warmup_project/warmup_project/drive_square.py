# Import necessary libraries and modules
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np


# Define the square_driver class that wraps the basic functionality of the node
class square_driver(Node):
    def __init__(self):
        super().__init__("square_driver")

        # Create a subscription to Odometry messages to monitor robot position
        self.create_subscription(Odometry, "odom", self.process_odom, 10)

        # Create a publisher for Twist messages to control robot motion
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Create a timer to run the loop
        self.create_timer(0.05, self.run_loop)

        # Determine whether the robot should be turning
        self.turn = False

        # Robot position variables
        self.xpos = 0.0
        self.ypos = 0.0

        # Position of the last turn
        self.xpos_b = 0.0
        self.ypos_b = 0.0
        self.orientation_bench = Quaternion()
        self.turns = 0.0
        self.orientation = Quaternion()
        self.wait = False
        self.w_count = 0

    # Turn the robot by 90 degrees
    def turn_90(self):
        msg = Twist()

        # Calculate the distance and angular distance to the desired orientation
        distance, angdist = self.pythag()

        # Stop the turn if the desired orientation is reached
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
                # Adjust angular velocity based on remaining angular distance
                msg.angular.z = -0.05 - 0.5 * abs(90 - angdist) / 90

            msg.linear.x = 0.0
        self.vel_pub.publish(msg)

    # Move the robot forward
    def forward(self):
        msg = Twist()

        # Calculate the distance and angular distance to the next turn
        distance, angdist = self.pythag()

        # Stop the forward motion if the desired distance is reached
        if distance >= 1.0:
            msg.linear.x = 0.0
            self.turn = True
            self.wait = True
            self.xpos_b = self.xpos
            self.xpos_b = self.ypos
            self.orientation_bench = self.orientation
            msg.angular.z = 0.0
        else:
            # Gradually increase linear velocity as the robot approaches the turn point
            msg.linear.x = 1.1 - distance
            print(distance)
        self.vel_pub.publish(msg)

    # Calculate the Euclidean distance and angular distance between two positions
    def pythag(self):
        distance = math.sqrt(
            (self.xpos - self.xpos_b) ** 2 + (self.ypos - self.ypos_b) ** 2
        )

        # Calculate angular distance using a quaternion-based approach
        angular_distance = quat_angle(self.orientation, self.orientation_bench)

        print(angular_distance)

        return distance, angular_distance

    # Process incoming Odometry messages to update the robot's position and orientation
    def process_odom(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.orientation = msg.pose.pose.orientation

    # Wait for a certain number of iterations to ensure stability
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

    # Main loop for controlling the robot's movement
    def run_loop(self):
        # Stop after 4 turns to form a square
        if self.turns >= 5:
            self.wait_robot()
        elif self.wait:
            self.wait_robot()
        elif self.turn:
            self.turn_90()
        else:
            self.forward()


# Function to calculate the angle between two quaternions
def quat_angle(q1, q2):
    inner_prod = q1.w * q2.w + q1.x * q2.x + q1.y + q2.y + q1.z * q2.z

    # Calculate the angle in degrees between two quaternion objects
    return 180 / np.pi * np.arccos(2 * (inner_prod**2) - 1)


# Main function to initialize the node and spin it
def main(args=None):
    rclpy.init(args=args)
    node = square_driver()
    rclpy.spin(node)
    rclpy.shutdown()


# Entry point of the script
if __name__ == "__main__":
    main()
