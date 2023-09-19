# Import necessary libraries and message types
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


# Define the Obstacle_avoider class, which is a ROS2 node
class Obstacle_avoider(Node):
    def __init__(self):
        super().__init__("ObstacleAvoider")

        # Create a timer to periodically execute the run_loop function
        self.create_timer(0.1, self.run_loop)

        # Subscribe to LaserScan and Odometry topics
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.create_subscription(Odometry, "odom", self.process_odom, 10)

        # Create a publisher to send Twist commands to control the robot
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize variables to store obstacle and robot positions
        self.person_coord = [1, 0, 0, 1]
        self.xpos = 0.0
        self.ypos = 0.0
        self.x_list = [10.0, 10.0, 10.0]
        self.y_list = [10.0, 10.0, 10.0]

        # Create a publisher for visualization markers (optional)
        self.viz_pub = self.create_publisher(Marker, "Goal", 10)

        # Create a timer to update the gradient descent map
        self.create_timer(0.05, self.run_loop)

        # Initialize a variable to store the gradient sum
        self.fsum = [0.01, 0.01]

    # Define the main control loop
    def run_loop(self):
        msg = Twist()

        # Update the gradient descent map to calculate the desired heading
        self.update_grad()

        # Calculate the desired heading angle based on gradient descent
        theta = np.arctan2(self.fsum[1], self.fsum[0])

        # Set angular and linear velocities to control the robot
        msg.angular.z = theta
        msg.linear.x = 0.5

        # Publish the Twist message to control the robot's motion
        self.vel_pub.publish(msg)

    # Update the gradient descent map based on obstacles and goals
    def update_grad(self):
        # Parameters for gradient descent
        point_weight = 20 * 0.1
        goal_weight = 5.0
        fsum = [0.001, 0.001]

        # Repulsion from obstacles
        for i in range(len(self.x_list)):
            extent = 3  # Obstacle extent in meters

            # Calculate the angle and distance to the obstacle
            theta = np.arctan2(self.y_list[i] - self.ypos, self.x_list[i] - self.xpos)
            dist = np.sqrt(
                (self.x_list[i] - self.xpos) ** 2 + (self.y_list[i] - self.ypos) ** 2
            )

            # Calculate the repulsive force
            force = [
                -point_weight * (extent - dist) * np.cos(theta),
                -point_weight * (extent - dist) * np.sin(theta),
            ]

            if dist > extent:
                force = [0, 0]

            # Accumulate the force components
            fsum[0] += force[0]
            fsum[1] += force[1]

        # Add a goal attraction force
        fsum[0] += 1 * goal_weight
        fsum[1] += 0 * goal_weight

        # Update the gradient sum
        self.fsum = fsum

    # Process the Odometry message to update the robot's position
    def process_odom(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y

    # Process the LaserScan message to update obstacle positions
    def process_scan(self, msg):
        cutoff_dist = 1  # Cutoff distance for detecting obstacles (in meters)

        # Create a list of [angle, distance] pairs for all laser scan points
        short_range = [[i, msg.ranges[i]] for i in range(360)]

        # Initialize empty lists to store obstacle positions
        self.x_list = []
        self.y_list = []

        # Loop through the short_range list to extract valid coordinates
        for pair in short_range:
            i = pair[0]  # Laser scan angle index
            dist = pair[1]  # Distance reading at the given angle

            # Check for invalid readings (e.g., infinity or NaN) and extremely close readings (e.g., noise)
            if (
                np.isinf(dist)
                or np.isinf(i)
                or np.isnan(dist)
                or np.isnan(i)
                or dist < 0.01
            ):
                continue

            # Calculate the x and y coordinates of the detected obstacle in Cartesian coordinates
            in_rads = i * np.pi / 180  # Convert angle to radians
            self.x_list.append(dist * np.cos(in_rads))  # Calculate x-coordinate
            self.y_list.append(dist * np.sin(in_rads))  # Calculate y-coordinate

        # Print a message to indicate that the scan data has been updated
        print("Scan data updated")
        print("Number of obstacles detected:", len(self.x_list))


# Define the main function to initialize the ROS2 node and run the program
def main(args=None):
    rclpy.init(args=args)
    node = Obstacle_avoider()
    rclpy.spin(node)
    rclpy.shutdown()


# Check if the script is being run as the main program
if __name__ == "__main__":
    main()
