# Import necessary libraries and modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


# Define the PersonFollower class to control a robot following a person
class PersonFollower(Node):
    def __init__(self):
        super().__init__("PersonFollower")

        # Create a timer for the main loop
        self.create_timer(0.05, self.run_loop)

        # Create a subscription to LaserScan messages for processing sensor data
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)

        # Create a subscription to Odometry messages to monitor the robot's position
        self.create_subscription(Odometry, "odom", self.process_odom, 10)

        # Create a publisher for Twist messages to control robot motion
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize the coordinates of the person being followed
        # Format: [xcoord, ycoord, angle from center, positive is left, distance from robot]
        self.person_coord = [1, 0, 0, 1]

        # Initialize robot position variables
        self.xpos = 0
        self.ypos = 0

        # Create a publisher for visualization markers
        self.viz_pub = self.create_publisher(Marker, "Sphere", 10)

    # Process incoming Odometry messages to update the robot's position
    def process_odom(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y

    # Main loop for controlling the robot's movement
    def run_loop(self):
        msg = Twist()
        print(self.person_coord)

        # Adjust robot angular velocity to follow the person
        if abs(self.person_coord[2]) > np.pi / 2:
            msg.angular.z = -1.0
        else:
            msg.angular.z = self.person_coord[2] / 1.5
            msg.linear.x = 0.5 * (
                self.person_coord[3] - 0.3
            )  # Stop around 0.5 meters away

        # Publish the Twist message to control robot motion
        self.vel_pub.publish(msg)

        # Create and publish visualization markers
        my_mark = self.get_mark()
        self.viz_pub.publish(my_mark)

    # Generate a visualization marker for the person being followed
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

    # Process incoming LaserScan messages to detect and track the person
    def process_scan(self, msg):
        # Set a distance threshold for detecting nearby objects
        cutoff_dist = 1.5  # meters

        # Filter out short-range laser readings using list comprehension
        short_range = [
            [i, msg.ranges[i]] for i in range(360) if msg.ranges[i] < cutoff_dist
        ]

        # Initialize empty lists to store x and y coordinates of detected objects
        x_list = []
        y_list = []

        # Loop through the short_range list to extract valid coordinates
        for pair in short_range:
            i = pair[0]  # Laser scan angle index
            dist = pair[1]  # Distance reading at the given angle

            # Check for invalid or extremely close readings (e.g., noise)
            if (
                np.isinf(dist)
                or np.isinf(i)
                or np.isnan(dist)
                or np.isnan(i)
                or dist < 0.01
            ):
                continue

            # Calculate the x and y coordinates of the detected point in Cartesian coordinates
            in_rads = i * np.pi / 180  # Convert angle to radians
            x_list.append(dist * np.cos(in_rads))  # Calculate x-coordinate
            y_list.append(dist * np.sin(in_rads))  # Calculate y-coordinate

        # Check if any valid points were detected
        if len(x_list) == 0:
            print(
                "Fallback coordinates"
            )  # Display a message if no valid points are detected
            self.person_coord = [
                1,
                0,
                0,
                1,
            ]  # Set a default coordinate (e.g., assume a person at a fixed position)
        else:
            # Calculate the mean (average) x and y coordinates of detected points
            x_mean = np.mean(x_list)
            y_mean = np.mean(y_list)

            # Calculate the angle from the robot to the detected point (in radians)
            angle = np.arctan2(y_mean, x_mean)

            # Calculate the distance from the robot to the detected point
            distance = self.pythag(x_mean, y_mean)

            # Update the person's coordinates with the calculated values
            self.person_coord = [x_mean, y_mean, angle, distance]

    # Calculate the Euclidean distance
    def pythag(self, x, y):
        distance = np.sqrt(x**2 + y**2)
        return distance


# Main function to initialize the node and spin it
def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()


# Entry point of the script
if __name__ == "__main__":
    main()
