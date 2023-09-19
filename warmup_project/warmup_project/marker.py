# Import necessary libraries and message types
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


# Define the sphere_publisher class, which is a ROS2 node
class sphere_publisher(Node):
    def __init__(self):
        super().__init__("sphere_publisher")

        # Create a publisher to send visualization markers (Sphere) messages
        self.viz_pub = self.create_publisher(Marker, "Sphere", 10)

        # Create a timer to periodically execute the run_loop function
        self.create_timer(0.1, self.run_loop)

    # Define the main run loop function
    def run_loop(self):
        # Create a Marker message to represent a sphere
        my_mark = Marker()
        my_mark.type = Marker.SPHERE
        my_mark.action = Marker.ADD
        my_mark.color.r = 254.0
        my_mark.color.g = 0.0
        my_mark.color.b = 0.0
        my_mark.color.a = 1.0
        my_mark.pose.position.x = 1.0
        my_mark.pose.position.y = 2.0
        my_mark.pose.position.z = 0.0
        my_mark.pose.orientation.x = 0.0
        my_mark.pose.orientation.y = 0.0
        my_mark.pose.orientation.z = 0.0
        my_mark.pose.orientation.w = 1.0
        my_mark.scale.x = 5.0
        my_mark.scale.y = 5.1
        my_mark.scale.z = 5.1
        my_mark.header.frame_id = "odom"
        my_mark.header.stamp = self.get_clock().now().to_msg()
        my_mark.ns = "my_namespace"
        my_mark.id = 0

        # Publish the Marker message
        self.viz_pub.publish(my_mark)


# Define the main function to initialize the ROS2 node and run the program
def main(args=None):
    rclpy.init(args=args)
    node = sphere_publisher()
    rclpy.spin(node)
    rclpy.shutdown()


# Check if the script is being run as the main program
if __name__ == "__main__":
    main()
