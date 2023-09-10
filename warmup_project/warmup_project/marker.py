import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class sphere_publisher(Node):
    def __init__(self):
        super().__init__('sphere_publisher')
        

        self.viz_pub = self.create_publisher(Marker,'Sphere',10)
        self.create_timer(.1,self.run_loop)



    def run_loop(self):
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

        self.viz_pub.publish(my_mark)
    


def main(args=None):
    rclpy.init(args=args)
    node = sphere_publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
