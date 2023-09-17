import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

class PersonFollower(Node):
    def __init__(self):
        super().__init__('PersonFollower')
        self.create_timer(0.05, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #xcoord,ycoord,angle from center, positive is left, distance from robot
        self.person_coord = [1,0,0,1]
        self.xpos = 0
        self.ypos = 0

        self.viz_pub = self.create_publisher(Marker,'Sphere',10)

    def process_odom(self,msg):

        #PoseWithCovariance.pose.
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y


    def run_loop(self):
        msg = Twist()
        print(self.person_coord)
        if abs(self.person_coord[2]) > np.pi/2:
            msg.angular.z = -1.0
        else:
            msg.angular.z =  self.person_coord[2]/1.5
            msg.linear.x =   .5*(self.person_coord[3] -.3) #stop around .5 meters away
        self.vel_pub.publish(msg)
        my_mark = self.get_mark()
        self.viz_pub.publish(my_mark)
    
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
        my_mark.scale.x = .3
        my_mark.scale.y = .3
        my_mark.scale.z = .3
        my_mark.header.frame_id = "odom"
        my_mark.header.stamp = self.get_clock().now().to_msg()
        my_mark.ns = "my_namespace"
        my_mark.id = 0
        return my_mark

            
            

    def process_scan(self, msg):
        cutoff_dist = 1.5 #meters
        #short_range = [[i,msg.ranges[i]] for i in range(360) if msg.ranges[i] > cutoff_dist if (i > 30 or 329 < i)]

        short_range = [[i,msg.ranges[i]] for i in range(360) if msg.ranges[i] < cutoff_dist]
        x_list = []
        y_list = []

        for pair in short_range:

            i = pair[0]
            dist = pair[1]

            if np.isinf(dist) or np.isinf(i) or np.isnan(dist) or np.isnan(i) or dist <.01:
                continue
            in_rads = i * np.pi/180
            x_list.append(dist * np.cos(in_rads))
            y_list.append(  dist * np.sin(in_rads))

        if len(x_list) == 0:
            print("fallback coordinates")
            self.person_coord = [1,0,0,1]
        else:
            x_mean = np.mean(x_list)
            y_mean = np.mean(y_list)
            self.person_coord = [x_mean,y_mean,np.arctan2(y_mean,x_mean),self.pythag(x_mean,y_mean)]


    def pythag(self,x,y):
        distance = np.sqrt( ( x)**2 +  ( y)**2)
        return distance
        
        

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()
