import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class PersonFollower(Node):
    def __init__(self):
        super().__init__('PersonFollower')
        self.create_timer(0.05, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #xcoord,ycoord,angle from center, positive is left, distance from robot
        self.person_coord = [1,0,0,1]

    
    def run_loop(self):
        msg = Twist()
        if abs(self.person_coord[2]) > np.pi/2:
            msg.angular.z = -1.0
        else:
            msg.angular.z =  -self.person_coord[2]/1.5
            msg.linear.x =   .5*(self.person_coord[3] -.3) #stop around .5 meters away
        self.vel_pub.publish(msg)
            
            

    def process_scan(self, msg):
        cutoff_dist = .8 #meters
        short_range = [[i,msg.ranges[i]] for i in range(360) if msg.ranges[i] > cutoff_dist if (i > 30 or 329 < i)]

        x_list = []
        y_list = []
        for pair in short_range:
            i = pair[0]
            dist = pair[1]
            in_rads = i * np.pi/180
            x_list.append(dist * np.cos(in_rads))
            y_list.append(  dist * np.sin(in_rads))

        if len(x_list) == 0:
            self.person_coord = [1,0,0,1]
        else:
            x_mean = np.mean(x_list)
            y_mean = np.mean(y_list)
            self.person_coord = [x_mean,y_mean,np.arctan2(x_mean,y_mean),self.pythag(x_mean,y_mean)]


    def pythag(self,x,y):
        distance = np.sqrt( ( x)**2 +  ( y)**2)
        return distance
        
        

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()
