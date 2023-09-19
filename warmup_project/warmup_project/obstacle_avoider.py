import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

class Obstacle_avoider(Node):
    def __init__(self):
        super().__init__('PersonFollower')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        #self.create_timer(.5,self.update_grad)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #xcoord,ycoord,angle from center, positive is left, distance from robot
        self.person_coord = [1,0,0,1]
        self.xpos = 0.0
        self.ypos = 0.0
        self.x_list = [10.0,10.0,10.0]
        self.y_list = [10.0,10.0,10.0]

        self.viz_pub = self.create_publisher(Marker,'Goal',10)

        self.create_timer(0.05, self.run_loop)
        self.fsum = [.01,.01]

    def run_loop(self):
        msg = Twist()

        self.update_grad()

        theta = np.arctan2(self.fsum[1],self.fsum[0])
        

        msg.angular.z = theta
        msg.linear.x = .5

        self.vel_pub.publish(msg)


    def update_grad(self):
        #update gradient descent map

        point_weight = 20 * .1

        goal_weight = 5.0

        fsum = [.001,0.001]

        #repulsion
        for i in range(len(self.x_list)):

            extent = 3 # meter

            theta = np.arctan2(self.y_list[i] - self.ypos,self.x_list[i]-self.xpos)

            dist = np.sqrt( (self.x_list[i] - self.xpos)**2 +  ( self.y_list[i] - self.ypos)**2)

            #assuming obstacle radius 0
            force = [-point_weight*(extent -dist)*np.cos(theta),-point_weight*(extent -dist)*np.sin(theta)]

            if dist > extent:
                force = [0,0]

            #print("adding force",force[0],force[1])

            fsum[0] += force[0]
            fsum[1] += force[1]

        #now have a direction to go
        fsum[0] += 1*goal_weight
        fsum[1] += 0*goal_weight

        self.fsum = fsum



    def process_odom(self,msg):

        #PoseWithCovariance.pose.
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y

    def process_scan(self, msg):
        cutoff_dist = 1 #meters
        #short_range = [[i,msg.ranges[i]] for i in range(360) if msg.ranges[i] < cutoff_dist]

        short_range = [[i,msg.ranges[i]] for i in range(360)]
        self.x_list = []
        self.y_list = []

        for pair in short_range:

            i = pair[0]
            dist = pair[1]
            

            if np.isinf(dist) or np.isinf(i) or np.isnan(dist) or np.isnan(i) or dist <.01:
                continue
            in_rads = i * np.pi/180
            self.x_list.append(dist * np.cos(in_rads))
            self.y_list.append(  dist * np.sin(in_rads))
        
        print("scan updated")
        print(len(self.x_list))
        

def main(args=None):
    rclpy.init(args=args)
    node = Obstacle_avoider()
    rclpy.spin(node)
    rclpy.shutdown()
