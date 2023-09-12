"""
Used online repository at
https://github.com/rohbotics/ros2_teleop_keyboard/blob/master/teleop_twist_keyboard.py
to understand how teleop works and get scaffold
"""

#!/usr/bin/env python3
import rclpy
import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist

settings = termios.tcgetattr(sys.stdin)
key = None

intro_msg = """
    Reading from the keyboard  and Publishing to Twist!
    ---------------------------
    Moving around:
    u    i    o
    j    k    l
    m    ,    .

    For Holonomic mode (strafing), hold down the shift key:
    ---------------------------
    U    I    O
    J    K    L
    M    <    >

    t : up (+z)
    b : down (-z)

    anything else : stop

    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%

    CTRL-C to quit
    """

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)

    print(intro_msg)

    # setup node and publisher for transmitting telemetry
    node = rclpy.create_node('teleop')		
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # define necessary telemetry defaults
    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    key = ""

    try:
        print(f"current telemetry:\tspeed {speed}\tturn {turn}")
        while key != '\x03':
            key = getKey()
            print(key)

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(f"current telemetry:\tspeed {speed}\tturn {turn}")
            else:
                x = 0
                y = 0
                z = 0
                th = 0
            
            twist = Twist()
            twist.linear.x = x*speed
            twist.linear.y = y*speed
            twist.linear.z = z*speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    main()
