# This script is a Python program for teleoperation (remote control) of a robot using keyboard input.
# It allows the user to control the robot's movement, speed, and direction using specific keys.
# It is designed for use with ROS 2 (Robot Operating System 2) and publishes Twist messages to control the robot.

# Import necessary libraries and modules
import rclpy
import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist

# Save the current terminal settings for later restoration
settings = termios.tcgetattr(sys.stdin)
key = None

# Multiline introduction message explaining how to use the teleop control
intro_msg = """
    Reading from the keyboard and Publishing to Twist!
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

# Dictionary mapping keyboard keys to movement commands
moveBindings = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

# Dictionary mapping keys for adjusting speed
speedBindings = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1),
    "x": (0.9, 1),
    "e": (1, 1.1),
    "c": (1, 0.9),
}


# Function to read a single keypress from the terminal
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# Main function for teleop control
def main(args=None):
    rclpy.init(args=args)

    print(intro_msg)

    # Setup node and publisher for transmitting telemetry
    node = rclpy.create_node("teleop")
    pub = node.create_publisher(Twist, "cmd_vel", 10)

    # Define necessary telemetry defaults
    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    key = ""

    try:
        print(f"current telemetry:\tspeed {speed}\tturn {turn}")
        # While key is not ctrl+c
        while key != "\x03":
            key = getKey()
            print(key)

            # Update movement commands based on user input
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            # Update speed settings based on user input
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(f"current telemetry:\tspeed {speed}\tturn {turn}")
            else:
                x = 0
                y = 0
                z = 0
                th = 0

            # Create a Twist message with updated velocity values and publish it
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    # Stop the robot by publishing a Twist message with all zero velocities
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)


if __name__ == "__main__":
    main()
