import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- KEY MAPPINGS ---
settings = termios.tcgetattr(sys.stdin)

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d

space key : force stop
CTRL-C    : quit
"""

# (x, y, z, angular_z)
moveBindings = {
    'w': (0.5, 0, 0, 0),   # Forward
    's': (-0.5, 0, 0, 0),  # Backward
    'a': (0, 0, 0, 1.0),   # Rotate Left
    'd': (0, 0, 0, -1.0),  # Rotate Right
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select_result = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return select_result

def main():
    rclpy.init()
    node = rclpy.create_node('wasd_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 1.0
    turn = 1.0
    x = 0.0
    th = 0.0

    print(msg)

    try:
        while True:
            key = getKey()
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][3]
            elif key == ' ':
                x = 0.0
                th = 0.0
            elif key == '\x03': # CTRL+C
                break
            else:
                x = 0.0
                th = 0.0

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()