#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
========================================
   SAUVC 3D VECTORED 6-DoF AUV TELEOP
========================================
Translation (Linear 3-DoF):
   w / s : Surge Forward / Backward
   a / d : Sway Left / Right (Strafe)
   r / f : Heave Up / Down (Ascend / Descend)

Rotation (Angular 3-DoF):
   q / e : Yaw Left / Right (Turn)
   i / k : Pitch Down / Up (Nose down / up)
   j / l : Roll Left / Right (Bank)

   space : Full Stop (Zero all velocities)
   CTRL-C to exit
========================================
"""

# Format: (linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
moveBindings = {
    # Translations
    'w': ( 0.5,  0.0,  0.0,  0.0,  0.0,  0.0),
    's': (-0.5,  0.0,  0.0,  0.0,  0.0,  0.0),
    'a': ( 0.0,  0.4,  0.0,  0.0,  0.0,  0.0),
    'd': ( 0.0, -0.4,  0.0,  0.0,  0.0,  0.0),
    'r': ( 0.0,  0.0,  0.3,  0.0,  0.0,  0.0),
    'f': ( 0.0,  0.0, -0.3,  0.0,  0.0,  0.0),

    # Rotations (6-DoF)
    'q': ( 0.0,  0.0,  0.0,  0.0,  0.0,  0.5), # Yaw Left
    'e': ( 0.0,  0.0,  0.0,  0.0,  0.0, -0.5), # Yaw Right
    'i': ( 0.0,  0.0,  0.0,  0.0,  0.4,  0.0), # Pitch nose down
    'k': ( 0.0,  0.0,  0.0,  0.0, -0.4,  0.0), # Pitch nose up
    'j': ( 0.0,  0.0,  0.0, -0.4,  0.0,  0.0), # Roll Left
    'l': ( 0.0,  0.0,  0.0,  0.4,  0.0,  0.0), # Roll Right

    ' ': ( 0.0,  0.0,  0.0,  0.0,  0.0,  0.0)  # Stop
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('sauvc_6dof_teleop')
    pub = node.create_publisher(Twist, '/sauvc/cmd_vel', 10)

    print(msg)
    try:
        while True:
            key = getKey(settings)
            if key in moveBindings:
                lx, ly, lz, ax, ay, az = moveBindings[key]
                twist = Twist()
                twist.linear.x = lx
                twist.linear.y = ly
                twist.linear.z = lz
                twist.angular.x = ax
                twist.angular.y = ay
                twist.angular.z = az
                pub.publish(twist)
            elif key == '\x03': # CTRL-C
                break
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
