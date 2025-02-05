#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
ANG_VEL_STEP_SIZE = 0.01

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)


        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Arm!
        ---------------------------
        Link 1: Z   X

        Link 2: A   S

        Link 3: Q   W

        Link 4: N   M

        Link 5: K   L

        Link 6: O   P

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions_1 = Float64MultiArray()
        joint_positions_2 = Float64MultiArray()
        joint_positions_3 = Float64MultiArray()
        joint_positions_4 = Float64MultiArray()
        joint_positions_5 = Float64MultiArray()
        joint_positions_6 = Float64MultiArray()

        joint_angle_1=0.0
        joint_angle_2=0.0
        joint_angle_3=0.0
        joint_angle_4=0.0
        joint_angle_5=0.0
        joint_angle_6=0.0



        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'z':  # Left_joint_1
                    joint_angle_1 -= ANG_VEL_STEP_SIZE
                    joint_positions_1.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_1)
                    self.get_logger().info(f"joint 1: {joint_angle_1}")
                elif key == 'x':  # Right_joint_1
                    joint_angle_1 += ANG_VEL_STEP_SIZE
                    joint_positions_1.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_1)
                    self.get_logger().info(f"joint 1: {joint_angle_1}")
                elif key == 'a':  # Left_joint_2
                    joint_angle_2 -= ANG_VEL_STEP_SIZE
                    joint_positions_2.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_2)
                    self.get_logger().info(f"joint 2: {joint_angle_2}")
                elif key == 's':  # Right_joint_2
                    joint_angle_2 += ANG_VEL_STEP_SIZE
                    joint_positions_2.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_2)
                    self.get_logger().info(f"joint 2: {joint_angle_2}")
                elif key == 'q':  # Left_joint_3
                    joint_angle_3 -= ANG_VEL_STEP_SIZE
                    joint_positions_3.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_3)
                    self.get_logger().info(f"joint 3: {joint_angle_3}")
                elif key == 'w':  # Right_joint_3
                    joint_angle_3 += ANG_VEL_STEP_SIZE
                    joint_positions_3.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_3)
                    self.get_logger().info(f"joint 3: {joint_angle_3}")
                elif key == 'n':  # Left_joint_4
                    joint_angle_4 -= ANG_VEL_STEP_SIZE
                    joint_positions_4.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_4)
                    self.get_logger().info(f"joint 4: {joint_angle_4}")
                elif key == 'm':  # Right_joint_4
                    joint_angle_4 += ANG_VEL_STEP_SIZE
                    joint_positions_4.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_4)
                    self.get_logger().info(f"joint 4: {joint_angle_4}")
                elif key == 'k':  # Left_joint_5
                    joint_angle_5 -= ANG_VEL_STEP_SIZE
                    joint_positions_5.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_5)
                    self.get_logger().info(f"joint 5: {joint_angle_5}")
                elif key == 'l':  # Right_joint_5
                    joint_angle_5 += ANG_VEL_STEP_SIZE
                    joint_positions_5.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_5)
                    self.get_logger().info(f"joint 5: {joint_angle_5}")
                elif key == 'o':  # Left_joint_6
                    joint_angle_6 -= ANG_VEL_STEP_SIZE
                    joint_positions_6.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_6)
                    self.get_logger().info(f"joint 6: {joint_angle_6}")
                elif key == 'p':  # Right_joint_6
                    joint_angle_6 += ANG_VEL_STEP_SIZE
                    joint_positions_6.data = [joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6]
                    self.joint_position_pub.publish(joint_positions_6)
                    self.get_logger().info(f"joint 6: {joint_angle_6}")
                


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()