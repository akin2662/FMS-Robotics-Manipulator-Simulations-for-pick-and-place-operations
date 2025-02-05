#!/usr/bin/env python3

#import important libraires
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from std_srvs.srv import SetBool


class TestNode(Node):
    def __init__(self):
        super().__init__("TestNode")
        self.joint_position_publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.joint_positions_1 = Float64MultiArray()
        self.joint_positions_2 = Float64MultiArray()
        self.joint_positions_3 = Float64MultiArray()
        self.joint_positions_4 = Float64MultiArray()
        self.joint_positions_5 = Float64MultiArray()
        self.joint_positions_6 = Float64MultiArray()

        self.joint_angle_1= 0.0
        self.joint_angle_2= 0.0
        self.joint_angle_3= 0.0
        self.joint_angle_4= 0.0
        self.joint_angle_5= 0.0
        self.joint_angle_6= 0.0

    def way_point1(self):
        for i in np.arange(0,-0.0661256774797806, 0.00001):
            self.joint_positions_1.data = [i, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_1)
            self.joint_angle_1 = i

        for i in np.arange(0,1.55363663023134, 0.00001):
            self.joint_positions_2.data = [self.joint_angle_1, i, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_2)
            self.joint_angle_2 = i

        for i in np.arange(0,0.0170533619329440, 0.00001):
            self.joint_positions_3.data = [self.joint_angle_1, self.joint_angle_2, i, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_3)
            self.joint_angle_3 = i

        for i in np.arange(0,-0.142294697319441, 0.00001):
            self.joint_positions_4.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, i, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_4)
            self.joint_angle_4 = i

        for i in np.arange(0,1.53962047901867, 0.00001):
            self.joint_positions_5.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, i, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_5)
            self.joint_angle_5 = i

        for i in np.arange(0,0.0668132858336931, 0.00001):
            self.joint_positions_6.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, i]
            self.joint_position_publisher.publish(self.joint_positions_6)  
            self.joint_angle_6 = i     

    def way_point2(self):
        for i in np.arange(-0.0661256774797806,0, -0.00001):
            self.joint_positions_1.data = [i, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_1)
            self.joint_angle_1 = i

        for i in np.arange(1.55363663023134,0, -0.00001):
            self.joint_positions_2.data = [self.joint_angle_1, i, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_2)
            self.joint_angle_2 = i

        for i in np.arange(0.0170533619329440,0, -0.00001):
            self.joint_positions_3.data = [self.joint_angle_1, self.joint_angle_2, i, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_3)
            self.joint_angle_3 = i

        for i in np.arange(-0.142294697319441,0, -0.00001):
            self.joint_positions_4.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, i, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_4)
            self.joint_angle_4 = i

        for i in np.arange(1.53962047901867,0, -0.00001):
            self.joint_positions_5.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, i, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_5)
            self.joint_angle_5 = i

        for i in np.arange(0.0668132858336931,0, -0.00001):
            self.joint_positions_6.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, i]
            self.joint_position_publisher.publish(self.joint_positions_6)  
            self.joint_angle_6 = i   

    def way_point3(self):
        for i in np.arange(0,0.0271980083006373, -0.00001):
            self.joint_positions_1.data = [i, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_1)
            self.joint_angle_1 = i

        for i in np.arange(0,-0.542046978628637, -0.00001):
            self.joint_positions_2.data = [self.joint_angle_1, i, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_2)
            self.joint_angle_2 = i

        for i in np.arange(0,-0.00471773067753613, -0.00001):
            self.joint_positions_3.data = [self.joint_angle_1, self.joint_angle_2, i, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_3)
            self.joint_angle_3 = i

        for i in np.arange(0,0.0431905470427443, -0.00001):
            self.joint_positions_4.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, i, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_4)
            self.joint_angle_4 = i

        for i in np.arange(0,-0.534335511457615, -0.00001):
            self.joint_positions_5.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, i, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_5)
            self.joint_angle_5 = i

        for i in np.arange(0,-0.0156081507508706, -0.00001):
            self.joint_positions_6.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, i]
            self.joint_position_publisher.publish(self.joint_positions_6)  
            self.joint_angle_6 = i    

    def way_point4(self):
        for i in np.arange(0.0271980083006373,0, -0.00001):
            self.joint_positions_1.data = [i, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_1)
            self.joint_angle_1 = i

        for i in np.arange(-0.542046978628637,0, 0.00001):
            self.joint_positions_2.data = [self.joint_angle_1, i, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_2)
            self.joint_angle_2 = i

        for i in np.arange(-0.00471773067753613,0, 0.00001):
            self.joint_positions_3.data = [self.joint_angle_1, self.joint_angle_2, i, self.joint_angle_4, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_3)
            self.joint_angle_3 = i

        for i in np.arange(0.0431905470427443,0, -0.00001):
            self.joint_positions_4.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, i, self.joint_angle_5, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_4)
            self.joint_angle_4 = i

        for i in np.arange(-0.534335511457615,0, 0.00001):
            self.joint_positions_5.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, i, self.joint_angle_6]
            self.joint_position_publisher.publish(self.joint_positions_5)
            self.joint_angle_5 = i

        for i in np.arange(-0.0156081507508706,0, 0.00001):
            self.joint_positions_6.data = [self.joint_angle_1, self.joint_angle_2, self.joint_angle_3, self.joint_angle_4, self.joint_angle_5, i]
            self.joint_position_publisher.publish(self.joint_positions_6)  
            self.joint_angle_6 = i    

# Note* The value for waypoint1,waypoint2,waypoint3,waypoint4 are obtained from 'inverse_kinematics.py'
def main(args = None):
    rclpy.init(args=args)
    node = TestNode()
    node.way_point1()
    time.sleep(5)
    node.way_point2()
    time.sleep(5)
    node.way_point3()
    time.sleep(5)
    node.way_point4()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()