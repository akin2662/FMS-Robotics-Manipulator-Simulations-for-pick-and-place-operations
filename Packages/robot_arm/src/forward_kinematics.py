#!/usr/bin/env python3

import sympy
from sympy import *
import math


# creating an object of transformation matrix of each frame
class TransformationMatrix:

    # creating an object of the basic DH table (Home Pose)

    def __init__(self, theta, alpha, a, d):
        self.theta = theta
        self.alpha = alpha
        self.a = a
        self.d = d

    # writing each element of the matrix
    def matrix(self):
        r11 = cos(self.theta)
        r12 = -sin(self.theta) * cos(self.alpha)
        r13 = sin(self.theta) * sin(self.alpha)
        r14 = self.a * cos(self.theta)
        r21 = sin(self.theta)
        r22 = cos(self.theta) * cos(self.alpha)
        r23 = -cos(self.theta) * sin(self.alpha)
        r24 = self.a * sin(self.theta)
        r31 = 0
        r32 = sin(self.alpha)
        r33 = cos(self.alpha)
        r34 = self.d

        matrix = Matrix([[r11, r12, r13, r14], [r21, r22, r23, r24], [r31, r32, r33, r34], [0, 0, 0, 1]])

        return matrix


# creating an object of each unique robot configuration on top of basic DH table
class DhParameters:
    def __init__(self, joint_angle1, joint_angle2, joint_angle3, joint_angle4,
                 joint_angle5, joint_angle6):
        self.joint_angle1 = joint_angle1
        self.joint_angle2 = joint_angle2
        self.joint_angle3 = joint_angle3
        self.joint_angle4 = joint_angle4
        self.joint_angle5 = joint_angle5
        self.joint_angle6 = joint_angle6

        self.frame_0_1 = TransformationMatrix(math.pi + self.joint_angle1, math.pi/2, 0, 100)
        self.frame_1_2 = TransformationMatrix(-math.pi/2 + self.joint_angle2, -math.pi, -500, -100)
        self.frame_2_3 = TransformationMatrix(math.pi/2 + self.joint_angle3, -math.pi/2, 0, 100)
        self.frame_3_4 = TransformationMatrix(0 + self.joint_angle4, math.pi/2, 0, 500)
        self.frame_4_5 = TransformationMatrix(0 + self.joint_angle5, -math.pi/2, 0, 100)
        self.frame_5_6 = TransformationMatrix(0 + self.joint_angle6, 0, 0, -100)

        final_matrix = (self.frame_0_1.matrix() * self.frame_1_2.matrix() * self.frame_2_3.matrix() * self.frame_3_4.matrix() *
                        self.frame_4_5.matrix() * self.frame_5_6.matrix())
        pprint(final_matrix)



# Getting user inputs to create objects of each uniq joint configuration
def main():
    joint_angle_values = []
    for i in range(0, 6):
        user_input = float(input(f"Please enter the value for joint angle {i}"))
        joint_angle = user_input * (pi / 180)
        joint_angle_values.append(joint_angle)

    robot_config = DhParameters(joint_angle_values[0], joint_angle_values[1], joint_angle_values[2],
                                joint_angle_values[3], joint_angle_values[4], joint_angle_values[5])


if __name__ == '__main__':
    main()