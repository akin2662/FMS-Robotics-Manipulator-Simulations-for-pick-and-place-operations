#!/usr/bin/env python3
#importing important libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create an object of the node
class  Plotter(Node):
    def __init__(self):
        super().__init__("Plotter")
        self.odom_sub = self.create_subscription(PoseStamped,'/odom',self.plot_callback,10)
        self.list_1 = []
        self.list_2 = []
        self.list_3 = []
        self.fig = plt.figure().add_subplot(projection = '3d')

    def plot_callback(self,points:PoseStamped):
        values = points
        self.list_1.append(values.pose.position.x)
        self.list_2.append(values.pose.position.y)
        self.list_3.append(values.pose.position.z)
        self.fig.scatter(self.list_1,self.list_2,self.list_3)
        plt.draw()
        plt.pause(0.01)

# Initialize ROS2, Create Node and plot, shutdown ROS2       
def main(args=None):
    rclpy.init(args=args)
    node = Plotter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()