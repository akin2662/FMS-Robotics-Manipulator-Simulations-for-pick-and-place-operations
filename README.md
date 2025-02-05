# FMS-Robotics-Manipulator-Simulations-for-pick-and-place-operations
**Note:** This project was a requirement for the course ENPM 662- Introduction to Robot Modeling at University of Maryland, College Park

## Project Description:
This project involved developing a 6-DOF robotic manipulator simulation for a Flexible Manufacturing System (FMS) to automate pick-and-place operations, focusing on the implementation of forward and inverse kinematics as well as Lagrangian dynamics. The manipulator was designed in SolidWorks and exported as a URDF file using the SW2URDF exporter.Python and NumPy were used for kinematic calculations, enabling precise robot control and trajectory planning. A custom Gazebo simulation environment was created, where the fully functional manipulator successfully performed pick-and-place operations, mimicking real-world FMS automation.

## Dependencies:

* python 3.11 (any version above 3 should work)
* Python running IDE (Used VSCode)
  
## Libraries Used:
* numpy
* sympy
* sys
* select
* matplotlib
* tty
* termios

## Instructions:

1. Download the zip file and extract it
2. The assembly.zip contains the different parts of the the rally car
3. To run, you will first need to create a ROS2 workspace and build it.
   
   `mkdir -p ~/your_workspace/src`
   
   `cd ~/your_workspace/src`
   
4. Before building the workspace, you need to resolve package dependencies.

   `rosdep install -i --from-path src --rosdistro galactic -y
   
   **Note:** Make sure you are not in the src directory
5. Then run the following command (make sure you are still in your workspace)
   
   `colcon build`
   
6. After successfully building the workspace, you will see 4 directories: build install log src
7. Then go to the src folder and paste the 'robot_arm','odometry' and 'IFRA_ConveyorBelt' packages, extracted from the zip file you downloaded
8. Go to the root of the workspace and colcon build. Then source the overlay
9. Then run the following commands for the simulations:
    **a. For teleop.py:**

     `ros2 launch robot_arm gazebo.launch.py`
     
     to spawn the robot in custom world 'pick_place_can.world' in gazebo
     
    `ros2 run robot_arm teleop.py` 
    
     to run the teleop script
     
    `ros2 run robot_arm gripper.py` 
    
     to turn on the gripper
     
    `ros2 run robot_arm odom_pub.py`
    
     to plot the trajectory
     
    `ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 20}"`
    
     to turn on the conveyor belt
       
    `ros2 service call /switch std_srvs/srv/SetBool data:\ false`
    
     to turn off the gripper

   **b. For test.py:**

   `ros2 launch robot_arm gazebo.launch.py` 
   
    to spawn the robot in custom world 'pick_place_can.world' in gazebo
    
  `ros2 run robot_arm test.py` 
  
    to run the test script
    
`ros2 run robot_arm gripper.py`

  to turn on the gripper)
ros2 run robot_arm odom_pub.py (to plot the trajectory)
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power:
20}" - to turn on the conveyor belt
ros2 service

**Test case:**
**Input:**
Clearance from the obstacles: 5
Enter the start node : 10 10 30
Enter the goal node: 580 180
RPM1 for the robot: 10
RPM2 for the robot: 10
Final Cost:  1001.3822195135446

**Output:**
 reached the goal
Runtime: 25.256956100463867 seconds
note:a video will be saved in the same directory as the code!

### Part2:
1. Place the turtlebot3_project3 package in source folder of your workspace
2. colcon build
3. source install/setup.bash
4. ros2 launch turtlebot3_project3 competition_world.launch.py
5. In new terminal,source install/setup.bash
6. ros2 run turtlebot3_project3 part2_new.py
7. Input the values as asked in the terminal

**Test case:**
**Input:**
1. Clearance: 0.01
2. Start coordinates and angle: 0 0 0
3. Goal coordinates: 5.1 0
4. RPM1 and RPM2: 10 16

## Demo videos:
Following are the links to the recorded simulation
part1: https://drive.google.com/file/d/14HxtT1DbY8zS777Tv_Qrf_5FtBZ3wkad/view?usp=sharing
part2: https://drive.google.com/file/d/1IWEEaPTUh4zzWwEZcqSbLXrzmgNFMoEE/view?usp=sharing

***NOTE:** The simulation environment (world) in gazebo is taken from https://github.com/shantanuparabumd/turtlebot3_project3.git as required for ENPM 661 Project3 Phase2



