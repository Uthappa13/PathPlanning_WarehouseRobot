## Path Planning for Warehouse Robots Using Informed RRT* and ROS2

## Project Overview

This project implements the Informed RRT* path planning algorithm and simulates the generated path using TurtleBot3 in the Gazebo environment. The algorithm efficiently finds an optimal path between a start and a goal position while avoiding obstacles.

## Languages & Tech Stack

- Programming Language: **Python**

- Robotics Framework: **ROS2**
  
- Simulation Environment: **Gazebo**
  
- Visualization: **Matplotlib**

- Libraries: **numpy, matplotlib, rclpy, geometry_msgs, nav_msgs**

## Features

- Implements the Informed RRT* algorithm for path planning.

- Detects and avoids obstacles within a defined map.

- Simulates path execution using TurtleBot3 in Gazebo.

- Uses ROS2 for robot control and navigation.

- Visualizes the planned path using Matplotlib.

##  Contents

 - `informed_rrt` : This folder contains the source code for the implementation and simulation of Informed RRT* algorithm.

 - `README.md` : markdown file containing instructions and details.

##  How to Use:

To start, please download the .zip file and extract it to retrive the files.

Please follow the following steps to implement and simulate the algorithm in a local system:

   - In the `informed_rrt` folder, you will find the ROS package that contains the *competition_world* launch files along with the python script `informed_rrtstar.py` and `rrtstar.py` to implement the path planning and turtlebot maneuvers in Gazebo. The following steps are to be followed:

      - Create a workspace and add the package retrieved from the above folder to the `src` folder in the workspace.
      - Now, open the workspace in a terminal and build it using the following code:

         > colcon build

      - Then, source the environment using:

         > source install/setup.bash

      - Launch the Gazebo environment in Gazebo:

         > ros2 launch informed_rrt competition_world.launch.py 

      - Now open a new window and source the environment once again

         > source install/setup.bash

      - Run the python node for the Informed RRT* implementation

         > ros2 run informed_rrt informed_rrtstar.py

      - Upon running the above code, the user will be asked to enter the start and goal position coordinates.
   
   **Note:** The coordinates are in cms with x_range (-300, 300) and y_range (-100, 100). Test cases used are:
               1. (-250,0) and (50,-50)
               2. (-250,0) and (0,50)
               3. (-250,0) and (-100,0)

   - Once the goal points are added, the program will explore the nodes.

   - Once the goal is reached, the node exploration and backtrack path is displayed. Close the visulaization window to start the simulation in Gazebo. It then starts to publish the command velocities to the `cmd_vel` topic to run the turtlebot along the trajectory.

 **Note:** To run the RRT* implementation, run the following command:

      > ros2 run informed_rrt rrtstar.py

      This is display the node exploration and the optimal path for RRT* algorithm. There is no Gazebo simulation for RRT*.
