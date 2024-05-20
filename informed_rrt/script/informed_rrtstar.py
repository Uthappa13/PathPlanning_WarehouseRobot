#!/usr/bin/env python3

###################################################### import the required libraries ###################################################################
import numpy as np
import math
import time
import random
from math import pow, atan2, sqrt
from matplotlib import pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


################################################# Function definition to convert quaternion to euler ####################################################
def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    # Compute the roll angle in radians
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    # Compute the pitch angle in radians
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    # Compute the yaw angle in radians
    yaw = math.atan2(t3, t4)

    # return the roll, pitch and yaw angles in radians
    return roll, pitch, yaw 


#################################################### Define a class for the informed RRT* algorithm ######################################################
class InformedRRTStar(object):
    
    
    # Define the init function
    def __init__(self, start, goal):
        
        # start variable - tuple of of form (x, y)
        self.start = start
        
        # goal variable - tuple of form (x, y)
        self.goal = goal
        
        # map size is 600 cm x 200 cm
        self.x_length = 300
        self.y_length = 100
        
        # clearance of the robot from the obstacle
        self.clearance = 5.0
        
        # radius of the robot 
        self.radius = 10.0
                
        # hashmap to store the distance of the nodes from the start node
        self.costToCome = {}
        
        # hashmap used for backtracking from the goal node to the start node
        self.path = {}
        
        # goalThreshold - threshold from goal node
        self.goal_threshold = 15
        
        # vertices of the graph
        self.vertices = []
        
        # step size
        self.stepSize = 6
        
        # step factor
        self.stepFactor = 9
        
        

    # Function definition to check if a node is within the map dimensions
    def node_in_map(self, curr_X, curr_Y):
        
        nodeInMap = (curr_X >= (-self.x_length + self.radius + self.clearance) and curr_X <= (self.x_length - self.radius - self.clearance) and curr_Y >= (-self.y_length + self.radius + self.clearance) and curr_Y <= (self.y_length - self.radius - self.clearance))
        # return the bool value
        return nodeInMap
    
    

    # Function definition to check the obstacles in the map
    def node_in_obstacle(self, x_pos, y_pos):
        
        # define the constants
        c_r = self.clearance + self.radius
        cl_value = 1.4142 * c_r
        
        # check if the node is in the circle obstacle
        obs_1 = ((x_pos - 120.0) * (x_pos - 120.0) + (y_pos - 20) * (y_pos - 20)) - ((60 + c_r) * (60 + c_r))
        
        # check if the node is in the first rectangle
        (x1, y1) = (-50 - cl_value, -100)
        (x2, y2) = (-50 - cl_value, cl_value)
        (x3, y3) = (-25 + cl_value, cl_value)
        (x4, y4) = (-25 + cl_value, -100)
        side_1 = ((y_pos - y1) * (x2 - x1)) - ((y2 - y1) * (x_pos - x1))
        side_2 = ((y_pos - y2) * (x3 - x2)) - ((y3 - y2) * (x_pos - x2))
        side_3 = ((y_pos - y3) * (x4 - x3)) - ((y4 - y3) * (x_pos - x3))
        side_4 = ((y_pos - y4) * (x1 - x4)) - ((y1 - y4) * (x_pos - x4))
        obs_2_1 = 1
        obs_2_2 = 1
        if(side_1 <= 0 and side_2 <= 0 and side_3 <= 0 and side_4 <= 0):
            obs_2_1 = 0
            obs_2_2 = 0
        
        # check if the node is in the second rectangle
        (x1, y1) = (-150 - cl_value, 100)
        (x2, y2) = (-150 - cl_value,  - cl_value)
        (x3, y3) = (-125 + cl_value, - cl_value)
        (x4, y4) = (-125 + cl_value, 100)
        side_1 = ((y_pos - y1) * (x2 - x1)) - ((y2 - y1) * (x_pos - x1))
        side_2 = ((y_pos - y2) * (x3 - x2)) - ((y3 - y2) * (x_pos - x2))
        side_3 = ((y_pos - y3) * (x4 - x3)) - ((y4 - y3) * (x_pos - x3))
        side_4 = ((y_pos - y4) * (x1 - x4)) - ((y1 - y4) * (x_pos - x4))
        obs_3_1 = 1
        obs_3_2 = 1
        if(side_1 >= 0 and side_2 >= 0 and side_3 >= 0 and side_4 >= 0):
            obs_3_1 = 0
            obs_3_2 = 0

        
        # return true if obstacle, otherwise false
        if(obs_1 <= 0 or obs_2_1 == 0 or obs_2_2 == 0 or obs_3_1 == 0 or obs_3_2 == 0):
            return True
        return False
    
    
    # Function definition to visualize the algorithm
    def animate(self, exploredStates, backtrackStates):

        startX = []
        startY = []
        endX = []
        endY = []
        explored_startX = []
        explored_startY = []
        explored_endX = []
        explored_endY = []

        # Plot the map boundary
        fig, ax = plt.subplots()
        plt.xlabel("x-coordinate(in cm)")
        plt.ylabel("y-coordinate(in cm)")
        plt.grid()
        ax.set_aspect('equal')
        plt.xlim(-self.x_length , self.x_length )
        plt.ylim(-self.y_length , self.y_length )
        count = 0
        
        # Plot the obstacle space
        O_X = []
        O_Y = []
        size = []
        for index1 in range(-self.x_length, self.x_length):
            for index2 in range(-self.y_length, self.y_length):
                if(self.node_in_obstacle(index1, index2)):
                    O_X.append(index1 )
                    O_Y.append(index2 )     
                    size.append(15)      
        O_X = np.array(O_X)
        O_Y = np.array(O_Y)
        plt.scatter(O_X, O_Y, color='b', s=size)

        # explore the node space
        for index in range(1, len(exploredStates)):
            parentNode = self.path[exploredStates[index]]
            explored_startX.append(parentNode[0] )
            explored_startY.append(parentNode[1] )
            explored_endX.append((exploredStates[index][0] - parentNode[0]) )
            explored_endY.append((exploredStates[index][1] - parentNode[1]) )    
            count = count + 1

        # backtrack the states
        if(len(backtrackStates) > 0):
            for index in range(1, len(backtrackStates)):
                startX.append(backtrackStates[index-1][0] )
                startY.append(backtrackStates[index-1][1] )
                endX.append((backtrackStates[index][0] - backtrackStates[index-1][0]) )
                endY.append((backtrackStates[index][1] - backtrackStates[index-1][1]) )    
                count = count + 1

        # Plot the node exploration and the backtrack
        plt.quiver(np.array((explored_startX)), np.array((explored_startY)), np.array((explored_endX)), np.array((explored_endY)), units = 'xy', scale = 1, color = 'g', label = 'Explored region')
        if(len(backtrackStates) > 0):
            plt.quiver(np.array((startX)), np.array((startY)), np.array((endX)), np.array((endY)), units = 'xy', scale = 1, color = 'r', label = 'Backtrack path')
        plt.legend()
        plt.show()
        plt.close()
    
    

    # Define the eucledian heuristic
    def euc_heuristic(self, point1, point2):

        # return the eucledian distance between two points
        return (np.sqrt(((point2[0] - point1[0]) ** 2) + ((point2[1] - point1[1]) ** 2)))
    
    

    # Function definition to generate the random nodes
    def get_random_node(self, cMax, cMin, xCenter, c_matrix):
        
        (rand_X, rand_Y) = (None, None)

        if(cMax < float('inf')):
            
            # generate the r matrix
            if (cMax ** 2) >= (cMin ** 2):
                sqrt_component = np.sqrt((cMax ** 2) - (cMin ** 2)) / 2.0
            else:
                sqrt_component = 0 
            r_matrix = [(cMax / 2.0), sqrt_component, sqrt_component]

            
            # generate l matrix
            l_matrix = np.diag(r_matrix)
            

            # generate xBall matrix
            a = random.random()
            b = random.random()
            
            if(b < a):
                a, b = b, a
            
            sample = (b * np.cos(2 * np.pi * a / b), b * np.sin(2 * np.pi * a / b))
            xBall = np.array([[sample[0]], [sample[1]], [0]])
            rand = np.dot(np.dot(c_matrix, l_matrix), xBall) + xCenter
            rand_X = round(rand[(0, 0)], 2)
            rand_Y = round(rand[(1, 0)], 2)
        else:
            rand_X = round(random.uniform((-self.x_length + self.radius + self.clearance), (self.x_length - self.radius - self.clearance)), 2)
            rand_Y = round(random.uniform((-self.y_length + self.radius + self.clearance), (self.y_length - self.radius - self.clearance)), 2)
        return (rand_X, rand_Y)
    
    

    # Function definition to find the nearest neighbour in the graph
    def get_nearest_neighbour(self, curr_X, curr_Y):
        
        # set vertex to -1
        min_d = float('inf')
        nearestVertex = -1
        
        # loop through the vertices of graph
        for vertex in self.vertices:
            dst = self.euc_heuristic(vertex, (curr_X, curr_Y))
            if(dst < min_d):
                min_d = dst
                nearestVertex = vertex
        
        # return nearest vertex
        return nearestVertex
    
    

    # Function to check obstacle between points
    def check_obstacle(self, point1, point2):
        
        # calculate the difference
        diff_1 = point2[0] - point1[0]
        diff_2 = point2[1] - point1[1]
        
        # points to check for obstacle
        points_to_check = []
        points_to_check.append(point1)
        
        # get value of diff
        if(np.abs(diff_1) > np.abs(diff_2)):
            diff = np.abs(diff_1)
        else:
            diff = np.abs(diff_2)
        
        for index in range(1, int(np.abs(diff))):
            point = (point1[0] + (index * diff_1 / np.abs(diff)), point1[1] + (index * diff_2 / np.abs(diff)))
            points_to_check.append(point)
        
        # check for obstacle
        for point in points_to_check:
            if(self.node_in_obstacle(point[0], point[1]) or self.node_in_map(point[0], point[1]) == False):
                return True
        return False
    
    
    # Function definition to find a new node
    def get_new_node(self, x_rand, x_nearest):
        
        # slope of line joining x_rand and x_nearest
        slope = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0])
        factor = self.stepSize * np.sqrt(1.0 / (1.0 + (slope ** 2)))
        
        # get the two possible points
        point_1 = (round(x_nearest[0] + factor, 2), round(x_nearest[1] + (slope * factor), 2))
        point_2 = (round(x_nearest[0] - factor, 2), round(x_nearest[1] - (slope * factor), 2))
        flag1 = False
        flag2 = False
        
        # check for obstacles
        if(self.check_obstacle(x_nearest, point_1)):
            flag1 = True
        if(self.check_obstacle(x_nearest, point_2)):
            flag2 = True
        
        # return point with minimum distance to random node
        d_1 = self.euc_heuristic(x_rand, point_1)
        d_2 = self.euc_heuristic(x_rand, point_2)
        if(d_1 < d_2):
            return (flag1, point_1)
        else:
            return (flag2, point_2)
    
    

    # Function definition to get the neighbourhood
    def get_neighbourhood(self, x_new):
        
        # iterate through the vertices and get nodes within a certain radius
        neighbourhood = []
        for index in range(0, len(self.vertices)):
            dist = self.euc_heuristic(x_new, self.vertices[index])
            if(dist < self.stepFactor):
                neighbourhood.append(self.vertices[index])
        return neighbourhood
    
    

    # Function definition to get neighbourhood parent
    def get_neighbourhood_parent(self, neighbourhood):
        
        dist = self.costToCome[neighbourhood[0]]
        parent = neighbourhood[0]
        for index in range(1, len(neighbourhood)):
            curr_dist = self.costToCome[neighbourhood[index]]
            if(curr_dist < dist):
                dist = curr_dist
                parent = neighbourhood[index]
        return parent
    
    
    # Define the search algorithm for informed rrt*
    def search(self):
        
        # initializations
        self.costToCome[self.start] = 0
        self.vertices.append(self.start)
        backtrackStates = []
        c_max = float('inf')
        c_min = np.sqrt((self.start[0] - self.goal[0]) ** 2 + (self.start[1] - self.goal[1]) ** 2)
        xCenter = np.matrix([[(self.start[0] + self.goal[0]) / 2.0], [(self.start[1] + self.goal[1]) / 2.0], [0]])
        m_matrix = np.dot(np.matrix([[(self.goal[0] - self.start[0]) / c_min], [(self.goal[1] - self.start[1]) / c_min], [0]]) , np.matrix([1.0, 0.0, 0.0]))
        u_matrix, s_matrix, vh_matrix = np.linalg.svd(m_matrix, 1, 1)
        c_matrix = np.dot(np.dot(u_matrix, np.diag([1.0, 1.0, np.linalg.det(u_matrix) * np.linalg.det(np.transpose(vh_matrix))])), vh_matrix)
        

        for step in range(0, 10000):
            
            # get the random node
            (x_rand_x, x_rand_y) = self.get_random_node(c_max, c_min, xCenter, c_matrix)
            x_rand = (x_rand_x, x_rand_y)
            
            # get the nearest node
            (x_nearest_x, x_nearest_y) = self.get_nearest_neighbour(x_rand_x, x_rand_y)
            x_nearest = (x_nearest_x, x_nearest_y)
            
            # check whether x_nearest[0] == x_rand[0] or x_nearest[1] == x_rand[1]
            if((x_nearest[0] == x_rand[0]) or (x_nearest[1] == x_rand[1])):
                continue
    
            # get the new node between x_nearest and x_rand
            (flag, x_new) = self.get_new_node(x_rand, x_nearest)
            if(flag == True):
                continue
            
            # get the neighbourhood region for x_new
            neighbourhood = self.get_neighbourhood(x_new)
            
            # get the parent for the neighbourhood region
            parent = self.get_neighbourhood_parent(neighbourhood)
            x_nearest = parent
            
            # check the obstacle between x_nearest and x_new
            if(self.check_obstacle(x_nearest, x_new)):
                continue
            
            # add x_new to graph
            self.vertices.append(x_new)
            self.path[x_new] = x_nearest
            self.costToCome[x_new] = self.costToCome[x_nearest] + self.euc_heuristic(x_nearest, x_new)
            
            # rewire the graph
            for index in range(0, len(neighbourhood)):
                distance_from_start = self.costToCome[x_new] + self.euc_heuristic(x_new, neighbourhood[index])
                if(distance_from_start < self.costToCome[neighbourhood[index]]):
                    self.costToCome[neighbourhood[index]] = distance_from_start
                    self.path[neighbourhood[index]] = x_new
            
            # check distance between goal and x_new
            dist_from_goal = self.euc_heuristic(x_new, self.goal)
            if(dist_from_goal <= self.goal_threshold):
                backtrackNode = x_new
                
                # backtrack path
                temp_path = []
                temp_len = self.costToCome[backtrackNode]
                while(backtrackNode != self.start):
                    temp_path.append(backtrackNode)
                    backtrackNode = self.path[backtrackNode]
                temp_path.append(self.start)
                temp_path = list(reversed(temp_path))
                
                # update path if length minimal
                if(c_max > temp_len):
                    c_max = temp_len
                    backtrackStates = temp_path
        
        # return explored and backtrack states
        return (self.vertices, backtrackStates, c_max)
    

###################################################### Class definition for ROS node ############################################################
class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_rrt')

        # Global variables initialization
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # create a publisher to cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # create a subscriber to odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  

    # Function definition for the subscriber callback
    def odom_callback(self, msg):
        # Update robot's current position and orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
            )

    # Move the robot to the waypoint
    def go_to_point(self, robot_x, robot_y):
        goal = Point()
        goal.x = (robot_x + 250.0) / 100.0
        goal.y = robot_y / 100.0
        vel_msg = Twist()

        while rclpy.ok():
            inc_x = goal.x - self.x
            inc_y = goal.y - self.y
            angle_to_goal = atan2(inc_y, inc_x)

            angle_difference = angle_to_goal - self.theta

            if abs(angle_difference) > 0.1:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.3 if angle_difference > 0 else -0.3
            else:
                vel_msg.linear.x = 0.2
                vel_msg.angular.z = 0.0

            # Check proximity to the goal, considering a small tolerance for arriving at the goal
            if abs(inc_x) < 0.02 and abs(inc_y) < 0.02:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.publisher_.publish(vel_msg)
                break

            self.publisher_.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)


########################################################### Instantiate the node class ###############################################################
def turtlebot(robot_x, robot_y, args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    turtlebot_controller.go_to_point(robot_x, robot_y)  
    turtlebot_controller.destroy_node()
    rclpy.shutdown()


############################################################### Main function ########################################################################
if __name__ == '__main__':

    # enter the start and the goal nodes
    print()
    print()
    startX = float(input("Enter the x-coordinate for start node(in cm) : "))
    startY = float(input("Enter the y-coordinate for start node(in cm) : "))
    goalX = float(input("Enter the x-coordinate for goal node(in cm) : "))
    goalY = float(input("Enter the y-coordinate for goal node(in cm) : "))

    start = (startX , startY)
    goal = (goalX , goalY)
    informed_rrt = InformedRRTStar(start, goal)

    # check if the start and the goal nodes are valid
    if(informed_rrt.node_in_map(start[0], start[1])):
        if(informed_rrt.node_in_map(goal[0], goal[1])):
            if(informed_rrt.node_in_obstacle(start[0],start[1]) == False):
                if(informed_rrt.node_in_obstacle(goal[0], goal[1]) == False):
                    start_time = time.time()
                    (explored_states, backtrack_states, goal_cost) = informed_rrt.search()
                    end_time = time.time()
                    duration = end_time - start_time

                    print(f"The search method took: {duration:.2f} secs")
                    print('Cost of Optimal Path: ', goal_cost)
                    
                    # animate the path
                    informed_rrt.animate(explored_states, backtrack_states)
                    
                    print('\nPrinting Waypoints...\n')
                    # Pass the waypoints to the ROS node to move the robot in Gazebo
                    for index in range(1, len(backtrack_states), 5):
                        print(backtrack_states[index])
                        turtlebot(backtrack_states[index][0], backtrack_states[index][1])
                else:
                    print("The entered goal node is an obstacle ")
                    print("Please check README.md file for running turtlebot_rrt.py file.")
            else:
                print("The entered start node is an obstacle ")
                print("Please check README.md file for running turtlebot_rrt.py file.")
        else:
            print("The entered goal node outside the map ")
            print("Please check README.md file for running turtlebot_rrt.py file.")
    else:
        print("The entered start node is outside the map ")
        print("Please check README.md file for running turtlebot_rrt.py file.")