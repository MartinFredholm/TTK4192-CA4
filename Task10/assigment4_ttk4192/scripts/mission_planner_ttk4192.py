#!/usr/bin/env python3
import rospy
import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, tan
from os import system, name
import time
import re
import fileinput
import sys
import argparse
import random
import matplotlib.animation as animation
from datetime import datetime
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point
from utils.dubins_path import DubinsPath
from utils.environment import Environment_robplan
from utils.grid import Grid_robplan
from utils.car import SimpleCar
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import shutil
import copy
# Import here the packages used in your codes
from gnc_robot_modules.gnc_module import *
import subprocess
import re
from pathlib import Path
from utils.reeds_shepp import RSPath
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point,distance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
Date:20.03.23
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
""" 

# 1) Program here your AI planner 
"""
AI planner ---------------------------------------------------------------------------
"""
def run_STP_planner(run=False):
    catkin_ws_src = str(Path.home() / "catkin_ws" / "src")
    venv_activate = "source temporal-planning-main/bin/activate"
    planner_dir = "temporal-planning-main/temporal-planning"
    planner_cmd = (
        "python2.7 bin/plan.py stp-2 "
        f"\"{catkin_ws_src}/temporal-planning-main/temporal-planning/"
            "domains/ttk4192/domain/PDDL_domain_1.pddl\" "
        f"\"{catkin_ws_src}/temporal-planning-main/temporal-planning/"
            "domains/ttk4192/problem/PDDL_problem_task11.pddl\""
    )

    command = f"""
    cd "{catkin_ws_src}" && \
    {venv_activate} && \
    cd {planner_dir} && \
    {planner_cmd}
    """
    if run:
        process = subprocess.run(command, shell=True, executable="/bin/bash", check=True)

    plan_file_path = os.path.join(catkin_ws_src, planner_dir, "tmp_sas_plan.1")

    with open(plan_file_path, "r") as plan_file:
        plan_output = plan_file.read()

    return plan_output




# 3) Program here your path-finding algorithm
""" Hybrid A-star pathfinding --------------------------------------------------------------------
"""

class HybridAstar:
    """ Hybrid A* search procedure. """
    def __init__(self, car, grid, reverse, unit_theta=pi/12, dt=5e-2):
        self.car = car
        self.grid = grid
        self.reverse = reverse
        self.unit_theta = unit_theta
        self.dt = dt

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)
        self.drive_steps = int(sqrt(2)*self.grid.cell_size/self.dt) + 1
        self.arc = self.drive_steps * self.dt
        self.phil = [-self.car.max_phi, 0, self.car.max_phi]
        self.ml = [1, -1]

        if reverse:
            self.comb = list(product(self.ml, self.phil))
        else:
            self.comb = list(product([1], self.phil))

        self.RSPath = RSPath(self.car)  
        self.dubins = DubinsPath(car,False)
        self.astar = Astar(self.grid, self.goal[:2])
        
        self.w1 = 0.95 # weight for astar heuristic
        self.w2 = 0.05 # weight for simple heuristic
        self.w3 = 1.0 # weight for extra cost of steering angle change
        self.w4 = 1.0 # weight for extra cost of turning
        self.w5 = 0 # weight for extra cost of reversing

        self.thetas = get_discretized_thetas(self.unit_theta)
    
    def construct_node(self, pos):
        """ Create node for a pos. """

        theta = pos[2]
        pt = pos[:2]

        theta = round_theta(theta % (2*pi), self.thetas)
        
        cell_id = self.grid.to_cell_id(pt)
        grid_pos = cell_id + [theta]

        node = Node(grid_pos, pos)

        return node
    
    def simple_heuristic(self, pos):
        """ Heuristic by Manhattan distance. """

        return abs(self.goal[0]-pos[0]) + abs(self.goal[1]-pos[1])
        
    def astar_heuristic(self, pos):
        """ Heuristic by standard astar. """

        h2 = self.simple_heuristic(pos[:2])
        try:
            h1 = self.astar.search_path(pos[:2]) * self.grid.cell_size
        except TypeError:
            h1 = h2
        return self.w1*h1 + self.w2*h2

    def get_children(self, node, heu, extra):
        """ Get successors from a state. """

        children = []
        for m, phi in self.comb:

            # don't go back
            # if node.m and node.phi == phi and node.m*m == -1:
            #     continue

            # if node.m and node.m == 1 and m == -1:
            #     continue

            pos = node.pos
            branch = [m, pos[:2]]

            for _ in range(self.drive_steps):
                pos = self.car.step(pos, phi, m,self.dt)
                branch.append(pos[:2])

            # check safety of route-----------------------
            pos1 = node.pos if m == 1 else pos
            pos2 = pos if m == 1 else node.pos
            if phi == 0:
                safe = self.RSPath.is_straight_route_safe(pos1, pos2)
            else:
                d, c, r = self.car.get_params(pos1, phi)
                safe = self.RSPath.is_turning_route_safe(pos1, pos2, d, c, r)
            # --------------------------------------------
            
            if not safe:
                continue
            
            child = self.construct_node(pos)
            child.phi = phi
            child.m = m
            child.parent = node
            child.g = node.g + self.arc
            child.g_ = node.g_ + self.arc

            if extra:
                # extra cost for changing steering angle
                if phi != node.phi:
                    child.g += self.w3 * self.arc
                
                # extra cost for turning
                if phi != 0:
                    child.g += self.w4 * self.arc
                
                # extra cost for reverse
                if m == -1:
                    child.g += self.w5 * self.arc

            if heu == 0:
                child.f = child.g + self.simple_heuristic(child.pos)
            if heu == 1:
                child.f = child.g + self.astar_heuristic(child.pos)
            
            children.append([child, branch])

        return children
    
    def best_final_shot(self, open_, closed_, best, cost, d_route, n=10):
        """ Search best final shot in open set. """

        open_.sort(key=lambda x: x.f, reverse=False)

        for t in range(min(n, len(open_))):
            best_ = open_[t]
            solutions_ = self.dubins.find_tangents(best_.pos,self.goal)
            d_route_, cost_, valid_ = self.dubins.best_tangent(solutions_)
            dist = distance(best.pos,self.goal)
            if valid_ == False and dist < 1.5:
                d_route_, cost_, valid_ = self.RSPath.get_best_path(best_.pos,self.goal)
        
            if valid_ and cost_ + best_.g_ < cost + best.g_:
                best = best_
                cost = cost_
                d_route = d_route_
        
        if best in open_:
            open_.remove(best)
            closed_.append(best)
        
        return best, cost, d_route
    
    def backtracking(self, node):
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.pos, node.phi, node.m))
            node = node.parent
        
        return list(reversed(route))
    
    def search_path(self, heu=1, extra=False):
        """ Hybrid A* pathfinding. """

        root = self.construct_node(self.start)
        root.g = float(0)
        root.g_ = float(0)
        
        if heu == 0:
            root.f = root.g + self.simple_heuristic(root.pos)
        if heu == 1:
            root.f = root.g + self.astar_heuristic(root.pos)

        closed_ = []
        open_ = [root]

        count = 0
        print(f"Count: {count}")
        print(f"Open nodes: {len(open_)}")
        print(f"Closed nodes: {len(closed_)}")
        while open_:
            count += 1
            if count % 1 == 0:
                sys.stdout.write("\033[3A")  # Move up 3 lines
                sys.stdout.write(f"Count: {count}\n")
                sys.stdout.write(f"Open nodes: {len(open_)}\n")
                sys.stdout.write(f"Closed nodes: {len(closed_)}\n")
                sys.stdout.flush()

            best = min(open_, key=lambda x: x.f)

            open_.remove(best)
            closed_.append(best)

            # check RS path
            solutions = self.dubins.find_tangents(best.pos,self.goal)
            d_route, cost, valid = self.dubins.best_tangent(solutions)
            dist = distance(best.pos,self.goal)
            if valid == False and dist < 1.5:
                d_route, cost, valid = self.RSPath.get_best_path(best.pos,self.goal)
                
            if valid:
                best, cost, d_route = self.best_final_shot(open_, closed_, best, cost, d_route)
                route = self.backtracking(best) + d_route
                path = self.car.get_path(self.start, route) 
                cost += best.g_
                print('Shortest path: {}'.format(round(cost, 2)))
                print('Total iteration:', count)
                
                return path, closed_

            children = self.get_children(best, heu, extra)

            for child, branch in children:

                if child in closed_:
                    continue

                if child not in open_:
                    best.branches.append(branch)
                    open_.append(child)

                elif child.g < open_[open_.index(child)].g:
                    best.branches.append(branch)

                    c = open_[open_.index(child)]
                    p = c.parent
                    for b in p.branches:
                        if same_point(b[-1], c.pos[:2]):
                            p.branches.remove(b)
                            break
                    
                    open_.remove(child)
                    open_.append(child)

        return None, None


def main_hybrid_a(heu,start_pos, end_pos,reverse, extra, grid_on):

    tc = map_grid_robplan()
    env = Environment_robplan(tc.obs)
    car = SimpleCar(env, start_pos, end_pos)
    grid = Grid_robplan(env)

    hastar = HybridAstar(car, grid, reverse)

    t = time.time()
    path, closed_ = hastar.search_path(heu, extra)
    print('Total time: {}s'.format(round(time.time()-t, 3)))

    if not path:
        print('No valid path!')
        return
    # a post-processing is required to have path list
    path = path[::5] + [path[-1]]
    #for i in range(len(path)):
    #    print(path[i].pos[0])
    
    branches = []
    bcolors = []
    for node in closed_:
        for b in node.branches:
            branches.append(b[1:])
            bcolors.append('y' if b[0] == 1 else 'b')

    xl, yl = [], []
    xl_np1,yl_np1=[],[]
    carl = []
    dt_s=int(5)  # samples for gazebo simulator
    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])
        if i==0 or i==len(path):
            xl_np1.append(path[i].pos[0])
            yl_np1.append(path[i].pos[1])            
        elif dt_s*i<len(path):
            xl_np1.append(path[i*dt_s].pos[0])
            yl_np1.append(path[i*dt_s].pos[1])
    xl_np1.append(path[-1].pos[0]) # Adding the last point to the path
    yl_np1.append(path[-1].pos[1]) # Adding the last point to the path

    # defining way-points (traslandado el origen a (0,0))
    xl_np=np.array(xl_np1)
    xl_np=xl_np#-0.2
    yl_np=np.array(yl_np1)
    yl_np=yl_np#-0.3
    global WAYPOINTS
    WAYPOINTS=np.column_stack([xl_np,yl_np])
    print(WAYPOINTS)

    # Removing redundant waypoints
    def prune_path(WP):
        tol = 0.1
        pts = WP.tolist()
        i = 1
        while i < len(pts) - 1:
            x1, y1 = pts[i-1]
            x2, y2 = pts[i]
            x3, y3 = pts[i+1]
            if (abs(x2-x1)<tol and abs(x3-x2) < tol) or (abs(y2-y1)<tol and abs(y3-y2) < tol):
                pts.pop(i)
            elif x2 == x3 and y2 == y3:
                pts.pop(i)
            else:
                i += 1
        return np.array(pts)
    
    WAYPOINTS = prune_path(WAYPOINTS)
    print("Waypoints after removing redundant ones:")
    print(WAYPOINTS)
    
    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    #plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")

    if grid_on:
        ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
        ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        plt.grid(which='both')
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=6)
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)

    # _branches = LineCollection(branches, color='b', alpha=0.8, linewidth=1)
    # ax.add_collection(_branches)

    # _carl = PatchCollection(carl[::20], color='m', alpha=0.1, zorder=3)
    # ax.add_collection(_carl)
    # ax.plot(xl, yl, color='whitesmoke', linewidth=2, zorder=3)
    # _car = PatchCollection(path[-1].model, match_original=True, zorder=4)
    # ax.add_collection(_car)

    _branches = LineCollection([], linewidth=1)
    ax.add_collection(_branches)

    _path, = ax.plot([], [], color='lime', linewidth=2)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='w', linewidth=2)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(branches) + len(path) + 1

    def init():
        _branches.set_paths([])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])

        return _branches, _path, _carl, _path1, _car

    def animate(i):

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']

        if i < len(branches):
            _branches.set_paths(branches[:i+1])
            _branches.set_color(bcolors)
        
        else:
            _branches.set_paths(branches)

            j = i - len(branches)

            _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])

            sub_carl = carl[:min(j+1, len(path))]
            _carl.set_paths(sub_carl[::4])
            _carl.set_edgecolor('k')
            _carl.set_facecolor('m')
            _carl.set_alpha(0.1)
            _carl.set_zorder(3)

            _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
            _path1.set_zorder(3)

            _car.set_paths(path[min(j, len(path)-1)].model)
            _car.set_edgecolor(edgecolor)
            _car.set_facecolor(facecolor)
            _car.set_zorder(3)

        return _branches, _path, _carl, _path1, _car

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                  interval=1, repeat=True, blit=True)

    plt.show()

class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):

        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.g_ = None
        self.f = None
        self.parent = None
        self.phi = 0
        self.m = None
        self.branches = []

    def __eq__(self, other):

        return self.grid_pos == other.grid_pos
    
    def __hash__(self):

        return hash((self.grid_pos))
    
class map_grid_robplan:
    """ Here the obstacles are defined for a 20x20 map. """
    def __init__(self):

        self.start_pos2 = [4, 4, 0]  # default values
        self.end_pos2 = [4, 8, -pi]  # default
        self.obs = [
            # walls
            [0,0, 3.3, 0.01],
            [3.3,0,0.01, 0.2],
            [3.3,0.2,5.21-3.3,0.01],
            [0,0,0.01,1],
            [0,1,0.5,0.01],
            [0.5,1,0.01,0.2],
            [0.5,1+0.2,-0.5,0],
            [0,1.2,0.01,2.55+0.2-1.2],
            [0,2.75,5.21,0.01],
            [5.21,2.75,0.01,-2.55],
            
            # boxes
            [1.3-0.2/2,1.85-0.4/2,0.2,0.4],
            [2.4-0.4/2,1.85-0.4/2,0.4,0.4],

            #valves
            [1.77-0.5/2,0.73-0.2/2,0.5,0.2],
            [3.48-0.5/2,.79-0.2/2,0.5,0.2],

            #pump
            [3.75-0.4/2,1.77-0.2/2,0.4,0.2],
            

        ]

#4) Program here the turtlebot actions (based in your AI planner)
"""
Turtlebot 3 actions-------------------------------------------------------------------------
"""

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

def taking_photo_exe():
    # Initialize
    camera = TakePhoto()

    # Default value is 'photo.jpg'
    # now = datetime.now()
    # dt_string = now.strftime("%d%m%Y_%H%M%S")
    # img_title = rospy.get_param('~image_title', 'photo'+dt_string+'.jpg')

    # if camera.take_picture(img_title):
    #     rospy.loginfo("Saved image " + img_title)
    # else:
    #     rospy.loginfo("No images received")

    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")
    img_name = f"photo{dt_string}.jpg"

    photos_dir = Path.home() / "catkin_ws" / "src" / "assigment4_ttk4192" / "scripts" / "photos"
    photos_dir.mkdir(parents=True, exist_ok=True)

    img_path = photos_dir / img_name

    if camera.take_picture(str(img_path)):
        rospy.loginfo(f"Saved image {img_path}")
    else:
        rospy.logwarn("No images received")
	#eog photo.jpg
    # Sleep to give the last log messages time to be sent

	# saving photo in a desired directory
    # file_source = Path.home() / "catkin_ws" / "src" / "assigment4_ttk4192" / "scripts"
    # file_destination = str(Path.home() / "catkin_ws/src/assigment4_ttk4192/scripts/photos")
    # g='photo'+dt_string+'.jpg'

    # shutil.move(str(file_source / g), file_destination)
    rospy.sleep(1)


def move_robot_arm(joint_positions_rad, duration=5):
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

    traj = JointTrajectory() # Constructs empty JointTrajectory message to be sent
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4'] # Adding the joint names to the message

    pt = JointTrajectoryPoint() # Constructs empty point message to be added to the trajectory message
    pt.positions = joint_positions_rad
    pt.time_from_start = rospy.Duration(duration)

    traj.points = [pt]

    rospy.sleep(0.5) 
    pub.publish(traj)
    rospy.loginfo("Arm command sent")
    rospy.sleep(duration)  # wait for the arm to finish moving

def move_gripper(position, duration=2):
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)

    traj = JointTrajectory() # Constructs empty JointTrajectory message to be sent
    traj.joint_names = ['gripper'] # Adding the joint names to the message
    
    pt = JointTrajectoryPoint() # Constructs empty point message to be added to the trajectory message
    pt.positions = [position]
    pt.time_from_start = rospy.Duration(duration)

    traj.points = [pt]

    rospy.sleep(0.5)
    pub.publish(traj)
    rospy.loginfo("Gripper command sent")
    rospy.sleep(duration)  # wait for the gripper to finish moving


def move_robot(start_pos, end_pos):
    print("Moving robot between ", start_pos, "and ", end_pos)
    reverse = True
    add_extra_cost = False
    grid_on = False
    heu = 1

    x_wp = [0.3,  1.7, 3.48, 3.33, 5, 0.93 , 3.75]
    y_wp = [0.3, 0.3, 1.2, 2.4, 0.45, 2.4, 1.4]
    theta_wp = [0, 0, pi, pi, pi, -pi/4, 0]
    wp_0 = [x_wp[0],y_wp[0],theta_wp[0]]
    wp_1 = [x_wp[1],y_wp[1],theta_wp[1]]
    wp_2 = [x_wp[2], y_wp[2], theta_wp[2]]
    wp_3 = [x_wp[3],y_wp[3],theta_wp[3]]
    wp_4 = [x_wp[4],y_wp[4],theta_wp[4]]
    wp_5 = [x_wp[5],y_wp[5],theta_wp[5]]
    wp_6 = [x_wp[6],y_wp[6],theta_wp[6]]

    waypoints = [wp_0, wp_1, wp_2, wp_3, wp_4, wp_5, wp_6]
    start_pos = waypoints[int(start_pos[-1])]
    end_pos = waypoints[int(end_pos[-1])]
    print("Startpos inside moverobot:", start_pos)
    print("Endpos inside moverobot:", end_pos)
    main_hybrid_a(heu,start_pos,end_pos,reverse,add_extra_cost,grid_on)
    print("Executing path following")
    turtlebot_move(WAYPOINTS)


def pick_object():
    print("Picking object")
    move_robot_arm([-pi/2, pi/4, 0.1, 0.0], 5)
    move_gripper(-0.02, 2)
    move_robot_arm([0.0, -pi/2, 0.0, 0.0], 5)
    #move_gripper(0.02, 2)
    
def do_some_inspection():
    print("Doing some inspection")
    move_robot_arm([pi/4, 0.2, 0.2, 0.0], 5)
    print("Taking picture ...")
    taking_photo_exe()
    time.sleep(5)
    move_robot_arm([-pi/4, 0.2, 0.2, 0.0], 5)
    print("Taking picture ...")
    taking_photo_exe()
    time.sleep(5)
    move_robot_arm([0.0, -pi/2, 0.0, 0.0], 5)


def making_turn_exe():
    print("Executing Make a turn")
    time.sleep(1)
    #Starts a new node
    #rospy.init_node('turtlebot_move', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    #speed = input("Input your speed (degrees/sec):")
    #angle = input("Type your distance (degrees):")
    #clockwise = input("Clockwise?: ") #True or false

    speed = 5
    angle = 180
    clockwise = True

    #Converting from angles to radians
    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0   #should be from the odometer

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin()

def check_pump_picture_ir_waypoint0():
    print("Manipulating the pump ...")
    do_some_inspection()
    pick_object()
    
def check_seals_valve_picture_eo_waypoint0():
    print("Checking seals valve ...")
    do_some_inspection()
    
def charge_battery_waypoint0():
    print("chargin battert")
    time.sleep(5)


# Define the global varible: WAYPOINTS  Wpts=[[x_i,y_i]];
global WAYPOINTS
WAYPOINTS = [[2,2],[1,0.5]]


# AI parser
def parse_stp_plan(plan_text):
    actions = []

    for line in plan_text.strip().split("\n"):
        match = re.match(r"([\d.]+):\s+\(\s*(\S+)\s+([^)]+)\s*\)\s+\[([\d.]+)\]", line)  # This was made by AI
        if match:
            start_time = float(match.group(1))
            action = match.group(2)
            args = match.group(3).split()
            duration = float(match.group(4))

            actions.append({
                "start_time": start_time,
                "action": action,
                "args": args,
                "duration": duration
            })

    return actions


# 5) Program here the main commands of your mission planner code
""" Main code ---------------------------------------------------------------------------
"""
if __name__ == '__main__':
    try:
        rospy.init_node('mission_planner', anonymous=True)
        rospy.loginfo("Starting mission planner")
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass
    try:
        print()
        print("************ TTK4192 - Assigment 4 **************************")
        print()
        print("AI planners: GraphPlan")
        print("Path-finding: Hybrid A-star")
        print("GNC Controller: PID path-following")
        print("Robot: Turtlebot3 waffle-pi")
        print("date: 20.03.23")
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        input_t=input("")

        # 5.0) Testing the GNC module (uncomment lines to test)
        # move_robot_arm([pi/4, 0.2, 0.2, 0.0], 5)
        # move_robot_arm([-pi/4, 0.2, 0.2, 0.0], 5)
        # move_robot_arm([0.0, 0.0, 0.0, 0.0], 5)
        # move_gripper(0.0, 2)
        # move_gripper(-0.02, 2)
        # move_robot_arm([-pi/2, -pi/2, 0.0, 0.0], 5)

        run_GNC_test = False

        if run_GNC_test:
            print("Testing GNC module")
            reverse = True
            add_extra_cost = False
            grid_on = True
            heu = 1

            x_wp = [0.3,  1.7, 3.48, 3.33, 5, 0.93 , 3.75]
            y_wp = [0.3, 0.3, 1.2, 2.4, 0.45, 2.4, 1.4]
            theta_wp = [0, 0, pi, pi, pi, -pi/4, 0]
            wp_0 = [x_wp[0],y_wp[0],theta_wp[0]]
            wp_1 = [x_wp[1],y_wp[1],theta_wp[1]]
            wp_2 = [x_wp[2], y_wp[2], theta_wp[2]]
            wp_3 = [x_wp[3],y_wp[3],theta_wp[3]]
            wp_4 = [x_wp[4],y_wp[4],theta_wp[4]]
            wp_5 = [x_wp[5],y_wp[5],theta_wp[5]]
            wp_6 = [x_wp[6],y_wp[6],theta_wp[6]]
            start_pos = wp_2
            end_pos = wp_0
            main_hybrid_a(heu,start_pos,end_pos,reverse,add_extra_cost,grid_on)
            print("Executing path following")
            turtlebot_move(WAYPOINTS)

		# 5.1) Starting the AI Planner
        plan_output = run_STP_planner(False)
        print("STP Plan Output:\n")
        print(plan_output)
    
        # 5.2) Reading the plan 
        print("  ")
        print("Reading the plan from AI planner")
        print("  ")
        actions = parse_stp_plan(plan_output)
        action_names =  [a["action"] for a in actions]
        arguments = [a["args"] for a in actions]
        print("Arguments:", arguments)
        start_times = [a["start_time"] for a in actions]
        durations = [a["duration"] for a in actions]
    
        # 5.3) Start mission execution 
        print("")
        print("Starting mission execution")
        action_number = 0

        for action in action_names:
            print("Executing action:", action)
            if action == "move_robot":
                action_arguments = arguments[action_number]
                start_pos = action_arguments[1]
                print("Startpos:", start_pos)
                end_pos = action_arguments[2]
                print("Endpos:", end_pos)
                move_robot(start_pos, end_pos)
            elif action == "take_photo_of_pump":
                check_pump_picture_ir_waypoint0()
            elif action == "check_seals_valve_picture_eo":
                check_seals_valve_picture_eo_waypoint0()
            elif action == "charge_battery":
                charge_battery_waypoint0()
            else:
                print("Unknown action:", action)

            action_number += 1

        print("")
        print("--------------------------------------")
        print("All tasks were performed successfully")
        time.sleep(10)  

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")