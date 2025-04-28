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
from datetime import datetime
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import shutil
import copy
# Import here the packages used in your codes
from gnc_robot_modules.gnc_module import *
from pathfinding_modules.hybrid_a_star import *
from pathfinding_modules.astar import *
import subprocess
import re
from pathlib import Path
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import actionlib


""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
Date:20.03.23
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
""" 

# Define the global varible: WAYPOINTS  Wpts=[[x_i,y_i]];
global WAYPOINTS
WAYPOINTS = [[2,2],[1,0.5]]


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
            "domains/ttk4192/domain/PDDL_domain_11.pddl\" "
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


#4) Program here the turtlebot actions (based in your AI planner)
"""
Turtlebot 3 actions-------------------------------------------------------------------------
"""

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/image"
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
            print("Picture taken")
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


# def move_robot_arm(joint_positions_rad, duration=5):
#     # Using /arm_controller/follow_trajectory/goal topic instead
#     client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
#     client.wait_for_server()

#     goal = FollowJointTrajectoryGoal()
#     goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

#     pt = JointTrajectoryPoint()
#     pt.positions = joint_positions_rad
#     pt.time_from_start = rospy.Duration(duration)
#     goal.trajectory.points.append(pt)

#     client.send_goal(goal)
#     client.wait_for_result()

#     # Maybe add a sleep here to allow time for the action to complete

#     rospy.loginfo("Arm command sent")

#     return client.get_result()

def move_robot_arm(joint_positions_rad, duration=5.0):
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Arm action‐server not available!")
        return None

    goal = FollowJointTrajectoryGoal()
    traj = goal.trajectory
    traj.joint_names = ['joint1','joint2','joint3','joint4']

    traj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

    # Get current joint positions, and tell robot to move from there to desired position
    js = rospy.wait_for_message('/joint_states', JointState, timeout=2.0)
    p0 = JointTrajectoryPoint()
    p0.positions = [ js.position[js.name.index(j)] for j in traj.joint_names ]
    p0.time_from_start = rospy.Duration(0.0)
    traj.points.append(p0)

    p1 = JointTrajectoryPoint()
    p1.positions = joint_positions_rad
    p1.time_from_start = rospy.Duration(duration)
    traj.points.append(p1)

    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo(f"Arm moved over {duration:.1f}s")
    return client.get_result()

def move_gripper(position, duration=2):
    client = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Gripper action‐server not available!")
        return None
    
    goal = FollowJointTrajectoryGoal()
    traj = goal.trajectory
    traj.joint_names = ['gripper']

    traj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

    js = rospy.wait_for_message('/joint_states', JointState, timeout=2.0)
    p0 = JointTrajectoryPoint()
    p0.positions = [ js.position[js.name.index('gripper')] ]
    p0.time_from_start = rospy.Duration(0.0)
    traj.points.append(p0)

    p1 = JointTrajectoryPoint()
    p1.positions = [position]
    p1.time_from_start = rospy.Duration(duration)
    traj.points.append(p1)

    client.send_goal(goal)
    client.wait_for_result()

    # Maybe add a sleep here to allow time for the action to complete

    rospy.loginfo("Gripper command sent")

    return client.get_result()

def move_robot_to_waypoint(end_pos_str):
    current_pos = get_current_odom()
    rospy.logwarn("Moving robot to waypoint " + end_pos_str +  " from current position " + str(current_pos))
    reverse = True
    add_extra_cost = False
    grid_on = False
    heu = 1

    x_wp = [0.3,  1.7, 3.48, 3.33, 4.9, 0.93 , 3.75]
    y_wp = [0.3, 0.3, 1.2, 2.4, 0.5, 2.4, 1.4]
    theta_wp = [0, 0, pi, pi, pi, -pi/4, 0]
    wp_0 = [x_wp[0],y_wp[0],theta_wp[0]]
    wp_1 = [x_wp[1],y_wp[1],theta_wp[1]]
    wp_2 = [x_wp[2], y_wp[2], theta_wp[2]]
    wp_3 = [x_wp[3],y_wp[3],theta_wp[3]]
    wp_4 = [x_wp[4],y_wp[4],theta_wp[4]]
    wp_5 = [x_wp[5],y_wp[5],theta_wp[5]]
    wp_6 = [x_wp[6],y_wp[6],theta_wp[6]]

    waypoints = [wp_0, wp_1, wp_2, wp_3, wp_4, wp_5, wp_6]
    end_pos = waypoints[int(end_pos_str[-1])]
    # WAYPOINTS = main_hybrid_a(heu,current_pos,end_pos,reverse,add_extra_cost,grid_on)
    WAYPOINTS = main_astar(False, current_pos, end_pos)
    print("Executing path following")
    theta_real = [0, pi/2, -pi/2, pi, pi, pi/2, pi/2]
    turtlebot_move(WAYPOINTS, theta_real[int(end_pos_str[-1])])

def move_robot_between_wp(start_pos_str, end_pos_str):
    rospy.logwarn("Moving robot between waypoints " + start_pos_str +  " and  " + end_pos_str)
    reverse = True
    add_extra_cost = False
    grid_on = False
    heu = 1

    x_wp = [0.3,  1.7, 3.48, 3.33, 4.9, 0.93 , 3.75]
    y_wp = [0.3, 0.3, 1.2, 2.4, 0.5, 2.4, 1.4]
    theta_real = [0, 0, pi, pi, pi, pi/2, 0]
    theta_wp = [0, 0, pi, pi, pi, pi/2, 0]
    wp_0 = [x_wp[0],y_wp[0],theta_wp[0]]
    wp_1 = [x_wp[1],y_wp[1],theta_wp[1]]
    wp_2 = [x_wp[2], y_wp[2], theta_wp[2]]
    wp_3 = [x_wp[3],y_wp[3],theta_wp[3]]
    wp_4 = [x_wp[4],y_wp[4],theta_wp[4]]
    wp_5 = [x_wp[5],y_wp[5],theta_wp[5]]
    wp_6 = [x_wp[6],y_wp[6],theta_wp[6]]

    waypoints = [wp_0, wp_1, wp_2, wp_3, wp_4, wp_5, wp_6]
    start_pos = waypoints[int(start_pos_str[-1])]
    end_pos = waypoints[int(end_pos_str[-1])]
    # print("Startpos inside moverobot:", start_pos)
    # print("Endpos inside moverobot:", end_pos)
    # WAYPOINTS = main_hybrid_a(heu,start_pos,end_pos,reverse,add_extra_cost,grid_on)
    WAYPOINTS = main_astar(False, start_pos, end_pos)
    print("Executing path following")
    theta_real = [0, pi/2, -pi/2, pi, pi, pi/2, pi/2]
    #print("End theta:", theta_real[int(end_pos_str[-1])])
    turtlebot_move(WAYPOINTS, theta_real[int(end_pos_str[-1])])


def pick_object():
    print("Picking object ...")
    move_robot_arm([-pi/2, pi/4, 0.1, 0.0], 3)
    rospy.sleep(3)
    move_gripper(-0.02, 2)
    rospy.sleep(2)
    move_robot_arm([0.0, -0.6, 0.6, 0.0], 3)
    rospy.sleep(3)
    move_gripper(0.02, 2)
    
def do_some_inspection():
    print("Doing some inspection")
    move_robot_arm([pi/4, 0.2, 0.2, 0.0], 3)
    rospy.sleep(3)
    print("Taking picture ...")
    taking_photo_exe()
    rospy.sleep(2)
    move_robot_arm([-pi/4, 0.2, 0.2, 0.0], 3)
    rospy.sleep(3)
    # print("Taking picture ...")
    # taking_photo_exe()
    # rospy.sleep(2)
    move_robot_arm([0.0, -0.6, 0.6, 0.0], 3)


def making_turn_exe():
    print("Executing Make a turn")
    rospy.sleep(1)
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
    rospy.sleep(5)

def get_current_odom():
    # Get the current odometry data
    odom = rospy.wait_for_message("/odom", Odometry, timeout=5)
    x_initial_offset = 0.23
    y_initial_offset = 0.18
    x = odom.pose.pose.position.x + x_initial_offset
    y = odom.pose.pose.position.y + y_initial_offset
    orientation = odom.pose.pose.orientation
    _, _, theta = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return [round(x, 2), round(y, 2), 0] # Need to be changed to theta

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
        print("AI planners: STP-planer")
        print("Path-finding: A-star")
        print("GNC Controller: PID path-following")
        print("Robot: Turtlebot3 waffle-pi")
        print("date: 27.04.25")
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        input_t=input("")

        # 5.0) Testing the GNC module (uncomment lines to test)
        #move_robot_arm([pi/4, 0.2, 0.2, 0.0], 5)
        # move_robot_arm([-pi/4, 0.2, 0.2, 0.0], 5)
        # move_robot_arm([0.0, 0.0, 0.0, 0.0], 5)
        # move_gripper(0.0, 2)
        # move_gripper(-0.02, 2)
        #move_robot_between_wp("6", "4")

        

        run_GNC_test = False

        if run_GNC_test:
            print("Testing GNC module")
            reverse = True
            add_extra_cost = False
            grid_on = True
            heu = 1

            x_wp = [0.3,  1.7, 3.48, 3.33, 4.9, 0.93 , 3.75]
            y_wp = [0.3, 0.3, 1.2, 2.4, 0.5, 2.4, 1.4]
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
            # WAYPOINTS = main_hybrid_a(heu,start_pos,end_pos,reverse,add_extra_cost,grid_on)
            WAYPOINTS = main_astar(False, start_pos, end_pos)
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
        start_times = [a["start_time"] for a in actions]
        durations = [a["duration"] for a in actions]
    
        # 5.3) Start mission execution 
        print("")
        print("Starting mission execution")
        action_number = 0
        move_robot_to_waypoint(arguments[0][1])
        for action in action_names:
            print("Executing action:", action)
            if action == "move_robot":
                action_arguments = arguments[action_number]
                start_pos = action_arguments[1]
                end_pos = action_arguments[2]
                move_robot_between_wp(start_pos, end_pos)
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

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")