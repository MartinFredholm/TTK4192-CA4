from math import sqrt, atan2, pi, hypot
import numpy as np
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

global simulation
simulation = True

# GNC module (path-followig and PID controller for the robot)
"""  Robot GNC module ----------------------------------------------------------------------
"""
class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        PI = 3.1415926535897
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class turtlebot_move():
    """
    Path-following module
    """
    def __init__(self, WAYPOINTS, end_theta=0):
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # reading vehicle speed
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.trajectory = list()

        self.yaw = list()

        self.thetas = list()

        # track a sequence of waypoints
        # Wait for the first odometry message to get the initial position
        rospy.wait_for_message("odom", Odometry, timeout=2.0)

        for point in WAYPOINTS:
            dist = hypot(point[0] - self.x, point[1] - self.y)
            if dist < 0.05:  #5 cm tolerance
                continue
            else:
                self.move_to_point(point[0], point[1])
        self.stop()
        # turn to the end angle
        self.turn_to_angle(end_theta)
        rospy.logwarn("Action done.")

        # plot trajectory
        data = np.array(self.trajectory)
        np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:,0],data[:,1])
        plt.title("Trajectory")
        plt.xlabel("X in m")
        plt.ylabel("Y in m")
        plt.show()

        # Plot of heading angle/yaw
        # data = np.array(self.yaw)
        # np.savetxt('yaw.csv', data, fmt='%f', delimiter=',')
        # plt.plot(data)
        # plt.title("Yaw")
        # plt.xlabel("Time")
        # plt.ylabel("Yaw in rad")
        # for theta in self.thetas:
        #     plt.axhline(y=theta, color='r', linestyle='--')
        # self.thetas = list()
        # plt.legend(['Yaw', 'Target Yaws'])
        # # Add pi/2 and -pi/2 lines
        # # plt.axhline(y=pi/2, color='r', linestyle='--')
        # # plt.axhline(y=-pi/2, color='r', linestyle='--')

        # plt.show()


    def move_to_point(self, x, y):
        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = atan2(diff_y, diff_x)

        self.pid_theta.setPID(1, 0, 0) # Using P control while steering
        self.pid_theta.setPoint(theta)
        self.thetas.append(theta)
        rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")

        
        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.pid_theta.setPoint(theta)
        self.thetas.append(theta)
        self.pid_theta.setPID(5, 0.02, 0.5)  # PID control while moving

        # Move to the target point
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) # projection
            if abs(linear) > 0.2:
                linear = linear/abs(linear)*0.2

            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2

            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            self.vel.linear.x = 1.5*linear   # Here can adjust speed
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()

    # Turn the robot to a specific angle
    def turn_to_angle(self, angle):
        print("### PID: set target theta = " + str(angle) + " ###")
        self.pid_theta.setPoint(angle)
        self.thetas.append(angle)
        self.pid_theta.setPID(1, 0, 0) # P control while steering
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()



    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        if not simulation:
            x_initial_offset = 0.23
            y_initial_offset = 0.18
        else:
            x_initial_offset = 0.0
            y_initial_offset = 0.0
        self.x = msg.pose.pose.position.x + x_initial_offset
        self.y = msg.pose.pose.position.y + y_initial_offset

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            self.yaw.append(yaw)
