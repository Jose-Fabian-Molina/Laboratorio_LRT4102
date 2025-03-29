#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, sin, cos

class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('turtle_pid_control_node')
        
        # Subscribe to the turtle's pose topic
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publisher for the turtle's velocity commands
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Set the publishing rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Initialize current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Initialize PID variables for linear (distance) control
        self.last_error_distance = 0.0
        self.error_accum_distance = 0.0
        
        # Initialize PID variables for angular (angle) control
        self.last_error_angle = 0.0
        self.error_accum_angle = 0.0

    def pose_callback(self, pose):
        # Callback function executed each time a new turtle pose is received
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_xy(self, desired_x, desired_y):
        # PID constants for linear (distance) control (adjustable)
        Kp_linear = 1.0
        Ki_linear = 0.01
        Kd_linear = 0.1
        
        # PID constants for angular (angle) control (adjustable)
        Kp_angular = 4.0
        Ki_angular = 0.01
        Kd_angular = 0.1
        
        while not rospy.is_shutdown():
            # Calculate the position error
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            distance = sqrt(error_x**2 + error_y**2)
            
            # Calculate the desired angle and the angular error
            desired_angle = atan2(error_y, error_x)
            error_angle = desired_angle - self.current_theta
            # Normalize the angle error to the range [-pi, pi]
            error_angle = atan2(sin(error_angle), cos(error_angle))
            
            # PID control for linear velocity
            d_error_distance = distance - self.last_error_distance
            self.error_accum_distance += distance
            linear_velocity = Kp_linear * distance + Ki_linear * self.error_accum_distance + Kd_linear * d_error_distance
            self.last_error_distance = distance
            
            # PID control for angular velocity
            d_error_angle = error_angle - self.last_error_angle
            self.error_accum_angle += error_angle
            angular_velocity = Kp_angular * error_angle + Ki_angular * self.error_accum_angle + Kd_angular * d_error_angle
            self.last_error_angle = error_angle
            
            # Create a Twist message with computed linear and angular velocities
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            
            # Publish the velocity command
            self.velocity_publisher.publish(twist_msg)
            
            rospy.loginfo("Current position: (%.2f, %.2f), Distance: %.2f, Angle error: %.2f", 
                          self.current_x, self.current_y, distance, error_angle)
            
            # Check if the desired position is reached
            if distance < 0.1:
                rospy.loginfo("Desired position reached")
                break
            
            self.rate.sleep()

    def get_desired_xy_from_user(self):
        # Request the desired x and y coordinates from the user
        desired_x = float(input("Enter desired x coordinate: "))
        desired_y = float(input("Enter desired y coordinate: "))
        return desired_x, desired_y

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Get the desired coordinates from the user
            desired_x, desired_y = self.get_desired_xy_from_user()
            # Move the turtle to the desired coordinates using PID control
            self.move_turtle_to_desired_xy(desired_x, desired_y)

if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass