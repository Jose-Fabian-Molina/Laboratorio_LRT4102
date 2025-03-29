#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, sin, cos

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('turtle_control_node')
        
        # Subscribe to the turtle's pose topic
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publisher for the turtle's velocity commands
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Set the publishing rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Initialize current position and orientation variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def pose_callback(self, pose):
        # Callback function executed whenever a new turtle pose is received
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_xy(self, desired_x, desired_y):
        # Proportional control constants (adjustable)
        Kp_linear = 1.0
        Kp_angular = 4.0
        
        while not rospy.is_shutdown():
            # Calculate the error in position
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            distance = sqrt(error_x**2 + error_y**2)
            
            # Calculate the desired angle and the angle error
            desired_angle = atan2(error_y, error_x)
            angle_error = desired_angle - self.current_theta
            # Normalize the angle error to the range [-pi, pi]
            angle_error = atan2(sin(angle_error), cos(angle_error))
            
            twist_msg = Twist()
            
            # If the angle error is significant, rotate first
            if abs(angle_error) > 0.1:
                twist_msg.angular.z = Kp_angular * angle_error
                twist_msg.linear.x = 0.0
            else:
                # Otherwise, move forward toward the target
                twist_msg.linear.x = Kp_linear * distance
                twist_msg.angular.z = 0.0
            
            # Publish the velocity command
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Current position: (%.2f, %.2f), Distance: %.2f, Angle error: %.2f", 
                          self.current_x, self.current_y, distance, angle_error)
            
            # Check if the turtle is close enough to the desired position
            if distance < 0.1:
                rospy.loginfo("Desired position reached")
                break
            
            self.rate.sleep()

    def get_desired_xy_from_user(self):
        # Ask the user to input the desired x and y coordinates
        desired_x = float(input("Enter desired x coordinate: "))
        desired_y = float(input("Enter desired y coordinate: "))
        return desired_x, desired_y

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Get the desired coordinates from the user
            desired_x, desired_y = self.get_desired_xy_from_user()
            # Move the turtle to the desired coordinates
            self.move_turtle_to_desired_xy(desired_x, desired_y)

if __name__ == '__main__':
    try:
        move_turtle = MoveTurtleProportionalControl()
        move_turtle.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass