#!/usr/bin/env python3
import rospy
from turtlesim.srv import Spawn, SpawnRequest
from turtlesim.msg import Pose
import math

class TurtleLab3Node:
    def __init__(self):
        rospy.init_node('turtle_lab3_node')
        
        # Subscriber to get the current pose of turtle1 (the default turtle)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Service proxy for spawning a new turtle
        rospy.wait_for_service('spawn')
        self.spawn_service = rospy.ServiceProxy('spawn', Spawn)
        
        # Current pose of turtle1 (used for calculating DTG and ATG)
        self.current_pose = None
        
        # Controller parameters and thresholds (these remain fixed)
        self.Kp = 1.0
        
        # Loop rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Turtle counter for unique naming of spawned turtles
        self.turtle_count = 1

    def pose_callback(self, pose):
        # Update the current pose from turtle1
        self.current_pose = pose

    def get_user_goal(self):
        # Prompt the user for the desired goal coordinates and orientation
        x = float(input("Enter desired x coordinate: "))
        y = float(input("Enter desired y coordinate: "))
        theta = float(input("Enter desired orientation (radians): "))
        return x, y, theta

    def calculate_and_display_metrics(self, goal_x, goal_y):
        if self.current_pose is None:
            rospy.loginfo("Current pose not available yet.")
            return None, None

        dtg = math.sqrt((goal_x - self.current_pose.x)**2 + (goal_y - self.current_pose.y)**2)
        atg = math.atan2(goal_y - self.current_pose.y, goal_x - self.current_pose.x)
        rospy.loginfo("Calculated DTG: %.2f, ATG: %.2f", dtg, atg)
        return dtg, atg

    def spawn_turtle_at_goal(self, goal_x, goal_y, goal_theta):
        """
        Spawn a new turtle directly at the user-defined goal position with the specified orientation.
        """
        # Create a unique name for the spawned turtle
        turtle_name = f"goal_turtle_{self.turtle_count}"
        self.turtle_count += 1

        req = SpawnRequest()
        req.x = goal_x
        req.y = goal_y
        req.theta = goal_theta
        req.name = turtle_name

        try:
            response = self.spawn_service(req.x, req.y, req.theta, req.name)
            rospy.loginfo("Spawned turtle '%s' at (%.2f, %.2f) with orientation %.2f",
                          response.name, req.x, req.y, req.theta)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service call failed: %s", e)

    def run(self):
        # Infinite loop for continuous operation
        while not rospy.is_shutdown():
            # Get the desired goal parameters from the user (x, y, theta)
            goal_x, goal_y, goal_theta = self.get_user_goal()
            
            # Calculate and display DTG and ATG based on the current pose of turtle1
            dtg, atg = self.calculate_and_display_metrics(goal_x, goal_y)
            if dtg is None:
                rospy.sleep(1)
                continue
            
            # Spawn a new turtle directly at the goal position with the desired orientation.
            self.spawn_turtle_at_goal(goal_x, goal_y, goal_theta)
            
            # Wait a moment before next iteration
            rospy.loginfo("Waiting for new goal input...\n")
            rospy.sleep(2)

if __name__ == '__main__':
    try:
        node = TurtleLab3Node()
        node.run()
    except rospy.ROSInterruptException:
        pass