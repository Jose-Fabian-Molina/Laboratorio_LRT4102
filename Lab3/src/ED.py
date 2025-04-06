#!/usr/bin/env python3
import rospy
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
import math

class TurtleTeleportController:
    def __init__(self):
        rospy.init_node('turtle_teleport_controller')
        
        # Wait for the teleport service for turtle1 to be available
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleport_service = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        
        # Subscribe to turtle1's pose topic to get its current pose
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        self.current_pose = None
        self.rate = rospy.Rate(10)

    def pose_callback(self, pose):
        # Update the current pose of turtle1
        self.current_pose = pose

    def get_goal_from_user(self):
        # Prompt the user for desired goal coordinates and orientation (in radians)
        x = float(input("Enter desired x coordinate: "))
        y = float(input("Enter desired y coordinate: "))
        theta = float(input("Enter desired orientation (radians): "))
        return x, y, theta

    def calculate_dtg_atg(self, goal_x, goal_y):
        """
        Calculate and display:
         - DTG: Euclidean distance between the current pose and the goal.
         - ATG: Angle to the goal using atan2.
         
         Velocity mapping explanation (if movement were applied):
           - Linear velocity ∝ DTG (turtle moves faster when farther away)
           - Angular velocity ∝ (ATG - current orientation) (to correct the heading)
        """
        if self.current_pose is None:
            rospy.loginfo("Current pose not available yet.")
            return None, None
        dtg = math.sqrt((goal_x - self.current_pose.x)**2 + (goal_y - self.current_pose.y)**2)
        atg = math.atan2(goal_y - self.current_pose.y, goal_x - self.current_pose.x)
        rospy.loginfo("DTG: %.2f, ATG: %.2f", dtg, atg)
        return dtg, atg

    def run(self):
        # Infinite loop: get new goal parameters from the user and teleport turtle1 accordingly
        while not rospy.is_shutdown():
            goal_x, goal_y, goal_theta = self.get_goal_from_user()
            # Calculate and display DTG and ATG based on turtle1's current pose
            self.calculate_dtg_atg(goal_x, goal_y)
            # Directly teleport turtle1 to the user-defined goal pose (no gradual movement)
            try:
                self.teleport_service(goal_x, goal_y, goal_theta)
                rospy.loginfo("Teleported turtle1 to (%.2f, %.2f) with orientation %.2f",
                              goal_x, goal_y, goal_theta)
            except rospy.ServiceException as e:
                rospy.logerr("Teleport service call failed: %s", e)
            rospy.sleep(2)

if __name__ == '__main__':
    try:
        controller = TurtleTeleportController()
        controller.run()
    except rospy.ROSInterruptException:
        pass