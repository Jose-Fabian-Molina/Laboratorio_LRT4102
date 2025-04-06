#!/usr/bin/env python3
import rospy
from turtlesim.srv import Spawn, SpawnRequest, TeleportAbsolute
from turtlesim.msg import Pose
import math

class TurtleTeleportController:
    def __init__(self):
        rospy.init_node('turtle_teleport_controller')
        
        # Wait for the spawn service and create a service proxy
        rospy.wait_for_service('spawn')
        self.spawn_service = rospy.ServiceProxy('spawn', Spawn)
        
        # Use a fixed turtle name for the single turtle that will be reused
        self.turtle_name = "goal_turtle"
        
        # Spawn the turtle only once at an initial default position
        self.spawn_turtle_once()
        
        # Wait for the teleport service for the spawned turtle and create a proxy
        rospy.wait_for_service(f'/{self.turtle_name}/teleport_absolute')
        self.teleport_service = rospy.ServiceProxy(f'/{self.turtle_name}/teleport_absolute', TeleportAbsolute)
        
        # Subscribe to the turtle's pose topic to get its current pose
        rospy.Subscriber(f'/{self.turtle_name}/pose', Pose, self.pose_callback)
        
        self.current_pose = None
        self.rate = rospy.Rate(10)

    def spawn_turtle_once(self):
        req = SpawnRequest()
        # Spawn at a default position (this initial position is arbitrary)
        req.x = 5.5
        req.y = 5.5
        req.theta = 0.0
        req.name = self.turtle_name
        try:
            response = self.spawn_service(req.x, req.y, req.theta, req.name)
            rospy.loginfo("Spawned turtle '%s' at (%.2f, %.2f) with orientation %.2f", 
                          response.name, req.x, req.y, req.theta)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service call failed: %s", e)

    def pose_callback(self, pose):
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
         - DTG: Euclidean distance between current pose and goal.
         - ATG: Angle to goal using atan2.
         
         Velocity mapping explanation (if movement were used):
           - Linear velocity ∝ DTG (moves faster when farther away)
           - Angular velocity ∝ (ATG - current orientation) (to correct heading)
        """
        if self.current_pose is None:
            rospy.loginfo("Current pose not available yet.")
            return None, None

        dtg = math.sqrt((goal_x - self.current_pose.x)**2 + (goal_y - self.current_pose.y)**2)
        atg = math.atan2(goal_y - self.current_pose.y, goal_x - self.current_pose.x)
        rospy.loginfo("DTG: %.2f, ATG: %.2f", dtg, atg)
        return dtg, atg

    def run(self):
        # Infinite loop: get new goal parameters from the user and teleport the turtle there.
        while not rospy.is_shutdown():
            goal_x, goal_y, goal_theta = self.get_goal_from_user()
            # Calculate and display DTG and ATG based on the current pose of the turtle
            self.calculate_dtg_atg(goal_x, goal_y)
            # Directly teleport the turtle to the user-defined goal position and orientation
            try:
                self.teleport_service(goal_x, goal_y, goal_theta)
                rospy.loginfo("Teleported turtle to (%.2f, %.2f) with orientation %.2f",
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