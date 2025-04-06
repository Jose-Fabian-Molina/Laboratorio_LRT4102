#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, SpawnRequest
import math

class TurtleLab3SingleTurtle:
    def __init__(self):
        rospy.init_node('turtle_lab3_single_turtle_controller')
        
        # Fixed starting position for the controlled turtle
        self.start_x = 5.5
        self.start_y = 5.5
        self.start_theta = 0.0
        
        # Desired goal parameters (user defined)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        
        # Controller gain for proportional control
        self.Kp = 1.0
        
        # Thresholds for position and orientation errors
        self.position_threshold = 0.1
        self.angle_threshold = 0.05  # in radians
        
        # Set the loop rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Current pose of the controlled turtle (goal_turtle)
        self.current_pose = None
        
        # Flag to indicate if the turtle has been spawned
        self.spawned = False
        
        # Create a publisher for the turtle's velocity commands
        self.velocity_publisher = rospy.Publisher('/goal_turtle/cmd_vel', Twist, queue_size=10)
        
        # Wait for the spawn service to be available and create a proxy
        rospy.wait_for_service('spawn')
        self.spawn_service = rospy.ServiceProxy('spawn', Spawn)
        
        # (Teleport service proxy will be created later after the turtle is spawned)

    def pose_callback(self, pose):
        # Update the current pose of the controlled turtle
        self.current_pose = pose

    def spawn_or_reset_turtle(self):
        """
        If the turtle is not spawned yet, spawn it at the fixed starting position.
        Otherwise, teleport the existing turtle to the starting position.
        """
        if not self.spawned:
            req = SpawnRequest()
            req.x = self.start_x
            req.y = self.start_y
            req.theta = self.start_theta
            req.name = "goal_turtle"
            try:
                response = self.spawn_service(req.x, req.y, req.theta, req.name)
                rospy.loginfo("Spawned turtle '%s' at starting position (%.2f, %.2f)", response.name, req.x, req.y)
                self.spawned = True
                # Subscribe to the spawned turtle's pose topic
                rospy.Subscriber('/goal_turtle/pose', Pose, self.pose_callback)
            except rospy.ServiceException as e:
                rospy.logerr("Spawn service call failed: %s", e)
        else:
            # Turtle already spawned; use teleport service to reset its pose
            rospy.wait_for_service('/goal_turtle/teleport_absolute')
            try:
                teleport = rospy.ServiceProxy('/goal_turtle/teleport_absolute', TeleportAbsolute)
                teleport(self.start_x, self.start_y, self.start_theta)
                rospy.loginfo("Teleported 'goal_turtle' to starting position (%.2f, %.2f)", self.start_x, self.start_y)
            except rospy.ServiceException as e:
                rospy.logerr("Teleport service call failed: %s", e)

    def get_goal_from_user(self):
        # Prompt the user for desired goal coordinates and orientation (radians)
        self.goal_x = float(input("Enter desired x coordinate: "))
        self.goal_y = float(input("Enter desired y coordinate: "))
        self.goal_theta = float(input("Enter desired orientation (radians): "))

    def calculate_dtg_atg(self):
        if self.current_pose is None:
            rospy.loginfo("Current pose not available yet.")
            return None, None

        dtg = math.sqrt((self.goal_x - self.current_pose.x)**2 + (self.goal_y - self.current_pose.y)**2)
        atg = math.atan2(self.goal_y - self.current_pose.y, self.goal_x - self.current_pose.x)
        rospy.loginfo("DTG: %.2f, ATG: %.2f", dtg, atg)
        return dtg, atg

    def move_to_goal(self):
        """
        Use a proportional controller to move the turtle from the starting position to the user-defined goal.
        """
        while not rospy.is_shutdown():
            if self.current_pose is None:
                self.rate.sleep()
                continue

            dtg, atg = self.calculate_dtg_atg()
            if dtg is None:
                self.rate.sleep()
                continue

            twist_msg = Twist()
            if dtg > self.position_threshold:
                # Control for moving to the goal position
                angle_error = atg - self.current_pose.theta
                # Normalize angle error to [-pi, pi]
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                twist_msg.linear.x = self.Kp * dtg
                twist_msg.angular.z = self.Kp * angle_error
                rospy.loginfo("Moving: DTG=%.2f, ATG=%.2f", dtg, atg)
            else:
                # Position reached; now adjust orientation to desired goal_theta
                orientation_error = self.goal_theta - self.current_pose.theta
                orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.Kp * orientation_error
                rospy.loginfo("Adjusting orientation: Orientation error=%.2f", orientation_error)
                if abs(orientation_error) < self.angle_threshold:
                    rospy.loginfo("Goal position and desired orientation reached!")
                    break

            self.velocity_publisher.publish(twist_msg)
            self.rate.sleep()

    def run(self):
        # Infinite loop: for each iteration, get new goal parameters and drive the turtle accordingly
        while not rospy.is_shutdown():
            rospy.loginfo("----- New Goal Input -----")
            self.get_goal_from_user()
            # Reset the turtle to the fixed starting position
            self.spawn_or_reset_turtle()
            rospy.sleep(2)  # Wait for the turtle's pose to update
            # Calculate and display DTG and ATG (for informational purposes)
            self.calculate_dtg_atg()
            # Use a proportional controller to drive the turtle from the starting position to the goal
            self.move_to_goal()
            rospy.loginfo("Goal reached. Waiting for new goal input...\n")
            rospy.sleep(2)

if __name__ == '__main__':
    try:
        controller = TurtleLab3SingleTurtle()
        controller.run()
    except rospy.ROSInterruptException:
        pass