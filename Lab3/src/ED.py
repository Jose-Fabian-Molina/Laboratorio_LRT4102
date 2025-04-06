#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SpawnRequest
import math

class TurtleControllerLab3Infinite:
    def __init__(self):
        rospy.init_node('turtle_lab3_infinite_controller')
        
        # Subscribe to the turtle's pose topic to get current position and orientation
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publisher for sending velocity commands
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Wait for the spawn service and create a service proxy for spawning turtles
        rospy.wait_for_service('spawn')
        self.spawn_service = rospy.ServiceProxy('spawn', Spawn)
        
        # Current pose variables
        self.current_pose = None
        
        # Goal parameters (to be set by the user)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        
        # Controller gains (for position and orientation control)
        self.Kp_pos = 1.0
        self.Kp_ang = 1.0
        
        # Thresholds for determining when the goal is reached
        self.position_threshold = 0.1
        self.angle_threshold = 0.05  # in radians
        
        # Loop rate (10 Hz)
        self.rate = rospy.Rate(10)

    def pose_callback(self, pose):
        # Update the current pose of the turtle
        self.current_pose = pose

    def get_user_parameters(self):
        # Prompt the user for goal coordinates, desired orientation, and controller gains
        self.goal_x = float(input("Enter desired x coordinate: "))
        self.goal_y = float(input("Enter desired y coordinate: "))
        self.goal_theta = float(input("Enter desired goal orientation (radians): "))
        self.Kp_pos = float(input("Enter position proportional gain (Kp_pos): "))
        self.Kp_ang = float(input("Enter angular proportional gain (Kp_ang): "))

    def spawn_turtle_at_goal(self):
        """
        Spawn a new turtle at the goal position with an initial orientation of 0.
        The spawned turtle is named "goal_turtle".
        """
        req = SpawnRequest()
        req.x = self.goal_x
        req.y = self.goal_y
        req.theta = 0.0  # Spawn with 0 orientation so controller can adjust to desired angle
        req.name = "goal_turtle"
        try:
            response = self.spawn_service(req.x, req.y, req.theta, req.name)
            rospy.loginfo("Spawned turtle '%s' at (%.2f, %.2f) with initial orientation 0", 
                          response.name, req.x, req.y)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service call failed: %s", e)

    def move_turtle_to_goal_and_angle(self):
        """
        Use a proportional controller to move the turtle to the goal position and then adjust its orientation.
        
        The control strategy is as follows:
          - If the position error (DTG) is above the threshold, control both linear and angular velocities.
            Linear velocity is proportional to the Euclidean distance, and angular velocity is proportional to the
            error between the current heading and the direction towards the goal.
          - Once the turtle is at the desired position (DTG < threshold), control the orientation.
            Angular velocity is then proportional to the difference between the current orientation and the desired angle.
        This loop runs until both the position and orientation errors are below their respective thresholds.
        """
        while not rospy.is_shutdown():
            if self.current_pose is None:
                self.rate.sleep()
                continue

            # Calculate position error (Distance To Goal, DTG)
            error_x = self.goal_x - self.current_pose.x
            error_y = self.goal_y - self.current_pose.y
            dtg = math.sqrt(error_x**2 + error_y**2)
            
            twist_msg = Twist()
            
            if dtg > self.position_threshold:
                # Control for position
                atg = math.atan2(error_y, error_x)
                angle_error = atg - self.current_pose.theta
                # Normalize the angular error to [-pi, pi]
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                
                twist_msg.linear.x = self.Kp_pos * dtg
                twist_msg.angular.z = self.Kp_ang * angle_error
                rospy.loginfo("Moving: DTG=%.2f, ATG=%.2f", dtg, atg)
            else:
                # Position reached; now control orientation
                orientation_error = self.goal_theta - self.current_pose.theta
                orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))
                
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.Kp_ang * orientation_error
                rospy.loginfo("Adjusting Orientation: Orientation error=%.2f", orientation_error)
                
                # Check if desired orientation is reached
                if abs(orientation_error) < self.angle_threshold:
                    rospy.loginfo("Desired position and orientation reached!")
                    # Exit the loop once both position and orientation errors are within thresholds
                    break

            self.velocity_publisher.publish(twist_msg)
            self.rate.sleep()

    def run(self):
        # Main loop: continuously get new parameters and move the turtle
        while not rospy.is_shutdown():
            rospy.loginfo("----- New Goal -----")
            self.get_user_parameters()
            # Spawn a new turtle at the goal position
            self.spawn_turtle_at_goal()
            # Wait a bit for the spawned turtle's pose to update
            rospy.sleep(2)
            # Control loop to move the turtle to the desired position and orientation
            self.move_turtle_to_goal_and_angle()
            rospy.loginfo("Goal reached. Waiting for new goal input...\n")
            # Optionally, you can reset or wait before repeating

if __name__ == '__main__':
    try:
        controller = TurtleControllerLab3Infinite()
        controller.run()
    except rospy.ROSInterruptException:
        pass