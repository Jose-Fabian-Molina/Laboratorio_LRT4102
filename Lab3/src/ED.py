#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SpawnRequest
import math

class TurtleControllerLab3:
    def __init__(self):
        rospy.init_node('turtle_lab3_controller')
        
        # Subscribe to the turtle's pose topic to get the current position
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publisher for sending velocity commands to the turtle
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Wait for the spawn service and create a service proxy
        rospy.wait_for_service('spawn')
        self.spawn_service = rospy.ServiceProxy('spawn', Spawn)
        
        # Initialize current pose variable
        self.current_pose = None
        
        # Initialize desired goal coordinates (to be determined by the user)
        self.goal_x = None
        self.goal_y = None
        
        # Controller gain for proportional control
        self.Kp = 1.0
        
        # Set the loop rate (10 Hz)
        self.rate = rospy.Rate(10)

    def pose_callback(self, pose):
        # Callback function executed when a new turtle pose is received
        self.current_pose = pose

    def get_desired_xy(self):
        # Prompt the user for the desired goal coordinates
        desired_x = float(input("Enter desired x coordinate: "))
        desired_y = float(input("Enter desired y coordinate: "))
        return desired_x, desired_y

    def calculate_dtg_atg(self):
        """
        Calculate the Distance To Goal (DTG) and the Angle To Goal (ATG).
        DTG is the Euclidean distance between the current position and the goal.
        ATG is calculated using the atan2 function.
        """
        dtg = math.sqrt((self.goal_x - self.current_pose.x)**2 + (self.goal_y - self.current_pose.y)**2)
        atg = math.atan2(self.goal_y - self.current_pose.y, self.goal_x - self.current_pose.x)
        return dtg, atg

    def spawn_turtle_at_goal(self):
        """
        Instead of moving the existing turtle, spawn a new turtle at the user-defined goal position.
        """
        req = SpawnRequest()
        req.x = self.goal_x
        req.y = self.goal_y
        req.theta = 0.0
        req.name = "goal_turtle"
        try:
            response = self.spawn_service(req.x, req.y, req.theta, req.name)
            rospy.loginfo("Spawned turtle '%s' at goal position (%.2f, %.2f)", response.name, req.x, req.y)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service call failed: %s", e)

    def move_to_goal(self):
        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue
            
            dtg, atg = self.calculate_dtg_atg()
            rospy.loginfo("DTG: %.2f, ATG: %.2f", dtg, atg)
            
            # Calculate the difference between desired angle and current orientation
            angle_diff = atg - self.current_pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Map the errors to velocities using proportional control
            linear_velocity = self.Kp * dtg
            angular_velocity = self.Kp * angle_diff
            
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            
            self.velocity_publisher.publish(twist_msg)
            
            # Optionally break if goal is reached
            if dtg < 0.1:
                rospy.loginfo("Goal reached!")
                break
            
            self.rate.sleep()

    def run(self):
        # Wait for initial pose update
        rospy.sleep(2)
        
        # Get desired goal coordinates from the user
        self.goal_x, self.goal_y = self.get_desired_xy()
        
        # Calculate and display initial DTG and ATG
        if self.current_pose is not None:
            dtg, atg = self.calculate_dtg_atg()
            rospy.loginfo("Initial DTG: %.2f, Initial ATG: %.2f", dtg, atg)
        else:
            rospy.loginfo("Waiting for turtle pose...")
        
        # Spawn a new turtle at the goal position
        self.spawn_turtle_at_goal()
        
        # Use a proportional controller to move the turtle to the goal in an infinite loop
        self.move_to_goal()

if __name__ == '__main__':
    try:
        controller = TurtleControllerLab3()
        controller.run()
    except rospy.ROSInterruptException:
        pass