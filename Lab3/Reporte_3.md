# Report_3_175629

## Euclidean Distance

This node uses the default turtle (`turtle1`) launched by turtlesim. Every time the user provides new goal coordinates (x, y) and a desired orientation (θ in radians), the node calculates the Distance To Goal (DTG) and Angle To Goal (ATG) based on the current pose, logs these metrics, and then directly teleports `turtle1` to the specified pose without gradual movement. The node runs in an infinite loop so you can continuously test new goal inputs.

## Code

```python
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
```

## Explanation

1. **Node Initialization:**
   - The node is initialized with the name `turtle_teleport_controller`.
   - It waits for the `/turtle1/teleport_absolute` service to become available and creates a service proxy.

2. **Pose Subscription:**
   - The node subscribes to the `/turtle1/pose` topic to get the current pose of `turtle1`.
   - The `pose_callback` function updates the internal `current_pose` variable.

3. **User Input:**
   - The `get_goal_from_user()` function prompts the user to enter the desired goal coordinates (x, y) and the desired orientation (θ, in radians).

4. **Metrics Calculation (DTG and ATG):**
   - The `calculate_dtg_atg()` function calculates:
     - **DTG (Distance To Goal):** Using the Euclidean distance formula.
     - **ATG (Angle To Goal):** Using the `atan2` function.
   - These metrics are logged for user feedback. The comments also explain that if the turtle were to move gradually, linear velocity would be proportional to DTG and angular velocity to the heading error.

5. **Direct Teleportation:**
   - Instead of gradually moving the turtle, the node uses the teleport service to instantly move `turtle1` to the user-specified position and orientation.

6. **Infinite Loop:**
   - The `run()` method contains an infinite loop that continuously prompts the user for new goal inputs and teleports `turtle1` accordingly, allowing for repeated testing.

This solution meets the requirements by using the existing turtle (`turtle1`), allowing user-defined goal parameters, calculating and displaying DTG and ATG, and teleporting the turtle directly to the desired pose in an infinite loop.