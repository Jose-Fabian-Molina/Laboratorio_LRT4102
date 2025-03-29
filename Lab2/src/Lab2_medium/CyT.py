#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty

# Global variable to store the current pose of the turtle.
current_pose = None

def pose_callback(msg):
    """
    Callback function to update the current pose of the turtle.
    """
    global current_pose
    current_pose = msg

def move_to_point(target_x, target_y):
    """
    Moves the turtle to the specified (target_x, target_y) in two phases:
    Phase 1: Rotate in place until the turtle is aligned with the target.
    Phase 2: Move straight toward the target.
    """
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()

    # Phase 1: Rotate to align with the target.
    while not rospy.is_shutdown():
        # Calculate the desired angle.
        angle_to_target = math.atan2(target_y - current_pose.y, target_x - current_pose.x)
        angle_error = angle_to_target - current_pose.theta
        # Normalize the angular error to the range [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        if abs(angle_error) < 0.01:  # Threshold for orientation
            break
        
        twist.linear.x = 0.0
        twist.angular.z = 2.0 * angle_error  # Proportional gain for rotation.
        pub.publish(twist)
        rate.sleep()

    # Phase 2: Move straight towards the target.
    while not rospy.is_shutdown():
        # Calculate the distance to the target.
        distance = math.sqrt((target_x - current_pose.x)**2 + (target_y - current_pose.y)**2)
        if distance < 0.05:  # Threshold for position
            break
        
        # Calculate the angle error again (should be minimal)
        angle_to_target = math.atan2(target_y - current_pose.y, target_x - current_pose.x)
        angle_error = angle_to_target - current_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        twist.linear.x = 1.0 * distance   # Proportional gain for linear movement.
        twist.angular.z = 1.0 * angle_error  # Small angular correction if needed.
        pub.publish(twist)
        rate.sleep()
    
    # Stop the turtle once the target is reached.
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)

def clear_screen():
    """
    Calls the ROS service to clear the turtlesim screen.
    """
    rospy.wait_for_service('clear')
    try:
        clear = rospy.ServiceProxy('clear', Empty)
        clear()
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call service clear: %s", e)

def teleport_to(x, y, theta=0):
    """
    Teleports the turtle to the specified (x, y) position with orientation theta.
    """
    rospy.wait_for_service('turtle1/teleport_absolute')
    try:
        teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
        teleport(x, y, theta)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call service teleport_absolute: %s", e)

def is_within_area(x, y, x_min=0, x_max=11, y_min=0, y_max=11):
    """
    Checks if the point (x, y) is within the defined work area.
    """
    return x_min <= x <= x_max and y_min <= y <= y_max

def draw_triangle(initial_x, initial_y):
    """
    Draws an equilateral triangle starting from the initial position.
    The triangle's vertices are calculated based on preset dimensions.
    It displays the vertex coordinates, validates that they lie within the work area,
    and then moves the turtle to draw the triangle.
    """
    side = 2.0
    height = (math.sqrt(3) / 2) * side
    v1 = (initial_x, initial_y)
    v2 = (initial_x + side, initial_y)
    v3 = (initial_x + side/2, initial_y + height)
    vertices = [v1, v2, v3]
    
    print("\n--- Triangle ---")
    for i, v in enumerate(vertices):
        print("Vertex {}: ({:.2f}, {:.2f})".format(i+1, v[0], v[1]))
    
    # Validate that all vertices are within the work area.
    for v in vertices:
        if not is_within_area(v[0], v[1]):
            print("Error: The triangle is out of the work area. Please enter a new initial position.\n")
            return False

    # Draw the triangle by moving to each vertex sequentially.
    for v in vertices:
        move_to_point(v[0], v[1])
    # Close the shape by returning to the first vertex.
    move_to_point(v1[0], v1[1])
    
    print("Triangle drawing completed.")
    input("Press any key to return to the main menu...")
    clear_screen()
    teleport_to(initial_x, initial_y)
    return True

def draw_square(initial_x, initial_y):
    """
    Draws a square starting from the initial position (defined as the bottom-left vertex).
    The square's vertices are calculated based on preset dimensions.
    It displays the vertex coordinates, validates that they lie within the work area,
    and then moves the turtle to draw the square.
    """
    length = 2.0
    # Define the vertices:
    v1 = (initial_x, initial_y)
    v2 = (initial_x + length, initial_y)
    v3 = (initial_x + length, initial_y + length)
    v4 = (initial_x, initial_y + length)
    vertices = [v1, v2, v3, v4]
    
    print("\n--- Square ---")
    for i, v in enumerate(vertices):
        print("Vertex {}: ({:.2f}, {:.2f})".format(i+1, v[0], v[1]))
    
    # Validate that all vertices are within the work area.
    for v in vertices:
        if not is_within_area(v[0], v[1]):
            print("Error: The square is out of the work area. Please enter a new initial position.\n")
            return False

    # Draw the square by moving to each vertex sequentially.
    for v in vertices:
        move_to_point(v[0], v[1])
    move_to_point(v1[0], v1[1])
    
    print("Square drawing completed.")
    input("Press any key to return to the main menu...")
    clear_screen()
    teleport_to(initial_x, initial_y)
    return True

def main():
    """
    Main function that initializes the ROS node, handles the user menu,
    and calls the appropriate functions to draw the selected shape.
    """
    rospy.init_node('turtle_drawer', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.sleep(2)  # Wait for the turtle to initialize.
    
    while not rospy.is_shutdown():
        print("\nMain Menu:")
        print("1. Draw Triangle")
        print("2. Draw Square")
        print("3. Exit")
        option = input("Select an option: ")
        
        if option == "1" or option == "2":
            valid_input = False
            while not valid_input:
                try:
                    initial_x = float(input("Enter the initial X position: "))
                    initial_y = float(input("Enter the initial Y position: "))
                except ValueError:
                    print("Invalid input. Please enter numbers.")
                    continue
                if not is_within_area(initial_x, initial_y):
                    print("The initial position is out of the work area. Try again.")
                    continue
                valid_input = True

            # Teleport the turtle to the initial position before starting.
            teleport_to(initial_x, initial_y)
            rospy.sleep(1)
            
            if option == "1":
                if not draw_triangle(initial_x, initial_y):
                    continue
            elif option == "2":
                if not draw_square(initial_x, initial_y):
                    continue

        elif option == "3":
            print("Exiting the program.")
            break
        else:
            print("Invalid option. Please select again.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass