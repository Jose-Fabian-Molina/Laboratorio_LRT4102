# Report_2_175629

## Lab2 Basic
- Create a ros package called Prcaticas_lab with the dependencies rospy, roscpp & std_msgs. 
- Insert the files listener.py & talker.py
- Compile the package.
- Run the talker file.
- Run the listener file.
- Conclude about its functioning.

### talker.py
This is a basic ROS publisher node that sends "hello world" messages to the "chatter" topic at a rate of 1 Hz. It demonstrates the 
creation of a publisher, node initialization, and periodic message publication using a simple loop.

### listener.py
This is a basic ROS subscriber node that listens to the "chatter" topic and logs any messages received. It uses a callback function to 
process incoming messages and prints the information to the console.

## Lab 2 Medium
- Create a keyboard control for turtlesim.
- Draw a square and a equilateral triangule withb turtlesim (No control).

### teleop.py
This code allows keyboard teleoperation of the turtlesim turtle. It captures key presses (without needing to press Enter) and maps 
specific keys to linear and angular velocity commands. For example, keys such as 'w', 'a', 's', and 'd' are used for translational 
movement, while 'o' and 'i' control rotation (right and left, respectively). The node publishes these commands to the `/turtle1/
cmd_vel` topic.

### CyT.py
This code demonstrates how to draw geometric shapes (such as a triangle and a square) using turtlesim. It calculates vertex coordinates 
based on preset dimensions, validates that the vertices are within the work area, and then moves the turtle sequentially between them 
to draw the shape. Additionally, it includes functions to clear the screen and teleport the turtle back to its initial position after 
drawing.

## Lab2 Advanced
- Position control for turtlesim (P).
- Position control for turtlesim (P).
- Position control for turtlesim (P).

### turtle_pc.py
This code uses a **proportional controller** to move the turtlesim turtle to a user-specified (x, y) position. It subscribes to the 
turtle's pose and publishes velocity commands to drive the turtle. The program prompts the user for x and y coordinates, then 
calculates the error in position and adjusts the turtle's movement accordingly using proportional control.

### turtle_pdc.py
This code implements a **PD (Proportional-Derivative) controller** to move the turtlesim turtle to a user-specified (x, y) position. 
The PD controller computes the control signal based on both the current error and the rate of change of the error. This results in 
smoother movement. The node subscribes to the turtle's pose and continuously updates the command until the turtle reaches the desired 
position.

### turtle_pidc.py
This code uses a **PID (Proportional-Integral-Derivative) controller** to move the turtlesim turtle to a user-specified (x, y) 
position. It calculates the proportional, integral, and derivative terms for both the linear (distance) and angular (orientation) 
movements. The code prompts the user for the target coordinates, and the PID control loop computes the necessary commands to drive the 
turtle accurately to the goal.