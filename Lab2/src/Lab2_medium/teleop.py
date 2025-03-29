#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controla la tortuga con las teclas:")
    print("  w -> Mover en Y")
    print("  d -> Mover en X")
    print("  a -> Mover en -X")
    print("  s -> Mover en -Y")
    print("  -> para girar a la derecha")
    print("  <- para girar a la izquierda")
    print("  p -> Detenerse")
    print("Presiona 'q' para salir.")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        if key == 'w':
            msg.linear.y = 2.0  # Avanza en Y
        elif key == 'd':
            msg.linear.x = 2.0  # Avanza en X
        elif key == 'a':
            msg.linear.x = -2.0 # Avanza en -X
        elif key == 's':
            msg.linear.y = -2.0 # Avanza en -Y
        elif key == 'o':
            msg.angular.z = -1.57 # Gira hacia la derecha
        elif key == 'i':
            msg.angular.z = 1.57 # Gira hacia la izquierda
        elif key == 'p':
            msg.linear.x = 0.0
            msg.linear.y = 0.0  # Detiene el movimiento
        elif key == 'q':  
            print("Saliendo...")
            break  # Sale del loop
        
        pub.publish(msg)

if __name__ == '__main__':
    main()
