"""WASD driving from a focused terminal; release timeout stops the car."""
import os
import select
import sys
import termios
import time
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter

# Speeds: linear m/s, angular rad/s; timeout s.
LINEAR_SPEED = 0.3
ANGULAR_SPEED = 0.8
KEY_TIMEOUT = 0.6


def main():
    if not sys.stdin.isatty():
        raise SystemExit('WASD controller needs an interactive terminal. Run: ros2 run my_car_description keyboard_drive')
    rclpy.init()
    node=Node('wasd_keyboard_drive',parameter_overrides=[Parameter('use_sim_time',value=True)])
    publisher=node.create_publisher(Twist,'/cmd_vel',10)
    stdin_fd = sys.stdin.fileno()
    settings=termios.tcgetattr(stdin_fd)
    command=Twist()
    last_key=0.0
    print('WASD: w forward | s reverse | a turn left | d turn right | Space/x stop | q quit\n'
          'Keep this terminal focused. Release timeout: 0.6 s. Speeds: 0.3 m/s, 0.8 rad/s.',flush=True)
    try:
        tty.setraw(stdin_fd)
        while rclpy.ok():
            if select.select([stdin_fd],[],[],0.05)[0]:
                data = os.read(stdin_fd, 1)
                if not data:
                    break
                key = data.decode('ascii', errors='ignore').lower()
                if key in ('q','\x03'): break
                if key in ('w','a','s','d',' ','x'):
                    command=Twist()
                    if key=='w':command.linear.x=LINEAR_SPEED
                    elif key=='s':command.linear.x=-LINEAR_SPEED
                    elif key=='a':command.angular.z=ANGULAR_SPEED
                    elif key=='d':command.angular.z=-ANGULAR_SPEED
                    last_key=time.monotonic()
            if time.monotonic()-last_key>KEY_TIMEOUT:command=Twist()
            publisher.publish(command)
            rclpy.spin_once(node,timeout_sec=0)
    finally:
        termios.tcsetattr(stdin_fd,termios.TCSADRAIN,settings)
        for _ in range(5):
            publisher.publish(Twist())
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
