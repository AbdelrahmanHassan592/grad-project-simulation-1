#!/usr/bin/env python3
"""Exercise the WASD controller through a real PTY against a live simulation."""
import math
import os
import pty
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node=rclpy.create_node('wasd_terminal_test',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    values={}
    def save_cmd(m): values['cmd']=m
    def save_odom(m): values['odom']=m
    a=node.create_subscription(Twist,'/cmd_vel',save_cmd,10)
    b=node.create_subscription(Odometry,'/odom',save_odom,qos_profile_sensor_data)
    master,slave=pty.openpty()
    process=subprocess.Popen(['ros2','run','my_car_description','keyboard_drive'],stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
    os.close(slave)
    os.set_blocking(master,False)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def heading():
        q=values['odom'].pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def require(ok,text):
        print(('PASS: ' if ok else 'FAIL: ')+text,flush=True)
        if not ok:raise AssertionError(text)
    try:
        deadline=time.monotonic()+15
        while ('odom' not in values or 'cmd' not in values) and time.monotonic()<deadline and process.poll() is None:spin(.1)
        if 'odom' not in values or 'cmd' not in values:
            try: print('PTY output:',os.read(master,65536).decode(errors='replace'),flush=True)
            except OSError: pass
            print('Observed streams:',list(values),'keyboard exit:',process.poll(),flush=True)
        require('odom' in values and 'cmd' in values,'PTY keyboard node and simulation are connected')
        for key in ['w','a','d']:
            start=values['odom'].pose.pose.position
            x0,y0=start.x,start.y;h0=heading()
            for _ in range(15):
                os.write(master,key.encode());spin(.1)
            cmd=values['cmd']
            if key=='w':
                displacement=math.hypot(values['odom'].pose.pose.position.x-x0,values['odom'].pose.pose.position.y-y0)
                require(cmd.linear.x==.3 and displacement>.25,f'w drives forward: {displacement:.4f} m')
            else:
                delta=math.atan2(math.sin(heading()-h0),math.cos(heading()-h0))
                require((delta>.5 and cmd.angular.z==.8) if key=='a' else (delta<-.5 and cmd.angular.z==-.8),f'{key} turns '+('left' if key=='a' else 'right')+f': yaw change {delta:.4f} rad')
            spin(.8)
            require(values['cmd'].linear.x==0 and values['cmd'].angular.z==0,'Key release timeout sends zero velocity')
        os.write(master,b'w');spin(.2);os.write(master,b' ');spin(.15)
        require(values['cmd'].linear.x==0 and values['cmd'].angular.z==0,'Space stops immediately')
        os.write(master,b'xq');process.wait(timeout=5)
        require(process.returncode==0,'q exits cleanly when buffered with another key')
    finally:
        if process.poll() is None:
            os.write(master,b'q')
            process.wait(timeout=5)
        os.close(master);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
