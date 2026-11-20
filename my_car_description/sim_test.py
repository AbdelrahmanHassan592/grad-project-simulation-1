"""End-to-end assertions against a running, stationary test_car."""
import argparse
import json
import math
import re
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image, JointState
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def angle(a): return math.atan2(math.sin(a),math.cos(a))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',default='test_world')
    parser.add_argument('--log',type=Path,required=True,help='Launch log to inspect')
    parser.add_argument('--output',type=Path,default=Path('/tmp/car_test'))
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=Node('car_end_to_end_test',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    latest,stamps,edges,owners={}, {}, {}, {}
    duplicate_stamps=[]
    seen_stamps=set()
    def receive(topic,msg):
        latest[topic]=msg
        if hasattr(msg,'header'):
            stamps.setdefault(topic,[]).append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    subs=[]
    for topic,typ in [('/odom',Odometry),('/scan',LaserScan),('/imu',Imu),('/camera/image_raw',Image),('/joint_states',JointState),('/clock',Clock)]:
        subs.append(node.create_subscription(typ,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data))
    def transforms(msg,source):
        for t in msg.transforms:
            key=t.child_frame_id
            edges[key]=t.header.frame_id
            owners.setdefault(key,set()).add(source)
            stamp=(key,t.header.stamp.sec,t.header.stamp.nanosec)
            if stamp in seen_stamps: duplicate_stamps.append(stamp)
            seen_stamps.add(stamp)
    subs.append(node.create_subscription(TFMessage,'/tf',lambda m:transforms(m,'dynamic'),100))
    subs.append(node.create_subscription(TFMessage,'/tf_static',lambda m:transforms(m,'static'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def require(ok,detail):
        print(('PASS: ' if ok else 'FAIL: ')+detail,flush=True)
        if not ok: raise AssertionError(detail)
    def model_pose():
        data=subprocess.run(['gz','topic','-e','-t',f'/world/{args.world}/dynamic_pose/info','-n','1','--json-output'],capture_output=True,text=True,timeout=15,check=True)
        # Transport discovery blocks this thread; refresh ROS timestamps before timing a drive.
        spin(.2)
        # gz topic can deliver a second JSON message before its -n 1 shutdown completes.
        messages=[json.loads(line) for line in data.stdout.splitlines() if line.strip()]
        return next(p for message in reversed(messages) for p in message.get('pose',[]) if p['name']=='test_car')
    def physical(p):
        pos=p['position'];q=p['orientation']
        z=pos.get('z',0);tilt=2*math.acos(min(1,math.sqrt(q.get('w',0)**2+q.get('z',0)**2)))
        require(.035<z<.075 and tilt<.1,f'Ground truth axle z={z:.6f} m, tilt={tilt:.6f} rad')
    try:
        spin(5)
        required={'/cmd_vel','/odom','/scan','/imu','/joint_states','/tf','/clock','/camera/image_raw'}
        require(required <= {n for n,_ in node.get_topic_names_and_types()},'All eight required topics exist')
        deadline=time.monotonic()+40
        while len(latest)<6 and time.monotonic()<deadline:spin(.2)
        require(len(latest)==6,'All bridged streams deliver messages')
        # Run actual ROS CLI rate tools concurrently; use --wall-time explicitly.
        hz=[]
        for topic in ['/scan','/imu','/camera/image_raw']:
            stream=(args.output/(topic.strip('/').replace('/','_')+'_hz.log')).open('w')
            proc=subprocess.Popen(['ros2','topic','hz',topic,'--wall-time'],stdout=stream,stderr=stream)
            hz.append((topic,proc,stream))
        spin(10)
        for topic,proc,stream in hz:
            proc.terminate();proc.wait(timeout=8);stream.close()
            text=(args.output/(topic.strip('/').replace('/','_')+'_hz.log')).read_text()
            rates=[float(v) for v in re.findall(r'average rate: ([0-9.]+)',text)]
            expected={'/scan':10,'/imu':50,'/camera/image_raw':15}[topic]
            require(bool(rates) and .8*expected<rates[-1]<1.2*expected,f'{topic} wall rate={rates[-1] if rates else None} Hz (target {expected} Hz)')
            ts=stamps[topic];rate=(len(ts)-1)/(ts[-1]-ts[0])
            require(.9*expected<rate<1.1*expected,f'{topic} simulation stamp rate={rate:.3f} Hz')
        scan=latest['/scan'];finite=[r for r in scan.ranges if math.isfinite(r)]
        require(len(scan.ranges)==360 and len(finite)>180 and .12<min(finite)<6 and max(finite)<12,'360-ray scan sees nearby room surfaces at sensible ranges')
        require(scan.header.frame_id=='lidar_link','LiDAR frame is lidar_link')
        image=latest['/camera/image_raw']
        require(image.width==640 and image.height==480 and len(image.data)>=640*480*3 and max(image.data)>min(image.data),'640x480 camera image contains nonuniform pixels')
        require(image.header.frame_id=='camera_optical_link','Camera optical frame is correct')
        require(latest['/imu'].header.frame_id=='imu_link','IMU frame is imu_link')
        require(set(latest['/joint_states'].name)=={n+'_wheel_joint' for n in ['front_left','front_right','rear_left','rear_right']},'Four wheel joints received')
        physical(model_pose())
        odom0=latest['/odom'];p0=model_pose()['position']
        heading=yaw(odom0.pose.pose.orientation)
        # Three seconds measured by simulation time, at 0.3 m/s.
        start=stamps['/odom'][-1];cmd=Twist();cmd.linear.x=.3
        while stamps['/odom'][-1]-start<3:
            pub.publish(cmd);spin(.025)
        pub.publish(Twist());spin(.5)
        odom1=latest['/odom'];p1=model_pose()['position']
        dx=odom1.pose.pose.position.x-odom0.pose.pose.position.x
        dy=odom1.pose.pose.position.y-odom0.pose.pose.position.y
        forward=dx*math.cos(heading)+dy*math.sin(heading)
        actual=math.hypot(p1.get('x',0)-p0.get('x',0),p1.get('y',0)-p0.get('y',0))
        print(f'Drive diagnostic: odom={forward:.4f} m, ground truth={actual:.4f} m, wheel velocities={list(latest["/joint_states"].velocity)}',flush=True)
        require(.65<forward<1.15,f'Forward odometry displacement={forward:.4f} m in 3 s')
        if abs(heading)<.1:require(dx>.65,f'Forward command increases odom x: {dx:.4f} m')
        require(.55<actual<1.2,f'Ground truth translation={actual:.4f} m')
        physical0=model_pose()['orientation']
        physical_yaw0=math.atan2(2*(physical0.get('w',1)*physical0.get('z',0)+physical0.get('x',0)*physical0.get('y',0)),1-2*(physical0.get('y',0)**2+physical0.get('z',0)**2))
        h0=yaw(odom1.pose.pose.orientation);cmd=Twist();cmd.angular.z=.5;start=stamps['/odom'][-1]
        while stamps['/odom'][-1]-start<2:
            pub.publish(cmd);spin(.025)
        pub.publish(Twist());spin(.5)
        change=angle(yaw(latest['/odom'].pose.pose.orientation)-h0)
        require(.7<change<1.3,f'Turn changes odometry yaw by {change:.4f} rad')
        turned=model_pose();physical(turned)
        q=turned['orientation']
        physical_yaw1=math.atan2(2*(q.get('w',1)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
        require(.3<angle(physical_yaw1-physical_yaw0)<1.5,f'Ground truth yaw changes by {angle(physical_yaw1-physical_yaw0):.4f} rad')
        poses=[]
        for _ in range(4):poses.append(model_pose());spin(.25)
        jitter=max(p['position'].get('z',0) for p in poses)-min(p['position'].get('z',0) for p in poses)
        require(jitter<.005,f'Stationary ground-truth vertical jitter={jitter:.6f} m')
        expected={'base_link':'odom',**{n+'_wheel_link':'base_link' for n in ['front_left','front_right','rear_left','rear_right']},'lidar_link':'base_link','camera_link':'base_link','camera_optical_link':'camera_link','imu_link':'base_link'}
        require(all(edges.get(k)==v for k,v in expected.items()),f'Connected TF tree: {edges}')
        dynamic_publishers=node.get_publishers_info_by_topic('/tf')
        static_publishers=node.get_publishers_info_by_topic('/tf_static')
        names=sorted(p.node_name for p in dynamic_publishers)
        require(len(dynamic_publishers)==2 and 'robot_state_publisher' in names and any('bridge' in n for n in names) and len(static_publishers)==1,f'TF publishers are exactly state publisher and bridge: {names}; static count={len(static_publishers)}')
        require(all(len(v)==1 for v in owners.values()) and not duplicate_stamps,'No child appears on both TF channels and no duplicate transform timestamps')
        echo=subprocess.run(['timeout','5','ros2','run','tf2_ros','tf2_echo','odom','base_link','--ros-args','-p','use_sim_time:=true'],capture_output=True,text=True)
        (args.output/'tf2_echo.log').write_text(echo.stdout+echo.stderr)
        require('Translation:' in echo.stdout,'tf2_echo odom base_link returns transforms')
        frames=subprocess.run(['ros2','run','tf2_tools','view_frames','--ros-args','-p','use_sim_time:=true'],cwd=args.output,capture_output=True,text=True,timeout=20)
        (args.output/'view_frames.log').write_text(frames.stdout+frames.stderr)
        require(frames.returncode==0 and bool(list(args.output.glob('*.pdf'))),'view_frames generated TF graph PDF')
        log=args.log.read_text(errors='replace')
        bad=[line for line in log.splitlines() if re.search(r'\[ERROR\]|\[Err\]|extrapolat|TF_REPEATED_DATA|TF_OLD_DATA|earlier than all the data in the transform cache',line,re.I)]
        require(not bad,'Launch logs contain no errors or TF timestamp warnings: '+str(bad))
        print('PASS: complete end-to-end simulation test',flush=True)
    finally:
        for _ in range(5):pub.publish(Twist());spin(.02)
        node.destroy_node();rclpy.shutdown()
