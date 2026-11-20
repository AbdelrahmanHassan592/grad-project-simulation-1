#!/usr/bin/env python3
"""Launch both worlds, test live data and WASD, and stop each server cleanly."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from my_car_description.processes import stop_gazebo_children


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path.home()/'car_ws/verification/final')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('my_car_description'))
    env=os.environ.copy()
    nvidia=Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    if nvidia.exists():env.setdefault('__EGL_VENDOR_LIBRARY_FILENAMES',str(nvidia))
    def run(command,logfile,timeout=180):
        with (output/logfile).open('w') as stream:
            result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=env,timeout=timeout)
        print(f'{"PASS" if result.returncode==0 else "FAIL"}: {logfile}',flush=True)
        if result.returncode:raise RuntimeError((output/logfile).read_text())
    def scenario(name,arguments,world,test=False,keyboard=False):
        logpath=output/(name+'_launch.log')
        with logpath.open('w') as log:
            proc=subprocess.Popen(['ros2','launch','my_car_description','sim.launch.py',*arguments],stdout=log,stderr=log,env=env,start_new_session=True)
            try:
                # Wall time in s: allow shader compilation and RViz startup.
                time.sleep(18)
                if proc.poll() is not None:raise RuntimeError(logpath.read_text())
                if 'Entity creation successful' not in logpath.read_text():raise RuntimeError('Spawn failed: '+str(logpath))
                if test:
                    results = output / ('absolute_test_world' if name == 'absolute' else world)
                    run(['ros2','run','my_car_description','test_sim','--world',world,'--log',str(logpath),'--output',str(results)],name+'_test.log')
                else:
                    run(['ros2','topic','echo','/scan','--once'],name+'_scan.log',30)
                    run(['ros2','topic','echo','/camera/image_raw','--once','--field','header'],name+'_camera.log',30)
                if keyboard:run(['python3',str(share/'scripts/test_keyboard.py')],name+'_keyboard.log',60)
            finally:
                stop_gazebo_children(proc.pid)
                subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','stop: true'],capture_output=True,timeout=8)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    proc.send_signal(signal.SIGINT)
                    try:proc.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)
        text=logpath.read_text()
        bad=[line for line in text.splitlines() if '[ERROR]' in line or '[Err]' in line]
        if bad:raise RuntimeError('\n'.join(bad))
        print('PASS: '+name+' launched, tested and stopped with no errors',flush=True)
    scenario('default',[],'test_world',test=True,keyboard=True)
    scenario('warehouse',['world:=sample_warehouse.sdf','x:=2.0','y:=1.0','yaw:=1.57'],'sample_warehouse',test=True)
    scenario('absolute',['world:='+str(share/'worlds/test_world.sdf'),'headless:=true','rviz:=false'],'test_world',test=True)
    print('PASS: all launch scenarios and live tests',flush=True)

if __name__=='__main__':main()
