from pathlib import Path
import shlex
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('my_car_description'))
    description = ParameterValue(Command(['xacro ', shlex.quote(str(share/'urdf/car.urdf.xacro'))]), value_type=str)
    # Wall time: this launch has no simulation clock.
    params = {'use_sim_time': False}
    return LaunchDescription([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[params,{'robot_description':description}]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[params]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/display.rviz')],parameters=[params])])
