import os
from pathlib import Path
import shlex
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnShutdown
from my_car_description.processes import stop_gazebo_children
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from my_car_description.world_check import check_world


def start(context):
    share = Path(get_package_share_directory('my_car_description'))
    requested = LaunchConfiguration('world').perform(context)
    world = Path(requested).expanduser()
    if not world.is_absolute():
        if world.name != requested:
            raise RuntimeError('world must be a file name or absolute path')
        world = share/'worlds'/requested
    if not world.is_file():
        available = ', '.join(sorted(p.name for p in (share/'worlds').glob('*.sdf')))
        raise RuntimeError(f'World not found: {world}. Available worlds: {available}')
    roots = [share/'models', share/'worlds', share.parent, world.parent]
    resource_path = os.pathsep.join(map(str, roots))
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']
    # Set before checking so URI resolution and Gazebo use identical roots.
    os.environ['GZ_SIM_RESOURCE_PATH'] = resource_path
    errors, warnings, _ = check_world(world)
    for warning in warnings: print('World WARNING: ' + warning, flush=True)
    if errors: raise RuntimeError('World validation failed: ' + '; '.join(errors))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    args = '-r ' + ('-s --headless-rendering ' if headless else '') + shlex.quote(str(world))
    description = ParameterValue(Command(['xacro ', shlex.quote(str(share/'urdf/car.urdf.xacro')), ' use_gazebo:=true']), value_type=str)
    params = {'use_sim_time': True}
    gazebo = {}
    def remember(event, context):
        if event.process_name.startswith('gazebo-'): gazebo['pid'] = event.pid
    def stop(event, context):
        if 'pid' in gazebo:
            stop_gazebo_children(gazebo['pid'])
            # Give the GUI/server children time to exit before the shell is signaled (s).
            time.sleep(2.0)
    nodes = [RegisterEventHandler(OnProcessStart(on_start=remember)),
        RegisterEventHandler(OnShutdown(on_shutdown=stop)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', str(share/'config/fastdds.xml'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')), launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[params, {'robot_description': description}], output='screen'),
        Node(package='ros_gz_sim', executable='create', arguments=['-topic','robot_description','-name','test_car','-x',LaunchConfiguration('x'),'-y',LaunchConfiguration('y'),'-z',LaunchConfiguration('z'),'-Y',LaunchConfiguration('yaw')], parameters=[params], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[params, {'config_file': str(share/'config/bridge.yaml')}], output='screen'),
        Node(package='ros_gz_image', executable='image_bridge', arguments=['/camera/image_raw'], parameters=[params], output='screen')]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        # Delay the viewer past initial shader compilation and first sensor stamps (s).
        nodes.append(TimerAction(period=15.0, actions=[Node(package='rviz2', executable='rviz2', arguments=['-d',str(share/'rviz/sim.rviz')], parameters=[params], output='screen')]))
    return nodes


def generate_launch_description():
    arguments = [('world','test_world.sdf','Package world filename or absolute SDF path'),
                 ('x','0','Spawn x position (m)'), ('y','0','Spawn y position (m)'),
                 ('z','0.1','Spawn axle height (m)'), ('yaw','0','Spawn heading (rad)'),
                 ('headless','false','Run server only with EGL rendering (true/false)'),
                 ('rviz','true','Open RViz2 (true/false)')]
    return LaunchDescription([DeclareLaunchArgument(n,default_value=v,description=d) for n,v,d in arguments] + [OpaqueFunction(function=start)])
