# ROS 2 Jazzy car in Gazebo Harmonic

A four-wheel skid-steer car with a 360-ray LiDAR, RGB camera and IMU. The model uses meters (m), kilograms (kg), radians (rad), seconds (s), and a right-handed x-forward, y-left, z-up frame. Wheel axle height at rest is 0.05 m. `base_link` is the root; Gazebo DiffDrive alone publishes `odom -> base_link`. Robot state publisher supplies the remaining transforms using simulated joint states. Odometry is wheel-integrated, not ground truth. The chassis underside is a frictionless skid plate because the specified 0.10 m body height reaches the tire contact plane. The Harmonic WheelSlip system gives tires lateral compliance for physical skid steering while preserving wheel friction coefficients of 1.0; longitudinal compliance is zero. Physical yaw can differ from wheel odometry during a skid turn.

## Install, build and run

On Ubuntu 24.04 with ROS 2 Jazzy configured:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools liburdfdom-tools graphviz python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
cd ~/car_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select my_car_description
source install/setup.bash
ros2 launch my_car_description sim.launch.py
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/car_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
```

For WASD control, stop the other teleop controller and run:

```bash
ros2 run my_car_description keyboard_drive
```

Focus that terminal: hold `w` to drive forward, `a` to turn left, `d` to turn right, `s` to reverse. Space or `x` stops; `q` quits. Linear speed is 0.3 m/s and turn speed is 0.8 rad/s. If no key arrives for 0.6 s, the controller sends a stop. Keyboard auto-repeat must be enabled to hold a command continuously. Run only one driving controller at a time.

For the standard ROS teleop command above, use `i` for forward, `j`/`l` to turn and `k` to stop. Keep the teleop terminal focused; Ctrl-C stops teleop. Publish a zero Twist before leaving an automated controller: DiffDrive keeps its most recent velocity command.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| world | test_world.sdf | Installed world filename or absolute path |
| x, y | 0, 0 | Spawn coordinates (m) |
| z | 0.1 | Initial wheel axle height (m) |
| yaw | 0 | Initial heading (rad) |
| headless | false | Server only, with EGL rendering |
| rviz | true | Start RViz; set false without a display |

```bash
ros2 launch my_car_description sim.launch.py world:=sample_warehouse.sdf x:=2.0 y:=1.0 yaw:=1.57
ros2 launch my_car_description sim.launch.py world:=/home/me/worlds/anything.sdf headless:=true rviz:=false
ros2 launch my_car_description display.launch.py
```

The display launch uses wall time and a separate RViz configuration fixed to `base_link`, because no `odom` or simulation clock exists in that launch. Every simulation node uses simulation time. There is no joint state publisher GUI in the simulation launch.

## Validation and live tests

```bash
cd ~/car_ws
source install/setup.bash
xacro src/my_car_description/urdf/car.urdf.xacro > /tmp/car.urdf
xacro src/my_car_description/urdf/car.urdf.xacro use_gazebo:=true > /tmp/car_gz.urdf
check_urdf /tmp/car.urdf
check_urdf /tmp/car_gz.urdf
gz sdf -p /tmp/car_gz.urdf > /tmp/car.sdf
urdf_to_graphviz /tmp/car.urdf /tmp/car_tree
gz sdf -k src/my_car_description/worlds/test_world.sdf
gz sdf -k src/my_car_description/worlds/sample_warehouse.sdf
ros2 run my_car_description check_world src/my_car_description/worlds/test_world.sdf
```

Launch to a log, then run the test from another terminal. Start a fresh simulation before each test: the test drives forward at 0.3 m/s for 3 s and turns at 0.5 rad/s for 2 s. It requires free space ahead of the initial spawn.

```bash
ros2 launch my_car_description sim.launch.py > /tmp/car_launch.log 2>&1
# In another sourced terminal:
~/car_ws/src/my_car_description/scripts/test_sim.sh --world test_world --log /tmp/car_launch.log --output /tmp/test_world_results
```

Repeat in the warehouse with `world:=sample_warehouse.sdf x:=2.0 y:=1.0 yaw:=1.57`; use `--world sample_warehouse` in the test. The test checks ROS CLI wall-clock rates within 20 percent of 10 Hz scan, 50 Hz IMU and 15 Hz camera, and message timestamp rates within 10 percent. Rendering overload can reduce wall-clock rates without changing simulated sensor periods. The launch supplies `config/fastdds.xml` through `FASTRTPS_DEFAULT_PROFILES_FILE` unless that variable is already set. Its 16 MiB shared-memory segments prevent drops of uncompressed RGB frames with the default Fast DDS transport; other RMW implementations ignore the profile. It also inspects Gazebo model poses, image pixels, scan ranges, TF publishers, `tf2_echo`, `view_frames`, and launch errors. Logs and TF graph PDFs are saved under `--output`.

Stop launch with Ctrl-C and wait for shutdown before starting another. If a prior launch crashed, inspect `pgrep -af 'gz sim|ros2 launch'` and stop those simulation processes before relaunching. Saved development verification logs are in `~/car_ws/verification/`. To reproduce all three launch scenarios, live tests, and WASD terminal tests automatically with a display available, first stop any running simulation, then run:

```bash
python3 ~/car_ws/src/my_car_description/scripts/verify_all.py
```

This runner saves logs and graphs under `~/car_ws/verification/final/` by default. Use `--output` to choose another directory. The default room, warehouse and absolute-path headless tests save their graphs and topic-rate logs in `test_world/`, `sample_warehouse/` and `absolute_test_world/`, respectively. The runner stops each Gazebo server before starting the next. RViz starts after a 15 s renderer warm-up.

## Using your own world

Use a descriptive `.sdf` filename such as `my_room.sdf`. Put it in `src/my_car_description/worlds/`, rebuild, source the workspace, and launch `world:=my_room.sdf`. Alternatively pass an absolute path; this needs no rebuild or code change:

```bash
ros2 run my_car_description check_world /home/me/worlds/my_room.sdf
ros2 run my_car_description check_world /home/me/worlds/my_room.sdf --fix
ros2 launch my_car_description sim.launch.py world:=/home/me/worlds/my_room.sdf
```

`--fix` never edits the source file. It writes `worlds/my_room_fixed.sdf` in the package containing the checker, adding missing Harmonic systems and correcting the Sensors engine to `ogre2`. When moving the corrected copy to another folder, resolved local URIs become absolute paths so existing assets remain reachable. Launch the printed path to use it. Missing plugins are warnings when checking and launching; malformed XML, an invalid SDF, a missing file, or an SDF without a world are fatal. Floor and model-intent checks are heuristic, especially for included models. For the standalone SDFormat validator, the checker mirrors resource roots into `SDF_PATH` while preserving its existing value. Remote Fuel include lookup errors are reported as deferred checks because `gz sdf` cannot fetch them; Gazebo performs the actual download and load. The checker reports every explicit `model://` URI and mesh URI in the supplied file, resolving local resource paths and cached Fuel models. It does not recursively audit included model files, download remote resources, or infer the real-world units of an arbitrary mesh. OBJ, STL and DAE collision face counts over 10,000 trigger a warning.

Each world needs these systems: `gz-sim-physics-system`, `gz-sim-user-commands-system`, `gz-sim-scene-broadcaster-system`, `gz-sim-sensors-system` with `ogre2`, and `gz-sim-imu-system`. Copy a supplied world as a starting point. Use static floors and building geometry, at least one light, gravity `0 0 -9.8` m/s² and a physics step no larger than 0.01 s (supplied worlds use 0.001 s).

The launch preserves `GZ_SIM_RESOURCE_PATH` and adds package `models/`, `worlds/`, the package share parent, and the selected world's directory. For `model://rack`, put `rack/model.config` and `rack/model.sdf` under `models/` or another resource root. Mesh paths relative to the world directory work; for included models, place meshes relative to their model SDF. Add any other asset roots before launching:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/my_gazebo_assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

Gazebo Fuel URLs need internet on first download; offline use requires cached assets. The checker reports remote or unresolved resources as warnings.

To build a room, start with the provided floor and wall boxes and change their SDF poses and dimensions. Trace a floor plan using one static box per wall segment, converting drawing distances to m. Gazebo Harmonic's GUI can insert primitives and manipulate their poses. The older Gazebo Building Editor belongs to Gazebo Classic; if using it externally, export geometry and rebuild the world with the Harmonic systems above rather than loading Classic plugins.

Blender can export DAE, OBJ or STL. Set scene units to meters, apply object transforms, and export simple geometry. A mesh authored in millimeters needs `<scale>0.001 0.001 0.001</scale>` to convert to m, unless the export already converted units. The checker flags suggestive filenames or extreme scales; always inspect actual dimensions. Use detailed meshes for visuals and simple boxes, cylinders or low-poly convex meshes for collisions. Set `<static>true</static>` on walls, shelves and floors.

Spawn `z` refers to axle height, not chassis bottom. For a floor whose upper surface is at height H m, use `z = H + 0.10` m to start 0.05 m above wheel contact and let the car settle. Select x/y in free space; shelves and obstacles are solid.

## Troubleshooting

| Symptom | Cause and remedy |
|---|---|
| Missing world | Error lists installed filenames; rebuild after adding a packaged world, or use an absolute path. |
| No scan/camera | Confirm Sensors system uses `ogre2`; inspect renderer logs and `/clock`. |
| EGL driver warnings or slow headless rendering | On NVIDIA, try `export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`; otherwise use the normal display or Mesa software rendering. |
| RViz scan absent | Scan display uses Best Effort; check `/scan` data and TF. |
| TF timestamp/extrapolation | Source the correct workspace, use simulation time on consumers, and stop stale simulations. |
| Root inertia KDL warning | Expected because the required root is `base_link` with inertia; Gazebo physics still uses it. |
| SDF warns about `gz_frame_id` | Harmonic preserves this sensor extension; the live test verifies frame IDs. |
| Car appears sunken or hits a shelf | Choose a free spawn and set z relative to the floor's upper surface. |
| Keyboard does nothing | Focus the teleop terminal and check `/cmd_vel`; only run one driving controller. |
| Missing model/mesh | Run the checker and add its asset root to `GZ_SIM_RESOURCE_PATH`; Fuel needs internet. |
| Two odometry transforms | Stop other controllers/bridges; this package uses only DiffDrive for odom TF. |

API references: [Harmonic DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html), [Harmonic WheelSlip](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WheelSlip.html), [Gazebo sensor frame migration](https://gazebosim.org/docs/all/migrating_gazebo_classic_ros2_packages/).
