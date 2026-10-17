# The Webots simulator (`ugv_webots`)

`src/ugv_main/ugv_webots` simulates the Waveshare UGV Rover (`UGV_MODEL=ugv_rover`) in
Webots R2025a with ROS 2 Humble. It replaces Gazebo as the team's default simulator.
`ugv_gazebo` is still in the repo, untouched.

The goal is one rule: **the simulator publishes the same topics, types and frames as the
Gazebo setup** (`ugv_gazebo/config/ros_gz_bridge.yaml`), so the SLAM and Nav2 launch files and
configs of `ugv_slam` and `ugv_nav` run unchanged.

## Contents

* [Running it](#running-it)
* [How it works](#how-it-works)
* [Topic and frame contract](#topic-and-frame-contract)
* [Robot model and regenerating the PROTO](#robot-model-and-regenerating-the-proto)
* [Drive and contact tuning](#drive-and-contact-tuning)
* [Worlds and adding a world](#worlds-and-adding-a-world)
* [Performance](#performance)
* [Checking the simulator](#checking-the-simulator)
* [Known gaps versus the real robot](#known-gaps-versus-the-real-robot)

## Running it

| Command | What it starts |
|---|---|
| `ros2 launch ugv_webots ugv_webots.launch.py` | Webots, the robot, controllers, sensors |
| `ros2 launch ugv_webots gmapping.launch.py` | Simulation + gmapping SLAM + RViz |
| `ros2 launch ugv_webots cartographer.launch.py` | Simulation + Cartographer SLAM + RViz |
| `ros2 launch ugv_webots slam_toolbox.launch.py use_slam:=sync` | Simulation + slam_toolbox (`sync` or `async`) + RViz |
| `ros2 launch ugv_webots nav.launch.py` | Simulation + Nav2 with AMCL on `maps/apartment.yaml` + RViz |
| `ros2 launch ugv_webots slam_nav.launch.py` | Simulation + SLAM and Nav2 together |

Arguments of `ugv_webots.launch.py` (see `--show-args`):

| Argument | Default | Meaning |
|---|---|---|
| `world` | `apartment.wbt` | World in `ugv_webots/worlds` (or an absolute path) |
| `mode` | `realtime` | Webots start mode: `realtime`, `fast`, `pause` |
| `headless` | `false` | Start Webots minimized without rendering its 3D view (sensors still render) |
| `use_rviz` | `false` | Open RViz |
| `rviz_config` | `webots` | `webots` (all sensors), or the upstream configs `description`, `bringup`, `slam_2d`, `slam_3d`, `nav_2d`, `nav_3d` |
| `use_oak_points` | `true` | Build `/oak/points` from the depth image |

The wrappers accept `world`, `use_rviz` (default `true`) and `headless`. `nav.launch.py`
also mirrors `ugv_nav/launch/nav.launch.py`: `map`, `use_localization` (`amcl`, `emcl`,
`cartographer`, `slam_toolbox`), `use_localplan` (`dwa`, `teb`, `rpp`, `mppi`), `use_slam`,
`use_keepout_zones`, `keepout_mask`.

Drive with `ros2 run ugv_tools keyboard_ctrl`, or publish `geometry_msgs/msg/Twist` on
`/cmd_vel`. Save a map with
`ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p use_sim_time:=true`.

### Why the wrappers do not call `ugv_slam`'s launch files directly

Upstream switches to simulation with `use_sim_time:=true`, which makes `gmapping.launch.py`,
`nav.launch.py` and the others include `ugv_gazebo/launch/bringup_gazebo.launch.py`. The
Webots wrappers therefore include the same *inner* launch files the upstream ones use
(`slam_gmapping/mapping.launch.py`, `cartographer/mapping.launch.py`, slam_toolbox's
`online_*_launch.py` with `ugv_slam/config/*.yaml`, `ugv_nav/launch/nav_bringup/nav2_bringup.launch.py`
with `ugv_nav/params/*.yaml`, `robot_pose_publisher`) with `use_sim_time:=true`, and start
Webots where upstream starts Gazebo or the real robot drivers. No configuration file is copied.

## How it works

```
 Webots (Windows, macOS or Linux)              ROS 2 Humble (WSL, VM or Linux)
+----------------------------------+   TCP    +-------------------------------------------+
| worlds/*.wbt                     |  1234    | webots_controller_ugv_rover (driver)      |
| protos/UgvRover.proto            |<-------->|   ros2_control: diffdrive_controller,     |
|   motors, encoders, LD19 lidar,  |          |     joint_state_broadcaster,              |
|   pan-tilt camera, OAK RGB+depth |          |     pt_joint_position_controller          |
| Ros2Supervisor robot (/clock)    |<-------->|   sensor plugins, ground truth plugin     |
+----------------------------------+          | ros2_supervisor.py  -> /clock             |
                                              | robot_state_publisher (upstream URDF)     |
                                              | scan_normalizer     -> /scan              |
                                              | point_cloud_xyz_node -> /oak/points       |
                                              +-------------------------------------------+
```

Package layout:

```
ugv_webots/
  launch/      ugv_webots.launch.py, gmapping, cartographer, slam_toolbox, nav, slam_nav (+ _common.py)
  worlds/      empty_arena.wbt, apartment.wbt
  protos/      UgvRover.proto (generated, then tuned by the generator script)
  resource/    webots_extras.urdf (driver plugins, ros2_control), ros2_control.yaml
  scripts/     generate_proto.py (URDF to PROTO)
  maps/        apartment.yaml/.pgm (made with gmapping in the apartment world)
  rviz/        view_webots.rviz
  ugv_webots/  description.py, scan_normalizer.py, ground_truth.py, webots_host.py,
               motion_check.py, rtf_meter.py
```

It is an `ament_python` package: everything in it is Python (launch logic, the URDF
builder, two small nodes, a Webots Python plugin, tools). The C++ parts are the stock
`webots_ros2` plugins.

**Robot description at launch.** `ugv_webots/description.py` runs xacro on the upstream
`ugv_description/urdf/bases/ugv_rover.xacro` every time the simulation starts. It removes
only the `<xacro:include>` of `urdf/gazebo/ugv_rover.gazebo` (Gazebo plugins), processes with
`use_gazebo:=true` so the camera frames match the Gazebo setup, drops the upstream
`<ros2_control>`, `<transmission>` and `<gazebo>` blocks, and appends
`resource/webots_extras.urdf`. The same URDF feeds `robot_state_publisher` and the driver,
so a change in `ugv_description` reaches the TF tree immediately (the PROTO, which holds the
physical model, needs regenerating, see below). Print it with `ros2 run ugv_webots print_urdf`.

**Webots on another machine.** When ROS runs in WSL, `webots_ros2` starts `webots.exe` on
Windows and the controller connects back over TCP. `webots_host.py` fixes the address
lookup of `webots_ros2` 2025.0.0 (see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)): default
gateway with WSL NAT networking, `127.0.0.1` with mirrored networking, the gateway (the Mac)
in the macOS VM. `UGV_WEBOTS_HOST` overrides it.

**Clock.** `/clock` comes from the `Ros2Supervisor` node that `WebotsLauncher` adds
(`ros2_supervisor=True`). Every node is started with `use_sim_time: true`.

## Topic and frame contract

Mirrors `ugv_gazebo/config/ros_gz_bridge.yaml` and the frames the Gazebo sensors use
(`ugv_description/urdf/gazebo/ugv_rover.gazebo`).

| Topic | Type | Frame | Rate (target) | Webots source |
|---|---|---|---|---|
| `/cmd_vel` (in) | `geometry_msgs/msg/Twist` | | | `diffdrive_controller` (remapped from `~/cmd_vel_unstamped`) |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` to `base_footprint` | 50 Hz | `diffdrive_controller` |
| `/tf` | `tf2_msgs/msg/TFMessage` | | | `diffdrive_controller` (`odom -> base_footprint`) + `robot_state_publisher` (joints) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | | | `robot_state_publisher` |
| `/clock` | `rosgraph_msgs/msg/Clock` | | 100 Hz (every 10 ms step) | `Ros2Supervisor` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `base_lidar_link` | 10 Hz | Lidar `ld19_lidar` via `scan_normalizer` |
| `/image_raw` | `sensor_msgs/msg/Image` (`bgra8`, 640x480) | `pt_camera_link` | 20 Hz | Camera `pt_camera` on the pan-tilt |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | `pt_camera_link` | 20 Hz | same |
| `/oak/image_raw` | `sensor_msgs/msg/Image` (`bgra8`, 640x480) | `3d_camera_link` | 20 Hz | Camera `oak_rgb` |
| `/oak/camera_info` | `sensor_msgs/msg/CameraInfo` | `3d_camera_link` | 20 Hz | same |
| `/oak/depth/image_raw` | `sensor_msgs/msg/Image` (`32FC1`, metres) | `3d_camera_link` | 20 Hz | RangeFinder `oak_depth` |
| `/oak/points` | `sensor_msgs/msg/PointCloud2` | `3d_camera_link` | 20 Hz | `depth_image_proc` `point_cloud_xyz_node` |
| `/joint_states` | `sensor_msgs/msg/JointState` | | 100 Hz | `joint_state_broadcaster` |
| `/pt_joint_position_controller/commands` (in) | `std_msgs/msg/Float64MultiArray` | | | `pt_joint_position_controller`, same name as upstream |

Simulator-only extras: `/oak/depth/camera_info`, `/ugv_webots/scan_raw` (raw Webots scan),
`/ugv_webots/ground_truth` (`nav_msgs/msg/Odometry`, frame `world`, the true pose, no TF).

Camera images are `bgra8` (Webots' native format) where Gazebo publishes `rgb8`. Consumers
using `cv_bridge` with a target encoding are unaffected.

TF tree (one publisher per transform):

```
map -> odom                                   SLAM or AMCL
odom -> base_footprint                        diffdrive_controller
base_footprint -> base_link -> base_lidar_link
                            -> oak_base -> 3d_camera_link (optical)
                            -> pt_base_link -> pt_link1 -> pt_link2 -> pt_camera_link
                            -> {left,right}_{up,down}_wheel_link      robot_state_publisher
```

### Sensor specifications

| Sensor | Spec in simulation | Notes |
|---|---|---|
| LD19 lidar | 360 degrees, 480 samples, 10 Hz, 0.2 to 12 m, noise 1 cm (sd), 1 cm resolution | Output layout copies the real `ldlidar` driver: `angle_min` 0, counter-clockwise, 480 bins, frame `base_lidar_link`. Gazebo used 360 samples at 20 Hz. The scan plane is 2 cm above the frame origin to clear the chassis top plate. |
| Pan-tilt camera | 640x480, horizontal FOV 1.02974 rad, 20 Hz, noise 0.007 | As Gazebo. Pan -3.14 to 3.14 rad, tilt -0.52 to 1.57 rad (positive is up), 1 rad/s. |
| OAK RGB | 640x480, FOV 1.02974 rad, 20 Hz | As Gazebo. |
| OAK depth | 640x480, FOV 1.02974 rad, 0.2 to 8 m, 20 Hz | As Gazebo's depth clip. |

## Robot model and regenerating the PROTO

`protos/UgvRover.proto` is **generated**. Do not edit it by hand: change
`scripts/generate_proto.py` and regenerate:

```bash
cd ~/ugv_rover
colcon build --symlink-install --packages-select ugv_description ugv_webots
source install/setup.bash
python3 src/ugv_main/ugv_webots/scripts/generate_proto.py
```

Pipeline: xacro (the same function the launch file uses), then urdf2webots (bundled with
`webots_ros2_importer`), then the following tuning, all applied by the script:

| Step | What and why |
|---|---|
| Inertia check | Every URDF mass and inertia is checked (positive, triangle inequality). All passed. The values are consistent with solid boxes and cylinders of each mesh's size (for example base 2.185 kg, 0.245 x 0.23 x 0.12 m box gives 0.0123 / 0.0135 / 0.0206 kg m^2 against 0.0123 / 0.0136 / 0.0206 in the URDF), so they are not vendor placeholders and **no mass or inertia value was changed**. Total robot mass is 3.07 kg. |
| Rigid bodies | urdf2webots makes the root `base_footprint` a Robot without physics and every fixed link a separate Solid. Webots then ignores the collisions of physics-less child Solids and the robot falls through the floor. The script groups links joined by fixed joints into one body, puts the combined mass, centre of mass and inertia (parallel axis theorem) on the body root, and turns the other links into `Pose` nodes. Bodies: main body 2.336 kg (base, lidar, OAK, pan-tilt base; centre of mass 6.5 cm above the floor), pan link 0.006 kg, tilt link with camera 0.081 kg, four wheels 0.163 kg each. |
| Meshes | All STL meshes are inlined as `IndexedFaceSet`, so the PROTO is one self-contained file. The macOS shared-folder mode copies only the files the world names directly. |
| Collisions | `ugv_rover_base.stl` reaches the ground (it includes the six-wheel side frames), so the chassis collides as two boxes that clear the wheels: 0.240 x 0.128 x 0.045 m between the wheels and a 0.245 x 0.230 x 0.040 m top plate. Lidar, OAK and pan-tilt parts collide as their mesh bounding boxes. Wheels keep the URDF cylinders (r 0.04 m, width 0.0425 m). |
| Wheels | `contactMaterial "ugv_wheel"`, motors limited to 40 rad/s (1.6 m/s) and 1.2 N m. |
| Sensors | Lidar, cameras and RangeFinder added at the frames listed above. |
| Header | Robot name `ugv_rover`, controller `<extern>`, `supervisor TRUE` (for the ground truth plugin). |

The script stops with an error if the upstream xacro changes in a way it does not expect.

## Drive and contact tuning

The drive reproduces the Gazebo DiffDrive: `diff_drive_controller` on four joints (two per
side), `wheel_separation` 0.17452 m, `wheel_radius` 0.04 m, limits 1.5 m/s, 3 m/s^2,
3.14 rad/s, 6 rad/s^2 (the Gazebo Harmonic plugin's values). Settled values:

| Setting | Value | Where |
|---|---|---|
| Wheel/floor friction | `ContactProperties { material1 "ugv_wheel" coulombFriction [0.3, 1.0, 1.0] bounce 0 softCFM 1e-05 }` | both worlds |
| `wheel_separation_multiplier` | 1.25 (effective track 0.218 m) | `resource/ros2_control.yaml` |
| Physics step | `basicTimeStep 10` ms, controller update 100 Hz | worlds, `ros2_control.yaml` |

How we got there (command: 360 degrees in place at 0.6 rad/s, `ros2 run ugv_webots motion_check`,
empty arena; "truth" is the real rotation from the ground truth plugin):

| Friction | Multiplier | Odometry | Truth |
|---|---|---|---|
| symmetric 0.7 | 1.0 | 360.6 deg | 214.2 deg |
| `[1.0, 0.3, 1.0]` | 1.0 | 360.3 deg | 0.4 deg (no rotation) |
| `[0.3, 1.0, 1.0]` | 1.0 | 360.6 deg | 288.4 deg |
| `[0.15, 1.0, 1.0]`, `[0.05, 1.0, 1.0]`, `[0.15, 1.0]`, floor 0.6 | 1.0 | 360.3 to 360.6 deg | 287.1 to 288.7 deg |
| `[0.3, 1.0, 1.0]` | 1.25 | 359.9 deg (0.3 rad/s), 360.9 deg (1.0 rad/s) | 360.7 deg, 358.7 deg |

* A four-wheel skid steer with an almost square footprint (wheelbase 0.171 m, track
  0.175 m) scrubs its wheels sideways when turning, so with isotropic friction the body
  turns only 60 percent of what the wheels command. It never stalled: the wheels always
  reached the commanded speed.
* Webots supports asymmetric friction on cylinders. The first coefficient turned out to act
  across the wheel, the second along the rolling direction (measured: swapping them stops
  rotation completely). A low lateral coefficient approximates the real six-wheel chassis,
  whose middle wheels barely scrub.
* The remaining 20 percent did not depend on the friction values, so it is calibrated in the
  controller with `wheel_separation_multiplier`, as for real skid steer robots. The
  `wheel_separation` value itself stays the Gazebo/URDF 0.17452 m.

Result after tuning: 1 m forward at 0.2 to 0.4 m/s is within 5 mm on both odometry and
ground truth with no lateral drift, and a full turn is within 1.3 degrees of the command,
with odometry within 2.2 degrees of the truth.

## Worlds and adding a world

| World | Content |
|---|---|
| `empty_arena.wbt` | 6 m x 6 m walled floor, three boxes (ahead at 1.5 m, left at 1.2 m, back right). For tuning and sensor checks. |
| `apartment.wbt` | About 9.9 m x 6.6 m apartment with kitchen, dining, office and living areas. Derived from Webots' sample `projects/samples/environments/indoor/worlds/apartment.wbt` (Apache 2.0): sample robots removed, rover added at (-2.2, -2.9), follow camera. Default world. |

To add a world:

1. Copy `empty_arena.wbt` (or any Webots sample world) into `ugv_webots/worlds/`.
2. Keep the header line `#VRML_SIM R2025a utf8`, add
   `EXTERNPROTO "../protos/UgvRover.proto"` and one robot:
   `UgvRover { translation x y 0 name "ugv_rover" }` (the name must stay `ugv_rover`).
3. Set `basicTimeStep 10` and copy the `contactProperties` block from `empty_arena.wbt`
   into `WorldInfo`.
4. Use stock objects through `EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/..."`
   URLs. Avoid local textures and meshes next to the world; the macOS shared-folder mode only
   copies files the world references directly.
5. Put comments only at the top of the file. A comment inside a node makes Webots R2025a
   hang while loading.
6. `colcon build --symlink-install --packages-select ugv_webots` (new files need a build),
   then `ros2 launch ugv_webots ugv_webots.launch.py world:=my_world.wbt`.

To build a map for Nav2 in a new world: run `gmapping.launch.py world:=my_world.wbt`, drive
around, save with `map_saver_cli`, and pass `map:=/path/to/map.yaml` to `nav.launch.py`. AMCL
starts at the map origin, which is where the robot was when mapping started.

## Performance

Real-time factor measured with `ros2 run ugv_webots rtf_meter` (WSL2 Ubuntu 22.04, Webots
R2025a on Windows 11, all sensors active):

| Scene | RTF |
|---|---|
| Empty arena, headless | 0.887 |
| Apartment + gmapping, headless | 0.932 |
| Apartment, Webots GUI + RViz | 0.885 |

The sensors (two 640x480 cameras and a 640x480 depth image at 20 Hz) dominate; rates scale
with the RTF (for example `/scan` 8.8 to 9.0 Hz for a 10 Hz target). For more speed:
`use_oak_points:=false`, `headless:=true`, or `mode:=fast`.

**Gazebo comparison: not measured.** The upstream Gazebo Classic bringup did not start the
robot in our WSL setup (first the `gazebo_ros2_control` and sensor plugins were missing until
`ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control` were installed, then
`spawn_entity` failed with `Service /spawn_entity unavailable`). Fixing upstream's Gazebo
setup was out of scope.

## Checking the simulator

Run after changing the model, the worlds, the controllers or after an upstream merge.
The quick automated version is the sim smoke test (`colcon test --packages-select ugv_webots`,
see [CI.md](CI.md)); the manual checks below go further:

```bash
ros2 launch ugv_webots ugv_webots.launch.py world:=empty_arena.wbt headless:=true
# in another terminal:
ros2 control list_controllers            # joint_state_broadcaster, diffdrive_controller,
                                         # pt_joint_position_controller: all active
ros2 topic hz /scan                      # about 10 Hz (times the RTF)
ros2 topic hz /clock                     # about 100 Hz (times the RTF)
ros2 topic info /tf -v | grep "Node name"  # only diffdrive_controller and robot_state_publisher
ros2 run tf2_tools view_frames           # one tree from odom
ros2 run ugv_webots motion_check         # forward and rotation errors, see the tuning table
ros2 run ugv_webots rtf_meter
```

Recorded results on 2026-10-16 (empty arena unless noted):

| Check | Result |
|---|---|
| `colcon build` of the whole workspace | 31 packages, 0 failures |
| Topic rates (NAT networking) | `/clock` 94, `/odom` 47, `/tf` 62, `/scan` 8.8, `/image_raw` 16.9, `/camera_info` 16.7, `/oak/image_raw` 16.8, `/oak/camera_info` 16.8, `/oak/depth/image_raw` 14.8, `/oak/points` 14.8, `/joint_states` 92 Hz |
| Topic rates (mirrored networking) | `/scan` 8.96, `/clock` 92.4, `/odom` 43.4, `/image_raw` 16.7, `/oak/points` 16.3 Hz |
| TF | one tree; `/tf` publishers: `diffdrive_controller`, `robot_state_publisher`; `/odom`: `diffdrive_controller` only |
| Lidar geometry | box ahead read 1.320 m (expected 1.318), box left 1.050 m (expected 1.051), wall right 3.000 m |
| `motion_check` | forward 1.000 m commanded: 1.000 odometry, 1.000 truth; 360 degrees: 360.3 odometry, 359.7 truth (mirrored); apartment: 1.002 m, 359.3 degrees truth |
| Pan-tilt | commands `[0.8, 0.4]`, `[-0.6, 1.2]`, `[0, 0]` reached exactly |
| gmapping (apartment) | 18-waypoint loop, map saved as `maps/apartment`, straight walls, loop closed |
| Nav2 + AMCL (apartment) | goal across the apartment SUCCEEDED in 49 s, robot 4.7 cm from the goal; second goal around the partition wall SUCCEEDED in 85 s, 15.7 cm (tolerance 25 cm) |
| Cartographer, slam_toolbox, `slam_nav` | `/map` published, `map -> base_footprint` resolves, no errors |
| Visual check | Webots robot, RViz lidar and both cameras confirmed by a person |

## Known gaps versus the real robot

* **Four wheels, not six.** The URDF (and Gazebo) model four wheels. The real rover has six;
  the asymmetric friction and `wheel_separation_multiplier` approximate its turning, but
  slip on different floors is not modelled.
* **No IMU and no EKF.** On the robot, `/odom` comes from `robot_localization` fusing the
  wheels with the IMU. In simulation `/odom` comes straight from `diff_drive_controller`
  and there is no `/imu/data`. Anything that needs the IMU will not work in simulation yet.
* **No base driver topics.** `ugv_bringup` topics (`/odom/odom_raw`, `/ugv/voltage`,
  `/ugv/led_ctrl`, `/ugv/pt_steady_ctrl`, the serial JSON protocol) are not simulated. The
  pan-tilt is driven by `pt_joint_position_controller`, as in Gazebo; the real robot reads
  `/joint_states` instead.
* **Lidar.** No intensities, no crop of the real driver's 225 to 315 degree sector (the
  pan-tilt mast, 3 cm behind the lidar, reads `inf` in a 76 degree sector behind the robot
  instead), ideal Gaussian noise.
* **Camera frames are not optical.** `/image_raw` is stamped `pt_camera_link` (x forward),
  as in Gazebo. Projecting points into that image needs an extra rotation. The OAK frame
  `3d_camera_link` is optical and the point cloud is correct in it.
* **Image encoding** `bgra8` instead of the real cameras' formats; no compressed image topics,
  no stereo pair, no OAK neural network outputs.
* **Contact materials.** One floor friction for every surface; carpets and thresholds behave
  like the floor.
* **Not tested here:** native Linux Webots, the macOS setup ([setup/MACOS.md](setup/MACOS.md)),
  and the robot's other models (`ugv_beast`, `rasp_rover`).
