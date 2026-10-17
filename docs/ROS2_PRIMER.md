# ROS 2 in 15 minutes, using our rover

This is the minimum you need to work with this repo. It uses the UGV Rover and the Webots
simulator as the running example. You do not need to read the official tutorials first, but
they are good next steps (links at the end).

## 1. What ROS 2 is

ROS 2 is not an operating system. It is a set of libraries and tools that let many small
programs talk to each other on one machine or across a network. We use **ROS 2 Humble**
on **Ubuntu 22.04**. Every terminal that uses ROS must first load it:

```bash
source /opt/ros/humble/setup.bash
```

## 2. Nodes

A **node** is one running program with one job. When the simulator runs, you get nodes like:

| Node | Job |
|---|---|
| `webots_controller_ugv_rover` | Talks to Webots: reads the simulated sensors, drives the simulated motors |
| `robot_state_publisher` | Knows the robot's shape (from the URDF) and publishes where each part is |
| `scan_normalizer` | Reformats the simulated lidar scan to look exactly like the real LD19 |
| `slam_gmapping` | Builds a map from the lidar while you drive |

```bash
ros2 node list
```

## 3. Topics

Nodes talk by publishing **messages** on named **topics**. Anyone can publish, anyone can
subscribe. Each topic has one message type. The rover's main topics:

| Topic | Type | What it carries |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Drive command: forward speed (m/s) and turn rate (rad/s) |
| `/odom` | `nav_msgs/msg/Odometry` | Where the wheels think the robot is |
| `/scan` | `sensor_msgs/msg/LaserScan` | One 360 degree lidar sweep (480 distances) |
| `/image_raw` | `sensor_msgs/msg/Image` | Pan-tilt camera picture |
| `/oak/points` | `sensor_msgs/msg/PointCloud2` | 3D points from the OAK depth camera |
| `/tf` | `tf2_msgs/msg/TFMessage` | Positions of all coordinate frames (see TF below) |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time (only in simulation) |

The simulator and the real robot publish **the same topics**. That is the whole point: code
that works in simulation works on the robot.

```bash
ros2 topic list                    # what exists
ros2 topic info /scan              # type, publishers and subscribers
ros2 topic echo /odom --once       # print one message
ros2 topic hz /scan                # how often it is published (about 10 Hz)
```

Make the robot drive in a circle by publishing yourself (Ctrl+C to stop, it stops after 0.5 s):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

## 4. Packages, workspaces and colcon

Code is organised in **packages** (a folder with a `package.xml`). This repo is a
**workspace**: all packages live under `src/`. You build them with **colcon**:

```bash
cd ~/ugv_rover
colcon build --symlink-install       # builds everything into build/ and install/
source install/setup.bash            # makes our packages visible to ros2 commands
```

`--symlink-install` means edits to Python files, launch files and worlds take effect without
rebuilding. You still need to rebuild after adding new files or changing C++ code.

Packages you will meet: `ugv_description` (robot model), `ugv_bringup` (real robot drivers),
`ugv_slam` and `ugv_nav` (mapping and navigation), `ugv_tools` (keyboard and joystick
control), `ugv_webots` (the simulator).

## 5. Launch files

Starting ten nodes by hand is tedious, so a **launch file** starts a whole set with the right
settings. Arguments are passed as `name:=value`:

```bash
ros2 launch ugv_webots ugv_webots.launch.py world:=empty_arena.wbt use_rviz:=true
ros2 launch ugv_webots ugv_webots.launch.py --show-args      # list the arguments
```

`ros2 run` starts a single node:

```bash
ros2 run ugv_tools keyboard_ctrl      # drive with the keyboard (needs its own terminal)
```

## 6. TF: where things are

Every sensor and body part has a **coordinate frame**. TF keeps track of how frames relate
over time. The rover's tree:

```
map -> odom -> base_footprint -> base_link -> base_lidar_link
                                           -> 3d_camera_link (OAK)
                                           -> pt_base_link -> pt_link1 -> pt_link2 -> pt_camera_link
                                           -> four wheel links
```

* `base_footprint` is the robot's centre on the floor. `base_link` is the chassis.
* `odom -> base_footprint` comes from wheel odometry. It is smooth but drifts.
* `map -> odom` comes from SLAM or localization (AMCL). It corrects the drift.
* Sensor messages say which frame they were measured in (`header.frame_id`), so any node
  can convert a lidar point into map coordinates.

```bash
ros2 run tf2_ros tf2_echo map base_footprint      # where is the robot on the map?
ros2 run tf2_tools view_frames                    # writes frames_<date>.pdf with the tree
```

## 7. Simulation time

The simulator may run faster or slower than real time. It publishes its own clock on
`/clock`, and every node in a simulation must use it, or timestamps will not match and TF
lookups fail ("extrapolation into the future" errors). Our launch files do this for you. When you
run your own node against the simulator, add:

```bash
ros2 run my_package my_node --ros-args -p use_sim_time:=true
```

## 8. RViz

RViz is the 3D viewer. It shows what the robot *thinks*: the map, the lidar points, the
camera images, the planned path. `use_rviz:=true` on our launch files opens it with a ready
made layout. Click **2D Goal Pose** in the toolbar to send a navigation goal.

## 9. Commands you will use every day

```bash
# environment (put the first two in ~/.bashrc, the setup scripts do this for you)
source /opt/ros/humble/setup.bash
source ~/ugv_rover/install/setup.bash

# build
cd ~/ugv_rover && colcon build --symlink-install
colcon build --symlink-install --packages-select ugv_webots     # one package only

# simulate
ros2 launch ugv_webots ugv_webots.launch.py use_rviz:=true      # just the robot
ros2 launch ugv_webots gmapping.launch.py                       # make a map
ros2 launch ugv_webots nav.launch.py                            # navigate on a map
ros2 run ugv_tools keyboard_ctrl                                # drive (separate terminal)

# inspect
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run rqt_graph rqt_graph                                    # who talks to whom

# save a map (while SLAM runs)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p use_sim_time:=true
```

## 10. Where to go next

* Official tutorials (the "Beginner: CLI tools" set takes about two hours):
  https://docs.ros.org/en/humble/Tutorials.html
* Nav2 concepts: https://docs.nav2.org/concepts/index.html
* This repo: [SIMULATOR.md](SIMULATOR.md) for how the simulator works, and
  [TROUBLESHOOTING.md](TROUBLESHOOTING.md) when something breaks.
* Waveshare's own tutorials for the real robot: [UPSTREAM_README.md](UPSTREAM_README.md) and the
  other files in this folder.
