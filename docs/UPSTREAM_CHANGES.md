# Changes to upstream files

This repository started from Waveshare's `ros2-humble-develop-251125` branch
(https://github.com/waveshareteam/ugv_ws) and is developed independently; we do not sync
with it. We still keep Waveshare's packages untouched where possible, so their code stays
easy to compare with the original. Everything we added lives in new files:
`src/ugv_main/ugv_webots/`, `docs/ROS2_PRIMER.md`, `docs/SIMULATOR.md`,
`docs/TROUBLESHOOTING.md`, `docs/setup/`, `docs/UPSTREAM_CHANGES.md`, `docs/CI.md`, `scripts/`,
`.github/` (CI workflows and the pull request template).

This page lists every change to a file that exists upstream, and why.

| File | Change | Reason |
|---|---|---|
| `README.md` | Replaced by our landing page. The original is kept verbatim (only relative links adjusted) as `docs/UPSTREAM_README.md`. | The team needs an entry point for the simulator and per OS setup. |
| `.gitignore` | Extended (Python caches, Webots generated files, editor files, Windows metadata). Upstream entries kept. | Keep build output and Webots temporary files out of commits. |
| `build_first.sh` | `WS=/home/ws/ugv_ws` became `WS="${UGV_WS:-/home/ws/ugv_ws}"`. Added `ros-humble-webots-ros2` and `ros-humble-depth-image-proc` to the apt list. Builds `ugv_webots` when `ros-humble-webots-ros2` is installed. | The script hard codes the path used inside Waveshare's Docker image. Setting `UGV_WS` lets us run it on a laptop. The default is unchanged, so it behaves the same on the robot. |
| `build_common.sh` | Same `UGV_WS` override, `ugv_webots` added to the package menu. | Same as above. |
| `src/ugv_main/ugv_description/urdf/bases/ugv_rover.xacro` | New argument `kit` (default `$UGV_KIT`, else `jetson_orin`) that moves the lidar, OAK and pan-tilt joints to the positions measured on Waveshare's STEP of the Jetson Orin kit; `UGV_KIT=upstream` gives the original URDF. | The upstream frames were up to 19 mm off for our kit, on the robot and in simulation. Evidence: [MODEL_CHANGES.md](MODEL_CHANGES.md). |
| `src/ugv_main/ugv_description/urdf/sensors/pt.xacro` | Optional macro parameters `pt_base_link_joint_pose_x` and `pan_joint_pose_z`, defaults equal to the old values. | Lets `ugv_rover` set them without changing `rasp_rover` and `ugv_beast`. |

No other upstream ROS package (`src/ugv_main/*` other than `ugv_webots` and the two
`ugv_description` files above, `src/ugv_else/*`) is modified. `ugv_gazebo` is untouched: the Webots robot
model is generated from `ugv_description` at build and launch time (see
[SIMULATOR.md](SIMULATOR.md)), and the simulator reuses the SLAM and Nav2 configuration
files of `ugv_slam` and `ugv_nav` as they are.

## Upstream quirks we work around (not changed upstream)

| Upstream behaviour | How ugv_webots handles it |
|---|---|
| `ugv_rover.xacro` only produces the simulation camera frames (`oak_base`, optical `3d_camera_link`) when `use_gazebo:=true`, and that flag also pulls in the Gazebo plugin file. | `ugv_webots/description.py` removes the Gazebo include from a temporary copy of the xacro, then processes it with `use_gazebo:=true`. |
| The pan-tilt `<ros2_control>` block in `pt.xacro` selects a Gazebo or mock hardware plugin. | Dropped from the generated URDF and replaced by `resource/webots_extras.urdf`. |
| The chassis mesh `ugv_rover_base.stl` reaches the ground (it contains the side frames and six wheels), so it cannot be used for collisions. | The generated PROTO uses two collision boxes that clear the wheels. |
| The pan-tilt camera frame `pt_camera_link` and the Gazebo OAK frame are not optical frames (x forward), though images are stamped with them. | Kept as is for compatibility, documented in [SIMULATOR.md](SIMULATOR.md#known-gaps-versus-the-real-robot). |
| `rf2o_laser_odometry/package.xml` depends on `cmake_modules`, a ROS 1 package with no Humble rosdep rule. | Harmless: rosdep reports it, the package builds fine without it. |
