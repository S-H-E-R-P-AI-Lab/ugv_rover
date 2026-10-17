# UGV Rover workspace (S-H-E-R-P AI Lab)

[![CI](https://github.com/S-H-E-R-P-AI-Lab/ugv_rover/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/S-H-E-R-P-AI-Lab/ugv_rover/actions/workflows/ci.yml)

ROS 2 Humble workspace for the **Waveshare UGV Rover** (6-wheel skid steer, LD19 lidar,
pan-tilt USB camera, OAK-D depth camera), with a **Webots simulator** that behaves like the
real robot.

This is our fork of [waveshareteam/ugv_ws](https://github.com/waveshareteam/ugv_ws)
(branch `ros2-humble-develop-251125`). Waveshare's packages drive the real robot; we added
`src/ugv_main/ugv_webots` (the simulator) and the documentation below. Webots replaces Gazebo
as our default simulator (`ugv_gazebo` is still there, untouched).

New to ROS? Read the [ROS 2 primer](docs/ROS2_PRIMER.md) first (15 minutes).

## Quick start

| Your computer | Guide | Status |
|---|---|---|
| Windows 11 | [docs/setup/WINDOWS_WSL2.md](docs/setup/WINDOWS_WSL2.md): Ubuntu 22.04 in WSL2, Webots on Windows | Tested |
| Ubuntu 22.04 | [docs/setup/LINUX.md](docs/setup/LINUX.md) | ROS side tested; native Webots install not tested |
| Mac (Apple Silicon) | [docs/setup/MACOS.md](docs/setup/MACOS.md): Webots on macOS, ROS in an Ubuntu VM | Not tested yet |

In short, on Ubuntu 22.04 (or in WSL or the Mac's VM), with Webots R2025a installed:

```bash
sudo apt update && sudo apt install -y git git-lfs && git lfs install
git clone https://github.com/S-H-E-R-P-AI-Lab/ugv_rover.git ~/ugv_rover
cd ~/ugv_rover && bash scripts/setup_ubuntu.sh
# open a new terminal, then:
ros2 launch ugv_webots ugv_webots.launch.py use_rviz:=true
```

## What you can do in simulation

```bash
ros2 launch ugv_webots ugv_webots.launch.py use_rviz:=true   # robot in an apartment
ros2 run ugv_tools keyboard_ctrl                              # drive it (second terminal)
ros2 launch ugv_webots gmapping.launch.py                     # build a map
ros2 launch ugv_webots nav.launch.py                          # navigate (click "2D Goal Pose" in RViz)
```

The simulator publishes the same topics and frames as the robot setup (`/scan`, `/odom`,
`/tf`, `/image_raw`, `/oak/...`, `/cmd_vel`), so code tested in simulation runs on the robot.
Details: [docs/SIMULATOR.md](docs/SIMULATOR.md).

## Documentation

| Page | For |
|---|---|
| [docs/ROS2_PRIMER.md](docs/ROS2_PRIMER.md) | ROS 2 basics with this robot, daily commands |
| [docs/setup/](docs/setup/) | Installation per operating system |
| [docs/SIMULATOR.md](docs/SIMULATOR.md) | How the Webots simulation works, topics and frames, adding worlds, tuning, known gaps |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Problems we hit and how to fix them |
| [docs/CI.md](docs/CI.md) | What GitHub Actions checks on each pull request, running the checks locally |
| [docs/UPSTREAM_CHANGES.md](docs/UPSTREAM_CHANGES.md) | What we changed in the files that came from Waveshare, and why |
| [docs/UPSTREAM_README.md](docs/UPSTREAM_README.md) | Waveshare's original README, and from there their tutorials for the real robot (bringup, mapping, navigation, vision, web app) |

## Repository layout

```
src/ugv_main/   robot packages: description, bringup, slam, nav, tools, vision, voice, web app,
                ugv_gazebo (Gazebo), ugv_webots (Webots, ours)
src/ugv_else/   third party packages: cartographer, gmapping, ldlidar, teb, emcl2, ...
docs/           documentation (ours and Waveshare's)
scripts/        setup_ubuntu.sh
build_first.sh  Waveshare's install and build script (used on the robot)
```

## Branches

* `main`: our development branch.
* Work on feature branches and open pull requests into `main`.

## Origin

This project started from Waveshare's [ugv_ws](https://github.com/waveshareteam/ugv_ws)
(branch `ros2-humble-develop-251125`, November 2025). It is developed independently: we do
not sync with it or send changes back.
