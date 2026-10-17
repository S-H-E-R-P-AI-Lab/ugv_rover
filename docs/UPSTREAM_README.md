![GitHub top language](https://img.shields.io/github/languages/top/waveshareteam/ugv_ws)
![GitHub language count](https://img.shields.io/github/languages/count/waveshareteam/ugv_ws)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/waveshareteam/ugv_ws/ros2-humble-develop-251125)

# ROS2 for WaveShare UGV

**ugv_ws** is a **ROS2 Humble** colcon workspace for **WaveShare UGV**. It connects **RViz2**, **SLAM**, and **Nav2** to real hardware over serial, with optional vision, voice, web UI, and Gazebo simulation.

## Documentation

Tutorials live in [`docs/`](./). On GitHub, open any `.md` file to read the rendered preview — no Wiki or separate site required.

| | |
|---|---|
| **Browse on GitHub** | [docs/](https://github.com/waveshareteam/ugv_ws/tree/ros2-humble-develop-251125/docs) — start with [index.md](https://github.com/waveshareteam/ugv_ws/blob/ros2-humble-develop-251125/docs/index.md) |
| **Optional local site** | `pip install -r docs/requirements.txt` then `mkdocs serve` (sidebar nav + copy buttons; same source files) |

### Getting Started

| Section | Page |
|---------|------|
| Overview | [docs/index.md](index.md) |
| ROS2 Basics | [docs/ros2_basics.md](ros2_basics.md) |
| UGV Basics | [docs/ugv_basics.md](ugv_basics.md) |
| Installation | [docs/installation.md](installation.md) |

### Tutorials

| Section | Page |
|---------|------|
| Robot Description | [docs/description.md](description.md) |
| Hardware Driver | [docs/bringup.md](bringup.md) |
| Keyboard & Gamepad Control | [docs/teleoperation.md](teleoperation.md) |
| LiDAR Interaction | [docs/lidar.md](lidar.md) |
| Vision | [docs/vision.md](vision.md) |
| Mapping | [docs/mapping.md](mapping.md) |
| Web App | [docs/web_app.md](web_app.md) |
| Navigation | [docs/navigation.md](navigation.md) |
| Behavior Command Control | [docs/behavior_ctrl.md](behavior_ctrl.md) |
| Experimental | [docs/experimental.md](experimental.md) |
| Gazebo | [docs/gazebo.md](gazebo.md) |

Package layout: [docs/packages.md](packages.md)

Suggested order: [docs/index.md — Suggested reading order](index.md#suggested-reading-order).

**Factory image:** Docker + SSH port **23** (`root`/`ws`). **Camera:** `:8889/cam/`. **Web AI (experimental):** `:5000`.

## Quick start

**Ubuntu 22.04** + **ROS2 Humble**. Factory images: run `bash ros2.sh` and SSH into the container.

Clone **on the host** (install **git-lfs** and pull weights **now**). Then run **`build_first.sh` inside Docker** — that script does not pull LFS, and running `git lfs pull` as **root** in the container can break host file permissions.

```bash
sudo apt install -y git git-lfs
git lfs install
git clone -b ros2-humble-develop-251125 https://github.com/waveshareteam/ugv_ws.git
cd ugv_ws
git lfs pull
```

Inside the container:

```bash
sudo bash build_first.sh
```

`build_first.sh` installs dependencies, prompts for `UGV_MODEL` and `LDLIDAR_MODEL`, and runs `colcon build`. Gazebo is optional (default: skip). Details: [Installation](installation.md).

### Model settings

| Hardware | `UGV_MODEL` | `LDLIDAR_MODEL` |
|----------|-------------|-----------------|
| UGV Rover (6-wheel 4WD) | `ugv_rover` | `ld06` / `ld19` / `stl27l` |
| UGV Beast (tracked) | `ugv_beast` | per your kit |
| RaspRover (4WD) | `rasp_rover` | per your kit |

See [UGV Basics](ugv_basics.md) for AI Kit vs ROS2 Kit naming.

### Typical real-robot workflow

1. **T0** — robot stack: bringup, SLAM, or Nav2 (each is one launch — see [Typical paths](index.md#typical-paths))
2. **T1** — teleop while mapping (`keyboard_ctrl` / gamepad), Web App, or **`explore_lite`** — only when the workflow needs a second terminal

Example — teleop only: **T0** `bringup_lidar.launch.py` · **T1** `keyboard_ctrl`. Example — mapping: **T0** `slam_toolbox.launch.py` · **T1** `keyboard_ctrl`.

Simulation: **VM or desktop only** — see [Gazebo](gazebo.md); add `use_sim_time:=true` on SLAM/Nav launches.

## Useful resources

- [Nav2 Setup Guides](https://navigation.ros.org/setup_guides/index.html)
- [UGV Rover PI ROS2 Wiki](https://www.waveshare.com/wiki/UGV_Rover_PI_ROS2)
- [UGV Beast PI ROS2 Wiki](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2)
