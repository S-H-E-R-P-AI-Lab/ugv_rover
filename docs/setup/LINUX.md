# Setup on Ubuntu 22.04 (native Linux)

Time: about 45 minutes. Everything runs on one machine: ROS 2 Humble, this workspace and
Webots.

> **Tested status.** The ROS side (`scripts/setup_ubuntu.sh`, build, launches) was tested on
> Ubuntu 22.04.5 under WSL2, which is the same Ubuntu userland. The native Linux Webots
> install below follows the official Cyberbotics instructions but was not run on a native
> Linux desktop by us. Please report back (or fix this page) if a step differs.

## 1. Requirements

* **Ubuntu 22.04** (Jammy). ROS 2 Humble has no packages for 24.04. Check with `lsb_release -a`.
* A GPU with working OpenGL drivers (NVIDIA proprietary driver, or Mesa for Intel and AMD).
  Webots needs it to render the cameras and the lidar.
* About 25 GB of free disk.

## 2. Install Webots R2025a

Use the Cyberbotics apt repository (official procedure,
https://cyberbotics.com/doc/guide/installation-procedure):

```bash
sudo mkdir -p /etc/apt/keyrings
sudo wget -q -O /etc/apt/keyrings/Cyberbotics.asc https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update
sudo apt install webots
```

This installs Webots in `/usr/local/webots`, where `webots_ros2` finds it automatically.
Check: `webots --version` should print `Webots version: R2025a`. If you installed Webots
elsewhere (tarball or snap), add `export WEBOTS_HOME=/path/to/webots` to `~/.bashrc`.

## 3. Get the code and install everything

```bash
sudo apt update && sudo apt install -y git git-lfs gh
git lfs install
gh auth login                                                # authenticate with GitHub (HTTPS)
cd ~
gh repo clone S-H-E-R-P-AI-Lab/ugv_rover
cd ugv_rover
bash scripts/setup_ubuntu.sh
```

`setup_ubuntu.sh` installs ROS 2 Humble and every package the repo needs, downloads the
Git LFS model files, runs rosdep, builds the workspace and adds the environment to
`~/.bashrc`. It is safe to run again. Open a **new terminal** when it finishes.

If you prefer to do it by hand, the script is short and commented; it follows
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html and then
`rosdep install --from-paths src --ignore-src -y` and `colcon build --symlink-install`.

## 4. First simulation

```bash
ros2 launch ugv_webots ugv_webots.launch.py use_rviz:=true
```

The first start of a world downloads furniture and textures (a few minutes, cached
afterwards). In a second terminal:

```bash
ros2 run ugv_tools keyboard_ctrl
```

Next: [the ROS 2 primer](../ROS2_PRIMER.md), then [SIMULATOR.md](../SIMULATOR.md) for mapping
and navigation.

## Talking to the real robot

On native Linux, ROS 2 discovery works on the LAN out of the box. Make sure your machine
and the rover use the same `ROS_DOMAIN_ID` (unset means 0) and that no firewall blocks UDP
multicast (`sudo ufw status`).
