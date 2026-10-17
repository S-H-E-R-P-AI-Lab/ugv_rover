# Setup on macOS (Apple Silicon)

> **Not tested on a Mac.** This page was written and checked from a Windows machine. What was
> verified: the approach is the one in the official ROS 2 Humble Webots guide for macOS;
> every ROS package we need exists for arm64 in the ROS apt repository (checked against the
> package index); the ROS side of the steps (`setup_ubuntu.sh`, the launch files) works on
> Ubuntu 22.04; and `ugv_webots` handles the macOS shared-folder mode in code. What was **not**
> verified: the UTM VM itself, Webots on macOS, and the connection between them. Webots R2025a
> officially supports macOS 12 to 14; newer macOS versions are not listed by Cyberbotics.
> The first teammate to follow this, please fix anything that differs and mark it tested.

## The approach

ROS 2 Humble does not run natively on macOS in a practical way. The officially recommended
setup (https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-MacOS.html)
splits the work:

```
macOS (Apple Silicon)                          UTM virtual machine: Ubuntu 22.04 arm64
+----------------------------------+           +--------------------------------------+
| Webots R2025a (native, Metal GPU)|  TCP 2000 | ROS 2 Humble, ~/ugv_rover, RViz      |
| local_simulation_server.py       |<----------| ros2 launch ugv_webots ...           |
|                                  |  TCP 1234 |                                      |
|                                  |<----------| robot controller                     |
| ~/webots_shared  <== shared folder ==>  ~/webots_shared                          |
+----------------------------------+           +--------------------------------------+
```

* Webots runs natively on the Mac, so it gets the GPU.
* ROS runs in an Ubuntu 22.04 VM. When you launch, `webots_ros2` copies the world and the
  robot PROTO into the shared folder and asks a small server on the Mac to start Webots on it.
* The robot controller in the VM then connects to Webots over TCP. `ugv_webots` finds the
  Mac at the VM's default gateway.

We picked a **UTM VM** over Docker because it is the documented `webots_ros2` route, the
shared folder works the way `webots_ros2` expects, and RViz and other GUI tools run in the VM
desktop without extra X11 setup. Our robot PROTO is a single self-contained file on purpose,
because the shared-folder mode copies only files the world names directly.

Time: about 1.5 hours. You need about 60 GB of free disk and 16 GB of RAM is recommended.

## 1. Install Webots on the Mac

1. Download https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a.dmg
2. Open it and drag **Webots** into **Applications**.
3. Start Webots once from Applications (right click, Open, the first time) and accept the
   security prompts. Quit it again.

## 2. Create the shared folder on the Mac

The path **must not contain spaces** (the launch command is split on spaces):

```bash
mkdir -p ~/webots_shared
```

## 3. Create the Ubuntu 22.04 VM in UTM

1. Install UTM from https://mac.getutm.app/ (free download) and open it.
2. Download the Ubuntu 22.04 server image for ARM:
   https://cdimage.ubuntu.com/releases/jammy/release/ubuntu-22.04.5-live-server-arm64.iso
3. In UTM: **Create a New Virtual Machine**, **Virtualize**, **Linux**.
   * Boot ISO image: the file you downloaded.
   * Memory: 8 GB or more. CPU cores: 4 or more. Leave hardware OpenGL acceleration off (as
     the official guide says).
   * Storage: 64 GB.
   * **Shared Directory**: choose `~/webots_shared` from step 2.
   * Network: keep the default **Shared Network**.
4. Start the VM and install Ubuntu Server with the defaults. Pick a username (below we
   assume `ubuntu`). When it finishes, shut the VM down, remove the ISO from the CD/DVD drive
   in the VM settings, and start it again.
5. Log in and add a desktop (needed for RViz):
   ```bash
   sudo apt update && sudo apt install -y ubuntu-desktop-minimal spice-vdagent
   sudo reboot
   ```

## 4. Mount the shared folder in the VM

In a terminal in the VM:

```bash
mkdir -p ~/webots_shared
sudo mount -t 9p -o trans=virtio share ~/webots_shared -oversion=9p2000.L
echo "share $HOME/webots_shared 9p trans=virtio,version=9p2000.L,rw,_netdev,nofail 0 0" | sudo tee -a /etc/fstab
```

Check: create a file in `~/webots_shared` in the VM and see it appear in Finder.

Tell `webots_ros2` about the folder (replace `yourname` with your Mac username):

```bash
echo 'export WEBOTS_SHARED_FOLDER=/Users/yourname/webots_shared:/home/ubuntu/webots_shared' >> ~/.bashrc
```

The format is `<path on the Mac>:<path in the VM>`.

## 5. Get the code and install everything (in the VM)

```bash
sudo apt update && sudo apt install -y git git-lfs
git lfs install
cd ~
git clone https://github.com/S-H-E-R-P-AI-Lab/ugv_rover.git
cd ugv_rover
bash scripts/setup_ubuntu.sh
```

This installs ROS 2 Humble and everything the repo needs (all available for arm64), builds
the workspace and sets up `~/.bashrc`. Expect 30 to 60 minutes. Open a new terminal afterwards.

## 6. Start the simulation server on the Mac

In the Mac's Terminal:

```bash
mkdir -p ~/webots_server && cd ~/webots_server
curl -fLO https://raw.githubusercontent.com/cyberbotics/webots-server/main/local_simulation_server.py
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
```

It prints `Waiting for connection on port 2000...`. Leave it running. If macOS asks whether
Python (and later Webots) may accept incoming network connections, allow it: the VM
connects to port 2000 and the robot controller to port 1234. (`python3` comes with the Xcode
command line tools; macOS offers to install them the first time.)

## 7. First simulation (in the VM)

```bash
ros2 launch ugv_webots ugv_webots.launch.py use_rviz:=true
```

Webots opens on the Mac, RViz in the VM. The log line
`Robot controller will connect to Webots at <ip>:1234` shows the Mac's address as seen from
the VM (UTM's shared network usually makes it `192.168.64.1`). If it is wrong, set it:
`UGV_WEBOTS_HOST=<mac ip> ros2 launch ...`.

The first start of a world downloads furniture and textures; if Webots stays empty for more
than 3 minutes, stop and relaunch, or install the offline asset cache (see
[TROUBLESHOOTING.md](../TROUBLESHOOTING.md#webots-shows-nothing-for-minutes-no-error-clock-never-appears-seen)).
The cache folder on macOS is `~/Library/Caches/Cyberbotics/Webots`.

Drive from a second VM terminal with `ros2 run ugv_tools keyboard_ctrl`. Next steps:
[the ROS 2 primer](../ROS2_PRIMER.md) and [SIMULATOR.md](../SIMULATOR.md).

## Talking to the real robot

UTM's Shared Network puts the VM behind NAT, so ROS 2 discovery cannot see the rover on your
Wi-Fi. A **Bridged** network gives the VM its own address on the LAN, but then the default
gateway is your router, not the Mac, and the simulator cannot find Webots
(`webots_ros2`'s launcher always uses the gateway). Suggested setup (untested): keep the
Shared Network adapter for simulation and add a second, Bridged adapter for the robot in the
VM settings, making sure the default route stays on the shared adapter (`ip route`).

## If something is off

* `WARNING: Unable to start Webots. Please start the local simulation server...`: the server
  from step 6 is not running, or the Mac firewall blocks port 2000.
* `FAIL: The world file ... doesn't exist`: the shared folder is not mounted in the VM, or
  `WEBOTS_SHARED_FOLDER` has the wrong Mac path (check spelling and that it has no spaces).
* Webots starts but the robot never moves and the controller waits: port 1234 is blocked,
  or the address in the log line is wrong (use `UGV_WEBOTS_HOST`).
* RViz is slow: it renders in software inside the VM. Close panels you do not need, or use
  `rviz_config:=bringup` (fewer displays).
