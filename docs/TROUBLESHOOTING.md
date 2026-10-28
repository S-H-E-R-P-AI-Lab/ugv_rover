# Troubleshooting

Problems we actually hit while setting up the Webots simulator are marked **(seen)**.
The rest are the classic ROS 2 mistakes everyone makes once.

Quick health check with the simulator running:

```bash
ros2 topic hz /clock          # about 100 Hz times the real-time factor
ros2 topic hz /scan           # about 10 Hz
ros2 control list_controllers # three controllers, all "active"
```

---

## Setup and build

### `git clone` fails with `Authentication failed` or `Support for password authentication was removed` **(seen)**
GitHub no longer accepts account passwords for Git operations over HTTPS. The easiest way to clone without having to create and configure SSH keys is to authenticate with the GitHub CLI (`gh`):

```bash
sudo apt update && sudo apt install -y gh
gh auth login
```

Choose **GitHub.com**, **HTTPS**, **Yes** to authenticate Git credentials, and **Login with a web browser**. Open [https://github.com/login/device](https://github.com/login/device), paste the one-time code and authorize. Then clone with:

```bash
gh repo clone S-H-E-R-P-AI-Lab/ugv_rover ~/ugv_rover
```

### "Unable to locate package ros-humble-..." or ROS installs as Jazzy **(seen)**
ROS 2 Humble only exists for **Ubuntu 22.04**. Check with `lsb_release -a`. Our first WSL
machine had Ubuntu 24.04 (default for `wsl --install`). Install a 22.04 distro next to it:
`wsl --install -d Ubuntu-22.04` in PowerShell (see [setup/WINDOWS_WSL2.md](setup/WINDOWS_WSL2.md)).

### `ros2: command not found`, or `Package 'ugv_webots' not found`
You forgot to source. Every new terminal needs:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_rover/install/setup.bash
```

The setup script adds both lines to `~/.bashrc`. If you just built for the first time, open a
new terminal or source by hand.

### Models crash at load, "invalid header" or files of about 130 bytes in `ugv_voice/models` or `ugv_vision/models`
Those are Git LFS **pointer files**, the real weights were not downloaded. Fix:

```bash
sudo apt install git-lfs && git lfs install
cd ~/ugv_rover && git lfs pull
git lfs ls-files     # every line should show '*' (downloaded), not '-'
```

### Building is extremely slow, or permissions are weird (`chmod` does nothing)
The workspace is under `/mnt/c/...` (the Windows drive). Clone into the Linux file system
(`~/ugv_rover`). Builds on `/mnt/c` are 10 to 20 times slower and lose Linux permissions.

### Windows line endings (`/bin/bash^M: bad interpreter`)
The repo was cloned with Windows Git or `core.autocrlf=true`. In WSL:
`git config --global core.autocrlf input`, then re-clone inside WSL.

### `rosdep` reports `Cannot locate rosdep definition for [cmake_modules]` **(seen)**
Harmless. `rf2o_laser_odometry` declares a ROS 1 dependency that has no Humble entry. The
package builds without it.

### `build_first.sh` fails with `cd: /home/ws/ugv_ws: No such file or directory` **(seen)**
Upstream hard codes the path of Waveshare's Docker image. Run it with your path:
`UGV_WS=$HOME/ugv_rover bash build_first.sh`.

---

## Simulator start up

### The robot controller never connects, `webots_controller_ugv_rover` waits forever (WSL) **(seen)**
`webots_ros2` 2025.0.0 looks for Windows at the first `nameserver` of `/etc/resolv.conf`.
With current WSL (DNS tunneling on by default) that is `10.255.255.254`, which is not
Windows. `ugv_webots` replaces that lookup: it prints
`Robot controller will connect to Webots at <ip>:1234` at start. It uses the default
gateway with NAT networking and `127.0.0.1` with mirrored networking (both tested). If that address is
wrong for your setup, override it:

```bash
UGV_WEBOTS_HOST=127.0.0.1 ros2 launch ugv_webots ugv_webots.launch.py
```

If the address is right but nothing connects, Windows Defender Firewall is blocking
`webots-bin.exe`. Allow it on private and public networks (Windows Security, Firewall,
Allow an app), or see the firewall step in [setup/WINDOWS_WSL2.md](setup/WINDOWS_WSL2.md).

### `Switch controller timed out after 5.000000 seconds!` and `Failed to activate controller` **(seen)**
ros2_control only runs when simulation time advances, and in webots_ros2 2025.0.0 `/clock`
comes from the `Ros2Supervisor` node. Our launch file starts it. If you write your own
launch file, use `WebotsLauncher(..., ros2_supervisor=True)` and add `webots._supervisor`
to the launch description.

### Webots shows nothing for minutes, no error, `/clock` never appears **(seen)**
Two causes we hit:

1. **First start of a world.** Webots for Windows downloads every stock object (walls,
   furniture, textures) from GitHub the first time and caches it in
   `%LOCALAPPDATA%\Cyberbotics\Webots\cache`. GitHub sometimes drops the download with
   `Server is unable to maintain the header compression context for the connection`, and
   Webots then waits forever. Stop the launch (Ctrl+C, close Webots) and start again. Each try
   fills the cache further; the apartment world needed three tries on a fresh install.
   To avoid the downloads entirely, use the official offline cache: download
   `assets-R2025a.zip` (660 MB) from https://github.com/cyberbotics/webots/releases/tag/R2025a,
   set Webots' Preferences, Network, Disk cache to at least 1024 MB, and extract the archive
   so that an `assets` folder ends up in the cache folder: `%LOCALAPPDATA%\Cyberbotics\Webots\cache`
   on Windows, `~/.cache/Cyberbotics/Webots` on Linux, `~/Library/Caches/Cyberbotics/Webots`
   on macOS.
2. **A comment inside a node of a `.wbt` file.** Webots R2025a hangs while parsing a world
   that has a `#` comment inside a node (header comments are fine). Keep comments at the top.

### The robot falls through the floor, warning `As 'physics' is set to NULL, collisions will have no effect` **(seen)**
Happened with the raw urdf2webots output: fixed links became separate Solids without
physics, Webots ignores their collision shapes and attaches the wheel joints to the static
world. `scripts/generate_proto.py` restructures the robot (one body per rigid group). If you
see this again, regenerate the PROTO instead of editing it by hand.

### I pressed "Reload world" (or Ctrl+Shift+R) in Webots and ROS stopped working **(seen)**
Reloading disconnects the ROS controller and the launch shuts down. Stop everything and run
the `ros2 launch` command again. Use the pause and reset buttons only if you know you need
them; to restart cleanly, relaunch.

### RViz does not open in WSL
Update WSL (`wsl --update` in PowerShell); WSLg provides the display. Check `echo $DISPLAY`
shows `:0`.

---

## Running

### TF errors: `Lookup would require extrapolation into the future`, or the map does not grow
**Sim time mismatch.** One node uses wall time while the others use `/clock`. Every node
that talks to the simulator needs `use_sim_time:=true`:

```bash
ros2 run my_pkg my_node --ros-args -p use_sim_time:=true
ros2 param get /my_node use_sim_time      # check a running node
```

The same applies to `map_saver_cli` (`--ros-args -p use_sim_time:=true`).

### Odometry says the robot turned more than it did
That is wheel slip, as on the real skid steer robot. The simulator is calibrated so that
odometry and the real rotation agree within about 2 degrees per turn
(`wheel_separation_multiplier` in `ugv_webots/resource/ros2_control.yaml`). Re-measure with
`ros2 run ugv_webots motion_check` after any change to the wheels or the floor.

### The back of the lidar scan is empty
Correct: the pan-tilt mast stands 3 cm behind the lidar, inside its 0.2 m minimum range, so
about 76 degrees behind the robot read `inf`. The real LD19 driver crops a 90 degree sector
for the same reason.

### The real robot on the LAN is not visible from WSL (`ros2 topic list` misses its topics)
DDS discovery uses multicast, which WSL's default NAT networking does not pass through. Use
mirrored networking (see [setup/WINDOWS_WSL2.md](setup/WINDOWS_WSL2.md)): create
`%USERPROFILE%\.wslconfig` with

```ini
[wsl2]
networkingMode=mirrored
```

then `wsl --shutdown` in PowerShell and reopen Ubuntu. Also make sure both machines use the
same `ROS_DOMAIN_ID` (unset means 0) and that the Hyper-V firewall lets inbound traffic in:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

(run as administrator; the GUID is the WSL VM creator id from Microsoft's documentation).

### Topics from the simulator appear on teammates' machines
ROS 2 discovery reaches the whole LAN. Give yourself a private domain:
`export ROS_DOMAIN_ID=42` (any number 0 to 101, same in every terminal of your session),
or `export ROS_LOCALHOST_ONLY=1` when you only simulate.
