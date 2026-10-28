# Setup on Windows 11 (WSL2)

Time: about 1 hour, mostly downloads. You end up with ROS 2 running in Ubuntu inside WSL2
and the Webots simulator running natively on Windows (with your GPU). The two talk over a
local network connection; `ugv_webots` sets that up for you.

```
Windows 11                          WSL2: Ubuntu 22.04
+---------------------------+       +------------------------------------+
| Webots R2025a (GPU)       | <---> | ROS 2 Humble, this workspace, RViz |
| C:\Program Files\Webots   |  TCP  | ~/ugv_rover                        |
+---------------------------+  1234 +------------------------------------+
```

This was tested on Windows 11 with WSL 2.7, Ubuntu 22.04.5 and Webots R2025a.

## 1. Install WSL2 with Ubuntu 22.04

Open **PowerShell** (Start menu, type "PowerShell") and run:

```powershell
wsl --install -d Ubuntu-22.04
```

* If WSL was never installed, Windows asks to restart. Do it, then run the command again.
* A window opens and asks for a **username and password** for Ubuntu. Pick anything; you
  will type this password for `sudo`.
* **Use Ubuntu 22.04 exactly.** ROS 2 Humble does not exist for 24.04, which is what plain
  `wsl --install` gives you. If you already have another Ubuntu, that is fine: 22.04 is
  installed next to it. Open it from the Start menu ("Ubuntu 22.04") or with
  `wsl -d Ubuntu-22.04`.

Keep WSL current (this also gives you working GUI apps such as RViz):

```powershell
wsl --update
```

## 2. Networking for the real robot (recommended)

By default WSL sits behind a private NAT network. The simulator works with that, but ROS 2
on WSL then cannot discover the real rover on your Wi-Fi. **Mirrored networking** puts WSL
on the same network as Windows.

1. In PowerShell, open the WSL config file (it is usually new):
   ```powershell
   notepad $env:USERPROFILE\.wslconfig
   ```
2. Put this in it and save:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
3. Allow inbound traffic into WSL. Run PowerShell **as administrator**:
   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   ```
4. Restart WSL (this closes all Ubuntu windows):
   ```powershell
   wsl --shutdown
   ```

Check in Ubuntu: `wslinfo --networking-mode` prints `mirrored`, and `ip addr` shows your
Wi-Fi address. The simulator detects the mode by itself (it connects to Webots at
`127.0.0.1` when mirrored, and at the Windows gateway address otherwise).
Both modes have been tested with WSL Ubuntu 22.04 and Webots R2025a: the simulator connects,
all controllers start, and `ros2 run ugv_webots motion_check` passes.

## 3. Install Webots on Windows

Download and run the installer:
https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe

* Choose **Install for all users**, so it lands in `C:\Program Files\Webots`. That is where
  the ROS side looks for it.
* When Windows Defender Firewall asks about Webots the first time it runs, **allow it on
  private and public networks**. The WSL network adapter counts as public, and ROS reaches
  Webots through it.

R2025a is the current stable release and the version matching our `webots_ros2` package
(2025.0.0). Do not use a nightly build.

## 4. Get the code and install everything

Open **Ubuntu 22.04** and run:

```bash
sudo apt update && sudo apt install -y git git-lfs gh
git lfs install
git config --global core.autocrlf input
```

### GitHub authentication (GitHub CLI)

Authenticate with GitHub so you can clone the repository without having to configure SSH keys:

```bash
gh auth login
```

Choose the default options:
* **What account do you want to log into?** `GitHub.com`
* **What is your preferred protocol for Git operations?** `HTTPS`
* **Authenticate Git with your GitHub credentials?** `Yes`
* **How would you like to authenticate GitHub CLI?** `Login with a web browser`

Copy the 8-character one-time code shown in the terminal, press Enter to open your browser (or go to [https://github.com/login/device](https://github.com/login/device) on Windows), paste the code, and authorize.

*(Note: If `apt install gh` cannot find the package, ensure Ubuntu's universe repository is enabled with `sudo add-apt-repository -y universe && sudo apt update`, or follow the [official GitHub CLI install instructions](https://github.com/cli/cli/blob/trunk/docs/install_linux.md).)*

### Clone and run setup

```bash
cd ~
gh repo clone S-H-E-R-P-AI-Lab/ugv_rover
cd ugv_rover
bash scripts/setup_ubuntu.sh
```

*(You can also use `git clone https://github.com/S-H-E-R-P-AI-Lab/ugv_rover.git`—`gh auth login` configures Git credentials automatically.)*

Important: clone into your Linux home (`~`), **not** into `/mnt/c/...`. The Windows drive
is very slow from Linux and breaks file permissions.

`setup_ubuntu.sh` installs ROS 2 Humble and every package the repo needs, downloads the
model files (Git LFS), builds the workspace and adds the environment to `~/.bashrc`. It
takes 20 to 40 minutes and asks for your password once. You can run it again at any time;
it skips what is already done.

When it finishes, **close the terminal and open a new one** so the environment is loaded.

## 5. First simulation

```bash
ros2 launch ugv_webots ugv_webots.launch.py use_rviz:=true
```

Webots opens on Windows with the rover in an apartment, and RViz opens with the lidar,
cameras and depth points.

* **The first start of each world downloads furniture and textures from GitHub** and can
  take a few minutes. If nothing happens for more than 3 minutes, press Ctrl+C in the
  terminal, close Webots, and run the command again (see
  [TROUBLESHOOTING.md](../TROUBLESHOOTING.md#webots-shows-nothing-for-minutes-no-error-clock-never-appears-seen)).
* Do not use **Reload world** in Webots; stop and relaunch instead.

Drive it from a second Ubuntu terminal:

```bash
ros2 run ugv_tools keyboard_ctrl
```

Then continue with [the ROS 2 primer](../ROS2_PRIMER.md) and [SIMULATOR.md](../SIMULATOR.md)
(mapping and navigation).

## Optional: VS Code

Install VS Code on Windows with the **WSL** extension, then in Ubuntu run `code ~/ugv_rover`.
Files are edited in Linux directly.
